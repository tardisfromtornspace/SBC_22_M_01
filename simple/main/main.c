/* Simple HTTP + SSL Server Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "esp_netif.h"
#include "esp_eth.h"
#include "protocol_examples_common.h"

#include <esp_https_server.h>
#include "esp_tls.h"

// Del LED
#include <stdio.h>
#include "driver/gpio.h"
#include "led_strip.h"
#include "sdkconfig.h"
// Fin de includes del LED

/*Del ADC específicamente*/
#include "driver/adc.h"
#include "esp_adc_cal.h"
// Fin de la seccion ADC

// Del reseteo de fabrica
#include "esp_partition.h"
#include "esp_https_ota.h"
#include "esp_ota_ops.h"
#include "esp_log.h"

// MQTT

#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

#include "mqtt_client.h"

#include <stdint.h>
#include <stddef.h>
#include "esp_wifi.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_event.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

// Aniadir libreria cJSON
#include "cJSON.h"

// Para I2C
#include "driver/i2c.h"
// Para i2c test
#include "cmd_i2ctools.h"

//Para mensajes genéricos
#include <string.h>

/* A simple example that demonstrates how to create GET and POST
 * handlers and start an HTTPS server.
 */

static const char *TAG = "ServidorSimple";

// LED
#define PIN_SWITCH 35
#define BLINK_GPIO CONFIG_BLINK_GPIO

static uint8_t s_led_state = 0; // Estado del led

static uint8_t s_switch_state = 0; // Estado del switch

int s_reset_state = 0; // tiempo hasta resetear. 0 es que no se resetea

int voltajeHidro = 0; // Voltaje generado por las turbinas

int voltajeSolar = 0; // Voltaje generado por los paneles

// Fin del LED

// I2C

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU9250_SENSOR_ADDR                 0x68        /*!< Slave address of the MPU9250 sensor */
#define MPU9250_WHO_AM_I_REG_ADDR           0x75        /*!< Register addresses of the "who am I" register */

#define MPU9250_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
#define MPU9250_RESET_BIT                   7

#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                 /*!< I2C ack value */
#define NACK_VAL 0x1                /*!< I2C nack value */

uint8_t data[2]; // Para sensor I2C lumen

/**
 * @brief Read a sequence of bytes from a MPU9250 sensor registers
 */
static esp_err_t mpu9250_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    i2c_master_driver_initialize();
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, chip_addr << 1 | WRITE_BIT, ACK_CHECK_EN); // todo CHANGE CHIP ADDRESS THING
    int i = 0; // TODO mover a var globales
    int len = 2;
    for (i = 0; i < len - 1; i++){ // Solo son 2 datos
        i2c_master_read(cmd, data, len - 1, ACK_VAL);
    }
        i2c_master_read_byte(cmd, 1, NACK_VAL); // data + len - 1
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK) {
        for (int i = 0; i < len; i++) {
            printf("0x%02x ", data[i]);
            if ((i + 1) % 16 == 0) {
                printf("\r\n");
            }
        }
        if (len % 16) {
            printf("\r\n");
        }
    } else if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "Bus is busy");
    } else {
        ESP_LOGW(TAG, "Read failed");
    }
    free(data);
    i2c_driver_delete(i2c_port);
    return 0;
    // return i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);
}

/**
 * @brief Write a byte to a MPU9250 sensor register
 */
static esp_err_t mpu9250_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}
// Fin I2C

// MQTT
esp_mqtt_client_config_t mqtt_cfg = {
    //.uri =  "mqtt://iot.etsisi.upm.es",
    //.uri =  "mqtt://mqtt.eclipseprojects.io",
    //.uri =   "https://demo.thingsboard.io/" // Luego estoapi/v1/Y0kg9ua7tm6s4vaB0X1H/telemetry" ?
    .uri = "mqtt://demo.thingsboard.io",
    //.event_handle = mqtt_event_handler,
    .port = 1883,
    .username = "YSRNEFDXnyIGhX9OaylG", // token
};

esp_mqtt_client_handle_t client;

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
    // esp_mqtt_client_handle_t client = event->client;
    // int msg_id;
    // Los de arriba me dice que no están usados
    // your_context_t *context = event->context;
    switch (event->event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED");
        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED");
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED");
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
        printf("DATA=%.*s\r\n", event->data_len, event->data);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }

    return ESP_OK;
}
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id,
                               void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
    mqtt_event_handler_cb(event_data);
}
static void mqtt_app_start(void)
{

    // Parámetros para la conexión
    /*
    esp_mqtt_client_config_t mqtt_cfg = {
        //.uri =  "mqtt://iot.etsisi.upm.es",
        //.uri =  "mqtt://mqtt.eclipseprojects.io",
        //.uri =   "https://demo.thingsboard.io/" // Luego estoapi/v1/Y0kg9ua7tm6s4vaB0X1H/telemetry" ?
        .uri = "mqtt://demo.thingsboard.io",
        //.event_handle = mqtt_event_handler,
        .port = 1883,
        .username = "jeSK4atEMgZkSdwdPA5L", // token
    };
    */
    // Establecer la conexión
    // esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    // esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    // esp_mqtt_client_start(client);

    // Crear json que se quiere enviar al ThingsBoard
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "energiaSolar", voltajeSolar);      // En la telemetría de Thingsboard aparecerá temperature = counter y value = 26
    cJSON_AddNumberToObject(root, "energiaHidraulica", voltajeHidro); // En la telemetría de Thingsboard aparecerá temperature = counter y value = 26 TO-DO VERIFICAR
    //cJSON_AddNumberToObject(root, "temperaturaI2C", data[0]); // En la telemetría de Thingsboard aparecerá temperature = counter y value = 26 TO-DO VERIFICAR
    char *post_data = cJSON_PrintUnformatted(root);
    // Enviar los datos
    esp_mqtt_client_publish(client, "v1/devices/me/telemetry", post_data, 0, 1, 0); // En v1/devices/me/telemetry sale de la MQTT Device API Reference de ThingsBoard
    cJSON_Delete(root);
    // Free is intentional, it's client responsibility to free the result of cJSON_Print
    free(post_data);
}

// ADC Channels
#if CONFIG_IDF_TARGET_ESP32
#define ADC1_EXAMPLE_CHAN0 ADC1_CHANNEL_6
#define ADC2_EXAMPLE_CHAN0 ADC1_CHANNEL_4 // Usamos Wifi, no podemos usar el ADC2
static const char *TAG_CH[2][10] = {{"ADC1_CH6"}, {"ADC2_CH0"}};
#else
#define ADC1_EXAMPLE_CHAN0 ADC1_CHANNEL_2
#define ADC2_EXAMPLE_CHAN0 ADC2_CHANNEL_0
static const char *TAG_CH[2][10] = {{"ADC1_CH2"}, {"ADC2_CH0"}};
#endif

#define PIN_ANALOG 34
#define PIN_ANALOG2 32

// ADC Attenuation
#define ADC_EXAMPLE_ATTEN ADC_ATTEN_DB_11

// ADC Calibration
#if CONFIG_IDF_TARGET_ESP32
#define ADC_EXAMPLE_CALI_SCHEME ESP_ADC_CAL_VAL_EFUSE_VREF
#elif CONFIG_IDF_TARGET_ESP32S2
#define ADC_EXAMPLE_CALI_SCHEME ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32C3
#define ADC_EXAMPLE_CALI_SCHEME ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32S3
#define ADC_EXAMPLE_CALI_SCHEME ESP_ADC_CAL_VAL_EFUSE_TP_FIT
#endif

static int adc_raw[2][10];

static esp_adc_cal_characteristics_t adc1_chars;
static esp_adc_cal_characteristics_t adc2_chars;

static bool adc_calibration_init(void)
{
    esp_err_t ret;
    bool cali_enable = false;

    ret = esp_adc_cal_check_efuse(ADC_EXAMPLE_CALI_SCHEME);
    if (ret == ESP_ERR_NOT_SUPPORTED)
    {
        ESP_LOGW(TAG, "Calibration scheme not supported, skip software calibration");
    }
    else if (ret == ESP_ERR_INVALID_VERSION)
    {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    }
    else if (ret == ESP_OK)
    {
        cali_enable = true;
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_EXAMPLE_ATTEN, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
        esp_adc_cal_characterize(ADC_UNIT_2, ADC_EXAMPLE_ATTEN, ADC_WIDTH_BIT_DEFAULT, 0, &adc2_chars);
    }
    else
    {
        ESP_LOGE(TAG, "Invalid arg");
    }

    return cali_enable;
}

/* An HTTP GET handler */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    char mensaje[] = "<h1> Servidor Web </h1>"
                     "<h1> (Pagina principal, se refresca cada 10 segundos) </h1> <p><a href='/on'><button style='height:50px;width:100px'>ON</button></a></p> <p><a href='/off'><button style='height:50px;width:100px'>OFF</button></a></p><p><a href='/reset'><button style='height:50px;width:100px'>Resetear ESP32</button></a></p><h1>Estado del switch ";
    char onSWITCH[] = "ON</h1>";
    char offSWITCH[] = "OFF</h1>";
    char voltajeSol[] = "<h1> Voltaje solar generado (mV): ";
    char voltajeAgua[] = "<h1> Voltaje hidraulico generado (mV): ";
    char finEncabezado[] = "</h1>";
    char reseteo[] = "<h>La ESP32 se va a resetear en ";
    char finDePagina[] = "";

    const char *mess;

    //"<title>Servidor Especial</title><meta http-equiv='refresh' content='10'>";
    httpd_resp_set_hdr(req, "Refresh", "10");
    httpd_resp_set_type(req, "text/html");

    if (s_switch_state == true)
    {
        ESP_LOGI(TAG, "Le digo que tengo switch a ON");
        // strcat(mensaje, onSWITCH);
        strcat(mensaje, onSWITCH);
        // httpd_resp_send(req, "<h1>Estado del switch ON</h1>", HTTPD_RESP_USE_STRLEN);
    }
    else
    {
        ESP_LOGI(TAG, "Le digo que tengo switch a OFF");
        strcat(mensaje, offSWITCH);
    }
    strcat(mensaje, voltajeSol);

    char voltajeSolecito[20];
    itoa(voltajeSolar, voltajeSolecito, 10);
    strcat(mensaje, voltajeSolecito);
    strcat(mensaje, finEncabezado);

    strcat(mensaje, voltajeAgua);

    char voltajeAqua[20];
    itoa(voltajeHidro, voltajeAqua, 10);
    strcat(mensaje, voltajeAqua);
    strcat(mensaje, finEncabezado);

   // char i2cA[20];
   // itoa(data[0], i2cA, 10);
   // strcat(mensaje, i2cA);
   // strcat(mensaje, finEncabezado);

    mess = strcat(mensaje, finDePagina);
    if (s_reset_state != 0)
    {
        const char *resetmess;
        char tiempo[20];
        itoa(s_reset_state, tiempo, 10);
        strcat(reseteo, tiempo);
        resetmess = strcat(reseteo, "</h1>");
        httpd_resp_send(req, resetmess, HTTPD_RESP_USE_STRLEN);
    }
    else
        httpd_resp_send(req, mess, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

static esp_err_t buttON_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Activo el LED");
    s_led_state = true;
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_set_status(req, "302");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, "Redirigiendo", HTTPD_RESP_USE_STRLEN);
    // httpd_resp_set_type(req, "text/html");
    // root_get_handler(req);

    return ESP_OK;
}

static esp_err_t buttOFF_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Desactivo el LED");
    s_led_state = false;
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_set_status(req, "302");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, "Redirigiendo", HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}
static esp_err_t buttReset_get_handler(httpd_req_t *req)
{
    s_reset_state = 30;
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_set_status(req, "302");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, "Redirigiendo", HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

#if CONFIG_EXAMPLE_ENABLE_HTTPS_USER_CALLBACK
/**
 * Example callback function to get the certificate of connected clients,
 * whenever a new SSL connection is created
 *
 * Can also be used to other information like Socket FD, Connection state, etc.
 *
 * NOTE: This callback will not be able to obtain the client certificate if the
 * following config `Set minimum Certificate Verification mode to Optional` is
 * not enabled (enabled by default in this example).
 *
 * The config option is found here - Component config → ESP-TLS
 *
 */
void https_server_user_callback(esp_https_server_user_cb_arg_t *user_cb)
{
    ESP_LOGI(TAG, "Session Created!");
    ESP_LOGI(TAG, "Socket FD: %d", user_cb->tls->sockfd);

    const mbedtls_x509_crt *cert;
    const size_t buf_size = 1024;
    char *buf = calloc(buf_size, sizeof(char));
    if (buf == NULL)
    {
        ESP_LOGE(TAG, "Out of memory - Callback execution failed!");
        return;
    }

    mbedtls_x509_crt_info((char *)buf, buf_size - 1, "    ", &user_cb->tls->servercert);
    ESP_LOGI(TAG, "Server certificate info:\n%s", buf);
    memset(buf, 0x00, buf_size);

    cert = mbedtls_ssl_get_peer_cert(&user_cb->tls->ssl);
    if (cert != NULL)
    {
        mbedtls_x509_crt_info((char *)buf, buf_size - 1, "    ", cert);
        ESP_LOGI(TAG, "Peer certificate info:\n%s", buf);
    }
    else
    {
        ESP_LOGW(TAG, "Could not obtain the peer certificate!");
    }

    free(buf);
}
#endif

static const httpd_uri_t root = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = root_get_handler};

static const httpd_uri_t reset = {
    .uri = "/reset",
    .method = HTTP_GET,
    .handler = buttReset_get_handler};

static const httpd_uri_t off = {
    .uri = "/off",
    .method = HTTP_GET,
    .handler = buttOFF_get_handler};
static const httpd_uri_t on = {
    .uri = "/on",
    .method = HTTP_GET,
    .handler = buttON_get_handler};

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server");

    httpd_ssl_config_t conf = HTTPD_SSL_CONFIG_DEFAULT();

    extern const unsigned char cacert_pem_start[] asm("_binary_cacert_pem_start");
    extern const unsigned char cacert_pem_end[] asm("_binary_cacert_pem_end");
    conf.cacert_pem = cacert_pem_start;
    conf.cacert_len = cacert_pem_end - cacert_pem_start;

    extern const unsigned char prvtkey_pem_start[] asm("_binary_prvtkey_pem_start");
    extern const unsigned char prvtkey_pem_end[] asm("_binary_prvtkey_pem_end");
    conf.prvtkey_pem = prvtkey_pem_start;
    conf.prvtkey_len = prvtkey_pem_end - prvtkey_pem_start;

#if CONFIG_EXAMPLE_ENABLE_HTTPS_USER_CALLBACK
    conf.user_cb = https_server_user_callback;
#endif
    esp_err_t ret = httpd_ssl_start(&server, &conf);
    if (ESP_OK != ret)
    {
        ESP_LOGI(TAG, "Error starting server!");
        return NULL;
    }

    // Set URI handlers
    ESP_LOGI(TAG, "Registering URI handlers");
    httpd_register_uri_handler(server, &root); // Es importante poner los uri handlers acá para iniciarlos
    httpd_register_uri_handler(server, &reset);
    httpd_register_uri_handler(server, &off);
    httpd_register_uri_handler(server, &on);
    return server;
}

static void stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    httpd_ssl_stop(server);
}

static void disconnect_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    httpd_handle_t *server = (httpd_handle_t *)arg;
    if (*server)
    {
        stop_webserver(*server);
        *server = NULL;
    }
}

static void connect_handler(void *arg, esp_event_base_t event_base,
                            int32_t event_id, void *event_data)
{
    httpd_handle_t *server = (httpd_handle_t *)arg;
    if (*server == NULL)
    {
        *server = start_webserver();
    }
}

// Blinking led

#ifdef CONFIG_BLINK_LED_RMT
static led_strip_t *pStrip_a;

static void blink_led(void)
{
    /* If the addressable LED is enabled */
    if (s_led_state)
    {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        pStrip_a->set_pixel(pStrip_a, 0, 16, 16, 16);
        /* Refresh the strip to send data */
        pStrip_a->refresh(pStrip_a, 100);
    }
    else
    {
        /* Set all LED off to clear all pixels */
        pStrip_a->clear(pStrip_a, 50);
    }
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    pStrip_a = led_strip_init(CONFIG_BLINK_LED_RMT_CHANNEL, BLINK_GPIO, 1);
    /* Set all LED off to clear all pixels */
    pStrip_a->clear(pStrip_a, 50);
}

#elif CONFIG_BLINK_LED_GPIO

static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}

static void configure_switch(void)
{
    ESP_LOGI(TAG, "Example configured to switch GPIO LED!");
    gpio_reset_pin(PIN_SWITCH);
    /* Set the GPIO 34 as a push/pull output */
    gpio_set_direction(PIN_SWITCH, GPIO_MODE_INPUT);
}

#endif

void backtofactory()
{
    esp_partition_iterator_t pi;    // Iterator for find
    const esp_partition_t *factory; // Factory partition
    esp_err_t err;
    pi = esp_partition_find(ESP_PARTITION_TYPE_APP,            // Get partition iterator for
                            ESP_PARTITION_SUBTYPE_APP_FACTORY, // factory partition
                            "factory");
    if (pi == NULL) // Check result
    {
        ESP_LOGE(TAG, "Failed to find factory partition");
    }
    else
    {
        factory = esp_partition_get(pi);           // Get partition struct
        esp_partition_iterator_release(pi);        // Release the iterator
        err = esp_ota_set_boot_partition(factory); // Set partition for boot
        if (err != ESP_OK)                         // Check error
        {
            ESP_LOGE(TAG, "Failed to set boot partition");
        }
        else
        {
            esp_restart(); // Restart ESP
        }
    }
}

static void configure_analog(void)
{
    ESP_LOGI(TAG, "Configuring analog pins");
    gpio_reset_pin(PIN_ANALOG);
    /* Set the GPIO 34 as a push/pull output */
    gpio_set_direction(PIN_ANALOG, GPIO_MODE_INPUT);

    gpio_reset_pin(PIN_ANALOG2);
    /* Set the GPIO 32 as a push/pull output */
    gpio_set_direction(PIN_ANALOG2, GPIO_MODE_INPUT);
}

void app_main(void)
{
    /* Configure the peripheral according to the LED type */
    configure_led();
    configure_switch();
    configure_analog();

    //register_i2ctools();

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    esp_err_t ret = ESP_OK;
    uint32_t voltage = 0;
    bool cali_enable = adc_calibration_init();

    // ADC1 config
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_EXAMPLE_CHAN0, ADC_EXAMPLE_ATTEN));

    // ADC2 config
    ESP_ERROR_CHECK(adc2_config_channel_atten(ADC2_EXAMPLE_CHAN0, ADC_EXAMPLE_ATTEN));

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

    static httpd_handle_t server = NULL;

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Register event handlers to start server when Wi-Fi or Ethernet is connected,
     * and stop server when disconnection happens.
     */

#ifdef CONFIG_EXAMPLE_CONNECT_WIFI
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
#endif // CONFIG_EXAMPLE_CONNECT_WIFI
#ifdef CONFIG_EXAMPLE_CONNECT_ETHERNET
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ETHERNET_EVENT_DISCONNECTED, &disconnect_handler, &server));
#endif // CONFIG_EXAMPLE_CONNECT_ETHERNET

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());
    // Establecer la conexión MQTT, creo cliente
    client = esp_mqtt_client_init(&mqtt_cfg);
    /* The last argument may be used to pass data to the event handler, in this example mqtt_event_handler */
    //esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);

    while (1)
    {
        blink_led();
        if (gpio_get_level(PIN_SWITCH))
        {
            s_switch_state = true;
            ESP_LOGI(TAG, " Eh2 Tengo switch a ON");
        }
        else
        {
            s_switch_state = false;
            ESP_LOGI(TAG, "Tengo switch a OFF");
        }
        if (s_reset_state != 0)
        {
            s_reset_state -= 1;

            char c[] = "Reseteo en ";

            char tiempo[20];

            itoa(s_reset_state, tiempo, 10);
            const char *mens;
            mens = strcat(c, tiempo);

            ESP_LOGI(TAG, "%s", mens);

            if (s_reset_state == 0)
            {
                fflush(stdout);
                backtofactory();
            }
        }

        /*ADC Part*/

        ESP_LOGI(TAG, "Procedo a medir ADC");
        adc_raw[0][0] = adc1_get_raw(ADC1_EXAMPLE_CHAN0);

        ESP_LOGI(TAG_CH[0][0], "raw  data: %d", adc_raw[0][0]);
        if (cali_enable)
        {
            voltage = esp_adc_cal_raw_to_voltage(adc_raw[0][0], &adc1_chars);
            voltajeHidro = voltage;
            ESP_LOGI(TAG_CH[0][0], "cali data: %d mV", voltage);
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // 1000
                                         // Espera

        // El WiFi usa en adc2 así que no lo podemos usar, mejor el adc1

#if CONFIG_IDF_TARGET_ESP32
        adc_raw[1][0] = adc1_get_raw(ADC2_EXAMPLE_CHAN0);
#else
        do
        {
            ret = adc2_get_raw(ADC2_EXAMPLE_CHAN0, ADC_WIDTH_BIT_DEFAULT, &adc_raw[1][0]);
        } while (ret == ESP_ERR_INVALID_STATE);
        // ret = ESP_OK;
        ESP_ERROR_CHECK(ret);
#endif

        ESP_LOGI(TAG_CH[1][0], "raw  data: %d", adc_raw[1][0]);
        if (cali_enable)
        {
#if CONFIG_IDF_TARGET_ESP32
            voltage = esp_adc_cal_raw_to_voltage(adc_raw[1][0], &adc1_chars);
#else
            voltage = esp_adc_cal_raw_to_voltage(adc_raw[1][0], &adc2_chars);
#endif
            voltajeSolar = voltage;
            ESP_LOGI(TAG_CH[1][0], "cali data: %d mV", voltage);
        }

        //Delays para asegurar lecturas ADC correctas
        vTaskDelay(pdMS_TO_TICKS(2000));
        /* Read the MPU9250 WHO_AM_I register, on power up the register should have the value 0x71 */
        ESP_ERROR_CHECK(mpu9250_register_read(MPU9250_WHO_AM_I_REG_ADDR, data, 1));
        ESP_LOGI(TAG, "Valor de data 0 = %X", data[0]);

        /* Demonstrate writing by reseting the MPU9250 */
    //    ESP_ERROR_CHECK(mpu9250_register_write_byte(MPU9250_PWR_MGMT_1_REG_ADDR, 1 << MPU9250_RESET_BIT));

    //    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    //    ESP_LOGI(TAG, "I2C unitialized successfully");

        //MQTT
        mqtt_app_start(); // Envio datos
        //fin
    }
}