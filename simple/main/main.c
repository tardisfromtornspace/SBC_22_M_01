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
//#include "cmd_i2ctools.h"

// Para mensajes genéricos
#include <string.h>

// Para Telegram
#include <stdlib.h>
#include <sys/time.h>
#include "lwip/apps/sntp.h"

#include "sh2lib.h"

// Para los sleep
#include <time.h>
#include "soc/soc_caps.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "soc/rtc.h"
#include "esp32/ulp.h"

//Para display
#include "ssd1306.h"
#include "font8x8_basic.h"

/* A simple example that demonstrates how to create GET and POST
 * handlers and start an HTTPS server.
 */

static const char *TAG = "ServidorSimple";

// LED
#define PIN_SWITCH 2 // 35 // TO-DO TAMPOCO USES EL PIN 2 A MENOS QUE QUIERAS QUE SI EL SWITCH ESTÉ A ON NO SE PUEDA COMUNICAR CON LA FLASH PARA SUBIR PROGRAMAS POR CABLE Si aplicamos lo de la optimización, se puede hacer para despertar al procesador cuando se activa el switch del display (recomendable cambiar el switch a otro pin, sin embargo).
#define BLINK_GPIO CONFIG_BLINK_GPIO
// Fin del LED

// I2C nota: hemos elegido el 4 y el 0, puede que de incompatibilidades con otros módulos fuera de ESP32
/*
     GPIO num         RTC GPIO Num
SCL: gpio4  Y gpio2   10 y 12
SDA: gpio0  y MTDO    11 y 13

EN LOS PINES NO PONGAIS DE 6 A 11 QUE ESOS SON DE LA FLASH
*/

#define I2C_MASTER_SCL_IO GPIO_NUM_4  /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO GPIO_NUM_15 /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_0      /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 100000     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0   /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0   /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 10000

#define CO2_SENSOR_ADDR 0x5A /*!< Slave address of the CO2 sensor is 0x5A, when we add an additional binary 1 behind it transforms into 0xB5 */
#define CO2_REG_ADDR 0x5A    /*!< Register addresses of the CO2 lecture register */

#define TEMPHUM_SENSOR_ADDR 0x44 /*Dir sensor tempHum*/
#define COMANDO_TEMPHUM_MSB 0x24
#define COMANDO_TEMPHUM_LSB 0x16 /*Los comandos de este sensor son de 16 bits, indican el MSB el modo y el LSB la frecuencia, así 0x24 0x16 es modo SIngle Shot con baja repetibilidad*/

#define LUZ_SENSOR_ADDR 0x10 /*Dir sensor luminosidad*/
#define LUZ_REG_ADDR 0x04    /*Dir registro de luminosidad es 0x04*/

#define MPU9250_PWR_MGMT_1_REG_ADDR 0x6B /*!< Register addresses of the power managment register */
#define MPU9250_RESET_BIT 7

#define READ_BIT I2C_MASTER_READ /*Bits de lectura y escritura*/
#define WRITE_BIT I2C_MASTER_WRITE
#define ACK_CHECK_EN 0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0             /*!< I2C master will not check ack from slave */
#define ACK_VAL I2C_MASTER_ACK        /*!< I2C ack value */
#define NACK_VAL I2C_MASTER_LAST_NACK // I2C_MASTER_NACK    /*!< I2C nack value */

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
// Telegram

extern const uint8_t server_rootTelegram_cert_pem_start[] asm("_binary_http2_telegram_root_cert_pem_start");
extern const uint8_t server_rootTelegram_cert_pem_end[] asm("_binary_http2_telegram_root_cert_pem_end");

/* The HTTP/2 server to connect to */
#define HTTP2_SERVER_URI "https://api.telegram.org"
/* A GET request that keeps streaming current time every second */
#define TELEGRAMTOKEN "CAMBIALO POR EL TUYO" // TO-DO NO LO SUBAS CON ESTO A LA ENTREGA!!!!
#define CHATTOKEN "-891728903"               //"CAMBIA POR EL TUYO" // TO-DO NO LO SUBAS CON ESTO A LA ENTREGA!!!!
#define UNIVERSITY "SBC22_M01"               //"UPM"
#define TOKENMQTT "YSRNEFDXnyIGhX9OaylG"
#define MQTTURI "mqtt://demo.thingsboard.io"
int ini_OFFSET = 0;                
int last_msg_received = 559291163; 
int extraOffset = 0;
int conexionEnProceso = 1;
int conproc = -1;

// DESCOMENTA static const char GETBOT[] = "/bot" TELEGRAMTOKEN "/";
// static const char GETUPDATES[] = "/bot" TELEGRAMTOKEN "/getUpdates?limit=5";
static const char POSTUPDATES[] = "/bot" TELEGRAMTOKEN "/sendMessage?chat_id=" CHATTOKEN;

// Sleeps
static RTC_DATA_ATTR struct timeval sleep_enter_time;

// LEDes y algunos datos

static uint8_t s_led_state = 0; // Estado del led

static uint8_t s_switch_state = 0; // Estado del switch

int s_reset_state = 0; // tiempo hasta resetear. 0 es que no se resetea

int voltajeHidro = 0; // Voltaje generado por las turbinas

int voltajeSolar = 0; // Voltaje generado por los paneles

int datoI2CCO2legible = 0; // Concentracion de quimicos en el agua, usamos el sensor de CO2 como mock

int datoI2CFotonlegible = 0; // Concentración de químicos en el algua filtrada, usamos el sensor de luz como mock

// ADC

static int adc_raw[2][10];

static esp_adc_cal_characteristics_t adc1_chars;
static esp_adc_cal_characteristics_t adc2_chars;

// I2C CO2

static i2c_port_t i2c_master_port = I2C_MASTER_NUM;

uint8_t data[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; // Para sensor I2C CO2 - tomamos los 8 bytes pero solo necesitamos los 2 primeros

// LEDes

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

#endif

// Switch, se usará para encender o apagar el LCD
static void configure_switch(void)
{
    ESP_LOGI(TAG, "Example configured to switch GPIO LED!");
    gpio_reset_pin(PIN_SWITCH);
    /* Set the GPIO 34 as a push/pull output */
    gpio_set_direction(PIN_SWITCH, GPIO_MODE_INPUT);
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(SSD1306_t * dev, int16_t sda, int16_t scl, int16_t reset) // TO-DO Arreglar con el principal de Raúl, necesita el dev, este se vuelve redundante
{

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_DISABLE, // NO ENABLE porque las resistencias son externas
        .scl_pullup_en = GPIO_PULLUP_DISABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };

    ESP_ERROR_CHECK(i2c_param_config(i2c_master_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, ESP_INTR_FLAG_LEVEL1)); // conf.mode

    if (reset >= 0) {
		//gpio_pad_select_gpio(reset);
		gpio_reset_pin(reset);
		gpio_set_direction(reset, GPIO_MODE_OUTPUT);
		gpio_set_level(reset, 0);
		vTaskDelay(50 / portTICK_PERIOD_MS);
		gpio_set_level(reset, 1);
	}
	dev->_address = I2CAddress;
	dev->_flip = false;
    return ESP_OK;
}
/**
 * @brief Read a sequence of bytes from a sensor register
 */
static esp_err_t CO2_register_read(uint8_t slave_addr, uint8_t reg_addr, size_t len) // Los dispositivos que requieren de clock stretching no son soportados por ESP32 bien, cambiamos a un tercer sensor
{

    uint8_t dato[9] = {1, 2, 3, 4, 5, 6, 7, 8, 9};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // Primero hacemos que el sensor nos lea el comando
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (slave_addr << 1) | READ_BIT, ACK_VAL));

    int i = 0;
    for (i = 0; i < len - 1; i++)
    {
        ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &dato[i], ACK_VAL));
    }
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &dato[len - 1], NACK_VAL));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));

    esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK)
    {
        for (int i = 0; i < len; i++)
        {
            printf("0x%02x ", dato[i]);
            if ((i + 1) % 16 == 0)
            {
                printf("\r\n");
            }
        }
        if (len % 16)
        {
            printf("\r\n");
        }
    }
    else if (ret == ESP_ERR_TIMEOUT)
    {
        ESP_LOGW(TAG, "Bus is busy");
    }
    else if (ret == ESP_ERR_INVALID_ARG)
    {
        ESP_LOGW(TAG, "Parameter error");
    }
    else if (ret == ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(TAG, "I2C driver not installed on not in master mode");
    }
    else if (ret == ESP_FAIL)
    {
        ESP_LOGW(TAG, "Command error, slave hasn't ACK the transfer");
    }
    else
    {
        ESP_LOGW(TAG, "Read failed");
    }
    ESP_LOGI(TAG, "My ESP-CODE is %d", ret);
    // i2c_driver_delete(i2c_master_port);

    esp_log_buffer_hex(TAG, dato, 9);
    datoI2CCO2legible = dato[0] * 256 + dato[1];
    ESP_LOGI(TAG, "El CO2 me sale %X", datoI2CCO2legible);
    return ESP_OK;
}

static esp_err_t tempHum_register_read(uint8_t slave_addr, uint8_t reg_addrMSB, uint8_t reg_addrLSB, size_t len)
{

    uint8_t dato[6] = {1, 2, 3, 4, 5, 6}; // El primero y segundo son temepratura, el 4 y 5 humedad. el 3 y 6 son checksum

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // Primero hacemos que el sensor nos lea el comando
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (slave_addr << 1) | WRITE_BIT, ACK_VAL));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg_addrMSB, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg_addrLSB, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));

    esp_err_t retA = i2c_master_cmd_begin(i2c_master_port, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    vTaskDelay(pdMS_TO_TICKS(20)); // Needs 1 ms to prepare

    i2c_cmd_link_delete(cmd);
    if (retA == ESP_OK)
    {
        ESP_LOGI(TAG, "Logre llamar al sensor i2c tempHum y darle un comando");
    }
    else if (retA == ESP_ERR_TIMEOUT)
    {
        ESP_LOGW(TAG, "Bus is busy");
    }
    else if (retA == ESP_ERR_INVALID_ARG)
    {
        ESP_LOGW(TAG, "Parameter error");
    }
    else if (retA == ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(TAG, "I2C driver not installed on not in master mode");
    }
    else if (retA == ESP_FAIL)
    {
        ESP_LOGW(TAG, "Command error, slave hasn't ACK the transfer");
    }
    else
    {
        ESP_LOGW(TAG, "Read failed");
    }
    ESP_LOGI(TAG, "My ESP-CODE is %d", retA);

    i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd3));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd3, (slave_addr << 1) | READ_BIT, ACK_VAL));
    int i = 0;
    for (i = 0; i < len - 1; i++)
    {
        ESP_ERROR_CHECK(i2c_master_read_byte(cmd3, &dato[i], ACK_VAL));
    }
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd3, &dato[len - 1], NACK_VAL));
    ESP_ERROR_CHECK(i2c_master_stop(cmd3));

    esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd3, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));

    i2c_cmd_link_delete(cmd3);
    if (ret == ESP_OK)
    {
        ESP_LOGW(TAG, "Recibi dato tempHum");
        for (int i = 0; i < len; i++)
        {
            printf("0x%02x ", dato[i]);
            if ((i + 1) % 16 == 0)
            {
                printf("\r\n");
            }
        }
        if (len % 16)
        {
            printf("\r\n");
        }
    }
    else if (ret == ESP_ERR_TIMEOUT)
    {
        ESP_LOGW(TAG, "Bus is busy");
    }
    else if (ret == ESP_ERR_INVALID_ARG)
    {
        ESP_LOGW(TAG, "Parameter error");
    }
    else if (ret == ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(TAG, "I2C driver not installed on not in master mode");
    }
    else if (ret == ESP_FAIL)
    {
        ESP_LOGW(TAG, "Command error, slave hasn't ACK the transfer");
    }
    else
    {
        ESP_LOGW(TAG, "Read failed");
    }
    ESP_LOGI(TAG, "My ESP-CODE is %d", ret);

    esp_log_buffer_hex(TAG, dato, len);
    datoI2CCO2legible = dato[3] * 256 + dato[4]; // Este sensor manda primero el MSB y luego el LSB
    ESP_LOGI(TAG, "Lectura de humedad me sale %X", datoI2CCO2legible);
    return ESP_OK;
}

static esp_err_t register_read_commando(uint8_t slave_addr, uint8_t reg_addr, size_t len) // , uint8_t *data
{
    /*
     * El sensor de luminosidad tiene una forma curiosa de funcionar, en vez de enviar su dirección y luego el comando para leer, debes enviar su dirección con intención
     * de escribir y luego envias la dirección el registro que quieres leer, tras ello envías de nuevo la dirección pero con la intención de leer, y ahora ya recibes los
     * (2) byte(s) de respuesta
     */
    uint8_t dato[2] = {5, 7};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // Primero hacemos que el sensor nos lea el comando
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (slave_addr << 1) | WRITE_BIT, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN));
    // Ahora le pedimos que nos de el valor asociado al comando
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (slave_addr << 1) | READ_BIT, ACK_CHECK_EN));

    int i = 0;
    for (i = 0; i < len - 1; i++)
    {
        ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &dato[i], ACK_VAL));
        ESP_LOGI(TAG, "Leo valor bien: %x", dato[i]);
    }
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &dato[len - 1], NACK_VAL));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));

    esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    ESP_LOGI(TAG, "Leo valor bien (tras lectura sale): %x", dato[0]);
    if (ret == ESP_OK)
    {
        for (int i = 0; i < len; i++)
        {
            printf("0x%02x ", dato[i]);
            if ((i + 1) % 16 == 0)
            {
                printf("\r\n");
            }
        }
        if (len % 16)
        {
            printf("\r\n");
        }
    }
    else if (ret == ESP_ERR_TIMEOUT)
    {
        ESP_LOGW(TAG, "Bus is busy");
    }
    else if (ret == ESP_ERR_INVALID_ARG)
    {
        ESP_LOGW(TAG, "Parameter error");
    }
    else if (ret == ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(TAG, "I2C driver not installed on not in master mode");
    }
    else if (ret == ESP_FAIL)
    {
        ESP_LOGW(TAG, "Command error, slave hasn't ACK the transfer");
    }
    else
    {
        ESP_LOGW(TAG, "Read failed");
    }
    ESP_LOGI(TAG, "My ESP-CODE is %d", ret);
    // i2c_driver_delete(i2c_master_port);

    esp_log_buffer_hex(TAG, dato, 2);
    datoI2CFotonlegible = dato[1] * 256 + dato[0];
    // free(dato);
    ESP_LOGI(TAG, "El Lumen me sale %X", datoI2CFotonlegible);
    return ESP_OK;
}

static esp_err_t iniciar_sensorLuz(uint8_t slave_addr, uint8_t reg_addr, size_t len) // , uint8_t *data
{
    /*
     * El sensor de luminosidad tiene una forma curiosa de funcionar, en vez de enviar su dirección y luego el comando para leer, debes enviar su dirección con intención
     * de escribir y luego envias la dirección el registro que quieres leer, tras ello envías de nuevo la dirección pero con la intención de leer, y ahora ya recibes los
     * (2) byte(s) de respuesta
     */

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // Inicialización
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (slave_addr << 1) | WRITE_BIT, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN)); // Select the 0x00 register
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x42, ACK_CHECK_EN)); // Write on the 0x00 register lowThreshold 10000, highThreshols 20000 e InteeruptEnable 1 0x0843. El LSB
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0x08, ACK_CHECK_EN)); // El MSB
    ESP_ERROR_CHECK(i2c_master_stop(cmd));

    esp_err_t ret = i2c_master_cmd_begin(i2c_master_port, cmd, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "Inicialización Sensor Lumen A exitoso");
    }
    else if (ret == ESP_ERR_TIMEOUT)
    {
        ESP_LOGW(TAG, "Bus is busy");
    }
    else if (ret == ESP_ERR_INVALID_ARG)
    {
        ESP_LOGW(TAG, "Parameter error");
    }
    else if (ret == ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(TAG, "I2C driver not installed on not in master mode");
    }
    else if (ret == ESP_FAIL)
    {
        ESP_LOGW(TAG, "Command error, slave hasn't ACK the transfer");
    }
    else
    {
        ESP_LOGW(TAG, "Read failed");
    }

    i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
    // Inicialización
    ESP_ERROR_CHECK(i2c_master_start(cmd2));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd2, (slave_addr << 1) | WRITE_BIT, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd2, 0x01, ACK_CHECK_EN)); // Select the 0x01 register (high threshold)
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd2, 0x20, ACK_CHECK_EN)); // Write on the 0x00 register lowThreshold 10000 (0x2710), highThreshols 2000 (4E20) e InteeruptEnable 1 0x0843. El LSB
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd2, 0x4E, ACK_CHECK_EN)); // El MSB
    ESP_ERROR_CHECK(i2c_master_stop(cmd2));

    esp_err_t ret2 = i2c_master_cmd_begin(i2c_master_port, cmd2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd2);
    if (ret2 == ESP_OK)
    {
        ESP_LOGI(TAG, "Inicialización Sensor Lumen B exitoso");
    }
    else if (ret2 == ESP_ERR_TIMEOUT)
    {
        ESP_LOGW(TAG, "Bus is busy");
    }
    else if (ret2 == ESP_ERR_INVALID_ARG)
    {
        ESP_LOGW(TAG, "Parameter error");
    }
    else if (ret2 == ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(TAG, "I2C driver not installed on not in master mode");
    }
    else if (ret2 == ESP_FAIL)
    {
        ESP_LOGW(TAG, "Command error, slave hasn't ACK the transfer");
    }
    else
    {
        ESP_LOGW(TAG, "Read failed");
    }

    i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
    // Inicialización
    ESP_ERROR_CHECK(i2c_master_start(cmd3));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd3, (slave_addr << 1) | WRITE_BIT, ACK_CHECK_EN));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd3, 0x02, ACK_CHECK_EN)); // Select the 0x01 register (low threshold)
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd3, 0x10, ACK_CHECK_EN)); // Write on the 0x00 register lowThreshold 10000 (0x2710), highThreshols 2000 (4E20) e InteeruptEnable 1 0x0843. El LSB
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd3, 0x27, ACK_CHECK_EN)); // El MSB
    ESP_ERROR_CHECK(i2c_master_stop(cmd3));

    esp_err_t ret3 = i2c_master_cmd_begin(i2c_master_port, cmd3, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd3);
    if (ret3 == ESP_OK)
    {
        ESP_LOGI(TAG, "Inicialización Sensor Lumen C exitoso");
    }
    else if (ret3 == ESP_ERR_TIMEOUT)
    {
        ESP_LOGW(TAG, "Bus is busy");
    }
    else if (ret3 == ESP_ERR_INVALID_ARG)
    {
        ESP_LOGW(TAG, "Parameter error");
    }
    else if (ret3 == ESP_ERR_INVALID_STATE)
    {
        ESP_LOGW(TAG, "I2C driver not installed on not in master mode");
    }
    else if (ret3 == ESP_FAIL)
    {
        ESP_LOGW(TAG, "Command error, slave hasn't ACK the transfer");
    }
    else
    {
        ESP_LOGW(TAG, "Read failed");
    }
    // register_read_commando(slave_addr, 0x00, 2);
    // register_read_commando(slave_addr, 0x01, 2);
    // register_read_commando(slave_addr, 0x02, 2);

    return ESP_OK;
}

/**
 * @brief Write a byte to a MPU9250 sensor register
 * TO-DO Ajustar para el Display
 */
static esp_err_t display_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, CO2_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_RATE_MS);

    return ret;
}

// Fin I2C

// MQTT
esp_mqtt_client_config_t mqtt_cfg = {
    .uri = MQTTURI, //"mqtt://demo.thingsboard.io", // api/v1/Y0kg9ua7tm6s4vaB0X1H/telemetry" ?
    //.event_handle = mqtt_event_handler,
    .port = 1883,
    .username = TOKENMQTT, //"YSRNEFDXnyIGhX9OaylG", // token MQTT
};

esp_mqtt_client_handle_t client;

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{

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
    // Crear json que se quiere enviar al ThingsBoard
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "energiaSolar", voltajeSolar);      // En la telemetría de Thingsboard aparecerá
    cJSON_AddNumberToObject(root, "energiaHidraulica", voltajeHidro); // En la telemetría de Thingsboard aparecerá
    cJSON_AddNumberToObject(root, "co2I2C", datoI2CCO2legible);       // En la telemetría de Thingsboard aparecerá lo sacado del I2C 
    cJSON_AddNumberToObject(root, "luzI2C", datoI2CFotonlegible);     // En la telemetría de Thingsboard aparecerá lo sacado del I2C 
    cJSON_AddNumberToObject(root, "botonDisplay", s_switch_state);    // En la telemetría de Thingsboard aparecerá como valor true/false
    char *post_data = cJSON_PrintUnformatted(root);
    // Enviar los datos
    esp_mqtt_client_publish(client, "v1/devices/me/telemetry", post_data, 0, 1, 0); // En v1/devices/me/telemetry sale de la MQTT Device API Reference de ThingsBoard
    cJSON_Delete(root);
    // Free is intentional, it's client responsibility to free the result of cJSON_Print
    free(post_data);
}

// ADC

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
/*
 * TELEGRAM
 */
int handle_echo_response(struct sh2lib_handle *handle, const char *data, size_t len, int flags)
{
    if (len)
    {
        printf("[echo-response] %.*s\n", len, data);
    }
    if (flags == DATA_RECV_FRAME_COMPLETE)
    {
        printf("[echo-response] Frame fully received\n");
    }
    if (flags == DATA_RECV_RST_STREAM)
    {
        printf("[echo-response] Stream Closed\n");
        conexionEnProceso = 0;
    }
    //    vTaskDelete(NULL);
    return 0;
}

char *cutoff(const char *str, int from, int to)
{
    if (from >= to)
        return NULL;

    char *cut = calloc(sizeof(char), (to - from) + 1);
    char *begin = cut;
    if (!cut)
        return NULL;

    const char *fromit = str + from;
    const char *toit = str + to;
    (void)toit;
    memcpy(cut, fromit, to);
    return begin;
}

int handle_get_response(struct sh2lib_handle *handle, const char *data, size_t len, int flags)
{
    if (len)
    {
        printf("[get-response] %.*s\n", len, data);
        // Ahora busco cosas
        if (data != NULL)
        {
            // Sacamos el json de los datos
            char auxData[len];
            sprintf(auxData, "%.*s", len, data);
            cJSON *root = cJSON_Parse(data);
            char *post_data = cJSON_Print(root);

            if (cJSON_HasObjectItem(root, "ok"))
            {
                printf("Tengo un resultado con algo\n");
                if (cJSON_HasObjectItem(root, "result"))
                {
                    printf("Tengo un resultado no vacio\n");
                    cJSON *parameters = cJSON_GetObjectItemCaseSensitive(root, "result");
                    puts("Parameters:");
                    cJSON *parameter;
                    cJSON_ArrayForEach(parameter, parameters)
                    {

                        /* Each element is an object with unknown field(s) */
                        cJSON *elem;
                        int puedo = 0;
                        cJSON_ArrayForEach(elem, parameter)
                        {

                            if (strcmp(elem->string, "update_id") == 0)
                            {
                                cJSON *update_id = cJSON_GetObjectItemCaseSensitive(parameter, "update_id");
                                char *postid_data = cJSON_Print(update_id);
                                int extraUOffset = atoi(postid_data);

                                printf("El aux del offset: %d\n", extraUOffset);
                                if (ini_OFFSET == 0 || ini_OFFSET < extraUOffset)
                                {
                                    ini_OFFSET = extraUOffset;
                                    puedo = 1;
                                }
                                else if (conproc < extraUOffset)
                                {
                                    conproc = extraUOffset;
                                    printf("valor de conproc ahora = %d", conproc);
                                }
                            }
                            // Esta es nuestra cabeza, la próxima vez que pasamos ni respondemos
                            if ((puedo == 1) && strcmp(elem->string, "message") == 0)
                            {
                                printf("Mira si hay un mensaje encontrado");
                                cJSON *unMensaje = cJSON_GetObjectItemCaseSensitive(parameter, "message");
                                if (cJSON_HasObjectItem(unMensaje, "chat"))
                                {

                                    cJSON *elChat = cJSON_GetObjectItemCaseSensitive(unMensaje, "chat");
                                    cJSON *miId = cJSON_GetObjectItemCaseSensitive(elChat, "id");
                                    if (miId != NULL)
                                    {
                                        printf("Found key '%s'\n", miId->string);
                                        char *postman_data = cJSON_Print(miId);
                                        if (strcmp(postman_data, CHATTOKEN) == 0)
                                        {
                                            printf("Estoy en el chat correcto :)");
                                            if (cJSON_HasObjectItem(unMensaje, "text"))
                                            {
                                                cJSON *elMensajeDeLectura = cJSON_GetObjectItemCaseSensitive(unMensaje, "text");
                                                char *auxMensajeC = cJSON_Print(elMensajeDeLectura); // cJSON_Print

                                                // printf("Mensaje que tengo: %s , de %d caracteres", auxMensajeC, strlen(auxMensajeC)); // Deja las 2 " y no se pueden quitar con facilidad"
                                                char *auxMensaja = cutoff(auxMensajeC, 1, strlen(auxMensajeC));
                                                // printf("Mensaje que tengo: %s , de %d caracteres", auxMensaja, strlen(auxMensaja));
                                                char *auxMensaje = cutoff(auxMensaja, 0, strlen(auxMensaja) - 1);
                                                printf("Mensaje que tengo: %s , de %d caracteres", auxMensaje, strlen(auxMensaje));
                                                // RESPUESTA SI TODO VACÍO O MAL:
                                                char auxRep[] = "No entiendo";

                                                char cmd1[] = "/saluda";
                                                // RESPUESTA: Hola Mundo
                                                char cmd1Rep[] = "Hola Mundo";

                                                char cmd2[] = "/myId";
                                                // RESPUESTA: La id
                                                // char cmd2Rep[] = <Mi Id>; que ya hicimos antes (postman_data)

                                                char cmd3[] = "/restartPlaca";
                                                // RESPUESTA: Hola Mundo
                                                char cmd3Rep[] = "Enseguida la reseteo";

                                                char cmd4[] = "/datos";
                                                // RESPUESTA: Devuelve la info del sistema que recoge
                                                char cmd4Rep[] = "El sistema detecta: ";

                                                char cmdP1[] = "/Q-Que son los sumideros de carbono";
                                                char cmdP1Rep[] = "Los sumideros de carbono son depositos naturales que absorben el carbono de la atmósfera y lo fijan.";

                                                char cmdP2[] = "/Q-Por que en la fotosintesis una planta toma CO2 y libera unicamente O2";
                                                // b. ¿Por qué en la fotosíntesis una planta toma CO2 y libera únicamente O2?, ¿qué pasa con el Carbono en ese proceso?
                                                char cmdP2Rep[] = "Los procesos fotosinteticos de las plantas emplean el CO2 y la luz para producir glucosa, el producto de desecho es el O2, mientras que el carbono se fija.";

                                                char cmdP3[] = "/Q-Que componentes son imprescindibles para que una planta crezca";
                                                // c. ¿Qué componentes son imprescindibles para que una planta crezca?
                                                char cmdP3Rep[] = "las plantas son aerobeas asi que requieren tambien de oxigeno para sobrevivir, ya que tambien respiran, aunque por el dia no se note. Ademas necesitan de agua y sales minerales para realizar sus procesos metabolicos.";

                                                char str[2048];

                                                if (strcmp(auxMensaje, cmd1) == 0)
                                                {
                                                    sprintf(str, "%s&text=%s : %s: %s", POSTUPDATES, auxMensaje, UNIVERSITY, cmd1Rep);
                                                    // sh2lib_do_post( aparentemente para casos simples se hace con un get, que curioso
                                                    sh2lib_do_get(handle, str, handle_echo_response);
                                                }
                                                else if (strcmp(auxMensaje, cmd2) == 0)
                                                {
                                                    sprintf(str, "%s&text=%s : %s: %s", POSTUPDATES, auxMensaje, UNIVERSITY, postman_data);
                                                    sh2lib_do_get(handle, str, handle_echo_response);
                                                }
                                                else if (strcmp(auxMensaje, cmd3) == 0)
                                                {
                                                    sprintf(str, "%s&text=%s : %s: %s", POSTUPDATES, auxMensaje, UNIVERSITY, cmd3Rep);
                                                    sh2lib_do_get(handle, str, handle_echo_response);
                                                    s_reset_state = 20;
                                                }
                                                else if (strcmp(auxMensaje, cmd4) == 0)
                                                {
                                                    sprintf(str, "%s&text=%s : %s: %s Vsolar = %d, Vhidro = %d, Tox pre-filtro = %d, Tox post-filtro = %d. SwitchDisplay = %d", POSTUPDATES, auxMensaje, UNIVERSITY, cmd4Rep, voltajeSolar, voltajeHidro, datoI2CCO2legible, datoI2CFotonlegible, s_switch_state);
                                                    sh2lib_do_get(handle, str, handle_echo_response);
                                                    s_reset_state = 20;
                                                }
                                                // Las preguntas
                                                else if (strcmp(auxMensaje, cmdP1) == 0)
                                                {
                                                    sprintf(str, "%s&text=%s : %s: %s", POSTUPDATES, auxMensaje, UNIVERSITY, cmdP1Rep);
                                                    sh2lib_do_get(handle, str, handle_echo_response);
                                                }
                                                else if (strcmp(auxMensaje, cmdP2) == 0)
                                                {
                                                    sprintf(str, "%s&text=%s : %s: %s", POSTUPDATES, auxMensaje, UNIVERSITY, cmdP2Rep);
                                                    sh2lib_do_get(handle, str, handle_echo_response);
                                                }
                                                else if (strcmp(auxMensaje, cmdP3) == 0)
                                                {
                                                    sprintf(str, "%s&text=%s : %s: %s", POSTUPDATES, auxMensaje, UNIVERSITY, cmdP3Rep);
                                                    sh2lib_do_get(handle, str, handle_echo_response);
                                                    s_reset_state = 20;
                                                }
                                                else
                                                {
                                                    sprintf(str, "%s&text=%s : %s: %s", POSTUPDATES, auxMensaje, UNIVERSITY, auxRep);
                                                    sh2lib_do_get(handle, str, handle_echo_response);
                                                }
                                            }
                                            else
                                                printf("Mensaje de otro tipo");
                                        }
                                        else
                                        {
                                            printf("Este no es mi chat :|");
                                        }
                                        free(postman_data);
                                    }

                                    // cJSON_Delete(elChat);
                                    // cJSON_Delete(miId);
                                }
                                else
                                    printf("No se encuentra chat :/");
                            }
                        }
                        // cJSON_Delete(elem);
                    }
                    // cJSON_Delete(parameters);
                    // cJSON_Delete(parameter);
                }
                else
                    printf("Algo va mal con el resultado");
            }
            else
                printf("Algo va mal con el ok");

            cJSON_Delete(root);
            // Free is intentional, it's client responsibility to free the result of cJSON_Print
            free(post_data);
        }
    }
    if (flags == DATA_RECV_FRAME_COMPLETE)
    {
        printf("[get-response] Frame fully received\n");
    }
    if (flags == DATA_RECV_RST_STREAM)
    {
        printf("[get-response] Stream Closed\n");
        if (ini_OFFSET > conproc)
        {
            conexionEnProceso = 1;
        }
        else
            conexionEnProceso = 0;
        printf("primer get %d con ini offset = %d: Conexion en proceso = %d", conproc, ini_OFFSET, conexionEnProceso);
    }
    return 0;
}

int send_put_data(struct sh2lib_handle *handle, char *buf, size_t length, uint32_t *data_flags)
{
#define DATA_TO_SEND "Hello World"
    int copylen = strlen(DATA_TO_SEND);
    if (copylen < length)
    {
        printf("[data-prvd] Sending %d bytes\n", copylen);
        memcpy(buf, DATA_TO_SEND, copylen);
    }
    else
    {
        copylen = 0;
    }

    (*data_flags) |= NGHTTP2_DATA_FLAG_EOF;
    return copylen;
}

static void set_time(void) // Tiempo es necesario para HTTP2
{
    struct timeval tv = {
        .tv_sec = 1509449941,
    };
    struct timezone tz = {
        0, 0};
    settimeofday(&tv, &tz);

    /* Start SNTP service */
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_init();
}

static void http2_task(void *args)
{
    /* Set current time: proper system time is required for TLS based
     * certificate verification.
     */
    set_time();

    /* HTTP2: one connection multiple requests. Do the TLS/TCP connection first */
    printf("Connecting to server\n");
    struct sh2lib_config_t cfg = {
        .uri = HTTP2_SERVER_URI,
        .cacert_buf = server_rootTelegram_cert_pem_start,
        .cacert_bytes = server_rootTelegram_cert_pem_end - server_rootTelegram_cert_pem_start,
    };

    while (1)
    {
        conexionEnProceso = 1;
        struct sh2lib_handle hd;
        if (sh2lib_connect(&cfg, &hd) != 0)
        {
            printf("Failed to connect\n");
            vTaskDelete(NULL);
            return;
        }
        printf("Connection done\n");

        int offset = ini_OFFSET; // Offset inicial

        char str[256];
        offset = ini_OFFSET;
        printf("My offset es %d", offset);
        sprintf(str, "/bot%s/getUpdates?offset=%d", TELEGRAMTOKEN, offset);
        sh2lib_do_get(&hd, str, handle_get_response);
        /* HTTP GET  */
        while (conexionEnProceso > 0) // Ahora se pide ejecutar todo lo que se ponga arriba hasta que desconecte
        {
            /* Process HTTP2 send/receive */
            if (sh2lib_execute(&hd) < 0)
            {
                printf("Error in send/receive\n");
                break;
            }
            vTaskDelay(1);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));

        sh2lib_free(&hd);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    vTaskDelete(NULL);
}

/* An HTTP GET handler */
static esp_err_t root_get_handler(httpd_req_t *req)
{
    char mensajito[494];

    char finDePagina[] = "";
    const char *mess;

    // Esto es para refrescar cada 10 segundos, es la versión válida de "<title>Servidor Especial</title><meta http-equiv='refresh' content='10'>";
    httpd_resp_set_hdr(req, "Refresh", "10");
    httpd_resp_set_type(req, "text/html");
    if (s_reset_state != 0)
    {
        sprintf(mensajito, "<h1>La ESP32 se va a resetear en %d</h1>", s_reset_state);
        mess = strcat(mensajito, finDePagina);
        // httpd_resp_send(req, resetmess, HTTPD_RESP_USE_STRLEN);
    }
    else
    {
        char estadoSwitch[] = "N/A";
        if (s_switch_state == true)
        {
            ESP_LOGI(TAG, "Le digo que tengo switch a ON");
            sprintf(estadoSwitch, " ON");
        }
        else
        {
            ESP_LOGI(TAG, "Le digo que tengo switch a OFF");
            sprintf(estadoSwitch, "OFF");
        }

        sprintf(mensajito, "<h1> Servidor Web </h1><h1> (Refresh 10 segundos) </h1> <p><a href='/on'><button style='height:50px;width:100px'>ON</button></a></p> <p><a href='/off'><button style='height:50px;width:100px'>OFF</button></a></p><p><a href='/reset'><button style='height:50px;width:100px'>Resetear ESP32</button></a></p><h1>Display switch %s</h1><h1> V sol (mV): %d</h1><h1> V hidro (mV): %d</h1><h1> Toxinas pre-filtro (ppm): %d</h1><h1> Toxinas post-filtro (ppm): %d</h1>", estadoSwitch, voltajeSolar, voltajeHidro, datoI2CCO2legible, datoI2CFotonlegible);

        mess = strcat(mensajito, finDePagina);
    }
    httpd_resp_send(req, mess, HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

static esp_err_t buttON_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Activo el LED");
    s_led_state = true;
    blink_led();
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_set_status(req, "302");
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, "Redirigiendo", HTTPD_RESP_USE_STRLEN);

    return ESP_OK;
}

static esp_err_t buttOFF_get_handler(httpd_req_t *req)
{
    ESP_LOGI(TAG, "Desactivo el LED");
    s_led_state = false;
    blink_led();
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

/*
#define BOT_TOKEN tokenBotUCAEP
const unsigned long BOT_MTBS = 1000; // mean time between scan messages
WiFiClientSecure secured_client;
UniversalTelegramBot bot(BOT_TOKEN, secured_client);
unsigned long bot_lasttime; // last time messages' scan has been done

*/
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
/*
 * TELEGRAM
 */
esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;
    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        break;
    }
    return ESP_OK;
}
// Volver a la partición de fábrica
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

// Configuración de los dos pines analógicos
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

void sleepDelDisplay(void)
{
#ifdef CONFIG_EXAMPLE_EXT1_WAKEUP
    const int ext_wakeup_pin_1 = 2;
    const uint64_t ext_wakeup_pin_1_mask = 1ULL << ext_wakeup_pin_1;

    printf("Enabling EXT1 wakeup on pin GPIO%d\n", ext_wakeup_pin_1);
    esp_sleep_enable_ext1_wakeup(ext_wakeup_pin_1_mask, ESP_EXT1_WAKEUP_ANY_HIGH);

    /* If there are no external pull-up/downs, tie wakeup pins to inactive level with internal pull-up/downs via RTC IO
     * during deepsleep. However, RTC IO relies on the RTC_PERIPH power domain. Keeping this power domain on will
     * increase some power comsumption. */
#if CONFIG_EXAMPLE_EXT1_USE_INTERNAL_PULLUPS
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
    rtc_gpio_pullup_dis(ext_wakeup_pin_1);
    rtc_gpio_pulldown_en(ext_wakeup_pin_1);
#endif // CONFIG_EXAMPLE_EXT1_USE_INTERNAL_PULLUPS
#endif // CONFIG_EXAMPLE_EXT1_WAKEUP
}
void deQueMeLevante(int sleep_time_ms)
{
    switch (esp_sleep_get_wakeup_cause())
    {
#if CONFIG_EXAMPLE_EXT0_WAKEUP
    case ESP_SLEEP_WAKEUP_EXT0:
    {
        printf("Wake up from ext0\n");
        break;
    }
#endif // CONFIG_EXAMPLE_EXT0_WAKEUP
#ifdef CONFIG_EXAMPLE_EXT1_WAKEUP
    case ESP_SLEEP_WAKEUP_EXT1:
    {
        uint64_t wakeup_pin_mask = esp_sleep_get_ext1_wakeup_status();
        if (wakeup_pin_mask != 0)
        {
            int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
            printf("Wake up from GPIO %d\n", pin);
        }
        else
        {
            printf("Wake up from GPIO\n");
        }
        break;
    }
#endif // CONFIG_EXAMPLE_EXT1_WAKEUP
#if SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
    case ESP_SLEEP_WAKEUP_GPIO:
    {
        uint64_t wakeup_pin_mask = esp_sleep_get_gpio_wakeup_status();
        if (wakeup_pin_mask != 0)
        {
            int pin = __builtin_ffsll(wakeup_pin_mask) - 1;
            printf("Wake up from GPIO %d\n", pin);
        }
        else
        {
            printf("Wake up from GPIO\n");
        }
        break;
    }
#endif // SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
    case ESP_SLEEP_WAKEUP_TIMER:
    {
        printf("Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
        break;
    }
#ifdef CONFIG_EXAMPLE_TOUCH_WAKEUP
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
    {
        printf("Wake up from touch on pad %d\n", esp_sleep_get_touchpad_wakeup_status());
        break;
    }
#endif // CONFIG_EXAMPLE_TOUCH_WAKEUP
#ifdef CONFIG_EXAMPLE_ULP_TEMPERATURE_WAKEUP
#if CONFIG_IDF_TARGET_ESP32
    case ESP_SLEEP_WAKEUP_ULP:
    {
        printf("Wake up from ULP\n");
        int16_t diff_high = (int16_t)ulp_data_read(3);
        int16_t diff_low = (int16_t)ulp_data_read(4);
        if (diff_high < 0)
        {
            printf("High temperature alarm was triggered\n");
        }
        else if (diff_low < 0)
        {
            printf("Low temperature alarm was triggered\n");
        }
        else
        {
            assert(false && "temperature has stayed within limits, but got ULP wakeup\n");
        }
        break;
    }
#endif // CONFIG_IDF_TARGET_ESP32
#endif // CONFIG_EXAMPLE_ULP_TEMPERATURE_WAKEUP
    case ESP_SLEEP_WAKEUP_UNDEFINED:
    default:
        printf("Not a deep sleep reset\n");
    }

#ifdef CONFIG_EXAMPLE_ULP_TEMPERATURE_WAKEUP
#if CONFIG_IDF_TARGET_ESP32
    if (esp_sleep_get_wakeup_cause() != ESP_SLEEP_WAKEUP_UNDEFINED)
    {
        printf("ULP did %d temperature measurements in %d ms\n", ulp_data_read(1), sleep_time_ms);
        printf("Initial T=%d, latest T=%d\n", ulp_data_read(0), ulp_data_read(2));
    }
#endif // CONFIG_IDF_TARGET_ESP32
#endif // CONFIG_EXAMPLE_ULP_TEMPERATURE_WAKEUP
}

// PROGRAMA PRINCIPAL
void app_main(void)
{
    /*
    * Información del sleep
    */
    struct timeval now;
    gettimeofday(&now, NULL);
    int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;
    /*
     * Configurar periféricos, LED, switch y los inputs analógicos
     */
    configure_led();
    configure_switch();
    configure_analog();

    // iniciar I2C
    // Iniciar el display
    SSD1306_t dev;
    ESP_ERROR_CHECK(i2c_master_init(&dev, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, 0));
    //ESP_ERROR_CHECK(i2c_master_init()); // TO-DO 
    ESP_ERROR_CHECK(iniciar_sensorLuz(LUZ_SENSOR_ADDR, LUZ_REG_ADDR, 2));
    ESP_LOGI(TAG, "I2C initialized successfully");
    // Variables auxiliares del I2C display, e inicialización extra del display
    char primeralineChar[32];
    char segundalineChar[32];
    char terceralineChar[32];
    char cuartalineChar[32];
    ssd1306_clear_screen(&dev, false); // El display a 0, no a valores basura

#if CONFIG_SSD1306_128x64
	ESP_LOGI(TAG, "Panel is 128x64");
	ssd1306_init(&dev, 128, 64);
#endif // CONFIG_SSD1306_128x64
#if CONFIG_SSD1306_128x32
	ESP_LOGI(TAG, "Panel is 128x32");
	ssd1306_init(&dev, 128, 32);
#endif // CONFIG_SSD1306_128x32

    // Iniciar ADC
#if CONFIG_IDF_TARGET_ESP32 // El WiFi usa en adc2 así que no podemos usar ese segundo módulo, mejor multiplexamos el adc1 y hay variables qur no necesitamos usar
    ESP_LOGI(TAG, "Estamos en ESP-32");
#else
    esp_err_t ret = ESP_OK;
#endif

    uint32_t voltage = 0;
    bool cali_enable = adc_calibration_init();

    // ADC1 config
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_EXAMPLE_CHAN0, ADC_EXAMPLE_ATTEN));

    // ADC2 config
    ESP_ERROR_CHECK(adc2_config_channel_atten(ADC2_EXAMPLE_CHAN0, ADC_EXAMPLE_ATTEN));

    /*
     * Informacion del chip
     */
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

    /*
    * Cosas del Sleep TO-DO
    */
    //deQueMeLevante(sleep_time_ms);
    // Sleep segun el boton del display TO-DO
    //sleepDelDisplay();

    //Sleep de 20 segundos
    //const int wakeup_time_sec = 20;
    //printf("Enabling timer wakeup, %ds\n", wakeup_time_sec);
    //esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);

    /*
     * Registrar handlers de evento para montar el servidor cuando se conectan el Wi-Di o Ethernet, y parar cuando se desconecta.
     */

#ifdef CONFIG_EXAMPLE_CONNECT_WIFI
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &disconnect_handler, &server));
#endif
#ifdef CONFIG_EXAMPLE_CONNECT_ETHERNET
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_ETH_GOT_IP, &connect_handler, &server));
    ESP_ERROR_CHECK(esp_event_handler_register(ETH_EVENT, ETHERNET_EVENT_DISCONNECTED, &disconnect_handler, &server));
#endif

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    /*
     * Establecer la conexión MQTT, crearé al cliente
     */
    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_start(client);

    // Task HTTP2 para Telegram
    xTaskCreate(&http2_task, "http2_task", (1024 * 32), NULL, 5, NULL);
    
    // Task mqtt? TO-DO?
    //  xTaskCreate(mqtt_app_start, "mqtt_send_data_0", 1024 * 2, (void *)0, 10, NULL);

    // Task ADC? TO-DO?
    // xTaskCreate(adc_app_loop, "adc_receive_data_0", 1024 * 2, (void *)0, 10, NULL);

    /*
     * Bucle infinito TO-DO mejorar con Tasks?
     */
    while (1)
    {
        if (gpio_get_level(PIN_SWITCH))
        {
            ESP_LOGI(TAG, "Switch display: ON");
            s_switch_state = true; // Este estado se usa para encender o apagar el LCD TO-DO El que se encargue del I2C que meta esto.
            ssd1306_contrast(&dev, 0xff);
            sprintf(primeralineChar, " Tox pre: %02d", datoI2CCO2legible);
            sprintf(segundalineChar, " Tox pos: %02d", datoI2CFotonlegible);
            sprintf(terceralineChar, " V Hidro: %02d", voltajeHidro);
            sprintf(cuartalineChar,  " V Solar: %02d", voltajeSolar);
            ssd1306_display_text(&dev, 0, primeralineChar, 32, false);
	        ssd1306_display_text(&dev, 1, segundalineChar, 32, false);
	        ssd1306_display_text(&dev, 2, terceralineChar, 32, false);
	        ssd1306_display_text(&dev, 3, cuartalineChar, 32, false);
#if CONFIG_SSD1306_128x64
            ssd1306_display_text(&dev, 4, "                ", 32, false);
	        ssd1306_display_text(&dev, 5, "     GyFhi      ", 32, false);
	        ssd1306_display_text(&dev, 6, "                ", 32, false);
	        ssd1306_display_text(&dev, 7, "     U.P.M.     ", 32, false);
#endif // CONFIG_SSD1306_128x32
        }
        else
        {
            ESP_LOGI(TAG, "Switch display: OFF");
            s_switch_state = false;
            ssd1306_clear_screen(&dev, false);
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

        vTaskDelay(pdMS_TO_TICKS(2000)); // Delays para asegurar lecturas ADC correctas

#if CONFIG_IDF_TARGET_ESP32 // El WiFi usa en adc2 así que no podemos usar ese segundo módulo, mejor multiplexamos el adc1
        adc_raw[1][0] = adc1_get_raw(ADC2_EXAMPLE_CHAN0);
#else
        do
        {
            ret = adc2_get_raw(ADC2_EXAMPLE_CHAN0, ADC_WIDTH_BIT_DEFAULT, &adc_raw[1][0]);
        } while (ret == ESP_ERR_INVALID_STATE);
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

        vTaskDelay(pdMS_TO_TICKS(2000)); // Delays para asegurar lecturas ADC correctas

        /* Read the register, on power up the register should have the value 0xB5 */
        // ESP_LOGI(TAG, "Procedo a leer I2C de CO2"); // TO-DO REEMPLAZAR CON EL OTRO SENSOR
        // ESP_ERROR_CHECK(register_read(CO2_SENSOR_ADDR, CO2_REG_ADDR, 9));
        ESP_LOGI(TAG, "Procedo a leer I2C de TempHum"); // TO-DO REEMPLAZAR CON EL OTRO SENSOR
        ESP_ERROR_CHECK(tempHum_register_read(TEMPHUM_SENSOR_ADDR, COMANDO_TEMPHUM_MSB, COMANDO_TEMPHUM_LSB, 6));
        ESP_LOGI(TAG, "Procedo a leer I2C de Luminosidad");
        ESP_ERROR_CHECK(register_read_commando(LUZ_SENSOR_ADDR, LUZ_REG_ADDR, 2));

        vTaskDelay(pdMS_TO_TICKS(1000)); // El I2C puede tardar hasta 11 segundos
        /* Demonstrate writing by reseting the MPU9250 */
        //    ESP_ERROR_CHECK(mpu9250_register_write_byte(MPU9250_PWR_MGMT_1_REG_ADDR, 1 << MPU9250_RESET_BIT));

        //    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
        //    ESP_LOGI(TAG, "I2C unitialized successfully");

        /*
         *MQTT: los datos obtenidos los mandamos a Thingsboard
         */
        mqtt_app_start();

        // TO-DO Light sleep de varios segundos por RTC? AJUSTAR A LIGHT SLEEP
        //printf("Entering deep sleep\n");
        //gettimeofday(&sleep_enter_time, NULL);
        //esp_light_sleep_start();
        //esp_deep_sleep_start();
        printf("Entering light sleep\n");
        //esp_sleep_enable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
        esp_light_sleep_start();

        esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
		//ESP_LOGD(TAG, "WakeupFrom: %i", cause);
		if(cause == ESP_SLEEP_WAKEUP_GPIO)
		{
			esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO);
			//sleepManager_wakeNotify();
		}
    }
}