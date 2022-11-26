idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    EMBED_TXTFILES "certs/http2_telegram_root_cert.pem"
                                   "certs/cacert.pem"
                                   "certs/prvtkey.pem")
