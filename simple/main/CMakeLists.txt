idf_component_register(SRCS "cmd_i2ctools.c" "main.c"
                    INCLUDE_DIRS "."
                    EMBED_TXTFILES "certs/cacert.pem"
                                   "certs/prvtkey.pem")
