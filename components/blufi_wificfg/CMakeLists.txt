idf_component_register(SRCS "blufi_wificfg.c"
                            "blufi_security.c"
                            "blufi_init.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver esp_wifi bt mbedtls)
