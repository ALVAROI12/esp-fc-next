# fc_link (P4 <-> C5 wire formats + AES-CCM radio envelope) is shared with the flight chip; built from fc_core in place.
idf_component_register(SRCS "main.c" "../../fc_core/src/fc_link.c" "../../fc_core/src/fc_link_aead.c"
                       INCLUDE_DIRS "../../fc_core/include"
                       PRIV_REQUIRES nvs_flash esp_app_format mbedtls)
