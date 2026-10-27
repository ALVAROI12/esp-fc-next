# fc_link (P4 <-> C5 wire formats) is shared with the flight chip; built from fc_core in place.
idf_component_register(SRCS "main.c" "../../fc_core/src/fc_link.c"
                       INCLUDE_DIRS "../../fc_core/include"
                       PRIV_REQUIRES nvs_flash esp_app_format)
