#pragma once

#include <stdbool.h>

#include "esp_zigbee_cluster.h"
#include "zcl/esp_zigbee_zcl_command.h"

#define S1_PRO_OTA_MANUFACTURER_CODE 0x1234U
#define S1_PRO_OTA_IMAGE_TYPE 0x5331U
#define S1_PRO_OTA_HARDWARE_VERSION 1U
#define S1_PRO_OTA_PARTITION_SIZE 0x180000U

esp_err_t zigbee_ota_add_cluster(esp_zb_cluster_list_t *clusters);
void zigbee_ota_network_changed(bool joined);
esp_err_t zigbee_ota_handle_upgrade(const esp_zb_zcl_ota_upgrade_value_message_t *message);
esp_err_t zigbee_ota_handle_query(const esp_zb_zcl_ota_upgrade_query_image_resp_message_t *message);
esp_err_t zigbee_ota_confirm_boot(void);
bool zigbee_ota_is_active(void);
