#include "zigbee_ota.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_zigbee_core.h"
#include "esp_zigbee_ota.h"
#include "ota_version.h"
#include "s1_pro_config.h"
#include "zcl/esp_zigbee_zcl_ota.h"
#include "zdo/esp_zigbee_zdo_command.h"

typedef struct {
    const esp_partition_t *partition;
    esp_ota_handle_t handle;
    bool active;
    bool open;
    bool validated;
    uint8_t element_header[6];
    size_t header_received;
    uint32_t file_version;
    uint32_t total_size;
    uint32_t received;
    uint32_t image_size;
    uint32_t written;
} zigbee_ota_state_t;

static zigbee_ota_state_t s_ota;
static bool s_joined;

static void reset_download(void)
{
    if (s_ota.open) {
        esp_ota_abort(s_ota.handle);
    }
    memset(&s_ota, 0, sizeof(s_ota));
}

static esp_err_t fail_download(esp_err_t error)
{
    reset_download();
    return error;
}

static bool compatible_image(uint16_t manufacturer, uint16_t image_type,
                             uint32_t file_version)
{
    return manufacturer == S1_PRO_OTA_MANUFACTURER_CODE &&
           image_type == S1_PRO_OTA_IMAGE_TYPE &&
           file_version > S1_PRO_OTA_FILE_VERSION;
}

static void restart_after_upgrade(uint8_t argument)
{
    (void)argument;
    esp_restart();
}

static esp_err_t receive_image(const esp_zb_zcl_ota_upgrade_value_message_t *message)
{
    const esp_zb_ota_file_header_t *header = &message->ota_header;
    if (!s_ota.active || s_ota.validated ||
        !compatible_image(header->manufacturer_code, header->image_type,
                          header->file_version) ||
        header->image_size <= sizeof(s_ota.element_header) ||
        header->image_size > s_ota.partition->size + sizeof(s_ota.element_header) ||
        (message->payload_size != 0U && message->payload == NULL)) {
        return fail_download(ESP_ERR_INVALID_ARG);
    }
    if ((header->field_control & ESP_ZB_ZCL_OTA_UPGRADE_FILE_HEADER_FC_HW_VER) &&
        (header->optional.minimum_hardware_version > S1_PRO_OTA_HARDWARE_VERSION ||
         header->optional.maximum_hardware_version < S1_PRO_OTA_HARDWARE_VERSION)) {
        return fail_download(ESP_ERR_NOT_SUPPORTED);
    }
    if (s_ota.total_size == 0U) {
        s_ota.total_size = header->image_size;
        s_ota.file_version = header->file_version;
    }
    if (header->image_size != s_ota.total_size ||
        header->file_version != s_ota.file_version ||
        message->payload_size > s_ota.total_size - s_ota.received) {
        return fail_download(ESP_ERR_INVALID_SIZE);
    }

    const uint8_t *data = message->payload;
    size_t remaining = message->payload_size;
    s_ota.received += message->payload_size;
    if (s_ota.header_received < sizeof(s_ota.element_header)) {
        size_t count = sizeof(s_ota.element_header) - s_ota.header_received;
        if (count > remaining) {
            count = remaining;
        }
        if (count != 0U) {
            memcpy(s_ota.element_header + s_ota.header_received, data, count);
            s_ota.header_received += count;
            data += count;
            remaining -= count;
        }
        if (s_ota.header_received < sizeof(s_ota.element_header)) {
            return ESP_OK;
        }
        const uint8_t *element = s_ota.element_header;
        s_ota.image_size = (uint32_t)element[2] | ((uint32_t)element[3] << 8U) |
                           ((uint32_t)element[4] << 16U) | ((uint32_t)element[5] << 24U);
        if (element[0] != 0U || element[1] != 0U || s_ota.image_size == 0U ||
            s_ota.image_size > s_ota.partition->size ||
            s_ota.image_size != s_ota.total_size - sizeof(s_ota.element_header)) {
            return fail_download(ESP_ERR_INVALID_SIZE);
        }
        const esp_err_t status = esp_ota_begin(s_ota.partition, OTA_WITH_SEQUENTIAL_WRITES,
                                               &s_ota.handle);
        if (status != ESP_OK) {
            return fail_download(status);
        }
        s_ota.open = true;
    }
    if (remaining > s_ota.image_size - s_ota.written) {
        return fail_download(ESP_ERR_INVALID_SIZE);
    }
    if (remaining != 0U) {
        const esp_err_t status = esp_ota_write(s_ota.handle, data, remaining);
        if (status != ESP_OK) {
            return fail_download(status);
        }
        s_ota.written += remaining;
    }
    return ESP_OK;
}

esp_err_t zigbee_ota_handle_upgrade(const esp_zb_zcl_ota_upgrade_value_message_t *message)
{
    if (message == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (message->info.status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        return fail_download(ESP_FAIL);
    }
    switch (message->upgrade_status) {
    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_START:
        reset_download();
        s_ota.partition = esp_ota_get_next_update_partition(NULL);
        if (s_ota.partition == NULL) {
            return ESP_ERR_NOT_FOUND;
        }
        s_ota.active = true;
        return ESP_OK;
    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_RECEIVE:
        return receive_image(message);
    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_CHECK: {
        if (!s_ota.active || !s_ota.open || s_ota.received != s_ota.total_size ||
            s_ota.written != s_ota.image_size) {
            return fail_download(ESP_ERR_INVALID_SIZE);
        }
        s_ota.open = false;
        const esp_err_t status = esp_ota_end(s_ota.handle);
        if (status != ESP_OK) {
            return fail_download(status);
        }
        s_ota.validated = true;
        return ESP_OK;
    }
    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_FINISH: {
        if (!s_ota.active || !s_ota.validated) {
            return fail_download(ESP_ERR_INVALID_STATE);
        }
        const esp_err_t status = esp_ota_set_boot_partition(s_ota.partition);
        if (status != ESP_OK) {
            return fail_download(status);
        }
        esp_zb_scheduler_alarm(restart_after_upgrade, 0, 1000);
        return ESP_OK;
    }
    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_ABORT:
    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_ERROR:
    case ESP_ZB_ZCL_OTA_UPGRADE_STATUS_SERVER_NOT_FOUND:
        reset_download();
        return ESP_OK;
    default:
        return ESP_OK;
    }
}

esp_err_t zigbee_ota_handle_query(const esp_zb_zcl_ota_upgrade_query_image_resp_message_t *message)
{
    if (message == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (message->info.status != ESP_ZB_ZCL_STATUS_SUCCESS ||
        message->query_status != ESP_ZB_ZCL_STATUS_SUCCESS) {
        return ESP_OK;
    }
    const esp_partition_t *partition = esp_ota_get_next_update_partition(NULL);
    if (s_ota.active || partition == NULL ||
        !compatible_image(message->manufacturer_code, message->image_type,
                          message->file_version) ||
        message->image_size <= 62U || message->image_size > partition->size + 66U) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    return ESP_OK;
}

static void discover_server(uint8_t argument);

static void server_found(esp_zb_zdp_status_t status, uint16_t address,
                          uint8_t endpoint, void *context)
{
    (void)context;
    if (!s_joined) {
        return;
    }
    if (status == ESP_ZB_ZDP_STATUS_SUCCESS &&
        esp_zb_ota_upgrade_client_query_interval_set(S1_PRO_ENDPOINT_ID, 1440) == ESP_OK &&
        esp_zb_ota_upgrade_client_query_image_req(address, endpoint) == ESP_OK) {
        return;
    }
    esp_zb_scheduler_alarm(discover_server, 0, 60000);
}

static void discover_server(uint8_t argument)
{
    (void)argument;
    if (!s_joined) {
        return;
    }
    uint16_t cluster = ESP_ZB_ZCL_CLUSTER_ID_OTA_UPGRADE;
    esp_zb_zdo_match_desc_req_param_t request = {
        .addr_of_interest = S1_PRO_COORDINATOR_SHORT_ADDRESS,
        .dst_nwk_addr = S1_PRO_COORDINATOR_SHORT_ADDRESS,
        .num_in_clusters = 1,
        .num_out_clusters = 0,
        .profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .cluster_list = &cluster,
    };
    if (esp_zb_zdo_match_cluster(&request, server_found, NULL) != ESP_OK) {
        esp_zb_scheduler_alarm(discover_server, 0, 60000);
    }
}

void zigbee_ota_network_changed(bool joined)
{
    s_joined = joined;
    esp_zb_scheduler_alarm_cancel(discover_server, 0);
    if (joined) {
        esp_zb_scheduler_alarm(discover_server, 0, 3000);
    } else {
        reset_download();
    }
}

esp_err_t zigbee_ota_add_cluster(esp_zb_cluster_list_t *clusters)
{
    esp_zb_ota_cluster_cfg_t configuration = {
        .ota_upgrade_file_version = S1_PRO_OTA_FILE_VERSION,
        .ota_upgrade_downloaded_file_ver = UINT32_MAX,
        .ota_upgrade_manufacturer = S1_PRO_OTA_MANUFACTURER_CODE,
        .ota_upgrade_image_type = S1_PRO_OTA_IMAGE_TYPE,
        .ota_upgrade_server_id = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    };
    static esp_zb_zcl_ota_upgrade_client_variable_t variables = {
        .timer_query = 1440,
        .hw_version = S1_PRO_OTA_HARDWARE_VERSION,
        .max_data_size = 64,
    };
    static uint16_t server_address = 0xffff;
    static uint8_t server_endpoint = 0xff;
    esp_zb_attribute_list_t *cluster = esp_zb_ota_cluster_create(&configuration);
    if (cluster == NULL) {
        return ESP_ERR_NO_MEM;
    }
    esp_err_t status = esp_zb_ota_cluster_add_attr(cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_CLIENT_DATA_ID,
                                                  &variables);
    if (status == ESP_OK) {
        status = esp_zb_ota_cluster_add_attr(cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ADDR_ID,
                                            &server_address);
    }
    if (status == ESP_OK) {
        status = esp_zb_ota_cluster_add_attr(cluster, ESP_ZB_ZCL_ATTR_OTA_UPGRADE_SERVER_ENDPOINT_ID,
                                            &server_endpoint);
    }
    if (status == ESP_OK) {
        status = esp_zb_cluster_list_add_ota_cluster(clusters, cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    }
    return status;
}

esp_err_t zigbee_ota_confirm_boot(void)
{
    esp_ota_img_states_t state;
    const esp_partition_t *partition = esp_ota_get_running_partition();
    if (partition != NULL && esp_ota_get_state_partition(partition, &state) == ESP_OK &&
        state == ESP_OTA_IMG_PENDING_VERIFY) {
        return esp_ota_mark_app_valid_cancel_rollback();
    }
    return ESP_OK;
}

bool zigbee_ota_is_active(void)
{
    return s_ota.active;
}
