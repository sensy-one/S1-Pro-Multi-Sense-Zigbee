from typing import Final

import zigpy.profiles.zha as zha_profile
import zigpy.types as t
from zigpy.zcl.clusters.general import Groups, Identify, LevelControl, OnOff, Scenes
from zigpy.zcl.clusters.lighting import Color
from zigpy.zcl.foundation import BaseAttributeDefs, ZCLAttributeDef
from zhaquirks.builder import (
    BinarySensorDeviceClass,
    CONCENTRATION_PARTS_PER_MILLION,
    EntityType,
    LIGHT_LUX,
    PERCENTAGE,
    QuirkBuilder,
    ReportingConfig,
    SensorDeviceClass,
    SensorStateClass,
    UnitOfLength,
    UnitOfPressure,
    UnitOfTemperature,
)
from zhaquirks.clusters import CustomCluster


MANUFACTURER: Final = "Sensy-One"
MODEL: Final = "S1 Pro Multi Sense (Zigbee)"
MODEL_PREFIX: Final = "S1 Pro Multi Sense "
LEGACY_MODEL: Final = "S1-Pro-C6-LD2450"
ENDPOINT: Final = 10
CLUSTER_ID: Final = 0xFC00


class CentiDegreeScale:
    def __rtruediv__(self, value: float) -> int:
        return round(value * 100)

    def __rmul__(self, value: int) -> float:
        return value / 100


CENTI_DEGREE_SCALE: Final = CentiDegreeScale()


class BME688IAQClassification(t.enum8):
    Excellent = 0
    Good = 1
    Lightly_polluted = 2
    Moderately_polluted = 3
    Heavily_polluted = 4
    Severely_polluted = 5
    Extremely_polluted = 6


class RadarTargetState(t.enum8):
    No_target = 0
    Moving = 1
    Stationary = 2
    Holding = 3


IAQ_CLASSIFICATION_LABELS: Final = (
    "Excellent",
    "Good",
    "Lightly polluted",
    "Moderately polluted",
    "Heavily polluted",
    "Severely polluted",
    "Extremely polluted",
)

IAQ_ACCURACY_LABELS: Final = (
    "Stabilizing",
    "Low accuracy",
    "Medium accuracy",
    "High accuracy",
)

TARGET_STATE_LABELS: Final = (
    "No target",
    "Moving",
    "Stationary",
    "Holding",
)


def matches_s1_pro_model(device) -> bool:
    model = getattr(device, "model", None)
    if model in (MODEL, LEGACY_MODEL):
        return True
    if not isinstance(model, str) or not model.startswith(MODEL_PREFIX):
        return False
    suffix = model[len(MODEL_PREFIX) :]
    return len(suffix) == 6 and all(character in "0123456789abcdefABCDEF" for character in suffix)


def iaq_classification_label(value: int) -> str:
    index = int(value)
    if 0 <= index < len(IAQ_CLASSIFICATION_LABELS):
        return IAQ_CLASSIFICATION_LABELS[index]
    return "Unknown"


def iaq_accuracy_label(value: int) -> str:
    index = int(value)
    if 0 <= index < len(IAQ_ACCURACY_LABELS):
        return IAQ_ACCURACY_LABELS[index]
    return "Unknown"


def target_state_label(value: int) -> str:
    index = int(value)
    if 0 <= index < len(TARGET_STATE_LABELS):
        return TARGET_STATE_LABELS[index]
    return "Unknown"


class S1ProMmWaveCluster(CustomCluster):
    cluster_id = CLUSTER_ID
    ep_attribute = "s1_pro_mmwave"
    manufacturer_id_override = None

    class AttributeDefs(BaseAttributeDefs):
        target_1_x = ZCLAttributeDef(
            id=0x0000, type=t.int16s, access="rp", is_manufacturer_specific=False
        )
        target_1_y = ZCLAttributeDef(
            id=0x0001, type=t.int16s, access="rp", is_manufacturer_specific=False
        )
        target_1_distance = ZCLAttributeDef(
            id=0x0002, type=t.uint16_t, access="rp", is_manufacturer_specific=False
        )
        target_1_speed = ZCLAttributeDef(
            id=0x0003, type=t.int16s, access="rp", is_manufacturer_specific=False
        )
        target_1_state = ZCLAttributeDef(
            id=0x0004,
            type=RadarTargetState,
            access="rp",
            is_manufacturer_specific=False,
        )
        target_2_x = ZCLAttributeDef(
            id=0x0010, type=t.int16s, access="rp", is_manufacturer_specific=False
        )
        target_2_y = ZCLAttributeDef(
            id=0x0011, type=t.int16s, access="rp", is_manufacturer_specific=False
        )
        target_2_distance = ZCLAttributeDef(
            id=0x0012, type=t.uint16_t, access="rp", is_manufacturer_specific=False
        )
        target_2_speed = ZCLAttributeDef(
            id=0x0013, type=t.int16s, access="rp", is_manufacturer_specific=False
        )
        target_2_state = ZCLAttributeDef(
            id=0x0014,
            type=RadarTargetState,
            access="rp",
            is_manufacturer_specific=False,
        )
        target_3_x = ZCLAttributeDef(
            id=0x0020, type=t.int16s, access="rp", is_manufacturer_specific=False
        )
        target_3_y = ZCLAttributeDef(
            id=0x0021, type=t.int16s, access="rp", is_manufacturer_specific=False
        )
        target_3_distance = ZCLAttributeDef(
            id=0x0022, type=t.uint16_t, access="rp", is_manufacturer_specific=False
        )
        target_3_speed = ZCLAttributeDef(
            id=0x0023, type=t.int16s, access="rp", is_manufacturer_specific=False
        )
        target_3_state = ZCLAttributeDef(
            id=0x0024,
            type=RadarTargetState,
            access="rp",
            is_manufacturer_specific=False,
        )
        all_targets_count = ZCLAttributeDef(
            id=0x00F3, type=t.uint8_t, access="rp", is_manufacturer_specific=False
        )
        any_presence = ZCLAttributeDef(
            id=0x00F4, type=t.Bool, access="rp", is_manufacturer_specific=False
        )
        detection_range = ZCLAttributeDef(
            id=0x00F5,
            type=t.uint16_t,
            access="rwp",
            is_manufacturer_specific=False,
        )
        any_movement = ZCLAttributeDef(
            id=0x00F6, type=t.Bool, access="rp", is_manufacturer_specific=False
        )
        any_movement_threshold = ZCLAttributeDef(
            id=0x00F7,
            type=t.uint16_t,
            access="rwp",
            is_manufacturer_specific=False,
        )
        any_presence_delay = ZCLAttributeDef(
            id=0x00F8,
            type=t.uint16_t,
            access="rwp",
            is_manufacturer_specific=False,
        )
        radar_flip_y_axis = ZCLAttributeDef(
            id=0x00F9, type=t.Bool, access="rwp", is_manufacturer_specific=False
        )
        radar_bluetooth = ZCLAttributeDef(
            id=0x00FA, type=t.Bool, access="rwp", is_manufacturer_specific=False
        )
        radar_single_target = ZCLAttributeDef(
            id=0x00FB, type=t.Bool, access="rwp", is_manufacturer_specific=False
        )
        radar_restart_module = ZCLAttributeDef(
            id=0x00FC, type=t.Bool, access="rw", is_manufacturer_specific=False
        )
        radar_factory_reset = ZCLAttributeDef(
            id=0x00FD, type=t.Bool, access="rw", is_manufacturer_specific=False
        )
        esp32_restart_module = ZCLAttributeDef(
            id=0x00FE, type=t.Bool, access="rw", is_manufacturer_specific=False
        )
        esp32_factory_reset = ZCLAttributeDef(
            id=0x00FF, type=t.Bool, access="rw", is_manufacturer_specific=False
        )

        bme_temperature = ZCLAttributeDef(
            id=0x0100, type=t.int16s, access="rp", is_manufacturer_specific=False
        )
        bme_humidity = ZCLAttributeDef(
            id=0x0101, type=t.uint16_t, access="rp", is_manufacturer_specific=False
        )
        bme_pressure = ZCLAttributeDef(
            id=0x0102, type=t.uint32_t, access="rp", is_manufacturer_specific=False
        )
        bme_gas_resistance = ZCLAttributeDef(
            id=0x0103, type=t.uint32_t, access="rp", is_manufacturer_specific=False
        )
        bme_co2_equivalent = ZCLAttributeDef(
            id=0x0106, type=t.uint16_t, access="rp", is_manufacturer_specific=False
        )
        bme_voc_equivalent = ZCLAttributeDef(
            id=0x0107, type=t.uint16_t, access="rp", is_manufacturer_specific=False
        )
        bme_bsec_accuracy = ZCLAttributeDef(
            id=0x0108, type=t.uint8_t, access="rp", is_manufacturer_specific=False
        )
        bme_iaq = ZCLAttributeDef(
            id=0x010A, type=t.uint16_t, access="rp", is_manufacturer_specific=False
        )
        bme_iaq_classification = ZCLAttributeDef(
            id=0x010B,
            type=BME688IAQClassification,
            access="rp",
            is_manufacturer_specific=False,
        )
        bme_temperature_offset = ZCLAttributeDef(
            id=0x010C,
            type=t.uint16_t,
            access="rwp",
            is_manufacturer_specific=False,
        )
        scd40_co2 = ZCLAttributeDef(
            id=0x0200, type=t.uint16_t, access="rp", is_manufacturer_specific=False
        )
        scd40_temperature = ZCLAttributeDef(
            id=0x0201, type=t.int16s, access="rp", is_manufacturer_specific=False
        )
        scd40_humidity = ZCLAttributeDef(
            id=0x0202, type=t.uint16_t, access="rp", is_manufacturer_specific=False
        )
        scd40_calibration_reference = ZCLAttributeDef(
            id=0x0204,
            type=t.uint16_t,
            access="rwp",
            is_manufacturer_specific=False,
        )
        scd40_forced_calibration = ZCLAttributeDef(
            id=0x0205,
            type=t.Bool,
            access="rw",
            is_manufacturer_specific=False,
        )
        scd40_factory_reset = ZCLAttributeDef(
            id=0x0206,
            type=t.Bool,
            access="rw",
            is_manufacturer_specific=False,
        )
        scd40_temperature_offset = ZCLAttributeDef(
            id=0x0207,
            type=t.uint16_t,
            access="rwp",
            is_manufacturer_specific=False,
        )
        ltrf216a_illuminance = ZCLAttributeDef(
            id=0x0300, type=t.uint32_t, access="rp", is_manufacturer_specific=False
        )
        buzzer_power = ZCLAttributeDef(
            id=0x0400, type=t.Bool, access="rwp", is_manufacturer_specific=False
        )

        esp32_temperature = ZCLAttributeDef(
            id=0x0500, type=t.int16s, access="rp", is_manufacturer_specific=False
        )
        esp32_connected = ZCLAttributeDef(
            id=0x0501, type=t.Bool, access="rp", is_manufacturer_specific=False
        )
        holding_enabled = ZCLAttributeDef(
            id=0x0700, type=t.Bool, access="rwp", is_manufacturer_specific=False
        )
        holding_gate_radius = ZCLAttributeDef(
            id=0x0701,
            type=t.uint16_t,
            access="rwp",
            is_manufacturer_specific=False,
        )
        stationary_speed_threshold = ZCLAttributeDef(
            id=0x0702,
            type=t.uint16_t,
            access="rwp",
            is_manufacturer_specific=False,
        )
        stationary_time = ZCLAttributeDef(
            id=0x0703,
            type=t.uint16_t,
            access="rwp",
            is_manufacturer_specific=False,
        )
        dropout_hold_time = ZCLAttributeDef(
            id=0x0704,
            type=t.uint16_t,
            access="rwp",
            is_manufacturer_specific=False,
        )
        exclusion_zone_p1_x = ZCLAttributeDef(
            id=0x0600, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        exclusion_zone_p1_y = ZCLAttributeDef(
            id=0x0601, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        exclusion_zone_p2_x = ZCLAttributeDef(
            id=0x0602, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        exclusion_zone_p2_y = ZCLAttributeDef(
            id=0x0603, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        exclusion_zone_p3_x = ZCLAttributeDef(
            id=0x0604, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        exclusion_zone_p3_y = ZCLAttributeDef(
            id=0x0605, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        exclusion_zone_p4_x = ZCLAttributeDef(
            id=0x0606, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        exclusion_zone_p4_y = ZCLAttributeDef(
            id=0x0607, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        exclusion_zone_p5_x = ZCLAttributeDef(
            id=0x0608, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        exclusion_zone_p5_y = ZCLAttributeDef(
            id=0x0609, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        exclusion_zone_p6_x = ZCLAttributeDef(
            id=0x060A, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        exclusion_zone_p6_y = ZCLAttributeDef(
            id=0x060B, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        exclusion_zone_p7_x = ZCLAttributeDef(
            id=0x060C, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        exclusion_zone_p7_y = ZCLAttributeDef(
            id=0x060D, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        exclusion_zone_p8_x = ZCLAttributeDef(
            id=0x060E, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        exclusion_zone_p8_y = ZCLAttributeDef(
            id=0x060F, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        exclusion_zone_points_count = ZCLAttributeDef(
            id=0x0610, type=t.uint8_t, access="rwp", is_manufacturer_specific=False
        )
        zone_1_p1_x = ZCLAttributeDef(
            id=0x0620, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_1_p1_y = ZCLAttributeDef(
            id=0x0621, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_1_p2_x = ZCLAttributeDef(
            id=0x0622, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_1_p2_y = ZCLAttributeDef(
            id=0x0623, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_1_p3_x = ZCLAttributeDef(
            id=0x0624, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_1_p3_y = ZCLAttributeDef(
            id=0x0625, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_1_p4_x = ZCLAttributeDef(
            id=0x0626, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_1_p4_y = ZCLAttributeDef(
            id=0x0627, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_1_p5_x = ZCLAttributeDef(
            id=0x0628, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_1_p5_y = ZCLAttributeDef(
            id=0x0629, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_1_p6_x = ZCLAttributeDef(
            id=0x062A, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_1_p6_y = ZCLAttributeDef(
            id=0x062B, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_1_p7_x = ZCLAttributeDef(
            id=0x062C, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_1_p7_y = ZCLAttributeDef(
            id=0x062D, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_1_p8_x = ZCLAttributeDef(
            id=0x062E, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_1_p8_y = ZCLAttributeDef(
            id=0x062F, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_1_points_count = ZCLAttributeDef(
            id=0x0630, type=t.uint8_t, access="rwp", is_manufacturer_specific=False
        )
        zone_1_movement_threshold = ZCLAttributeDef(
            id=0x0631, type=t.uint16_t, access="rwp", is_manufacturer_specific=False
        )
        zone_1_presence_delay = ZCLAttributeDef(
            id=0x0632, type=t.uint16_t, access="rwp", is_manufacturer_specific=False
        )
        zone_1_presence = ZCLAttributeDef(
            id=0x0633, type=t.Bool, access="rp", is_manufacturer_specific=False
        )
        zone_1_movement = ZCLAttributeDef(
            id=0x0634, type=t.Bool, access="rp", is_manufacturer_specific=False
        )
        zone_1_target_count = ZCLAttributeDef(
            id=0x0635, type=t.uint8_t, access="rp", is_manufacturer_specific=False
        )
        zone_2_p1_x = ZCLAttributeDef(
            id=0x0640, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_2_p1_y = ZCLAttributeDef(
            id=0x0641, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_2_p2_x = ZCLAttributeDef(
            id=0x0642, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_2_p2_y = ZCLAttributeDef(
            id=0x0643, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_2_p3_x = ZCLAttributeDef(
            id=0x0644, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_2_p3_y = ZCLAttributeDef(
            id=0x0645, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_2_p4_x = ZCLAttributeDef(
            id=0x0646, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_2_p4_y = ZCLAttributeDef(
            id=0x0647, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_2_p5_x = ZCLAttributeDef(
            id=0x0648, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_2_p5_y = ZCLAttributeDef(
            id=0x0649, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_2_p6_x = ZCLAttributeDef(
            id=0x064A, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_2_p6_y = ZCLAttributeDef(
            id=0x064B, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_2_p7_x = ZCLAttributeDef(
            id=0x064C, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_2_p7_y = ZCLAttributeDef(
            id=0x064D, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_2_p8_x = ZCLAttributeDef(
            id=0x064E, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_2_p8_y = ZCLAttributeDef(
            id=0x064F, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_2_points_count = ZCLAttributeDef(
            id=0x0650, type=t.uint8_t, access="rwp", is_manufacturer_specific=False
        )
        zone_2_movement_threshold = ZCLAttributeDef(
            id=0x0651, type=t.uint16_t, access="rwp", is_manufacturer_specific=False
        )
        zone_2_presence_delay = ZCLAttributeDef(
            id=0x0652, type=t.uint16_t, access="rwp", is_manufacturer_specific=False
        )
        zone_2_presence = ZCLAttributeDef(
            id=0x0653, type=t.Bool, access="rp", is_manufacturer_specific=False
        )
        zone_2_movement = ZCLAttributeDef(
            id=0x0654, type=t.Bool, access="rp", is_manufacturer_specific=False
        )
        zone_2_target_count = ZCLAttributeDef(
            id=0x0655, type=t.uint8_t, access="rp", is_manufacturer_specific=False
        )
        zone_3_p1_x = ZCLAttributeDef(
            id=0x0660, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_3_p1_y = ZCLAttributeDef(
            id=0x0661, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_3_p2_x = ZCLAttributeDef(
            id=0x0662, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_3_p2_y = ZCLAttributeDef(
            id=0x0663, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_3_p3_x = ZCLAttributeDef(
            id=0x0664, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_3_p3_y = ZCLAttributeDef(
            id=0x0665, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_3_p4_x = ZCLAttributeDef(
            id=0x0666, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_3_p4_y = ZCLAttributeDef(
            id=0x0667, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_3_p5_x = ZCLAttributeDef(
            id=0x0668, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_3_p5_y = ZCLAttributeDef(
            id=0x0669, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_3_p6_x = ZCLAttributeDef(
            id=0x066A, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_3_p6_y = ZCLAttributeDef(
            id=0x066B, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_3_p7_x = ZCLAttributeDef(
            id=0x066C, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_3_p7_y = ZCLAttributeDef(
            id=0x066D, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_3_p8_x = ZCLAttributeDef(
            id=0x066E, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_3_p8_y = ZCLAttributeDef(
            id=0x066F, type=t.int16s, access="rwp", is_manufacturer_specific=False
        )
        zone_3_points_count = ZCLAttributeDef(
            id=0x0670, type=t.uint8_t, access="rwp", is_manufacturer_specific=False
        )
        zone_3_movement_threshold = ZCLAttributeDef(
            id=0x0671, type=t.uint16_t, access="rwp", is_manufacturer_specific=False
        )
        zone_3_presence_delay = ZCLAttributeDef(
            id=0x0672, type=t.uint16_t, access="rwp", is_manufacturer_specific=False
        )
        zone_3_presence = ZCLAttributeDef(
            id=0x0673, type=t.Bool, access="rp", is_manufacturer_specific=False
        )
        zone_3_movement = ZCLAttributeDef(
            id=0x0674, type=t.Bool, access="rp", is_manufacturer_specific=False
        )
        zone_3_target_count = ZCLAttributeDef(
            id=0x0675, type=t.uint8_t, access="rp", is_manufacturer_specific=False
        )


REPORT_EVERY_SECOND: Final = ReportingConfig(
    min_interval=1,
    max_interval=1,
    reportable_change=0,
)

REPORT_BME: Final = ReportingConfig(
    min_interval=3,
    max_interval=30,
    reportable_change=0,
)

REPORT_SCD40: Final = ReportingConfig(
    min_interval=5,
    max_interval=30,
    reportable_change=0,
)

REPORT_LTRF216A: Final = ReportingConfig(
    min_interval=3,
    max_interval=30,
    reportable_change=0,
)

REPORT_ESP32: Final = ReportingConfig(
    min_interval=10,
    max_interval=60,
    reportable_change=10,
)

builder = (
    QuirkBuilder(MANUFACTURER, None)
    .applies_to(MANUFACTURER, MODEL)
    .applies_to(MANUFACTURER, LEGACY_MODEL)
    .filter(matches_s1_pro_model)
    .replaces_endpoint(
        endpoint_id=ENDPOINT,
        device_type=zha_profile.DeviceType.COLOR_DIMMABLE_LIGHT,
    )
    .replaces(S1ProMmWaveCluster, endpoint_id=ENDPOINT)
    .removes(Identify, endpoint_id=ENDPOINT)
    .adds(Groups, endpoint_id=ENDPOINT)
    .adds(Scenes, endpoint_id=ENDPOINT)
    .adds(OnOff, endpoint_id=ENDPOINT)
    .adds(LevelControl, endpoint_id=ENDPOINT)
    .adds(Color, endpoint_id=ENDPOINT)
    .change_entity_metadata(
        endpoint_id=ENDPOINT,
        cluster_id=OnOff.cluster_id,
        unique_id_suffix="10",
        new_translation_key="ws2812_led",
        new_fallback_name="WS2812 LED",
        new_entity_category=EntityType.CONFIG,
    )
    .prevent_default_entity_creation(
        function=lambda entity: entity.__class__.__name__ == "RSSISensor",
    )
    .change_entity_metadata(
        function=lambda entity: entity.__class__.__name__ == "LQISensor",
        new_translation_key="link_quality",
        new_fallback_name="Link Quality",
        new_entity_category=EntityType.DIAGNOSTIC,
        new_entity_registry_enabled_default=True,
    )
)

builder.sensor(
    "esp32_temperature",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.DIAGNOSTIC,
    device_class=SensorDeviceClass.TEMPERATURE,
    state_class=SensorStateClass.MEASUREMENT,
    unit=UnitOfTemperature.CELSIUS,
    divisor=100,
    suggested_display_precision=2,
    reporting_config=REPORT_ESP32,
    unique_id_suffix="esp32_temperature",
    translation_key="esp32_temperature",
    fallback_name="ESP32 Temperature",
)

builder.binary_sensor(
    "esp32_connected",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.DIAGNOSTIC,
    device_class=BinarySensorDeviceClass.CONNECTIVITY,
    reporting_config=REPORT_ESP32,
    unique_id_suffix="esp32_status",
    translation_key="esp32_status",
    fallback_name="ESP32 Status",
)

for target in range(1, 4):
    for axis in ("x", "y"):
        attribute = f"target_{target}_{axis}"
        builder.sensor(
            attribute,
            CLUSTER_ID,
            endpoint_id=ENDPOINT,
            entity_type=EntityType.STANDARD,
            state_class=SensorStateClass.MEASUREMENT,
            unit=UnitOfLength.CENTIMETERS,
            divisor=10,
            suggested_display_precision=1,
            reporting_config=REPORT_EVERY_SECOND,
            unique_id_suffix=attribute,
            translation_key=attribute,
            fallback_name=f"Target {target} {axis.upper()}",
        )

    distance_attribute = f"target_{target}_distance"
    builder.sensor(
        distance_attribute,
        CLUSTER_ID,
        endpoint_id=ENDPOINT,
        entity_type=EntityType.STANDARD,
        state_class=SensorStateClass.MEASUREMENT,
        unit=UnitOfLength.CENTIMETERS,
        divisor=10,
        suggested_display_precision=1,
        reporting_config=REPORT_EVERY_SECOND,
        unique_id_suffix=distance_attribute,
        translation_key=distance_attribute,
        fallback_name=f"Target {target} distance",
    )

    speed_attribute = f"target_{target}_speed"
    builder.sensor(
        speed_attribute,
        CLUSTER_ID,
        endpoint_id=ENDPOINT,
        entity_type=EntityType.STANDARD,
        state_class=SensorStateClass.MEASUREMENT,
        unit="cm/s",
        suggested_display_precision=0,
        reporting_config=REPORT_EVERY_SECOND,
        unique_id_suffix=speed_attribute,
        translation_key=speed_attribute,
        fallback_name=f"Target {target} speed",
    )

    state_attribute = f"target_{target}_state"
    builder.sensor(
        state_attribute,
        CLUSTER_ID,
        endpoint_id=ENDPOINT,
        entity_type=EntityType.STANDARD,
        attribute_converter=target_state_label,
        reporting_config=REPORT_EVERY_SECOND,
        unique_id_suffix=state_attribute,
        translation_key=state_attribute,
        fallback_name=f"Target {target} state",
    )

builder.sensor(
    "all_targets_count",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.STANDARD,
    state_class=SensorStateClass.MEASUREMENT,
    suggested_display_precision=0,
    reporting_config=REPORT_EVERY_SECOND,
    unique_id_suffix="all_targets_count",
    translation_key="all_targets_count",
    fallback_name="All Targets Count",
)

builder.binary_sensor(
    "any_presence",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.STANDARD,
    device_class=BinarySensorDeviceClass.PRESENCE,
    reporting_config=REPORT_EVERY_SECOND,
    unique_id_suffix="any_presence",
    translation_key="any_presence",
    fallback_name="Any Presence",
)

builder.number(
    "detection_range",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.STANDARD,
    min_value=20,
    max_value=600,
    step=1,
    unit=UnitOfLength.CENTIMETERS,
    mode="box",
    reporting_config=REPORT_EVERY_SECOND,
    unique_id_suffix="detection_range",
    translation_key="detection_range",
    fallback_name="Detection Range",
)

builder.binary_sensor(
    "any_movement",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.STANDARD,
    device_class=BinarySensorDeviceClass.MOTION,
    reporting_config=REPORT_EVERY_SECOND,
    unique_id_suffix="any_movement",
    translation_key="any_movement",
    fallback_name="Any Movement",
)

builder.number(
    "any_movement_threshold",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.STANDARD,
    min_value=0,
    max_value=500,
    step=1,
    unit="cm/s",
    mode="box",
    reporting_config=REPORT_EVERY_SECOND,
    unique_id_suffix="any_movement_threshold",
    translation_key="any_movement_threshold",
    fallback_name="Any Movement Threshold",
)

builder.number(
    "any_presence_delay",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.STANDARD,
    min_value=0,
    max_value=300,
    step=1,
    unit="s",
    mode="box",
    reporting_config=REPORT_EVERY_SECOND,
    unique_id_suffix="any_presence_delay",
    translation_key="any_presence_delay",
    fallback_name="Any Presence Delay",
)

builder.switch(
    "holding_enabled",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.CONFIG,
    reporting_config=REPORT_EVERY_SECOND,
    unique_id_suffix="holding_enabled",
    translation_key="holding_enabled",
    fallback_name="Radar Holding Engine",
)

builder.number(
    "holding_gate_radius",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.STANDARD,
    min_value=5,
    max_value=300,
    step=1,
    unit=UnitOfLength.CENTIMETERS,
    mode="box",
    reporting_config=REPORT_EVERY_SECOND,
    unique_id_suffix="holding_gate_radius",
    translation_key="holding_gate_radius",
    fallback_name="Radar Holding Gate Radius",
)

builder.number(
    "stationary_speed_threshold",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.STANDARD,
    min_value=0,
    max_value=100,
    step=1,
    unit="cm/s",
    mode="box",
    reporting_config=REPORT_EVERY_SECOND,
    unique_id_suffix="stationary_speed_threshold",
    translation_key="stationary_speed_threshold",
    fallback_name="Radar Stationary Speed Threshold",
)

builder.number(
    "stationary_time",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.STANDARD,
    min_value=0,
    max_value=120,
    step=1,
    unit="s",
    mode="box",
    reporting_config=REPORT_EVERY_SECOND,
    unique_id_suffix="stationary_time",
    translation_key="stationary_time",
    fallback_name="Radar Stationary Time",
)

builder.number(
    "dropout_hold_time",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.STANDARD,
    min_value=0,
    max_value=60,
    step=1,
    unit="min",
    mode="box",
    reporting_config=REPORT_EVERY_SECOND,
    unique_id_suffix="dropout_hold_time",
    translation_key="dropout_hold_time",
    fallback_name="Radar Dropout Hold Time",
)

builder.number(
    "exclusion_zone_points_count",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.STANDARD,
    min_value=0,
    max_value=8,
    step=1,
    unit="pts",
    mode="box",
    unique_id_suffix="exclusion_zone_points_count",
    translation_key="exclusion_zone_points_count",
    fallback_name="Exclusion Zone Points Count",
)

for point in range(1, 9):
    for axis in ("x", "y"):
        attribute = f"exclusion_zone_p{point}_{axis}"
        builder.number(
            attribute,
            CLUSTER_ID,
            endpoint_id=ENDPOINT,
            entity_type=EntityType.STANDARD,
            min_value=-600,
            max_value=600,
            step=1,
            unit=UnitOfLength.CENTIMETERS,
            mode="box",
            unique_id_suffix=attribute,
            translation_key=attribute,
            fallback_name=f"Exclusion Zone P{point} {axis.upper()}",
        )

for zone in range(1, 4):
    movement_threshold_attribute = f"zone_{zone}_movement_threshold"
    builder.number(
        movement_threshold_attribute,
        CLUSTER_ID,
        endpoint_id=ENDPOINT,
        entity_type=EntityType.STANDARD,
        min_value=0,
        max_value=500,
        step=1,
        unit="cm/s",
        mode="box",
        unique_id_suffix=movement_threshold_attribute,
        translation_key=movement_threshold_attribute,
        fallback_name=f"Zone {zone} Movement Threshold",
    )

    for point in range(1, 9):
        for axis in ("x", "y"):
            attribute = f"zone_{zone}_p{point}_{axis}"
            builder.number(
                attribute,
                CLUSTER_ID,
                endpoint_id=ENDPOINT,
                entity_type=EntityType.STANDARD,
                min_value=-600,
                max_value=600,
                step=1,
                unit=UnitOfLength.CENTIMETERS,
                mode="box",
                unique_id_suffix=attribute,
                translation_key=attribute,
                fallback_name=f"Zone {zone} P{point} {axis.upper()}",
            )

    points_count_attribute = f"zone_{zone}_points_count"
    builder.number(
        points_count_attribute,
        CLUSTER_ID,
        endpoint_id=ENDPOINT,
        entity_type=EntityType.STANDARD,
        min_value=0,
        max_value=8,
        step=1,
        unit="pts",
        mode="box",
        unique_id_suffix=points_count_attribute,
        translation_key=points_count_attribute,
        fallback_name=f"Zone {zone} Points Count",
    )

    presence_delay_attribute = f"zone_{zone}_presence_delay"
    builder.number(
        presence_delay_attribute,
        CLUSTER_ID,
        endpoint_id=ENDPOINT,
        entity_type=EntityType.STANDARD,
        min_value=0,
        max_value=300,
        step=1,
        unit="s",
        mode="box",
        unique_id_suffix=presence_delay_attribute,
        translation_key=presence_delay_attribute,
        fallback_name=f"Zone {zone} Presence Delay",
    )

    presence_attribute = f"zone_{zone}_presence"
    builder.binary_sensor(
        presence_attribute,
        CLUSTER_ID,
        endpoint_id=ENDPOINT,
        entity_type=EntityType.STANDARD,
        device_class=BinarySensorDeviceClass.PRESENCE,
        unique_id_suffix=presence_attribute,
        translation_key=presence_attribute,
        fallback_name=f"Zone {zone} Presence",
    )

    movement_attribute = f"zone_{zone}_movement"
    builder.binary_sensor(
        movement_attribute,
        CLUSTER_ID,
        endpoint_id=ENDPOINT,
        entity_type=EntityType.STANDARD,
        device_class=BinarySensorDeviceClass.MOTION,
        unique_id_suffix=movement_attribute,
        translation_key=movement_attribute,
        fallback_name=f"Zone {zone} Movement",
    )

    target_count_attribute = f"zone_{zone}_target_count"
    builder.sensor(
        target_count_attribute,
        CLUSTER_ID,
        endpoint_id=ENDPOINT,
        entity_type=EntityType.STANDARD,
        state_class=SensorStateClass.MEASUREMENT,
        suggested_display_precision=0,
        unique_id_suffix=target_count_attribute,
        translation_key=target_count_attribute,
        fallback_name=f"Zone {zone} Target Count",
    )

builder.switch(
    "radar_flip_y_axis",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.CONFIG,
    reporting_config=REPORT_EVERY_SECOND,
    unique_id_suffix="radar_flip_y_axis",
    translation_key="radar_flip_y_axis",
    fallback_name="Radar Flip Y Axis",
)

builder.switch(
    "radar_bluetooth",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.CONFIG,
    reporting_config=REPORT_EVERY_SECOND,
    unique_id_suffix="radar_bluetooth",
    translation_key="radar_bluetooth",
    fallback_name="Radar Bluetooth",
)

builder.switch(
    "radar_single_target",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.CONFIG,
    reporting_config=REPORT_EVERY_SECOND,
    unique_id_suffix="radar_single_target",
    translation_key="radar_single_target",
    fallback_name="Radar Single Target",
)

builder.write_attr_button(
    "radar_restart_module",
    1,
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.CONFIG,
    unique_id_suffix="radar_restart_module",
    translation_key="radar_restart_module",
    fallback_name="Radar Restart Module",
)

builder.write_attr_button(
    "radar_factory_reset",
    1,
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.CONFIG,
    unique_id_suffix="radar_factory_reset",
    translation_key="radar_factory_reset",
    fallback_name="Radar Factory Reset",
)

builder.write_attr_button(
    "esp32_restart_module",
    1,
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.CONFIG,
    unique_id_suffix="esp32_restart_module",
    translation_key="esp32_restart_module",
    fallback_name="ESP32 Restart Module",
)

builder.write_attr_button(
    "esp32_factory_reset",
    1,
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.CONFIG,
    unique_id_suffix="esp32_factory_reset",
    translation_key="esp32_factory_reset",
    fallback_name="ESP32 Factory Reset",
)

builder.sensor(
    "bme_temperature",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.STANDARD,
    device_class=SensorDeviceClass.TEMPERATURE,
    state_class=SensorStateClass.MEASUREMENT,
    unit=UnitOfTemperature.CELSIUS,
    divisor=100,
    suggested_display_precision=2,
    reporting_config=REPORT_BME,
    unique_id_suffix="bme_temperature",
    translation_key="bme_temperature",
    fallback_name="BME688 temperature",
)

builder.sensor(
    "bme_humidity",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.STANDARD,
    device_class=SensorDeviceClass.HUMIDITY,
    state_class=SensorStateClass.MEASUREMENT,
    unit=PERCENTAGE,
    divisor=100,
    suggested_display_precision=2,
    reporting_config=REPORT_BME,
    unique_id_suffix="bme_humidity",
    translation_key="bme_humidity",
    fallback_name="BME688 humidity",
)

builder.sensor(
    "bme_pressure",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.STANDARD,
    device_class=SensorDeviceClass.ATMOSPHERIC_PRESSURE,
    state_class=SensorStateClass.MEASUREMENT,
    unit=UnitOfPressure.HPA,
    divisor=100,
    suggested_display_precision=2,
    reporting_config=REPORT_BME,
    unique_id_suffix="bme_pressure",
    translation_key="bme_pressure",
    fallback_name="BME688 pressure",
)

builder.sensor(
    "bme_gas_resistance",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.STANDARD,
    state_class=SensorStateClass.MEASUREMENT,
    unit="kΩ",
    divisor=1000,
    suggested_display_precision=2,
    reporting_config=REPORT_BME,
    unique_id_suffix="bme_gas_resistance",
    translation_key="bme_gas_resistance",
    fallback_name="BME688 gas resistance",
)

builder.sensor(
    "bme_co2_equivalent",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.STANDARD,
    device_class=SensorDeviceClass.CO2,
    state_class=SensorStateClass.MEASUREMENT,
    unit=CONCENTRATION_PARTS_PER_MILLION,
    suggested_display_precision=0,
    reporting_config=REPORT_BME,
    unique_id_suffix="bme_co2_equivalent",
    translation_key="bme_co2_equivalent",
    fallback_name="BME688 CO2 equivalent",
)

builder.sensor(
    "bme_voc_equivalent",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.STANDARD,
    device_class=SensorDeviceClass.VOLATILE_ORGANIC_COMPOUNDS_PARTS,
    state_class=SensorStateClass.MEASUREMENT,
    unit=CONCENTRATION_PARTS_PER_MILLION,
    divisor=100,
    suggested_display_precision=2,
    reporting_config=REPORT_BME,
    unique_id_suffix="bme_voc_equivalent",
    translation_key="bme_voc_equivalent",
    fallback_name="BME688 VOC equivalent",
)

builder.sensor(
    "bme_bsec_accuracy",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.STANDARD,
    attribute_converter=iaq_accuracy_label,
    reporting_config=REPORT_BME,
    unique_id_suffix="bme_bsec_accuracy",
    translation_key="bme_bsec_accuracy",
    fallback_name="BME688 IAQ Accuracy",
)

builder.sensor(
    "bme_iaq",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.STANDARD,
    device_class=SensorDeviceClass.AQI,
    state_class=SensorStateClass.MEASUREMENT,
    suggested_display_precision=0,
    reporting_config=REPORT_BME,
    unique_id_suffix="bme_iaq",
    translation_key="bme_iaq",
    fallback_name="BME688 IAQ",
)

builder.sensor(
    "bme_iaq_classification",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.STANDARD,
    attribute_converter=iaq_classification_label,
    reporting_config=REPORT_BME,
    unique_id_suffix="bme_iaq_classification",
    translation_key="bme_iaq_classification",
    fallback_name="BME688 IAQ Classification",
)

builder.number(
    "bme_temperature_offset",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.STANDARD,
    min_value=0,
    max_value=30,
    step=0.01,
    multiplier=CENTI_DEGREE_SCALE,
    unit=UnitOfTemperature.CELSIUS,
    mode="box",
    reporting_config=REPORT_BME,
    unique_id_suffix="bme_temperature_offset",
    translation_key="bme_temperature_offset",
    fallback_name="BME688 Temp Offset",
)

builder.sensor(
    "scd40_co2",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.STANDARD,
    device_class=SensorDeviceClass.CO2,
    state_class=SensorStateClass.MEASUREMENT,
    unit=CONCENTRATION_PARTS_PER_MILLION,
    suggested_display_precision=0,
    reporting_config=REPORT_SCD40,
    unique_id_suffix="scd40_co2",
    translation_key="scd40_co2",
    fallback_name="SCD40 CO2",
)

builder.sensor(
    "scd40_temperature",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.STANDARD,
    device_class=SensorDeviceClass.TEMPERATURE,
    state_class=SensorStateClass.MEASUREMENT,
    unit=UnitOfTemperature.CELSIUS,
    divisor=100,
    suggested_display_precision=2,
    reporting_config=REPORT_SCD40,
    unique_id_suffix="scd40_temperature",
    translation_key="scd40_temperature",
    fallback_name="SCD40 temperature",
)

builder.sensor(
    "scd40_humidity",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.STANDARD,
    device_class=SensorDeviceClass.HUMIDITY,
    state_class=SensorStateClass.MEASUREMENT,
    unit=PERCENTAGE,
    divisor=100,
    suggested_display_precision=2,
    reporting_config=REPORT_SCD40,
    unique_id_suffix="scd40_humidity",
    translation_key="scd40_humidity",
    fallback_name="SCD40 humidity",
)

builder.number(
    "scd40_calibration_reference",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.STANDARD,
    min_value=400,
    max_value=2000,
    step=1,
    unit=CONCENTRATION_PARTS_PER_MILLION,
    mode="box",
    reporting_config=REPORT_SCD40,
    unique_id_suffix="scd40_calibration_reference",
    translation_key="scd40_calibration_reference",
    fallback_name="SCD40 Calibration Reference",
)

builder.write_attr_button(
    "scd40_forced_calibration",
    1,
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.CONFIG,
    unique_id_suffix="scd40_forced_calibration",
    translation_key="scd40_forced_calibration",
    fallback_name="SCD40 Forced Calibration",
)

builder.write_attr_button(
    "scd40_factory_reset",
    1,
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.CONFIG,
    unique_id_suffix="scd40_factory_reset",
    translation_key="scd40_factory_reset",
    fallback_name="SCD40 Factory Reset",
)

builder.number(
    "scd40_temperature_offset",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.STANDARD,
    min_value=0,
    max_value=30,
    step=0.01,
    multiplier=CENTI_DEGREE_SCALE,
    unit=UnitOfTemperature.CELSIUS,
    mode="box",
    reporting_config=REPORT_SCD40,
    unique_id_suffix="scd40_temperature_offset",
    translation_key="scd40_temperature_offset",
    fallback_name="SCD40 Temp Offset",
)

builder.sensor(
    "ltrf216a_illuminance",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.STANDARD,
    device_class=SensorDeviceClass.ILLUMINANCE,
    state_class=SensorStateClass.MEASUREMENT,
    unit=LIGHT_LUX,
    divisor=100,
    suggested_display_precision=2,
    reporting_config=REPORT_LTRF216A,
    unique_id_suffix="ltrf216a_illuminance",
    translation_key="ltrf216a_illuminance",
    fallback_name="LTR-F216A Illuminance",
)

builder.switch(
    "buzzer_power",
    CLUSTER_ID,
    endpoint_id=ENDPOINT,
    entity_type=EntityType.CONFIG,
    reporting_config=REPORT_EVERY_SECOND,
    unique_id_suffix="buzzer_power",
    translation_key="buzzer_power",
    fallback_name="MLT8530 Buzzer",
)

builder.add_to_registry()
