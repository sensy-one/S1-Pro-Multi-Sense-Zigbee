import {Zcl} from "zigbee-herdsman";
import * as m from "zigbee-herdsman-converters/lib/modernExtend";

const ENDPOINT = 10;
const CLUSTER_NAME = "sensyOneS1Pro";
const CLUSTER_ID = 0xfc00;

const attributes = {};

function addAttribute(name, ID, type, writable = false) {
    const definition = {name, ID, type};
    if (writable) definition.write = true;
    if (type === Zcl.DataType.INT16) definition.min = -32768;
    if (type === Zcl.DataType.UINT8 || type === Zcl.DataType.ENUM8) definition.max = 0xff;
    if (type === Zcl.DataType.UINT16) definition.max = 0xffff;
    if (type === Zcl.DataType.UINT32) definition.max = 0xffffffff;
    attributes[name] = definition;
}

for (let target = 1; target <= 3; target++) {
    const base = (target - 1) * 0x10;
    addAttribute(`target_${target}_x`, base, Zcl.DataType.INT16);
    addAttribute(`target_${target}_y`, base + 1, Zcl.DataType.INT16);
    addAttribute(`target_${target}_distance`, base + 2, Zcl.DataType.UINT16);
    addAttribute(`target_${target}_speed`, base + 3, Zcl.DataType.INT16);
    addAttribute(`target_${target}_state`, base + 4, Zcl.DataType.ENUM8);
}

addAttribute("all_targets_count", 0x00f3, Zcl.DataType.UINT8);
addAttribute("any_presence", 0x00f4, Zcl.DataType.BOOLEAN);
addAttribute("detection_range", 0x00f5, Zcl.DataType.UINT16, true);
addAttribute("any_movement", 0x00f6, Zcl.DataType.BOOLEAN);
addAttribute("any_movement_threshold", 0x00f7, Zcl.DataType.UINT16, true);
addAttribute("any_presence_delay", 0x00f8, Zcl.DataType.UINT16, true);
addAttribute("radar_flip_y_axis", 0x00f9, Zcl.DataType.BOOLEAN, true);
addAttribute("radar_bluetooth", 0x00fa, Zcl.DataType.BOOLEAN, true);
addAttribute("radar_single_target", 0x00fb, Zcl.DataType.BOOLEAN, true);
addAttribute("radar_restart_module", 0x00fc, Zcl.DataType.BOOLEAN, true);
addAttribute("radar_factory_reset", 0x00fd, Zcl.DataType.BOOLEAN, true);
addAttribute("esp32_restart_module", 0x00fe, Zcl.DataType.BOOLEAN, true);
addAttribute("esp32_factory_reset", 0x00ff, Zcl.DataType.BOOLEAN, true);

addAttribute("bme_temperature", 0x0100, Zcl.DataType.INT16);
addAttribute("bme_humidity", 0x0101, Zcl.DataType.UINT16);
addAttribute("bme_pressure", 0x0102, Zcl.DataType.UINT32);
addAttribute("bme_gas_resistance", 0x0103, Zcl.DataType.UINT32);
addAttribute("bme_co2_equivalent", 0x0106, Zcl.DataType.UINT16);
addAttribute("bme_voc_equivalent", 0x0107, Zcl.DataType.UINT16);
addAttribute("bme_bsec_accuracy", 0x0108, Zcl.DataType.UINT8);
addAttribute("bme_iaq", 0x010a, Zcl.DataType.UINT16);
addAttribute("bme_iaq_classification", 0x010b, Zcl.DataType.ENUM8);
addAttribute("bme_temperature_offset", 0x010c, Zcl.DataType.UINT16, true);

addAttribute("scd40_co2", 0x0200, Zcl.DataType.UINT16);
addAttribute("scd40_temperature", 0x0201, Zcl.DataType.INT16);
addAttribute("scd40_humidity", 0x0202, Zcl.DataType.UINT16);
addAttribute("scd40_calibration_reference", 0x0204, Zcl.DataType.UINT16, true);
addAttribute("scd40_forced_calibration", 0x0205, Zcl.DataType.BOOLEAN, true);
addAttribute("scd40_factory_reset", 0x0206, Zcl.DataType.BOOLEAN, true);
addAttribute("scd40_temperature_offset", 0x0207, Zcl.DataType.UINT16, true);

addAttribute("ltrf216a_illuminance", 0x0300, Zcl.DataType.UINT32);
addAttribute("buzzer_power", 0x0400, Zcl.DataType.BOOLEAN, true);
addAttribute("esp32_temperature", 0x0500, Zcl.DataType.INT16);
addAttribute("esp32_connected", 0x0501, Zcl.DataType.BOOLEAN);

addAttribute("holding_enabled", 0x0700, Zcl.DataType.BOOLEAN, true);
addAttribute("holding_gate_radius", 0x0701, Zcl.DataType.UINT16, true);
addAttribute("stationary_speed_threshold", 0x0702, Zcl.DataType.UINT16, true);
addAttribute("stationary_time", 0x0703, Zcl.DataType.UINT16, true);
addAttribute("dropout_hold_time", 0x0704, Zcl.DataType.UINT16, true);

for (let point = 1; point <= 8; point++) {
    const base = 0x0600 + (point - 1) * 2;
    addAttribute(`exclusion_zone_p${point}_x`, base, Zcl.DataType.INT16, true);
    addAttribute(`exclusion_zone_p${point}_y`, base + 1, Zcl.DataType.INT16, true);
}
addAttribute("exclusion_zone_points_count", 0x0610, Zcl.DataType.UINT8, true);

for (let zone = 1; zone <= 3; zone++) {
    const base = 0x0620 + (zone - 1) * 0x20;
    for (let point = 1; point <= 8; point++) {
        const pointBase = base + (point - 1) * 2;
        addAttribute(`zone_${zone}_p${point}_x`, pointBase, Zcl.DataType.INT16, true);
        addAttribute(`zone_${zone}_p${point}_y`, pointBase + 1, Zcl.DataType.INT16, true);
    }
    addAttribute(`zone_${zone}_points_count`, base + 0x10, Zcl.DataType.UINT8, true);
    addAttribute(`zone_${zone}_movement_threshold`, base + 0x11, Zcl.DataType.UINT16, true);
    addAttribute(`zone_${zone}_presence_delay`, base + 0x12, Zcl.DataType.UINT16, true);
    addAttribute(`zone_${zone}_presence`, base + 0x13, Zcl.DataType.BOOLEAN);
    addAttribute(`zone_${zone}_movement`, base + 0x14, Zcl.DataType.BOOLEAN);
    addAttribute(`zone_${zone}_target_count`, base + 0x15, Zcl.DataType.UINT8);
}

const S1_PRO_ATTRIBUTES = Object.freeze(attributes);
const S1_PRO_CLUSTER = Object.freeze({
    name: CLUSTER_NAME,
    ID: CLUSTER_ID,
    attributes: S1_PRO_ATTRIBUTES,
    commands: {},
    commandsResponse: {},
});

function withHomeAssistant(extend, homeassistant) {
    if (homeassistant) {
        for (const expose of extend.exposes ?? []) expose.withHomeAssistant(homeassistant);
    }
    return extend;
}

function numeric(args, homeassistant) {
    return withHomeAssistant(m.numeric({cluster: CLUSTER_NAME, access: "STATE", ...args}), homeassistant);
}

function binary(args, homeassistant) {
    return withHomeAssistant(m.binary({
        cluster: CLUSTER_NAME,
        access: "STATE",
        // zigbee-herdsman decodes ZCL BOOLEAN values as the numeric bytes 1/0.
        valueOn: [true, 1],
        valueOff: [false, 0],
        ...args,
    }), homeassistant);
}

function commandButton(name, description, label, icon) {
    return binary({
        name,
        attribute: name,
        access: "SET",
        description,
        label,
        entityCategory: "config",
    }, {type: "button", entityCategory: "config", icon});
}

function initialAttributeRead() {
    const commandAttributes = new Set([
        "radar_restart_module",
        "radar_factory_reset",
        "esp32_restart_module",
        "esp32_factory_reset",
        "scd40_forced_calibration",
        "scd40_factory_reset",
    ]);
    const readable = Object.keys(S1_PRO_ATTRIBUTES).filter((name) => !commandAttributes.has(name));
    return {
        isModernExtend: true,
        configure: [async (device) => {
            const endpoint = device.getEndpoint(ENDPOINT);
            if (!endpoint) throw new Error(`S1 Pro endpoint ${ENDPOINT} is missing`);
            for (let offset = 0; offset < readable.length; offset += 8) {
                try {
                    await endpoint.read(CLUSTER_NAME, readable.slice(offset, offset + 8));
                } catch {
                    // Periodic reports populate state if a coordinator rejects an initial read.
                }
            }
        }],
    };
}

const led = m.light({
    color: {modes: ["xy"]},
    configureReporting: false,
    effect: false,
    powerOnBehavior: false,
});
for (const expose of led.exposes ?? []) {
    expose.withLabel("WS2812 LED").withHomeAssistant({
        entityCategory: "config",
        icon: "mdi:led-strip-variant",
    });
}

const extend = [
    m.deviceAddCustomCluster(CLUSTER_NAME, S1_PRO_CLUSTER),
    m.forcePowerSource({powerSource: "Mains (single phase)"}),
    led,
    numeric({
        name: "esp32_temperature",
        attribute: "esp32_temperature",
        description: "ESP32-C6 internal temperature",
        label: "ESP32 Temperature",
        unit: "°C",
        scale: 100,
        precision: 2,
        entityCategory: "diagnostic",
    }, {entityCategory: "diagnostic", icon: "mdi:thermometer"}),
    binary({
        name: "esp32_connected",
        attribute: "esp32_connected",
        description: "ESP32 Zigbee connection status",
        label: "ESP32 Status",
        entityCategory: "diagnostic",
    }, {entityCategory: "diagnostic", icon: "mdi:lan-connect"}),
];

for (let target = 1; target <= 3; target++) {
    for (const axis of ["x", "y"]) {
        extend.push(numeric({
            name: `target_${target}_${axis}`,
            attribute: `target_${target}_${axis}`,
            description: `Target ${target} ${axis.toUpperCase()} coordinate`,
            label: `Target ${target} ${axis.toUpperCase()}`,
            unit: "cm",
            scale: 10,
            precision: 1,
        }, {icon: axis === "x" ? "mdi:alpha-x-box-outline" : "mdi:alpha-y-box-outline"}));
    }
    extend.push(numeric({
        name: `target_${target}_distance`,
        attribute: `target_${target}_distance`,
        description: `Distance to target ${target}`,
        label: `Target ${target} distance`,
        unit: "cm",
        scale: 10,
        precision: 1,
    }, {icon: "mdi:map-marker-distance"}));
    extend.push(numeric({
        name: `target_${target}_speed`,
        attribute: `target_${target}_speed`,
        description: `Speed of target ${target}`,
        label: `Target ${target} speed`,
        unit: "cm/s",
        precision: 0,
    }, {icon: "mdi:speedometer"}));
    extend.push(withHomeAssistant(m.enumLookup({
        name: `target_${target}_state`,
        cluster: CLUSTER_NAME,
        attribute: `target_${target}_state`,
        access: "STATE",
        description: `Holding-engine state of target ${target}`,
        label: `Target ${target} state`,
        lookup: {"No target": 0, Moving: 1, Stationary: 2, Holding: 3},
    }), {icon: "mdi:account-clock"}));
}

extend.push(
    numeric({
        name: "all_targets_count",
        attribute: "all_targets_count",
        description: "Number of detected targets",
        label: "All Targets Count",
        precision: 0,
    }, {icon: "mdi:counter"}),
    binary({
        name: "any_presence",
        attribute: "any_presence",
        description: "Presence detected by any radar target",
        label: "Any Presence",
    }, {icon: "mdi:home-account"}),
    binary({
        name: "any_movement",
        attribute: "any_movement",
        description: "Movement detected by any radar target",
        label: "Any Movement",
    }, {icon: "mdi:motion-sensor"}),
    numeric({
        name: "detection_range",
        attribute: "detection_range",
        access: "ALL",
        description: "Maximum radar detection range",
        label: "Detection Range",
        unit: "cm",
        valueMin: 20,
        valueMax: 600,
        valueStep: 1,
    }, {icon: "mdi:signal-distance-variant"}),
    numeric({
        name: "any_movement_threshold",
        attribute: "any_movement_threshold",
        access: "ALL",
        description: "Speed threshold used for movement detection",
        label: "Any Movement Threshold",
        unit: "cm/s",
        valueMin: 0,
        valueMax: 500,
        valueStep: 1,
    }, {icon: "mdi:motion-sensor"}),
    numeric({
        name: "any_presence_delay",
        attribute: "any_presence_delay",
        access: "ALL",
        description: "Time presence remains active after the last target disappears",
        label: "Any Presence Delay",
        unit: "s",
        valueMin: 0,
        valueMax: 300,
        valueStep: 1,
    }, {icon: "mdi:timer-outline"}),
    binary({
        name: "holding_enabled",
        attribute: "holding_enabled",
        access: "ALL",
        description: "Keep qualified stationary targets active during unexpected radar dropouts",
        label: "Radar Holding Engine",
        entityCategory: "config",
    }, {entityCategory: "config", icon: "mdi:account-clock"}),
    numeric({
        name: "holding_gate_radius",
        attribute: "holding_gate_radius",
        access: "ALL",
        description: "Maximum target displacement before the stationary timer restarts",
        label: "Radar Holding Gate Radius",
        unit: "cm",
        valueMin: 5,
        valueMax: 300,
        valueStep: 1,
    }, {icon: "mdi:crosshairs-gps"}),
    numeric({
        name: "stationary_speed_threshold",
        attribute: "stationary_speed_threshold",
        access: "ALL",
        description: "Maximum speed at which a target is considered stationary",
        label: "Radar Stationary Speed Threshold",
        unit: "cm/s",
        valueMin: 0,
        valueMax: 100,
        valueStep: 1,
    }, {icon: "mdi:speedometer-slow"}),
    numeric({
        name: "stationary_time",
        attribute: "stationary_time",
        access: "ALL",
        description: "Time a target must remain stationary before it can be held",
        label: "Radar Stationary Time",
        unit: "s",
        valueMin: 0,
        valueMax: 120,
        valueStep: 1,
    }, {icon: "mdi:timer-sand"}),
    numeric({
        name: "dropout_hold_time",
        attribute: "dropout_hold_time",
        access: "ALL",
        description: "Maximum time to retain a qualified target after a radar dropout",
        label: "Radar Dropout Hold Time",
        unit: "min",
        valueMin: 0,
        valueMax: 60,
        valueStep: 1,
    }, {icon: "mdi:timer-lock-outline"}),
);

for (const [name, label, description] of [
    ["radar_flip_y_axis", "Radar Flip Y Axis", "Reverse the radar Y axis"],
    ["radar_bluetooth", "Radar Bluetooth", "Enable the LD2450 Bluetooth radio"],
    ["radar_single_target", "Radar Single Target", "Use LD2450 single-target mode"],
]) {
    extend.push(binary({
        name,
        attribute: name,
        access: "ALL",
        description,
        label,
        entityCategory: "config",
    }, {
        entityCategory: "config",
        icon: name === "radar_flip_y_axis" ? "mdi:axis-y-rotate-clockwise" :
            name === "radar_bluetooth" ? "mdi:bluetooth" : "mdi:account",
    }));
}

extend.push(
    commandButton("radar_restart_module", "Restart the LD2450 radar module", "Radar Restart Module", "mdi:restart"),
    commandButton("radar_factory_reset", "Factory-reset the LD2450 radar module", "Radar Factory Reset", "mdi:factory"),
    commandButton("esp32_restart_module", "Restart the complete ESP32-C6 device", "ESP32 Restart Module", "mdi:restart"),
    commandButton("esp32_factory_reset", "Erase settings and the Zigbee network", "ESP32 Factory Reset", "mdi:factory"),

    numeric({name: "bme_temperature", attribute: "bme_temperature", description: "BME688 temperature", label: "BME688 temperature", unit: "°C", scale: 100, precision: 2}, {icon: "mdi:thermometer"}),
    numeric({name: "bme_humidity", attribute: "bme_humidity", description: "BME688 relative humidity", label: "BME688 humidity", unit: "%", scale: 100, precision: 2}, {icon: "mdi:water-percent"}),
    numeric({name: "bme_pressure", attribute: "bme_pressure", description: "BME688 atmospheric pressure", label: "BME688 pressure", unit: "hPa", scale: 100, precision: 2}, {icon: "mdi:gauge"}),
    numeric({name: "bme_gas_resistance", attribute: "bme_gas_resistance", description: "BME688 gas resistance", label: "BME688 gas resistance", unit: "kΩ", scale: 1000, precision: 2}, {icon: "mdi:omega"}),
    numeric({name: "bme_co2_equivalent", attribute: "bme_co2_equivalent", description: "BME688 estimated CO2 equivalent", label: "BME688 CO2 equivalent", unit: "ppm", precision: 0}, {icon: "mdi:molecule-co2"}),
    numeric({name: "bme_voc_equivalent", attribute: "bme_voc_equivalent", description: "BME688 estimated VOC equivalent", label: "BME688 VOC equivalent", unit: "ppm", scale: 100, precision: 2}, {icon: "mdi:molecule"}),
    numeric({name: "bme_iaq", attribute: "bme_iaq", description: "BME688 indoor air-quality index", label: "BME688 IAQ", precision: 0}, {icon: "mdi:gauge"}),
    withHomeAssistant(m.enumLookup({
        name: "bme_bsec_accuracy",
        cluster: CLUSTER_NAME,
        attribute: "bme_bsec_accuracy",
        access: "STATE",
        description: "BME688 BSEC calibration accuracy",
        label: "BME688 IAQ Accuracy",
        lookup: {Stabilizing: 0, "Low accuracy": 1, "Medium accuracy": 2, "High accuracy": 3},
    }), {icon: "mdi:checkbox-marked-circle-outline"}),
    withHomeAssistant(m.enumLookup({
        name: "bme_iaq_classification",
        cluster: CLUSTER_NAME,
        attribute: "bme_iaq_classification",
        access: "STATE",
        description: "BME688 IAQ classification",
        label: "BME688 IAQ Classification",
        lookup: {Excellent: 0, Good: 1, "Lightly polluted": 2, "Moderately polluted": 3, "Heavily polluted": 4, "Severely polluted": 5, "Extremely polluted": 6},
    }), {icon: "mdi:air-filter"}),
    numeric({name: "bme_temperature_offset", attribute: "bme_temperature_offset", access: "ALL", description: "BME688 temperature correction", label: "BME688 Temp Offset", unit: "°C", scale: 100, precision: 2, valueMin: 0, valueMax: 30, valueStep: 0.01}, {icon: "mdi:thermometer-plus"}),

    numeric({name: "scd40_co2", attribute: "scd40_co2", description: "SCD40 measured CO2 concentration", label: "SCD40 CO2", unit: "ppm", precision: 0}, {icon: "mdi:molecule-co2"}),
    numeric({name: "scd40_temperature", attribute: "scd40_temperature", description: "SCD40 temperature", label: "SCD40 temperature", unit: "°C", scale: 100, precision: 2}, {icon: "mdi:thermometer"}),
    numeric({name: "scd40_humidity", attribute: "scd40_humidity", description: "SCD40 relative humidity", label: "SCD40 humidity", unit: "%", scale: 100, precision: 2}, {icon: "mdi:water-percent"}),
    numeric({name: "scd40_calibration_reference", attribute: "scd40_calibration_reference", access: "ALL", description: "CO2 reference value for forced calibration", label: "SCD40 Calibration Reference", unit: "ppm", valueMin: 400, valueMax: 2000, valueStep: 1}, {icon: "mdi:tune"}),
    commandButton("scd40_forced_calibration", "Run SCD40 forced calibration", "SCD40 Forced Calibration", "mdi:tune"),
    commandButton("scd40_factory_reset", "Factory-reset the SCD40", "SCD40 Factory Reset", "mdi:factory"),
    numeric({name: "scd40_temperature_offset", attribute: "scd40_temperature_offset", access: "ALL", description: "SCD40 temperature correction", label: "SCD40 Temp Offset", unit: "°C", scale: 100, precision: 2, valueMin: 0, valueMax: 30, valueStep: 0.01}, {icon: "mdi:thermometer-plus"}),

    numeric({name: "ltrf216a_illuminance", attribute: "ltrf216a_illuminance", description: "LTR-F216A ambient illuminance", label: "LTR-F216A Illuminance", unit: "lx", scale: 100, precision: 2}, {icon: "mdi:brightness-5"}),
    binary({name: "buzzer_power", attribute: "buzzer_power", access: "ALL", description: "MLT8530 buzzer power", label: "MLT8530 Buzzer", entityCategory: "config"}, {entityCategory: "config", icon: "mdi:surround-sound"}),
);

extend.push(numeric({
    name: "exclusion_zone_points_count",
    attribute: "exclusion_zone_points_count",
    access: "STATE_SET",
    description: "Number of active exclusion-zone polygon points",
    label: "Exclusion Zone Points Count",
    unit: "pts",
    valueMin: 0,
    valueMax: 8,
    valueStep: 1,
}, {icon: "mdi:counter"}));

for (let point = 1; point <= 8; point++) {
    for (const axis of ["x", "y"]) {
        const name = `exclusion_zone_p${point}_${axis}`;
        extend.push(numeric({
            name,
            attribute: name,
            access: "STATE_SET",
            description: `Exclusion-zone point ${point} ${axis.toUpperCase()} coordinate`,
            label: `Exclusion Zone P${point} ${axis.toUpperCase()}`,
            unit: "cm",
            valueMin: -600,
            valueMax: 600,
            valueStep: 1,
            precision: 0,
        }, {icon: "mdi:map-marker"}));
    }
}

for (let zone = 1; zone <= 3; zone++) {
    for (let point = 1; point <= 8; point++) {
        for (const axis of ["x", "y"]) {
            const name = `zone_${zone}_p${point}_${axis}`;
            extend.push(numeric({
                name,
                attribute: name,
                access: "STATE_SET",
                description: `Zone ${zone} point ${point} ${axis.toUpperCase()} coordinate`,
                label: `Zone ${zone} P${point} ${axis.toUpperCase()}`,
                unit: "cm",
                valueMin: -600,
                valueMax: 600,
                valueStep: 1,
                precision: 0,
            }, {icon: "mdi:map-marker"}));
        }
    }
    extend.push(
        numeric({name: `zone_${zone}_points_count`, attribute: `zone_${zone}_points_count`, access: "STATE_SET", description: `Number of active polygon points in zone ${zone}`, label: `Zone ${zone} Points Count`, unit: "pts", valueMin: 0, valueMax: 8, valueStep: 1}, {icon: "mdi:counter"}),
        numeric({name: `zone_${zone}_movement_threshold`, attribute: `zone_${zone}_movement_threshold`, access: "STATE_SET", description: `Movement threshold for zone ${zone}`, label: `Zone ${zone} Movement Threshold`, unit: "cm/s", valueMin: 0, valueMax: 500, valueStep: 1}, {icon: "mdi:motion-sensor"}),
        numeric({name: `zone_${zone}_presence_delay`, attribute: `zone_${zone}_presence_delay`, access: "STATE_SET", description: `Presence delay for zone ${zone}`, label: `Zone ${zone} Presence Delay`, unit: "s", valueMin: 0, valueMax: 300, valueStep: 1}, {icon: "mdi:timer-outline"}),
        binary({name: `zone_${zone}_presence`, attribute: `zone_${zone}_presence`, description: `Presence detected in zone ${zone}`, label: `Zone ${zone} Presence`}, {icon: "mdi:home-account"}),
        binary({name: `zone_${zone}_movement`, attribute: `zone_${zone}_movement`, description: `Movement detected in zone ${zone}`, label: `Zone ${zone} Movement`}, {icon: "mdi:motion-sensor"}),
        numeric({name: `zone_${zone}_target_count`, attribute: `zone_${zone}_target_count`, description: `Number of targets in zone ${zone}`, label: `Zone ${zone} Target Count`, precision: 0}, {icon: "mdi:counter"}),
    );
}

extend.push(initialAttributeRead());

function overrideHaDiscoveryPayload(payload, options) {
    if (payload.device) {
        const identifier = String(options?.ID ?? payload.device.identifiers?.[0] ?? "");
        const ieeeSuffix = identifier.match(/([0-9a-f]{6})$/i)?.[1]?.toLowerCase();
        payload.device.name = ieeeSuffix ? `S1 Pro Multi Sense ${ieeeSuffix}` : "S1 Pro Multi Sense";

        // The short suffix already distinguishes identical units in Home Assistant.
        // An explicit empty value also clears a model ID cached by the device registry.
        payload.device.model_id = "";
    }

    const entityId = payload.default_entity_id;
    if (typeof entityId !== "string") return;

    if (entityId.endsWith("_linkquality")) {
        payload.enabled_by_default = true;
        payload.name = "Link Quality";
        payload.entity_category = "diagnostic";
        payload.icon = "mdi:signal";
    }

    if (entityId.startsWith("light.")) {
        payload.name = "WS2812 LED";
        payload.entity_category = "config";
        const currentObjectId = entityId.slice("light.".length);
        if (!currentObjectId.endsWith("_ws2812_led")) {
            payload.default_entity_id = `light.${currentObjectId}_ws2812_led`;
            payload.object_id = `${currentObjectId}_ws2812_led`;
        }
    }

    if (entityId.endsWith("_ltrf216a_illuminance")) {
        payload.device_class = "illuminance";
        payload.state_class = "measurement";
    }

    // Match the ZHA quirk: editable numeric settings use text input boxes.
    if (entityId.startsWith("number.")) payload.mode = "box";

    // Polygon coordinates are stored by the firmware as signed whole centimetres.
    const coordinate = entityId.match(/((?:exclusion_zone|zone_[1-3])_p[1-8]_[xy])$/)?.[1];
    if (coordinate) {
        payload.value_template = `{{ (value_json["${coordinate}"] | round(0) | int) if value_json["${coordinate}"] is number else none }}`;
    }
}

const definition = {
    zigbeeModel: ["S1 Pro Multi Sense (Zigbee)", "S1-Pro-C6-LD2450"],
    fingerprint: [{manufacturerName: "Sensy-One", endpoints: [{ID: ENDPOINT}]}],
    model: "S1-Pro-ZB",
    vendor: "Sensy-One",
    description: "S1 Pro Multi Sense",
    meta: {overrideHaDiscoveryPayload},
    extend,
};

export default definition;
