#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/drivers/sensor.h>
#include <bluetooth/mesh/models.h>
#include <bluetooth/mesh/sensor_types.h>
#include <dk_buttons_and_leds.h>
#include "model_handler.h"

// Conditional compilation to choose the correct sensor node
#if DT_NODE_HAS_STATUS(DT_NODELABEL(bme680), okay)
/** Thingy53 */
#define SENSOR_NODE DT_NODELABEL(bme680)
#define SENSOR_DATA_TYPE SENSOR_CHAN_AMBIENT_TEMP
#elif DT_NODE_HAS_STATUS(DT_NODELABEL(temp), okay)
/** nRF52 DK */
#define SENSOR_NODE DT_NODELABEL(temp)
#define SENSOR_DATA_TYPE SENSOR_CHAN_DIE_TEMP
#else
#error "Unsupported board!"
#endif

// Macro to initialize temperature sensor value in the correct format
#define TEMP_INIT(_val) { .format = &bt_mesh_sensor_format_temp, .raw = { \
    FIELD_GET(GENMASK(7, 0), (_val) * 100),                                \
    FIELD_GET(GENMASK(15, 8), (_val) * 100)                                \
}}

// Retrieve the device object for the selected sensor node
static const struct device *dev = DEVICE_DT_GET(SENSOR_NODE);

// Function to fetch and return the temperature data from the sensor
static int chip_temp_get(struct bt_mesh_sensor_srv *srv,
                         struct bt_mesh_sensor *sensor,
                         struct bt_mesh_msg_ctx *ctx,
                         struct bt_mesh_sensor_value *rsp)
{
    struct sensor_value channel_val;
    int err;

    // Fetch data from the sensor
    sensor_sample_fetch(dev);

    // Get the temperature data from the sensor's channel
    err = sensor_channel_get(dev, SENSOR_DATA_TYPE, &channel_val);
    if (err) {
        printk("Error getting temperature sensor data (%d)\n", err);
    }

    // Convert the sensor value to a mesh-compatible format
    err = bt_mesh_sensor_value_from_sensor_value(
        sensor->type->channels[0].format, &channel_val, rsp);
    if (err) {
        printk("Error encoding temperature sensor data (%d)\n", err);
    }

    return err;
}

// Descriptor for the temperature sensor, specifying its tolerance and sampling type
static const struct bt_mesh_sensor_descriptor chip_temp_descriptor = {
    .tolerance = {
        .negative = BT_MESH_SENSOR_TOLERANCE_ENCODE(4),
        .positive = BT_MESH_SENSOR_TOLERANCE_ENCODE(4),
    },
    .sampling_type = BT_MESH_SENSOR_SAMPLING_INSTANTANEOUS,
};

// Define the temperature sensor with its type and getter function
static struct bt_mesh_sensor chip_temp = {
    .type = &bt_mesh_sensor_present_dev_op_temp,
    .get = chip_temp_get,
    .descriptor = &chip_temp_descriptor,
};

// Array of sensor pointers for the sensor server
static struct bt_mesh_sensor *const chip_temp_sensor[] = {
    &chip_temp,
};

// Initialize the sensor server with the temperature sensor
static struct bt_mesh_sensor_srv chip_temp_sensor_srv =
    BT_MESH_SENSOR_SRV_INIT(chip_temp_sensor, ARRAY_SIZE(chip_temp_sensor));

// Observer client function to handle received sensor data
static void sensor_cli_data_cb(struct bt_mesh_sensor_cli *cli,
                               struct bt_mesh_msg_ctx *ctx,
                               const struct bt_mesh_sensor_type *sensor,
                               const struct bt_mesh_sensor_value *value)
{
    if (sensor->id == bt_mesh_sensor_present_dev_op_temp.id) {
        printk("Received chip temperature from device %d: %s\n", ctx->addr, bt_mesh_sensor_ch_str(value));
    }
}

// Handlers for the sensor client
static const struct bt_mesh_sensor_cli_handlers bt_mesh_sensor_cli_handlers = {
    .data = sensor_cli_data_cb,
};

// Initialize the sensor client
static struct bt_mesh_sensor_cli sensor_cli =
    BT_MESH_SENSOR_CLI_INIT(&bt_mesh_sensor_cli_handlers);

static struct k_work_delayable get_data_work;

// Function to retrieve sensor data periodically
static void get_data(struct k_work *work)
{
    if (!bt_mesh_is_provisioned()) {
        k_work_schedule(&get_data_work, K_MSEC(GET_DATA_INTERVAL));
        return;
    }

    // Request temperature data from the sensor
    int err = bt_mesh_sensor_cli_get(
        &sensor_cli, NULL, &bt_mesh_sensor_present_dev_op_temp,
        NULL);
    if (err) {
        printk("Error getting chip temperature (%d)\n", err);
    }

    // Schedule the next data retrieval
    k_work_schedule(&get_data_work, K_MSEC(GET_DATA_INTERVAL));
}

// Button handler callback to manually trigger temperature data retrieval
static void button_handler_cb(uint32_t pressed, uint32_t changed)
{
    if (!bt_mesh_is_provisioned()) {
        return;
    }

    if (pressed & changed & BIT(0)) {
        int err = bt_mesh_sensor_cli_get(&sensor_cli, NULL,
                                         &bt_mesh_sensor_present_dev_op_temp, NULL);
        if (err) {
            printk("Error getting chip temperature (%d)\n", err);
        }
    }
}

// Initialize the button handler with the callback function
static struct button_handler button_handler = {
    .cb = button_handler_cb,
};

// Variables and functions for handling attention state and LED blinking
static struct k_work_delayable attention_blink_work;
static bool attention;

// Function to handle LED blinking when attention is requested
static void attention_blink(struct k_work *work)
{
    static int idx;
    const uint8_t pattern[] = {
    BIT(0) | BIT(1),  // LEDs 0 and 1 are turned on
    BIT(1) | BIT(2),  // LEDs 1 and 2 are turned on
    BIT(2) | BIT(3),  // LEDs 2 and 3 are turned on
    BIT(3) | BIT(0),  // LEDs 3 and 0 are turned on
};
// Array defining different LED patterns to create a rotating blink effect

if (attention) {  
    // Check if the attention mode is active
    dk_set_leds(pattern[idx++ % ARRAY_SIZE(pattern)]);
    // Set the LEDs to the current pattern, then increment the index to move to the next pattern
    // The modulo operation ensures the index wraps around when it reaches the end of the array
    k_work_reschedule(&attention_blink_work, K_MSEC(30));
    // Reschedule the work to run again after 30 milliseconds to create a continuous blinking effect
} else {  
    // If attention mode is not active
    dk_set_leds(DK_NO_LEDS_MSK);
    // Turn off all LEDs
}
}

// Function to start attention blinking
static void attention_on(const struct bt_mesh_model *mod)
{
    attention = true;
    k_work_reschedule(&attention_blink_work, K_NO_WAIT);
}

// Function to stop attention blinking
static void attention_off(const struct bt_mesh_model *mod)
{
    attention = false;
}

// Callbacks for the health server to handle attention state
static const struct bt_mesh_health_srv_cb health_srv_cb = {
    .attn_on = attention_on,
    .attn_off = attention_off,
};

// Initialize the health server with the callbacks
static struct bt_mesh_health_srv health_srv = {
    .cb = &health_srv_cb,
};

// Define the elements and models for the mesh node
static struct bt_mesh_elem elements[] = {
    BT_MESH_ELEM(1,
                 BT_MESH_MODEL_LIST(BT_MESH_MODEL_CFG_SRV,
                                    BT_MESH_MODEL_HEALTH_SRV(&health_srv,
                                                             &health_pub),
                                    BT_MESH_MODEL_SENSOR_CLI(&sensor_cli),
                                    BT_MESH_MODEL_SENSOR_SRV(&chip_temp_sensor_srv)),
                 BT_MESH_MODEL_NONE),
};

// Define the mesh composition data
static const struct bt_mesh_comp comp = {
    .cid = CONFIG_BT_COMPANY_ID,
    .elem = elements,
    .elem_count = ARRAY_SIZE(elements),
};

// Function to initialize the model handler
const struct bt_mesh_comp *model_handler_init(void)
{
    // Initialize delayed work for attention blinking and data retrieval
    k_work_init_delayable(&attention_blink_work, attention_blink);
    k_work_init_delayable(&get_data_work, get_data);

    // Check if the sensor device is ready
    if (!device_is_ready(dev)) {
        printk("Temperature sensor not ready\n");
    } else {
        printk("Temperature sensor (%s) initiated\n", dev->name);
    }

    // Add the button handler
    dk_button_handler_add(&button_handler);
    // Schedule the first data retrieval
    k_work_schedule(&get_data_work, K_MSEC(GET_DATA_INTERVAL));

    return &comp;
}
