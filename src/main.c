#include <zephyr/bluetooth/bluetooth.h>
#include <bluetooth/mesh/models.h>
#include <bluetooth/mesh/dk_prov.h>
#include <dk_buttons_and_leds.h>
#include "model_handler.h"

// Callback function that is called when Bluetooth is ready
static void bt_ready(int err)
{
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    // Initialize LEDs and buttons on the board
    dk_leds_init();
    dk_buttons_init(NULL);

    // Initialize Bluetooth Mesh with provisioning and model handler
    err = bt_mesh_init(bt_mesh_dk_prov_init(), model_handler_init());
    if (err) {
        printk("Initializing mesh failed (err %d)\n", err);
        return;
    }

    // Load stored settings if available
    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }

    // Enable provisioning over advertising and GATT
    bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);

    printk("Mesh initialized\n");
}

// Main function that starts the Bluetooth initialization process
int main(void)
{
    int err;

    printk("Initializing...\n");

    // Enable Bluetooth and provide a callback when ready
    err = bt_enable(bt_ready);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
    }

    return 0;
}
