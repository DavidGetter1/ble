/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <sys/printk.h>
#include <sys/util.h>
#define LOG_MODULE_NAME ext_log_system
#include <logging/log.h>

LOG_MODULE_REGISTER(ext_log_system);
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <stdio.h>
#define SERVICE_DATA_LEN 10
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define RECEIVED_DATA_LEN 16
#define MAX_NODES 10

uint8_t parent_bt_addr[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

uint8_t own_bt_addr[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

uint8_t elements_in_buffer = 0;

bool is_valid_data(uint8_t data[RECEIVED_DATA_LEN])
{
    uint8_t bad_data[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    return memcmp(&data[8], bad_data, 6) != 0;
}
// TODO wir kriegen keine nachrichten von peripherals

// buffer ads, each ad gets x seconds for advertising
uint8_t buffer_ads[MAX_NODES][RECEIVED_DATA_LEN] = {
    {0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x7c}, // Das datenformat sieht folgendermaßen aus: 0x7c sind die guards die angeben wann unsere nachricht anfängt und endet
    {0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x7c}, // an stelle 1 steht unsere generierte random number
    {0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x7c}, // an stelle 2-7 steht die bt_addresse unseres parents
    {0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x7c}, // stelle 8-13 steht die bt_addresse von der die nachricht stammt
    {0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x7c}, // an stelle 14 steht unsere ident nummber 0x01 für central, 0x02 für peripheral
    {0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x7c},
    {0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x7c},
    {0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x7c},
    {0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x7c},
};

uint8_t *buffer_runner_ptr = buffer_ads[0];

uint8_t *buffer_ads_ptr = buffer_ads[0];

void run_buffer()
{
    static bool is_running = false;
    if (is_running)
    {
        return;
    }
    is_running = true;
    bool while_is_running = true;
    while (while_is_running)
    {
        LOG_INF("%s", "Starting Buffer\n");

        const struct bt_data ad[] = {
            BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
            BT_DATA(BT_DATA_SVC_DATA16, buffer_runner_ptr, RECEIVED_DATA_LEN),
        };

        LOG_INF("The parent bt_addr in buffer is %02x:%02x:%02x:%02x:%02x:%02x\n", buffer_ads[0][2], buffer_ads[0][3], buffer_ads[0][4], buffer_ads[0][5], buffer_ads[0][6], buffer_ads[0][7]);
        LOG_INF("The first bt_addr in buffer is %02x:%02x:%02x:%02x:%02x:%02x\n", buffer_ads[0][8], buffer_ads[0][9], buffer_ads[0][10], buffer_ads[0][11], buffer_ads[0][12], buffer_ads[0][13]);
        LOG_INF("The second bt_addr in buffer is %02x:%02x:%02x:%02x:%02x:%02x\n", buffer_ads[1][8], buffer_ads[1][9], buffer_ads[1][10], buffer_ads[1][11], buffer_ads[1][12], buffer_ads[1][13]);
        LOG_INF("This is the whole message im sending: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x:",
                buffer_ads[0][0], buffer_ads[0][1], buffer_ads[0][2], buffer_ads[0][3], buffer_ads[0][4], buffer_ads[0][5], buffer_ads[0][6], buffer_ads[0][7], buffer_ads[0][8], buffer_ads[0][9],
                buffer_ads[0][10], buffer_ads[0][11], buffer_ads[0][12], buffer_ads[0][13], buffer_ads[0][14]);
        LOG_INF("%02x\n", buffer_ads[0][15]);

        int err = bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY, ad, ARRAY_SIZE(ad), NULL, 0);
        if (err)
        {
            LOG_ERR("Advertising failed to start (err %d)\n", err);
            return;
        }
        LOG_INF("%s", "Advertising successfully started\n");
        k_sleep(K_SECONDS(1.5));
        err = bt_le_adv_stop();
        if (err)
        {
            LOG_ERR("Advertising failed to stop (err %d)\n", err);
            return;
        }

        // check if next data is valid, data is valid if it has a random number and a bt_address
        int is_valid = is_valid_data((buffer_runner_ptr + 16));
        if (is_valid)
        {
            buffer_runner_ptr += 16;
        }
        else
        {
            elements_in_buffer = 0;
            while_is_running = false;
            is_running = false;
            LOG_INF("%s", "stopping Buffer\n");
        }
    }
    // after fiunishing buffer, reset buffer
    for (int i = 0; i < MAX_NODES; i++)
    {
        buffer_ads[i][1] = 0x00; // reset random number
        buffer_ads[i][8] = 0x00; // reset addressed bt_address
        buffer_ads[i][9] = 0x00;
        buffer_ads[i][10] = 0x00;
        buffer_ads[i][11] = 0x00;
        buffer_ads[i][12] = 0x00;
        buffer_ads[i][13] = 0x00;
    }
    buffer_runner_ptr = buffer_ads[0];
    is_running = false;
}

bool is_data_in_buffer(uint8_t data[RECEIVED_DATA_LEN - 2])
{
    // wrap data in 0x7c characters, because of some poor design choices
    uint8_t new_data[RECEIVED_DATA_LEN] = {0x7c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x7c};
    memcpy(new_data + 1, data, RECEIVED_DATA_LEN - 2);
    int i = 0, j = 0;
    for (i = 0; i < MAX_NODES; i++)
    {
        int match = 1;
        for (j = 0; j < RECEIVED_DATA_LEN; j++)
        {
            if (data[j] != buffer_ads[i][j])
            {
                match = 0;
                break;
            }
        }
        if (match)
        {
            return true;
        }
    }
    return false;
}

bool is_initialized(uint8_t *arr, size_t size)
{
    for (size_t i = 0; i < size; i++)
    {
        if (arr[i] != 0)
        {
            return true;
        }
    }
    return false;
}

uint8_t distance_counter = 0xFF;

static void bt_ready(int err)
{
    char addr_s[BT_ADDR_LE_STR_LEN] = {0};
    bt_addr_le_t addr = {0};
    size_t count = 1;

    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    bt_id_get(&addr, &count);
    // Hier sollte die bt_addr an den richtigen Ort kopiert werden
    memcpy(own_bt_addr, addr.a.val, 6);
    LOG_INF("The own bt_addr is %02x:%02x:%02x:%02x:%02x:%02x\n", own_bt_addr[0], own_bt_addr[1], own_bt_addr[2], own_bt_addr[3], own_bt_addr[4], own_bt_addr[5]);

    bt_addr_le_to_str(&addr, addr_s, sizeof(addr_s));

    printk("Beacon started, advertising as %s\n", addr_s);
}

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
                    struct net_buf_simple *buf)
{
    LOG_INF("%s", "mpu hunter vor inits");
    // Upon receiving, save in array, increment pointers
    uint16_t data_len = net_buf_simple_max_len(buf);
    uint8_t data = 0;
    uint8_t *data_ptr = &data;
    uint8_t received_data[RECEIVED_DATA_LEN - 2] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint8_t *received_data_ptr = received_data;
    bool put_in_received_data = false;
    LOG_INF("%s", "mpu hunter nach inits");
    for (uint8_t i = 0; i < data_len; i++)
    {
        printk("%02x ", *data_ptr);

        if (*data_ptr == 0x7c)
        {
            put_in_received_data = !put_in_received_data;
        }
        data_ptr = net_buf_simple_pull(buf, 1);
        if (put_in_received_data && *data_ptr != 0x7c)
        {
            *(received_data_ptr++) = *data_ptr;
        }
    }
    printk("\n");
    if (received_data[SERVICE_DATA_LEN - 3] == 0x01)
    {
        // Hier wurde eine nachricht vom central node empfangen, es handelt sich entweder um die nachricht "Baue Baum" oder "Runde geht los"
        LOG_INF("%s", "Received message from central node");
        LOG_INF("The parent_bt_addr is initialized: %d", is_initialized(parent_bt_addr, sizeof(parent_bt_addr)));
        if (received_data[0] < distance_counter && !is_initialized(parent_bt_addr, sizeof(parent_bt_addr)))
        {
            // Hier sind wir im Falle, dass wir den Baum bauen sollen, wir leiten die Nachricht die wir bekommen haben weiter
            // initialize parent_bt_addr
            memcpy(parent_bt_addr, &received_data[1], sizeof(parent_bt_addr));
            // increment distance counter
            // update advertisement data
            distance_counter = received_data[0];
            uint8_t baum_ad_data[SERVICE_DATA_LEN] = {
                0x7c,
                distance_counter + 1, // distance_counter also acts as round_counter if higher than 0
                own_bt_addr[0],       // bt_addr 1
                own_bt_addr[1],       // bt_addr 2
                own_bt_addr[2],       // bt_addr 3
                own_bt_addr[3],       // bt_addr 4
                own_bt_addr[4],       // bt_addr 5
                own_bt_addr[5],       // bt_addr 6
                0x01,                 // ident_number 0x01 = central, 0x02 = peripheral
                0x7c};

            const struct bt_data ad_baum[] = {
                BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
                BT_DATA(BT_DATA_SVC_DATA16, baum_ad_data, ARRAY_SIZE(baum_ad_data)),
            };

            // start advertising
            bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY, ad_baum, ARRAY_SIZE(ad_baum), NULL, 0);
            // sleep
            k_sleep(K_SECONDS(1));
            // stop advertising
            LOG_INF("%s", "Stopping the 0x01 advertisement");
            int err = bt_le_adv_stop();
            if (err)
            {
                LOG_ERR("Advertising failed to stop (err %d)\n", err);
                return;
            }
            // hier geht die runde los, wir erstellen ein random value und schicken es
            // get random number
            uint8_t random_int[1] = {0};
            bt_rand(random_int, 1);
            LOG_INF("OUR RANDOM NUMBER IS: %d", random_int[0]);
            // wir packen unser ad in den buffer
            *(buffer_ads_ptr + 1) = random_int[0];

            memcpy(buffer_ads_ptr + 2, parent_bt_addr, sizeof(parent_bt_addr));
            memcpy(buffer_ads_ptr + 8, own_bt_addr, sizeof(own_bt_addr));
            buffer_ads_ptr += 16;

            elements_in_buffer++;
        }
    }
    else
    {
        // TODO nachricht stirbt nicht ab, sollte aber, denn wir addressieren grundsätzlich nur an parents, also unidirektional
        // TODO im else teil noch die nachricht richtig in den buffer pasten
        // TODO es läuft sehr nichtdeterministisch, je nachdem ob central nachricht weitergeleitet wird oder ob peripheral erstmal die runde schickt
        LOG_INF("%s", "Message from peripheral");
        // Hier wurde eine nachricht vom peripheral node empfangen
        // Wenn die Nachricht an uns addressiert wird, dann leiten wir sie weiter
        // received_data[1] ist die bt_addr des nodes, an den die Nachricht adressiert ist
        // received_data[2] ist unsere Addresse
        LOG_INF("The received bt_addr is %02x:%02x:%02x:%02x:%02x:%02x\n", received_data[1], received_data[2], received_data[3], received_data[4], received_data[5], received_data[6]);
        LOG_INF("Our bt_addr is %02x:%02x:%02x:%02x:%02x:%02x\n", own_bt_addr[0], own_bt_addr[1], own_bt_addr[2], own_bt_addr[3], own_bt_addr[4], own_bt_addr[5]);
        // here we check if the advertisement is for us
        if (memcmp(&received_data[1], own_bt_addr, 6) == 0)
        {
            if (is_data_in_buffer(received_data))
            {
                LOG_INF("%s", "already in buffer");
            }
            else
            {
                // TODO this should be wrong as im not targeting my parent
                LOG_INF("%s", "adding to buffer");
                memcpy(buffer_ads_ptr + 1, received_data, RECEIVED_DATA_LEN - 2);
                LOG_INF("%s", "after potential mpu");
                buffer_ads_ptr += 16;
                LOG_INF("%s", "after potential mpu 1");
                elements_in_buffer++;
                LOG_INF("%s", "after potential mpu 2");
            }
        }
    }
}

int main(void)
{

    struct bt_le_scan_param scan_param = {
        .type = BT_HCI_LE_SCAN_PASSIVE,
        .options = BT_LE_SCAN_OPT_NONE,
        .interval = BT_GAP_ADV_FAST_INT_MIN_2,
        .window = BT_GAP_ADV_FAST_INT_MIN_2,
    };
    int err;

    err = bt_enable(bt_ready);
    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
        return 0;
    }

    /* Initialize the Bluetooth Subsystem */

    err = bt_le_scan_start(&scan_param, scan_cb);
    if (err)
    {
        printk("Starting scanning failed (err %d)\n", err);
        return 0;
    }

    while (1)
    {
        k_sleep(K_SECONDS(0.6));
        // check if elements are in buffer
        if (elements_in_buffer > 0)
        {
            run_buffer();
        }
    }

    printk("Starting Scanner/Advertiser Demo\n");

    return 0;
}