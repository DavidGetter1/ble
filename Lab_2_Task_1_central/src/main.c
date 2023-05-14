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
#define SERVICE_DATA_LEN 11
#define RECEIVED_DATA_LEN 16
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

#define MAX_NODES 2

static uint8_t service_data[SERVICE_DATA_LEN] = {
    0x7c,
    0x00, // round_counter
    0x01, // distance_counter
    0x00, // bt_addr 1
    0x00, // bt_addr 2
    0x00, // bt_addr 3
    0x00, // bt_addr 4
    0x00, // bt_addr 5
    0x00, // bt_addr 6
    0x01, // ident_number 0x01 = central, 0x02 = peripheral
    0x7c};

struct node_data
{
    uint8_t bt_addr[6];
    uint8_t rand_node_number;
};

struct node_data nodes[MAX_NODES] = {
    {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 0},
    {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, 0}};

uint8_t *node_data_bt_addr_ptr = &nodes[0].bt_addr[0];

uint8_t *node_data_rand_node_number_ptr = &nodes[0].rand_node_number;

void print_highest_rand_node_num(void)
{
    uint8_t received_data_count = 0;
    uint8_t highest = 0;
    uint8_t empty_bt_addr = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t bt_addr[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t *bt_addr_ptr = &bt_addr[0];
    for (int i = 0; i < MAX_NODES; i++)
    {
        if (memcmp(nodes[i].bt_addr, empty_bt_addr, 6) != 0)
        {
            LOG_INF("The corresponding bt_addr is %02x:%02x:%02x:%02x:%02x:%02x\n", nodes[i].bt_addr[0], nodes[i].bt_addr[1], nodes[i].bt_addr[2], nodes[i].bt_addr[3], nodes[i].bt_addr[4], nodes[i].bt_addr[5]);

            received_data_count++;
        }
        LOG_INF("The rand_node_number of node %d is %d\n", i, nodes[i].rand_node_number);
        if (nodes[i].rand_node_number > highest)
        {
            highest = nodes[i].rand_node_number;
            memcpy(bt_addr, nodes[i].bt_addr, 6);
        }
    }
    LOG_INF("The round count is %d\n", service_data[1]);
    LOG_INF("I received data from %d nodes", received_data_count);
    LOG_INF("The highest rand_node_number is %d\n", highest);
    LOG_INF("The corresponding bt_addr is %02x:%02x:%02x:%02x:%02x:%02x\n", bt_addr[0], bt_addr[1], bt_addr[2], bt_addr[3], bt_addr[4], bt_addr[5]);
}

bool is_address_in_array(struct node_data *nodes, int num_nodes, uint8_t *addr)
{
    for (int i = 0; i < num_nodes; i++)
    {
        if (memcmp(nodes[i].bt_addr, addr, 6) == 0)
        {
            return true;
        }
    }
    return false;
}

uint8_t *service_data_ptr = &service_data[3];

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
    BT_DATA(BT_DATA_SVC_DATA16, service_data, ARRAY_SIZE(service_data)),
};

static void bt_ready(int err)
{
    char addr_s[BT_ADDR_LE_STR_LEN];
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
    memcpy(service_data_ptr, addr.a.val, 6);
    bt_addr_le_to_str(&addr, addr_s, sizeof(addr_s));
    LOG_INF("The own bt_addr is %02x:%02x:%02x:%02x:%02x:%02x\n", *service_data_ptr, *(service_data_ptr + 1), *(service_data_ptr + 2), *(service_data_ptr + 3), *(service_data_ptr + 4), *(service_data_ptr + 5));
    printk("Beacon started, advertising as %s\n", addr_s);
}

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
                    struct net_buf_simple *buf)
{
    // Upon receiving, save in array, increment pointers
    uint16_t data_len = net_buf_simple_max_len(buf);
    uint8_t *data_ptr = 0;
    uint8_t received_data[RECEIVED_DATA_LEN - 2];
    uint8_t *received_data_ptr = &received_data[0];
    bool put_in_received_data = false;

    for (uint8_t i = 0; i < data_len; i++)
    {

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
    if (received_data[SERVICE_DATA_LEN - 3] == 0x01)
    {
        //   LOG_INF("%s", "This is a central");
    }
    else if (received_data[RECEIVED_DATA_LEN - 3] == 0x02)
    {
        //   LOG_INF("%s", "This is a Peripheral");
        //   LOG_INF("its bt_addr is %02x:%02x:%02x:%02x:%02x:%02x\n", received_data[7], received_data[8], received_data[9], received_data[10], received_data[11], received_data[12]);

        // ab der 7 dürfte die bt_addr des absenders stehen
        if (is_address_in_array(nodes, MAX_NODES, &received_data[7]))
        {
            //  LOG_INF("The corresponding bt_addr is %02x:%02x:%02x:%02x:%02x:%02x\n", received_data[7], received_data[8], received_data[9], received_data[10], received_data[11], received_data[12]);
            //
            //  LOG_INF("%s", "Address already in array");
        }
        else
        {

            // copy address to array
            memcpy(node_data_bt_addr_ptr, &received_data[7], 6);
            //    LOG_INF("THIS IS THE RECEIVED DATA: %02x:%02x:%02x:%02x:%02x:%02x \n", received_data[7], received_data[8], received_data[9], received_data[10], received_data[11], received_data[12]);
            //    LOG_INF("THIS IS THE DATA THAT GOT COPIED:  %02x:%02x:%02x:%02x:%02x:%02x\n", node_data_bt_addr_ptr[0], node_data_bt_addr_ptr[1], node_data_bt_addr_ptr[2], node_data_bt_addr_ptr[3], node_data_bt_addr_ptr[4], node_data_bt_addr_ptr[5]);

            // copy received data to array
            memcpy(node_data_rand_node_number_ptr, &received_data[0], 1);

            // increment pointers
            node_data_bt_addr_ptr += 7;
            node_data_rand_node_number_ptr += 7;
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
    /* Initialize the Bluetooth Subsystem */
    err = bt_enable(bt_ready);
    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
        return 0;
    }

    /// Build a Tree
    LOG_INF("%s", "Building Tree");

    err = bt_le_scan_start(&scan_param, scan_cb);
    if (err)
    {
        printk("Starting scanning failed (err %d)\n", err);
        return 0;
    }
    /// start first rount
    LOG_INF("%s", "Starting first round");
    // Advertise distance_counter which is 0 initially
    err = bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY,
                          ad, ARRAY_SIZE(ad),
                          NULL, 0);
    if (err)
    {
        printk("Advertising failed to start (err %d)\n", err);
        return 0;
    }

    // wait for the tree to be built
    // This is a test value as i dont know how long the tree building takes
    k_sleep(K_MSEC(1000));
    bt_le_adv_stop();
    k_sleep(K_MSEC(4000));

    while (1)
    {
        // TODO memset problem idk
        memset(nodes, 0x00, sizeof(nodes));
        for (int i = 0; i < MAX_NODES; i++)
        {
        
            LOG_INF("after memsetting to 0 the addr at %d is %02x:%02x:%02x:%02x:%02x:%02x\n", i, nodes[i].bt_addr[0], nodes[i].bt_addr[1], nodes[i].bt_addr[2], nodes[i].bt_addr[3], nodes[i].bt_addr[4], nodes[i].bt_addr[5]);
        }

        node_data_bt_addr_ptr = &nodes[0].bt_addr;
        node_data_rand_node_number_ptr = &nodes[0].rand_node_number;
        // update round_counter
        service_data[1] += 1;
        // update distance_counter in ad
        err = bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY,
                              ad, ARRAY_SIZE(ad),
                              NULL, 0);
        if (err)
        {
            printk("Advertising failed to start (err %d)\n", err);
            return 0;
        }

        k_sleep(K_MSEC(1000));
        bt_le_adv_stop();
        k_sleep(K_MSEC(4000));
        print_highest_rand_node_num();

        LOG_INF("starting round %d", service_data[1]);
        // reset node information

        // start scanning
    }

    printk("Starting Scanner/Advertiser Demo\n");

    return 0;
}