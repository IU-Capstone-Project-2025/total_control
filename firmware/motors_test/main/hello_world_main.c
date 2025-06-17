/*
 * SPDX-FileCopyrightText: 2010-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/*
 * The following example demonstrates a master node in a TWAI network. The master
 * node is responsible for initiating and stopping the transfer of data messages.
 * The example will execute multiple iterations, with each iteration the master
 * node will do the following:
 * 1) Start the TWAI driver
 * 2) Repeatedly send ping messages until a ping response from slave is received
 * 3) Send start command to slave and receive data messages from slave
 * 4) Send stop command to slave and wait for stop response from slave
 * 5) Stop the TWAI driver
 */
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"

/* --------------------- Definitions and static variables ------------------ */
//Example Configuration
#define PING_PERIOD_MS          1000
#define NO_OF_DATA_MSGS         50
#define NO_OF_ITERS             3
#define ITER_DELAY_MS           1000
#define RX_TASK_PRIO            8
#define TX_TASK_PRIO            9
#define CTRL_TSK_PRIO           10
#define TX_GPIO_NUM             21
#define RX_GPIO_NUM             22
#define EXAMPLE_TAG             "TWAI Master"

#define TWAI_MESSAGE(_id, _dlc, _data...) \
            { .extd = 0, .rtr = 0, .ss = 1, \
              .self = 0, .dlc_non_comp = 0, \
              .identifier = _id, \
              .data_length_code = _dlc, \
              .data = _data }

typedef enum {
    TX_SEND_PINGS,
    TX_SEND_START_CMD,
    TX_SEND_STOP_CMD,
    TX_TASK_EXIT,
} tx_task_action_t;

typedef enum {
    RX_RECEIVE_PING_RESP,
    RX_RECEIVE_DATA,
    RX_RECEIVE_STOP_RESP,
    RX_TASK_EXIT,
} rx_task_action_t;

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);

static SemaphoreHandle_t twai_task_start_sem;
static SemaphoreHandle_t twai_task_done_sem;

/* --------------------------- Tasks and Functions -------------------------- */

static bool twai_request(const twai_message_t *_tx_message, twai_message_t *_rx_message)
{
    twai_transmit(_tx_message, portMAX_DELAY);
    _rx_message->identifier = 0;
    esp_err_t res = twai_receive(_rx_message, pdMS_TO_TICKS(100));
    if(res == ESP_OK)
        return true; // insert check for ID to be correct?
    else
        return false;
}

static void twai_output(twai_message_t *message)
{
    ESP_LOGI(EXAMPLE_TAG, "%lx [%u] %02x %02x %02x %02x %02x %02x %02x %02x",
        message->identifier,
        message->data_length_code,
        message->data[0], message->data[1], message->data[2], message->data[3], 
        message->data[4], message->data[5], message->data[6], message->data[7]);
}

static void twai_control_task(void *arg)
{
    // Wait for end of startup
    xSemaphoreTake(twai_task_start_sem, portMAX_DELAY);
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(EXAMPLE_TAG, "Driver started");

    twai_message_t rx_msg;
    {twai_message_t tx_message = TWAI_MESSAGE(0x141, 8, {0x9A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
    twai_output(&tx_message);
    twai_request(&tx_message, &rx_msg);
    twai_output(&rx_msg);}

    vTaskDelay(pdMS_TO_TICKS(1000));
    
    {twai_message_t tx_message = TWAI_MESSAGE(0x141, 8, {0xA2, 0x00, 0x00, 0x00, 0x88, 0x13, 0x00, 0x00});
    twai_output(&tx_message);
    twai_request(&tx_message, &rx_msg);
    twai_output(&rx_msg);}

    vTaskDelay(pdMS_TO_TICKS(1000));
    
    {twai_message_t tx_message = TWAI_MESSAGE(0x141, 8, {0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
    twai_output(&tx_message);
    twai_request(&tx_message, &rx_msg);
    twai_output(&rx_msg);}


    vTaskDelay(pdMS_TO_TICKS(1000));
    
    {twai_message_t tx_message = TWAI_MESSAGE(0x141, 8, {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
    twai_output(&tx_message);
    twai_request(&tx_message, &rx_msg);
    twai_output(&rx_msg);}


    ESP_ERROR_CHECK(twai_stop());
    ESP_LOGI(EXAMPLE_TAG, "Driver stopped");
    //Delete Control task
    xSemaphoreGive(twai_task_done_sem);
    vTaskDelete(NULL);
}

void app_main(void)
{
    //Create tasks, queues, and semaphores
    twai_task_start_sem = xSemaphoreCreateBinary();
    twai_task_done_sem = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(twai_control_task, "TWAI", 4096, NULL, CTRL_TSK_PRIO, NULL, tskNO_AFFINITY);

    //Install TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(EXAMPLE_TAG, "Driver installed");

    xSemaphoreGive(twai_task_start_sem);              //Start control task
    xSemaphoreTake(twai_task_done_sem, portMAX_DELAY);    //Wait for completion

    //Uninstall TWAI driver
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(EXAMPLE_TAG, "Driver uninstalled");

    //Cleanup
    vSemaphoreDelete(twai_task_start_sem);
    vSemaphoreDelete(twai_task_done_sem);
}
