#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"

#include "driver/twai.h"
#include "driver/uart.h"

/* --------------------- Definitions and static variables ------------------ */
#define CTRL_TSK_PRIO           10
#define SERIAL_TSK_PRIO         24

#define TX_GPIO_NUM             21
#define RX_GPIO_NUM             22
#define TXD_PIN                 1               
#define RXD_PIN                 3    
#define UART_PORT               UART_NUM_0     
#define BUF_SIZE                128            
#define RD_BUF_SIZE             BUF_SIZE    
#define BAUD_RATE               921600     

#define EXAMPLE_TAG             "TWAI Master"
#define SEND_TAG                "Sending"
#define RECIEVE_TAG             "Recieving"


static uint32_t motor_id = 0x142;

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);



static SemaphoreHandle_t twai_task_start_sem;
static SemaphoreHandle_t twai_task_done_sem;

static SemaphoreHandle_t need_a_move;


/* --------------------------- Hardware CAN Functions -------------------------- */
static bool twai_request(const twai_message_t *_tx_message, twai_message_t *_rx_message)
{
    twai_transmit(_tx_message, portMAX_DELAY);
    _rx_message->identifier = 0;
    esp_err_t res = twai_receive(_rx_message, pdMS_TO_TICKS(100));

    if (_rx_message->identifier != motor_id)
    {
        ESP_LOGI(EXAMPLE_TAG, "strange");
        // res = twai_receive(_rx_message, pdMS_TO_TICKS(100));
    }
    if(res == ESP_OK)
        return true; 
    else
        return false;
}

static void twai_output(twai_message_t *message, char* tag)
{
    ESP_LOGI(tag, "%lx [%u] %02x %02x %02x %02x %02x %02x %02x %02x",
        message->identifier,
        message->data_length_code,
        message->data[0], message->data[1], message->data[2], message->data[3], 
        message->data[4], message->data[5], message->data[6], message->data[7]);
}


static void find_my_id(){
    for (uint32_t i = 0x141; i <= 0x160; i++){
        twai_message_t rx_msg;
        twai_message_t tx_message = {.extd = 0, 
                                    .rtr = 0, 
                                    .ss = 1, 
                                    .self = 0, 
                                     .dlc_non_comp = 0, 
                                    .identifier = motor_id, 
                                    .data_length_code = 8, 
                                    .data = {0x9A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
        twai_request(&tx_message, &rx_msg);
        if (rx_msg.identifier == i){
            motor_id = i;
            break;
        }
    }
}



/* --------------------------- Motor Control Functions -------------------------- */

static void motor_request(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
{
    twai_message_t rx_msg;
    twai_message_t tx_message = {.extd = 0, 
                                 .rtr = 0, 
                                 .ss = 1, 
                                 .self = 0, 
                                 .dlc_non_comp = 0, 
                                 .identifier = motor_id, 
                                 .data_length_code = 8, 
                                 .data = {d0, d1, d2, d3, d4, d5, d6, d7}};
    twai_output(&tx_message, SEND_TAG);
    twai_request(&tx_message, &rx_msg);
    twai_output(&rx_msg, RECIEVE_TAG);
    
}


static void motor_request_speed(int32_t vel, int32_t time){
    vel *= 100;
    motor_request(0xA2, 0x00, 0x00, 0x00, vel, vel >> 8, vel >> 16, vel >> 24);   // sending speed setting
    vTaskDelay(pdMS_TO_TICKS(time));
    motor_request(0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00); // sending stop motor
}

static void motor_request_system_reset(){
    motor_request(0x76, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
    vTaskDelay(pdMS_TO_TICKS(5000));
}

static void motor_request_info(){
    motor_request(0x9A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
    vTaskDelay(pdMS_TO_TICKS(1000));
}


static void motor_request_shutdown(){
    motor_request(0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
}




/* --------------------------- Motor Control Task (Process) -------------------------- */

static void twai_control_task(void *arg)
{
    // Wait for end of startup
    xSemaphoreTake(twai_task_start_sem, portMAX_DELAY);
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(EXAMPLE_TAG, "Driver started");

    find_my_id();
    
    motor_request_system_reset();
    motor_request_info();
    
    while(1){
        xSemaphoreTake(need_a_move, portMAX_DELAY); 
        motor_request_speed(500, 1000);
    }
    
    motor_request_shutdown();


    ESP_ERROR_CHECK(twai_stop());
    ESP_LOGI(EXAMPLE_TAG, "Driver stopped");
    //Delete Control task
    xSemaphoreGive(need_a_move);
    vTaskDelete(NULL);
}

/* --------------------------- Serial Handler -------------------------- */


void uart_init_setup() {
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE * 2, 0, 0, NULL, 0));
}


void uart_event_task(void *pvParameters) {
    uint8_t *data = (uint8_t *) malloc(RD_BUF_SIZE + 1);
    if (!data) {
        ESP_LOGE(EXAMPLE_TAG, "Ошибка выделения памяти");
        vTaskDelete(NULL);
    }


    while (true) {
        const int rxBytes = uart_read_bytes(UART_PORT, data, RD_BUF_SIZE, pdMS_TO_TICKS(25));
        if (rxBytes > 0) {
            // uart_write_bytes(UART_PORT, (const char*)data, rxBytes);
            
            if (strstr((const char *)data, "ROTATION")) {
                xSemaphoreGive(need_a_move); 
            }
        }
    }
    free(data);
    vTaskDelete(NULL);
}



/* --------------------------- Main -------------------------- */

void app_main(void)
{
    //Create tasks, queues, and semaphores
    twai_task_start_sem = xSemaphoreCreateBinary();
    twai_task_done_sem = xSemaphoreCreateBinary();
    need_a_move = xSemaphoreCreateBinary();

    uart_init_setup();


    xTaskCreatePinnedToCore(twai_control_task, "motor_control", 4096, NULL, CTRL_TSK_PRIO, NULL, 1);
    xTaskCreatePinnedToCore(uart_event_task, "serial_handler", 4096, NULL, SERIAL_TSK_PRIO, NULL, 0);

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
