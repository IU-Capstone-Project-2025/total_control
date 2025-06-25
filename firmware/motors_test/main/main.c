#include <stdio.h>
#include <stdlib.h>
#include <math.h>
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
#include "driver/timer.h"

/* --------------------- Definitions and static variables ------------------ */
#define CTRL_TSK_PRIO           10
#define SERIAL_TSK_PRIO         20
#define SAVER_TSK_PRIO          20

#define ESP_INTR_FLAG_DEFAULT   0

#define TX_GPIO_CAN             22
#define RX_GPIO_CAN             21
#define ENC_LINEAR_GPIO_1       19
#define ENC_LINEAR_GPIO_2       18
#define ENC_ANGULAR_GPIO_A      5
#define ENC_ANGULAR_GPIO_B      4
#define ENC_ANGULAR_GPIO_C      2
#define BTN_GPIO                23
#define GPIO_PIN_MASK(PIN)      (1ULL<<PIN)

#define ANGLE_STEP_SIZE         0.08789
#define MAX_ECNODER_DATA        12213
#define SAFE_REGION             1000
#define INIT_TORQUE             70

#define TXD_PIN                 1               
#define RXD_PIN                 3    
#define UART_PORT               UART_NUM_0     
#define BUF_SIZE                128          
#define BAUD_RATE               921600     
#define SERIAL_READ_MS_DELAY    20

#define ERROR_TAG               "Error"
#define MAIN_TAG                "Main"
#define DEBUG_TAG               "Debug"
#define SEND_TAG                "Motor/Sending"
#define RECIEVE_TAG             "Motor/Recieving"
#define READ_TAG                "PC/Sending"
#define WRITE_TAG               "PC/Recieving"

#define MOTOR_STOP_COMMAND      1000000
#define GT_READY_STATE_COMMAND  1000001 
#define GET_MOTOR_INFO          1000002




static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_CAN, RX_GPIO_CAN, TWAI_MODE_NORMAL);

static SemaphoreHandle_t motor_control_task_start_sem;
static SemaphoreHandle_t motor_control_task_done_sem;
static SemaphoreHandle_t init_start_sem;
static SemaphoreHandle_t init_done_sem;
static SemaphoreHandle_t btn_sem;
static SemaphoreHandle_t move_sem;
static SemaphoreHandle_t smert_sem;

static uint32_t motor_id = 0x141;
static int64_t recieved_packet;

volatile int16_t encoder_position;
volatile int16_t encoder_position_prev;
volatile double encoder_angle = -80;
volatile double encoder_angle_prev;
volatile bool init_in_progress = false;
static bool not_inited_yet = true;

void uart_ready_state(char* data);
void uart_get_delay(char* data);
void uart_get_mode(char* data);
void uart_oper_state(char* data);

static void (*uart_state)(char*) = uart_ready_state; 

static uint16_t sensor_info_sender_delay = 20;
TimerHandle_t sensor_info_sender_timer;
void sensor_info_sender();



/* --------------------------- Hardware CAN Functions -------------------------- */

static bool twai_request(const twai_message_t *_tx_message, twai_message_t *_rx_message)
{
    twai_transmit(_tx_message, portMAX_DELAY);
    _rx_message->identifier = 0;
    esp_err_t res = twai_receive(_rx_message, pdMS_TO_TICKS(100));

    if (_rx_message->identifier != motor_id) {
        ESP_LOGI(ERROR_TAG, "no reply from motor");
    }

    if(res == ESP_OK)
        return true; 
    else
        return false;
}

static void twai_output(char* tag, twai_message_t *message)
{
    ESP_LOGI(tag, "%lx [%u] %02x %02x %02x %02x %02x %02x %02x %02x",
        message->identifier,
        message->data_length_code,
        message->data[0], message->data[1], message->data[2], message->data[3], 
        message->data[4], message->data[5], message->data[6], message->data[7]);
}


static void find_my_id()
{
    for (uint32_t i = 0x141; i <= 0x160; i++) {
        twai_message_t rx_message;
        twai_message_t tx_message ={.extd = 0, 
                                    .rtr = 0, 
                                    .ss = 1, 
                                    .self = 0, 
                                    .dlc_non_comp = 0, 
                                    .identifier = motor_id, 
                                    .data_length_code = 8, 
                                    .data = {0x9A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
        twai_request(&tx_message, &rx_message);
        if (rx_message.identifier == i) {
            motor_id = i;
            ESP_LOGI(MAIN_TAG, "Motor found, motor_id is %lx", motor_id);
            return;
        }
    }
    ESP_LOGI(ERROR_TAG, "Motor not found");
}





/* --------------------------- Motor Control Functions -------------------------- */

static void motor_request(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
{
    twai_message_t rx_message;
    twai_message_t tx_message = {.extd = 0, 
                                 .rtr = 0, 
                                 .ss = 1, 
                                 .self = 0, 
                                 .dlc_non_comp = 0, 
                                 .identifier = motor_id, 
                                 .data_length_code = 8, 
                                 .data = {d0, d1, d2, d3, d4, d5, d6, d7}};
    twai_output(SEND_TAG, &tx_message);
    twai_request(&tx_message, &rx_message);
    twai_output(RECIEVE_TAG, &rx_message);
    
}


static void motor_request_stop()
{
    motor_request(0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00); 
    motor_request(0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);   
}

static void motor_request_system_reset()
{
    motor_request(0x76, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
    vTaskDelay(pdMS_TO_TICKS(1000));
}

static void motor_request_info()
{
    motor_request(0x9A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
}


static void motor_request_shutdown()
{
    motor_request(0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);    
}

static void motor_request_torque(int16_t tau)
{
    motor_request(0xA1, 0x00, 0x00, 0x00, (uint8_t) (-tau), (uint8_t) (-tau >> 8), 0x00, 0x00);   // sending speed setting
}

static void motor_request_speed_on_time(int32_t vel, int32_t time)
{
    vel *= 100;
    motor_request(0xA2, 0x00, 0x00, 0x00, (uint8_t) (vel), (uint8_t) (vel >> 8),  (uint8_t) (vel >> 16),  (uint8_t) (vel >> 24));   // sending speed setting
    vTaskDelay(pdMS_TO_TICKS(time));
    motor_request(0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00); // sending stop motor
}




/* --------------------------- Motor Control Task -------------------------- */

static void motor_control_task(void *arg)
{
    xSemaphoreTake(motor_control_task_start_sem, portMAX_DELAY);
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(MAIN_TAG, "Driver started");
    
    motor_request_system_reset();
    motor_request_info();
    
    while(1){
        xSemaphoreTake(move_sem, portMAX_DELAY);
        motor_request_torque(recieved_packet);
    }
    
    // motor_request_shutdown();


    // ESP_ERROR_CHECK(twai_stop());
    // ESP_LOGI(MAIN_TAG, "Driver stopped");
    // xSemaphoreGive(motor_control_task_done_sem);
    // vTaskDelete(NULL);
}

static void motor_init_function()
{
    motor_request_stop();
    init_in_progress = true;

    xSemaphoreTake(btn_sem, 0);
    motor_request_torque(-INIT_TORQUE);
    

    xSemaphoreTake(btn_sem, portMAX_DELAY);
    ESP_LOGI(MAIN_TAG, "Button CLICKED");
    
    motor_request_stop();
    encoder_position = 0;
    motor_request_torque(INIT_TORQUE);

    while (encoder_position < MAX_ECNODER_DATA / 2) {}
    motor_request_stop();
    if (not_inited_yet){
        xSemaphoreGive(init_done_sem);
    }
    init_in_progress = false;
}

static void motor_self_saver_task(void *arg)
{
    xSemaphoreTake(init_done_sem, portMAX_DELAY);
    not_inited_yet = false;
    while(1){
        if (!init_in_progress){
            if(encoder_position < SAFE_REGION || encoder_position > MAX_ECNODER_DATA - SAFE_REGION){
                ESP_LOGI(ERROR_TAG, "Danger Situation");
                motor_init_function();
            }
        } 
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


/* --------------------------- UART Handler -------------------------- */

void uart_init_setup() 
{
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


void uart_event_task(void *pvParameters) 
{
    char* data = (char*) malloc(BUF_SIZE);
    memset(data, 0, BUF_SIZE);
    if (!data) {
        ESP_LOGE(ERROR_TAG, "Ошибка выделения памяти");
        vTaskDelete(NULL);
    }
    
    while (true)
    {
        const int rxBytes = uart_read_bytes(UART_PORT, data, BUF_SIZE, pdMS_TO_TICKS(SERIAL_READ_MS_DELAY));
        if (rxBytes > 0) {
            uart_state(data);    
        }
    }
    free(data);
    vTaskDelete(NULL);
}




/* --------------------------- UART states -------------------------- */

void rtoo(){   // READY to OPERATIONAL
    uart_state = uart_oper_state;
    sensor_info_sender_timer = xTimerCreate("sensor_info_sender", pdMS_TO_TICKS(sensor_info_sender_delay), pdTRUE, 0, sensor_info_sender);

    if(sensor_info_sender_timer != NULL) {
        xTimerStart(sensor_info_sender_timer, 0);
    }
}

void otor(){   // OPERATIONAL to READY
    uart_state = uart_ready_state;
    if(sensor_info_sender_timer != NULL) {
        xTimerStop(sensor_info_sender_timer, 0);
        xTimerDelete(sensor_info_sender_timer, 0);
    }
}



void uart_ready_state(char* data)
{
    // start button
    if (strstr((const char *)data, "START_OPER")) {
        ESP_LOGI(WRITE_TAG, "Starting operational state");
        rtoo();
    } else if (strstr((const char *)data, "HELP")) {
        ESP_LOGI(WRITE_TAG, "Possible commands\nSTART_OPER\nFIND_MOTOR, MOTOR_INFO, RESET_MOTOR, MOTOR_INIT, MOTOR_MODE\nDATA_SEND_MODE\nENCODER_POSITION, ENCODER_ANGLE");
    } 

    // motoro control mode
    else if (strstr((const char *)data, "MOTOR_MODE")) {
        ESP_LOGI(WRITE_TAG, "Possible variants\nTORQUE, SPEED, POSITION");
        uart_state = uart_get_mode;
    } else if (strstr((const char *)data, "FIND_MOTOR")) {
        find_my_id();
    } else if (strstr((const char *)data, "RESET_MOTOR")) {
        motor_request_system_reset();
    } else if (strstr((const char *)data, "MOTOR_INFO")) {
        motor_request_info();
    } else if (strstr((const char *)data, "MOTOR_INIT")) {
        motor_init_function();
    }


    // set data sending mode
    else if (strstr((const char *)data, "DATA_SEND_MODE")) {
        ESP_LOGI(WRITE_TAG, "Conifgure data sending protocols, write delay (ms)");
        uart_state = uart_get_delay;
    } 

    // getters
    else if (strstr((const char *)data, "ENCODER_POSITION")) {
        ESP_LOGI(WRITE_TAG, "Position is %d", encoder_position);
    } else if (strstr((const char *)data, "ENCODER_ANGLE")) {
        ESP_LOGI(WRITE_TAG, "Angle is %f", encoder_angle);
    } else {
        ESP_LOGI(ERROR_TAG, "Undefined behaviour");
    }
}


void uart_get_mode(char* data){ // TODO
    if (strstr((const char *)data, "TORQUE")){
        ESP_LOGI(WRITE_TAG, "Motor is configured to torque control mode");
    } else if (strstr((const char *)data, "SPEED")){
        ESP_LOGI(WRITE_TAG, "Motor is configured to speed control mode");
    } else if (strstr((const char *)data, "POSITION")){
        ESP_LOGI(WRITE_TAG, "Motor is configured to position control mode");
    } else {
        ESP_LOGI(ERROR_TAG, "Undefined behaviour");
    }
    uart_state = uart_ready_state;
}


void uart_get_delay(char* data){
    sensor_info_sender_delay = atoi(data);
    ESP_LOGI(WRITE_TAG, "delay is configured to %d", sensor_info_sender_delay);
    uart_state = uart_ready_state;
}


void uart_oper_state(char* data)
{
    recieved_packet = atoi(data);
    if (recieved_packet < INT16_MIN || recieved_packet > INT16_MAX){
        if (recieved_packet == MOTOR_STOP_COMMAND){
            motor_request_stop();
            return;
        } else if (recieved_packet == GT_READY_STATE_COMMAND) {
            motor_request_stop();
            ESP_LOGI(MAIN_TAG, "starting ready state");
            otor();
            return;
        } else if (recieved_packet == GET_MOTOR_INFO) {
            motor_request_info();
            return;
        }
        recieved_packet = 0;
    }
    xSemaphoreGive(move_sem);       
}


void sensor_info_sender(){
    int32_t linear_velocity = (encoder_position - encoder_position_prev) * 1000 / sensor_info_sender_delay;
    double angular_velocity = (encoder_angle - encoder_angle_prev) * 1000 / sensor_info_sender_delay;
    encoder_angle_prev = encoder_angle;
    encoder_position_prev = encoder_position;

    printf("%d %ld %f %f\n", encoder_position, linear_velocity, encoder_angle, angular_velocity);
}



/*-------------------------- Sensors --------------------------*/




static void IRAM_ATTR enc_linear_isr_handler(void* arg)
{
    encoder_position += (gpio_get_level(ENC_LINEAR_GPIO_1) != gpio_get_level(ENC_LINEAR_GPIO_2)) ? 1 : -1;
}

static void IRAM_ATTR enc_angular_change_isr_handler(void* arg)
{
    encoder_angle -= ANGLE_STEP_SIZE * (1 - gpio_get_level(ENC_ANGULAR_GPIO_A) * 2);
}

static void IRAM_ATTR enc_angular_zero_isr_handler(void* arg)
{
    encoder_angle = 0;
}

static void IRAM_ATTR btn_isr_handler(void* arg)
{
    if (init_in_progress){
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(btn_sem, &xHigherPriorityTaskWoken);
    }
}


static void gpio_init_setup(){

    // linear
    gpio_config_t io_conf_1;
    io_conf_1.pull_down_en = 1;
    io_conf_1.mode = GPIO_MODE_INPUT;
    io_conf_1.intr_type = GPIO_INTR_ANYEDGE;
    io_conf_1.pin_bit_mask = GPIO_PIN_MASK(ENC_LINEAR_GPIO_1);
    gpio_config(&io_conf_1);

    gpio_config_t io_conf_2;
    io_conf_2.pull_down_en = 1;
    io_conf_2.mode = GPIO_MODE_INPUT;
    io_conf_2.intr_type = GPIO_INTR_DISABLE;
    io_conf_2.pin_bit_mask = GPIO_PIN_MASK(ENC_LINEAR_GPIO_2);
    gpio_config(&io_conf_2);

    // angular
    gpio_config_t io_conf_3;
    io_conf_3.pull_down_en = 1;
    io_conf_3.mode = GPIO_MODE_INPUT;
    io_conf_3.pin_bit_mask = GPIO_PIN_MASK(ENC_ANGULAR_GPIO_A);
    io_conf_3.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf_3);

    gpio_config_t io_conf_4;
    io_conf_4.pull_down_en = 1;
    io_conf_4.mode = GPIO_MODE_INPUT;
    io_conf_4.pin_bit_mask = GPIO_PIN_MASK(ENC_ANGULAR_GPIO_B);
    io_conf_4.intr_type = GPIO_INTR_POSEDGE;
    gpio_config(&io_conf_4);

    gpio_config_t io_conf_5;
    io_conf_5.pull_down_en = 1;
    io_conf_5.mode = GPIO_MODE_INPUT;
    io_conf_5.pin_bit_mask = GPIO_PIN_MASK(ENC_ANGULAR_GPIO_C);
    io_conf_5.intr_type = GPIO_INTR_POSEDGE;
    gpio_config(&io_conf_5);

    // button 
    gpio_config_t io_conf_6;
    io_conf_6.pull_down_en = 1;
    io_conf_6.mode = GPIO_MODE_INPUT;
    io_conf_6.pin_bit_mask = GPIO_PIN_MASK(BTN_GPIO);
    io_conf_6.intr_type = GPIO_INTR_POSEDGE;
    gpio_config(&io_conf_6);

    // interrupts 
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(ENC_LINEAR_GPIO_1, enc_linear_isr_handler, 0);
    gpio_isr_handler_add(ENC_ANGULAR_GPIO_B, enc_angular_change_isr_handler, 0);
    gpio_isr_handler_add(ENC_ANGULAR_GPIO_C, enc_angular_zero_isr_handler, 0);
    gpio_isr_handler_add(BTN_GPIO, btn_isr_handler, 0);
    
}


/* --------------------------- Main -------------------------- */

void app_main(void)
{
    //Create tasks, queues, and semaphores
    motor_control_task_start_sem = xSemaphoreCreateBinary();
    motor_control_task_done_sem = xSemaphoreCreateBinary();
    move_sem = xSemaphoreCreateBinary();
    smert_sem = xSemaphoreCreateBinary();
    btn_sem = xSemaphoreCreateBinary();
    init_start_sem = xSemaphoreCreateBinary();
    init_done_sem = xSemaphoreCreateBinary();


    gpio_init_setup();
    uart_init_setup();


    //Install TWAI driver
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(MAIN_TAG, "Driver installed");


    
    xTaskCreate(motor_control_task,     "motor_control",            4096,   NULL,   CTRL_TSK_PRIO,      NULL);
    xTaskCreate(uart_event_task,        "serial_handler",           4096,   NULL,   SERIAL_TSK_PRIO,    NULL);
    xTaskCreate(motor_self_saver_task,  "motor_self_saver_task",    4096,   NULL,   SAVER_TSK_PRIO,     NULL);


    xSemaphoreGive(motor_control_task_start_sem);
    xSemaphoreTake(motor_control_task_done_sem, portMAX_DELAY);    
    

    //Uninstall TWAI driver
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(MAIN_TAG, "Driver uninstalled");


    //Cleanup
    vSemaphoreDelete(motor_control_task_start_sem);
    vSemaphoreDelete(motor_control_task_done_sem);
}
