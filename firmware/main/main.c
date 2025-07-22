/**
 * \file main.c
 * \brief Main application entry point and firmware for CartPole controller.
 *
 * Initializes CPU power management, GPIOs, UART, and CAN interfaces.
 * Creates FreeRTOS tasks for motor control, safety monitoring,
 * UART command handling, sensor data reporting, and hardware tests.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_pm.h"
#include "driver/twai.h"
#include "driver/uart.h"
#include "driver/timer.h"


/* --------------------- Definitions ------------------ */
/**< Motor control task priority */
#define CTRL_TSK_PRIO           10
/**< UART and sensor info task priority */          
#define SERIAL_TSK_PRIO         15          
/**< Safety monitor task priority */
#define SAVER_TSK_PRIO          20          
/**< Hardware test task priority */
#define TEST_TASK_PRIO          1           
/**< Default interrupt flag */
#define ESP_INTR_FLAG_DEFAULT   0           
/**< CAN TX GPIO */
#define TX_GPIO_CAN             21          
/**< CAN RX GPIO */
#define RX_GPIO_CAN             22          
/**< Linear encoder channel A */
#define ENC_LINEAR_GPIO_1       19          
/**< Linear encoder channel B */
#define ENC_LINEAR_GPIO_2       18          
/**< Angular encoder channel A */
#define ENC_ANGULAR_GPIO_A      5           
/**< Angular encoder channel B */
#define ENC_ANGULAR_GPIO_B      4           
/**< Angular encoder index signal */
#define ENC_ANGULAR_GPIO_C      2           
/**< User button GPIO */
#define BTN_GPIO                23          
#define GPIO_PIN_MASK(PIN)      (1ULL<<PIN)
/**< Angular step [rad] */
#define ANGLE_STEP_SIZE         0.08789     
/**< Linear encoder max count */
#define MAX_ECNODER_DATA        12213       
/**< Safe region threshold */
#define SAFE_REGION             1000        
/**< Initial torque for motor initialization */
#define INIT_TORQUE             75          
/**< Reinitialization torque for motor */
#define REINIT_TORQUE           75          

/**< UART TX pin */
#define TXD_PIN                 1           
/**< UART RX pin */
#define RXD_PIN                 3           
/**< UART port */
#define UART_PORT               UART_NUM_0  
/**< UART buffer size */
#define BUF_SIZE                128         
/**< UART baud rate */
#define BAUD_RATE               921600      
/**< UART read delay [ms] */
#define SERIAL_MS_DELAY         20          

/**< Error log tag */
#define ERROR_TAG               "Error"     
/**< Main log tag */
#define MAIN_TAG                "Main"      
/**< Debug log tag */
#define DEBUG_TAG               "Debug"     
/**< Motor sending log tag */
#define SEND_TAG                "Motor/Sending" 
/**< Motor receiving log tag */
#define RECIEVE_TAG             "Motor/Recieving"   
/**< PC receiving log tag */
#define READ_TAG                "PC/Recieving"      
/**< PC sending log tag */
#define WRITE_TAG               "PC/Sending"        

/**< Stop command code */
#define MOTOR_STOP_COMMAND      1000000 
/**< Ready state command code */
#define READY_STATE_COMMAND     1000001 
/**< Info request command code */
#define MOTOR_INFO_COMMMAND     1000002 
/**< Reset command code */
#define RESET_COMMAND           1000003 

// #define DEBUG                    0                   ///< Debug mode flag (uncomment for debug)



/* --------------------- Static variables ------------------ */

/* CAN driver configurations */
static const twai_timing_config_t can_timing_config  = TWAI_TIMING_CONFIG_1MBITS();
static const twai_filter_config_t can_filter_config  = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_general_config_t can_general_config  = TWAI_GENERAL_CONFIG_DEFAULT(TX_CAN_GPIO, RX_CAN_GPIO, TWAI_MODE_NORMAL);

/* System synchronization primitives */
static SemaphoreHandle_t restart_command_semaphore;
static SemaphoreHandle_t initialization_start_semaphore;
static SemaphoreHandle_t initialization_done_semaphore;
static SemaphoreHandle_t button_press_semaphore;
static SemaphoreHandle_t motor_command_semaphore;
static SemaphoreHandle_t sensor_data_request_semaphore;
static SemaphoreHandle_t test_sync_semaphore;

/* Device identifiers */
static uint32_t motor_can_id = 0x141;

/* Communication buffers */
static int64_t uart_recieved_packet;

/* Motor comunication function */
static void motor_operate_by_torque(uint32_t torque);
static void motor_operate_by_speed(uint32_t speed);
static void motor_operate_by_position(uint32_t position);

static void (*motor_operate) (uint32_t) = motor_operate_by_torque;


/* Sensor state */
static volatile double current_encoder_position;
static volatile double current_encoder_angle = 179.93;
static double previous_encoder_position;
static double previous_encoder_angle;

/* System state flags */
static bool initialization_in_progress = false;
static bool not_initiazatied_yet = true;
static bool system_in_safe_state = true;

/* UART state machine function pointers */
static void uart_ready_state(char* data);
static void uart_operational_state(char* data);
static void uart_set_sender_delay(char* data);
static void uart_set_motor_drive_mode(char* data);
static void uart_test_button (char* data);
static void uart_test_encoder (char* data);
static void uart_test_angle (char* data);

static void (*uart_state)(char*) = uart_ready_state; 

/* Sensor reporting */
static uint16_t sensor_timer_delay = UART_DEFAULT_SEND_DELAY;
static TimerHandle_t sensor_timer_handle;
static void sensor_timer_callback();

/* Task handles */
static TaskHandle_t motor_control_task_handle;
static TaskHandle_t uart_event_task_handle;
static TaskHandle_t safety_monitor_task_handle;
static TaskHandle_t sensor_info_sender_task_handle;
static TaskHandle_t hardware_test_task_handle;

/* Test status */
static bool hardware_tests_passed = false;


/* --------------------------- Hardware CAN Functions -------------------------- */

/**
 * @brief Sends TWAI message and verifies response ID.
 * @param[in] _tx_message Transmit message pointer.
 * @param[out] _rx_message Receive buffer pointer.
 * @return true if response ID matches, false otherwise.
 */
static bool twai_request(const twai_message_t *_tx_message, twai_message_t *_rx_message)
{
    twai_transmit(_tx_message, portMAX_DELAY);
    _rx_message->identifier = 0;
    esp_err_t res = twai_receive(_rx_message, pdMS_TO_TICKS(100));

    if (_rx_message->identifier != motor_can_id) {
        ESP_LOGE(ERROR_LOG_TAG, "No reply from motor");
        return false;
    }

    if(res == ESP_OK)
        return true; 
    else
        return false;
}

/**
 * @brief Sends TWAI message without ID check.
 * @param[in] _tx_message Transmit message pointer.
 * @param[out] _rx_message Receive buffer pointer.
 * @return true if any response received, false otherwise.
 */
static bool twai_request_wo_id_check(const twai_message_t *_tx_message, twai_message_t *_rx_message)
{
    twai_transmit(_tx_message, portMAX_DELAY);
    _rx_message->identifier = 0;
    esp_err_t res = twai_receive(_rx_message, pdMS_TO_TICKS(100));

    if(res == ESP_OK)
        return true; 
    else
        return false;
}

/**
 * @brief Logs TWAI message content.
 * @param[in] tag Log tag string.
 * @param[in] message Message pointer to log.
 */
static void twai_output(char* tag, twai_message_t *message)
{
    ESP_LOGI(tag, "%lx [%u] %02x %02x %02x %02x %02x %02x %02x %02x",
        message->identifier,
        message->data_length_code,
        message->data[0],
        message->data[1],
        message->data[2],
        message->data[3], 
        message->data[4],
        message->data[5],
        message->data[6],
        message->data[7]);
}

/**
 * @brief Discovers motor CAN ID by scanning range.
 */
static void find_my_id()
{
    for (uint32_t i = 0x141; i <= 0x160; i++) {
        twai_message_t rx_message;
        twai_message_t tx_message ={.extd = 0, 
                                    .rtr = 0, 
                                    .ss = 1, 
                                    .self = 0, 
                                    .dlc_non_comp = 0, 
                                    .identifier = motor_can_id, 
                                    .data_length_code = 8, 
                                    .data = {0x9A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
        twai_transaction(&tx_message, &rx_message);
        if (rx_message.identifier == i) {
            motor_can_id = i;
            ESP_LOGI(MAIN_LOG_TAG, "Motor found, motor_can_id is %lx", motor_can_id);
            return;
        }
    }
    ESP_LOGE(ERROR_LOG_TAG, "Motor not found");
}




/* --------------------------- Motor Control Functions -------------------------- */
/**
 * @brief Sends motor command over CAN and waits for optional reply.
 *
 * Constructs a CAN frame with the provided data bytes and transmits it.
 * If DEBUG is disabled, waits for a confirmation response via twai_request().
 *
 * @param dX Data bytes to include in the frame. (0-7)
 * 
 * @return true if transmission (and optional reply) succeeded, false otherwise.
 */
static bool motor_request(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
{
    twai_message_t rx_message;
    twai_message_t tx_message = {.extd = 0, 
                                 .rtr = 0, 
                                 .ss = 1, 
                                 .self = 0, 
                                 .dlc_non_comp = 0, 
                                 .identifier = motor_can_id, 
                                 .data_length_code = 8, 
                                 .data = {d0, d1, d2, d3, d4, d5, d6, d7}};

    #ifdef DEGUB
    log_can_message(CAN_TX_LOG_TAG, &tx_message);
    if(!twai_transaction(&tx_message, &rx_message)){
        log_can_message(CAN_RX_LOG_TAG, &rx_message);
        return false;
    }
    log_can_message(CAN_RX_LOG_TAG, &rx_message);
    #endif

    #ifndef DEBUG
    if(!twai_transaction(&tx_message, &rx_message)){
        return false;
    }
    #endif

    return true;
}

/**
 * @brief Sends motor command over CAN without waiting for a reply.
 *
 * Constructs and transmits a CAN frame with the provided data bytes,
 * then ignores any response.
 *
 * @param dX Data bytes to include in the frame. (0-7)
 */
static void motor_request_wo_reply(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
{
    twai_message_t rx_message;
    twai_message_t tx_message = {.extd = 0, 
                                 .rtr = 0, 
                                 .ss = 1, 
                                 .self = 0, 
                                 .dlc_non_comp = 0, 
                                 .identifier = motor_can_id, 
                                 .data_length_code = 8, 
                                 .data = {d0, d1, d2, d3, d4, d5, d6, d7}};

    twai_request_wo_id_check(&tx_message, &rx_message);

}

/**
 * @brief Issues immediate stop command to motor.
 *
 * Sends two sequential stop frames to ensure motor halts.
 */
static void motor_request_stop()
{
    motor_request(0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00); 
    motor_request(0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);   
}

/**
 * @brief Triggers a system reset command on the motor.
 *
 * Sends reset frame and delays for 1 second to allow reboot.
 */
static void motor_request_system_reset()
{
    motor_request_wo_reply(0x76, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
    vTaskDelay(pdMS_TO_TICKS(1000));
}

/**
 * @brief Requests telemetry information from the motor.
 *
 * Sends info request frame to retrieve voltage, temperature, and error codes.
 */
static void motor_request_info()
{
    send_motor_command(0x9A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
}

/**
 * @brief Commands motor to enter shutdown state.
 */
static void motor_request_shutdown()
{
    motor_request(0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);    
}


/**
 * @brief Applies signed torque command to the motor.
 *
 * @param tau Desired torque value (signed).
 */
static void motor_request_torque(int16_t tau)
{
    motor_request(0xA1, 0x00, 0x00, 0x00, (uint8_t) (-tau), (uint8_t) (-tau >> 8), 0x00, 0x00);   
}

/**
 * @brief Sets motor target velocity.
 *
 * Multiplies velocity by 100 (unit scaling) and sends command frame.
 *
 * @param vel Desired speed in encoder units/sec.
 */
static void motor_request_speed(int32_t vel)
{
    vel *= 100;
    motor_request(0xA2, 0x00, 0x00, 0x00, (uint8_t) (vel), (uint8_t) (vel >> 8),  (uint8_t) (vel >> 16),  (uint8_t) (vel >> 24));  
}

/**
 * @brief Checks if motor responds to info request.
 *
 * @return true if motor acknowledges, false otherwise.
 */
static bool motor_request_is_connected(){
    return motor_request(0x9A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
}


/* --------------------------- Motor Control Task -------------------------- */
/**
 * @brief FreeRTOS task for motor control operations.
 *
 * Performs a system reset and info request on the motor, then enters a loop
 * waiting for torque commands via the move_sem semaphore.
 *
 * @param arg Unused parameter.
 */
static void motor_control_task(void *arg)
{
    send_motor_command_system_reset();
    if (!send_motor_command_is_connected()){
        ESP_LOGE(ERROR_LOG_TAG, "Motor no connected");
    }
    
    while(1){
        xSemaphoreTake(motor_command_semaphore, portMAX_DELAY);
        motor_operate(uart_recieved_packet);
    }
    
}

/**
 * @brief Initializes motor by centering encoder and establishing zero position.
 *
 * Applies torque in opposite directions and waits for user confirmation via btn_sem.
 * Monitors encoder_position to detect center; enforces timeout of 3 seconds.
 * Signals completion through init_done_sem if first initialization.
 */
static void motor_init_function()
{

    send_motor_command_stop();
    initialization_in_progress = true;
    ESP_LOGI(MAIN_LOG_TAG, "Initializing motor");
    xSemaphoreTake(button_press_semaphore, 0);
    motor_operate_by_speed(-INITIALIZATION_SPPED);
    
    xSemaphoreTake(button_press_semaphore, pdMS_TO_TICKS(5000));

    #ifdef DEBUG
    ESP_LOGI(MAIN_LOG_TAG, "Button CLICKED");
    #endif
    
    send_motor_command_stop();
    current_encoder_position = 0;
    motor_operate_by_speed(INITIALIZATION_SPPED);

    int64_t time = esp_timer_get_time()/1000;
    while (current_encoder_position < MAX_LINEAR_ECNODER_VALUE / 2){
        if (time + 3000 < esp_timer_get_time()/1000){
            send_motor_command_stop();
            initialization_in_progress = false;
            ESP_LOGE(ERROR_LOG_TAG, "Motor initialization failed!");
            return;
        }
    }
    

    send_motor_command_stop();
    current_encoder_position = 0;
    if (not_initiazatied_yet){
        xSemaphoreGive(initialization_done_semaphore);
    }
    initialization_in_progress = false;
    send_motor_command_zero();
    ESP_LOGI(MAIN_LOG_TAG, "Initialize ended");
}

/**
 * @brief FreeRTOS safety monitor task for motor encoder limits.
 *
 * Waits for initial motor setup completion, then continuously checks the
 * encoder_position against SAFE_REGION boundaries. If out-of-bounds,
 * logs danger, reinitializes the motor, and restores safe state.
 *
 * @param arg Unused parameter.
 */
static void motor_self_saver_task(void *arg)
{
    xSemaphoreTake(initialization_done_semaphore, portMAX_DELAY);
    not_initiazatied_yet = false;
    while(1){
        if (!initialization_in_progress){
            if((current_encoder_position < -MAX_LINEAR_ECNODER_VALUE/2 + SAFE_REGION_MARGIN) || (current_encoder_position > MAX_LINEAR_ECNODER_VALUE/2 - SAFE_REGION_MARGIN)){
                system_in_safe_state = false;
                ESP_LOGE(ERROR_LOG_TAG, "Danger situation, stopping motor");
                motor_init_function();
                system_in_safe_state = true;
            }
        } 
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


/* --------------------------- UART Handler -------------------------- */
/**
 * @brief Configures UART parameters and installs driver.
 *
 * Sets UART baud rate, data bits, parity, stop bits, and flow control,
 * then configures TX/RX pins and installs the UART driver with buffer.
 */
void uart_init_setup() 
{
    uart_config_t uart_configuration = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUMBER, &uart_configuration));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUMBER, CAN_TX_PIN, CAN_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUMBER, UART_BUFFER_SIZE * 2, 0, 0, NULL, 0));
}

/**
 * @brief FreeRTOS task for UART event processing.
 *
 * Continuously reads bytes from UART into a buffer and invokes the
 * current state handler function pointer with the received data.
 *
 * @param pvParameters Unused task parameter.
 */
void uart_event_task(void *pvParameters) 
{
    char* data = (char*) malloc(UART_BUFFER_SIZE);
    memset(data, 0, UART_BUFFER_SIZE);
    if (!data) {
        ESP_LOGE(ERROR_LOG_TAG, "Memory allocation error");
        vTaskDelete(NULL);
    }
    
    while (true) {
        const int rxBytes = uart_read_bytes(UART_PORT_NUMBER, data, UART_BUFFER_SIZE, pdMS_TO_TICKS(UART_DEFAULT_SEND_DELAY));
        if (rxBytes > 0) {
            uart_state(data);    
        }
    }
}


/* --------------------------- UART states -------------------------- */
/**
 * @brief Switches UART handler to operational state.
 *
 * Creates and starts a periodic timer to trigger sensor info reporting.
 */
void rtoo()   // READY to OPERATIONAL
{   
    uart_state = uart_operational_state;
    sensor_timer_handle = xTimerCreate("sensor_timer_callback", pdMS_TO_TICKS(sensor_timer_delay), pdTRUE, 0, sensor_timer_callback);

    if(sensor_timer_handle != NULL) {
        xTimerStart(sensor_timer_handle, 0);
    }
}

/**
 * @brief Switches UART handler to ready state.
 *
 * Stops and deletes the sensor info reporting timer if active.
 */
void otor()   // OPERATIONAL to READY
{   
    uart_state = uart_ready_state;
    if(sensor_info_sender_timer_handle != NULL) {
        xTimerStop(sensor_info_sender_timer_handle, 0);
        xTimerDelete(sensor_info_sender_timer_handle, 0);
    }
}

/**
 * @brief UART handler for READY state commands.
 *
 * Parses incoming text commands such as START_OPER, RESTART, HELP,
 * and routes to appropriate actions or state transitions.
 *
 * @param data Null-terminated command string.
 */
void uart_ready_state(char* data)
{
    if (strstr((const char *)data, "START_OPER")) {
        ESP_LOGI(TO_PC_LOG_TAG, "Starting operational state");
        from_ready_to_operational();
    } else if (strstr((const char *)data, "RESTART")) {
        xSemaphoreGive(restart_command_semaphore);
    } else if (strstr((const char *)data, "HELP")) {
        ESP_LOGI(TO_PC_LOG_TAG, "Possible commands\n"
                                "START_OPER - starting operational (experiment) state\n"
                                "RESTART - software restart the controller (also sofware reboots motor)\n"
                                "START_TESTS - hardware sensor tests\n"
                                "MOTOR_FIND - try to find motor id and start communicating\n"
                                "MOTOR_INFO - requests info from motor (voltage, temperature, errors)\n"
                                "MOTOR_RESET - software resetting the motor\n"
                                "MOTOR_INIT - centering the carriage, initializing true encoder position and activating dead zones\n"
                                "MOTOR_MODE - switch contolling modes\n"
                                "DATA_SEND_MODE - cofigure sending joint info sending\n"
                                "ENCODER_POSITION - request single position\n"
                                "ENCODER_ANGLE - request single angle");
    } else if (strstr((const char *)data, "START_TESTS")) {
        uart_state = uart_test_button;
        xSemaphoreGive(test_sync_semaphore);
    } else if (strstr((const char *)data, "HELLO")) {
        ESP_LOGI(TO_PC_LOG_TAG, "hi there");
    }

    else if (strstr((const char *)data, "MOTOR_MODE")) {
        ESP_LOGI(TO_PC_LOG_TAG, "Possible variants\nTORQUE, SPEED, POSITION");
        uart_state = uart_set_motor_drive_mode;
    } else if (strstr((const char *)data, "MOTOR_FIND")) {
        find_motor_id();
    } else if (strstr((const char *)data, "MOTOR_RESET") || strstr((const char *)data, "1000003")) {
        send_motor_command_system_reset();
    } else if (strstr((const char *)data, "MOTOR_INFO")) {
        send_motor_command_status();
    } else if (strstr((const char *)data, "MOTOR_INIT")) {
        motor_init_function();
    }

    else if (strstr((const char *)data, "DATA_SEND_MODE")) {
        ESP_LOGI(TO_PC_LOG_TAG, "Conifgure data sending protocols, write delay (ms)");
        uart_state = uart_set_sender_delay;
    } 

    else if (strstr((const char *)data, "ENCODER_POSITION")) {
        ESP_LOGI(TO_PC_LOG_TAG, "Position is %f", current_encoder_position);
    } else if (strstr((const char *)data, "ENCODER_ANGLE")) {
        ESP_LOGI(TO_PC_LOG_TAG, "Angle is %f", current_encoder_angle);
    } 
    
    else {
        ESP_LOGE(ERROR_LOG_TAG, "Undefined behaviour");
    }

}

/**
 * @brief UART handler to set control mode.
 *
 * Parses TORQUE, SPEED, or POSITION strings and logs selected mode.
 * Then returns to ready state.
 *
 * @param data Null-terminated mode string.
 */
void uart_get_mode(char* data){ // TODO
    if (strstr((const char *)data, "TORQUE")){
        motor_operate = motor_operate_by_torque;
        ESP_LOGI(TO_PC_LOG_TAG, "Motor is configured to torque control mode");
    } else if (strstr((const char *)data, "SPEED")){
        motor_operate = motor_operate_by_speed;
        ESP_LOGI(TO_PC_LOG_TAG, "Motor is configured to speed control mode");
    } else if (strstr((const char *)data, "POSITION")){
        if (not_initiazatied_yet) {
            ESP_LOGE(ERROR_LOG_TAG, "Motor should be initialized for this behaviour");
            return;
        }
        motor_operate = motor_operate_by_position;
        ESP_LOGI(TO_PC_LOG_TAG, "Motor is configured to position control mode");
    } else {
        ESP_LOGE(ERROR_LOG_TAG, "Undefined behaviour");
    }
    uart_state = uart_ready_state;
}

/**
 * @brief UART handler to configure sensor info delay.
 *
 * Converts ASCII data to integer and updates delay for sensor info timer.
 *
 * @param data Null-terminated numeric string.
 */
void uart_get_delay(char* data){
    sensor_info_sender_delay = atoi(data);
    ESP_LOGI(WRITE_TAG, "Delay is configured to %d", sensor_info_sender_delay);
    uart_state = uart_ready_state;
}

/**
 * @brief UART handler for OPERATIONAL state control packets.
 *
 * Parses numeric packets for torque commands or special commands
 * (stop, ready, info, reset), with safety checks on encoder limits.
 *
 * @param data Null-terminated ASCII packet string.
 */
void uart_oper_state(char* data)
{
    #ifdef DEBUG
    printf("i recieved");
    printf(data);
    printf("in oper state\n");
    #endif
    uart_recieved_packet = atoi(data);

    if (!system_in_safe_state) {
        return;
    }

    if (uart_recieved_packet < INT16_MIN || uart_recieved_packet > INT16_MAX){
        if (uart_recieved_packet == MOTOR_STOP_COMMAND){
            send_motor_command_stop();
            return;
        } else if (uart_recieved_packet == READY_STATE_COMMAND) {
            send_motor_command_stop();
            ESP_LOGI(MAIN_LOG_TAG, "Starting ready state");
            from_operational_to_ready();
            return;
        } else if (uart_recieved_packet == MOTOR_INFO_COMMMAND) {
            send_motor_command_status();
            return;
        } else if (uart_recieved_packet == RESET_COMMAND) {
            from_operational_to_ready();
            xSemaphoreGive(restart_command_semaphore);
            return;
        }
        uart_recieved_packet = 0;
    }
    xSemaphoreGive(motor_command_semaphore);       
}

/**
 * @brief FreeRTOS task to publish sensor information.
 *
 * Waits on info_please_sem, computes linear and angular velocities,
 * and prints position, velocities, and angle if safe.
 */
void sensor_info_sender_task(){
    while (1){
        xSemaphoreTake(sensor_data_request_semaphore, portMAX_DELAY);
        double linear_velocity = (current_encoder_position - previous_encoder_position) * 1000 / sensor_timer_delay;
        previous_encoder_position = current_encoder_position;
        
        double angular_velocity = ((int) (current_encoder_angle - previous_encoder_angle) % 360) * 1000 / sensor_timer_delay;
        if (current_encoder_angle > 180){
            current_encoder_angle = -360 + current_encoder_angle;
        } else if (current_encoder_angle < -180){
            current_encoder_angle = 360 + current_encoder_angle;
        } 
        
        previous_encoder_angle = current_encoder_angle;
        
        

        

        if (system_in_safe_state){
            printf("%.3f %.3f %.3f %.3f\n", current_encoder_angle, angular_velocity, current_encoder_position, linear_velocity);
        }
    }
}

/**
 * @brief Timer callback to trigger sensor info task.
 *
 * Gives the info_please_sem semaphore to schedule data reporting.
 */
void sensor_info_sender_timer(){
   xSemaphoreGive(info_please_sem);
}


/*-------------------------- Sensors --------------------------*/

/**
 * @brief Interrupt Service Routine for linear encoder channel.
 *
 * Called on any edge of the linear encoder's first channel.
 * Compares states of both encoder channels to determine movement direction
 * and increments or decrements the linear encoder position counter.
 *
 * @param arg Unused argument.
 */
static void IRAM_ATTR enc_linear_isr_handler(void* arg)
{
    current_encoder_position += POSITION_STEP_SIZE * ((gpio_get_level(LINEAR_ENCODER_GPIO_A) != gpio_get_level(LINEAR_ENCODER_GPIO_B)) ? 1 : -1);
}

/**
 * @brief Interrupt Service Routine for angular encoder channel B.
 *
 * Called on the rising edge of angular encoder channel B.
 * Updates the angular position by a fixed step size depending on
 * the state of channel A to determine direction.
 *
 * @param arg Unused argument.
 */
static void IRAM_ATTR enc_angular_change_isr_handler(void* arg)
{
    current_encoder_angle += ANGLE_STEP_SIZE * (1 - gpio_get_level(ANGULAR_ENCODER_GPIO_A) * 2);
}

/**
 * @brief Interrupt Service Routine for angular encoder zero marker.
 *
 * Called when the zero position marker is detected on channel C.
 * Resets the angular encoder position to zero for calibration.
 *
 * @param arg Unused argument.
 */
static void IRAM_ATTR enc_angular_zero_isr_handler(void* arg)
{
    current_encoder_angle = 95.626;
}

/**
 * @brief Interrupt Service Routine for button press.
 *
 * If initialization is active, gives semaphore from ISR context
 * to signal that the button was pressed.
 *
 * @param arg Unused argument.
 */
static void IRAM_ATTR btn_isr_handler(void* arg)
{
    if (initialization_in_progress){
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(button_press_semaphore, &xHigherPriorityTaskWoken);
    }
}

/**
 * @brief Initialize GPIO pins and configure interrupts.
 *
 * Configures all required GPIOs for the linear and angular encoders
 * and the user button. Sets pin modes, pull-downs, and interrupt types.
 * Installs ISR service and attaches handlers to GPIO pins.
 */
static void gpio_init_setup()
{

    // linear
    gpio_config_t io_conf_1;
    io_conf_1.pull_down_en = 1;
    io_conf_1.mode = GPIO_MODE_INPUT;
    io_conf_1.intr_type = GPIO_INTR_ANYEDGE;
    io_conf_1.pin_bit_mask = GPIO_PIN_MASK(LINEAR_ENCODER_GPIO_A);
    gpio_config(&io_conf_1);

    gpio_config_t io_conf_2;
    io_conf_2.pull_down_en = 1;
    io_conf_2.mode = GPIO_MODE_INPUT;
    io_conf_2.intr_type = GPIO_INTR_DISABLE;
    io_conf_2.pin_bit_mask = GPIO_PIN_MASK(LINEAR_ENCODER_GPIO_B);
    gpio_config(&io_conf_2);

    // angular
    gpio_config_t io_conf_3;
    io_conf_3.pull_down_en = 1;
    io_conf_3.mode = GPIO_MODE_INPUT;
    io_conf_3.pin_bit_mask = GPIO_PIN_MASK(ANGULAR_ENCODER_GPIO_A);
    io_conf_3.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf_3);

    gpio_config_t io_conf_4;
    io_conf_4.pull_down_en = 1;
    io_conf_4.mode = GPIO_MODE_INPUT;
    io_conf_4.pin_bit_mask = GPIO_PIN_MASK(ANGULAR_ENCODER_GPIO_B);
    io_conf_4.intr_type = GPIO_INTR_POSEDGE;
    gpio_config(&io_conf_4);

    gpio_config_t io_conf_5;
    io_conf_5.pull_down_en = 1;
    io_conf_5.mode = GPIO_MODE_INPUT;
    io_conf_5.pin_bit_mask = GPIO_PIN_MASK(ANGULAR_ENCODER_GPIO_C);
    io_conf_5.intr_type = GPIO_INTR_POSEDGE;
    gpio_config(&io_conf_5);

    // button 
    gpio_config_t io_conf_6;
    io_conf_6.pull_down_en = 1;
    io_conf_6.mode = GPIO_MODE_INPUT;
    io_conf_6.pin_bit_mask = GPIO_PIN_MASK(END_BUTTON_GPIO);
    io_conf_6.intr_type = GPIO_INTR_POSEDGE;
    gpio_config(&io_conf_6);

    // interrupts 
    gpio_install_isr_service(DEFAULT_INTERRUPT_FLAGS);
    gpio_isr_handler_add(LINEAR_ENCODER_GPIO_A, linear_encoder_isr, 0);
    gpio_isr_handler_add(ANGULAR_ENCODER_GPIO_B, angular_encoder_isr, 0);
    gpio_isr_handler_add(ANGULAR_ENCODER_GPIO_C, angular_zero_isr, 0);
    gpio_isr_handler_add(END_BUTTON_GPIO, button_press_isr, 0);
    
}


/* ---------------------------- CPU ------------------------- */
/**
 * @brief Configure CPU power management settings.
 *
 * Sets the minimum and maximum CPU clock frequencies to maximum value,
 * and disables light sleep to ensure full performance during operation.
 */
void cpu_setup()
{
    esp_pm_config_t pm_config = {
        .max_freq_mhz = 240,
        .min_freq_mhz = 240,
        .light_sleep_enable = false
    };
    esp_pm_configure(&pm_config);
}


/* ---------------------------- Tests ------------------------- */

/**
 * @brief Runs a full sensor and button test sequence.
 *
 * Runs in a separate FreeRTOS task. Guides the user through manual tests
 * for the button, linear encoder, and angular encoder. Waits for user
 * confirmation via UART after each step and logs results. Loops endlessly.
 */
void sensor_tests()
{
    while (1) {
        hardware_tests_passed  = true;
        xSemaphoreTake(test_sync_semaphore, portMAX_DELAY);
        ESP_LOGI(TO_PC_LOG_TAG, "Press the button");
        initialization_in_progress = true;
        xSemaphoreTake(button_press_semaphore, 0);
        xSemaphoreTake(button_press_semaphore, portMAX_DELAY);
        initialization_in_progress = false;
        ESP_LOGI(TO_PC_LOG_TAG, "Did YOU press the button? y/n");

        xSemaphoreTake(test_sync_semaphore, portMAX_DELAY);
        int16_t start_encoder_position = current_encoder_position;
        ESP_LOGI(TO_PC_LOG_TAG, "Now move a carriage");
        while (start_encoder_position - current_encoder_position < 1000 && start_encoder_position - current_encoder_position > -1000){vTaskDelay(pdMS_TO_TICKS(100));}
        ESP_LOGI(TO_PC_LOG_TAG, "Did YOU move the carriage? y/n");

        xSemaphoreTake(test_sync_semaphore, portMAX_DELAY);
        double start_encoder_angle = current_encoder_angle;
        ESP_LOGI(TO_PC_LOG_TAG, "Swing a pendulum");
        while (start_encoder_angle - current_encoder_angle < 30 && start_encoder_angle - current_encoder_angle > -30){vTaskDelay(pdMS_TO_TICKS(100));}
        ESP_LOGI(TO_PC_LOG_TAG, "Did YOU swing the pendulum? y/n");
        xSemaphoreTake(test_sync_semaphore, portMAX_DELAY);

        if (hardware_tests_passed ){
            ESP_LOGI(TO_PC_LOG_TAG, "Tests are finished");
        } else {
            ESP_LOGE(ERROR_LOG_TAG, "Malfunctions found, robot needs a service!");
        }
        
    }
}

/**
 * @brief Handles UART input to verify button test result.
 *
 * Parses user input string to determine whether the button works.
 * Logs result, updates test status, and advances test sequence.
 *
 * @param data Pointer to user input string (yes/no).
 */
void uart_test_btn_state(char* data)
{
    if (strstr((const char *)data, "YES") || strstr((const char *)data, "yes") || strstr((const char *)data, "y")) {
        ESP_LOGI(TO_PC_LOG_TAG, "Button works properly");
    } else if (strstr((const char *)data, "NO") || strstr((const char *)data, "no") || strstr((const char *)data, "n")) {
        ESP_LOGE(ERROR_LOG_TAG, "Button doesnt work");
        hardware_tests_passed  = false;
    } else {
        ESP_LOGE(ERROR_LOG_TAG, "Undefined behaviour");
        return;
    }
    uart_state = uart_test_encoder;
    xSemaphoreGive(test_sync_semaphore);
}

/**
 * @brief Handles UART input to verify linear encoder test result.
 *
 * Parses user input string to determine whether the linear encoder works.
 * Logs result, updates test status, and advances test sequence.
 *
 * @param data Pointer to user input string (yes/no).
 */
void uart_test_encoder_state(char* data)
{
    if (strstr((const char *)data, "YES") || strstr((const char *)data, "yes") || strstr((const char *)data, "y")){
        ESP_LOGI(TO_PC_LOG_TAG, "Encoder works properly");
    } else if (strstr((const char *)data, "NO") || strstr((const char *)data, "no") || strstr((const char *)data, "n")) {
        ESP_LOGE(ERROR_LOG_TAG, "Encoder doesnt work");
        hardware_tests_passed  = false;
    } else {
        ESP_LOGE(ERROR_LOG_TAG, "Undefined behaviour");
        return;
    }
    uart_state = uart_test_angle;
    xSemaphoreGive(test_sync_semaphore);
}

/**
 * @brief Handles UART input to verify angular encoder test result.
 *
 * Parses user input string to determine whether the angular encoder works.
 * Logs result, updates test status, and finishes test sequence.
 *
 * @param data Pointer to user input string (yes/no).
 */
void uart_test_angle_state(char* data)
{
    if (strstr((const char *)data, "YES") || strstr((const char *)data, "yes") || strstr((const char *)data, "y")){
        ESP_LOGI(TO_PC_LOG_TAG, "Angle sensor works properly");
    } else if (strstr((const char *)data, "NO") || strstr((const char *)data, "no") || strstr((const char *)data, "n")) {
        ESP_LOGE(ERROR_LOG_TAG, "Angle sensor doesnt work");
        hardware_tests_passed  = false;
    } else {
        ESP_LOGE(ERROR_LOG_TAG, "Undefined behaviour");
        return;
    }
    uart_state = uart_ready_state;
    xSemaphoreGive(test_sync_semaphore);
}


/* --------------------------- Main -------------------------- */

/**
 * @brief Main entry point for the application.
 *
 * Initializes CPU, GPIO, UART, CAN bus, tasks, and semaphores.
 * Starts all system tasks and waits for completion. After tasks finish,
 * cleans up all resources and restarts the device.
 */
void app_main(void)
{
    configure_cpu();

    //Create tasks, queues, and semaphores
    restart_command_semaphore = xSemaphoreCreateBinary();
    motor_command_semaphore = xSemaphoreCreateBinary();
    button_press_semaphore = xSemaphoreCreateBinary();
    initialization_start_semaphore = xSemaphoreCreateBinary();
    initialization_done_semaphore = xSemaphoreCreateBinary();
    sensor_data_request_semaphore = xSemaphoreCreateBinary();
    test_sync_semaphore = xSemaphoreCreateBinary();

    configure_gpio();
    uart_configure();

    ESP_ERROR_CHECK(twai_driver_install(&can_general_config , &can_timing_config , &can_filter_config ));
    ESP_ERROR_CHECK(twai_start());

    xTaskCreate(motor_control_task,         "motor_control",            4096,   NULL,   CTRL_TSK_PRIORITY,      &motor_control_task_handle);
    xTaskCreate(uart_event_task,            "serial_handler",           4096,   NULL,   SERIAL_TSK_PRIORITY,    &uart_event_task_handle);
    xTaskCreate(motor_self_saver_task,      "motor_self_saver",         4096,   NULL,   SAVER_TSK_PRIORITY,     &safety_monitor_task_handle);
    xTaskCreate(sensor_info_sender_task,    "jonit_info",               4096,   NULL,   SERIAL_TSK_PRIORITY,    &sensor_info_sender_task_handle);
    xTaskCreate(hardware_test_sequence,     "tests",                    4096,   NULL,   TEST_TASK_PRIORITY,     &hardware_test_task_handle);

    ESP_LOGI(TO_PC_LOG_TAG, "Ready to use");

    xSemaphoreTake(restart_command_semaphore, portMAX_DELAY);    

    vTaskDelete(motor_control_task_handle);
    vTaskDelete(uart_event_task_handle);
    vTaskDelete(safety_monitor_task_handle);
    vTaskDelete(sensor_info_sender_task_handle);
    vTaskDelete(hardware_test_task_handle);

    ESP_ERROR_CHECK(twai_stop());
    ESP_ERROR_CHECK(twai_driver_uninstall());

    vSemaphoreDelete(restart_command_semaphore);
    vSemaphoreDelete(motor_command_semaphore);
    vSemaphoreDelete(button_press_semaphore);
    vSemaphoreDelete(initialization_start_semaphore);
    vSemaphoreDelete(initialization_done_semaphore);
    vSemaphoreDelete(test_sync_semaphore);

    ESP_LOGI(MAIN_LOG_TAG, "Restarting...");
    esp_restart();
}