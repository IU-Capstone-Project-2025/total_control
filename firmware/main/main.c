/**
 * @file main.c
 * @brief Main application entry point and firmware for CartPole controller.
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

#define CTRL_TSK_PRIO           10          /**< Motor control task priority */
#define SERIAL_TSK_PRIO         15          /**< UART and sensor info task priority */
#define SAVER_TSK_PRIO          20          /**< Safety monitor task priority */
#define TEST_TASK_PRIO          1           /**< Hardware test task priority */
#define ESP_INTR_FLAG_DEFAULT   0           /**< Default interrupt flag */
#define TX_GPIO_CAN             22          /**< CAN TX GPIO */
#define RX_GPIO_CAN             21          /**< CAN RX GPIO */
#define ENC_LINEAR_GPIO_1       19          /**< Linear encoder channel A */
#define ENC_LINEAR_GPIO_2       18          /**< Linear encoder channel B */
#define ENC_ANGULAR_GPIO_A      5           /**< Angular encoder channel A */
#define ENC_ANGULAR_GPIO_B      4           /**< Angular encoder channel B */
#define ENC_ANGULAR_GPIO_C      2           /**< Angular encoder index signal */
#define BTN_GPIO                23          /**< User button GPIO */
#define GPIO_PIN_MASK(PIN)      (1ULL<<PIN)
#define ANGLE_STEP_SIZE         0.08789     /**< Angular step [rad] */
#define MAX_ECNODER_DATA        12213       /**< Linear encoder max count */
#define SAFE_REGION             1000        /**< Safe region threshold */
#define INIT_TORQUE             75          /**< Initial torque for motor initialization */
#define REINIT_TORQUE           75          /**< Reinitialization torque for motor */

#define TXD_PIN                 1           /**< UART TX pin */
#define RXD_PIN                 3           /**< UART RX pin */
#define UART_PORT               UART_NUM_0  /**< UART port */
#define BUF_SIZE                128         /**< UART buffer size */
#define BAUD_RATE               921600      /**< UART baud rate */
#define SERIAL_MS_DELAY         20          /**< UART read delay [ms] */

#define ERROR_TAG               "Error"     /**< Error log tag */
#define MAIN_TAG                "Main"      /**< Main log tag */
#define DEBUG_TAG               "Debug"     /**< Debug log tag */
#define SEND_TAG                "Motor/Sending" /**< Motor sending log tag */
#define RECIEVE_TAG             "Motor/Recieving"   /**< Motor receiving log tag */
#define READ_TAG                "PC/Recieving"      /**< PC receiving log tag */
#define WRITE_TAG               "PC/Sending"        /**< PC sending log tag */

#define MOTOR_STOP_COMMAND      1000000 /**< Stop command code */
#define READY_STATE_COMMAND     1000001 /**< Ready state command code */
#define MOTOR_INFO_COMMMAND     1000002 /**< Info request command code */
#define RESET_COMMAND           1000003 /**< Reset command code */

// #define DEBUG 0


/* --------------------- Static variables ------------------ */

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_CAN, RX_GPIO_CAN, TWAI_MODE_NORMAL);

static SemaphoreHandle_t motor_control_task_start_sem;
static SemaphoreHandle_t all_done_sem;
static SemaphoreHandle_t init_start_sem;
static SemaphoreHandle_t init_done_sem;
static SemaphoreHandle_t btn_sem;
static SemaphoreHandle_t move_sem;
static SemaphoreHandle_t smert_sem;
static SemaphoreHandle_t info_please_sem;
static SemaphoreHandle_t test_sem;

static uint32_t motor_id = 0x141;
static int64_t recieved_packet;

volatile int16_t encoder_position;
volatile int16_t encoder_position_prev;
volatile double encoder_angle = -83;
volatile double encoder_angle_prev;
volatile bool init_in_progress = false;
static bool not_inited_yet = true;
static bool in_safe_state = true;

void uart_ready_state(char* data);
void uart_get_delay(char* data);
void uart_get_mode(char* data);
void uart_oper_state(char* data);

void uart_test_btn_state (char* data);
void uart_test_encoder_state (char* data);
void uart_test_angle_state (char* data);

static void (*uart_state)(char*) = uart_ready_state; 

static uint16_t sensor_info_sender_delay = SERIAL_MS_DELAY;
TimerHandle_t sensor_info_sender_timer_handle;
void sensor_info_sender_timer();

static TaskHandle_t mct_handle;
static TaskHandle_t uet_handle;
static TaskHandle_t msst_handle;
static TaskHandle_t sist_handle;
static TaskHandle_t stt_handle;

static bool tested = true;


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

    if (_rx_message->identifier != motor_id) {
        ESP_LOGI(ERROR_TAG, "No reply from motor");
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
        message->data[0], message->data[1], message->data[2], message->data[3], 
        message->data[4], message->data[5], message->data[6], message->data[7]);
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
                                 .identifier = motor_id, 
                                 .data_length_code = 8, 
                                 .data = {d0, d1, d2, d3, d4, d5, d6, d7}};

    #ifdef DEGUB
    twai_output(SEND_TAG, &tx_message);
    if(!twai_request(&tx_message, &rx_message)){
        twai_output(RECIEVE_TAG, &rx_message);
        return false;
    }
    twai_output(RECIEVE_TAG, &rx_message);
    #endif

    #ifndef DEBUG
    if(!twai_request(&tx_message, &rx_message)){
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
                                 .identifier = motor_id, 
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
    motor_request(0x9A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00);
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
    
    motor_request_system_reset();
    motor_request_info();
    
    while(1){
        xSemaphoreTake(move_sem, portMAX_DELAY);
        motor_request_torque(recieved_packet);
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

    int16_t torque = INIT_TORQUE;
    if (!not_inited_yet){
        torque = REINIT_TORQUE;
    }



    motor_request_stop();
    init_in_progress = true;
    ESP_LOGI(MAIN_TAG, "Initializing motor");
    xSemaphoreTake(btn_sem, 0);
    motor_request_torque(-torque);
    
    xSemaphoreTake(btn_sem, pdMS_TO_TICKS(5000));

    #ifdef DEBUG
    ESP_LOGI(MAIN_TAG, "Button CLICKED");
    #endif
    
    motor_request_stop();
    encoder_position = 0;
    motor_request_torque(torque);

    int64_t time = esp_timer_get_time()/1000;
    while (encoder_position < MAX_ECNODER_DATA / 2){
        if (time + 3000 < esp_timer_get_time()/1000){
            motor_request_stop();
            init_in_progress = false;
            ESP_LOGI(ERROR_TAG, "Motor initialization failed!");
            return;
        }
    }
    

    motor_request_stop();
    if (not_inited_yet){
        xSemaphoreGive(init_done_sem);
    }
    init_in_progress = false;
    ESP_LOGI(MAIN_TAG, "Initialize ended");
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
    xSemaphoreTake(init_done_sem, portMAX_DELAY);
    not_inited_yet = false;
    while(1){
        if (!init_in_progress){
            if(encoder_position < SAFE_REGION || encoder_position > MAX_ECNODER_DATA - SAFE_REGION){
                in_safe_state = false;
                ESP_LOGI(ERROR_TAG, "Danger situation, stopping motor");
                motor_init_function();
                in_safe_state = true;
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
    char* data = (char*) malloc(BUF_SIZE);
    memset(data, 0, BUF_SIZE);
    if (!data) {
        ESP_LOGE(ERROR_TAG, "Memory allocation error");
        vTaskDelete(NULL);
    }
    
    while (true)
    {
        const int rxBytes = uart_read_bytes(UART_PORT, data, BUF_SIZE, pdMS_TO_TICKS(SERIAL_MS_DELAY));
        if (rxBytes > 0) {
            uart_state(data);    
        }
    }

    free(data);
    vTaskDelete(NULL);
}


/* --------------------------- UART states -------------------------- */
/**
 * @brief Switches UART handler to operational state.
 *
 * Creates and starts a periodic timer to trigger sensor info reporting.
 */
void rtoo()   // READY to OPERATIONAL
{   
    uart_state = uart_oper_state;
    sensor_info_sender_timer_handle = xTimerCreate("sensor_info_sender_timer", pdMS_TO_TICKS(sensor_info_sender_delay), pdTRUE, 0, sensor_info_sender_timer);

    if(sensor_info_sender_timer_handle != NULL) {
        xTimerStart(sensor_info_sender_timer_handle, 0);
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
    // start button
    if (strstr((const char *)data, "START_OPER")) {
        ESP_LOGI(WRITE_TAG, "Starting operational state");
        rtoo();
    } else if (strstr((const char *)data, "RESTART")) {
        xSemaphoreGive(all_done_sem);
    } else if (strstr((const char *)data, "HELP")) {
        ESP_LOGI(WRITE_TAG, "Possible commands\nSTART_OPER - starting operational (experiment) state \nRESTART - software restart the controller (also sofware reboots motor)\nSTART_TESTS - hardware sensor tests\nMOTOR_FIND - try to find motor id and start communicating\nMOTOR_INFO - requests info from motor (voltage, temperature, errors)\nMOTOR_RESET - software resetting the motor\nMOTOR_INIT - centering the carriage, initializing true encoder position and activating dead zones\nMOTOR_MODE - switch contolling modes\nDATA_SEND_MODE - cofigure sending joint info sending\nENCODER_POSITION - request single position\nENCODER_ANGLE - request single angle");
    } else if (strstr((const char *)data, "START_TESTS")) {
        uart_state = uart_test_btn_state;
        xSemaphoreGive(test_sem);
    }

    // motoro control mode
    else if (strstr((const char *)data, "MOTOR_MODE")) {
        ESP_LOGI(WRITE_TAG, "Possible variants\nTORQUE, SPEED, POSITION");
        uart_state = uart_get_mode;
    } else if (strstr((const char *)data, "MOTOR_FIND")) {
        find_my_id();
    } else if (strstr((const char *)data, "MOTOR_RESET")) {
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
    } 
    
    
    
    else {
        ESP_LOGI(ERROR_TAG, "Undefined behaviour");
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
    recieved_packet = atoi(data);

    if (!in_safe_state) {
        return;
    }

    if (recieved_packet < INT16_MIN || recieved_packet > INT16_MAX){
        if (recieved_packet == MOTOR_STOP_COMMAND){
            motor_request_stop();
            return;
        } else if (recieved_packet == READY_STATE_COMMAND) {
            motor_request_stop();
            ESP_LOGI(MAIN_TAG, "Starting ready state");
            otor();
            return;
        } else if (recieved_packet == MOTOR_INFO_COMMMAND) {
            motor_request_info();
            return;
        } else if (recieved_packet == RESET_COMMAND) {
            otor();
            xSemaphoreGive(all_done_sem);
            return;
        }
        recieved_packet = 0;
    }
    xSemaphoreGive(move_sem);       
}

/**
 * @brief FreeRTOS task to publish sensor information.
 *
 * Waits on info_please_sem, computes linear and angular velocities,
 * and prints position, velocities, and angle if safe.
 */
void sensor_info_sender_task(){
    while (1){
        xSemaphoreTake(info_please_sem, portMAX_DELAY);
        int32_t linear_velocity = (encoder_position - encoder_position_prev) * 1000 / sensor_info_sender_delay;
        double angular_velocity = (encoder_angle - encoder_angle_prev) * 1000 / sensor_info_sender_delay;
        encoder_angle_prev = encoder_angle;
        encoder_position_prev = encoder_position;

        if (in_safe_state){
            printf("%d %ld %f %f\n", encoder_position, linear_velocity, encoder_angle, angular_velocity);
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
    encoder_position += (gpio_get_level(ENC_LINEAR_GPIO_1) != gpio_get_level(ENC_LINEAR_GPIO_2)) ? 1 : -1;
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
    encoder_angle -= ANGLE_STEP_SIZE * (1 - gpio_get_level(ENC_ANGULAR_GPIO_A) * 2);
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
    encoder_angle = 0;
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
    if (init_in_progress){
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(btn_sem, &xHigherPriorityTaskWoken);
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
        tested = true;
        xSemaphoreTake(test_sem, portMAX_DELAY);
        ESP_LOGI(WRITE_TAG, "Press the button");
        init_in_progress = true;
        xSemaphoreTake(btn_sem, 0);
        xSemaphoreTake(btn_sem, portMAX_DELAY);
        init_in_progress = false;
        ESP_LOGI(WRITE_TAG, "Did YOU press the button? y/n");

        xSemaphoreTake(test_sem, portMAX_DELAY);
        int16_t start_encoder_position = encoder_position;
        ESP_LOGI(WRITE_TAG, "Now move a carriage");
        while (start_encoder_position - encoder_position < 1000 && start_encoder_position - encoder_position > -1000){vTaskDelay(pdMS_TO_TICKS(100));}
        ESP_LOGI(WRITE_TAG, "Did YOU move the carriage? y/n");

        xSemaphoreTake(test_sem, portMAX_DELAY);
        double start_encoder_angle = encoder_angle;
        ESP_LOGI(WRITE_TAG, "Swing a pendulum");
        while (start_encoder_angle - encoder_angle < 30 && start_encoder_angle - encoder_angle > -30){vTaskDelay(pdMS_TO_TICKS(100));}
        ESP_LOGI(WRITE_TAG, "Did YOU swing the pendulum? y/n");
        xSemaphoreTake(test_sem, portMAX_DELAY);

        if (tested){
            ESP_LOGI(WRITE_TAG, "Tests are finished");
        } else {
            ESP_LOGI(ERROR_TAG, "Malfunctions found, robot needs a service!");
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
        ESP_LOGI(WRITE_TAG, "Button works properly");
    } else if (strstr((const char *)data, "NO") || strstr((const char *)data, "no") || strstr((const char *)data, "n")) {
        ESP_LOGI(ERROR_TAG, "Button doesnt work");
        tested = false;
    } else {
        ESP_LOGI(ERROR_TAG, "Undefined behaviour");
        return;
    }
    uart_state = uart_test_encoder_state;
    xSemaphoreGive(test_sem);
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
        ESP_LOGI(WRITE_TAG, "Encoder works properly");
    } else if (strstr((const char *)data, "NO") || strstr((const char *)data, "no") || strstr((const char *)data, "n")) {
        ESP_LOGI(ERROR_TAG, "Encoder doesnt work");
        tested = false;
    } else {
        ESP_LOGI(ERROR_TAG, "Undefined behaviour");
        return;
    }
    uart_state = uart_test_angle_state;
    xSemaphoreGive(test_sem);
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
        ESP_LOGI(WRITE_TAG, "Angle sensor works properly");
    } else if (strstr((const char *)data, "NO") || strstr((const char *)data, "no") || strstr((const char *)data, "n")) {
        ESP_LOGI(ERROR_TAG, "Angle sensor doesnt work");
        tested = false;
    } else {
        ESP_LOGI(ERROR_TAG, "Undefined behaviour");
        return;
    }
    uart_state = uart_ready_state;
    xSemaphoreGive(test_sem);
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
    cpu_setup();

    //Create tasks, queues, and semaphores
    all_done_sem = xSemaphoreCreateBinary();
    move_sem = xSemaphoreCreateBinary();
    smert_sem = xSemaphoreCreateBinary();
    btn_sem = xSemaphoreCreateBinary();
    init_start_sem = xSemaphoreCreateBinary();
    init_done_sem = xSemaphoreCreateBinary();
    info_please_sem = xSemaphoreCreateBinary();
    test_sem = xSemaphoreCreateBinary();

    gpio_init_setup();
    uart_init_setup();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());

    xTaskCreate(motor_control_task,         "motor_control",            4096,   NULL,   CTRL_TSK_PRIO,      &mct_handle);
    xTaskCreate(uart_event_task,            "serial_handler",           4096,   NULL,   SERIAL_TSK_PRIO,    &uet_handle);
    xTaskCreate(motor_self_saver_task,      "motor_self_saver",         4096,   NULL,   SAVER_TSK_PRIO,     &msst_handle);
    xTaskCreate(sensor_info_sender_task,    "jonit_info",               4096,   NULL,   SERIAL_TSK_PRIO,    &sist_handle);
    xTaskCreate(sensor_tests,               "tests",                    4096,   NULL,   TEST_TASK_PRIO,     &stt_handle);

    ESP_LOGI(WRITE_TAG, "Ready to use");

    xSemaphoreTake(all_done_sem, portMAX_DELAY);    

    vTaskDelete(mct_handle);
    vTaskDelete(uet_handle);
    vTaskDelete(msst_handle);
    vTaskDelete(sist_handle);
    vTaskDelete(stt_handle);

    ESP_ERROR_CHECK(twai_stop());
    ESP_ERROR_CHECK(twai_driver_uninstall());

    vSemaphoreDelete(all_done_sem);
    vSemaphoreDelete(move_sem);
    vSemaphoreDelete(smert_sem);
    vSemaphoreDelete(btn_sem);
    vSemaphoreDelete(init_start_sem);
    vSemaphoreDelete(init_done_sem);
    vSemaphoreDelete(test_sem);

    ESP_LOGI(MAIN_TAG, "Restarting...");
    esp_restart();
}
