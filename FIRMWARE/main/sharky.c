/* MCPWM basic config example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
 * This example will show you how to use each submodule of MCPWM unit.
 * The example can't be used without modifying the code first.
 * Edit the macros at the top of mcpwm_example_basic_config.c to enable/disable the submodules which are used in the example.
 */

/*  Registro de Fallas

 *  Error en acelerador, voltaje de salida del acelerador sin oprimir: 3.2V
 *  Voltaje de regulación en la segunda fuente de 18V -> Concetar resistencia de consumo en la salida de 100 ohms
*/

// ~|* Include Section *|~

// Include FreeRTOS general purpose
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

// Include for MCPWM
#include <stdio.h>
#include "string.h"
#include "freertos/queue.h"
#include "esp_attr.h"
#include "soc/rtc.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

// Include for ADC
#include <stdlib.h>
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

// Include for UART comm -> NEXTION
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "soc/uart_struct.h"


// ~|* Define Section *|~

// Define for ADC
#define DEFAULT_VREF 1100 //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 64  //Multisampling

static esp_adc_cal_characteristics_t *adc_chars;
#if CONFIG_IDF_TARGET_ESP32
static const adc_channel_t channel_Throttle = ADC_CHANNEL_6;    //THROTTLE - GPIO34 if ADC1, GPIO14 if ADC2
static const adc_channel_t channel_Current = ADC_CHANNEL_7;     //CURRENT SENSOR - GPIO35 if ADC1, GPIO27 if ADC2
static const adc_channel_t channel_Temperature = ADC_CHANNEL_4; //TEMPERATURE SENSOR - GPIO32 if ADC1, GPIO27 if ADC2
static const adc_channel_t channel_Voltage = ADC_CHANNEL_5;     //VOLTAGE SENSOR - GPIO33 if ADC1, GPIO27 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
#elif CONFIG_IDF_TARGET_ESP32S2
static const adc_channel_t channel_Throttle = ADC_CHANNEL_6;    //THROTTLE - GPIO34 if ADC1, GPIO14 if ADC2
static const adc_channel_t channel_Current = ADC_CHANNEL_7;     //CURRENT SENSOR - GPIO35 if ADC1, GPIO27 if ADC2
static const adc_channel_t channel_Temperature = ADC_CHANNEL_4; //TEMPERATURE SENSOR - GPIO32 if ADC1, GPIO27 if ADC2
static const adc_channel_t channel_Voltage = ADC_CHANNEL_5;     //VOLTAGE SENSOR - GPIO33 if ADC1, GPIO27 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_13;
#endif
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

xQueueHandle cap_queue_Throttle;
xQueueHandle cap_queue_Current;
xQueueHandle cap_queue_Temperature;
xQueueHandle cap_queue_Voltage;
xQueueHandle cap_queue_Real_Temperature;
xQueueHandle cap_queue_DutyCycle;
xQueueHandle cap_queue_Real_BattPorcentage;
xQueueHandle cap_queue_Real_BattVolts;
xQueueHandle cap_queue_Real_Current;

// Define for UART - NEXTION
static const int RX_BUF_SIZE = (1024 * 2);
static const char *TX_TASK_TAG = "TX_TASK";
static const char *RX_TASK_TAG = "RX_TASK";
static const char *ESP_SOFT_RESET = "espreset";

#define TXD_PIN 17
#define RXD_PIN 16
#define nUART (UART_NUM_2)

//Define for MCPWM
#define INITIAL_DUTY 10.0 //initial duty cycle is 10.0%
#define MCPWM_GPIO_INIT 0 //select which function to use to initialize gpio signals

#define GPIO_HALL_TEST_SIGNAL 0    //Make this 1 to enable generation of hall sensors test signal on GPIO13, 12, 14
#define CHANGE_DUTY_CONTINUOUSLY 0 //Make this 1 to change duty continuously

#define CAP_SIG_NUM 3       //three capture signals from HALL-A, HALL-B, HALL-C
#define CAP0_INT_EN BIT(27) //Capture 0 interrupt bit
#define CAP1_INT_EN BIT(28) //Capture 1 interrupt bit
#define CAP2_INT_EN BIT(29) //Capture 2 interrupt bit

#define GPIO_PWM0A_OUT 15 //Set GPIO 19 as PWM0A
#define GPIO_PWM0B_OUT 04 //Set GPIO 18 as PWM0B
#define GPIO_PWM1A_OUT 05 //Set GPIO 17 as PWM1A
#define GPIO_PWM1B_OUT 18 //Set GPIO 16 as PWM1B
#define GPIO_PWM2A_OUT 19 //Set GPIO 15 as PWM2A
#define GPIO_PWM2B_OUT 21 //Set GPIO 14 as PWM2B
// #define GPIO_PWM1A_OUT 16   //Set GPIO 17 as PWM1A
// #define GPIO_PWM1B_OUT 17   //Set GPIO 16 as PWM1B
// #define GPIO_PWM2A_OUT 05   //Set GPIO 15 as PWM2A
// #define GPIO_PWM2B_OUT 18   //Set GPIO 14 as PWM2B
#define GPIO_CAP0_IN 25 //Set GPIO 23 as  CAP0
#define GPIO_CAP1_IN 26 //Set GPIO 25 as  CAP1
#define GPIO_CAP2_IN 27 //Set GPIO 26 as  CAP2

#define GPIO_BREAK 22 //Set GPIO 26 as  CAP2

typedef struct
{
    uint32_t capture_signal;
    mcpwm_capture_signal_t sel_cap_signal;
} capture;

static uint32_t hall_sensor_value = 0;
static uint32_t hall_sensor_previous = 0;
xQueueHandle cap_queue;
static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1};

// ~|* Functions Section *|~

// Functions MCPWM
static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm bldc control gpio...\n");
#if MCPWM_GPIO_INIT
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_PWM1A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, GPIO_PWM1B_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2A, GPIO_PWM2A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM2B, GPIO_PWM2B_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, GPIO_CAP0_IN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_1, GPIO_CAP1_IN);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_2, GPIO_CAP2_IN);
#else
    mcpwm_pin_config_t pin_config = {
        .mcpwm0a_out_num = GPIO_PWM0A_OUT,
        .mcpwm0b_out_num = GPIO_PWM0B_OUT,
        .mcpwm1a_out_num = GPIO_PWM1A_OUT,
        .mcpwm1b_out_num = GPIO_PWM1B_OUT,
        .mcpwm2a_out_num = GPIO_PWM2A_OUT,
        .mcpwm2b_out_num = GPIO_PWM2B_OUT,
        .mcpwm_cap0_in_num = GPIO_CAP0_IN,
        .mcpwm_cap1_in_num = GPIO_CAP1_IN,
        .mcpwm_cap2_in_num = GPIO_CAP2_IN,
        .mcpwm_sync0_in_num = -1,  //Not used
        .mcpwm_sync1_in_num = -1,  //Not used
        .mcpwm_sync2_in_num = -1,  //Not used
        .mcpwm_fault0_in_num = -1, //Not used
        .mcpwm_fault1_in_num = -1, //Not used
        .mcpwm_fault2_in_num = -1  //Not used
    };
    mcpwm_set_pin(MCPWM_UNIT_0, &pin_config);
#endif
    gpio_pulldown_en(GPIO_CAP0_IN); //Enable pull down on CAP0   signal
    gpio_pulldown_en(GPIO_CAP1_IN); //Enable pull down on CAP1   signal
    gpio_pulldown_en(GPIO_CAP2_IN); //Enable pull down on CAP2   signal
}
#if GPIO_HALL_TEST_SIGNAL
/**
 * @brief Set gpio 13, 12, 14  as our test signal of hall sensors, that generates high-low waveform continuously
 *        Attach this pins to GPIO 27, 26, 25 respectively for capture unit
 */
static void gpio_test_signal(void *arg)
{
    printf("intializing test signal...\n");
    gpio_config_t gp;
    gp.intr_type = GPIO_INTR_DISABLE;
    gp.mode = GPIO_MODE_OUTPUT;
    gp.pin_bit_mask = GPIO_SEL_13 | GPIO_SEL_12 | GPIO_SEL_14;
    gpio_config(&gp);
    while (1)
    {
        gpio_set_level(GPIO_NUM_13, 1); //Set H1 high
        gpio_set_level(GPIO_NUM_12, 0); //Set H2 low
        gpio_set_level(GPIO_NUM_14, 1); //Set H3 high
        vTaskDelay(1);
        gpio_set_level(GPIO_NUM_14, 0); //Set H3 low
        vTaskDelay(1);
        gpio_set_level(GPIO_NUM_12, 1); //Set H2 high
        vTaskDelay(1);
        gpio_set_level(GPIO_NUM_13, 0); //Set H1 low
        vTaskDelay(1);
        gpio_set_level(GPIO_NUM_14, 1); //Set H3 high
        vTaskDelay(1);
        gpio_set_level(GPIO_NUM_12, 0); //Set H2 high
        vTaskDelay(1);
    }
}
#endif
/**
     * @brief When interrupt occurs, we receive the counter value and display the time between two rising edge
     */
static void disp_captured_signal(void *arg)
{
    uint32_t *current_cap_value = (uint32_t *)malloc(sizeof(CAP_SIG_NUM));
    uint32_t *previous_cap_value = (uint32_t *)malloc(sizeof(CAP_SIG_NUM));
    capture evt;
    while (1)
    {
        xQueueReceive(cap_queue, &evt, portMAX_DELAY);
        if (evt.sel_cap_signal == MCPWM_SELECT_CAP0)
        {
            current_cap_value[0] = evt.capture_signal - previous_cap_value[0];
            previous_cap_value[0] = evt.capture_signal;
            current_cap_value[0] = (current_cap_value[0] / 10000) * (10000000000 / rtc_clk_apb_freq_get());
            //printf("CAP0 : %d us\n", current_cap_value[0]);
        }
        if (evt.sel_cap_signal == MCPWM_SELECT_CAP1)
        {
            current_cap_value[1] = evt.capture_signal - previous_cap_value[1];
            previous_cap_value[1] = evt.capture_signal;
            current_cap_value[1] = (current_cap_value[1] / 10000) * (10000000000 / rtc_clk_apb_freq_get());
            //printf("CAP1 : %d us\n", current_cap_value[1]);
        }
        if (evt.sel_cap_signal == MCPWM_SELECT_CAP2)
        {
            current_cap_value[2] = evt.capture_signal - previous_cap_value[2];
            previous_cap_value[2] = evt.capture_signal;
            current_cap_value[2] = (current_cap_value[2] / 10000) * (10000000000 / rtc_clk_apb_freq_get());
            //printf("CAP2 : %d us\n", current_cap_value[2]);
        }
    }
}
/**
 * @brief this is ISR handler function, here we check for interrupt that triggers rising edge on CAP0 signal and according take action
 */
static void IRAM_ATTR isr_handler(void *arg)
{
    uint32_t mcpwm_intr_status;
    capture evt;
    mcpwm_intr_status = MCPWM[MCPWM_UNIT_0]->int_st.val; //Read interrupt status
    if (mcpwm_intr_status & CAP0_INT_EN)
    {                                                                                         //Check for interrupt on rising edge on CAP0 signal
        evt.capture_signal = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0); //get capture signal counter value
        evt.sel_cap_signal = MCPWM_SELECT_CAP0;
        xQueueSendFromISR(cap_queue, &evt, NULL);
    }
    if (mcpwm_intr_status & CAP1_INT_EN)
    {                                                                                         //Check for interrupt on rising edge on CAP1 signal
        evt.capture_signal = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP1); //get capture signal counter value
        evt.sel_cap_signal = MCPWM_SELECT_CAP1;
        xQueueSendFromISR(cap_queue, &evt, NULL);
    }
    if (mcpwm_intr_status & CAP2_INT_EN)
    {                                                                                         //Check for interrupt on rising edge on CAP2 signal
        evt.capture_signal = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP2); //get capture signal counter value
        evt.sel_cap_signal = MCPWM_SELECT_CAP2;
        xQueueSendFromISR(cap_queue, &evt, NULL);
    }
    MCPWM[MCPWM_UNIT_0]->int_clr.val = mcpwm_intr_status;
}
#if CHANGE_DUTY_CONTINUOUSLY
static void change_duty(void *arg)
{
    int j;
    while (1)
    {
        for (j = 0; j < 18; j++)
        {
            //printf("duty cycle: %d\n", (0 +j*50));
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, (INITIAL_DUTY + j * 5.0));
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, (INITIAL_DUTY + j * 5.0));
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, (INITIAL_DUTY + j * 5.0));
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, (INITIAL_DUTY + j * 5.0));
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, (INITIAL_DUTY + j * 5.0));
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, (INITIAL_DUTY + j * 5.0));
            vTaskDelay(500 / portTICK_RATE_MS);
        }
    }
}
#endif
/**
 * @brief Configure whole MCPWM module for bldc motor control
 */
static void mcpwm_example_bldc_control(void *arg)
{

    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm bldc control...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000; //frequency = 1000Hz
    pwm_config.cmpr_a = 0.0;     //duty cycle of PWMxA = 50.0%
    pwm_config.cmpr_b = 0.0;     //duty cycle of PWMxb = 50.0%
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_1;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); //Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config); //Configure PWM1A & PWM1B with above settings
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config); //Configure PWM2A & PWM2B with above settings
    //3. Capture configuration
    //configure CAP0, CAP1 and CAP2 signal to start capture counter on rising edge
    //we generate a gpio_test_signal of 20ms on GPIO 12 and connect it to one of the capture signal, the disp_captured_function displays the time between rising edge
    //In general practice you can connect Capture  to external signal, measure time between rising edge or falling edge and take action accordingly
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0); //capture signal on rising edge, pulse num = 0 i.e. 800,000,000 counts is equal to one second
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP1, MCPWM_POS_EDGE, 0); //capture signal on rising edge, pulse num = 0 i.e. 800,000,000 counts is equal to one second
    mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP2, MCPWM_POS_EDGE, 0); //capture signal on rising edge, pulse num = 0 i.e. 800,000,000 counts is equal to one second
    //enable interrupt, so each this a rising edge occurs interrupt is triggered
    MCPWM[MCPWM_UNIT_0]->int_ena.val = (CAP0_INT_EN | CAP1_INT_EN | CAP2_INT_EN);  //Enable interrupt on  CAP0, CAP1 and CAP2 signal
    mcpwm_isr_register(MCPWM_UNIT_0, isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL); //Set ISR Handler

    //According to the hall sensor input value take action on PWM0A/0B/1A/1B/2A/2B
    // float vReading_Throttle = 0;
    float vReading_Current = 0;
    float vReading_Temperature = 0;
    float vReading_Voltage = 0;
    float battery_charge = 0.0;
    float battery_voltage = 0.0;
    float temperature = 0.0;
    float current = 0.0;
    float speed = 0.0;
    float distance = 0.0;
    float time_crono = 0.0;
    float power = 0.0;
    float duty_cycle = 0.0;
    int contador_tics = 0;
    while (1)
    {
        // Read Hall Sensors
        hall_sensor_value = (gpio_get_level(GPIO_CAP2_IN) * 1) + (gpio_get_level(GPIO_CAP1_IN) * 2) + (gpio_get_level(GPIO_CAP0_IN) * 4);

        // Recieving ADC measurements
        xQueueReceive(cap_queue_Voltage, &vReading_Voltage, portMAX_DELAY);
        xQueueReceive(cap_queue_Temperature, &vReading_Temperature, portMAX_DELAY);
        xQueueReceive(cap_queue_Current, &vReading_Current, portMAX_DELAY);
        xQueueReceive(cap_queue_Throttle, &duty_cycle, portMAX_DELAY);

        // Calculations
        float x_v = vReading_Voltage;
        float add_volt_mult = -0.03496*x_v + 0.13;
        float y_v = (vReading_Voltage+add_volt_mult);
        float mult_volts_final = 0.2409*pow(y_v,2) - 1.067*y_v + 15.95;
        battery_voltage = (vReading_Voltage+add_volt_mult)*mult_volts_final;
        float x_b = battery_voltage;
        battery_charge = -0.0009234437677319396*pow(x_b,8) + 0.2726640151590356*pow(x_b,7) - 35.17179794706443*pow(x_b,6) + 2588.7822591511595*pow(x_b,5) - 118918.82535921388*pow(x_b,4) + 3491102.021002437*pow(x_b,3) - 63963904.907912984*pow(x_b,2) + 668735305.3250092*x_b -3054510821.0436487;

        temperature = (vReading_Temperature + 0.09) * 18.24;

        float x_c = vReading_Current;
        float add_curr_mult = 0.0016*pow(x_c,9) - 0.03199*pow(x_c,8) + 0.2708*pow(x_c,7) - 1.261*pow(x_c,6) + 3.507*pow(x_c,5) - 5.905*pow(x_c,4) + 5.795*pow(x_c,3) - 2.97*pow(x_c,2) + 0.6309*x_c + 0.043;
        float y_c = (vReading_Current+add_curr_mult);
        float mult_current_final = -0.9147*pow(y_c,9) + 17.04*pow(y_c,8) - 134.2*pow(y_c,7) + 580.2*pow(y_c,6) - 1501*pow(y_c,5) + 2372*pow(y_c,4) - 2235*pow(y_c,3) + 1173*pow(y_c,2) - 298*y_c + 40.03;
        current = (vReading_Current+add_curr_mult)*mult_current_final;


        // Send to UART -> NEXTION
        xQueueSend(cap_queue_Real_BattPorcentage, &battery_charge, portMAX_DELAY);
        xQueueSend(cap_queue_Real_BattVolts, &battery_voltage, portMAX_DELAY);
        xQueueSend(cap_queue_Real_Temperature, &temperature, portMAX_DELAY);
        xQueueSend(cap_queue_Real_Current, &current, portMAX_DELAY);
        xQueueSend(cap_queue_DutyCycle, &duty_cycle, portMAX_DELAY);

        // Log variables
        printf("ADC Voltage [V]\t: %f\n", vReading_Voltage);
        printf("Bat Voltage [V]\t: %f\n", battery_voltage);
        printf("Temperature [V]\t: %f\n", vReading_Temperature);
        printf("Temperature [C]\t: %f\n", temperature);
        printf("Current [V]\t: %f\n", current);
        printf("Tics [#]\t: %d\n", contador_tics);

        // printf("Throttle [V]\t: %f\n", vReading_Throttle);
        printf("duty_cycle\t: %f\n", duty_cycle);
        printf("Hall Sensors\t: %d %d %d\n", gpio_get_level(GPIO_CAP0_IN), gpio_get_level(GPIO_CAP1_IN), gpio_get_level(GPIO_CAP2_IN));

        if(hall_sensor_previous!=hall_sensor_value){
            contador_tics++;
        }
        if (gpio_get_level(GPIO_BREAK) == 1)
        {
            printf("hall_sen_val\t: %d\n", hall_sensor_value);
            mcpwm_config_t pwm_config;
            pwm_config.frequency = 1000; //frequency = 1000Hz
            pwm_config.cmpr_a = duty_cycle;
            pwm_config.cmpr_b = 0;
            pwm_config.counter_mode = MCPWM_UP_COUNTER;
            if (hall_sensor_value == 2)
            {
                printf("Condicion\t: %d\n", hall_sensor_value);
                mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A);
                mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B);
                mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
                mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
                //MCPWMXA to duty mode 1 and MCPWMXB to duty mode 0 or vice versa will generate MCPWM compliment signal of each other, there are also other ways to do it

                mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);                             //Configure PWM0A & PWM0B with above settings
                mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //Set PWM0A to duty mode one
                mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //Set PWM0B back to duty mode zero
                mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_BYPASS_FED, 100, 100);   //Deadtime of 10us
            }
            if (hall_sensor_value == 6)
            {
                printf("Condicion\t: %d\n", hall_sensor_value);
                mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
                mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
                mcpwm_deadtime_disable(MCPWM_UNIT_0, MCPWM_TIMER_0);
                mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
                mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
                //MCPWMXA to duty mode 1 and MCPWMXB to duty mode 0 or vice versa will generate MCPWM compliment signal of each other, there are also other ways to do it
                mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);                             //Configure PWM0A & PWM0B with above settings
                mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //Set PWM2A to duty mode one
                mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //Set PWM2B back to duty mode zero
                mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_BYPASS_FED, 100, 100);   //Deadtime of 10us
            }
            if (hall_sensor_value == 4)
            {
                printf("Condicion\t: %d\n", hall_sensor_value);
                mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
                mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
                mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
                mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
                //MCPWMXA to duty mode 1 and MCPWMXB to duty mode 0 or vice versa will generate MCPWM compliment signal of each other, there are also other ways to do it
                mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);                             //Configure PWM0A & PWM0B with above settings
                mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //Set PWM2A to duty mode one
                mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //Set PWM2B back to duty mode zero
                mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_BYPASS_FED, 100, 100);   //Deadtime of 10us
            }
            if (hall_sensor_value == 5)
            {
                printf("Condicion\t: %d\n", hall_sensor_value);
                mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A);
                mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B);
                mcpwm_deadtime_disable(MCPWM_UNIT_0, MCPWM_TIMER_2);
                mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
                mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
                //MCPWMXA to duty mode 1 and MCPWMXB to duty mode 0 or vice versa will generate MCPWM compliment signal of each other, there are also other ways to do it
                mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);                             //Configure PWM0A & PWM0B with above settings
                mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //Set PWM1A to duty mode one
                mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //Set PWM1B back to duty mode zero
                mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_BYPASS_FED, 100, 100);   //Deadtime of 10uss
            }
            if (hall_sensor_value == 1)
            {
                printf("Condicion\t: %d\n", hall_sensor_value);
                mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
                mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B);
                mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A);
                mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B);
                //MCPWMXA to duty mode 1 and MCPWMXB to duty mode 0 or vice versa will generate MCPWM compliment signal of each other, there are also other ways to do it
                mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);                             //Configure PWM0A & PWM0B with above settings
                mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //Set PWM1A to duty mode one
                mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //Set PWM1B back to duty mode zero
                mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_BYPASS_FED, 100, 100);   //Deadtime of 10uss
            }
            if (hall_sensor_value == 3)
            {
                printf("Condicion\t: %d\n", hall_sensor_value);
                mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
                mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B);
                mcpwm_deadtime_disable(MCPWM_UNIT_0, MCPWM_TIMER_1);
                mcpwm_set_signal_low(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_A);
                mcpwm_set_signal_high(MCPWM_UNIT_0, MCPWM_TIMER_2, MCPWM_OPR_B);
                //MCPWMXA to duty mode 1 and MCPWMXB to duty mode 0 or vice versa will generate MCPWM compliment signal of each other, there are also other ways to do it
                mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);                             //Configure PWM0A & PWM0B with above settings
                mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //Set PWM0A to duty mode one
                mcpwm_set_duty_type(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MCPWM_DUTY_MODE_0); //Set PWM0B back to duty mode zero
                mcpwm_deadtime_enable(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_BYPASS_FED, 100, 100);   //Deadtime of 10us
            }
            if (hall_sensor_value > 0 && hall_sensor_value < 7)
            {
                fprintf(stdout, "\33[2K\033[9A");
            }
            else
            {
                mcpwm_config_t pwm_config;
                pwm_config.frequency = 1000; //frequency = 1000Hz
                pwm_config.cmpr_a = 0.0;     //duty cycle of PWMxA = 50.0%
                pwm_config.cmpr_b = 0.0;     //duty cycle of PWMxb = 50.0%
                pwm_config.counter_mode = MCPWM_UP_COUNTER;
                pwm_config.duty_mode = MCPWM_DUTY_MODE_1;
                mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); //Configure PWM0A & PWM0B with above settings
                mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config); //Configure PWM1A & PWM1B with above settings
                mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config); //Configure PWM2A & PWM2B with above settings
                //3. Capture configuration
                //configure CAP0, CAP1 and CAP2 signal to start capture counter on rising edge
                //we generate a gpio_test_signal of 20ms on GPIO 12 and connect it to one of the capture signal, the disp_captured_function displays the time between rising edge
                //In general practice you can connect Capture  to external signal, measure time between rising edge or falling edge and take action accordingly
                mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0); //capture signal on rising edge, pulse num = 0 i.e. 800,000,000 counts is equal to one second
                mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP1, MCPWM_POS_EDGE, 0); //capture signal on rising edge, pulse num = 0 i.e. 800,000,000 counts is equal to one second
                mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP2, MCPWM_POS_EDGE, 0); //capture signal on rising edge, pulse num = 0 i.e. 800,000,000 counts is equal to one second
                fprintf(stdout, "\33[2K\033[8A");
            }
            hall_sensor_previous = hall_sensor_value;
        }
        else
        {
            printf("FRENADO!\n");
            fprintf(stdout, "\33[2K\033[6A");
            mcpwm_config_t pwm_config;
            pwm_config.frequency = 1000; //frequency = 1000Hz
            pwm_config.cmpr_a = 0.0;     //duty cycle of PWMxA = 50.0%
            pwm_config.cmpr_b = 0.0;     //duty cycle of PWMxb = 50.0%
            pwm_config.counter_mode = MCPWM_UP_COUNTER;
            pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
            mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config); //Configure PWM0A & PWM0B with above settings
            mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config); //Configure PWM1A & PWM1B with above settings
            mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config); //Configure PWM2A & PWM2B with above settings
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Functions for NEXTION - UART
void initNextion()
{

    // Setup of UART 1
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(nUART, &uart_config);
    uart_set_pin(nUART, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // We won't use a buffer for sending data.
    uart_driver_install(nUART, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
}

int sendData(const char *logName, const char *data)
{
    const int len = strlen(data);
    const int txBytes = uart_write_bytes(nUART, (const char *)data, len);
    // ESP_LOGI(logName, "Wrote '%s' of %d bytes",data, txBytes);
    return txBytes;
}

static void tx_task()
{
    esp_log_level_set(TX_TASK_TAG, ESP_LOG_INFO);
    float avg_battery_charge = 0.0;
    float avg_battery_voltage = 0.0;
    float avg_temperature = 0.0;
    float avg_current = 0.0;
    int counter_iterations = 0;
    float last_battery_charge = 0.0;
    float last_battery_voltage = 0.0;
    float last_temperature = 0.0;
    float last_current = 0.0;
    while (1)
    {
        // OK so here is where you can play around and change values or add text and/or numeric fields to the Nextion and update them
        // sendData(TX_TASK_TAG, "\x55");		//example text value - Outside temp
        float battery_charge = 0.0;
        float battery_voltage = 0.0;
        float temperature = 0.0;
        float duty_cycle = 0.0;
        float current = 0.0;

        xQueueReceive(cap_queue_Real_BattPorcentage, &battery_charge, portMAX_DELAY);
        xQueueReceive(cap_queue_Real_BattVolts, &battery_voltage, portMAX_DELAY);
        xQueueReceive(cap_queue_Real_Temperature, &temperature, portMAX_DELAY);
        xQueueReceive(cap_queue_Real_Current, &current, portMAX_DELAY);
        xQueueReceive(cap_queue_DutyCycle, &duty_cycle, portMAX_DELAY);

        // Variables that convert numbers to chars
        char charvalue_batteryCharge[10];
        char charvalue_battery_voltage[10];
        char charvalue_temperature[10];
        char charvalue_current[10];
        // char charvalue_speed[10];
        // char charvalue_distance[10];
        // char charvalue_time[10];
        // char charvalue_power[10];
        char charvalue_throttle[10];
        // Here is sign if needed
        // char *tmpSign = (adc_read < 0) ? "-" : "";
        // sprintf (charvalue_batteryCharge, "%s%0.2f", tmpSign, adc_read);

        // Variables that contain the text to Nextion
        char batteryCharge_chars[30];
        char battery_voltage_chars[30];
        char temperature_chars[30];
        char current_chars[30];
        // char speed_chars[30];
        // char distance_chars[30];
        // char time_chars[30];
        // char power_chars[30];
        char throttle_chars[30];
        if(counter_iterations == 0){
            last_battery_charge = battery_charge;
            last_battery_voltage = battery_voltage;
            last_temperature = temperature;
            last_current = current;

            avg_battery_charge += battery_charge;
            avg_battery_voltage += battery_voltage;
            avg_temperature += temperature;
            avg_current += current;
        }else if(counter_iterations<10){
            avg_battery_charge += battery_charge;
            avg_battery_voltage += battery_voltage;
            avg_temperature += temperature;
            avg_current += current;
        }else{
            avg_battery_charge /= (counter_iterations-1);
            avg_battery_voltage /= (counter_iterations-1);
            avg_temperature /= (counter_iterations-1);
            avg_current /= (counter_iterations-1);

            last_battery_charge = avg_battery_charge;
            last_battery_voltage = avg_battery_voltage;
            last_temperature = avg_temperature;
            last_current = avg_current;

            avg_battery_charge = 0.0;
            avg_battery_voltage = 0.0;
            avg_temperature = 0.0;
            avg_current = 0.0;
            counter_iterations = 0;
        }
        counter_iterations++;

        sprintf (charvalue_batteryCharge, "%0.1f", last_battery_charge);
        sprintf(charvalue_battery_voltage, "%0.2f", last_battery_voltage);
        sprintf(charvalue_temperature, "%0.1f", last_temperature);
        sprintf(charvalue_current, "%0.3f", last_current);
        // sprintf(charvalue_speed, "%0.0f", );
        // sprintf(charvalue_distance, "%0.0f", );
        // sprintf(charvalue_time, "%0.0f", );
        // sprintf(charvalue_power, "%0.0f", );
        sprintf(charvalue_throttle, "%0.1f", duty_cycle);

        sprintf (batteryCharge_chars, "bat.txt=\"%s\"\xff\xff\xff", (const char*)charvalue_batteryCharge);
        sprintf(battery_voltage_chars, "batv.txt=\"%s\"\xff\xff\xff", (const char *)charvalue_battery_voltage);
        sprintf(temperature_chars, "temp.txt=\"%s\"\xff\xff\xff", (const char *)charvalue_temperature);
        sprintf(current_chars, "curr.txt=\"%s\"\xff\xff\xff", (const char *)charvalue_current);
        // sprintf (speed_chars, "speed.txt=\"%s\"\xff\xff\xff", (const char*)charvalue_speed);
        // sprintf (distance_chars, "dist.txt=\"%s\"\xff\xff\xff", (const char*)charvalue_distance);
        // sprintf (time_chars, "time.txt=\"%s\"\xff\xff\xff", (const char*)charvalue_time);
        // sprintf (power_chars, "power.txt=\"%s\"\xff\xff\xff", (const char*)charvalue_power);
        sprintf(throttle_chars, "thro.txt=\"%s\"\xff\xff\xff", (const char *)charvalue_throttle);

        sendData(TX_TASK_TAG, batteryCharge_chars); //Porcentage in battery
        sendData(TX_TASK_TAG, battery_voltage_chars); //Voltage in battery
        sendData(TX_TASK_TAG, temperature_chars);  //Temperature in LM35
        sendData(TX_TASK_TAG, current_chars);      //Supply current for motor
        sendData(TX_TASK_TAG, throttle_chars);     //Throttle - DutyCycle
        // vTaskDelay(100 / portTICK_PERIOD_MS); //Transmit every 10 seconds
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void rx_task()
{
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t *data = (uint8_t *)malloc(RX_BUF_SIZE + 1);
    char *dstream = malloc(RX_BUF_SIZE + 1);
    while (1)
    {
        const int rxBytes = uart_read_bytes(nUART, data, RX_BUF_SIZE, 1000 / portTICK_RATE_MS);
        if (rxBytes > 0)
        {
            data[rxBytes] = 0;
            // ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
            // ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
            //
            // if it sees the incoming 'espreset' text then reboot the ESP32.
            snprintf(dstream, RX_BUF_SIZE + 1, "%s", data);
            char *ismatch = strstr(dstream, ESP_SOFT_RESET);
            if (ismatch)
            {
                //esp_restart() is an esp-idf function---check for deprecation
                esp_restart();
            }
        }
    }
    free(data);
    free(dstream);
}

// Functions for ADC
static void check_efuse(void)
{
#if CONFIG_IDF_TARGET_ESP32
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
    {
        printf("eFuse Two Point: Supported\n");
    }
    else
    {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
    {
        printf("eFuse Vref: Supported\n");
    }
    else
    {
        printf("eFuse Vref: NOT supported\n");
    }
#elif CONFIG_IDF_TARGET_ESP32S2
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
    {
        printf("eFuse Two Point: Supported\n");
    }
    else
    {
        printf("Cannot retrieve eFuse Two Point calibration values. Default calibration values will be used.\n");
    }
#else
#error "This example is configured for ESP32/ESP32S2."
#endif
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
    {
        printf("Characterized using Two Point Value\n");
    }
    else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
    {
        printf("Characterized using eFuse Vref\n");
    }
    else
    {
        printf("Characterized using Default Vref\n");
    }
}

/**
         * @brief Configure whole ADC module and print data
         */
static void setup_print_ADC(void *parameter)
{
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1)
    {
        adc1_config_width(width);
        adc1_config_channel_atten(channel_Throttle, atten);
        adc1_config_channel_atten(channel_Current, atten);
        adc1_config_channel_atten(channel_Temperature, atten);
        adc1_config_channel_atten(channel_Voltage, atten);
    }
    else
    {
        adc2_config_channel_atten((adc2_channel_t)channel_Throttle, atten);
        adc2_config_channel_atten((adc2_channel_t)channel_Current, atten);
        adc2_config_channel_atten((adc2_channel_t)channel_Temperature, atten);
        adc2_config_channel_atten((adc2_channel_t)channel_Voltage, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    //Continuously sample ADC1
    while (1)
    {
        uint32_t adc_reading_Throttle = 0;
        uint32_t adc_reading_Current = 0;
        uint32_t adc_reading_Temperature = 0;
        uint32_t adc_reading_Voltage = 0;
        float duty_cycle = 0.0;
        float vReading_Current = 0.0;
        float vReading_Temperature = 0.0;
        float vReading_Voltage = 0.0;
        //Multisampling
        for (int i = 0; i < NO_OF_SAMPLES; i++)
        {
            if (unit == ADC_UNIT_1)
            {
                adc_reading_Throttle += adc1_get_raw((adc1_channel_t)channel_Throttle);
                adc_reading_Current += adc1_get_raw((adc1_channel_t)channel_Current);
                adc_reading_Temperature += adc1_get_raw((adc1_channel_t)channel_Temperature);
                adc_reading_Voltage += adc1_get_raw((adc1_channel_t)channel_Voltage);
            }
            else
            {
                int raw_Throttle;
                int raw_Current;
                int raw_Temperature;
                int raw_Voltage;
                adc2_get_raw((adc2_channel_t)channel_Throttle, width, &raw_Throttle);
                adc2_get_raw((adc2_channel_t)channel_Current, width, &raw_Current);
                adc2_get_raw((adc2_channel_t)channel_Temperature, width, &raw_Temperature);
                adc2_get_raw((adc2_channel_t)channel_Voltage, width, &raw_Voltage);
                adc_reading_Throttle += raw_Throttle;
                adc_reading_Current += raw_Current;
                adc_reading_Temperature += raw_Temperature;
                adc_reading_Voltage += raw_Voltage;
            }
        }
        adc_reading_Throttle /= NO_OF_SAMPLES;
        adc_reading_Current /= NO_OF_SAMPLES;
        adc_reading_Temperature /= NO_OF_SAMPLES;
        adc_reading_Voltage /= NO_OF_SAMPLES;
        // vReading_Throttle = (adc_reading_Throttle*3.3/4095);

        duty_cycle = (adc_reading_Throttle - 950.0) / 31.45;
        if (duty_cycle < 0.0)
        {
            duty_cycle = 0.0;
        }
        else
        {
            duty_cycle = duty_cycle;
        }
        vReading_Current = (adc_reading_Current * 3.3 / 4095);
        vReading_Temperature = (adc_reading_Temperature * 3.3 / 4095);
        vReading_Voltage = (adc_reading_Voltage * 3.3 / 4095);
        //Convert adc_reading to voltage in mV
        // uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        // printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
        xQueueSend(cap_queue_Throttle, &duty_cycle, portMAX_DELAY);
        xQueueSend(cap_queue_Current, &vReading_Current, portMAX_DELAY);
        xQueueSend(cap_queue_Temperature, &vReading_Temperature, portMAX_DELAY);
        xQueueSend(cap_queue_Voltage, &vReading_Voltage, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// General Functions
void app_main(void)
{
    //SETUP MCPWM
    printf("Testing MCPWM...\n");
#if CHANGE_DUTY_CONTINUOUSLY
    xTaskCreate(change_duty, "change_duty", 2048, NULL, 2, NULL);
#endif
    cap_queue = xQueueCreate(1, sizeof(capture));                //comment if you don't want to use capture module
    cap_queue_Throttle = xQueueCreate(1, sizeof(float));         //comment if you don't want to use capture module
    cap_queue_Current = xQueueCreate(1, sizeof(float));          //comment if you don't want to use capture module
    cap_queue_Temperature = xQueueCreate(1, sizeof(float));      //comment if you don't want to use capture module
    cap_queue_Voltage = xQueueCreate(1, sizeof(float));          //comment if you don't want to use capture module
    cap_queue_Real_Temperature = xQueueCreate(1, sizeof(float)); //comment if you don't want to use capture module
    cap_queue_DutyCycle = xQueueCreate(1, sizeof(float));        //comment if you don't want to use capture module
    cap_queue_Real_BattPorcentage = xQueueCreate(1, sizeof(float));   //comment if you don't want to use capture module
    cap_queue_Real_BattVolts = xQueueCreate(1, sizeof(float));   //comment if you don't want to use capture module
    cap_queue_Real_Current = xQueueCreate(1, sizeof(float));     //comment if you don't want to use capture module
#if GPIO_HALL_TEST_SIGNAL
    xTaskCreate(gpio_test_signal, "gpio_test_signal", 2048, NULL, 2, NULL);
#endif
    if (cap_queue == NULL || cap_queue_Throttle == NULL ||
        cap_queue_Current == NULL || cap_queue_Temperature == NULL ||
        cap_queue_Voltage == NULL || cap_queue_Real_Temperature == NULL ||
        cap_queue_DutyCycle == NULL || cap_queue_Real_BattVolts == NULL ||
        cap_queue_Real_Current == NULL || cap_queue_Real_BattPorcentage == NULL)
    {
        printf("Error creating the cap_queue_Real_BattPorcentage or cap_queue_Real_Current or cap_queue or cap_queue_Throttle or cap_queue_Current or cap_queue_Real_Temperature or cap_queue_DutyCycle or cap_queue_Real_BattVolts");
    }

    printf("Init Nextion...\n");
    initNextion();
    xTaskCreate(disp_captured_signal, "mcpwm_config", 4096, NULL, 0, NULL); //comment if you don't want to use capture module
    xTaskCreate(setup_print_ADC, "setup_print_ADC", 4096, NULL, 1, NULL);   //comment if you don't want to use capture module
    // @TODO fix with que
    xTaskCreate(mcpwm_example_bldc_control, "mcpwm_example_a", 4096, NULL, configMAX_PRIORITIES - 1, NULL);
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(tx_task, "uart_tx_task", 1024 * 2, NULL, configMAX_PRIORITIES - 3, NULL);
}
