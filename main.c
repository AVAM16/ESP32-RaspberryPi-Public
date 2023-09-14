// The code in this file is based on the examples provided by Espressif Systems
// The backbones of the code are from SIMS-IOT-Devices https://github.com/SIMS-IOT-Devices/FreeRTOS-ESP-IDF-BLE-Server/blob/main/proj3.c
// The project is completed
// In the ESP32 menuconfig Bluetooth must be enabled and the NimBLE stack must be selected
// The goal of this code is to create a BLE server using the NimBLE stack that reads data from a battery

// All the mandatory includes, everything is used
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "sdkconfig.h"
#include "esp_sleep.h"
#include "driver/gpio.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/ledc.h"

#define LED_PIN 2                   // GPIO pin for the board LED
#define uS_TO_S_FACTOR 1000000ULL // Conversion factor for micro seconds to seconds [us -> s]

#define LEDC_TIMER LEDC_TIMER_0 // Use first timer of PWM
#define LEDC_MODE LEDC_LOW_SPEED_MODE   // Use low speed mode for PWM
#define LEDC_OUTPUT_IO (27) // GPIO number for PWM output
#define LEDC_CHANNEL LEDC_CHANNEL_0 // Use channel 0 of PWM
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT  // Set duty resolution to 8 bits
#define LEDC_DUTY102 (26) // Set DutyCycle to 10,2% ((2**8)-1)*0.102 = 26,01
#define LEDC_DUTY200 (51) // Set DutyCycle to 20,0% ((2**8)-1)*0.200 = 51,00
#define LEDC_DUTY400 (102) // Set DutyCycle to 40,0% ((2**8)-1)*0.400 = 102,00
#define LEDC_DUTY600 (153) // Set DutyCycle to 60,0% ((2**8)-1)*0.600 = 153,00
#define LEDC_DUTY800 (204) // Set DutyCycle to 80,0% ((2**8)-1)*0.800 = 204,00
#define LEDC_DUTY902 (230) // Set DutyCycle to 90,2% ((2**8)-1)*0.902 = 230,00
#define LEDC_DUTY1000 (255) // Set DutyCycle to 100,0% ((2**8)-1)*1.000 = 255,00
#define LEDC_FREQUENCY (1000) // Set frequency to 1kHz

#define DIVIDER_RATIO (0.130435) // Voltage divider ratio (10K volts in one end and 1.5K volts in the other end)

//config ADC with calibration
adc_oneshot_unit_handle_t adc1_handle;
adc_cali_handle_t adc_cali_handle;
adc_oneshot_chan_cfg_t config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = ADC_ATTEN_DB_11,
};
adc_cali_line_fitting_config_t fitting_config = {
    .unit_id = ADC_UNIT_1,
    .atten = ADC_ATTEN_DB_11,
    .bitwidth = ADC_BITWIDTH_DEFAULT,
};

// Data structure for the BLE server
typedef struct {
    float num1;
    float num2;
} data_t;

char *TAG = "NimBLE-Server";
uint8_t ble_addr_type;
SemaphoreHandle_t counterMutex; // Main mutex used to protect the counter
void ble_app_advertise(void);
char counter = 'n';             // Counter used to indicate what to do and what to send 
                                //"n" = nothing, "v" = voltage, "t" = temperature, "p" = ready to show Internal Resistance values, "f" = calculating Internal Resistance values, "s" = sleep
bool sleeping = false;          // Indicates if the ESP32 is sleeping
float calibration_ratio;        // Calibration ratio for the LM285, calculated in the calibration function using the LM285 voltage
int rcounter = 0;               // Counter used to indicate at which tier of the seven-step DC load the ESP32 is
float v, c;                     // Variables used to store the voltage and current values of the seven-step DC load
int sleeptime = 0;              // Variable used to store the sleep time

// Data structure for the BLE server initialized with 0
data_t data = {
    .num1 = 0.000,
    .num2 = 0.000,
};

// Initialize PWM configuration
static void pwm_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL,
        .duty = 0,
        .gpio_num = LEDC_OUTPUT_IO,
        .speed_mode = LEDC_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

// Calibrate the esp32 using the LM285
void calibrateLM285() {
    int adc_valueLM285;
    int adc_value_calibratedLM285;
    float LM285_voltage = 1.235;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_7, &adc_valueLM285));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, adc_valueLM285, &adc_value_calibratedLM285));
    calibration_ratio = LM285_voltage / ((float)adc_value_calibratedLM285/1000);
}

// Seven-step DC load
void seventierdcload(){
    int adc_valued;
    int adc_valued_calibrated;
    //1
    if (rcounter == 0){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY102));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    vTaskDelay(10000 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_5, &adc_valued));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, adc_valued, &adc_valued_calibrated));
    }
    //2
    else if (rcounter == 1){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY200));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    vTaskDelay(7000 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_5, &adc_valued));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, adc_valued, &adc_valued_calibrated));
    }
    //3
    else if (rcounter == 2){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY400));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    vTaskDelay(6000 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_5, &adc_valued));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, adc_valued, &adc_valued_calibrated));
    }
    //4
    else if (rcounter == 3){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY600));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_5, &adc_valued));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, adc_valued, &adc_valued_calibrated));
    }
    //5
    else if (rcounter == 4){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY800));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_5, &adc_valued));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, adc_valued, &adc_valued_calibrated));
    }
    //6
    else if (rcounter == 5){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY902));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    vTaskDelay(1500 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_5, &adc_valued));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, adc_valued, &adc_valued_calibrated));
    }
    //7
    else if (rcounter == 6){
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY1000));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_5, &adc_valued));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, adc_valued, &adc_valued_calibrated));
    }
    else{
        rcounter = 0;
        seventierdcload();
        return;
    }
    // Turn off the voltage
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
    // Calculate the corrected voltage and current
    int dividers[7] = {98, 50, 25, 17, 13, 11, 10}; 
    v = ((float)adc_valued_calibrated / (DIVIDER_RATIO*1000)) * calibration_ratio;
    c = v / dividers[rcounter];
    xSemaphoreTake(counterMutex, portMAX_DELAY); // Take the mutex
    counter = 'p'; // Change the counter
    xSemaphoreGive(counterMutex); // Give the mutex
    rcounter++; // Increase the rcounter
}

// Basic temperature reading
float read_temperature(){
    int adc_value;
    int adc_value_calibrated;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &adc_value));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, adc_value, &adc_value_calibrated));
    float temp;
    temp = (float)adc_value_calibrated * calibration_ratio;
    // Convert the voltage to temperature using the TMP36 datasheet
    temp = (temp-500) / 10;
    return temp;
}

// Basic voltage reading
float read_voltage(){
    int adc_value;
    int adc_value_calibrated;
    ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_5, &adc_value));
    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, adc_value, &adc_value_calibrated));
    float voltage = ((float)adc_value_calibrated / (DIVIDER_RATIO*1000)) * calibration_ratio;
    return voltage;
}

// Receive data from Raspberry Pi defined as server
static int device_write(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    printf("Data from the client: %.*s\n", ctxt->om->om_len, ctxt->om->om_data);
    if(ctxt->om->om_data[0] == 'v'){
        xSemaphoreTake(counterMutex, portMAX_DELAY); // Take the mutex
        counter = 'v'; // Increase the counter
        xSemaphoreGive(counterMutex); // Give the mutex
    }
    else if(isdigit(ctxt->om->om_data[0])){
        xSemaphoreTake(counterMutex, portMAX_DELAY); // Take the mutex
        counter = 's'; // Increase the counter
        xSemaphoreGive(counterMutex); // Give the mutex
        // Convert the sleep time from string to integer
        sleeptime = atoi((const char *)ctxt->om->om_data);
    }
    else if(ctxt->om->om_data[0] == 't'){
        xSemaphoreTake(counterMutex, portMAX_DELAY); // Take the mutex
        counter = 't'; // Increase the counter
        xSemaphoreGive(counterMutex); // Give the mutex
    }
    else if(ctxt->om->om_data[0] == 'r'){
        xSemaphoreTake(counterMutex, portMAX_DELAY); // Take the mutex
        counter = 'f'; // Increase the counter
        xSemaphoreGive(counterMutex); // Give the mutex
    }
    return 0;
}

// Send data to the Raspberry Pi defined as server
static int device_read(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    xSemaphoreTake(counterMutex, portMAX_DELAY); // Take the mutex
    if (counter == 'v'){
        xSemaphoreGive(counterMutex); // Give the mutex
        data.num1 = read_voltage(); // Read the voltage, due to the simplicity of the reading, the function is called on the NimBLE thread
        data.num2 = 0.000;
    }
    else if (counter == 't'){
        xSemaphoreGive(counterMutex); // Give the mutex
        data.num1 = read_temperature(); // Read the temperature, due to the simplicity of the reading, the function is called on the NimBLE thread
        data.num2 = 0.000;
    }
    else if (counter == 'p'){
        xSemaphoreGive(counterMutex); // Give the mutex
        data.num1 = v;
        data.num2 = c;
    }
    else if (counter == 'f'){
        xSemaphoreGive(counterMutex); // Give the mutex
        data.num1 = 0.000;
        data.num2 = 0.000;
    } 
    else{
        xSemaphoreGive(counterMutex); // Give the mutex
    }
    // Send the data to the client
    os_mbuf_append(ctxt->om, &data, sizeof(data));
    printf("Data sent to the client num1: %f\n", data.num1);
    printf("Data sent to the client num2: %f\n", data.num2);
    return 0;
}

// Array of pointers to other service definitions
// UUID - Universal Unique Identifier
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID16_DECLARE(0x180),                 // Define UUID for device type
     .characteristics = (struct ble_gatt_chr_def[]){
         {.uuid = BLE_UUID16_DECLARE(0xFEF4),           // Define UUID for reading
          .flags = BLE_GATT_CHR_F_READ,
          .access_cb = device_read},
         {.uuid = BLE_UUID16_DECLARE(0xDEAD),           // Define UUID for writing
          .flags = BLE_GATT_CHR_F_WRITE,
          .access_cb = device_write},
         {0}}},
    {0}};

// BLE event handling
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    // Advertise if connected
    case BLE_GAP_EVENT_CONNECT:
        // Reset the counters
        counter = 'n';
        rcounter = 0;
        ESP_LOGI("GAP", "BLE GAP EVENT CONNECT %s", event->connect.status == 0 ? "OK!" : "FAILED!");
        if (event->connect.status != 0)
        {
            ble_app_advertise();
        }
        break;
    // Advertise again after completion of the event
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI("GAP", "BLE GAP EVENT");
        ble_app_advertise();
        break;
    // Advertise again after disconnection
    case BLE_GAP_EVENT_DISCONNECT:
        // If the ESP32 disconnects without going to sleep, change the values to signal that a error occurred
        data.num1 = 1.000;
        data.num2 = 1.000;
        ESP_LOGI("GAP", "BLE GAP EVENT DISCONNECT");
        ble_app_advertise();
        break;
    default:
        break;
    }
    return 0;
}

// Define the BLE connection
void ble_app_advertise(void)
{
    int rc;
    // GAP - device name definition
    struct ble_hs_adv_fields fields;
    const char *device_name;
    memset(&fields, 0, sizeof(fields));
    device_name = ble_svc_gap_device_name(); // Read the BLE device name
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0 && !sleeping) {
        MODLOG_DFLT(ERROR, "error setting advertisement data; rc=%d\n", rc);
        return;
    }
    // GAP - device connectivity definition
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable or non-connectable
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // discoverable or non-discoverable
    rc = ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
    if (rc != 0 && !sleeping) {
        MODLOG_DFLT(ERROR, "error enabling advertisement; rc=%d\n", rc);
        return;
    }
}

// The application
void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type); // Determines the best address type automatically
    ble_app_advertise();                     // Define the BLE connection
}

// The infinite task
void host_task(void *param)
{
    ESP_LOGI(TAG, "NimBLE Server Task Started");
    gpio_set_level(LED_PIN, 1);     // Turn on the board LED
    nimble_port_run();              // This function will return only when nimble_port_stop() is executed
    nimble_port_freertos_deinit();  // Deinitialize the NimBLE host configuration
    gpio_set_level(LED_PIN, 0);     // Turn off the board LED
}

// Start the BLE server after the ESP32 wakes up
void start_ble(){
    esp_err_t ret = nimble_port_init();             // 2 - Initialize the host stack
    if (ret != ESP_OK)
    {
        MODLOG_DFLT(ERROR, "Failed to init nimble %d \n", ret);
        return;
    }
    ble_svc_gap_device_name_set("NimBLE-Server");   // 3 - Initialize NimBLE configuration - server name
    ble_svc_gap_init();                             // 3 - Initialize NimBLE configuration - gap service
    ble_svc_gatt_init();                            // 3 - Initialize NimBLE configuration - gatt service
    ble_gatts_count_cfg(gatt_svcs);                 // 3 - Initialize NimBLE configuration - config gatt services
    ble_gatts_add_svcs(gatt_svcs);                  // 3 - Initialize NimBLE configuration - queues gatt services.
    ble_hs_cfg.sync_cb = ble_app_on_sync;           // 4 - Initialize application
    nimble_port_freertos_init(host_task);           // 5 - Run the thread
}

void app_main()
{
    esp_err_t ret = nvs_flash_init();              // 1 - Initialize NVS flash using
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||        //    the default NVS partition
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());        //    if it fails, erase the partition
        ret = nvs_flash_init();                    //    and try again
    }
    ESP_ERROR_CHECK(ret);                          //    if it fails again, throw an error
    gpio_reset_pin(LED_PIN);                       // Set the GPIO pin
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT); // Set the GPIO pin as output
    
    // Initialize the PWM configuration
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle)); // Initialize ADC1

    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_7, &config)); // Configure ADC1 for channel 7 (LM285)
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &config)); // Configure ADC1 for channel 6 (TMP36)
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_5, &config)); // Configure ADC1 for channel 5 (Battery)
    ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&fitting_config, &adc_cali_handle)); // Create calibration scheme for ADC1

    pwm_init(); // Initialize PWM
    calibrateLM285(); // Calibrate the ESP32 using the LM285

    counterMutex = xSemaphoreCreateMutex(); // Create a mutex    
    while (1) {
        sleeping = false;
        start_ble();  // Start BLE
        while(1){
            xSemaphoreTake(counterMutex, portMAX_DELAY); // Take the mutex
            if (counter == 's'){
                xSemaphoreGive(counterMutex); // Give the mutex
                // If the ESP32 is must sleep, break the loop to start the sleep process
                break;
            }
            else if (counter == 'f'){
                xSemaphoreGive(counterMutex); // Give the mutex
                seventierdcload(); // Start the seven-step DC load, given the time this process takes it is executed in the main thread
            }
            else{
                xSemaphoreGive(counterMutex); // Give the mutex
                vTaskDelay(1); // This delay is mandatory to avoid a watchdog error
            }
        }
        vTaskDelay(500 / portTICK_PERIOD_MS); // This delay is just for safety
        printf("Going to sleep now\n");
        sleeping = true;
        nimble_port_stop();  // Stop BLE
        ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(sleeptime * uS_TO_S_FACTOR)); // Enable timer wakeup, sleep for given seconds
        esp_deep_sleep_start(); // Enter deep sleep
    }
}
