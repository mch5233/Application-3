/* --------------------------------------------------------------
   Application: 03 - Rev1
   Release Type: Interrupt-Driven Event Processing
   Class: Real Time Systems - Su 2025
   Author: Mya Camacho-Hill
   Context: Healthcare LED indicates device status, serial messages provide patient vitals,
            sensor monitors patient pulse (simulated by light sensor) with alert for
            abnormal readings, and button triggers data logging report.
---------------------------------------------------------------*/
/*
I used AI to help me figure out good variable names for my code,
how to implement the lux equation within my code, what each of my task 
should do to implement my chosen thematic and to create the comments 
within this code. Debugging my implementation of the button ISR 
as well as how to incoporate it into my thematic.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "math.h"
#include "esp_intr_alloc.h"

#define LED_PIN GPIO_NUM_2          // Using GPIO2 for the LED
#define LDR_PIN GPIO_NUM_32         // LDR on GPIO32
#define BUTTON_PIN GPIO_NUM_4       // Button on GPIO4 for interrupt
#define LDR_ADC_CHANNEL ADC1_CHANNEL_4 // GPIO32 ADC channel

// Sensor task defines
#define AVG_WINDOW 10               // Number of readings for moving average
#define SENSOR_LOG_SIZE 50          // Buffer size for sensor data logging
#define PULSE_RATE_THRESHOLD_LOW 50
#define PULSE_RATE_THRESHOLD_HIGH 100
#define V_SOURCE 3.3f
#define R_FIXED 10000.0f
#define GAMMA 0.7f
#define RL_CONSTANT 50.0f

// Priority levels (higher number = higher priority)
#define PRIORITY_LOW 1              // LED blink task
#define PRIORITY_MEDIUM 2           // Console print and sensor tasks
#define PRIORITY_HIGH 3             // Button logger task (highest)

// Global variables for sensor data logging
static float sensor_log_buffer[SENSOR_LOG_SIZE];
static int log_index = 0;
static int total_readings = 0;
static bool buffer_wrapped = false;

// Semaphore handle for button interrupt
static SemaphoreHandle_t xButtonSemaphore = NULL;

// ISR handler for button press
void IRAM_ATTR button_isr_handler(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Give semaphore from ISR to unblock logger task
    xSemaphoreGiveFromISR(xButtonSemaphore, &xHigherPriorityTaskWoken);
    
    // Request context switch if higher priority task was woken
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Function to add sensor reading to log buffer
void add_to_sensor_log(float lux_value) {
    sensor_log_buffer[log_index] = lux_value;
    log_index = (log_index + 1) % SENSOR_LOG_SIZE;
    total_readings++;
    
    if (log_index == 0 && total_readings >= SENSOR_LOG_SIZE) {
        buffer_wrapped = true;
    }
}

// Function to compress and print sensor log data
void compress_and_print_log(void) {
    int valid_readings = buffer_wrapped ? SENSOR_LOG_SIZE : (total_readings < SENSOR_LOG_SIZE ? total_readings : SENSOR_LOG_SIZE);
    
    if (valid_readings == 0) {
        printf("=== PATIENT DATA REPORT ===\n");
        printf("No sensor data available for compression.\n");
        printf("========================\n\n");
        return;
    }
    
    // Simulate compression by computing statistics
    float min_lux = sensor_log_buffer[0];
    float max_lux = sensor_log_buffer[0];
    float sum_lux = 0.0f;
    int high_readings = 0;
    
    for (int i = 0; i < valid_readings; i++) {
        float current = sensor_log_buffer[i];
        if (current < min_lux) min_lux = current;
        if (current > max_lux) max_lux = current;
        sum_lux += current;
        
        // Count readings above threshold (simulating abnormal light conditions)
        if (current > 1000.0f) high_readings++;
    }
    
    float avg_lux = sum_lux / valid_readings;
    
    // Print compressed report
    printf("=== PATIENT ENVIRONMENTAL DATA REPORT ===\n");
    printf("Nurse Request: Data compression completed\n");
    printf("Total readings processed: %d\n", valid_readings);
    printf("Light sensor summary (Lux):\n");
    printf("  Minimum: %.2f\n", min_lux);
    printf("  Maximum: %.2f\n", max_lux);
    printf("  Average: %.2f\n", avg_lux);
    printf("  High-light incidents: %d\n", high_readings);
    printf("Room lighting assessment: %s\n", 
           avg_lux > 500 ? "Bright environment" : 
           avg_lux > 100 ? "Moderate lighting" : "Low lighting");
    printf("========================================\n\n");
}

// Task 1: Device Status LED (Low Priority) - Updated timing and removed prints
void device_status_led_task(void *pvParameters) {
    bool led_on = false;
    const TickType_t periodTicks = pdMS_TO_TICKS(1400); // 1.4 seconds as required
    TickType_t lastWakeTime = xTaskGetTickCount();
    
    while (1) {
        gpio_set_level(LED_PIN, led_on);
        led_on = !led_on;
        // No printing in LED task as per requirements
        vTaskDelayUntil(&lastWakeTime, periodTicks);
    }
    vTaskDelete(NULL);
}

// Task 2: Patient Vitals Console Print (Medium Priority) - Updated timing
void patient_vitals_print_task(void *pvParameters) {
    TickType_t previousWakeTime = xTaskGetTickCount();
    const TickType_t periodTicks = pdMS_TO_TICKS(7000); // 7 seconds as required

    while (1) {
        // Print patient status with timestamp
        printf("Patient Status Update: HR: 72 bpm | SpO2: 99%% | BP: 120/80 | Room Temp: 22.5C | Time: %lu ms\n",
               (unsigned long)(xTaskGetTickCount() * portTICK_PERIOD_MS));
        vTaskDelayUntil(&previousWakeTime, periodTicks);
    }
    vTaskDelete(NULL);
}

// Task 3: Patient Pulse Sensor with Data Logging (Medium Priority)
void patient_pulse_sensor_task(void *pvParameters) {
    // Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(LDR_ADC_CHANNEL, ADC_ATTEN_DB_11);

    // Variables for sensor processing
    int raw_adc_value;
    float Vmeasure = 0.0f;
    float Rmeasure = 0.0f;
    float lux = 0.0f;

    // Moving average variables
    float lux_readings[AVG_WINDOW];
    int idx = 0;
    float sum = 0.0f;

    // Initialize moving average buffer
    for (int i = 0; i < AVG_WINDOW; ++i) {
        raw_adc_value = adc1_get_raw(LDR_ADC_CHANNEL);
        Vmeasure = (float)raw_adc_value * (V_SOURCE / 4095.0f);
        Rmeasure = (Vmeasure * R_FIXED) / (V_SOURCE - Vmeasure);
        lux = pow(RL_CONSTANT * pow(10, 3) * pow(10 / Rmeasure, GAMMA), (1.0f / GAMMA));

        if (isnan(lux) || isinf(lux)) {
            lux = 0.0f;
        }

        lux_readings[i] = lux;
        sum += lux_readings[i];
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    const TickType_t periodTicks = pdMS_TO_TICKS(250); // 250ms sampling
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (1) {
        // Read sensor
        raw_adc_value = adc1_get_raw(LDR_ADC_CHANNEL);
        Vmeasure = (float)raw_adc_value * (V_SOURCE / 4095.0f);
        Rmeasure = (Vmeasure * R_FIXED) / (V_SOURCE - Vmeasure);
        lux = pow(RL_CONSTANT * pow(10, 3) * pow(10 / Rmeasure, GAMMA), (1.0f / GAMMA));

        if (isnan(lux) || isinf(lux)) {
            lux = 0.0f;
        }

        // Update moving average
        sum -= lux_readings[idx];
        lux_readings[idx] = lux;
        sum += lux;
        idx = (idx + 1) % AVG_WINDOW;
        float avg_lux = sum / AVG_WINDOW;

        // Add to data log for button-triggered reports
        add_to_sensor_log(avg_lux);

        // Simulate pulse rate from light sensor
        int simulated_pulse_rate = (int)(avg_lux / 38.0f + 30.0f);

        // Check thresholds and print alerts
        if (simulated_pulse_rate < PULSE_RATE_THRESHOLD_LOW) {
            printf("**ALERT: Bradycardia detected!** Patient Pulse Rate: %d bpm (Room Lux: %.2f)\n", 
                   simulated_pulse_rate, avg_lux);
        } else if (simulated_pulse_rate > PULSE_RATE_THRESHOLD_HIGH) {
            printf("**ALERT: Tachycardia detected!** Patient Pulse Rate: %d bpm (Room Lux: %.2f)\n", 
                   simulated_pulse_rate, avg_lux);
        } else {
            printf("Patient Environment: Raw=%d, Vmeas=%.2fV, Rmeas=%.0f Ohm, Lux=%.2f, Avg Lux=%.2f, Pulse: %d bpm\n",
                   raw_adc_value, Vmeasure, Rmeasure, lux, avg_lux, simulated_pulse_rate);
        }

        vTaskDelayUntil(&lastWakeTime, periodTicks);
    }
    vTaskDelete(NULL);
}

// Task 4: Button-Triggered Logger (Highest Priority)
void button_logger_task(void *pvParameters) {
    while (1) {
        // Block until button semaphore is given by ISR
        xSemaphoreTake(xButtonSemaphore, portMAX_DELAY);
        
        printf("\n>>> NURSE CALL BUTTON PRESSED <<<\n");
        printf("Processing patient data report request...\n");
        
        // Simulate processing delay
        vTaskDelay(pdMS_TO_TICKS(100));
        
        // Compress and print the sensor log
        compress_and_print_log();
        
        printf("Data report complete. System ready for next request.\n\n");
    }
    vTaskDelete(NULL);
}

void app_main() {
    // Initialize LED GPIO
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    // Initialize LDR GPIO
    gpio_reset_pin(LDR_PIN);
    gpio_set_direction(LDR_PIN, GPIO_MODE_INPUT);

    // Initialize Button GPIO with pull-up and interrupt
    gpio_reset_pin(BUTTON_PIN);
    gpio_set_direction(BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON_PIN, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(BUTTON_PIN, GPIO_INTR_NEGEDGE); // Trigger on falling edge (button press)

    // Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(LDR_ADC_CHANNEL, ADC_ATTEN_DB_11);

    // Create binary semaphore for button interrupt
    xButtonSemaphore = xSemaphoreCreateBinary();
    if (xButtonSemaphore == NULL) {
        printf("Failed to create button semaphore!\n");
        return;
    }

    // Install GPIO ISR service
    gpio_install_isr_service(0);
    
    // Add ISR handler for button
    gpio_isr_handler_add(BUTTON_PIN, button_isr_handler, NULL);

    printf("=== Healthcare Patient Monitoring System Started ===\n");
    printf("Features: Device heartbeat, vitals monitoring, pulse sensing, nurse call button\n");
    printf("Press button on GPIO4 to request patient data report\n\n");

    // Create all tasks pinned to Core 1 with appropriate priorities
    xTaskCreatePinnedToCore(device_status_led_task, "DeviceHeartbeat", 2048, NULL, PRIORITY_LOW, NULL, 1);
    xTaskCreatePinnedToCore(patient_vitals_print_task, "VitalsMonitor", 2048, NULL, PRIORITY_MEDIUM, NULL, 1);
    xTaskCreatePinnedToCore(patient_pulse_sensor_task, "PulseSensor", 4096, NULL, PRIORITY_MEDIUM, NULL, 1);
    xTaskCreatePinnedToCore(button_logger_task, "NurseCallLogger", 4096, NULL, PRIORITY_HIGH, NULL, 1);

    printf("All tasks created and running. System operational.\n");
}
