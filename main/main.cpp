#include <stdio.h>
#include <esp_system.h>
#include <nvs_flash.h>

#include <driver/gpio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <driver/mcpwm.h>
#include <soc/mcpwm_periph.h>

#include <esp_log.h>

#include "ble.h"

volatile uint32_t captured_freq = 0;
volatile uint32_t captured_duty = 0;

#define DUTY_CAP_COUNT 800

extern "C" void IRAM_ATTR mcpwm_cap_it(void* arg) {
    if(MCPWM0.int_st.cap0_int_st == 1) {
        gpio_set_level(GPIO_NUM_26, 1);
        static uint32_t prev = 0;

        uint32_t cap0_val = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0);
        uint32_t cap1_val = MCPWM0.cap_val_ch[1];

        if(cap0_val >= prev && cap0_val >= cap1_val) {
            uint32_t captured_period = cap0_val - prev;
            uint32_t duty = ((cap1_val - prev) * 100) / captured_period;
            uint32_t freq = 80e6 / captured_period;
            if(freq >= 6000 && freq <= 15000) {
                static uint32_t duty_idx = 0;
                static uint32_t dutys[DUTY_CAP_COUNT];

                dutys[duty_idx] = duty;

                if(++duty_idx >= DUTY_CAP_COUNT) {
                    duty_idx = 0;
                    //result
                    uint32_t max = 0;
                    for(int i = 0; i < DUTY_CAP_COUNT; i++) {
                        if(max < dutys[i])
                            max = dutys[i];
                    }
                    captured_duty = max;
                    captured_freq = freq;
                }

            }
            // static bool pin_up = false;
            // gpio_set_level(GPIO_NUM_26, pin_up = !pin_up);
        }
        prev = cap0_val;
        gpio_set_level(GPIO_NUM_26, 0);
    }
}

void start_capture() {
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_0, GPIO_NUM_15));
    ESP_ERROR_CHECK(mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM_CAP_1, GPIO_NUM_15));
    ESP_ERROR_CHECK(mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP0, MCPWM_POS_EDGE, 0));
    ESP_ERROR_CHECK(mcpwm_capture_enable(MCPWM_UNIT_0, MCPWM_SELECT_CAP1, MCPWM_NEG_EDGE, 0));
    ESP_ERROR_CHECK(mcpwm_isr_register(MCPWM_UNIT_0, mcpwm_cap_it, NULL, ESP_INTR_FLAG_IRAM, NULL)); 
    MCPWM0.int_ena.cap0_int_ena = 1;
}

void prepare_nvs() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void print_task_list() {
    char c_buf[1024];
    vTaskList(c_buf);
    ESP_LOGI(__FILENAME__, "\n%s\n", c_buf);
}

void main_loop(void* arg) {
    start_capture();
    print_task_list();
    for(;;) {
        vTaskDelay(500 / portTICK_RATE_MS);
        // ESP_LOGI(__FILENAME__, "captured_freq=%u captured_duty=%u%%\n", captured_freq, captured_duty);
        ble_set_charc(captured_duty);
    }
}

extern "C" void app_main(void) {
    prepare_nvs();
    ble_init();
    ble_start_app();

    gpio_config_t pin_cfg = {.pin_bit_mask = 1 << GPIO_NUM_26, .mode = GPIO_MODE_OUTPUT, .pull_up_en = GPIO_PULLUP_DISABLE, .pull_down_en = GPIO_PULLDOWN_DISABLE, .intr_type = GPIO_INTR_DISABLE};
    ESP_ERROR_CHECK(gpio_config(&pin_cfg));

    xTaskCreatePinnedToCore(main_loop, "main_loop", 4096, NULL, 1, NULL, 1);

    // start_capture();
    // ble_start_app();

    // print_task_list();

    // ESP_LOGI(__FILENAME__, "start");
    
    // for(;;) {
    //     vTaskDelay(500 / portTICK_RATE_MS);
    //     ESP_LOGI(__FILENAME__, "captured_freq=%u captured_duty=%u%%\n", captured_freq, captured_duty);
    //     ble_set_charc(captured_duty);
    // }
}