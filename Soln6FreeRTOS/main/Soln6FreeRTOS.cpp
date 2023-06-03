#include <atomic>
#include <cstring>
#include <iostream>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/semphr.h"

#define LED_DELAY1_DEF (300U)
#define LED_DELAY2_DEF (500U)
#define LED_DELAY3_DEF (234U)

#define TASK_STACK_SIZE (12U * 1024U) // 12kb

#define LED_PIN GPIO_NUM_13

TaskHandle_t led1_task_handle = NULL;
TaskHandle_t led2_task_handle = NULL;
TaskHandle_t led3_task_handle = NULL;

constexpr std::string_view led_toggle_task_TAG = "led_toggle_task";
constexpr std::string_view app_main_task_TAG = "app_main_write_task";

SemaphoreHandle_t led_mutex = NULL;

using led_task_parm_st = struct led_param {
  uint8_t task_no;
  TickType_t blink_rate;
};

void led_toggle_task(void *params) {

  led_task_parm_st const led_params = *((const led_task_parm_st *)params);

  bool led_status = false;
  ESP_LOGI("led_toggle_task", "Task %u, blink rate = %ul", led_params.task_no,
           led_params.blink_rate);

  while (true) {
    ESP_LOGI("led_toggle_task", "Task %u", led_params.task_no);
    led_status = !led_status;
    gpio_set_level(LED_PIN, led_status);
    vTaskDelay(led_params.blink_rate / portTICK_RATE_MS);
  }
}

esp_err_t led_init() {
  gpio_config_t led_gpio_cfg = {.pin_bit_mask = BIT(LED_PIN),
                                .mode = GPIO_MODE_OUTPUT,
                                .pull_up_en = GPIO_PULLUP_DISABLE,
                                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                .intr_type = static_cast<gpio_int_type_t>(0)};

  return gpio_config(&led_gpio_cfg);
}

extern "C" void app_main(void) {

  led_task_parm_st led1_params = {.task_no = 1, .blink_rate = LED_DELAY1_DEF};
  led_task_parm_st led2_params = {.task_no = 2, .blink_rate = LED_DELAY2_DEF};
  led_task_parm_st led3_params = {.task_no = 3, .blink_rate = LED_DELAY3_DEF};
  ESP_ERROR_CHECK(led_init());

  led_mutex = xSemaphoreCreateMutex();
  xSemaphoreTake(led_mutex,0);

  BaseType_t xReturn0 =
      xTaskCreate(&led_toggle_task, led_toggle_task_TAG.data(), TASK_STACK_SIZE,
                  &led1_params, 1, &led1_task_handle);

  xSemaphoreGive(led_mutex);

  if (xReturn0 != pdPASS) {
    ESP_LOGE(app_main_task_TAG.data(), "Failed to create task: %s",
             led_toggle_task_TAG.data());
    led1_task_handle = NULL;
    vTaskDelete(NULL);
  }

  BaseType_t xReturn1 = // This will still read correct params as main task may
                        // be still in the scope
      xTaskCreate(&led_toggle_task, led_toggle_task_TAG.data(), TASK_STACK_SIZE,
                  &led2_params, 1, &led2_task_handle);

  if (xReturn1 != pdPASS) {
    ESP_LOGE(app_main_task_TAG.data(), "Failed to create task: %s",
             led_toggle_task_TAG.data());
    led2_task_handle = NULL;
    vTaskDelete(NULL);
  }
  // xSemaphoreTake(led_mutex,0); // This will solve the problem
  BaseType_t xReturn2 = // This task will read junk from params
      xTaskCreate(&led_toggle_task, led_toggle_task_TAG.data(), TASK_STACK_SIZE,
                  &led3_params, 1, &led3_task_handle);
  
  // xSemaphoreGive(led_mutex);
  /* //To make it go outof scope soon, ignoring the check
    if (xReturn1 != pdPASS) {
      ESP_LOGE(app_main_task_TAG.data(), "Failed to create task: %s",
               led_toggle_task_TAG.data());
      led3_task_handle = NULL;
      vTaskDelete(NULL);
    }
    vTaskDelete(NULL);
  */
}
