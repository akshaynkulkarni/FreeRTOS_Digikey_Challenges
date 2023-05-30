#include <stdint.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#define DELAY1 (200U / portTICK_RATE_MS)
#define DELAY2 (500U / portTICK_RATE_MS)

#define TASK_STACK_SIZE (12U * 1024U) // 12kb

#define LED_PIN GPIO_NUM_13

void led_toggle_task(void *params) {
  const TickType_t led_delay = (*((const uint8_t *)params)) ? DELAY2 : DELAY1;

  bool led_status = false;

  while (true) {
    ESP_LOGI("led_toggle_task", " %u", (*((const uint8_t *)params)));
    led_status = !led_status;
    gpio_set_level(LED_PIN, led_status);
    vTaskDelay(led_delay);
  }
}

esp_err_t led_init() {
  gpio_config_t led_gpio_cfg = {.pin_bit_mask = BIT(LED_PIN),
                                .mode = GPIO_MODE_OUTPUT,
                                .pull_up_en = GPIO_PULLUP_DISABLE,
                                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                .intr_type = 0};

  return gpio_config(&led_gpio_cfg);
}

void app_main(void) {
  static uint8_t const task0_param = 0;
  static uint8_t const task1_param = 1;

  ESP_ERROR_CHECK(led_init());

  BaseType_t xReturn0 =
      xTaskCreate(&led_toggle_task, "led_toggle_task0", TASK_STACK_SIZE,
                  (void *)&task0_param, 1, NULL);

  if (xReturn0 != pdPASS) {
    ESP_LOGE("app_main", "Failed to create task: led_toggle_task0");
    vTaskDelete(NULL);
  }

  BaseType_t xReturn1 =
      xTaskCreate(&led_toggle_task, "led_toggle_task1", TASK_STACK_SIZE,
                  (void *)&task1_param, 1, NULL);

  if (xReturn1 != pdPASS) {
    ESP_LOGE("app_main", "Failed to create task: led_toggle_task1");
    vTaskDelete(NULL);
  }
}
