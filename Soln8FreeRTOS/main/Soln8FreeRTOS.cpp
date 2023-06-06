#include <atomic>
#include <cstring>
#include <iostream>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"

#include "driver/dac.h"

#define UART_DELAY (100U / portTICK_RATE_MS)
#define LED_DELAY_DEF (500U)

#define TASK_STACK_SIZE (12U * 1024U) // 12kb

#define LED_PIN GPIO_NUM_26

TaskHandle_t led_turnoff_task_handle = NULL;
TaskHandle_t uart_task_handle = NULL;

constexpr std::string_view led_turnoff_task_TAG = "led_turnoff_task";
constexpr std::string_view uart_task_TAG = "uart_task";
constexpr std::string_view app_main_task_TAG = "app_main_write_task";

constexpr uart_port_t uart_port_num = UART_NUM_0;
constexpr size_t uart_buff_size = 512U;
constexpr size_t uart_queue_len = 10U;
QueueHandle_t uart_queue;

TimerHandle_t backlight_timer_handle = NULL;
SemaphoreHandle_t backlight_turnoff_sema = NULL;
constexpr TickType_t kBlackLightTimeout = (5000 / portTICK_RATE_MS);

void backlight_timer_cbk(void *params) {
  xSemaphoreGive(backlight_turnoff_sema);
}

void led_turnoff_task(void *params) {

  while (true) {
    if (xSemaphoreTake(backlight_turnoff_sema,
                       portMAX_DELAY)) { // be here untill you get signal from
                                         // timer callback fn
      uint8_t i = 255;
      while (i--) {
        dac_output_enable(DAC_CHANNEL_2);
        dac_output_voltage(DAC_CHANNEL_2, i);
        vTaskDelay(1);
      }
    }
    vTaskDelay(10 / portTICK_RATE_MS);
  }
  vTaskDelete(NULL);
}

esp_err_t led_init() {
  gpio_config_t led_gpio_cfg = {.pin_bit_mask = BIT(LED_PIN),
                                .mode = GPIO_MODE_OUTPUT,
                                .pull_up_en = GPIO_PULLUP_DISABLE,
                                .pull_down_en = GPIO_PULLDOWN_DISABLE,
                                .intr_type = static_cast<gpio_int_type_t>(0)};

  return gpio_config(&led_gpio_cfg);
}

esp_err_t uart_init() {

  const uart_config_t uart_config = {
      .baud_rate = 115200,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      //.rx_flow_ctrl_thresh = 0
  };

  esp_err_t err = uart_param_config(uart_port_num, &uart_config);
  err |= uart_set_pin(UART_NUM_0, 1, 3, -1, -1);

  err |= uart_driver_install(uart_port_num, uart_buff_size, uart_buff_size,
                             uart_queue_len, &uart_queue, 0);
  return err;
}

void uart_task(void *params) {

  while (true) {
    int ret = 0;
    static char read_buff[100] = {0};
    memset(read_buff, 0, strlen(read_buff)); // clear the local buffer
    do {
      ret = uart_read_bytes(uart_port_num, (void *)read_buff,
                            (sizeof(read_buff) - 1), 10 * UART_DELAY);
    } while (ret == 0);

    if (ret == -1) {
      ESP_LOGE(uart_task_TAG.data(), "Uart Read Error");
      continue;
    }

    if (ret) {
      uart_write_bytes(uart_port_num, (const char *)"\r", strlen("\r"));
      uart_write_bytes(uart_port_num, (const char *)read_buff,
                       strlen(read_buff));
      uart_write_bytes(uart_port_num, (const char *)"\n\r", strlen("\n\r"));
      /*Make LED HIGH*/

      dac_output_enable(DAC_CHANNEL_2);
      dac_output_voltage(DAC_CHANNEL_2, 255);

      if (!xTimerStart(backlight_timer_handle, portMAX_DELAY)) {
        ESP_LOGE(uart_task_TAG.data(), "Failed to (Re)Start timer: %s",
                 "backlight_timer");
      }
    }
    vTaskDelay(UART_DELAY);
  }
  vTaskDelete(NULL);
}

extern "C" void app_main(void) {

  ESP_ERROR_CHECK(led_init());
  ESP_ERROR_CHECK(uart_init());

  backlight_turnoff_sema = xSemaphoreCreateBinary();

  if (!backlight_turnoff_sema) {
    ESP_LOGE(app_main_task_TAG.data(),
             "Failed to create xSemaphoreCreateBinary");
    vTaskDelete(NULL);
  }

  backlight_timer_handle =
      xTimerCreate("backlight_timer", kBlackLightTimeout, pdFALSE, (void *)1,
                   &backlight_timer_cbk);

  if (!backlight_timer_handle) {
    ESP_LOGE(app_main_task_TAG.data(), "Failed to create timer: %s",
             "backlight_timer");
    vTaskDelete(NULL);
  }

  BaseType_t xReturn0 =
      xTaskCreate(&led_turnoff_task, led_turnoff_task_TAG.data(),
                  TASK_STACK_SIZE, NULL, 1, &led_turnoff_task_handle);

  if (xReturn0 != pdPASS) {
    ESP_LOGE(app_main_task_TAG.data(), "Failed to create task: %s",
             led_turnoff_task_TAG.data());
    led_turnoff_task_handle = NULL;
    vTaskDelete(NULL);
  }

  BaseType_t xReturn1 =
      xTaskCreate(&uart_task, uart_task_TAG.data(), TASK_STACK_SIZE, NULL, 1,
                  &uart_task_handle);

  if (xReturn1 != pdPASS) {
    ESP_LOGE(app_main_task_TAG.data(), "Failed to create task: %s",
             uart_task_TAG.data());
    uart_task_handle = NULL;
    vTaskDelete(NULL);
  }

  vTaskDelete(NULL);
}