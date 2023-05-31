

#include <atomic>
#include <iostream>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"

#define UART_DELAY (100U / portTICK_RATE_MS)
#define LED_DELAY_DEF (500U / portTICK_RATE_MS)

#define TASK_STACK_SIZE (12U * 1024U) // 12kb

#define LED_PIN GPIO_NUM_13

static std::atomic<TickType_t> led_delay = LED_DELAY_DEF;

TaskHandle_t led_task_handle = NULL;
TaskHandle_t uart_task_handle = NULL;

constexpr std::string_view led_task_TAG = "led_toggle_task";
constexpr std::string_view uart_read_task_TAG = "uart_read_task";
constexpr std::string_view uart_write_task_TAG = "uart_write_task";
constexpr std::string_view app_main_task_TAG = "app_main_write_task";

void led_toggle_task(void *params) {

  bool led_status = false;

  while (true) {
    led_status = !led_status;
    gpio_set_level(LED_PIN, led_status);
    ESP_LOGI(led_task_TAG.data(), "I am LED");
    vTaskDelay(led_delay.load());
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

const uart_port_t uart_port_num = UART_NUM_0;
const size_t uart_buff_size = 512U;
const size_t uart_queue_len = 10U;
QueueHandle_t uart_queue;

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

#if 0
void uart_write_task(void *params) {

  while (true) {

    std::string write_str = "uart_write_task: test string.\n\r";
    uart_write_bytes(uart_port_num, (const char *)write_str.data(),
                     write_str.size());

    vTaskDelay(20*UART_DELAY);
  }
  vTaskDelete(NULL);
}
#endif

void uart_read_task(void *params) {
  static char read_buff[11];
  while (true) {
    int ret = 0;
    do {
      ret = uart_read_bytes(uart_port_num, (void *)read_buff,
                            (sizeof(read_buff) - 1), 10 * UART_DELAY);
    } while (ret == 0);

    if (ret == -1) {
      ESP_LOGE(uart_read_task_TAG.data(), "Uart Read Error");
      continue;
    }

    read_buff[ret] = '\0';

    if (ret) {

      sscanf(read_buff, "%[0-9]s", read_buff);
      if ((atoi(read_buff))) {
        vTaskSuspend(
            led_task_handle); // Immediate effect: Suspend the led task as it is
                              // currently blocked due to vTaskDelay()
        led_delay.store((atoi(read_buff) / portTICK_RATE_MS));
        ESP_LOGI(uart_read_task_TAG.data(), "Update delay time to:%d ms",
                 atoi(read_buff));
        vTaskResume(led_task_handle); // Immediate effect: Start blinking with
                                      // new delay time
      } else {
        ESP_LOGE(uart_read_task_TAG.data(),
                 "Enter only numbers[0-9], No delay time update!");
      }
    }

    vTaskDelay(UART_DELAY);
  }
  vTaskDelete(NULL);
}

extern "C" void app_main(void) {

  ESP_ERROR_CHECK(led_init());
  ESP_ERROR_CHECK(uart_init());

  BaseType_t xReturn0 = xTaskCreate(&led_toggle_task, led_task_TAG.data(),
                                    TASK_STACK_SIZE, NULL, 1, &led_task_handle);

  if (xReturn0 != pdPASS) {
    ESP_LOGE(app_main_task_TAG.data(), "Failed to create task: %s",
             led_task_TAG.data());
    vTaskDelete(NULL);
  }

  BaseType_t xReturn1 =
      xTaskCreate(&uart_read_task, uart_read_task_TAG.data(), TASK_STACK_SIZE,
                  NULL, 1, &uart_task_handle);

  if (xReturn1 != pdPASS) {
    ESP_LOGE("app_main", "Failed to create task: %s",
             uart_read_task_TAG.data());
    vTaskDelete(NULL);
  }
  vTaskDelete(NULL);
}
