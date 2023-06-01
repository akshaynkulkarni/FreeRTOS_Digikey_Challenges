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
#include <mutex>
#define UART_DELAY (100U / portTICK_RATE_MS)
#define LED_DELAY_DEF (500U / portTICK_RATE_MS)

#define TASK_STACK_SIZE (12U * 1024U) // 12kb

#define LED_PIN GPIO_NUM_13

using shared_ptr = char *;
static std::atomic<shared_ptr> shared_data_ptr = NULL;
static std::atomic<size_t> shared_data_size = 0;

TaskHandle_t led_task_handle = NULL;
TaskHandle_t uart_read_task_handle = NULL;
TaskHandle_t uart_write_task_handle = NULL;

constexpr std::string_view led_task_TAG = "led_toggle_task";
constexpr std::string_view uart_read_task_TAG = "uart_read_task";
constexpr std::string_view uart_write_task_TAG = "uart_write_task";
constexpr std::string_view app_main_task_TAG = "app_main_write_task";

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

void uart_write_task(void *params) {

  char const *creturn = "\n\r";
  while (true) {
    shared_ptr temp = shared_data_ptr.load();
    if (temp != NULL) {
      //ESP_LOGI(uart_write_task_TAG.data(), "%s", (const char *)temp);
      uart_write_bytes(uart_port_num, (const char *)uart_write_task_TAG.data(),
                       uart_write_task_TAG.size());
      uart_write_bytes(uart_port_num, (const char *)": ",
                       sizeof(": "));
      uart_write_bytes(uart_port_num, (const char *)temp,
                       shared_data_size.load());
      uart_write_bytes(uart_port_num, (const char *)creturn, sizeof(creturn));
      vPortFree(temp);
      temp = NULL;
      shared_data_size.store(0);
      shared_data_ptr.store(temp);
    }

    vTaskDelay(UART_DELAY);
  }
  vTaskDelete(NULL);
}

void uart_read_task(void *params) {

  while (true) {
    int ret = 0;
    static char read_buff[100] = {0};

    do {
      ret = uart_read_bytes(uart_port_num, (void *)read_buff,
                            (sizeof(read_buff) - 1), 10 * UART_DELAY);
    } while (ret == 0);

    if (ret == -1) {
      ESP_LOGE(uart_read_task_TAG.data(), "Uart Read Error");
      continue;
    }

    if (ret >= 99)
      read_buff[99] = '\0';
    else
      read_buff[ret] = '\0';

    if (read_buff[ret - 1] == '\n' || read_buff[ret - 1] == '\r') {
      shared_ptr temp = shared_data_ptr.load();
      if (temp == NULL) {

        temp = ((char *)pvPortMalloc(ret * sizeof(char)));
        memcpy(temp, read_buff, strlen(read_buff));
        ESP_LOGI(uart_read_task_TAG.data(), "You entered: %s",(read_buff));
        shared_data_size.store(strlen(read_buff));
        shared_data_ptr.store(temp);
        memset(read_buff, 0, strlen(read_buff)); // clear the local buffer
      }
    }

    vTaskDelay(UART_DELAY);
  }
  vTaskDelete(NULL);
}

extern "C" void app_main(void) {

  ESP_ERROR_CHECK(uart_init());

  BaseType_t xReturn0 =
      xTaskCreate(&uart_write_task, uart_write_task_TAG.data(), TASK_STACK_SIZE,
                  NULL, 1, &uart_write_task_handle);

  if (xReturn0 != pdPASS) {
    ESP_LOGE(app_main_task_TAG.data(), "Failed to create task: %s",
             led_task_TAG.data());
    vTaskDelete(NULL);
  }

  BaseType_t xReturn1 =
      xTaskCreate(&uart_read_task, uart_read_task_TAG.data(), TASK_STACK_SIZE,
                  NULL, 1, &uart_read_task_handle);

  if (xReturn1 != pdPASS) {
    ESP_LOGE("app_main", "Failed to create task: %s",
             uart_read_task_TAG.data());
    vTaskDelete(NULL);
  }
  vTaskDelete(NULL);
}
