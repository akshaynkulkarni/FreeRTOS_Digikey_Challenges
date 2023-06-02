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
#define LED_DELAY_DEF (500U)

#define TASK_STACK_SIZE (12U * 1024U) // 12kb

#define LED_PIN GPIO_NUM_13

TaskHandle_t led_task_handle = NULL;
TaskHandle_t uart_task_handle = NULL;
TaskHandle_t uart_write_task_handle = NULL;

constexpr std::string_view led_toggle_task_TAG = "led_toggle_task";
constexpr std::string_view uart_task_TAG = "uart_task";
constexpr std::string_view app_main_task_TAG = "app_main_write_task";

constexpr uart_port_t uart_port_num = UART_NUM_0;
constexpr size_t uart_buff_size = 512U;
constexpr size_t uart_queue_len = 10U;
QueueHandle_t uart_queue;

constexpr size_t msg1_queue_len = 10U;
constexpr size_t msg2_queue_len = 10U;

using msg1_t = uint32_t;
using msg2_t = struct msg2_st {
  char bmsg[8];
  uint32_t blinks;
};
constexpr size_t msg1_size = sizeof(msg1_t);
constexpr size_t msg2_size = sizeof(msg2_t);

QueueHandle_t msg1_queue_handle = NULL;
QueueHandle_t msg2_queue_handle = NULL;

void led_toggle_task(void *params) {

  bool led_status = false;
  static uint32_t led_blink_count = 0;
  static msg2_t msg = {.bmsg = "Blinked", .blinks = led_blink_count};
  static msg1_t msg_buff = LED_DELAY_DEF;

  while (true) {
    led_status = !led_status;
    gpio_set_level(LED_PIN, led_status);
    led_blink_count++;
    if (led_blink_count % 100 == 0) {
      msg.blinks = led_blink_count;
      if (!xQueueSend(msg2_queue_handle, (const void *)&msg, 0))
        ESP_LOGE(led_toggle_task_TAG.data(), "Error Sending Queue for msg2");
    }
  //  ESP_LOGI(led_toggle_task_TAG.data(), "I am LED");

    if (xQueueReceive(msg1_queue_handle, &msg_buff, 0)) {
      ESP_LOGI(led_toggle_task_TAG.data(),
               " msg1 recived: update delay time: %d", msg_buff);
      led_blink_count = 0;
    }

    vTaskDelay(msg_buff / portTICK_RATE_MS);
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
      if (ret) {
        uart_write_bytes(uart_port_num, (const char *)"\r", strlen("\r"));
        uart_write_bytes(uart_port_num, (const char *)read_buff,
                         strlen(read_buff));
        uart_write_bytes(uart_port_num, (const char *)"\n\r", strlen("\n\r"));
      }
    static msg2_t msg_buff;
    if (xQueueReceive(msg2_queue_handle, &msg_buff, 0))
      ESP_LOGI(uart_task_TAG.data(), "Message from msg2: %s: %d", msg_buff.bmsg,
               msg_buff.blinks);
    } while (ret == 0);

    if (ret == -1) {
      ESP_LOGE(uart_task_TAG.data(), "Uart Read Error");
      continue;
    }

    if (ret >= 99)
      read_buff[99] = '\0';
    else
      read_buff[ret] = '\0';

    static int delay_time = 0;
    const char *delay_str = "delay";
    char delay_scanned[strlen(delay_str)] = {'0'};

    sscanf(read_buff, "%s %d", delay_scanned, &delay_time);

    if (!strcmp(delay_str, delay_scanned)) {
      // insert the delay
      xQueueSend(msg1_queue_handle, (const void *)&delay_time, 0);
    }

    vTaskDelay(UART_DELAY);
  }
  vTaskDelete(NULL);
}

extern "C" void app_main(void) {

  ESP_ERROR_CHECK(led_init());
  ESP_ERROR_CHECK(uart_init());

  msg1_queue_handle = xQueueCreate(msg1_queue_len, msg1_size);
  msg2_queue_handle = xQueueCreate(msg2_queue_len, msg2_size);

  if (msg1_queue_handle == NULL || msg2_queue_handle == NULL) {
    ESP_LOGE(app_main_task_TAG.data(), "Error creating queue!");
    vTaskDelete(NULL);
  }

  BaseType_t xReturn0 =
      xTaskCreate(&led_toggle_task, led_toggle_task_TAG.data(), TASK_STACK_SIZE,
                  NULL, 1, &led_task_handle);

  if (xReturn0 != pdPASS) {
    ESP_LOGE(app_main_task_TAG.data(), "Failed to create task: %s",
             led_toggle_task_TAG.data());
    vTaskDelete(led_task_handle);
    vTaskDelete(NULL);
  }

  BaseType_t xReturn1 =
      xTaskCreate(&uart_task, uart_task_TAG.data(), TASK_STACK_SIZE, NULL, 1,
                  &uart_task_handle);

  if (xReturn1 != pdPASS) {
    ESP_LOGE(app_main_task_TAG.data(), "Failed to create task: %s",
             uart_task_TAG.data());
    vTaskDelete(uart_task_handle);
    vTaskDelete(NULL);
  }
  vTaskDelete(NULL);
}
