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

#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "esp_timer.h"

SemaphoreHandle_t signal_buff_full_bin = NULL;
SemaphoreHandle_t signal_buff_empty_bin = NULL;
SemaphoreHandle_t avg_mutex = NULL;

#define UART_DELAY (10U / portTICK_RATE_MS)
#define ADC_DELAY_US (100U * 1000U) // 100ms

#define TASK_STACK_SIZE (12U * 1024U) // 12kb

TaskHandle_t adc_processing_task_handle = NULL;
TaskHandle_t uart_task_handle = NULL;

constexpr std::string_view adc_processing_task_TAG = "ADC_processing_task";
constexpr std::string_view uart_task_TAG = "uart_task";
constexpr std::string_view app_main_task_TAG = "app_main_write_task";

constexpr uart_port_t uart_port_num = UART_NUM_0;
constexpr size_t uart_buff_size = 512U;
constexpr size_t uart_queue_len = 10U;
QueueHandle_t uart_queue;

std::atomic<float> avg_adc = {0.0f};

static esp_timer_handle_t timer = NULL;
static esp_adc_cal_characteristics_t adc1_chars;

const uint8_t c_queue_len = 10;
volatile uint32_t c_queue[c_queue_len];
uint8_t head = 0;
uint8_t tail = 0;

void IRAM_ATTR ADC_Timer_ISR(void *params) {
  static uint8_t buffer_count = 0;
  if (buffer_count >= c_queue_len) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (xSemaphoreGiveFromISR(signal_buff_empty_bin,
                              &xHigherPriorityTaskWoken) == pdTRUE) {
      buffer_count = 0;
    }
    return;
  }

  // Producer: insert the raw adc value into the queue

  if (buffer_count < c_queue_len) {
    c_queue[head] = adc1_get_raw(ADC1_CHANNEL_5);
    head = (head + 1) % c_queue_len;
    buffer_count++;
  }
  if (buffer_count >= c_queue_len) {
    // signal buffer 1 is full
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(signal_buff_full_bin, &xHigherPriorityTaskWoken);
  }
}

void ADC_post_processing_Task(void *params) {
  uint32_t adc_sum = 0;
  while (true) {
    if (xSemaphoreTake(signal_buff_full_bin, portMAX_DELAY)) {
      adc_sum = 0;
      // consumer task
      for (uint8_t i = 0; i < c_queue_len; i++) {
        adc_sum += c_queue[tail];
        tail = ((tail + 1) % c_queue_len);
      }

      xSemaphoreTake(signal_buff_empty_bin, portMAX_DELAY);

      if (xSemaphoreTake(avg_mutex, portMAX_DELAY)) {
        avg_adc.store((float)(((float)adc_sum / (c_queue_len))));
        xSemaphoreGive(avg_mutex);
      }
    }
    // vTaskDelay(UART_DELAY);
  }
  vTaskDelete(NULL);
}

esp_err_t hw_timer_init() {
  const esp_timer_create_args_t timer_args = {.callback = &ADC_Timer_ISR,
                                              .arg = NULL,
                                              .dispatch_method = ESP_TIMER_TASK,
                                              .name = "ADC_Timer_ISR",
                                              .skip_unhandled_events = false};

  esp_err_t err = esp_timer_create(&timer_args, &timer);
  if (ESP_OK != err) {
    ESP_LOGE("hw_timer_init", "Could not create HW timer");
    return err;
  }
  err = esp_timer_start_periodic(timer, ADC_DELAY_US);
  if (ESP_OK != err) {
    ESP_LOGE("hw_timer_init", "Could not start HW timer");
    return err;
  }
  return ESP_OK;
}

esp_err_t adc_init() {
  esp_adc_cal_value_t adc_cal_value = esp_adc_cal_characterize(
      ADC_UNIT_1, ADC_ATTEN_DB_11,
      static_cast<adc_bits_width_t>(ADC_WIDTH_BIT_DEFAULT), 0, &adc1_chars);
  if (ESP_ADC_CAL_VAL_EFUSE_VREF == adc_cal_value)
    ESP_LOGI("adc_init",
             "Characterization based on reference voltage stored in eFuse");
  else if (ESP_ADC_CAL_VAL_EFUSE_TP == adc_cal_value)
    ESP_LOGI("adc_init",
             "Characterization based on Two Point values stored in eFuse");
  else if (ESP_ADC_CAL_VAL_DEFAULT_VREF == adc_cal_value)
    ESP_LOGI("adc_init",
             "Characterization based on Two Point values stored in eFuse");
  else {
    ESP_LOGE("adc_init", "Cannot characterize ADC");
    return ESP_FAIL;
  }
  esp_err_t err =
      adc1_config_width(static_cast<adc_bits_width_t>(ADC_WIDTH_BIT_DEFAULT));
  if (err != ESP_OK) {
    ESP_LOGE("adc_init", "Error Config width");
  }
  err = (adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_11));

  if (err != ESP_OK) {
    ESP_LOGI("adc_init", "Error in config_channel_atten: %#x", err);
    return err;
  }

  return hw_timer_init();
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
                            (sizeof(read_buff) - 1), 50 * UART_DELAY);
    } while (ret == 0);

    if (ret == -1) {
      ESP_LOGE(uart_task_TAG.data(), "UART Read Error");
      continue;
    }

    if (ret) {
      uart_write_bytes(uart_port_num, (const char *)"\r", strlen("\r"));
      uart_write_bytes(uart_port_num, (const char *)read_buff,
                       strlen(read_buff));
      uart_write_bytes(uart_port_num, (const char *)"\n\r", strlen("\n\r"));

      if (!strcmp("avg\r", read_buff) || !strcmp("avg\n", read_buff)) {
        if (xSemaphoreTake(avg_mutex, portMAX_DELAY)) {
          ESP_LOGI(uart_task_TAG.data(), "Average is %f", avg_adc.load());
          xSemaphoreGive(avg_mutex);
        }
      }
    }
    vTaskDelay(2 * UART_DELAY);
  }
  vTaskDelete(NULL);
}

extern "C" void app_main(void) {

  ESP_ERROR_CHECK(adc_init());
  ESP_ERROR_CHECK(uart_init());

  signal_buff_full_bin = xSemaphoreCreateBinary();
  signal_buff_empty_bin = xSemaphoreCreateBinary();
  avg_mutex = xSemaphoreCreateMutex();

  BaseType_t xReturn0 =
      xTaskCreate(&ADC_post_processing_Task, adc_processing_task_TAG.data(),
                  TASK_STACK_SIZE, NULL, 1, &adc_processing_task_handle);

  if (xReturn0 != pdPASS) {
    ESP_LOGE(app_main_task_TAG.data(), "Failed to create task: %s",
             adc_processing_task_TAG.data());
    adc_processing_task_handle = NULL;
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
