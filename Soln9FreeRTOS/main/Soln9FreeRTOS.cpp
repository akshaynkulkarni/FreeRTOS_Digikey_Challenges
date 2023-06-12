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

#define UART_DELAY (10U / portTICK_RATE_MS)
#define ADC_DELAY_US (100U * 1000U) // 100ms

#define TASK_STACK_SIZE (12U * 1024U) // 12kb

#define DBG 0

TaskHandle_t adc_processing_task_handle = NULL;
TaskHandle_t uart_task_handle = NULL;

SemaphoreHandle_t signal_buff_full = NULL; // Signal a buffer is full
SemaphoreHandle_t avg_mutex =
    NULL; // sync between the adc processor task and uart task

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

// static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED; // CS is
// required if we are accessing an element of the buffer.
static portMUX_TYPE avg_spinlock = portMUX_INITIALIZER_UNLOCKED;
constexpr size_t buffer_len =
    4; // The ring buffer has 'n' buffers that can be configured. Min. 2
constexpr size_t buffer_mem_len = 10;

using buf = struct {
  uint16_t adc_val[buffer_mem_len];
};

volatile buf ring_buffer[buffer_len] = {0};
volatile uint8_t head = 0;
volatile uint8_t tail = 0;

void IRAM_ATTR ADC_Timer_ISR(void *params) {

  if (((head + 1) % buffer_len) == tail) {
    // drop the elements, buffer full, sorry
    ESP_LOGE("ISR", "Sorry! Buffer Full!! dropping adc_values....");
    return;
  }

  static int buffer_mem_count = 0;

  if (buffer_mem_count < buffer_mem_len) {
    int l_buffer_mem_count = buffer_mem_count;
    // portENTER_CRITICAL_ISR(&spinlock);
    ring_buffer[head].adc_val[buffer_mem_count] = adc1_get_raw(ADC1_CHANNEL_5);
    buffer_mem_count++;
    // portEXIT_CRITICAL_ISR(&spinlock);
#if DBG
    ESP_LOGI("ISR", "ring_buffer[%d].adc_val[%d] = %d", head,
             l_buffer_mem_count, ring_buffer[head].adc_val[l_buffer_mem_count]);
#endif
  }

  if (buffer_mem_count >= buffer_mem_len) {

    // portENTER_CRITICAL_ISR(&spinlock);
    head = ((head + 1) % buffer_len); // let's fill the next buffer element
    buffer_mem_count = 0;             // reset the member buffer count
    // portEXIT_CRITICAL_ISR(&spinlock);

    BaseType_t xTaskWoken = pdFALSE;
    // Signal a buffer is ready to be consumed
    xSemaphoreGiveFromISR(signal_buff_full, &xTaskWoken);
    // We need to request context change within ISR if the woken task has higher
    // priority over the current running task. ADC post processing task has
    // priority 2.
    if (xTaskWoken) {
      portYIELD_FROM_ISR();
    }
  }
}

void ADC_post_processing_Task(void *params) {
  uint32_t adc_sum = 0;
  uint32_t l_tail = 0;
  while (true) {
    if (xSemaphoreTake(signal_buff_full, portMAX_DELAY)) {
      adc_sum = 0;

      // portENTER_CRITICAL(&spinlock);
      for (uint8_t i = 0; i < buffer_mem_len; i++) {
        adc_sum += ring_buffer[tail].adc_val[i];

        // ESP_LOGI("adc", "ring_buffer[%d].adc_val[%d] = %d", tail, i,
        // ring_buffer[tail].adc_val[i]);
      }
      l_tail = tail;
      tail = ((tail + 1) % buffer_len);
      // portEXIT_CRITICAL(&spinlock);
#if DBG
      for (uint8_t i = 0; i < buffer_mem_len; i++) {
        ESP_LOGI("adc", "ring_buffer[%d].adc_val[%d] = %d", l_tail, i,
                 ring_buffer[l_tail].adc_val[i]);
      }
      ESP_LOGI("=========", "==========");
#endif
      portENTER_CRITICAL(&avg_spinlock);
      avg_adc.store((float)(((float)adc_sum / (buffer_mem_len))));
      portEXIT_CRITICAL(&avg_spinlock);
    }
    vTaskDelay(UART_DELAY);
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

        portENTER_CRITICAL(&avg_spinlock);
        float l_avg = avg_adc.load();
        portEXIT_CRITICAL(&avg_spinlock);
        ESP_LOGI(uart_task_TAG.data(), "Average is %f, Voltage at Pin = %u mV",
                 l_avg, esp_adc_cal_raw_to_voltage(l_avg, &adc1_chars));
      }
    }
    vTaskDelay(2 * UART_DELAY);
  }
  vTaskDelete(NULL);
}

extern "C" void app_main(void) {

  ESP_ERROR_CHECK(adc_init());
  ESP_ERROR_CHECK(uart_init());

  signal_buff_full = xSemaphoreCreateCounting(buffer_len, 0);
  avg_mutex = xSemaphoreCreateMutex();

  BaseType_t xReturn0 =
      xTaskCreate(&ADC_post_processing_Task, adc_processing_task_TAG.data(),
                  TASK_STACK_SIZE, NULL, 2, &adc_processing_task_handle);

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
