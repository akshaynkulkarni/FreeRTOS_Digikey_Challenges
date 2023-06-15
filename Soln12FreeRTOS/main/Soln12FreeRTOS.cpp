#include <atomic>
#include <cstring>
#include <iostream>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "driver/adc.h"
#include "driver/timer.h"
#include "esp_adc_cal.h"

#include "esp_err.h"
#include "esp_log.h"

#define UART_DELAY (10U / portTICK_RATE_MS)
#define ADC_DELAY_US (100U * 1000U) // 100ms

#define TASK_STACK_SIZE (12U * 1024U) // 12kb

#define LED_PIN GPIO_NUM_13

#define DBG 0
#define DBG_ISR                                                                \
  0 // ISR: Do not call blocking statements in ISR !! Abort would occur

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

constexpr static uint8_t adc_core = 0U;
constexpr static uint8_t uart_core = 1U;

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

static int adc_task_core_value = -1;
static int adc_isr_core_value = -1;
static bool IRAM_ATTR ADC_Timer_ISR(void *params) {

  if (((head + 1) % buffer_len) == tail) {
    // drop the elements, buffer full, sorry
#if DBG_ISR // Do not call blocking statements in ISR !! Abort would occur
    ESP_LOGE("ISR", "Sorry! Buffer Full!! dropping adc_values....");
#endif
    // To indicate buffer full, glow an led
    gpio_set_level(LED_PIN, true);
    return pdFALSE;
  }
  gpio_set_level(LED_PIN, false);
  adc_isr_core_value = xPortGetCoreID();
#if DBG_ISR // Do not call blocking statements in ISR !! Abort would occur
// ESP_LOGI("ISR", "Executing from Core %d",xPortGetCoreID());
#endif
  static int buffer_mem_count = 0;

  if (buffer_mem_count < buffer_mem_len) {
    int l_buffer_mem_count = buffer_mem_count;
    // portENTER_CRITICAL_ISR(&spinlock);
    ring_buffer[head].adc_val[buffer_mem_count] = adc1_get_raw(ADC1_CHANNEL_5);
    buffer_mem_count++;
    // portEXIT_CRITICAL_ISR(&spinlock);
#if DBG_ISR // Do not call blocking statements in ISR !! Abort would occur
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
    // Returning xTaskWoken will do the job. That is, if pdTrue, Yield is
    // requested if (xTaskWoken) {
    //   portYIELD_FROM_ISR();
    // }
    return xTaskWoken;
  }
  return pdFALSE;
}

void ADC_post_processing_Task(void *params) {
  uint32_t adc_sum = 0;
  uint32_t l_tail = 0;
  adc_task_core_value = xPortGetCoreID();

  while (true) {
    if (xSemaphoreTake(signal_buff_full, portMAX_DELAY)) {
      adc_sum = 0;
      // ESP_LOGI("adc", "Executing from Core %d",xPortGetCoreID());
      //  portENTER_CRITICAL(&spinlock);
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
    // vTaskDelay(UART_DELAY);
  }
  vTaskDelete(NULL);
}

constexpr uint32_t timer_divider = 8U;
constexpr timer_group_t group_num = TIMER_GROUP_0;
constexpr timer_idx_t timer_num = TIMER_0;
constexpr uint64_t alarm_value = 1000000;

void hw_timer_init(void *params) {

  const timer_config_t conf = {.alarm_en = TIMER_ALARM_EN,
                               .counter_en = TIMER_PAUSE,
                               .counter_dir = TIMER_COUNT_UP,
                               .auto_reload = TIMER_AUTORELOAD_EN,
                               .divider = timer_divider};

  esp_err_t err = timer_init(TIMER_GROUP_0, TIMER_0, &conf);
  err |= timer_set_counter_value(group_num, timer_num, 0);
  err |= timer_set_alarm_value(group_num, timer_num, alarm_value);
  err |= timer_enable_intr(group_num, timer_num);
  err |= timer_isr_callback_add(group_num, timer_num, ADC_Timer_ISR, timer, 0);
  err |= timer_start(group_num, timer_num);

  if (err != ESP_OK) {
    ESP_LOGE("timer_init", "Error!!");
    esp_restart();
  }
  ESP_LOGI("timer_init", "Done");
  vTaskDelete(NULL);
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

  return ESP_OK;
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

        ESP_LOGI(
            uart_task_TAG.data(),
            "\nExecuting UART task from Core %d,\nTimer Interrupt from Core "
            "%d,\nADC Post Processing Task from Core %d",
            xPortGetCoreID(), adc_isr_core_value, adc_task_core_value);

        ESP_LOGI(uart_task_TAG.data(),
                 "\nADC RAW Average is %f,\nVoltage at Pin = %u mV", l_avg,
                 esp_adc_cal_raw_to_voltage(l_avg, &adc1_chars));
      }
    }
    vTaskDelay(2 * UART_DELAY);
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

extern "C" void app_main(void) {

  ESP_ERROR_CHECK(adc_init());
  ESP_ERROR_CHECK(led_init());
  ESP_ERROR_CHECK(uart_init());

  signal_buff_full = xSemaphoreCreateCounting(buffer_len, 0);
  avg_mutex = xSemaphoreCreateMutex();

  // The core for Timer ISR is implicitly set through this task, i.e.
  // Based on the core to which this task is pinned, the timer ISR
  // timer_isr_callback_add uses the same core to pin the ISR.
  BaseType_t xReturn = xTaskCreatePinnedToCore(
      &hw_timer_init, "HWTimerInit", TASK_STACK_SIZE, NULL, 1, NULL, adc_core);

  if (xReturn != pdPASS) {
    ESP_LOGE(app_main_task_TAG.data(), "Failed to create task: %s",
             "HWTimerInit");
    vTaskDelete(NULL);
  }

  BaseType_t xReturn0 = xTaskCreatePinnedToCore(
      &ADC_post_processing_Task, adc_processing_task_TAG.data(),
      TASK_STACK_SIZE, NULL, 2, &adc_processing_task_handle, adc_core);

  if (xReturn0 != pdPASS) {
    ESP_LOGE(app_main_task_TAG.data(), "Failed to create task: %s",
             adc_processing_task_TAG.data());
    adc_processing_task_handle = NULL;
    vTaskDelete(NULL);
  }

  BaseType_t xReturn1 =
      xTaskCreatePinnedToCore(&uart_task, uart_task_TAG.data(), TASK_STACK_SIZE,
                              NULL, 1, &uart_task_handle, uart_core);

  if (xReturn1 != pdPASS) {
    ESP_LOGE(app_main_task_TAG.data(), "Failed to create task: %s",
             uart_task_TAG.data());
    uart_task_handle = NULL;
    vTaskDelete(NULL);
  }

  vTaskDelete(NULL);
}
