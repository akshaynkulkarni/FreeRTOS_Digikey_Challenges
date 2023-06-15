/**
 * ESP32 Priority Inversion Demo
 *
 * Demonstrate priority inversion.
 *
 * Date: Jun 15, 2023
 * Author: Shawn Hymel, Akshay N Kulkarni
 * License: 0BSD
 */

#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/semphr.h"

// You'll likely need this on vanilla FreeRTOS
// #include semphr.h

// Use only core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

// Settings
TickType_t cs_wait = 250;   // Time spent in critical section (ms)
TickType_t med_wait = 1750; // Time medium task spends working (ms)

// Globals
// static SemaphoreHandle_t lock = NULL;

static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;

#define STACK_SIZE 12 * 1024

//*****************************************************************************//
// Tasks

// Task L (low priority)

constexpr std::string_view doTaskL_task_TAG = "doTaskL_task";

void doTaskL(void *parameters) {

  TickType_t timestamp;

  // Do forever
  while (1) {

    // Take lock
    ESP_LOGI(doTaskL_task_TAG.data(), "Task L trying to take lock...");
    TickType_t timestamp1 = xTaskGetTickCount() * portTICK_PERIOD_MS;
    // xSemaphoreTake(lock, portMAX_DELAY);

    // Enter CS
    portENTER_CRITICAL(&spinlock);
    timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    timestamp1 = timestamp - timestamp1;
    // Hog the processor for a while doing nothing
    while ((xTaskGetTickCount() * portTICK_PERIOD_MS) - timestamp < cs_wait)
      ;
    portEXIT_CRITICAL(&spinlock);
    // Say how long we spend waiting for a lock
    ESP_LOGI(doTaskL_task_TAG.data(),
             "Task L got lock. Spent %d ms waiting for lock. Did some work and "
             "released lock...",
             timestamp1);
    // Release lock
    // xSemaphoreGive(lock);

    // Go to sleep
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

constexpr std::string_view doTaskM_task_TAG = "doTaskM_task";
// Task M (medium priority)
void doTaskM(void *parameters) {

  TickType_t timestamp;

  // Do forever
  while (1) {

    // Hog the processor for a while doing nothing
    ESP_LOGI(doTaskM_task_TAG.data(), "Task M doing some work...");
    timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    while ((xTaskGetTickCount() * portTICK_PERIOD_MS) - timestamp < med_wait)
      ;

    // Go to sleep
    ESP_LOGI(doTaskM_task_TAG.data(), "Task M done!");
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

constexpr std::string_view doTaskH_task_TAG = "doTaskH_task";

// Task H (high priority)
void doTaskH(void *parameters) {

  TickType_t timestamp;

  // Do forever
  while (1) {

    // Take lock
    ESP_LOGI(doTaskH_task_TAG.data(), "Task H trying to take lock...");
    TickType_t timestamp1 = xTaskGetTickCount() * portTICK_PERIOD_MS;
    // xSemaphoreTake(lock, portMAX_DELAY);

    portENTER_CRITICAL(&spinlock);
    timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    timestamp1 = timestamp - timestamp1;
    // Hog the processor for a while doing nothing
    while ((xTaskGetTickCount() * portTICK_PERIOD_MS) - timestamp < cs_wait)
      ;
    portEXIT_CRITICAL(&spinlock);

    // Say how long we spend waiting for a lock

    ESP_LOGI(doTaskH_task_TAG.data(),
             "Task H got lock. Spent %d ms waiting for lock. Did some work and "
             "released lock...",
             timestamp1);
    // Release lock
    // ESP_LOGI(doTaskH_task_TAG.data(), "Task H releasing lock.");
    // xSemaphoreGive(lock);

    // Go to sleep
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

//*****************************************************************************//
// Main (runs as its own task with priority 1 on core 1)

constexpr std::string_view app_main_task_TAG = "app_main_task";

extern "C" void app_main(void) {

  ESP_LOGI(app_main_task_TAG.data(), "---FreeRTOS Priority Inversion Demo---");

  //  Create semaphores and mutexes before starting tasks
  // lock = xSemaphoreCreateBinary();
  // xSemaphoreGive(lock); // Make sure binary semaphore starts at 1

  // The order of starting the tasks matters to force priority inversion

  // Start Task L (low priority)
  xTaskCreatePinnedToCore(doTaskL, "Task L", STACK_SIZE, NULL, 1, NULL,
                          app_cpu);

  // Introduce a delay to force priority inversion
  vTaskDelay(250 / portTICK_PERIOD_MS);

  // Start Task H (high priority)
  xTaskCreatePinnedToCore(doTaskH, "Task H", STACK_SIZE, NULL, 3, NULL,
                          app_cpu);

  // Start Task M (medium priority)
  xTaskCreatePinnedToCore(doTaskM, "Task M", STACK_SIZE, NULL, 2, NULL,
                          app_cpu);

  // Delete "setup and loop" task
  vTaskDelete(NULL);
}
