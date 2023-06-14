/**
 * ESP32 Dining Philosophers
 *
 * The classic "Dining Philosophers" problem in FreeRTOS form.
 *
 * Based on http://www.cs.virginia.edu/luther/COA2/S2019/pa05-dp.html
 *
 * Date: February 8, 2021
 * Author: Shawn Hymel
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
enum { NUM_TASKS = 5 }; // Number of tasks (philosophers)
enum {
  TASK_STACK_SIZE = 12 * 1024
}; // Bytes in ESP32, words in vanilla FreeRTOS

// Globals
static SemaphoreHandle_t bin_sem = NULL;  // Wait for parameters to be read
static SemaphoreHandle_t done_sem = NULL; // Notifies main task when done
static SemaphoreHandle_t chopstick[NUM_TASKS] = {NULL};

constexpr std::string_view Philosopher_task_TAG = "Philosopher_task";
constexpr std::string_view app_main_task_TAG = "app_main_task";
//*****************************************************************************
// Tasks

// The only task: eating
void eat(void *parameters) {

  uint8_t num = 0;

  uint8_t left = 0;
  uint8_t right = 0;
  // Copy parameter and increment semaphore count
  num = *(uint8_t *)parameters;
  xSemaphoreGive(bin_sem);

  if (num < ((num + 1) % NUM_TASKS)) {
    left = num;
    right = (num + 1) % NUM_TASKS;
  } else {
    left = (num + 1) % NUM_TASKS;
    right = num;
  }

  // Take left chopstick
  xSemaphoreTake(chopstick[left], portMAX_DELAY);
  ESP_LOGI(Philosopher_task_TAG.data(), "Philosopher %u took chopstick %u", num,
           left);

  // Add some delay to force deadlock
  vTaskDelay(1 / portTICK_PERIOD_MS);

  // Take right chopstick
  xSemaphoreTake(chopstick[right], portMAX_DELAY);
  ESP_LOGI(Philosopher_task_TAG.data(), "Philosopher %u took chopstick %u", num,
           right);

  // Do some eating
  ESP_LOGI(Philosopher_task_TAG.data(), "Philosopher %u is eating", num);
  vTaskDelay(100 / portTICK_PERIOD_MS);

  // Put down right chopstick
  xSemaphoreGive(chopstick[right]);
  ESP_LOGI(Philosopher_task_TAG.data(), "Philosopher %u returned chopstick %u",
           num, right);

  // Put down left chopstick
  xSemaphoreGive(chopstick[left]);
  ESP_LOGI(Philosopher_task_TAG.data(), "Philosopher %u returned chopstick %u",
           num, left);

  // Notify main task and delete self
  xSemaphoreGive(done_sem);
  vTaskDelete(NULL);
}

extern "C" void app_main(void) {

  char task_name[30] = {0};

  ESP_LOGI(app_main_task_TAG.data(),
           "---FreeRTOS Dining Philosophers Challenge---");

  // Create kernel objects before starting tasks
  bin_sem = xSemaphoreCreateBinary();
  done_sem = xSemaphoreCreateCounting(NUM_TASKS, 0);

  for (int i = 0; i < NUM_TASKS; i++) {
    chopstick[i] = xSemaphoreCreateMutex();
  }

  // Have the philosphers start eating
  for (uint8_t i = 0; i < NUM_TASKS; i++) {
    sprintf(task_name, "Philosopher %u", i);
    xTaskCreatePinnedToCore(&eat, task_name, TASK_STACK_SIZE, (void *)&i, 1,
                            NULL, app_cpu);
    xSemaphoreTake(bin_sem, portMAX_DELAY);
  }

  // Wait until all the philosophers are done
  for (int i = 0; i < NUM_TASKS; i++) {
    xSemaphoreTake(done_sem, portMAX_DELAY);
  }

  // Say that we made it through without deadlock
  ESP_LOGI(app_main_task_TAG.data(), "Done! No deadlock occurred!");

  vTaskDelete(NULL);
}
