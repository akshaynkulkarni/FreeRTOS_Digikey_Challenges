/**
 * FreeRTOS Counting Semaphore Challenge
 *
 * Challenge: use a mutex and counting semaphores to protect the shared buffer
 * so that each number (0 throguh 4) is printed exactly 3 times to the Serial
 * monitor (in any order). Do not use queues to do this!
 *
 * Hint: you will need 2 counting semaphores in addition to the mutex, one for
 * remembering number of filled slots in the buffer and another for
 * remembering the number of empty slots in the buffer.
 *
 * Date: January 24, 2021
 * Author: Shawn Hymel
 * License: 0BSD
 */

#include <atomic>
#include <cstring>
#include <iostream>
#include <stdlib.h>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/semphr.h"
// Use only core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

// Settings
enum { BUF_SIZE = 5 };                   // Size of buffer array
static const uint8_t num_prod_tasks = 5; // Number of producer tasks
static const uint8_t num_cons_tasks = 2; // Number of consumer tasks
static const uint8_t num_writes = 3; // Num times each producer writes to buf

// Globals
static uint8_t buf[BUF_SIZE];     // Shared buffer
static uint8_t head = 0;          // Writing index to buffer
static uint8_t tail = 0;          // Reading index to buffer
static SemaphoreHandle_t bin_sem; // Waits for parameter to be read

static SemaphoreHandle_t
    bin_sem1; // Waits for new produced buff element, in consumer task
static SemaphoreHandle_t
    bin_sem2; // Waits for the buff element to be consumed, in producer task
static SemaphoreHandle_t mutex; // To access the shared buffer

//*****************************************************************************
// Tasks

// Producer: write a given number of times to shared buffer
void producer(void *parameters) {

  // Copy the parameters into a local variable
  uint8_t num = *(uint8_t *)parameters;

  // Release the binary semaphore
  xSemaphoreGive(bin_sem);

  // Fill shared buffer with task number
  for (uint8_t i = 0; i < num_writes; i++) {
    xSemaphoreTake(bin_sem2, portMAX_DELAY); // Empty buffer?
    // Critical section (accessing shared buffer)
    xSemaphoreTake(mutex, portMAX_DELAY); // Take muex lock to access buffer
    buf[head] = num;
    head = (head + 1) % BUF_SIZE;
    xSemaphoreGive(mutex); // Release mutex lock of buffer
    xSemaphoreGive(
        bin_sem1); // Signal the consumer that new queue item is available
  }

  // Delete self task
  vTaskDelete(NULL);
}

// Consumer: continuously read from shared buffer
void consumer(void *parameters) {

  uint8_t val;

  // Read from buffer
  while (1) {

    // Critical section (accessing shared buffer and Serial)
    xSemaphoreTake(
        bin_sem1, portMAX_DELAY); // wait till new buff is produced in the queue

    xSemaphoreTake(
        mutex, portMAX_DELAY); // mutex to access the shared buff: acquire lock
    val = buf[tail];
    tail = (tail + 1) % BUF_SIZE;
    ESP_LOGI("consumer", " %d", val);
    xSemaphoreGive(mutex);    // mutex to access the shared buff: release lock
    xSemaphoreGive(bin_sem2); // signal the buff value consumption
    vTaskDelay(1);
  }
}

extern "C" void app_main(void) {
  char task_name[15] = {0};

  // Configure Serial
  // Serial.begin(115200);

  // Wait a moment to start (so we don't miss Serial output)
  // vTaskDelay(1000 / portTICK_PERIOD_MS);
  // Serial.println();
  ESP_LOGI("app_main", "---FreeRTOS Semaphore Alternate Solution---");

  // Create mutexes and semaphores before starting tasks
  bin_sem = xSemaphoreCreateBinary();
  bin_sem1 = xSemaphoreCreateCounting(num_prod_tasks, 0); // incrementing semaphore
  bin_sem2 = xSemaphoreCreateCounting(num_cons_tasks, num_cons_tasks); // decrementing semaphore
  mutex = xSemaphoreCreateMutex();

  // Start producer tasks (wait for each to read argument)
  for (uint8_t i = 0; i < num_prod_tasks; i++) {
    sprintf(task_name, "Producer%d", i);
    xTaskCreatePinnedToCore(producer, task_name, 4 * 1024, (void *)&i, 1, NULL,
                            app_cpu);
    xSemaphoreTake(bin_sem, portMAX_DELAY);
  }

  // Start consumer tasks
  for (uint8_t i = 0; i < num_cons_tasks; i++) {
    sprintf(task_name, "Consumer %d", i);
    xTaskCreatePinnedToCore(consumer, task_name, 4 * 1024, NULL, 1, NULL,
                            app_cpu);
  }

  // Notify that all tasks have been created
  ESP_LOGI("app_main", "All tasks created");

  while (1) {

    // Do nothing but allow yielding to lower-priority tasks
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
