//ESP32-WROOM Configuration Note:
//CPU Frequency (240MHz) and Single-Core mode have been successfully 
//configured via ESP-IDF menuconfig prior to building and flashing.

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "esp_timer.h"
#include "esp_attr.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "monitor.h"

extern uint32_t WorkKernel(uint32_t budget_cycles, uint32_t seed);

//GPIO Pin Definitions 
#define GPIO_SYNC     4
#define GPIO_IN_A     16
#define GPIO_IN_B     17
#define GPIO_IN_S     18
#define GPIO_IN_MODE  19

#define GPIO_ACK_A    21
#define GPIO_ACK_B    22
#define GPIO_ACK_AGG  23
#define GPIO_ACK_C    25
#define GPIO_ACK_D    26
#define GPIO_ACK_S    27

//Task Budgets 
#define BUDGET_A    672000U
#define BUDGET_B    960000U
#define BUDGET_AGG  480000U
#define BUDGET_C   1680000U
#define BUDGET_D    960000U
#define BUDGET_S    600000U

//Periods in us 
#define PERIOD_A_US    10000ULL
#define PERIOD_B_US    20000ULL
#define PERIOD_AGG_US  20000ULL
#define PERIOD_C_US    50000ULL
#define PERIOD_D_US    50000ULL

//Stack Sizes 
#define STACK_SZ_A      4096
#define STACK_SZ_B      4096
#define STACK_SZ_AGG    4096
#define STACK_SZ_C      4096
#define STACK_SZ_D      4096
#define STACK_SZ_S      4096
#define STACK_SZ_MON    3072

//Priorities 
#define PRIO_A     6
#define PRIO_B     5
#define PRIO_AGG   4
#define PRIO_C     3
#define PRIO_D     3
#define PRIO_MON   2
#define PRIO_S     1


#define TASK_RUN_CORE 0

//O(1) Deterministic Edge Counting 
static volatile uint32_t hw_edge_count_A = 0;
static volatile uint32_t hw_edge_count_B = 0;

static void IRAM_ATTR isr_edge_a(void* arg) { hw_edge_count_A++; }
static void IRAM_ATTR isr_edge_b(void* arg) { hw_edge_count_B++; }

static uint32_t prev_edge_A = 0;
static uint32_t prev_edge_B = 0;

//Global Time Base 
static volatile uint64_t base_time_us = 0;
static volatile bool g_system_started = false;

static void IRAM_ATTR sync_isr(void* arg) {
    if (base_time_us == 0) {
        base_time_us = esp_timer_get_time();
    }
}

//Task Handles 
static TaskHandle_t g_taskA_handle   = NULL;
static TaskHandle_t g_taskB_handle   = NULL;
static TaskHandle_t g_taskAGG_handle = NULL;
static TaskHandle_t g_taskC_handle   = NULL;
static TaskHandle_t g_taskD_handle   = NULL;
static TaskHandle_t g_taskS_handle   = NULL;
static TaskHandle_t g_taskMON_handle = NULL;

//Synchronization
//Protect shared published/token data used by A/B/AGG
static SemaphoreHandle_t g_token_mutex = NULL;

//Task States
static uint32_t idxA   = 0;
static uint32_t idxB   = 0;
static uint32_t idxAGG = 0;
static uint32_t idxS   = 0;

static uint32_t idxC_log = 0;
static uint32_t idxD_log = 0;

static uint32_t last_tokenA = 0;
static uint32_t last_tokenB = 0;
static bool published_A = false;
static bool published_B = false;

//Helpers
static void reset_application_state(void) {
    idxA = 0;
    idxB = 0;
    idxAGG = 0;
    idxS = 0;
    idxC_log = 0;
    idxD_log = 0;

    last_tokenA = 0;
    last_tokenB = 0;
    published_A = false;
    published_B = false;

    prev_edge_A = hw_edge_count_A;
    prev_edge_B = hw_edge_count_B;
}
//gpio_set_level
static void wait_until_release_us(uint64_t target_us) {
    while (1) {
        int64_t remaining = (int64_t)target_us - (int64_t)esp_timer_get_time();
        if (remaining <= 0) {
            return;
        }

        if (remaining > 2000) {
            TickType_t ticks = pdMS_TO_TICKS((remaining - 1000) / 1000);
            if (ticks < 1) ticks = 1;
            vTaskDelay(ticks);
        } else if (remaining > 200) {
            esp_rom_delay_us((uint32_t)(remaining - 100));
        } else {
            while ((int64_t)esp_timer_get_time() < (int64_t)target_us) {

            }
            return;
        }
    }
}

static void set_all_acks_low(void) {
    gpio_set_level(GPIO_ACK_A, 0);
    gpio_set_level(GPIO_ACK_B, 0);
    gpio_set_level(GPIO_ACK_AGG, 0);
    gpio_set_level(GPIO_ACK_C, 0);
    gpio_set_level(GPIO_ACK_D, 0);
    gpio_set_level(GPIO_ACK_S, 0);
}

//ISR for Task S
static void IRAM_ATTR sporadic_isr(void* arg) {
    if (!g_system_started) {
        return; //ignore sporadic releases before SYNC
    }

    notifySRelease();

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (g_taskS_handle != NULL) {
        vTaskNotifyGiveFromISR(g_taskS_handle, &xHigherPriorityTaskWoken);
    }
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

//Task Bodies

//Task A
static void run_task_A(void) {
    beginTaskA(idxA);
    gpio_set_level(GPIO_ACK_A, 1);

    uint32_t current_A = hw_edge_count_A;
    uint32_t countA = current_A - prev_edge_A;
    prev_edge_A = current_A;

    uint32_t seed = ((idxA & 0xFFFFu) << 16) ^ countA ^ 0xA1u;
    uint32_t token = WorkKernel(BUDGET_A, seed);

    xSemaphoreTake(g_token_mutex, portMAX_DELAY);
    last_tokenA = token;
    published_A = true;
    xSemaphoreGive(g_token_mutex);

    gpio_set_level(GPIO_ACK_A, 0);
    endTaskA();

    printf("A,%u,%u,%u\n", (unsigned)idxA, (unsigned)countA, (unsigned)token);
    idxA++;
}

//Task B
static void run_task_B(void) {
    beginTaskB(idxB);
    gpio_set_level(GPIO_ACK_B, 1);

    uint32_t current_B = hw_edge_count_B;
    uint32_t countB = current_B - prev_edge_B;
    prev_edge_B = current_B;

    uint32_t seed = ((idxB & 0xFFFFu) << 16) ^ countB ^ 0xB2u;
    uint32_t token = WorkKernel(BUDGET_B, seed);

    xSemaphoreTake(g_token_mutex, portMAX_DELAY);
    last_tokenB = token;
    published_B = true;
    xSemaphoreGive(g_token_mutex);

    gpio_set_level(GPIO_ACK_B, 0);
    endTaskB();

    printf("B,%u,%u,%u\n", (unsigned)idxB, (unsigned)countB, (unsigned)token);
    idxB++;
}

//Task AGG
static void run_task_AGG(void) {
    beginTaskAGG(idxAGG);
    gpio_set_level(GPIO_ACK_AGG, 1);

    uint32_t tokenA_snapshot = 0;
    uint32_t tokenB_snapshot = 0;
    bool pubA = false;
    bool pubB = false;

    xSemaphoreTake(g_token_mutex, portMAX_DELAY);
    tokenA_snapshot = last_tokenA;
    tokenB_snapshot = last_tokenB;
    pubA = published_A;
    pubB = published_B;
    xSemaphoreGive(g_token_mutex);

    uint32_t agg;
    if (pubA && pubB) {
        agg = tokenA_snapshot ^ tokenB_snapshot;
    } else {
        agg = 0xDEADBEEFu;
    }

    uint32_t seed = ((idxAGG & 0xFFFFu) << 16) ^ agg ^ 0xD4u;
    uint32_t token = WorkKernel(BUDGET_AGG, seed);

    gpio_set_level(GPIO_ACK_AGG, 0);
    endTaskAGG();

    printf("AGG,%u,%u,%u\n", (unsigned)idxAGG, (unsigned)agg, (unsigned)token);
    idxAGG++;
}

//Task C
static void run_task_C(uint32_t monitor_slot) {
    beginTaskC(monitor_slot);
    gpio_set_level(GPIO_ACK_C, 1);

    uint32_t seed = ((idxC_log & 0xFFFFu) << 16) ^ 0xC3u;
    uint32_t token = WorkKernel(BUDGET_C, seed);

    gpio_set_level(GPIO_ACK_C, 0);
    endTaskC();

    printf("C,%u,%u\n", (unsigned)idxC_log, (unsigned)token);
    idxC_log++;
}

//Task D
static void run_task_D(uint32_t monitor_slot) {
    beginTaskD(monitor_slot);
    gpio_set_level(GPIO_ACK_D, 1);

    uint32_t seed = ((idxD_log & 0xFFFFu) << 16) ^ 0xD5u;
    uint32_t token = WorkKernel(BUDGET_D, seed);

    gpio_set_level(GPIO_ACK_D, 0);
    endTaskD();

    printf("D,%u,%u\n", (unsigned)idxD_log, (unsigned)token);
    idxD_log++;
}

//Task S
static void run_task_S(void) {
    beginTaskS(idxS);
    gpio_set_level(GPIO_ACK_S, 1);

    uint32_t seed = ((idxS & 0xFFFFu) << 16) ^ 0x55u;
    uint32_t token = WorkKernel(BUDGET_S, seed);

    gpio_set_level(GPIO_ACK_S, 0);
    endTaskS();

    printf("S,%u,%u\n", (unsigned)idxS, (unsigned)token);
    idxS++;
}

//FreeRTOS Task Wrappers 
static void task_A_entry(void *arg) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);	//wait for global start after SYNC


    uint32_t k = 0;
    while (1) {
        uint64_t release_us = base_time_us + ((uint64_t)k * PERIOD_A_US);
        wait_until_release_us(release_us);
        run_task_A();
        k++;
    }
}

static void task_B_entry(void *arg) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);	//wait for global start after SYNC

    uint32_t k = 0;
    while (1) {
        uint64_t release_us = base_time_us + ((uint64_t)k * PERIOD_B_US);
        wait_until_release_us(release_us);
        run_task_B();
        k++;
    }
}

static void task_AGG_entry(void *arg) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);	//wait for global start after SYNC

    uint32_t k = 0;
    while (1) {
        uint64_t release_us = base_time_us + ((uint64_t)k * PERIOD_AGG_US);
        wait_until_release_us(release_us);
        run_task_AGG();
        k++;
    }
}
static void task_C_entry(void *arg) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);	//wait for global start after SYNC

    uint32_t release_slot = 0;
    while (1) {
        uint64_t release_us = base_time_us + ((uint64_t)release_slot * PERIOD_C_US);
        wait_until_release_us(release_us);

        if (gpio_get_level(GPIO_IN_MODE) == 1) {
            run_task_C(release_slot);
        }

        release_slot++;
    }
}

static void task_D_entry(void *arg) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);	//wait for global start after SYNC

    uint32_t release_slot = 0;
    while (1) {
        uint64_t release_us = base_time_us + ((uint64_t)release_slot * PERIOD_D_US);
        wait_until_release_us(release_us);

        if (gpio_get_level(GPIO_IN_MODE) == 1) {
            run_task_D(release_slot);
        }

        release_slot++;
    }
}

static void task_S_entry(void *arg) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);//wait for global start after SYNC

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        run_task_S();
    }
}

static void task_monitor_entry(void *arg) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  //wait for global start after SYNC

    //Wait until T0 + 2s
    wait_until_release_us(base_time_us + 2000000ULL);

    g_system_started = false;
    gpio_intr_disable(GPIO_IN_S);

    //Suspend all worker tasks before printing the report
    if (g_taskA_handle)   vTaskSuspend(g_taskA_handle);
    if (g_taskB_handle)   vTaskSuspend(g_taskB_handle);
    if (g_taskAGG_handle) vTaskSuspend(g_taskAGG_handle);
    if (g_taskC_handle)   vTaskSuspend(g_taskC_handle);
    if (g_taskD_handle)   vTaskSuspend(g_taskD_handle);
    if (g_taskS_handle)   vTaskSuspend(g_taskS_handle);

    set_all_acks_low();

    //print final report
    monitorPrintFinalReport();
	printf("\nFINAL REPORT GENERATED \n");
	printf("Please take a screenshot of the logs above for your submission.\n");


    vTaskSuspend(NULL);
}

void app_main(void) {
    //Monitor Initialization
    monitorInit();
    monitorSetPeriodicReportEverySeconds(0);
    monitorSetFinalReportAfterSeconds(2);

    //GPIO Configurations
    gpio_config_t in_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << GPIO_SYNC) |
                        (1ULL << GPIO_IN_A) |
                        (1ULL << GPIO_IN_B) |
                        (1ULL << GPIO_IN_S) |
                        (1ULL << GPIO_IN_MODE),
        .pull_up_en = 0,
        .pull_down_en = 1
    };
    gpio_config(&in_cfg);

    gpio_config_t out_cfg = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << GPIO_ACK_A) |
                        (1ULL << GPIO_ACK_B) |
                        (1ULL << GPIO_ACK_AGG) |
                        (1ULL << GPIO_ACK_C) |
                        (1ULL << GPIO_ACK_D) |
                        (1ULL << GPIO_ACK_S),
        .pull_up_en = 0,
        .pull_down_en = 0
    };
    gpio_config(&out_cfg);

    set_all_acks_low();

    //sync objects
    g_token_mutex = xSemaphoreCreateMutex();
    if (g_token_mutex == NULL) {
        printf("ERROR: failed to create token mutex.\n");
        while (1) { vTaskDelay(portMAX_DELAY); }
    }

    //ISR Registration
    gpio_install_isr_service(0);

    gpio_set_intr_type(GPIO_SYNC, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(GPIO_IN_A, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(GPIO_IN_B, GPIO_INTR_POSEDGE);
    gpio_set_intr_type(GPIO_IN_S, GPIO_INTR_POSEDGE);

    gpio_isr_handler_add(GPIO_SYNC, sync_isr, NULL);
    gpio_isr_handler_add(GPIO_IN_A, isr_edge_a, NULL);
    gpio_isr_handler_add(GPIO_IN_B, isr_edge_b, NULL);
    gpio_isr_handler_add(GPIO_IN_S, sporadic_isr, NULL);

    //Create RTOS tasks and they will block until start notification
    xTaskCreatePinnedToCore(task_A_entry, "taskA", STACK_SZ_A, NULL, PRIO_A, &g_taskA_handle, TASK_RUN_CORE);
    xTaskCreatePinnedToCore(task_B_entry, "taskB", STACK_SZ_B, NULL, PRIO_B, &g_taskB_handle, TASK_RUN_CORE);
    xTaskCreatePinnedToCore(task_AGG_entry, "taskAGG", STACK_SZ_AGG, NULL, PRIO_AGG, &g_taskAGG_handle, TASK_RUN_CORE);
    xTaskCreatePinnedToCore(task_C_entry, "taskC", STACK_SZ_C, NULL, PRIO_C, &g_taskC_handle, TASK_RUN_CORE);
    xTaskCreatePinnedToCore(task_D_entry, "taskD", STACK_SZ_D, NULL, PRIO_D, &g_taskD_handle, TASK_RUN_CORE);
    xTaskCreatePinnedToCore(task_S_entry, "taskS", STACK_SZ_S, NULL, PRIO_S, &g_taskS_handle, TASK_RUN_CORE);
    xTaskCreatePinnedToCore(task_monitor_entry, "taskMON", STACK_SZ_MON, NULL, PRIO_MON, &g_taskMON_handle, TASK_RUN_CORE);

    //Wait for first sync
    printf("Waiting for first SYNC rising edge to set base T0...\n");
    while (base_time_us == 0) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    printf("Base T0 set at %llu us\n", (unsigned long long)base_time_us);

    //Reset monitor epoch and local task state at T0
    synch();
    reset_application_state();
    g_system_started = true;

    //Release all tasks
    xTaskNotifyGive(g_taskA_handle);
    xTaskNotifyGive(g_taskB_handle);
    xTaskNotifyGive(g_taskAGG_handle);
    xTaskNotifyGive(g_taskC_handle);
    xTaskNotifyGive(g_taskD_handle);
    xTaskNotifyGive(g_taskS_handle);
    xTaskNotifyGive(g_taskMON_handle);

    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}