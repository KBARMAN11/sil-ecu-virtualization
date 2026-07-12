#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#define configUSE_PREEMPTION                    1
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configUSE_TRACE_FACILITY                0
#define configUSE_16_BIT_TICKS                  0
#define configUSE_MUTEXES                       1
#define configUSE_TIMERS                        0
#define configUSE_COUNTING_SEMAPHORES           0
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_MALLOC_FAILED_HOOK            0
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0
#define configUSE_QUEUE_SETS                    0
#define configCHECK_FOR_STACK_OVERFLOW          0
#define configGENERATE_RUN_TIME_STATS           0
#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1

#define configCPU_CLOCK_HZ                      100000000
#define configTICK_RATE_HZ                      1000
#define configMAX_PRIORITIES                    7
#define configMINIMAL_STACK_SIZE                512
#define configTOTAL_HEAP_SIZE                   ( 1024 * 1024 )
#define configMAX_TASK_NAME_LEN                 16
#define configIDLE_SHOULD_YIELD                 1
#define configQUEUE_REGISTRY_SIZE               8
#define configTIMER_TASK_PRIORITY               2
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            256

#define INCLUDE_vTaskDelay                      1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_xTaskGetSchedulerState          1

#endif
#define INCLUDE_xQueueSend                      1
#define INCLUDE_xQueueReceive                   1
