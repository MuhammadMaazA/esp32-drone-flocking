#include "performance.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "performance";

void perf_init(TaskPerformance *perf, const char *task_name, uint32_t target_period_us)
{
    if (perf == NULL) {
        return;
    }

    memset(perf, 0, sizeof(TaskPerformance));
    perf->task_name = task_name;
    perf->target_period_us = target_period_us;
    perf->min_exec_time_us = UINT32_MAX;
    perf->max_exec_time_us = 0;
    perf->min_period_us = UINT32_MAX;
    perf->max_period_us = 0;
    perf->last_wake_time_us = 0;
    perf->execution_count = 0;
    perf->deadline_misses = 0;
    perf->total_exec_time_us = 0;
}

uint64_t perf_start(TaskPerformance *perf)
{
    if (perf == NULL) {
        return 0;
    }

    uint64_t now_us = esp_timer_get_time();

    // Calculate period (time since last wake)
    if (perf->last_wake_time_us > 0) {
        uint32_t period_us = (uint32_t)(now_us - perf->last_wake_time_us);
        
        // Update min/max period
        if (period_us < perf->min_period_us) {
            perf->min_period_us = period_us;
        }
        if (period_us > perf->max_period_us) {
            perf->max_period_us = period_us;
        }

        // Check for deadline miss (period exceeded target by more than 10%)
        if (period_us > perf->target_period_us * 1.1) {
            perf->deadline_misses++;
        }
    }

    perf->last_wake_time_us = now_us;
    return now_us;
}

void perf_end(TaskPerformance *perf, uint64_t start_time)
{
    if (perf == NULL || start_time == 0) {
        return;
    }

    uint64_t now_us = esp_timer_get_time();
    uint32_t exec_time_us = (uint32_t)(now_us - start_time);

    // Update execution time statistics
    perf->execution_count++;
    perf->total_exec_time_us += exec_time_us;

    if (exec_time_us < perf->min_exec_time_us) {
        perf->min_exec_time_us = exec_time_us;
    }
    if (exec_time_us > perf->max_exec_time_us) {
        perf->max_exec_time_us = exec_time_us;
    }
}

uint32_t perf_get_avg_exec_time_us(const TaskPerformance *perf)
{
    if (perf == NULL || perf->execution_count == 0) {
        return 0;
    }
    return (uint32_t)(perf->total_exec_time_us / perf->execution_count);
}

uint32_t perf_get_avg_period_us(const TaskPerformance *perf)
{
    if (perf == NULL || perf->execution_count <= 1) {
        return 0;
    }
    // Average period is approximately total time / (executions - 1)
    return (perf->min_period_us + perf->max_period_us) / 2;
}

uint32_t perf_get_jitter_us(const TaskPerformance *perf)
{
    if (perf == NULL) {
        return 0;
    }
    return perf->max_period_us - perf->min_period_us;
}

void perf_print(const TaskPerformance *perf)
{
    if (perf == NULL || perf->execution_count == 0) {
        return;
    }

    uint32_t avg_exec_us = perf_get_avg_exec_time_us(perf);
    uint32_t avg_period_us = perf_get_avg_period_us(perf);
    uint32_t jitter_us = perf_get_jitter_us(perf);

    ESP_LOGI(TAG, "=== Performance Stats: %s ===", perf->task_name);
    ESP_LOGI(TAG, "  Executions: %u", perf->execution_count);
    ESP_LOGI(TAG, "  Target period: %u us (%.2f Hz)", 
             perf->target_period_us, 1000000.0f / perf->target_period_us);
    ESP_LOGI(TAG, "  Execution time: avg=%u us, min=%u us, max=%u us",
             avg_exec_us, perf->min_exec_time_us, perf->max_exec_time_us);
    ESP_LOGI(TAG, "  Period: avg=%u us, min=%u us, max=%u us",
             avg_period_us, perf->min_period_us, perf->max_period_us);
    ESP_LOGI(TAG, "  Jitter: %u us (%.2f%%)",
             jitter_us, (100.0f * jitter_us) / perf->target_period_us);
    ESP_LOGI(TAG, "  Deadline misses: %u (%.2f%%)",
             perf->deadline_misses, (100.0f * perf->deadline_misses) / perf->execution_count);
    ESP_LOGI(TAG, "  CPU utilization: %.2f%%",
             (100.0f * avg_exec_us) / perf->target_period_us);
}

void perf_reset(TaskPerformance *perf)
{
    if (perf == NULL) {
        return;
    }

    const char *name = perf->task_name;
    uint32_t target = perf->target_period_us;
    perf_init(perf, name, target);
}

