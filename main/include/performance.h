#ifndef PERFORMANCE_H
#define PERFORMANCE_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Performance statistics for a task
 */
typedef struct {
    const char *task_name;
    uint32_t target_period_us;      // Target period in microseconds
    uint32_t execution_count;       // Number of executions
    uint64_t total_exec_time_us;    // Total execution time
    uint32_t min_exec_time_us;      // Minimum execution time
    uint32_t max_exec_time_us;      // Maximum execution time
    uint32_t min_period_us;         // Minimum period (jitter measurement)
    uint32_t max_period_us;         // Maximum period (jitter measurement)
    uint32_t deadline_misses;       // Number of deadline misses
    uint64_t last_wake_time_us;     // Last wake time for period calculation
} TaskPerformance;

/**
 * Initialize performance monitoring for a task
 * @param perf Pointer to TaskPerformance structure
 * @param task_name Name of the task
 * @param target_period_us Target period in microseconds
 */
void perf_init(TaskPerformance *perf, const char *task_name, uint32_t target_period_us);

/**
 * Mark the start of a task execution
 * @param perf Pointer to TaskPerformance structure
 * @return Start timestamp (for passing to perf_end)
 */
uint64_t perf_start(TaskPerformance *perf);

/**
 * Mark the end of a task execution
 * @param perf Pointer to TaskPerformance structure
 * @param start_time Start timestamp from perf_start
 */
void perf_end(TaskPerformance *perf, uint64_t start_time);

/**
 * Get average execution time in microseconds
 * @param perf Pointer to TaskPerformance structure
 * @return Average execution time in microseconds
 */
uint32_t perf_get_avg_exec_time_us(const TaskPerformance *perf);

/**
 * Get average period in microseconds
 * @param perf Pointer to TaskPerformance structure
 * @return Average period in microseconds
 */
uint32_t perf_get_avg_period_us(const TaskPerformance *perf);

/**
 * Get jitter (max period - min period) in microseconds
 * @param perf Pointer to TaskPerformance structure
 * @return Jitter in microseconds
 */
uint32_t perf_get_jitter_us(const TaskPerformance *perf);

/**
 * Print performance statistics to log
 * @param perf Pointer to TaskPerformance structure
 */
void perf_print(const TaskPerformance *perf);

/**
 * Reset performance statistics
 * @param perf Pointer to TaskPerformance structure
 */
void perf_reset(TaskPerformance *perf);

#endif // PERFORMANCE_H

