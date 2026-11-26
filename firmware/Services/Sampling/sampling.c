#include "sampling.h"
#include <stddef.h>

static SamplingWindow_t buffer_a;
static SamplingWindow_t buffer_b;

static SamplingWindow_t *active_buffer = &buffer_a;
static SamplingWindow_t *standby_buffer = &buffer_b;

static volatile uint8_t sample_due = 0U;
static volatile uint8_t window_ready = 0U;
static SamplingWindow_t *ready_buffer = NULL;
static uint32_t write_index = 0U;

static volatile uint32_t total_samples = 0U;
static volatile uint32_t missed_samples = 0U;
static volatile uint32_t completed_windows = 0U;
static volatile uint32_t dropped_windows = 0U;

void Sampling_Init(void)
{
    active_buffer = &buffer_a;
    standby_buffer = &buffer_b;
    ready_buffer = NULL;
    write_index = 0U;
    sample_due = 0U;
    window_ready = 0U;
    total_samples = 0U;
    missed_samples = 0U;
    completed_windows = 0U;
    dropped_windows = 0U;
}

void Sampling_TimerTickISR(void)
{
    /* Count timer events that could not be consumed before the next tick. */
    if (sample_due != 0U)
    {
        missed_samples++;
        return;
    }

    sample_due = 1U;
}

void Sampling_Process(void)
{
    LIS3DSH_RawData_t sample;
    SamplingWindow_t *temp_buffer;

    /* Atomically test and clear the pending sample flag.
     * Restore the previous interrupt mask rather than enabling
     * interrupts unconditionally. The SPI read runs with IRQs enabled.
     */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (sample_due == 0U)
    {
        __set_PRIMASK(primask);
        return;
    }

    sample_due = 0U;
    __set_PRIMASK(primask);

    if (LIS3DSH_ReadRaw(&sample) != LIS3DSH_STATUS_OK)
    {
        missed_samples++;
        return;
    }

    active_buffer->x[write_index] = sample.x;
    active_buffer->y[write_index] = sample.y;
    active_buffer->z[write_index] = sample.z;

    write_index++;
    total_samples++;

    if (write_index >= SAMPLING_WINDOW_SIZE)
    {
        completed_windows++;

        if (window_ready != 0U)
        {
            dropped_windows++;
            write_index = 0U;
            return;
        }

        ready_buffer = active_buffer;
        window_ready = 1U;

        temp_buffer = active_buffer;
        active_buffer = standby_buffer;
        standby_buffer = temp_buffer;
        write_index = 0U;
    }
}

const SamplingWindow_t *Sampling_GetReadyWindow(void)
{
    if (window_ready == 0U)
    {
        return NULL;
    }

    return ready_buffer;
}

void Sampling_ReleaseWindow(void)
{
    ready_buffer = NULL;
    window_ready = 0U;
}

void Sampling_GetStats(SamplingStats_t *stats)
{
    if (stats == NULL)
    {
        return;
    }

    stats->total_samples = total_samples;
    stats->missed_samples = missed_samples;
    stats->completed_windows = completed_windows;
    stats->dropped_windows = dropped_windows;
}
