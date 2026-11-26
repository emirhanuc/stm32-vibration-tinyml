#ifndef SAMPLING_H
#define SAMPLING_H

#include <stdint.h>
#include "lis3dsh_driver.h"

#define SAMPLING_WINDOW_SIZE 1024U

typedef struct
{
    int16_t x[SAMPLING_WINDOW_SIZE];
    int16_t y[SAMPLING_WINDOW_SIZE];
    int16_t z[SAMPLING_WINDOW_SIZE];
} SamplingWindow_t;

typedef struct
{
    uint32_t total_samples;
    uint32_t missed_samples;
    uint32_t completed_windows;
    uint32_t dropped_windows;
} SamplingStats_t;

void Sampling_Init(void);
void Sampling_TimerTickISR(void);
void Sampling_Process(void);

const SamplingWindow_t *Sampling_GetReadyWindow(void);
void Sampling_ReleaseWindow(void);
void Sampling_GetStats(SamplingStats_t *stats);

#endif
