/**
  ******************************************************************************
  * @file    pm_inference.c
  * @brief   On-device feature extraction and classifiers for vibration
  *          fault classification.
  *
  * Features (17 per axis x 3 axes = 51), computed on a mean-removed window:
  *   [0]  RMS
  *   [1]  peak-to-peak
  *   [2]  crest factor (|peak| / RMS)
  *   [3]  excess kurtosis  (== scipy.stats.kurtosis(fisher=True, bias=False))
  *   [4]  dominant frequency [Hz]  (PSD argmax, DC excluded)
  *   [5]  total power (trapezoidal integral of PSD)
  *   [6..16] band power in 11 bands (Hz): 0-25, 25-50, 50-60, 60-75, 75-95,
  *        95-110, 110-125, 125-160, 160-220, 220-250, 250-350
  *
  * PSD == scipy.signal.welch(fs=800, nperseg=1024, window='hann',
  *        scaling='density', detrend=False) on a single 1024-sample segment.
  *
  * NOTE: The PSD uses a direct reference DFT (3 x 513 x 1024 MACs in double
  * precision). It is numerically faithful to the Python training pipeline
  * but NOT optimized; replacing it with CMSIS-DSP arm_rfft_fast_f32 is the
  * planned speed-up.
  ******************************************************************************
  */
#include "pm_inference.h"
#include "generated/pm_models_data.h"

#include <math.h>
#include <stddef.h>

#define PM_PI            3.14159265358979323846
#define PM_FS_HZ         800.0
#define PM_N             PM_WINDOW
#define PM_BINS          ((PM_N / 2) + 1)          /* 513 one-sided bins */
#define PM_AXES          3
#define PM_FEAT_PER_AXIS 17
#define PM_BAND_COUNT    11

#if (PM_FEATURE_COUNT != PM_FEATURES) || (PM_CLASS_COUNT != PM_CLASSES)
#error "pm_inference.h and generated/pm_models_data.h disagree on sizes"
#endif

static const int band_lo_hz[PM_BAND_COUNT] = {0, 25, 50, 60, 75, 95, 110, 125, 160, 220, 250};
static const int band_hi_hz[PM_BAND_COUNT] = {25, 50, 60, 75, 95, 110, 125, 160, 220, 250, 350};

/* Static scratch avoids large stack allocations. Not reentrant. */
static float sig[PM_N];
static float psd[PM_BINS];

/* Welch density scaling for a periodic Hann window: fs * sum(w^2) = fs * 3N/8 */
static double welch_scale(void)
{
    return PM_FS_HZ * ((double)PM_N * 3.0 / 8.0);
}

/* One-sided PSD of sig[] into psd[]; returns argmax bin (DC excluded). */
static int compute_psd(void)
{
    float peak_power = -1.0f;
    int peak_bin = 1;
    const double scale = welch_scale();

    for (int k = 0; k < PM_BINS; k++)
    {
        double re = 0.0;
        double im = 0.0;

        for (int i = 0; i < PM_N; i++)
        {
            double w = 0.5 - (0.5 * cos(2.0 * PM_PI * i / PM_N));
            double theta = 2.0 * PM_PI * k * i / PM_N;
            double v = sig[i] * w;
            re += v * cos(theta);
            im -= v * sin(theta);
        }

        double p = ((re * re) + (im * im)) / scale;
        if ((k != 0) && (k != (PM_N / 2)))
        {
            p *= 2.0;   /* one-sided: double everything except DC and Nyquist */
        }
        psd[k] = (float)p;

        if ((k > 0) && (psd[k] > peak_power))
        {
            peak_power = psd[k];
            peak_bin = k;
        }
    }

    return peak_bin;
}

int pm_extract_features(const int16_t xyz[PM_WINDOW * 3], float features[PM_FEATURE_COUNT])
{
    if ((xyz == NULL) || (features == NULL))
    {
        return -1;
    }

    for (int axis = 0; axis < PM_AXES; axis++)
    {
        float *out = &features[axis * PM_FEAT_PER_AXIS];

        /* --- Time domain ------------------------------------------------ */
        double mean = 0.0;
        for (int i = 0; i < PM_N; i++)
        {
            mean += (double)xyz[(i * 3) + axis];
        }
        mean /= PM_N;

        double sum2 = 0.0;
        double sum4 = 0.0;
        float lo = INFINITY;
        float hi = -INFINITY;
        float abspeak = 0.0f;

        for (int i = 0; i < PM_N; i++)
        {
            float x = (float)((double)xyz[(i * 3) + axis] - mean);
            sig[i] = x;

            double q = (double)x * x;
            sum2 += q;
            sum4 += q * q;

            if (x < lo)
            {
                lo = x;
            }
            if (x > hi)
            {
                hi = x;
            }
            if (fabsf(x) > abspeak)
            {
                abspeak = fabsf(x);
            }
        }

        float rms = (float)sqrt(sum2 / PM_N);

        /* Unbiased excess kurtosis (scipy fisher=True, bias=False). */
        double m2 = sum2 / PM_N;
        double m4 = sum4 / PM_N;
        double kurt = 0.0;
        if (m2 > 0.0)
        {
            kurt = ((PM_N - 1.0) / ((PM_N - 2.0) * (PM_N - 3.0))) *
                   (((PM_N + 1.0) * ((m4 / (m2 * m2)) - 3.0)) + 6.0);
        }

        out[0] = rms;
        out[1] = hi - lo;
        out[2] = abspeak / fmaxf(rms, 1e-9f);
        out[3] = (float)kurt;

        /* --- Frequency domain ------------------------------------------- */
        int peak_bin = compute_psd();
        out[4] = (float)peak_bin * (float)(PM_FS_HZ / PM_N);

        double total = (psd[0] * 0.5) + (psd[PM_BINS - 1] * 0.5);
        for (int k = 1; k < (PM_BINS - 1); k++)
        {
            total += psd[k];
        }
        out[5] = (float)(total * PM_FS_HZ / PM_N);

        for (int b = 0; b < PM_BAND_COUNT; b++)
        {
            double power = 0.0;
            for (int k = 0; k < PM_BINS; k++)
            {
                double hz = k * (PM_FS_HZ / PM_N);
                if ((hz >= band_lo_hz[b]) && (hz < band_hi_hz[b]))
                {
                    power += psd[k];
                }
            }
            out[6 + b] = (float)(power * PM_FS_HZ / PM_N);
        }
    }

    return 0;
}

int pm_predict_lr(const float features[PM_FEATURE_COUNT], float scores[PM_CLASS_COUNT])
{
    int best = 0;
    float best_score = -INFINITY;

    for (int c = 0; c < PM_CLASSES; c++)
    {
        float s = pm_lr_b[c];
        for (int j = 0; j < PM_FEATURES; j++)
        {
            s += pm_lr_w[(c * PM_FEATURES) + j] * features[j];
        }

        if (scores != NULL)
        {
            scores[c] = s;
        }
        if (s > best_score)
        {
            best_score = s;
            best = c;
        }
    }

    return best;
}

int pm_predict_rf(const float features[PM_FEATURE_COUNT], float probabilities[PM_CLASS_COUNT])
{
    float p[PM_CLASSES] = {0};

    for (int t = 0; t < PM_RF_TREES; t++)
    {
        int node = pm_rf_offsets[t];

        /* Internal nodes have feature >= 0; leaves are marked with -1/-2. */
        while (pm_rf_feature[node] >= 0)
        {
            int j = pm_rf_feature[node];
            node = (features[j] <= pm_rf_threshold[node]) ? pm_rf_left[node]
                                                          : pm_rf_right[node];
        }

        for (int c = 0; c < PM_CLASSES; c++)
        {
            p[c] += pm_rf_values[(node * PM_CLASSES) + c] / PM_RF_TREES;
        }
    }

    int best = 0;
    for (int c = 0; c < PM_CLASSES; c++)
    {
        if (probabilities != NULL)
        {
            probabilities[c] = p[c];
        }
        if (p[c] > p[best])
        {
            best = c;
        }
    }

    return best;
}
