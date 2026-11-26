#ifndef PM_INFERENCE_H
#define PM_INFERENCE_H

#include <stdint.h>

#define PM_WINDOW        1024   /* samples per window (1.28 s @ 800 Hz) */
#define PM_FEATURE_COUNT 51     /* 17 features x 3 axes */
#define PM_CLASS_COUNT   6

/* xyz: interleaved signed raw counts [x0,y0,z0,x1,y1,z1,...]. Returns 0 on success. */
int pm_extract_features(const int16_t xyz[PM_WINDOW * 3], float features[PM_FEATURE_COUNT]);

/* Logistic-regression baseline. scores may be NULL. Returns class index. */
int pm_predict_lr(const float features[PM_FEATURE_COUNT], float scores[PM_CLASS_COUNT]);

/* Random Forest (150 trees). probabilities may be NULL. Returns class index. */
int pm_predict_rf(const float features[PM_FEATURE_COUNT], float probabilities[PM_CLASS_COUNT]);

#endif /* PM_INFERENCE_H */
