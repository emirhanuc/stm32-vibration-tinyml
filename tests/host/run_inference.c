/*
 * Host-side test harness for Services/Inference/pm_inference.c.
 * Reads a 1024-row CSV (header "x,y,z"), runs feature extraction and both
 * classifiers, and prints the results so they can be compared with the
 * Python reference (tests/host/reference_features.py).
 *
 * Build: see tests/host/Makefile
 */
#include <stdio.h>
#include <stdlib.h>
#include "pm_inference.h"

int main(int argc, char **argv)
{
    static int16_t xyz[PM_WINDOW * 3];
    float features[PM_FEATURE_COUNT];
    float prob[PM_CLASS_COUNT];
    float scores[PM_CLASS_COUNT];
    char line[128];

    if (argc != 2)
    {
        fprintf(stderr, "usage: %s window.csv\n", argv[0]);
        return 2;
    }

    FILE *f = fopen(argv[1], "r");
    if (f == NULL)
    {
        perror("fopen");
        return 2;
    }

    if (fgets(line, sizeof line, f) == NULL) /* header */
    {
        fclose(f);
        return 2;
    }

    for (int i = 0; i < PM_WINDOW; i++)
    {
        int x, y, z;
        if ((fgets(line, sizeof line, f) == NULL) ||
            (sscanf(line, "%d,%d,%d", &x, &y, &z) != 3))
        {
            fprintf(stderr, "bad row %d\n", i);
            fclose(f);
            return 2;
        }
        xyz[3 * i + 0] = (int16_t)x;
        xyz[3 * i + 1] = (int16_t)y;
        xyz[3 * i + 2] = (int16_t)z;
    }
    fclose(f);

    if (pm_extract_features(xyz, features) != 0)
    {
        return 1;
    }

    int rf = pm_predict_rf(features, prob);
    int lr = pm_predict_lr(features, scores);

    for (int j = 0; j < PM_FEATURE_COUNT; j++)
    {
        printf("feature[%d]=%.9g\n", j, features[j]);
    }
    for (int c = 0; c < PM_CLASS_COUNT; c++)
    {
        printf("rf_prob[%d]=%.6f\n", c, prob[c]);
    }
    printf("rf_class=1.%d\n", rf + 1);
    printf("lr_class=1.%d\n", lr + 1);
    return 0;
}
