# Host (PC) build of the inference code + numerical check against SciPy.
#   make test     -> builds run_inference, runs it on test_window.csv, compares with SciPy
INF := ../../firmware/Services/Inference
CC  ?= gcc

run_inference: run_inference.c $(INF)/pm_inference.c $(INF)/pm_inference.h
	$(CC) -O2 -Wall -Wextra -std=c11 -I$(INF) -o $@ run_inference.c $(INF)/pm_inference.c -lm

features_c.txt: run_inference test_window.csv
	./run_inference test_window.csv > $@

test: features_c.txt
	python3 reference_features.py test_window.csv features_c.txt

clean:
	rm -f run_inference features_c.txt

.PHONY: test clean
