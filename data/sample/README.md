# Sample data

`swv_itm_capture_raw.txt` is an early SWV/ITM capture from the board lying still (CSV: window,index,x,y,z; raw LIS3DSH counts, 16384 = 1 g). SWV drops some lines, so windows are incomplete; `tests/host/test_window.csv` is window 17 with the 5 missing samples linearly interpolated.
