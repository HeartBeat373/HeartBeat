#ifndef _LED_DRIVER_SK6812
#define _LED_DRIVER_SK6812
#include <stdint.h>
#include "arm_math.h"

#define FFT_SIZE 512
extern float32_t fft_mag[FFT_SIZE/2];     // magnitude of first N/2 bins
extern uint_fast8_t wr_buf_p;
void led_set_RGB(uint8_t index, uint8_t r, uint8_t g, uint8_t b);
void led_set_RGBW(uint8_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w);
void led_set_all_RGB(uint8_t r, uint8_t g, uint8_t b);
void led_set_all_RGBW(uint8_t r, uint8_t g, uint8_t b, uint8_t w);
void led_set_column_RGB(uint8_t col, uint8_t r, uint8_t g, uint8_t b);
void matrix_clear();
void init_matrix_test();
void pulse_column_level(uint8_t col, uint8_t level);
void led_render();
#endif

