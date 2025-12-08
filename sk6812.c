// Peripheral usage
#include "stm32l4xx_hal.h"
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_tim2_ch1;
#include "sk6812.h"
#define PWM_HI (38)
#define PWM_LO (19)
#define MAX_STEP 2
typedef struct {
   uint8_t r;
   uint8_t g;
   uint8_t b;
} Color_t;
typedef struct {
   uint8_t current;   // current LED height (0–15)
   uint8_t target;    // target from fft_mag
} BarState;
BarState bars[8] = {0};
Color_t column_color[16] = {
   {  20,   0,  40},  // col 0
   {  55,   0, 120},  // col 1
   { 110,   0, 180},  // col 2
   { 165,   0, 230},  // col 3
   { 255,   0, 150},  // col 4
   { 255,   0,  80},  // col 5
   { 255,   0,   0},  // col 6
   { 255, 122,   0},  // col 7
   { 255, 213,   0},  // col 8
   { 176, 255,   0},  // col 9
   {   0, 255,  42},  // col 10
   {   0, 255, 213},  // col 11
   {   0, 197, 255},  // col 12
   {   0,  80, 255},  // col 13
   {  26,   0, 204},  // col 14
   {   9,   0,  58},  // col 15
};
// LED parameters
#define NUM_BPP (3) // WS2812B
//#define NUM_BPP (4) // SK6812
#define NUM_PIXELS (256)
#define NUM_BYTES (NUM_BPP * NUM_PIXELS)
// LED color buffer
uint8_t rgb_arr[NUM_BYTES] = {0};
// LED write buffer
#define WR_BUF_LEN (NUM_BPP * 8 * 2)
uint8_t wr_buf[WR_BUF_LEN] = {0};
uint_fast8_t wr_buf_p = 0;
static inline uint8_t scale8(uint8_t x, uint8_t scale) {
 return ((uint16_t)x * scale) >> 8;
}
// Set a single color (RGB) to index
void led_set_RGB(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
#if (NUM_BPP == 4) // SK6812
 rgb_arr[4 * index] = scale8(g, 0xB0); // g;
 rgb_arr[4 * index + 1] = r;
 rgb_arr[4 * index + 2] = scale8(b, 0xF0); // b;
 rgb_arr[4 * index + 3] = 0;
#else // WS2812B
 rgb_arr[3 * index] = scale8(g, 0xB0); // g;
 rgb_arr[3 * index + 1] = r;
 rgb_arr[3 * index + 2] = scale8(b, 0xF0); // b;
#endif // End SK6812 WS2812B case differentiation
}
// Set a single color (RGBW) to index
void led_set_RGBW(uint8_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
 led_set_RGB(index, r, g, b);
#if (NUM_BPP == 4) // SK6812
 rgb_arr[4 * index + 3] = w;
#endif // End SK6812 WS2812B case differentiation
}
// Set all colors to RGB
void led_set_all_RGB(uint8_t r, uint8_t g, uint8_t b) {
 for(uint_fast8_t i = 0; i < NUM_PIXELS; ++i) led_set_RGB(i, r, g, b);
}
// Set all colors to RGBW
void led_set_all_RGBW(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
 for(uint_fast8_t i = 0; i < NUM_PIXELS; ++i) led_set_RGBW(i, r, g, b, w);
}
void led_set_column_RGB(uint8_t col, uint8_t r, uint8_t g, uint8_t b) {
   if (col >= 16) return;   // 16 physical columns total
   for (uint8_t row = 0; row < 16; row++) {
       uint16_t idx;
       if (row % 2 == 0) {
           // Even row → left-to-right
           idx = row * 16 + col;
       } else {
           // Odd row → right-to-left
           idx = row * 16 + (15 - col);
       }
       led_set_RGB(idx, r, g, b);
   }
}
// Clear entire matrix
void matrix_clear() {
   for (uint16_t i = 0; i < NUM_PIXELS; ++i) {
       led_set_RGB(i, 0, 0, 0);
   }
}
void init_matrix_test() {
   for (uint8_t col = 0; col < 16; col++) {
       Color_t c = column_color[col];
       led_set_column_RGB(col, c.r, c.g, c.b);
   }
}
// pulses ONE column
void pulse_column_level(uint8_t col, uint8_t level)
{
   if (level > 15) level = 15;
   Color_t c = column_color[col];
   // light from bottom to the level
   for (uint8_t row = 0; row <= level; row++) {
       uint16_t idx;
       if (row % 2 == 0) {
           idx = row * 16 + col;          // even row L→R
       } else {
           idx = row * 16 + (15 - col);   // odd row R→L
       }
       led_set_RGB(idx, c.r, c.g, c.b);
       led_render();
       HAL_Delay(20);
   }
   // turn off from [level] to bottom
   for (int8_t row = level; row >= 0; row--) {
       uint16_t idx;
       if (row % 2 == 0) {
           idx = row * 16 + col;
       } else {
           idx = row * 16 + (15 - col);
       }
       led_set_RGB(idx, 0, 0, 0);
       led_render();
       HAL_Delay(20);
   }
}
// convert fft_mag into a value for the bar
void update_fft_bars() {
	for (int b = 0; b < 8; b++) {
       float max_val = 0;
       int start;
       int size;
       // band 0: bins[16,32]
       // band 1: bins[32,47]
       // band 2: bins[48,63]
       // ...
       // band 7: bins[128,144]
       if (b == 0) {
           start = 16;
           size  = 16;
       } else {
           start = 16 * b + 16;
           size  = 16;
       }
       // Use the correct number of bins per band
       for (int i = 0; i < size; i++) {
           if (fft_mag[start + i] > max_val)
               max_val = fft_mag[start + i];
       }
       // Nonlinear scaling to even out bands
       float scaled = sqrtf(max_val) * 6.0f;
       if (scaled > 15) scaled = 15;
       bars[b].target = (uint8_t)scaled;
   }
}
/* desired behavior:
 Pulse from bottom to top;  target level computed from FFT
 Once it reaches that target level, the bar immediately begins falling back down to 0
 If next FFT magnitude is higher, the animation continues upward from wherever it currently is
 If the next FFT magnitude is lower, the animation still finishes falling from the higher point
*/
void animate_bars_pulse()
{
   for (int b = 0; b < 8; b++)
   {
       BarState *bar = &bars[b];
       // Pulse logic
       if (bar->current < bar->target) bar->current++;
       else if (bar->current > 0)      bar->current--;
       // MAP band to two columns
       int colA = b * 2;
       int colB = b * 2 + 1;
       Color_t cA = column_color[colA];
       Color_t cB = column_color[colB];
       for (int row = 0; row < 16; row++)
       {
           // ========== column A ==========
           uint16_t idxA;
           if (row % 2 == 0)
               idxA = row * 16 + colA;
           else
               idxA = row * 16 + (15 - colA);
           if (row <= bar->current)
               led_set_RGB(idxA, cA.r, cA.g, cA.b);
           else
               led_set_RGB(idxA, 0, 0, 0);
           // ========== column B ==========
           uint16_t idxB;
           if (row % 2 == 0)
               idxB = row * 16 + colB;
           else
               idxB = row * 16 + (15 - colB);
           if (row <= bar->current)
               led_set_RGB(idxB, cB.r, cB.g, cB.b);
           else
               led_set_RGB(idxB, 0, 0, 0);
       }
   }
}
// Shuttle the data to the LEDs!
void led_render() {
//  if(wr_buf_p != 0 || hdma_tim2_ch1.State != HAL_DMA_STATE_READY) {
//    // Ongoing transfer, cancel!
//    for(uint8_t i = 0; i < WR_BUF_LEN; ++i) wr_buf[i] = 0;
//    wr_buf_p = 0;
//    HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
//    return;
//	while (wr_buf_p != 0 || hdma_tim2_ch1.State != HAL_DMA_STATE_READY) {
//	    // do nothing, wait for transfer to finish
//	}
	__disable_irq();
 // the first data buffer half (and the second!)
#if (NUM_BPP == 4) // SK6812
 for(uint_fast8_t i = 0; i < 8; ++i) {
   wr_buf[i     ] = PWM_LO << (((rgb_arr[0] << i) & 0x80) > 0);
   wr_buf[i +  8] = PWM_LO << (((rgb_arr[1] << i) & 0x80) > 0);
   wr_buf[i + 16] = PWM_LO << (((rgb_arr[2] << i) & 0x80) > 0);
   wr_buf[i + 24] = PWM_LO << (((rgb_arr[3] << i) & 0x80) > 0);
   wr_buf[i + 32] = PWM_LO << (((rgb_arr[4] << i) & 0x80) > 0);
   wr_buf[i + 40] = PWM_LO << (((rgb_arr[5] << i) & 0x80) > 0);
   wr_buf[i + 48] = PWM_LO << (((rgb_arr[6] << i) & 0x80) > 0);
   wr_buf[i + 56] = PWM_LO << (((rgb_arr[7] << i) & 0x80) > 0);
 }
#else // WS2812B
 for(uint_fast8_t i = 0; i < 8; ++i) {
	wr_buf[i     ] = PWM_LO << (((rgb_arr[0] << i) & 0x80) > 0);
   wr_buf[i +  8] = PWM_LO << (((rgb_arr[1] << i) & 0x80) > 0);
   wr_buf[i + 16] = PWM_LO << (((rgb_arr[2] << i) & 0x80) > 0);
   wr_buf[i + 24] = PWM_LO << (((rgb_arr[3] << i) & 0x80) > 0);
   wr_buf[i + 32] = PWM_LO << (((rgb_arr[4] << i) & 0x80) > 0);
   wr_buf[i + 40] = PWM_LO << (((rgb_arr[5] << i) & 0x80) > 0);
 }
#endif // End SK6812 WS2812B case differentiation
 HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_1, (uint32_t *)wr_buf, WR_BUF_LEN);
 wr_buf_p = 2; // Since we're ready for the next buffer
 __enable_irq();
}
void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim) {
 // DMA buffer set from LED(wr_buf_p) to LED(wr_buf_p + 1)
 if(wr_buf_p < NUM_PIXELS) {
   // We're in. Fill the even buffer
#if (NUM_BPP == 4) // SK6812
   for(uint_fast8_t i = 0; i < 8; ++i) {
     wr_buf[i     ] = PWM_LO << (((rgb_arr[4 * wr_buf_p    ] << i) & 0x80) > 0);
     wr_buf[i +  8] = PWM_LO << (((rgb_arr[4 * wr_buf_p + 1] << i) & 0x80) > 0);
     wr_buf[i + 16] = PWM_LO << (((rgb_arr[4 * wr_buf_p + 2] << i) & 0x80) > 0);
     wr_buf[i + 24] = PWM_LO << (((rgb_arr[4 * wr_buf_p + 3] << i) & 0x80) > 0);
   }
#else // WS2812B
   for(uint_fast8_t i = 0; i < 8; ++i) {
     wr_buf[i     ] = PWM_LO << (((rgb_arr[3 * wr_buf_p    ] << i) & 0x80) > 0);
     wr_buf[i +  8] = PWM_LO << (((rgb_arr[3 * wr_buf_p + 1] << i) & 0x80) > 0);
     wr_buf[i + 16] = PWM_LO << (((rgb_arr[3 * wr_buf_p + 2] << i) & 0x80) > 0);
   }
#endif // End SK6812 WS2812B case differentiation
   wr_buf_p++;
 } else if (wr_buf_p < NUM_PIXELS + 2) {
   // Last two transfers are resets. SK6812: 64 * 1.25 us = 80 us == good enough reset
 	//                               WS2812B: 48 * 1.25 us = 60 us == good enough reset
   // First half reset zero fill
   for(uint8_t i = 0; i < WR_BUF_LEN / 2; ++i) wr_buf[i] = 0;
   wr_buf_p++;
 }
}
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
 // DMA buffer set from LED(wr_buf_p) to LED(wr_buf_p + 1)
 if(wr_buf_p < NUM_PIXELS) {
   // We're in. Fill the odd buffer
#if (NUM_BPP == 4) // SK6812
   for(uint_fast8_t i = 0; i < 8; ++i) {
     wr_buf[i + 32] = PWM_LO << (((rgb_arr[4 * wr_buf_p    ] << i) & 0x80) > 0);
     wr_buf[i + 40] = PWM_LO << (((rgb_arr[4 * wr_buf_p + 1] << i) & 0x80) > 0);
     wr_buf[i + 48] = PWM_LO << (((rgb_arr[4 * wr_buf_p + 2] << i) & 0x80) > 0);
     wr_buf[i + 56] = PWM_LO << (((rgb_arr[4 * wr_buf_p + 3] << i) & 0x80) > 0);
   }
#else // WS2812B
   for(uint_fast8_t i = 0; i < 8; ++i) {
     wr_buf[i + 24] = PWM_LO << (((rgb_arr[3 * wr_buf_p    ] << i) & 0x80) > 0);
     wr_buf[i + 32] = PWM_LO << (((rgb_arr[3 * wr_buf_p + 1] << i) & 0x80) > 0);
     wr_buf[i + 40] = PWM_LO << (((rgb_arr[3 * wr_buf_p + 2] << i) & 0x80) > 0);
   }
#endif // End SK6812 WS2812B case differentiation
   wr_buf_p++;
 } else if (wr_buf_p < NUM_PIXELS + 2) {
   // Second half reset zero fill
   for(uint8_t i = WR_BUF_LEN / 2; i < WR_BUF_LEN; ++i) wr_buf[i] = 0;
   ++wr_buf_p;
 } else {
   // We're done. Lean back and until next time!
   wr_buf_p = 0;
   HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_1);
 }
}

