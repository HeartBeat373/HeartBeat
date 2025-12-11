/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lvgl.h"
#include "ili9488.h"
#include "lcd_io.h"
#include "screens.h"
#include "arm_math.h"
#include "sk6812.h"
#include "stdio.h"
#include "apds9960.h"


#include "ui.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RESOLUTION_HORIZONTAL 480
#define RESOLUTION_VERTICAL 320

#define BYTES_PER_PIXEL (LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_RGB565))

#define AUDIO_SAMPLE_RATE_HZ 16000u   // or whatever your WAV sample rate is

volatile uint32_t g_samples_played = 0;  // samples in *current song*
volatile uint32_t g_last_samples_played = 0;  // samples in *current song*

volatile uint32_t g_ms_since_song_start = 0; // optional, derived in main

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;
DMA_HandleTypeDef hdma_tim2_ch1;

/* USER CODE BEGIN PV */

APDS9960_t PAUSE_SKIP_Sensor;
APDS9960_t VOLUME_SPEED_Sensor;

static uint8_t buf1[(RESOLUTION_HORIZONTAL * RESOLUTION_VERTICAL / 10 * BYTES_PER_PIXEL)];
static uint8_t buf2[(RESOLUTION_HORIZONTAL * RESOLUTION_VERTICAL / 10 * BYTES_PER_PIXEL)];
static const enum ScreensEnum SONG_SCREENS[] = {
	SCREEN_ID_ESPRESSO,
	SCREEN_ID_BLUE,
    SCREEN_ID_BOHEMIAN_RHAPSODY,
    SCREEN_ID_YELLOW,
    SCREEN_ID_IMISS_YOU,
    SCREEN_ID_TONIGHT,
    SCREEN_ID_CRAZY,
	SCREEN_ID_MR_BRIGHTSIDE,
	SCREEN_ID_LAST_CHRISTMAS,
	SCREEN_ID_GNARLY,
    SCREEN_ID_HEAVY,
	SCREEN_ID_PROMISE,
};

static const int NUM_SONGS =12;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_LPUART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define BUF_SIZE 2048
#define HALF 1024
#define TIMEOUT 1000
int playing = 0; //initial flag - have we started the process?
int paused = 0;
int accel = 0;
int terminated = 0;
int first_half = 0;
int audio_received = 0; //flag - have we received any subsequent data since last callback?
int fft_ready = 0;

uint8_t next_cmd = 'S';
uint8_t rx_buffer[BUF_SIZE] = {0};
uint8_t skip_requested = 0;


float volume = 2.0f;
const float VOL_MIN = 0.5f;			// MAYBE [0,5] with different speaker
const float VOL_MAX = 3.5f;
const float VOL_STEP = 0.5f;
float speed = 0.0f;
const float SPEED_STEP = 0.2f;
const float SPEED_MAX = 1.6f;
const float SPEED_MIN = -1.6f;
const float MULT_MIN = 0.25f;   // or 0.5f
const float MULT_MAX = 4.0f;

float32_t pcm_float[HALF];
float32_t fft_input[FFT_SIZE];    // time-domain samples (copied from rx_buffer)
float32_t fft_output[FFT_SIZE];   // freq-domain complex output
float32_t fft_mag[FFT_SIZE/2] = {0};    // magnitude of first N/2 bins
arm_rfft_fast_instance_f32 fft_instance;

typedef struct {
	const char *title;
	const char *artist;
	uint16_t bpm;
	uint16_t duration;
} SongMetaData;

#define NUM_SONGS	13

const SongMetaData g_songs[13] = {
		[1] = {"Espresso", "Sabrina Carpenter", 104, 175},
		[2] = {"BLUE", "Billie Eilish", 142, 344},
		[3] = {"Bohemian Rhapsody", "Queen", 144, 360},
		[4] = {"Yellow", "Coldplay", 88, 270},
		[5] = {"I Miss You", "Blink-182", 111, 230},
		[6] = {"Tonight", "Pink Pantheress", 141, 177},
		[7] = {"Crazy", "Le Sserafim", 130, 165},
		[8] = {"Mr. Brightside", "The Killers", 148, 223},
		[9] = {"Last Christmas", "Wham!", 107, 278},
		[10] = {"Gnarly", "Katseye", 135, 142},
		[11] = {"Heavy", "The Marias", 97, 254},
		[12] = {"Promise", "Laufey", 76, 234},
};

uint16_t g_current = 0;
uint16_t g_bpm = 0;

void my_flush_cb(lv_display_t * display, const lv_area_t * area, uint8_t * px_map)
{
    /* The most simple case (also the slowest) to send all rendered pixels to the
     * screen one-by-one.  `put_px` is just an example.  It needs to be implemented by you. */
	//ili9488_SetDisplayWindow(area->x1, area->y1, area->x2, area->y2);
    uint16_t * buf16 = (uint16_t *)px_map; /* Let's say it's a 16 bit (RGB565) display */

    int32_t x, y;

    uint16_t w = (uint16_t)(area->x2 - area->x1 + 1);
    uint16_t h = (uint16_t)(area->y2 - area->y1 + 1);

    ili9488_DrawRGBImage(area->x1, area->y1, w, h, buf16);

    /* IMPORTANT!!!
     * Inform LVGL that flushing is complete so buffer can be modified again. */
    lv_display_flush_ready(display);
}

// create a global array of page names
// display the next page when this is called
static lv_obj_t *get_root_for_screen(enum ScreensEnum screen) {
    switch (screen) {
        case SCREEN_ID_ESPRESSO:          return objects.espresso;
        case SCREEN_ID_BLUE:              return objects.blue;
        case SCREEN_ID_BOHEMIAN_RHAPSODY: return objects.bohemian_rhapsody;
        case SCREEN_ID_YELLOW:            return objects.yellow;
        case SCREEN_ID_IMISS_YOU:         return objects.imiss_you;
        case SCREEN_ID_TONIGHT:           return objects.tonight;
        case SCREEN_ID_CRAZY:             return objects.crazy;
        case SCREEN_ID_MR_BRIGHTSIDE:     return objects.mr_brightside;
        case SCREEN_ID_LAST_CHRISTMAS:    return objects.last_christmas;
        case SCREEN_ID_GNARLY:            return objects.gnarly;
        case SCREEN_ID_HEAVY:             return objects.heavy;
        case SCREEN_ID_PROMISE:           return objects.promise;
        default:                          return NULL;
    }
}


static lv_obj_t *get_bar_for_screen(enum ScreensEnum screen) {
    switch (screen) {
        case SCREEN_ID_ESPRESSO:  		return objects.obj0;
        case SCREEN_ID_BLUE:         	return objects.obj5;
        case SCREEN_ID_BOHEMIAN_RHAPSODY:return objects.obj14;
        case SCREEN_ID_YELLOW:			return objects.obj16;
        case SCREEN_ID_IMISS_YOU:           return objects.obj18;
        case SCREEN_ID_TONIGHT:            return objects.obj20;
        case SCREEN_ID_CRAZY:        return objects.obj23;
        case SCREEN_ID_MR_BRIGHTSIDE:          return objects.obj31;
        case SCREEN_ID_LAST_CHRISTMAS:            return objects.obj34;
        case SCREEN_ID_GNARLY:            return objects.obj40;
        case SCREEN_ID_HEAVY:            return objects.obj42;
        case SCREEN_ID_PROMISE:            return objects.obj46;
        default:                         return NULL;
    }
}

static lv_obj_t *get_arc_for_screen(enum ScreensEnum screen) {
    switch (screen) {
        case SCREEN_ID_ESPRESSO:  		return objects.obj3;
        case SCREEN_ID_BLUE:         	return objects.obj10;
        case SCREEN_ID_BOHEMIAN_RHAPSODY:return objects.obj15;
        case SCREEN_ID_YELLOW:			return objects.obj17;
        case SCREEN_ID_IMISS_YOU:           return objects.obj19;
        case SCREEN_ID_TONIGHT:            return objects.obj21;
        case SCREEN_ID_CRAZY:        return objects.obj28;
        case SCREEN_ID_MR_BRIGHTSIDE:          return objects.obj32;
        case SCREEN_ID_LAST_CHRISTMAS:            return objects.obj39;
        case SCREEN_ID_GNARLY:            return objects.obj41;
        case SCREEN_ID_HEAVY:            return objects.obj43;
        case SCREEN_ID_PROMISE:            return objects.obj47;
        default:                         return NULL;
    }
}

static lv_obj_t *get_obj_from_root(lv_obj_t *parent, int idx) {
	// IDX:
	// 1 = PAUSE/PLAY
	// 2 = LEFT
	// 3 = RIGHT
	// 5 = SPEED
    if (parent == NULL) return NULL;

    uint32_t cnt = lv_obj_get_child_cnt(parent);
    if (cnt == 0) return NULL;


    printf("%d \n\r", cnt-idx);
    return lv_obj_get_child(parent, cnt - idx);
}


static lv_obj_t *get_obj_for_screen(enum ScreensEnum screen, int idx) {
    lv_obj_t *root = get_root_for_screen(screen);
    if (!root) return NULL;
    return get_obj_from_root(root, idx);
}


static void update_play_pause_button(enum ScreensEnum screen) {
	// toggle the button from prev state when called
    lv_obj_t *btn = get_obj_for_screen(screen, 1);
    if (!btn) return;

    if (paused) {
        // show "play" image (PRESSED state uses img_play_332)
        printf("changing to play pic\n\r");
        lv_imagebutton_set_state(btn, LV_IMAGEBUTTON_STATE_PRESSED);
    } else {
        // show "pause" image (RELEASED state uses img_pause)
        printf("changing to pause pic\n\r");
        lv_imagebutton_set_state(btn, LV_IMAGEBUTTON_STATE_RELEASED);
    }
}



static void update_left_right_gesture(enum ScreensEnum screen, bool is_left, char dir) {
	lv_obj_t *btn;
	if (is_left) {
	    btn = get_obj_for_screen(screen, 2);

	}
	else{ // is right
		 btn = get_obj_for_screen(screen, 3);
	}
	   if (!btn) return;

	   if (dir == 'l') { // left
		   lv_imagebutton_set_state(btn, LV_IMAGEBUTTON_STATE_RELEASED);
	   } else if (dir == 'u'){
		   lv_imagebutton_set_state(btn, LV_IMAGEBUTTON_STATE_PRESSED);
	   }
	   else if (dir == 'r'){
	   		   lv_imagebutton_set_state(btn, LV_IMAGEBUTTON_STATE_CHECKED_RELEASED);
	   	   }

	   else if (dir == 'd'){
		   lv_imagebutton_set_state(btn, LV_IMAGEBUTTON_STATE_CHECKED_PRESSED);
	   }
}

static void update_volume_arc(enum ScreensEnum screen, int volume) {

	volume = volume * 2;
	 lv_obj_t *arc = get_arc_for_screen(screen);
	int scaled_volume = (volume* 100) / 7;
	lv_arc_set_value(arc, scaled_volume);

}

float map_speed_to_multiplier(float speed)
{
    const float SPEED_MIN = -1.6f;
    const float SPEED_MAX =  1.6f;

    // Pick the range YOU want:
    const float MULT_MIN = 0.25f;   // or 0.5f
    const float MULT_MAX = 4.0f;    // or 2.0f

    // clamp speed
    if (speed > SPEED_MAX) speed = SPEED_MAX;
    if (speed < SPEED_MIN) speed = SPEED_MIN;

    // linear map from [-1.6 , +1.6] to [MULT_MIN , MULT_MAX]
    float norm = (speed - SPEED_MIN) / (SPEED_MAX - SPEED_MIN);
    float mult = MULT_MIN + norm * (MULT_MAX - MULT_MIN);
    return mult;
}

static void update_speed_label(enum ScreensEnum screen, int speed) {
    lv_obj_t *label = get_obj_for_screen(screen, 5);

    float multiplier = map_speed_to_multiplier(speed);
    char buf[32];
    snprintf(buf, sizeof(buf), "Speed:\n%f", speed);

    lv_label_set_text(label, buf);
}



static float Audio_GetElapsedSeconds(void)
{
    return (float)g_samples_played / (float)AUDIO_SAMPLE_RATE_HZ;
}

static uint32_t Audio_GetElapsedMs(void)
{
    // avoid float if you want:
    return (g_samples_played * 1000u) / AUDIO_SAMPLE_RATE_HZ;
}



void HAL_GPIO_EXTI_Callback(uint16_t pin) {
	//printf("Gesture interrupt fired\n\r");
	if (pin == GESTURE1_INT_Pin) PAUSE_SKIP_Sensor.Gesture_Flag = 1;
	if (pin == GESTURE2_INT_Pin) VOLUME_SPEED_Sensor.Gesture_Flag = 1;
	//disableGesture();
}

static void RequestNextChunk(uint8_t *dest) {
	if (paused) {
	        // Don’t ask Python for more audio while paused
	        return;
	    }
	uint8_t cmd = next_cmd;
	HAL_UART_Transmit(&huart2, &cmd, 1, TIMEOUT);
	// if we want to return to the normal speed
	if (next_cmd == 'A' || next_cmd == 'D'  || next_cmd == 'K' || next_cmd == 'B' || next_cmd == 'S') {
			next_cmd = 'S';
	}
	HAL_UART_Receive_DMA(&huart2, dest, HALF);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  // If we aren't playing yet, start DAC DMA + timer
  if (!playing) {
      HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)rx_buffer, BUF_SIZE, DAC_ALIGN_8B_R);
      HAL_TIM_Base_Start(&htim5);
      RequestNextChunk(&rx_buffer[HALF]);
      playing = 1;
  }
  audio_received = 1;
}

// DAC DMA Callback when it is halfway through the buffer
// called when the DAC DMA has finished transferring the first half of the DAC buffer to the peripheral
// DMA reads sample from second half of buffer
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
  if (!audio_received) {
  	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
  	HAL_TIM_Base_Stop(&htim5);
  	playing = 0;
      HAL_UART_AbortReceive(&huart2);
      HAL_UART_Receive_DMA(&huart2, &rx_buffer[0], HALF);
  }
  else {
      // Copy SECOND half (512..1023) before reusing it

	  /* FFT LOGIC START */
      for (int i = 0; i < FFT_SIZE; i++) {
          fft_input[i] = ((float)rx_buffer[HALF + i] - 128.0f) / 128.0f;

      }

      fft_ready = 1;

      // === CMSIS-DSP FLOAT CONVERSION (second half) ===
      for (int i = 0; i < HALF; i++) {
          pcm_float[i] = ((float32_t)rx_buffer[HALF + i] - 128.0f) / 128.0f;
      }

      // === APPLY VOLUME ===
      arm_scale_f32(pcm_float, volume, pcm_float, HALF);

      // === BACK TO PCM BYTES ===
      for (int i = 0; i < HALF; i++) {
          float32_t f = pcm_float[i] * 128.0f + 128.0f;
          if (f < 0) f = 0;
          if (f > 255) f = 255;
          rx_buffer[HALF + i] = (uint8_t)f;
      }

	  /* FFT LOGIC END */
      /* VOLUME CONTROL END */

      g_samples_played += HALF;

      RequestNextChunk(&rx_buffer[0]);
		audio_received = 0;
  }
}

// DAC DMA Callback when it has reached the end of the buffer, DMA to DAC
// called when DMA has finished transferring the SECOND HALF to the DAC and will wrap to the FIRST HALF
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
	if (!audio_received) {
	    	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
	    	HAL_TIM_Base_Stop(&htim5);
	    	playing = 0;
	        HAL_UART_AbortReceive(&huart2);
	        HAL_UART_Receive_DMA(&huart2, &rx_buffer[0], HALF); //
	    }
	    else {

	  	  /* FFT LOGIC START */
	    	// Copy FIRST half (0..511) before reusing it
	        for (int i = 0; i < FFT_SIZE; i++) {
	            fft_input[i] = ((float)rx_buffer[i] - 128.0f) / 128.0f;
	        }
	        fft_ready = 1;

	        // === CMSIS FLOAT CONVERSION (first half) ===
	              for (int i = 0; i < HALF; i++) {
	                  pcm_float[i] = ((float32_t)rx_buffer[i] - 128.0f) / 128.0f;
	              }

	              // === SCALE ===
	              arm_scale_f32(pcm_float, volume, pcm_float, HALF);

	              // === BACK TO PCM BYTE ===
	              for (int i = 0; i < HALF; i++) {
	                  float32_t f = pcm_float[i] * 128.0f + 128.0f;
	                  if (f < 0) f = 0;
	                  if (f > 255) f = 255;
	                  rx_buffer[i] = (uint8_t)f;
	              }
	      /* VOLUME CONTROL END */
	        g_samples_played += HALF;

	  	  /* FFT LOGIC END */

	    	RequestNextChunk(&rx_buffer[HALF]);
	    	audio_received = 0;
	    }
}
void ReadHeaderFromHost(void)
{
    char buf[32];       // plenty of room: "HDR|123\n\0"
    uint8_t ch;
    int idx = 0;

    // Read until newline or buffer full
    while (idx < (int)(sizeof(buf) - 1)) {
        if (HAL_UART_Receive(&huart2, &ch, 1, HAL_MAX_DELAY) != HAL_OK) {
            // UART error, try again
            continue;
        }
        if (ch == '\n') {
            break;
        }
        buf[idx++] = (char)ch;
    }
    buf[idx] = '\0';

    int song_idx = 0;   // default if parsing fails

    if (strncmp(buf, "HDR|", 4) == 0) {
        // parse the integer after "HDR|"
        song_idx = atoi(&buf[4]);
    }

    // Clamp to valid range
    if (song_idx < 0 || song_idx >= NUM_SONGS) {
        song_idx = 0;
    }

    // Update global "current song" index
    g_current = (uint16_t)song_idx;

    // Pull BPM from  metadata table
    g_bpm = g_songs[g_current].bpm;

    // Optional but super useful for debugging:
    printf("Header RX: '%s' -> idx=%d, bpm=%d, mode=%d\r\n",
           buf, song_idx, g_bpm);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_SPI2_Init();
	MX_USART2_UART_Init();
	MX_DAC1_Init();
	MX_TIM5_Init();
	MX_TIM2_Init();
	MX_I2C1_Init();
	MX_I2C3_Init();
	MX_LPUART1_UART_Init();
	/* USER CODE BEGIN 2 */

	ili9488_Init();

	HAL_UART_Receive_DMA(&huart2, &rx_buffer[0], HALF);

	lv_init();
	lv_tick_set_cb(HAL_GetTick);

	lv_disp_t * display1 = lv_display_create(RESOLUTION_HORIZONTAL, RESOLUTION_VERTICAL);
	lv_display_set_rotation(display1, LV_DISPLAY_ROTATION_0); //setting it to LV_DISPLAY_ROTATION_180 made no difference
	lv_display_set_buffers(display1, buf1, buf2, sizeof(buf1), LV_DISPLAY_RENDER_MODE_PARTIAL);

	lv_display_set_flush_cb(display1, my_flush_cb);

	ui_init();

	int curr_index = 0;
	enum ScreensEnum curr_screen = SONG_SCREENS[curr_index];

	sensor_init(&PAUSE_SKIP_Sensor, &hi2c3);
	sensor_init(&VOLUME_SPEED_Sensor, &hi2c1);

	begin(&PAUSE_SKIP_Sensor);
	begin(&VOLUME_SPEED_Sensor);

	enableGesture(&PAUSE_SKIP_Sensor);
	enableGesture(&VOLUME_SPEED_Sensor);

	int curr_progress = 0;

	// get the bar for the current screen
	lv_obj_t *label_obj = get_obj_for_screen(curr_screen,5);
	lv_obj_add_flag(label_obj, LV_OBJ_FLAG_HIDDEN);
	lv_obj_t *bar_obj = get_bar_for_screen(curr_screen);
	int curr_max_value = lv_bar_get_max_value(bar_obj);
	lv_bar_set_value(bar_obj, curr_progress, LV_ANIM_OFF);

	uint32_t last_update = HAL_GetTick();

	arm_rfft_fast_init_f32(&fft_instance, FFT_SIZE);


	const uint8_t S = 'S';
	HAL_UART_Transmit(&huart2, &S, 1, TIMEOUT);

  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
	while (1)
	{
	  if (fft_ready)
	  {
		 fft_ready = 0;

		 // 1) Run FFT on fft_input (already filled in callbacks)
		 arm_rfft_fast_f32(&fft_instance, fft_input, fft_output, 0); // 0 = forward FFT

		 // 2) Compute magnitude of first N/2 complex bins
		 // fft_output is [Re0, Im0, Re1, Im1, ..., Re(N/2-1), Im(N/2-1)]
		 // fft_mag: how strong each frequency is in the audio chunk you just analyzed
		 arm_cmplx_mag_f32(fft_output, fft_mag, FFT_SIZE/2);
		 update_fft_bars();
	 }
	  while (hdma_tim2_ch1.State != HAL_DMA_STATE_READY || wr_buf_p != 0);
		 animate_bars_pulse();
		 led_render();

		 uint32_t time_till_next = lv_timer_handler();

		 uint32_t now = HAL_GetTick();

		 if (now - last_update >= 1000) {
			  last_update = now;

			  if (!paused) {
				  uint32_t elapsed_s = Audio_GetElapsedSeconds();

				  // clamp to bar max (which you set to song duration)
				  if (elapsed_s > (uint32_t)curr_max_value) {
					  elapsed_s = (uint32_t)curr_max_value;
				  }
				  bar_obj = get_bar_for_screen(curr_screen);
				  curr_progress = elapsed_s;
				  lv_bar_set_value(bar_obj, curr_progress, LV_ANIM_OFF);
			  }

			  // treat "bar is at max" as end-of-song
			  if (curr_progress >= curr_max_value) {
				  // reached end -> go to next song in playlist
				  curr_index = (curr_index + 1) % 12;
				  curr_screen = SONG_SCREENS[curr_index];
				  loadScreen(curr_screen);
				  lv_obj_t *label_obj = get_obj_for_screen(curr_screen,5);

				  lv_obj_add_flag(label_obj, LV_OBJ_FLAG_HIDDEN);


				  update_play_pause_button(curr_screen);

				  bar_obj = get_bar_for_screen(curr_screen);
				  if (bar_obj == NULL) {
					  while (1) {
						  // mapping bug
					  }
				  }

				  curr_max_value = lv_bar_get_max_value(bar_obj);
				  curr_progress = 0;
				  g_samples_played = 0;   // <-- reset for new track

				  lv_bar_set_value(bar_obj, curr_progress, LV_ANIM_OFF);
			  }
		  }


		  if(gestureAvailable(&PAUSE_SKIP_Sensor)) {
					 int gesture2 = readGesture(&PAUSE_SKIP_Sensor);

					 if(gesture2 != -1) {
			  	  			 printf("PAUSE_SKIP Gesture read : %d\n\r", gesture2);
					 }

					 if (gesture2 == 1){
						   if(!paused){
								// was previously playing, now paused
								const uint8_t P= 'P';

								paused = 1;
								HAL_UART_Transmit(&huart2, &P, 1, TIMEOUT);

							}

						   // PAUSE IS ON THE RIGHT
						   /* UPDATE GESTURE ON DISPLAY AND PAUSE PLAY */
							update_left_right_gesture(curr_screen, false, 'd');
							update_play_pause_button(curr_screen);
					 }

					 else if (gesture2 == 0){
						 if (paused) { // ws previously playing, now need to pause
							// was previously paused, now resume
							const uint8_t S = 'S';

							paused = 0;
							HAL_UART_Transmit(&huart2, &S, 1, TIMEOUT);

						}

						update_left_right_gesture(curr_screen, false, 'u');
						update_play_pause_button(curr_screen);

					 }

					 else if (gesture2 == 2 ) {
						 // next_cmd = 'A';
						next_cmd = 'B';
						curr_index = (curr_index - 1 + 12) % 12;
						curr_screen = SONG_SCREENS[curr_index];
						loadScreen(curr_screen);
						lv_obj_t *label_obj = get_obj_for_screen(curr_screen, 5);

						lv_obj_add_flag(label_obj, LV_OBJ_FLAG_HIDDEN);

						update_left_right_gesture(curr_screen, false, 'l');
						g_samples_played = 0;

					 }

					 else if (gesture2 == 3) {
						 // next_cmd = 'D';
						skip_requested = 1;
						next_cmd = 'K';
						update_left_right_gesture(curr_screen, false, 'r');
						g_samples_played = 0;
					 }
			  }

			  if(gestureAvailable(&VOLUME_SPEED_Sensor)) {

				  int gesture = readGesture(&VOLUME_SPEED_Sensor);

				  if (gesture == 0 ) {
						 volume += VOL_STEP;
						 if (volume > VOL_MAX) volume = VOL_MAX;
						 printf("Volume UP: %.2f\n\r", volume);
						 update_volume_arc(curr_screen, volume);
						 update_left_right_gesture(curr_screen, true, 'u');
				  }
				  else if (gesture == 1) {
						volume -= VOL_STEP;
						if (volume < VOL_MIN) volume = VOL_MIN;
						printf("Volume DOWN: %.2f\n\r", volume);
						update_volume_arc(curr_screen, volume);
						update_left_right_gesture(curr_screen, true, 'd');
					 }

				  else if (gesture == 2) {
						next_cmd = 'D';
			  		    printf("Slowing down\n\r");
						if (speed > SPEED_MIN) speed -= SPEED_STEP;
						update_left_right_gesture(curr_screen, true, 'l');
				  }

				  else { // GESTURE = 2
						next_cmd = 'A';
						if (speed < SPEED_MAX) speed += SPEED_STEP;
						update_left_right_gesture(curr_screen, true, 'r');
				  }
			  }
			if (skip_requested) {
				printf("before skip requested, paused state: %d\n\r", paused);
				skip_requested = 0; // reset flag
				curr_index = (curr_index + 1) % 12;
				curr_screen = SONG_SCREENS[curr_index];
				loadScreen(curr_screen);
				lv_obj_t *label_obj = get_obj_for_screen(curr_screen, 5);
				lv_obj_add_flag(label_obj, LV_OBJ_FLAG_HIDDEN);
				g_samples_played = 0;
			}
	}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T5_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_DISABLE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10805D88;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x10805D88;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 460800;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 59;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 2;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, DISPLAY_CS_Pin|DISPLAY_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DISPLAY_DC_RS_GPIO_Port, DISPLAY_DC_RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : GESTURE2_INT_Pin */
  GPIO_InitStruct.Pin = GESTURE2_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GESTURE2_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PF0 PF1 PF2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PF7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PC3 PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : GESTURE1_INT_Pin */
  GPIO_InitStruct.Pin = GESTURE1_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GESTURE1_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DISPLAY_CS_Pin DISPLAY_RESET_Pin */
  GPIO_InitStruct.Pin = DISPLAY_CS_Pin|DISPLAY_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE9 PE10 PE11 PE12
                           PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12
                          |GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE14 PE15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM1_COMP1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF14_TIM15;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PD8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : DISPLAY_DC_RS_Pin */
  GPIO_InitStruct.Pin = DISPLAY_DC_RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DISPLAY_DC_RS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF13_SAI2;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&hlpuart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
