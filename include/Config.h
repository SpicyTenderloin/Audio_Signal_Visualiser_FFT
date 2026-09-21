#pragma once

#include <Adafruit_ILI9341.h>
#include "driver/adc.h"

// -------------------- TFT Pins --------------------
#define TFT_CS 5
#define TFT_DC 21
#define TFT_RST 22
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_MISO 19

// -------------------- MIC (ADC) -------------------
#define MIC_CH ADC1_CHANNEL_0 // GPIO36

// -------------------- Buttons --------------------
#define BTN_AGG_DOWN 12 // aggregation down
#define BTN_AGG_UP 13   // aggregation up
#define BTN_N_DOWN 15   // N-
#define BTN_N_UP 2      // N+
#define BTN_PAUSE 0     // toggle pause
#define BTN_ZOOM_UP 4   // horizontal zoom in (+Fmax)
#define BTN_ZOOM_DN 16  // horizontal zoom out (-Fmax)

// -------------------- VU LEDs ---------------------
#define VU1 14
#define VU2 27
#define VU3 26
#define VU4 25
#define VU5 33
#define VU6 32

// -------------------- Display layout --------------
static const int SCREEN_W = 320;
static const int SCREEN_H = 240;

static const int LM = 30;
static const int RM = 16;
static const int TOP = 36;     // room for centered title
static const int HUD_H = 20;   // bottom HUD band
static const int BOT_GAP = 18; // gap for X labels above HUD

static const int PLOT_X = LM;
static const int PLOT_Y = TOP;
static const int PLOT_W = SCREEN_W - LM - RM;
static const int PLOT_H = SCREEN_H - TOP - HUD_H - BOT_GAP;
static const int BASE_Y = PLOT_Y + PLOT_H - 1;

// -------------------- Colors ----------------------
#define COL_BG ILI9341_BLACK
#define COL_AX ILI9341_WHITE
#define COL_TEXT ILI9341_WHITE
#define COL_GRID 0x2104
#define COL_GRID_MINOR 0x1082
#define COL_TITLE ILI9341_WHITE
#define COL_LINE ILI9341_CYAN

// -------------------- Buffers ---------------------
// esp-dsp's bundled FFT table generator caps out at 4096 (CONFIG_DSP_MAX_FFT_SIZE,
// baked into this precompiled library) - dsps_fft2r_init_fc32() fails above that.
#define FFT_MAX 4096

// -------------------- Magnitude scaling -----------
static const float ADC_FS = 4095.0f; // 12-bit ADC raw counts

// -------------------- Controls ---------------------
static const uint16_t DEBOUNCE_MS = 25;

// -------------------- Sample rate range -------------
static const uint32_t FS_MIN_HZ = 2000;
static const uint32_t FS_MAX_HZ = 200000;
