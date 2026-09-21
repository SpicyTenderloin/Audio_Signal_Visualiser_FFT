# Audio Signal Visualiser

A real-time audio spectrum analyser and oscilloscope built on an ESP32 and a 320×240 ILI9341 SPI
TFT. A microphone feeds ADC1 via the ESP32's I2S peripheral in built-in-ADC/DMA mode, giving
continuous, CPU-cheap sampling up to 200kHz; the display renders either a live FFT spectrum or the
raw time-domain waveform, with hardware buttons and a serial console for control.

## Features

- **Two display modes**, swapped with the PAUSE button or `mode=` over serial. Each keeps its own
  sample rate, so changing one mode's never affects the other's:
  - **Spectrum Analyser** — live FFT magnitude plot (linear or log frequency axis, dBFS or linear
    % full-scale amplitude axis, adjustable FFT length and bin aggregation, optional Hann window).
  - **Waveform Analyser** — raw time-domain trace ("scope" view), Y axis in real volts, X-axis
    zoom tied to sample rate with a built-in anti-aliasing floor so zooming out can't quietly
    under-sample the signal.
- **I2S/DMA ADC capture**: the ADC is driven continuously by DMA, not polled — no per-sample CPU
  cost, so sample rate isn't limited by driver call overhead.
- **Auto-tuned fidelity**: given a target max frequency, `recommend_fs_n()` picks the sample
  rate/FFT length combination that best fills the plot's pixel width while keeping enough
  oversampling margin to stay clean of aliasing (there's no analog anti-alias filter on the mic
  input, so this matters).
- **ADC calibration** (see below): the chip's factory calibration data is used automatically, and
  there's an on-device interactive tool to calibrate against a real known voltage.
- **Free-running rendering**: no frame-rate cap - frames come as fast as the FFT and drawing allow,
  with the measured FPS on the HUD. The FFT is real-input (a half-length complex FFT plus a split
  step, about twice as fast as a full complex one), and redraws are differential (one write per
  changed plot column) over 80MHz SPI, with flicker-free HUD digit updates.
- A 6-LED VU meter driven off the same peak-sample tracking as the display.
- A custom pixel font (`Aurora7pt7b`/`Aurora4pt7b`, in `include/`) used for on-screen titles.

## Hardware

- ESP32 dev board
- ILI9341 320×240 SPI TFT
- Electret/analog microphone (or any 0–3.3V analog signal) into ADC1 channel 0
- 7 momentary buttons, 6 LEDs (VU meter)

| Signal          | GPIO |
|------------------|------|
| TFT CS           | 5    |
| TFT DC           | 21   |
| TFT RST          | 22   |
| TFT MOSI         | 23   |
| TFT SCLK         | 18   |
| TFT MISO         | 19   |
| Mic (ADC1_CH0)   | 36   |
| BTN AGG−         | 12   |
| BTN AGG+         | 13   |
| BTN N−           | 15   |
| BTN N+           | 2    |
| BTN PAUSE / MODE | 0    |
| BTN ZOOM+        | 4    |
| BTN ZOOM−        | 16   |
| VU LED 1–6       | 14, 27, 26, 25, 33, 32 |

Pin assignments live in `include/Config.h`.

## Building and flashing

This is a [PlatformIO](https://platformio.org/) project (`framework = arduino`, `board =
esp32dev`).

```sh
pio run                # build
pio run -t upload      # build and flash
pio device monitor      # open the serial console (115200 baud)
```

## Usage

### Button controls

| Button        | Spectrum Analyser mode              | Waveform Analyser mode           |
|----------------|--------------------------------------|------------------------------------|
| PAUSE          | switch to Waveform Analyser          | switch to Spectrum Analyser        |
| AGG− / AGG+    | bin aggregation finer/coarser        | Y (amplitude) zoom out/in          |
| N− / N+        | FFT length down/up                   | no effect                          |
| ZOOM− / ZOOM+  | horizontal zoom (Fmax ∓5%)           | X (time) zoom via sample rate ∓5%  |

Holding PAUSE has a separate meaning inside calibration mode - see below.

### Serial console

Connect at 115200 baud and type `help` for the full list. Highlights:

| Command | Effect |
|---|---|
| `stats` | print current settings |
| `rawdump` | diagnostic: print raw I2S words from the ADC, before the sample pairing is reduced |
| `fs=2000..200000` | set sample rate (Hz) for the current mode |
| `n=32\|64\|128\|256\|512\|1024\|2048\|4096\|8192` | set FFT length |
| `agg=1..64` | bin aggregation |
| `xscale=lin\|log` | frequency axis scale |
| `yscale=db\|lin` | amplitude axis scale |
| `fmax=Hz` / `fmax=nyq` | horizontal zoom (max plotted frequency) / follow Nyquist |
| `ymax=<dB>` / `ymin=<dB>` | dBFS axis bounds |
| `hann=0\|1` | Hann window off/on |
| `mode=fft\|wave` | switch display mode |
| `pause` | toggle pause |
| `cal` | enter interactive ADC calibration |
| `cal cancel` | exit calibration without saving |
| `cal clear` | erase the saved calibration (asks for y/n confirmation) |

`fmax=`/`fmax=nyq` also print a suggested `fs=`/`n=` for the best fit to the display width at that
frequency range.

## ADC calibration

ADC readings are converted to volts through one of three tiers, resolved automatically at boot and
logged over serial (also visible via `stats`):

1. **A saved user calibration** (NVS flash), if one has been run — see below.
2. Otherwise, the chip's **factory eFuse calibration curve** (`esp_adc_cal`), if this chip has one
   burned in.
3. Otherwise, the nominal 3.3V / 4095-count ratio.

### Running an interactive calibration

Apply a known voltage to the mic input pin (e.g. from a bench supply) and type `cal` over serial.
The screen shows a target voltage and a live raw-ADC readout:

1. **ZOOM ±** cycles through 5 recommended target voltages (0 / 0.825 / 1.65 / 2.475 / 3.3V);
   **AGG ±** fine-adjusts the target by ±0.01V for a custom value.
2. Once the applied voltage matches the target, press **PAUSE** to capture that point (it averages
   ~256 live samples).
3. Repeat for as many points as you like (2 minimum). **N−** undoes the last captured point.
4. **Hold PAUSE** for about a second to finish: a line is fit to the points (exact for 2, a
   least-squares regression for more) and saved to flash, where it persists across power cycles
   until `cal clear` erases it.

## Project structure

```
include/, src/
  Config.h            pin/layout/color constants
  Globals.h/.cpp       shared mutable state
  Settings.h/.cpp      setters for all adjustable settings
  Controls.h/.cpp      button polling/debounce, dispatch by mode
  AudioCapture.h/.cpp  I2S/DMA ADC capture into a circular buffer
  DSPUtils.h/.cpp      FFT windowing, axis-label formatting, fidelity recommendation
  Calibration.h/.cpp   ADC calibration (eFuse + interactive multi-point NVS-backed tool)
  Display.h/.cpp       all drawing: axes, HUD, spectrum/waveform/calibration screens
  SpectrumTask.h/.cpp  the per-frame FFT/draw loop (runs on core 0)
  SerialConsole.h/.cpp serial command parsing
  VUMeter.h/.cpp       VU LED driver
  Aurora4pt7b.h, Aurora7pt7b.h, Aurora10pt7b.h
                       a custom pixel font family, used for on-screen titles
main.cpp               setup()/loop()
```
