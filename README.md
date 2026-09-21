# Audio Signal Visualiser

A real-time audio spectrum analyser and oscilloscope built on an ESP32 and a 320×240 ILI9341 SPI
TFT. A microphone feeds ADC1 through the ESP32's continuous-ADC/DMA driver; the display renders
either a live FFT spectrum or the raw time-domain waveform, with hardware buttons and a serial
console for control. Frames are computed and drawn as a two-core pipeline and run as fast as the
hardware allows, with the measured frame rate shown on the display.

## Features

- **Two display modes**, swapped with the PAUSE button or `mode=` over serial. Each keeps its own
  sample rate and ADC averaging, so changing one mode's never affects the other's:
  - **Spectrum Analyser**: live FFT magnitude plot (linear or log frequency axis, dBFS or linear
    % full-scale amplitude axis, FFT lengths from 32 to 8192, adjustable bin aggregation, optional
    Hann window).
  - **Waveform Analyser**: raw time-domain trace ("scope" view), Y axis in real volts, X-axis
    zoom tied to sample rate with a built-in anti-aliasing floor so zooming out can't quietly
    under-sample the signal.
- **Continuous-ADC/DMA capture**: the ADC is driven continuously by DMA, not polled, so there is
  no per-sample CPU cost and the sample rate isn't limited by driver call overhead. It is
  **oversampled and averaged**: by default the ADC runs at the highest multiple of the chosen
  sample rate that fits under 200kHz (192kHz at the 48kHz default), and each group of conversions
  is averaged down to that rate. That cuts random ADC noise (about 6dB at 48kHz) at no cost to the
  FFT. The `avg=` serial command sets the averaging explicitly.
- **Free-running, pipelined rendering**: there is no frame-rate target. The FFT for the next frame
  is computed on one core while the previous frame is drawn on the other, so the frame time is the
  longer of the two rather than their sum. Redraws are differential (one write per changed plot
  column) over 80MHz SPI, with flicker-free HUD digit updates. The measured FPS is shown on the
  HUD.
- **Real-input FFT**: the mic signal is real, so the FFT runs as a half-length complex FFT plus a
  cheap split step, about twice as fast as a full complex FFT of the same length.
- **ADC calibration** (see below): the chip's factory calibration data is used automatically, and
  there's an on-device interactive tool to calibrate against a real known voltage.
- A 6-LED VU meter driven off the same peak-sample tracking as the display.
- A custom pixel font (`Aurora7pt7b`/`Aurora4pt7b`, in `include/`) used for on-screen titles.

## Hardware

- ESP32 dev board
- ILI9341 320×240 SPI TFT
- Electret/analog microphone (or any 0-3.3V analog signal) into ADC1 channel 0, through an analog
  low-pass filter (see [Signal path and sample rate](#signal-path-and-sample-rate))
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
esp32dev`) on the [pioarduino](https://github.com/pioarduino/platform-espressif32) platform, which
carries the current Arduino-ESP32 core (3.3.12, on ESP-IDF 5.5.5). PlatformIO's own `espressif32`
platform still ships the old 2.0.17 core, so `platformio.ini` points at a pinned pioarduino
release instead. The first build downloads the toolchain and framework (a few GB).

```sh
pio run                # build
pio run -t upload      # build and flash
pio device monitor      # open the serial console (115200 baud)
```

Run these from a normal Windows terminal (VS Code's PlatformIO buttons, PowerShell or cmd), not
Git Bash: the platform's tool installer refuses to run under MSYS. Don't add `lib_ignore =
esp-dsp` to `platformio.ini` either; on this platform it removes the FFT library from the build.

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
| `stats` | print current settings and live performance figures (see below) |
| `rawdump` | diagnostic: print raw ADC words as pairs, to check for repeated samples |
| `fs=2000..200000` | set sample rate (Hz) for the current mode |
| `avg=1..100\|auto` | ADC conversions averaged into each sample, for the current mode (`auto` = as many as the ADC's top rate allows) |
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
frequency range. The suggestion never goes below the alias-safe sample rate, and prefers the
smaller FFT when a larger one would only fill a few more pixels.

Boot defaults (in `include/Config.h`): Fs 48kHz, N 4096 (about 11.7Hz bins), Fmax 3kHz, Y axis
-40 to 0 dBFS, Hann window on, ADC averaging automatic.

### Reading `stats`

```
Mode=FFT  Fs=48000  N=4096  Df=11.72Hz  BPP=1  X=LIN  Y=dB  Fmax=3000Hz  Hann=1
Y range: [-40.0, 0.0] dBFS
FPS=453.0  Compute=2.17ms (FFT=1.54ms)  Draw=0.95ms
Capture: 48008 samples/s arriving, 48000 set (ADC at 192000 Hz, 4 averaged per sample, avg=auto)
ADC cal: source=efuse  volts=raw*0.000738+0.142000
Heap: free=104672 bytes  largest block=63476 bytes
Chip: 240 MHz  ESP-IDF v5.5.5
```

- **FPS**: frames drawn per second, averaged over the last second. Compute and Draw are the two
  pipeline stages for the most recent frame; the frame rate is set by the slower one. Compute is
  windowing, FFT and power spectrum, and FFT is the FFT alone.
- **Capture**: samples per second actually arriving from the ADC, next to the rate that was
  requested, plus what the ADC hardware is set to. If "arriving" doesn't match "set", the ADC is
  not running at the rate the display assumes.
- **Heap**: free memory and the largest free block. RAM, not speed, is what limits the FFT length.
- These figures are averages or last-frame values, so right after a setting change they still
  describe the old setting for up to a second.

## Signal path and sample rate

The mic passes through an analog low-pass filter before the ADC. Its cutoff is currently about
3kHz (a Bode-plot sweep shows the response roughly flat to there, then falling steeply: about 40dB
down at 6kHz relative to 1kHz, though not a brick wall). The plan is to raise the cutoff to about
20kHz, and the sample-rate logic is built for that:

- With a 20kHz passband, Fs has to stay comfortably above twice that cutoff whatever range is
  displayed, or content between Fmax and the cutoff folds into the view. `MIN_ALIAS_SAFE_FS_HZ`
  (2.4 times the cutoff, 48kHz) is the floor, and the boot default. `recommend_fs_n()` never goes
  below it.
- There is deliberately no digital low-pass filter. The averaging in the ADC path is a boxcar, a
  weak filter that is worth having for noise but does not replace the analog one.
- Oversampling and averaging lowers random noise by about 10·log10(k) dB for k averaged
  conversions. It cannot remove ADC distortion, and the ESP32's ADC needs about 4µs per
  conversion, so 200kHz (`ADC_HW_MAX_HZ`) is near its limit. If the display looks noisy or
  distorted at high ADC rates, lower that constant or use `avg=`.

Open item: the waveform mode's zoom-out floor still reflects the current ~3kHz filter. Honouring a
20kHz cutoff there would cap the waveform at about 5.7ms of time on screen, so the waveform needs a
different time-base approach (for example a peak-detect mode) before the filter is changed.

## How it works

- **Capture task (core 0, highest priority)** drains the ADC's DMA frames, averages groups of
  conversions, and writes samples into a circular buffer. It wakes the compute task each time new
  samples land.
- **Compute task (core 0)** takes the newest window of samples, applies the Hann window, runs the
  real FFT and builds running sums of the per-bin power (or copies raw samples in waveform mode)
  into one of two frame buffers.
- **Draw task (core 1)** draws finished frames and owns everything that touches the display: axes,
  HUD, plot and the calibration screen. The two frame buffers circulate between the tasks through
  a pair of queues, and a frame computed before a settings change is discarded rather than drawn
  onto axes it no longer matches.
- **The Arduino loop task (core 1)** polls buttons and the serial port. It is given a higher
  priority than the draw task, so input handling preempts a draw within microseconds.
- Both cores can now run flat out, so the idle tasks are taken off the task watchdog
  (`esp_task_wdt_reconfigure()`, in `SpectrumTask.cpp`).

## Performance

Measured on the board at Fs 48kHz, N 4096, with the ADC at 192kHz: about 450 FPS, with the compute
stage at about 2.2ms (FFT about 1.5ms) and drawing at about 1ms. Smaller FFT lengths run faster,
larger ones slower (N 8192 spends about 3ms in the FFT alone). At very small N the ceiling becomes
the ADC frame size (64 samples per frame, so Fs/64 frames per second).

## Limits

- **FFT length**: 8192 is the ceiling. esp-dsp's precompiled twiddle table caps out at 4096
  points (`CONFIG_DSP_MAX_FFT_SIZE`), and the real-input FFT runs at half the length, which is what
  reaches 8192. Going further would need a rebuilt esp-dsp with a raised limit, plus a diet on the
  per-N buffers: at 8192 about 105KB of heap remains, and the largest single block is about 63KB.
- **Sample rate**: 2kHz to 200kHz, and the ADC's own minimum rate (20kHz) limits how little
  averaging is possible at low Fs.

## ADC calibration

ADC readings are converted to volts through one of three tiers, resolved automatically at boot and
logged over serial (also visible via `stats`):

1. **A saved user calibration** (NVS flash), if one has been run - see below.
2. Otherwise, the chip's **factory eFuse calibration curve** (the ADC line-fitting calibration
   driver), if this chip has one burned in.
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
  Config.h            pin/layout/color constants and boot defaults
  Globals.h/.cpp       shared mutable state
  Settings.h/.cpp      setters for all adjustable settings
  Controls.h/.cpp      button polling/debounce, dispatch by mode
  AudioCapture.h/.cpp  continuous-ADC/DMA capture, averaging, circular buffer
  DSPUtils.h/.cpp      FFT windowing, axis-label formatting, fidelity recommendation
  RealFFT.h/.cpp       real-input FFT (half-length complex FFT plus a split step)
  Calibration.h/.cpp   ADC calibration (eFuse + interactive multi-point NVS-backed tool)
  Display.h/.cpp       all drawing: axes, HUD, spectrum/waveform/calibration screens
  SpectrumTask.h/.cpp  the frame pipeline: FFT compute on core 0, drawing on core 1
  SerialConsole.h/.cpp serial command parsing
  VUMeter.h/.cpp       VU LED driver
  Aurora4pt7b.h, Aurora7pt7b.h, Aurora10pt7b.h
                       a custom pixel font family, used for on-screen titles
main.cpp               setup()/loop()
```
