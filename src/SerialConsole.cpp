#include "SerialConsole.h"
#include "Globals.h"
#include "DSPUtils.h"
#include "Settings.h"
#include "Calibration.h"

static char cmdBuf[96];
static uint8_t cmdLen = 0;

// Set by "cal clear" - the next line typed is treated as its y/n answer
// instead of a normal command, so erasing a saved calibration always needs
// an explicit confirmation.
static bool s_pendingCalClearConfirm = false;

void print_controls()
{
  Serial.println();
  Serial.println(F("=== Controls (Buttons) ==="));
  Serial.println(F("  PAUSE:         switch between FFT spectrum and waveform display"));
  Serial.println(F("  AGG- / AGG+:   FFT: aggregation finer/coarser | Waveform: Y (amplitude) zoom"));
  Serial.println(F("  N- / N+:       FFT: length down/up | Waveform: no effect"));
  Serial.println(F("  ZOOM- / ZOOM+: FFT: horizontal zoom (Fmax -/+5%) | Waveform: X (time) zoom via Fs (-/+5%)"));
  Serial.println();
  Serial.println(F("=== Serial Commands (with examples) ==="));
  Serial.println(F("  help                # show this help"));
  Serial.println(F("  stats               # print current settings"));
  Serial.println(F("  fs=2000..200000     # set sample rate, e.g. fs=15000"));
  Serial.println(F("  n=32|64|128|256|512|1024|2048|4096   # e.g. n=512"));
  Serial.println(F("  fps=10..120         # e.g. fps=60"));
  Serial.println(F("  agg=1..64           # e.g. agg=4 (coarser plot)"));
  Serial.println(F("  xscale=lin|log      # set X axis scale, e.g. xscale=lin"));
  Serial.println(F("  yscale=db|lin       # set Y axis scale (dBFS or linear %FS), e.g. yscale=lin"));
  Serial.println(F("  fmax=Hz             # set max plotted freq (horizontal zoom), e.g. fmax=3500"));
  Serial.println(F("  fmax=nyq            # make Fmax follow Nyquist (Fs/2) again"));
  Serial.println(F("                      #   (both print a suggested fs=/n= for best fit to the plot width)"));
  Serial.println(F("  ymax=<dB>           # set top of Y axis in dBFS, e.g. ymax=-10"));
  Serial.println(F("  ymin=<dB>           # set bottom of Y axis in dBFS, e.g. ymin=-80"));
  Serial.println(F("  hann=0|1            # Hann window off/on, e.g. hann=1"));
  Serial.println(F("  mode=fft|wave       # switch display mode, e.g. mode=wave"));
  Serial.println(F("  pause               # toggle pause"));
  Serial.println(F("  cal                 # enter interactive ADC calibration (see on-screen/button prompts)"));
  Serial.println(F("  cal cancel          # exit calibration mode without saving"));
  Serial.println(F("  cal clear           # erase the saved user calibration (revert to eFuse/naive) - asks y/n first"));
  Serial.println();
}

void print_stats()
{
  if (gDisplayMode == MODE_FFT)
  {
    uint16_t N = N_CHOICES[gNidx];
    int bpp = bins_per_point(N);
    float df_eff = (float)gFs / N * bpp;
    int hop = clampi((int)(gFs / gFPS), 1, N);
    float overlapPct = 100.0f * (1.0f - (float)hop / (float)N);
    Serial.printf("Mode=FFT  Fs=%lu  N=%u  Df=%.2fHz  FPS(target)=%u  BPP=%d  Overlap=%.0f%%  X=%s  Y=%s  Fmax=%.0fHz%s  Hann=%d\r\n",
                  (unsigned long)gFs, N, df_eff, gFPS, bpp, overlapPct, (gXScale == XS_LIN ? "LIN" : "LOG"),
                  (gYScale == YS_DB ? "dB" : "LIN"), gFmaxHz, gFmaxFollowNyq ? " (nyq)" : "", (int)gUseHann);
    Serial.printf("Y range: [%.1f, %.1f] dBFS\r\n", gYMin_dB, gYMax_dB);
    Serial.printf("FPS(actual)=%.1f  Frame=%.2fms  FFT=%.2fms\r\n",
                  (double)gMeasuredFPS, (double)gLastFrameUs / 1000.0, (double)gLastFFTus / 1000.0);
  }
  else // MODE_WAVEFORM
  {
    float spanMs = (float)PLOT_W / (float)gFs * 1000.0f;
    Serial.printf("Mode=Waveform  Fs=%lu  Span=%.2fms (%d samples)  FPS(target)=%u  Y range: +/-%.0f counts\r\n",
                  (unsigned long)gFs, (double)spanMs, PLOT_W, gFPS, (double)gWaveYRange);
    Serial.printf("FPS(actual)=%.1f  Frame=%.2fms\r\n",
                  (double)gMeasuredFPS, (double)gLastFrameUs / 1000.0);
  }
  const char *calSrc = gCalSource == CAL_USER ? "user" : gCalSource == CAL_EFUSE ? "efuse" : "none";
  Serial.printf("ADC cal: source=%s  volts=raw*%.6f+%.6f\r\n", calSrc, (double)gCalGain, (double)gCalOffset);
}

// Suggests the fs=/n= combo whose visible bin count (0..fmaxHz) best fills
// the plot's PLOT_W pixels - printed whenever Fmax changes via a serial
// command, since that's the setting that determines what "best" even means.
static void print_fidelity_tip(float fmaxHz)
{
  FsNRecommendation rec = recommend_fs_n(fmaxHz);
  Serial.printf("  tip: for the best match to this display's %d plot pixels at Fmax=%.0fHz, "
                "try fs=%lu n=%u (~%d bins across the plot)\r\n",
                PLOT_W, (double)fmaxHz, (unsigned long)rec.fs, rec.N, rec.kvis);
}

void print_prompt() { Serial.print("> "); }

void apply_command(const char *s)
{
  if (!s || !*s)
    return;

  if (s_pendingCalClearConfirm)
  {
    s_pendingCalClearConfirm = false;
    if (!strcasecmp(s, "y") || !strcasecmp(s, "yes"))
      calibration_clear_saved();
    else
      Serial.println(F("Cancelled - calibration not cleared."));
    return;
  }
  if (!strcmp(s, "help"))
  {
    print_controls();
  }
  else if (!strcmp(s, "stats"))
  {
    print_stats();
  }
  else if (!strncmp(s, "fs=", 3))
  {
    setFs((uint32_t)atoi(s + 3));
  }
  else if (!strncmp(s, "n=", 2))
  {
    uint16_t want = atoi(s + 2);
    for (uint8_t i = 0; i < sizeof(N_CHOICES) / sizeof(N_CHOICES[0]); i++)
      if (N_CHOICES[i] == want)
      {
        setNidx(i);
        break;
      }
  }
  else if (!strncmp(s, "fps=", 4))
  {
    setFPS(atoi(s + 4));
  }
  else if (!strncmp(s, "agg=", 4))
  {
    setAgg((uint8_t)atoi(s + 4));
  }
  else if (!strncmp(s, "xscale=", 7))
  {
    if (!strcasecmp(s + 7, "lin"))
      setXScale(0);
    else if (!strcasecmp(s + 7, "log"))
      setXScale(1);
  }
  else if (!strncmp(s, "yscale=", 7))
  {
    if (!strcasecmp(s + 7, "db"))
      setYScale(0);
    else if (!strcasecmp(s + 7, "lin"))
      setYScale(1);
  }
  else if (!strncmp(s, "fmax=", 5))
  {
    if (!strcasecmp(s + 5, "nyq"))
      setFmax_followNyq();
    else
      setFmax(atof(s + 5));
    print_fidelity_tip(gFmaxHz); // gFmaxHz: the actual post-clamp value
  }
  else if (!strncmp(s, "ymax=", 5))
  {
    setYMax((float)atof(s + 5));
  }
  else if (!strncmp(s, "ymin=", 5))
  {
    setYMin((float)atof(s + 5));
  }
  else if (!strncmp(s, "hann=", 5))
  {
    setHann(atoi(s + 5) != 0);
  }
  else if (!strncmp(s, "mode=", 5))
  {
    if (!strcasecmp(s + 5, "fft") && gDisplayMode != MODE_FFT)
      toggleDisplayMode();
    else if (!strcasecmp(s + 5, "wave") && gDisplayMode != MODE_WAVEFORM)
      toggleDisplayMode();
  }
  else if (!strcmp(s, "pause"))
  {
    gPaused = !gPaused;
  }
  else if (!strcmp(s, "cal"))
  {
    calibration_enter();
  }
  else if (!strcmp(s, "cal cancel"))
  {
    calibration_cancel();
  }
  else if (!strcmp(s, "cal clear"))
  {
    Serial.println(F("Really clear the saved calibration? (y/n)"));
    s_pendingCalClearConfirm = true;
  }
  else
  {
    Serial.println(F("Unknown. Type 'help'."));
  }
}

void service_serial()
{
  while (Serial.available())
  {
    int c = Serial.read();
    if (c == '\r' || c == '\n')
    {
      cmdBuf[cmdLen] = 0;
      Serial.print("\r\n");
      apply_command(cmdBuf);
      cmdLen = 0;
      print_stats();
      print_prompt();
    }
    else if (c == 8 || c == 127)
    { // backspace
      if (cmdLen > 0)
      {
        cmdLen--;
        Serial.print("\b \b");
      }
    }
    else if (c >= 32 && c < 127)
    {
      if (cmdLen < sizeof(cmdBuf) - 1)
      {
        cmdBuf[cmdLen++] = (char)c;
        Serial.write(c);
      }
    }
  }
}
