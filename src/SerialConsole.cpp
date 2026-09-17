#include "SerialConsole.h"
#include "Globals.h"
#include "DSPUtils.h"
#include "Settings.h"

static char cmdBuf[96];
static uint8_t cmdLen = 0;

void print_controls()
{
  Serial.println();
  Serial.println(F("=== Controls (Buttons) ==="));
  Serial.println(F("  PAUSE: toggle Play/Pause"));
  Serial.println(F("  AGG- / AGG+:   aggregation finer/coarser"));
  Serial.println(F("  N- / N+:       FFT length down/up"));
  Serial.println(F("  ZOOM- / ZOOM+: Horizontal zoom (Fmax -/+ 5%)"));
  Serial.println();
  Serial.println(F("=== Serial Commands (with examples) ==="));
  Serial.println(F("  help                # show this help"));
  Serial.println(F("  stats               # print current settings"));
  Serial.println(F("  fs=2000..40000      # set sample rate, e.g. fs=15000"));
  Serial.println(F("  n=32|64|128|256|512|1024|2048   # e.g. n=512"));
  Serial.println(F("  fps=10..120         # e.g. fps=60"));
  Serial.println(F("  agg=1..64           # e.g. agg=4 (coarser plot)"));
  Serial.println(F("  overlap=1..8         # capture window overlap divisor, e.g. overlap=4 (75% overlap)"));
  Serial.println(F("  xscale=lin|log      # set X axis scale, e.g. xscale=lin"));
  Serial.println(F("  fmax=Hz             # set max plotted freq (horizontal zoom), e.g. fmax=3500"));
  Serial.println(F("  fmax=nyq            # make Fmax follow Nyquist (Fs/2) again"));
  Serial.println(F("  ymax=<dB>           # set top of Y axis in dBFS, e.g. ymax=-10"));
  Serial.println(F("  ymin=<dB>           # set bottom of Y axis in dBFS, e.g. ymin=-80"));
  Serial.println(F("  hann=0|1            # Hann window off/on, e.g. hann=1"));
  Serial.println(F("  pause               # toggle pause"));
  Serial.println();
}

void print_stats()
{
  uint16_t N = N_CHOICES[gNidx];
  int bpp = bins_per_point(N);
  float df_eff = (float)gFs / N * bpp;
  Serial.printf("Fs=%lu  N=%u  Df=%.2fHz  FPS=%u  BPP=%d  Overlap=1/%u  X=%s  Fmax=%.0fHz%s  Hann=%d\r\n",
                (unsigned long)gFs, N, df_eff, gFPS, bpp, gOverlap, (gXScale == XS_LIN ? "LIN" : "LOG"),
                gFmaxHz, gFmaxFollowNyq ? " (nyq)" : "", (int)gUseHann);
  Serial.printf("Y range: [%.1f, %.1f] dBFS\r\n", gYMin_dB, gYMax_dB);
}

void print_prompt() { Serial.print("> "); }

void apply_command(const char *s)
{
  if (!s || !*s)
    return;
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
  else if (!strncmp(s, "overlap=", 8))
  {
    setOverlap((uint8_t)atoi(s + 8));
  }
  else if (!strncmp(s, "xscale=", 7))
  {
    if (!strcasecmp(s + 7, "lin"))
      setXScale(0);
    else if (!strcasecmp(s + 7, "log"))
      setXScale(1);
  }
  else if (!strncmp(s, "fmax=", 5))
  {
    if (!strcasecmp(s + 5, "nyq"))
      setFmax_followNyq();
    else
      setFmax(atof(s + 5));
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
    gUseHann = (atoi(s + 5) != 0);
    gAxesDirty = true;
    gHUDDirty = true;
  }
  else if (!strcmp(s, "pause"))
  {
    gPaused = !gPaused;
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
