#pragma once

#include <Arduino.h>

// Prints the button/command help text.
void print_controls();
// Prints the current settings as a single status line.
void print_stats();
// Prints the "> " input prompt.
void print_prompt();
// Parses and applies one typed command line (e.g. "fs=15000").
void apply_command(const char *s);
// Reads available serial bytes, echoes them, and dispatches completed lines.
void service_serial();
