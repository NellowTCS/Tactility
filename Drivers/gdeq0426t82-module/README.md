# GDEQ0426T82 Display Driver

A kernel driver for the GoodDisplay `GDEQ0426T82`, a 4.26" 800x480 SPI e-paper panel
(SSD1677 controller) used by the Xteink X4.

## Features

- **Flashless greyscale** via the panel's built-in default B/W LUT: LVGL renders
  `GRAYSCALE8` (L8) and the driver hard-quantizes each pixel to 4 levels, spatially
  dithering them with a Bayer 4x4 matrix onto a 1bpp shadow. Every refresh after the
  boot baseline is a differential partial update (`0x22 0xFC`) that drives only the
  pixels that changed - no full-screen flashing, unlike the old 4-grey `0xC7` path
- **Coalesced refresh**: `draw_bitmap()` only writes the shadow and re-arms an 80 ms
  FreeRTOS one-shot timer. The timer pushes the accumulated dirty window to the panel,
  then waits for BUSY *outside* the panel mutex, so LVGL never blocks on the panel's
  ~500 ms update time
- **Differential partial windows** driven from two full-frame 1bpp shadows (2 x 48 KB).
  This panel's SSD1677 compares the RED (`0x26`) plane against B/W (`0x24`) to decide
  which pixels to drive, so the driver keeps the previously-displayed frame in a second
  shadow and writes it to RED before each partial. No read-modify-write is needed -
  which is just as well, since this panel has no MISO
- **MINIMAL_BUFFER capability**: the LVGL bridge gives this display an 8-row draw
  buffer instead of `vres/10`, cutting LVGL-side RAM dramatically
- **BUSY active-high** polling with a 10 s timeout, matching this panel's GPIO polarity

## RAM Usage

- **1bpp shadow framebuffer** (bit 1 = white, matches panel polarity): 48 KB
- **1bpp old-frame shadow** (previously-displayed frame for the RED differential): 48 KB
- **Band scratch buffer** (one 16-row band): 1.6 KB
- **LVGL draw buffer** (800x8 L8): 6.4 KB (plus 6.4 KB rotate buffer on the bridge side)
- **Total driver allocation**: ~98 KB

The shadow is required, not optional: LVGL's partial draws are arbitrarily x-aligned, and
the panel RAM is byte-oriented, so boundary bytes must be merged with previously-displayed
pixels. Without a shadow that would force a read-back, which the (MISO-less) panel can't do.

## Refresh Timing

- Boot baseline full update (`0xF7`): ~1.8 s, one flash (the only flashing refresh)
- Differential partial update (`0xFC`): ~510 ms, flashless; the dirty window is written to
  B/W (new frame) and RED (previous frame) before triggering
- Power off: ~200 ms
- Draws during a refresh only touch the shadow; the next coalesce tick ships the new dirty
  window. A continuously-redrawing screen therefore settles at roughly 2 partials/second

## Implementation Notes

### Rendering Path

1. LVGL `GRAYSCALE8` partial-mode flush per 800x8 tile; `color_data` is the tile's top-left
   pixel, row-major, stride = tile width (LVGL reshapes the buffer to the area in PARTIAL mode)
2. `level = pixel >> 6` (0=black .. 3=white); `black = Bayer4x4(x,y) < threshold[level]` with
   thresholds `{16, 5, 10, 0}`; bit 7 = leftmost pixel; bit 1 = white
3. Pixels are merged into the 1bpp shadow and the dirty bounding box is expanded; the
   coalesce timer is re-armed. No SPI occurs in this call
4. Timer tick: byte-align the dirty window (`x &= ~7`), write it to B/W (`0x24`) from the
   current shadow and to RED (`0x26`) from the old-frame shadow, copy the current window
   into the old-frame shadow (still under the mutex), then refresh:
   `0x21(0x40,0x00)/0x3C(0xC0)/0x22(0xF7)/0x20` for the first baseline, else
   `0x21(0x00,0x00)/0x3C(0xC0)/0x22(0xFC)/0x20`
5. BUSY is waited on with the mutex released, so the next LVGL draw isn't stalled

### Greyscale vs. True 4-Grey

The old driver used the two-plane LUT mode (`lut_4G`, `0xC7`) for true 4-grey but had to
flash the whole panel on every frame. This driver trades true grey for a Bayer-dithered B/W
look that refreshes without any visible flashing. It uses the SSD1677's default OTP LUT;
the RED (`0x26`) plane is used as the differential's old frame, never for grey.

### Power Management

- `disp_on_off(false)` / `stop`: explicit power off (`0x22 0x83`) then deep sleep
  (`0x10 0x01`)
- `disp_on_off(true)`: reloads the registers. The hardware reset inside init discards the
  panel RAM, so the wake forces `first_refresh = true` and the next update is a full
  baseline (no diff against post-reset garbage). The full flash only happens on boot and
  after waking; normal operation stays flashless
- `reset`: hardware reset, forcing a full baseline refresh on the next draw

### Controller Commands

- `0x3C 0x80` — border follows the LUT at init; `0x3C 0xC0` is re-sent before every
  refresh (the partial/DU waveform does not engage without it)
- `0x24` / `0x26` — B/W and RED RAM plane writes. A differential partial writes the new
  frame to `0x24` and the previously-displayed frame to `0x26`; the panel's LUT compares
  the two and drives only the differing pixels (`0x21 0x00` normal mode)
- `0x44`/`0x45`/`0x4E`/`0x4F` — RAM window, Y-reversed (the panel's gates are wired
  bottom-to-top)
- `0x22 0xFC` — differential partial trigger, the stock X4 value from the hardware-verified
  FreeInk `Ssd1677Driver`. GxEPD2's `0x1C` fast value does NOT select the partial waveform
  on this panel (it runs the full OTP waveform every refresh) and is not used
- `0x21`/`0x22`/`0x20` — update control and master activation

Ported from [ZinggJM/GxEPD2](https://github.com/ZinggJM/GxEPD2)'s
`GxEPD2_426_GDEQ0426T82` B/W default-LUT mode; the differential partial sequence is taken
from FreeInk's [Ssd1677Driver](https://github.com/cypress2007/FreeInk/tree/main/libs/display/FreeInkDisplay/src/driver).

License: [Apache v2.0](LICENSE-Apache-2.0.md)
