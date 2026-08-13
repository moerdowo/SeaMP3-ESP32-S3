# E-Paper MP3 Player — Design

**Date:** 2026-08-13
**Target:** Waveshare ESP32-S3-ePaper-1.54G (ESP32-S3-PICO-1-N8R8, 8 MB flash / 8 MB PSRAM)

## Goal

Standalone MP3 player. MP3s live on the microSD card, play through the onboard
speaker, controlled by the two onboard buttons, with a static "now playing" card
on the e-paper display.

## Hardware constraints that shaped the design

Two facts drove every decision below.

**The panel is slow.** The 1.54" four-colour (G) e-paper takes 20 s for a full
refresh and 15 s in fast mode, with no partial-refresh mode. A live UI is
impossible. The display is a static card, redrawn only on track change.

**There are only two free buttons.** Every other GPIO is committed to the codec,
SD card, or panel. BOOT (GPIO0) and PWR (GPIO18) are the entire input surface.

### Pin map (from Waveshare's own examples)

| Function | GPIO |
|---|---|
| I2C (ES8311, PCF85063, SHTC3) | SDA 47, SCL 48 |
| I2S | MCLK 14, BCK 15, LRCK 38, DOUT 45, DIN 16 |
| Amplifier enable | PA_CTRL 46, PA_EN 42 |
| microSD (SDMMC, 1-bit) | CLK 39, CMD 41, D0 40 |
| e-Paper SPI | SCK 12, MOSI 13, CS 11, DC 10, RST 9, BUSY 8, PWR 6 |
| Buttons | BOOT 0 (active low), PWR 18 |
| Power latch | GPIO17 (hold high to stay on when on battery) |
| User LED | GPIO3 (active low) |

## Architecture

Single Arduino sketch. Drivers are vendored verbatim from Waveshare's examples
rather than rewritten — they are known-good on this exact board.

| File | Origin |
|---|---|
| `WaveshareMP3.ino` | new — all application logic |
| `es8311.cpp/.h`, `es8311_reg.h` | Waveshare `07_Audio_out`, verbatim |
| `DEV_Config`, `EPD_1in54g`, `GUI_Paint`, `font12/16/24` | Waveshare `08_E_paper_test`, verbatim |

**Toolchain:** `arduino-cli` 1.5.1 with esp32 core 3.2.0. Chosen over ESP-IDF
because Waveshare's Arduino examples supply the drivers directly, and the install
is ~1 GB instead of ~2.5 GB. Installed as a direct binary to `~/bin` rather than
via Homebrew, which could not reach `ghcr.io` at setup time.

**MP3 decoding:** `arduino-libhelix` — the Helix fixed-point decoder, standalone.

ESP8266Audio was rejected deliberately. Its I2S output layer targets the legacy
`driver/i2s.h` API; on Arduino core 3.x that is a compile-compatibility gamble,
and a failure there breaks the whole library since Arduino compiles every `.cpp`
in it. libhelix has no platform coupling at all — it hands back raw PCM in a
callback, which we write to core 3.x's `ESP_I2S`, the same API Waveshare's audio
example uses and therefore known-good on this board.

## Data flow

```
/sdcard/*.mp3 -> fread 4 KB -> MP3DecoderHelix -> PCM callback
              -> downmix L+R to mono -> I2SClass.write() -> ES8311 -> NS4150B -> speaker
```

The ES8311 drives a single mono DAC, so stereo frames are downmixed by averaging
the two channels. Sample rate is read from the decoded frame header; on the first
frame and on any subsequent change, both the I2S peripheral and the codec are
reconfigured. Files on one card are commonly a mix of 44.1 and 48 kHz, so a
hardcoded rate would play a portion of the library at the wrong pitch.

## Concurrency

- **Core 1:** audio task — read, decode, write to I2S.
- **Core 0:** `loop()` — buttons and e-paper redraw.

The split is required, not cosmetic. A 20 s panel refresh on the audio core would
starve the I2S buffer and produce an audible dropout on every track change.
E-paper refresh is almost entirely blocking on the BUSY pin, so it costs
negligible CPU on the core it does occupy.

## Controls

| Input | Action |
|---|---|
| BOOT short press (< 600 ms) | next track |
| BOOT long press (>= 600 ms) | play / pause |
| PWR press | volume cycle 40 -> 60 -> 80 -> 100 |

## Display behaviour

Static card: track index and count, filename, volume, play/pause state.

Redraw is debounced — it fires only after the track has been stable for 2 s, and
never while a refresh is already in flight. Without the debounce, five quick
skips would queue 100 s of refreshes and the panel would trail reality by over a
minute.

## Error handling

| Condition | Behaviour |
|---|---|
| No SD card | "NO SD CARD" on panel, no crash |
| Zero MP3s found | "NO TRACKS" on panel |
| Undecodable file | log over serial, skip to next track |

## Verification

Boot self-test writes one `SELFTEST` line per check over serial: SD mounted,
track count > 0, ES8311 ACKs on I2C, e-paper BUSY pin readable. Each line is
`PASS` or `FAIL`, so a bad flash is diagnosable without guesswork.

## Out of scope

ID3 tag parsing (filenames are the display text), playlists, shuffle, recursive
folder scan (card root only), FLAC/WAV, sleep modes. Track list caps at 256 files.
