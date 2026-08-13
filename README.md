# Waveshare ESP32-S3 ePaper 1.54G — MP3 Player

Standalone MP3 player for the [Waveshare ESP32-S3-ePaper-1.54G](https://github.com/waveshareteam/ESP32-S3-ePaper-1.54G)
board. Plays MP3s from the microSD card through the onboard ES8311 codec and
speaker, with a static "now playing" card on the 200x200 four-colour e-paper.

## Controls

| Input | Action |
|---|---|
| BOOT (GPIO0) short press | next track |
| BOOT long press (>= 600 ms) | play / pause |
| PWR (GPIO18) short press | volume 40 -> 60 -> 80 -> 100 |
| PWR long press (>= 1.5 s) | power off |

Power-off silences the amp immediately, then clears and sleeps the panel
before dropping the battery latch on GPIO17. **On USB power the rail cannot
actually be cut** — USB feeds it past the latch — so the board enters deep
sleep instead and stays quiet until you unplug or reset it.

## SD card

Put `.mp3` files in the **root** of the card. Subfolders are not scanned.
Files play in filename order, capped at 256 tracks.

## Build and flash

Requires [`arduino-cli`](https://arduino.github.io/arduino-cli/) plus the
`arduino-libhelix` decoder, which is not in the Arduino library index:

```bash
arduino-cli core install esp32:esp32@3.2.0
```

```bash
git clone --depth 1 https://github.com/pschatzmann/arduino-libhelix.git "$(arduino-cli config get directories.user)/libraries/arduino-libhelix"
```

Compile:

```bash
arduino-cli compile --fqbn "esp32:esp32:esp32s3:PSRAM=opi,FlashSize=8M,PartitionScheme=default_8MB,CDCOnBoot=cdc,USBMode=hwcdc,FlashMode=qio" WaveshareMP3
```

Flash (adjust the port):

```bash
arduino-cli upload -p /dev/cu.usbmodem83101 --fqbn "esp32:esp32:esp32s3:PSRAM=opi,FlashSize=8M,PartitionScheme=default_8MB,CDCOnBoot=cdc,USBMode=hwcdc,FlashMode=qio" WaveshareMP3
```

## Self-test

On every boot the firmware prints one line per subsystem at 115200 baud:

```
SELFTEST es8311         PASS
SELFTEST sdcard         PASS
SELFTEST tracks         PASS
SELFTEST epd_busy       PASS
SELFTEST framebuffer    PASS
```

A `FAIL` line names the subsystem directly, so a bad flash is diagnosable
without guesswork.

## Notes

The e-paper takes **15-20 s per refresh** and has no partial-refresh mode, so the
display is a static card, not a live UI. Redraws are debounced by 2 s and run on
core 0 while audio decodes on core 1 — otherwise the bit-banged SPI transfer
would starve the I2S buffer and drop sound on every track change.

Drivers under `WaveshareMP3/` (`es8311.*`, `DEV_Config.*`, `EPD_1in54g.*`,
`GUI_Paint.*`, `font*.cpp`) are vendored verbatim from Waveshare's own Arduino
examples for this board and keep their original licences.

Design rationale, including the full pin map and the libraries rejected along the
way, is in [the design spec](docs/superpowers/specs/2026-08-13-epaper-mp3-player-design.md).
