/*
 * MP3 player for the Waveshare ESP32-S3-ePaper-1.54G.
 *
 * MP3s are read from the root of the microSD card, decoded with libhelix, and
 * played through the onboard ES8311 codec / NS4150B amp / speaker. The 200x200
 * four-colour e-paper shows a static "now playing" card.
 *
 * Controls:
 *   BOOT (GPIO0)  short press -> next track, long press (>=600ms) -> play/pause
 *   PWR  (GPIO18) press       -> volume 40 -> 60 -> 80 -> 100 -> 40
 *
 * Drivers in es8311.*, DEV_Config.*, EPD_1in54g.*, GUI_Paint.*, font*.cpp are
 * vendored verbatim from Waveshare's own Arduino examples for this board.
 */
#include <Arduino.h>
#include <Wire.h>
#include <SD_MMC.h>
#include <algorithm>
#include <vector>

#include "ESP_I2S.h"
#include "MP3DecoderHelix.h"
#include "esp_sleep.h"
using namespace libhelix;

#include "es8311.h"
#include "EPD_1in54g.h"
#include "GUI_Paint.h"
#include "fonts.h"

// ---------------------------------------------------------------- pin map --
// Taken from Waveshare's 07_Audio_out, 04_SD_Card, 08_E_paper_test and
// 02_I2C_PCF85063/user_config.h examples.
#define PIN_I2C_SDA    47
#define PIN_I2C_SCL    48

#define PIN_I2S_MCLK   14
#define PIN_I2S_BCLK   15
#define PIN_I2S_LRCK   38
#define PIN_I2S_DOUT   45
#define PIN_I2S_DIN    16

#define PIN_PA_CTRL    46   // amplifier enable, active high
#define PIN_AUDIO_PWR  42   // audio power rail, active low
#define PIN_VBAT_PWR   17   // battery power latch, active high

#define PIN_SD_CLK     39
#define PIN_SD_CMD     41
#define PIN_SD_D0      40

#define PIN_BTN_BOOT   0
#define PIN_BTN_PWR    18
#define PIN_LED        3    // active low

// ------------------------------------------------------------------ config --
#define MCLK_MULTIPLE   256     // ES8311 MCLK = sample rate * 256
#define MAX_TRACKS      256     // ponytail: flat cap, revisit if a card ever holds more
#define READ_CHUNK      4096
#define LONG_PRESS_MS   600
#define SHUTDOWN_MS     1500    // PWR held this long -> power off
#define BTN_ARM_MS      4000    // ignore buttons until the power-on press is released
#define PWR_STABLE_MS   1500    // ...and until PWR has been still this long
#define DEBOUNCE_MS     40
#define REDRAW_SETTLE_MS 2000   // let the track selection settle before a 15s refresh

// Steps of 10 rather than 20. PWR only cycles upward and wraps, so each extra
// step is another press to get back around; 8 keeps the trip from 100 back
// down to 30 short enough to live with.
static const int VOLUME_STEPS[] = {30, 40, 50, 60, 70, 80, 90, 100};
#define VOLUME_STEP_COUNT (sizeof(VOLUME_STEPS) / sizeof(VOLUME_STEPS[0]))

// ------------------------------------------------------------------- state --
static I2SClass       i2s;
static es8311_handle_t codec = nullptr;

static std::vector<String> tracks;

static volatile int  trackIdx  = 0;
static volatile bool playing   = true;
static volatile bool skipReq   = false;  // set by buttons, consumed by the audio task
static volatile int  volumeReq = -1;     // pending volume; only the audio task talks I2C
static int           volIdx    = 3;      // -> 60, unchanged default

static uint32_t i2sRate  = 0;            // 0 = I2S not yet started
static uint8_t  i2sChans = 0;

static volatile bool     redrawPending = false;
static volatile uint32_t lastChangeMs  = 0;
static volatile bool     shutdownReq   = false;

static UBYTE *fb = nullptr;              // e-paper framebuffer, 200*200 at 2bpp

// Button idle levels are sampled at boot rather than assumed. GPIO18 doubles as
// the power-button sense line and its resting level is not documented; reading
// it once at startup is cheaper than being wrong about the polarity.
static int  bootIdle = HIGH, pwrIdle = HIGH;
static bool bootDown = false, pwrDown = false;
static uint32_t bootDownAt = 0;

static void onPcm(MP3FrameInfo &info, short *pcm, size_t len, void *ref);
static MP3DecoderHelix mp3(onPcm);

// ------------------------------------------------------------------- audio --

// Reconfigure I2S and the codec for a new stream format. MP3s on one card are
// commonly a mix of 44.1 and 48 kHz, so a hardcoded rate would play part of the
// library at the wrong pitch.
static void applyAudioFormat(uint32_t rate, uint8_t chans) {
  if (i2sRate != 0) i2s.end();

  i2s.setPins(PIN_I2S_BCLK, PIN_I2S_LRCK, PIN_I2S_DOUT, PIN_I2S_DIN, PIN_I2S_MCLK);
  if (!i2s.begin(I2S_MODE_STD, rate, I2S_DATA_BIT_WIDTH_16BIT,
                 I2S_SLOT_MODE_MONO, I2S_STD_SLOT_LEFT)) {
    Serial.printf("[audio] I2S begin failed at %u Hz\n", rate);
    i2sRate = 0;
    return;
  }
  if (codec) es8311_sample_frequency_config(codec, rate * MCLK_MULTIPLE, rate);

  i2sRate  = rate;
  i2sChans = chans;
  Serial.printf("[audio] format %u Hz, %u ch\n", rate, chans);
}

// Decoded PCM arrives here. The ES8311 has a single mono DAC, so stereo is
// downmixed in place -- writing index i while reading 2i is safe because the
// write pointer never overtakes the read pointer.
static void onPcm(MP3FrameInfo &info, short *pcm, size_t len, void *ref) {
  (void)ref;
  if (len == 0) return;

  if ((uint32_t)info.samprate != i2sRate || (uint8_t)info.nChans != i2sChans) {
    applyAudioFormat(info.samprate, info.nChans);
  }
  if (i2sRate == 0) return;

  size_t frames = len;
  if (info.nChans == 2) {
    frames = len / 2;
    for (size_t i = 0; i < frames; i++) {
      pcm[i] = (short)(((int)pcm[2 * i] + (int)pcm[2 * i + 1]) / 2);
    }
  }
  i2s.write((uint8_t *)pcm, frames * sizeof(short));
}

static void markChanged() {
  lastChangeMs  = millis();
  redrawPending = true;
}

static void nextTrack() {
  if (tracks.empty()) return;
  trackIdx = (trackIdx + 1) % (int)tracks.size();
  skipReq  = true;
  markChanged();
}

static void togglePlay() {
  playing = !playing;
  markChanged();
}

static void cycleVolume() {
  volIdx    = (volIdx + 1) % (int)VOLUME_STEP_COUNT;
  volumeReq = VOLUME_STEPS[volIdx];
  markChanged();
}

static void audioTask(void *arg) {
  (void)arg;
  static uint8_t buf[READ_CHUNK];
  File     f;
  int      openIdx = -1;
  uint32_t failures = 0;

  for (;;) {
    // The audio task owns the I2C bus so volume changes from the button task
    // are handed over as a request rather than a second bus master.
    if (volumeReq >= 0) {
      es8311_voice_volume_set(codec, volumeReq, NULL);
      Serial.printf("[audio] volume %d\n", volumeReq);
      volumeReq = -1;
    }

    if (tracks.empty()) { vTaskDelay(pdMS_TO_TICKS(200)); continue; }

    if (skipReq) {
      skipReq = false;
      if (f) f.close();
      openIdx = -1;
    }

    if (!playing) { vTaskDelay(pdMS_TO_TICKS(50)); continue; }

    if (openIdx != trackIdx) {
      if (f) f.close();
      mp3.end();
      mp3.begin();
      f = SD_MMC.open(tracks[trackIdx].c_str());
      if (!f) {
        Serial.printf("[audio] cannot open %s, skipping\n", tracks[trackIdx].c_str());
        if (++failures >= tracks.size()) {   // every file failed: stop spinning
          playing  = false;
          failures = 0;
          markChanged();
          continue;
        }
        nextTrack();
        continue;
      }
      failures = 0;
      openIdx  = trackIdx;
      Serial.printf("[audio] playing %s\n", tracks[trackIdx].c_str());
    }

    int n = f.read(buf, sizeof(buf));
    if (n <= 0) {                 // end of file
      f.close();
      openIdx = -1;
      nextTrack();
      continue;
    }
    mp3.write(buf, (size_t)n);
  }
}

// ----------------------------------------------------------------- display --

// Break a string into fixed-width lines. Filenames have no reliable word
// breaks, so wrapping on spaces alone would leave long names truncated.
static std::vector<String> wrapText(const String &s, size_t cols) {
  std::vector<String> out;
  for (size_t i = 0; i < s.length(); i += cols) {
    out.push_back(s.substring(i, min(i + cols, (size_t)s.length())));
  }
  if (out.empty()) out.push_back("");
  return out;
}

static String displayName(const String &path) {
  int slash = path.lastIndexOf('/');
  String n  = (slash >= 0) ? path.substring(slash + 1) : path;
  if (n.endsWith(".mp3") || n.endsWith(".MP3")) n = n.substring(0, n.length() - 4);
  return n;
}

// Bands are contiguous: each one starts exactly where the previous ends, so no
// panel white shows between them. Every pixel of the card belongs to a band.
#define MARGIN        6
#define TITLE_H       26    // red header, 0..26
#define BLOCK_TOP     27    // black band behind track number + song name
#define BLOCK_BOTTOM  112
#define STATUS_TOP    113   // status band, red playing / black paused
#define STATUS_BOTTOM 143
#define VOL_TOP       144   // black band: level bar + readouts
#define NAME_COLS     17    // Font16 is 11px wide -> 17 columns across 200px
#define NAME_LINES    3     // capped so the black band stays a fixed height

// Waveshare's Paint_DrawString_EN forwards its arguments to Paint_DrawChar as
// (Color_Background, Color_Foreground) -- swapped relative to its own parameter
// names, see GUI_Paint.cpp:590. So its 5th argument paints the character cell
// and its 6th paints the strokes. Every string drawn through it directly comes
// out inverted: coloured letters inside a white box. Wrap it once here so call
// sites can read as plain (foreground, background).
static void drawText(UWORD x, UWORD y, const char *s, sFONT *font, UWORD fg, UWORD bg) {
  Paint_DrawString_EN(x, y, s, font, bg, fg);   // deliberately swapped, see above
}

// Right-align against the panel edge using the font's own width, so the text
// stays put if the font ever changes.
static void drawRight(UWORD y, const char *s, sFONT *font, UWORD fg, UWORD bg) {
  int x = (int)EPD_1IN54G_WIDTH - MARGIN - (int)(strlen(s) * font->Width);
  if (x < 0) x = 0;
  drawText((UWORD)x, y, s, font, fg, bg);
}

static void renderCard(const char *banner) {
  const UWORD W = EPD_1IN54G_WIDTH - 1;
  const UWORD H = EPD_1IN54G_HEIGHT - 1;

  Paint_SelectImage(fb);
  Paint_Clear(EPD_1IN54G_BLACK);   // black, not white: any pixel the bands miss
                                   // stays dark instead of showing panel white

  // header: red band, white text
  Paint_DrawRectangle(0, 0, W, TITLE_H, EPD_1IN54G_RED, DOT_PIXEL_1X1, DRAW_FILL_FULL);
  drawText(MARGIN, 5, "Sea MP3 Player", &Font16,
                      EPD_1IN54G_WHITE, EPD_1IN54G_RED);

  if (banner) {
    Paint_DrawRectangle(0, 86, W, 112, EPD_1IN54G_RED, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    drawText(MARGIN, 91, banner, &Font16, EPD_1IN54G_WHITE, EPD_1IN54G_RED);
    EPD_1IN54G_Display(fb);
    return;
  }

  // track number + song name: white text on a black band
  Paint_DrawRectangle(0, BLOCK_TOP, W, BLOCK_BOTTOM, EPD_1IN54G_BLACK,
                      DOT_PIXEL_1X1, DRAW_FILL_FULL);

  char line[40];
  snprintf(line, sizeof(line), "TRACK %d/%d", trackIdx + 1, (int)tracks.size());
  drawText(MARGIN, 31, line, &Font12, EPD_1IN54G_WHITE, EPD_1IN54G_BLACK);

  std::vector<String> lines = wrapText(displayName(tracks[trackIdx]), NAME_COLS);
  for (size_t i = 0; i < lines.size() && i < NAME_LINES; i++) {
    drawText(MARGIN, 48 + i * 18, lines[i].c_str(), &Font16,
                        EPD_1IN54G_WHITE, EPD_1IN54G_BLACK);
  }

  // status band: red while playing, black when paused, white text either way.
  // The colour carries the state as well as the label does.
  UWORD statusBg = playing ? EPD_1IN54G_RED : EPD_1IN54G_BLACK;
  Paint_DrawRectangle(0, STATUS_TOP, W, STATUS_BOTTOM, statusBg,
                      DOT_PIXEL_1X1, DRAW_FILL_FULL);
  drawText(MARGIN, 120, playing ? "> PLAYING" : "|| PAUSED", &Font16,
                      EPD_1IN54G_WHITE, statusBg);

  // volume band: yellow trough on black, readouts in white beneath it. The
  // trough outline is yellow rather than white so the only white left on the
  // card is the lettering itself.
  Paint_DrawRectangle(0, VOL_TOP, W, H, EPD_1IN54G_BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);

  int vol = VOLUME_STEPS[volIdx];
  Paint_DrawRectangle(MARGIN, 150, 194, 166, EPD_1IN54G_YELLOW,
                      DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
  int w = MARGIN + (188 * vol) / 100;
  if (w > MARGIN + 1) {
    Paint_DrawRectangle(MARGIN + 1, 151, w, 165, EPD_1IN54G_YELLOW,
                        DOT_PIXEL_1X1, DRAW_FILL_FULL);
  }

  snprintf(line, sizeof(line), "VOL %d", vol);
  drawText(MARGIN, 172, line, &Font12, EPD_1IN54G_WHITE, EPD_1IN54G_BLACK);
  drawRight(172, "BOOT=SKIP", &Font12, EPD_1IN54G_WHITE, EPD_1IN54G_BLACK);

  EPD_1IN54G_Display(fb);
}

// Centre text on the panel using the font's own metrics rather than a
// hardcoded x, so the words stay centred if the font changes.
static void drawCentered(UWORD y, const char *s, sFONT *font, UWORD fg, UWORD bg) {
  int x = ((int)EPD_1IN54G_WIDTH - (int)(strlen(s) * font->Width)) / 2;
  if (x < 0) x = 0;
  drawText((UWORD)x, y, s, font, fg, bg);
}

// Filled isoceles triangle: apex at (ax, ay), widening to half-width hw by
// y = by. GUI_Paint has no triangle primitive, so scan it out as rows.
static void fillEar(int ax, int ay, int by, int hw, UWORD c) {
  int h = by - ay;
  if (h <= 0) return;
  for (int y = 0; y <= h; y++) {
    int w = hw * y / h;
    Paint_DrawLine(ax - w, ay + y, ax + w, ay + y, c, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
  }
}

// Cat head, occupying y 0..99 on a red field. The head is yellow so all four
// panel colours appear. The whiskers run off the head onto the field, which is
// why the field is red and not black -- black whiskers would vanish into it.
static void drawCat() {
  fillEar(78,  6, 40, 17, EPD_1IN54G_YELLOW);   // ears first, so the head then
  fillEar(122, 6, 40, 17, EPD_1IN54G_YELLOW);   // merges over their base
  Paint_DrawCircle(100, 62, 33, EPD_1IN54G_YELLOW, DOT_PIXEL_1X1, DRAW_FILL_FULL);
  // Inner ears are white, not red: on a red field a red inner ear disappears
  // and the ears read as hollow outlines instead of solid ones.
  fillEar(78,  16, 34, 9, EPD_1IN54G_WHITE);
  fillEar(122, 16, 34, 9, EPD_1IN54G_WHITE);

  Paint_DrawCircle(86,  58, 5, EPD_1IN54G_BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
  Paint_DrawCircle(114, 58, 5, EPD_1IN54G_BLACK, DOT_PIXEL_1X1, DRAW_FILL_FULL);
  Paint_DrawCircle(88,  56, 2, EPD_1IN54G_WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);
  Paint_DrawCircle(116, 56, 2, EPD_1IN54G_WHITE, DOT_PIXEL_1X1, DRAW_FILL_FULL);

  Paint_DrawCircle(100, 72, 4, EPD_1IN54G_RED, DOT_PIXEL_1X1, DRAW_FILL_FULL);

  // Mouth: the classic cat double-curve. Both halves must finish ABOVE where
  // they start -- two lines simply running down from the nose read as a frown,
  // which is what made the first cat look miserable. GUI_Paint has no arc, so
  // each half is a 3-segment polyline that dips then lifts.
  Paint_DrawLine(100, 76, 100, 80, EPD_1IN54G_BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
  const int smile[2][6] = { { 100, 80,  95, 84,  89, 80 },      // left half
                            { 100, 80, 105, 84, 111, 80 } };    // right half
  for (auto &s : smile) {
    Paint_DrawLine(s[0], s[1], s[2], s[3], EPD_1IN54G_BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawLine(s[2], s[3], s[4], s[5], EPD_1IN54G_BLACK, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
  }

  // cheeks, clear of both the whisker roots (x<=72) and the smile (x>=89)
  Paint_DrawCircle(80,  78, 4, EPD_1IN54G_RED, DOT_PIXEL_1X1, DRAW_FILL_FULL);
  Paint_DrawCircle(120, 78, 4, EPD_1IN54G_RED, DOT_PIXEL_1X1, DRAW_FILL_FULL);

  const int wk[3][4] = { {72, 70, 48, 64}, {72, 75, 46, 75}, {72, 80, 48, 88} };
  for (auto &s : wk) {
    Paint_DrawLine(s[0], s[1], s[2], s[3], EPD_1IN54G_BLACK,
                   DOT_PIXEL_1X1, LINE_STYLE_SOLID);
    Paint_DrawLine(200 - s[0], s[1], 200 - s[2], s[3], EPD_1IN54G_BLACK,
                   DOT_PIXEL_1X1, LINE_STYLE_SOLID);
  }
}

// Farewell screen: cat head over three Font24 (17x24) word bands. "PLAYER" at
// 6 chars is 102px and "MUSIC" 85px, so both clear the 200px panel.
//
// Every word is white, so each band behind one has to be red or black: white
// on yellow is the one unreadable pairing on a four-colour panel. Yellow
// carries the cat instead, which keeps all four colours on screen.
static void renderGoodbye() {
  const UWORD W = EPD_1IN54G_WIDTH - 1;

  Paint_SelectImage(fb);
  Paint_Clear(EPD_1IN54G_RED);
  drawCat();

  struct { UWORD top, bottom, bg; const char *word; } rows[] = {
    { 100, 132, EPD_1IN54G_BLACK, "SEA"    },
    { 133, 165, EPD_1IN54G_RED,   "MUSIC"  },
    { 166, 199, EPD_1IN54G_BLACK, "PLAYER" },
  };

  for (auto &r : rows) {
    Paint_DrawRectangle(0, r.top, W, r.bottom, r.bg, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    drawCentered(r.top + (r.bottom - r.top - Font24.Height) / 2, r.word, &Font24,
                 EPD_1IN54G_WHITE, r.bg);
  }

  EPD_1IN54G_Display(fb);
}

// Shut down and cut the battery rail. Runs on the display task because that
// task owns the bit-banged panel SPI -- doing this from the button task could
// land in the middle of a 15 s refresh and interleave two writers on the bus.
// Never returns.
static void powerOff() {
  Serial.println("[power] shutting down");
  digitalWrite(PIN_PA_CTRL, LOW);       // amp off
  digitalWrite(PIN_AUDIO_PWR, HIGH);    // audio rail off (active low)

  // Leave the farewell screen showing rather than a blank panel -- e-paper
  // holds its last image with no power. Sleeping the panel afterwards is still
  // required; that is what protects it, not the clear.
  EPD_1IN54G_Init();
  if (fb) renderGoodbye();
  else    EPD_1IN54G_Clear(EPD_1IN54G_WHITE);
  EPD_1IN54G_Sleep();
  DEV_Delay_ms(2000);                   // vendor requires >=2 s before cutting the module
  DEV_Module_Exit();

  digitalWrite(PIN_VBAT_PWR, LOW);      // battery latch off -- no effect on USB power
  Serial.println("[power] rail cut; deep sleep (USB keeps the chip alive)");
  Serial.flush();
  esp_deep_sleep_start();
}

static void displayTask(void *arg) {
  (void)arg;
  for (;;) {
    if (shutdownReq) powerOff();        // does not return

    // Debounced: a refresh takes ~15s, so redrawing on every keypress would
    // leave the panel minutes behind the actual state.
    if (redrawPending && (millis() - lastChangeMs) > REDRAW_SETTLE_MS) {
      redrawPending = false;
      EPD_1IN54G_Init_Fast();
      renderCard(tracks.empty() ? "NO TRACKS" : nullptr);
    }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// ----------------------------------------------------------------- buttons --
static void pollButtons() {
  uint32_t now = millis();

  // The board is powered on BY the PWR button, so the button is usually still
  // held while setup() runs. Sampling the idle level there recorded "pressed"
  // as idle and inverted the logic: once released, the line read as a
  // permanent hold, and SHUTDOWN_MS later the shutdown path fired and switched
  // the board straight back off -- the "turns itself off, press again"
  // symptom. Keep re-sampling and ignore input until the press is long gone.
  static bool     armed       = false;
  static int      lastPwr     = -1;
  static uint32_t pwrStableAt = 0;
  if (!armed) {
    int p = digitalRead(PIN_BTN_PWR);
    if (p != lastPwr) { lastPwr = p; pwrStableAt = now; }
    bootIdle = digitalRead(PIN_BTN_BOOT);
    pwrIdle  = p;
    bootDown = false;
    pwrDown  = false;

    // Arm only once the line has been quiet for a while, not merely once a
    // fixed delay has passed: holding PWR for longer than the delay would
    // otherwise capture "pressed" as the idle level and invert the logic all
    // over again. Waiting for stillness works however long the press lasts.
    if (now >= BTN_ARM_MS && (now - pwrStableAt) >= PWR_STABLE_MS) {
      armed = true;
      Serial.printf("[btn] armed; idle BOOT=%d PWR=%d\n", bootIdle, pwrIdle);
    }
    return;
  }

  bool down = (digitalRead(PIN_BTN_BOOT) != bootIdle);
  if (down && !bootDown) bootDownAt = now;
  if (!down && bootDown) {
    uint32_t held = now - bootDownAt;
    if (held >= DEBOUNCE_MS) {
      if (held >= LONG_PRESS_MS) togglePlay();
      else                       nextTrack();
    }
  }
  bootDown = down;

  // PWR: tap steps the volume, hold shuts down. Shutdown fires while the
  // button is still held rather than on release, so the music cutting out is
  // immediate confirmation that the hold registered.
  static uint32_t pwrDownAt  = 0;
  static bool     pwrHandled = false;

  bool p = (digitalRead(PIN_BTN_PWR) != pwrIdle);
  if (p && !pwrDown) { pwrDownAt = now; pwrHandled = false; }
  if (p && !pwrHandled && (now - pwrDownAt) >= SHUTDOWN_MS) {
    pwrHandled = true;
    playing    = false;
    digitalWrite(PIN_PA_CTRL, LOW);   // silence now; the panel and rail follow
    shutdownReq = true;
  }
  if (!p && pwrDown && !pwrHandled && (now - pwrDownAt) >= DEBOUNCE_MS) {
    cycleVolume();
  }
  pwrDown = p;
}

// ------------------------------------------------------------------- setup --
static void selftest(const char *what, bool ok) {
  Serial.printf("SELFTEST %-14s %s\n", what, ok ? "PASS" : "FAIL");
}

static void scanTracks() {
  File root = SD_MMC.open("/");
  if (!root) return;

  for (File e = root.openNextFile(); e && tracks.size() < MAX_TRACKS;
       e = root.openNextFile()) {
    if (!e.isDirectory()) {
      String name = String(e.name());
      String bare = name.substring(name.lastIndexOf('/') + 1);
      String low  = name;
      low.toLowerCase();
      // "._foo.mp3" are macOS AppleDouble stubs -- they end in .mp3 but hold
      // resource-fork metadata, and decode as noise.
      if (low.endsWith(".mp3") && !bare.startsWith("._")) {
        tracks.push_back(name.startsWith("/") ? name : "/" + name);
      }
    }
    e.close();
  }
  root.close();
  std::sort(tracks.begin(), tracks.end(),
            [](const String &a, const String &b) { return strcmp(a.c_str(), b.c_str()) < 0; });
}

static bool codecInit() {
  codec = es8311_create(I2C_NUM_0, ES8311_ADDRRES_0);
  if (!codec) return false;

  const es8311_clock_config_t clk = {
    .mclk_inverted      = false,
    .sclk_inverted      = false,
    .mclk_from_mclk_pin = true,
    .mclk_frequency     = 44100 * MCLK_MULTIPLE,
    .sample_frequency   = 44100,
  };
  if (es8311_init(codec, &clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16) != ESP_OK) return false;
  if (es8311_voice_volume_set(codec, VOLUME_STEPS[volIdx], NULL) != ESP_OK) return false;
  es8311_microphone_config(codec, false);
  return true;
}

void setup() {
  // Latch power on so the board stays alive when running from the battery.
  pinMode(PIN_VBAT_PWR, OUTPUT);
  digitalWrite(PIN_VBAT_PWR, HIGH);
  pinMode(PIN_AUDIO_PWR, OUTPUT);
  digitalWrite(PIN_AUDIO_PWR, LOW);    // active low
  pinMode(PIN_PA_CTRL, OUTPUT);
  digitalWrite(PIN_PA_CTRL, HIGH);     // amplifier enable
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);         // active low, so HIGH = off

  Serial.begin(115200);
  delay(400);
  Serial.println("\n=== Waveshare ePaper 1.54G MP3 player ===");

  pinMode(PIN_BTN_BOOT, INPUT_PULLUP);
  pinMode(PIN_BTN_PWR, INPUT_PULLUP);
  delay(10);
  bootIdle = digitalRead(PIN_BTN_BOOT);
  pwrIdle  = digitalRead(PIN_BTN_PWR);
  Serial.printf("[btn] idle levels BOOT=%d PWR=%d\n", bootIdle, pwrIdle);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  bool codecOk = codecInit();
  selftest("es8311", codecOk);

  SD_MMC.setPins(PIN_SD_CLK, PIN_SD_CMD, PIN_SD_D0);
  bool sdOk = SD_MMC.begin("/sdcard", true);   // true = 1-bit mode
  selftest("sdcard", sdOk);

  if (sdOk) scanTracks();
  Serial.printf("[sd] %d mp3 file(s)\n", (int)tracks.size());
  selftest("tracks", !tracks.empty());

  if (DEV_Module_Init() != 0) {
    Serial.println("[epd] DEV_Module_Init failed");
  }
  pinMode(EPD_BUSY_PIN, INPUT);
  selftest("epd_busy", digitalRead(EPD_BUSY_PIN) == 0 || digitalRead(EPD_BUSY_PIN) == 1);

  UWORD imageSize = ((EPD_1IN54G_WIDTH % 4 == 0) ? (EPD_1IN54G_WIDTH / 4)
                                                 : (EPD_1IN54G_WIDTH / 4 + 1)) * EPD_1IN54G_HEIGHT;
  fb = (UBYTE *)malloc(imageSize);
  selftest("framebuffer", fb != nullptr);
  if (!fb) return;

  EPD_1IN54G_Init();
  Paint_NewImage(fb, EPD_1IN54G_WIDTH, EPD_1IN54G_HEIGHT, 0, EPD_1IN54G_WHITE);
  Paint_SetScale(4);

  // No Clear() before the first card: Clear is itself a full ~20 s refresh
  // that paints a solid colour, and renderCard's Display() already drives
  // every pixel. Running both left the previous shutdown screen on the panel
  // for ~40 s after power-on, which reads as "it turned itself back off".

  const char *banner = nullptr;
  if (!sdOk)               banner = "NO SD CARD";
  else if (tracks.empty()) banner = "NO TRACKS";
  renderCard(banner);

  if (!codecOk || tracks.empty()) {
    Serial.println("[main] nothing to play, idling");
    playing = false;
  }

  // Audio on core 1, e-paper on core 0: a redraw bit-bangs ~10 KB over software
  // SPI, and doing that on the audio core would starve the I2S buffer and drop
  // sound on every track change.
  xTaskCreatePinnedToCore(audioTask,   "audio",   8192, NULL, 5, NULL, 1);
  xTaskCreatePinnedToCore(displayTask, "display", 8192, NULL, 2, NULL, 0);
}

void loop() {
  pollButtons();
  delay(20);
}
