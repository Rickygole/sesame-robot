#include "display.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <string.h>

#include "../../config.h"

namespace sesame {

namespace {

Adafruit_SSD1306 g_oled(kOledWidth, kOledHeight, &Wire, -1);

constexpr uint8_t kPageCount = kOledHeight / 8;  // 8 pages of 128 bytes
constexpr int16_t kEyeY = 24;
constexpr int16_t kEyeLX = 40;
constexpr int16_t kEyeRX = 88;
constexpr int16_t kEyeR = 10;

void drawEyeOpen(int16_t x, int16_t y, int16_t r) {
  g_oled.fillCircle(x, y, r, SSD1306_WHITE);
  // A small dark catchlight stops the eye reading as a flat blob.
  g_oled.fillCircle(x + r / 3, y - r / 3, r / 4, SSD1306_BLACK);
}

void drawEyeClosed(int16_t x, int16_t y, int16_t r) {
  g_oled.drawFastHLine(x - r, y, r * 2, SSD1306_WHITE);
  g_oled.drawFastHLine(x - r, y + 1, r * 2, SSD1306_WHITE);
}

void drawEyeSquint(int16_t x, int16_t y, int16_t r) {
  g_oled.fillCircle(x, y, r, SSD1306_WHITE);
  g_oled.fillRect(x - r - 1, y - r - 1, r * 2 + 2, r, SSD1306_BLACK);
}

// Smile (curve > 0), frown (curve < 0), flat line (curve == 0).
void drawMouth(int16_t cx, int16_t cy, int16_t halfWidth, int16_t curve) {
  if (curve == 0) {
    g_oled.drawFastHLine(cx - halfWidth, cy, halfWidth * 2, SSD1306_WHITE);
    return;
  }
  // Parabola sampled across the width; cheap and reads well at this size.
  for (int16_t dx = -halfWidth; dx <= halfWidth; ++dx) {
    const float t = float(dx) / float(halfWidth);
    const int16_t dy = int16_t(float(curve) * (1.f - t * t));
    g_oled.drawPixel(cx + dx, cy - dy, SSD1306_WHITE);
    g_oled.drawPixel(cx + dx, cy - dy + 1, SSD1306_WHITE);
  }
}

}  // namespace

bool Display::begin() {
  Wire.begin(kI2cSda, kI2cScl);
  if (!g_oled.begin(SSD1306_SWITCHCAPVCC, kOledAddr)) {
    ready_ = false;
    return false;
  }
  g_oled.clearDisplay();
  g_oled.display();
  ready_ = true;
  dirty_ = true;
  page_ = 0;
  return true;
}

void Display::setFace(Face face) {
  if (face != face_) {
    face_ = face;
    dirty_ = true;
    page_ = 0;
  }
}

void Display::setStatus(const char* text) {
  if (text == nullptr) {
    status_[0] = '\0';
  } else {
    strncpy(status_, text, sizeof(status_) - 1);
    status_[sizeof(status_) - 1] = '\0';
  }
  dirty_ = true;
}

void Display::tick(float dt) {
  animPhase_ += dt;
  if (animPhase_ > 1000.f) animPhase_ -= 1000.f;

  // Idle blink. A face that never blinks reads as switched-off rather
  // than alive.
  if (face_ == Face::Sleep) {
    return;
  }
  blinkTimer_ += dt;
  if (!blinking_ && blinkTimer_ >= nextBlinkIn_) {
    blinking_ = true;
    blinkTimer_ = 0.f;
    dirty_ = true;
  } else if (blinking_ && blinkTimer_ >= 0.12f) {
    blinking_ = false;
    blinkTimer_ = 0.f;
    // Vary the interval so the blink does not read as a metronome.
    // Derived from the animation phase rather than random(), so it stays
    // deterministic.
    nextBlinkIn_ = 2.5f + 2.f * (animPhase_ - float(int(animPhase_)));
    dirty_ = true;
  }

  if (face_ == Face::Thinking || face_ == Face::Talking ||
      face_ == Face::Listening) {
    dirty_ = true;  // these animate continuously
  }
}

void Display::redraw() {
  g_oled.clearDisplay();

  const bool closed = blinking_ || face_ == Face::Sleep;

  switch (face_) {
    case Face::Happy:
      if (closed) {
        drawEyeClosed(kEyeLX, kEyeY, kEyeR);
        drawEyeClosed(kEyeRX, kEyeY, kEyeR);
      } else {
        drawEyeSquint(kEyeLX, kEyeY, kEyeR);
        drawEyeSquint(kEyeRX, kEyeY, kEyeR);
      }
      drawMouth(64, 46, 18, 8);
      break;

    case Face::Sad:
      drawEyeOpen(kEyeLX, kEyeY + 2, kEyeR - 2);
      drawEyeOpen(kEyeRX, kEyeY + 2, kEyeR - 2);
      drawMouth(64, 52, 16, -7);
      break;

    case Face::Angry:
      drawEyeOpen(kEyeLX, kEyeY + 2, kEyeR - 2);
      drawEyeOpen(kEyeRX, kEyeY + 2, kEyeR - 2);
      // Angled brows.
      g_oled.drawLine(kEyeLX - 12, kEyeY - 12, kEyeLX + 10, kEyeY - 6,
                      SSD1306_WHITE);
      g_oled.drawLine(kEyeRX + 12, kEyeY - 12, kEyeRX - 10, kEyeY - 6,
                      SSD1306_WHITE);
      drawMouth(64, 50, 14, -5);
      break;

    case Face::Sleep:
      drawEyeClosed(kEyeLX, kEyeY, kEyeR);
      drawEyeClosed(kEyeRX, kEyeY, kEyeR);
      g_oled.setTextSize(1);
      g_oled.setTextColor(SSD1306_WHITE);
      g_oled.setCursor(96, 12);
      g_oled.print(F("z"));
      g_oled.setCursor(104, 6);
      g_oled.print(F("z"));
      break;

    case Face::Listening: {
      drawEyeOpen(kEyeLX, kEyeY, kEyeR);
      drawEyeOpen(kEyeRX, kEyeY, kEyeR);
      // Expanding rings: "I am hearing you."
      const int16_t r = 4 + int16_t(6.f * (animPhase_ - float(int(animPhase_))));
      g_oled.drawCircle(64, 48, r, SSD1306_WHITE);
      break;
    }

    case Face::Thinking: {
      drawEyeOpen(kEyeLX, kEyeY, kEyeR);
      drawEyeSquint(kEyeRX, kEyeY, kEyeR);
      // Travelling dots.
      const int step = int(animPhase_ * 3.f) % 3;
      for (int i = 0; i < 3; ++i) {
        const int16_t x = 52 + int16_t(i * 12);
        g_oled.fillCircle(x, 50, (i == step) ? 3 : 1, SSD1306_WHITE);
      }
      break;
    }

    case Face::Confused:
      drawEyeOpen(kEyeLX, kEyeY, kEyeR);
      drawEyeOpen(kEyeRX, kEyeY - 3, kEyeR - 3);  // uneven = puzzled
      // Wavy mouth.
      for (int16_t dx = -16; dx <= 16; dx += 1) {
        const int16_t dy = (((dx + 16) / 4) % 2) ? 2 : -2;
        g_oled.drawPixel(64 + dx, 48 + dy, SSD1306_WHITE);
      }
      g_oled.setTextSize(1);
      g_oled.setTextColor(SSD1306_WHITE);
      g_oled.setCursor(100, 10);
      g_oled.print(F("?"));
      break;

    case Face::Talking: {
      drawEyeOpen(kEyeLX, kEyeY, kEyeR);
      drawEyeOpen(kEyeRX, kEyeY, kEyeR);
      // Mouth opens and closes so speech reads as speech.
      const float t = animPhase_ - float(int(animPhase_));
      const int16_t h = 3 + int16_t(7.f * (t < 0.5f ? t * 2.f : (1.f - t) * 2.f));
      g_oled.fillRoundRect(50, 46 - h / 2, 28, h, 3, SSD1306_WHITE);
      break;
    }

    case Face::Neutral:
    default:
      if (closed) {
        drawEyeClosed(kEyeLX, kEyeY, kEyeR);
        drawEyeClosed(kEyeRX, kEyeY, kEyeR);
      } else {
        drawEyeOpen(kEyeLX, kEyeY, kEyeR);
        drawEyeOpen(kEyeRX, kEyeY, kEyeR);
      }
      drawMouth(64, 48, 14, 0);
      break;
  }

  if (status_[0] != '\0') {
    g_oled.setTextSize(1);
    g_oled.setTextColor(SSD1306_WHITE);
    g_oled.setCursor(0, 56);
    g_oled.print(status_);
  }
}

bool Display::flushChunk() {
  if (!ready_) {
    return false;
  }
  if (page_ == 0) {
    if (!dirty_) {
      return false;  // nothing changed; skip the I2C traffic entirely
    }
    redraw();
    dirty_ = false;
  }

  // Push exactly one page: set the column/page window, then 128 bytes.
  // ~2.9ms, comfortably inside the 20ms motion tick.
  const uint8_t* buf = g_oled.getBuffer() + (size_t(page_) * kOledWidth);
  Wire.beginTransmission(kOledAddr);
  Wire.write(0x00);                       // command stream
  Wire.write(0xB0 | page_);               // page start
  Wire.write(0x00);                       // lower column = 0
  Wire.write(0x10);                       // upper column = 0
  Wire.endTransmission();

  // I2C buffer is 128 bytes on ESP32 including the control byte, so send
  // the page in two halves rather than assuming one transmission fits.
  for (uint8_t half = 0; half < 2; ++half) {
    Wire.beginTransmission(kOledAddr);
    Wire.write(0x40);                     // data stream
    for (uint8_t i = 0; i < kOledWidth / 2; ++i) {
      Wire.write(buf[size_t(half) * (kOledWidth / 2) + i]);
    }
    Wire.endTransmission();
  }

  page_ = uint8_t((page_ + 1) % kPageCount);
  return page_ == 0;  // frame complete
}

}  // namespace sesame
