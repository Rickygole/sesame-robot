// display.h -- SSD1306 face rendering with a CHUNKED, non-blocking flush.
//
// The whole reason this class exists rather than calling
// Adafruit_SSD1306::display() directly:
//
//   display() pushes all 1024 framebuffer bytes over I2C in one blocking
//   call. At 400kHz that is ~23ms. The motion tick is 20ms. A single
//   frame would blow the entire budget and every servo update would
//   jitter behind the screen.
//
// So the framebuffer is pushed ONE PAGE PER TICK -- 8 rows, 128 bytes,
// ~2.9ms, about 15% of the tick. A full frame lands in 8 ticks (160ms,
// ~6fps), which is ample for a face and costs the motion loop nothing it
// cannot afford.
//
// Faces are drawn PROCEDURALLY with GFX primitives rather than stored as
// bitmaps. Upstream ships 297KB of frame data; we get nine expressions
// out of a few hundred bytes of drawing code, they scale cleanly, and
// there is no asset pipeline to maintain. The trade-off is that they are
// simpler-looking than hand-drawn art -- a deliberate choice, and easy to
// revisit by swapping drawFace() for a bitmap lookup later.
#pragma once

#include <stdint.h>

namespace sesame {

// Must match enum class FaceId in src/core/command.h.
enum class Face : uint8_t {
  Neutral = 0,
  Happy = 1,
  Sad = 2,
  Angry = 3,
  Sleep = 4,
  Listening = 5,
  Thinking = 6,
  Confused = 7,
  Talking = 8,
  Count = 9,
};

class Display {
 public:
  // Initializes I2C and the panel. Returns false if the SSD1306 did not
  // acknowledge -- usually SDA/SCL swapped or the wrong I2C address.
  bool begin();

  // Requests a face. Cheap: only marks the framebuffer dirty. The actual
  // redraw happens on the next flushChunk().
  void setFace(Face face);
  Face face() const { return face_; }

  // Pushes ONE page (~2.9ms). Call once per motion tick. Returns true
  // when a full frame has just completed.
  bool flushChunk();

  // Blinks and idle animation advance here. Takes dt rather than reading
  // millis() so the behaviour is deterministic and testable.
  void tick(float dt);

  // A short status line under the face -- IP address, battery, errors.
  // Kept separate from the face so a transient message never disturbs the
  // expression.
  void setStatus(const char* text);

  bool ready() const { return ready_; }

 private:
  void redraw();

  bool ready_ = false;
  Face face_ = Face::Neutral;
  uint8_t page_ = 0;        // next page to push
  bool dirty_ = true;
  bool blinking_ = false;
  float blinkTimer_ = 0.f;
  float nextBlinkIn_ = 3.f;
  float animPhase_ = 0.f;
  char status_[24] = {0};
};

}  // namespace sesame
