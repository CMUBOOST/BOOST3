#ifndef COLOR
#define COLOR

#include <cstdint>

namespace hebi {

typedef struct Color {
  uint8_t r_;
  uint8_t g_;
  uint8_t b_;

  Color(uint8_t r, uint8_t g, uint8_t b) : r_(r), g_(g), b_(b) {}

  uint8_t getRed() const { return r_; }
  uint8_t getGreen() const { return g_; }
  uint8_t getBlue() const { return b_; }
} Color;

} // namespace hebi

#endif // COLOR
