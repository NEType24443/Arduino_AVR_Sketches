#include<Streaming.h>

/// @brief A wrapped union type to access a 32 bit @c float
/// through different angles.
///
/// The implementation constants are taken from IEEE floating
/// point standard. Only tested on Intel processors, though.
struct float32_t{
  static unsigned const signbits = 1;
  static unsigned const expbits = 8;
  static unsigned const mantbits = 23;
  static unsigned const bits = signbits + expbits + mantbits;
  static unsigned const bias = 127;
  static unsigned const expmax = (1u << expbits) - 1u;
  
  /// @brief A structure type to split up a @c float into
  /// its different parts.
  
  struct anatomy
  {
    bool sign : signbits;
    uint16_t exponent : expbits;
    uint32_t mantissa : mantbits;
  };
  
  union
  {
    float f;
    anatomy a;
    uint32_t dw;
    uint16_t w[2];
  };
  
  operator float const &(void) const { return f; }
  operator float &(void) { return f; }
  operator uint32_t const &(void) const { return dw; }
  operator uint32_t &(void) { return dw; }
  float32_t(float const val) { f = val; }
  float32_t(uint32_t const val) { dw = val; }
  
  template <unsigned p>
  uint16_t &
  word(void) { return w[p]; }
  
  template <unsigned p>
  uint16_t
  word(void) const { return w[p]; }
  
};

float32_t _32bitfloat = float32_t(float(23.232324)); 

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
  Serial  << "float value: " << _FLOAT(23.232324,10) << endl
          << "uint32_t value: " << uint32_t(23.232324) << endl
          << "_32bitfloat.f: " << _32bitfloat.f << endl
          << "_32bitfloat.dw: " << _32bitfloat.dw << endl
          << "_32bitfloat.w: " << _32bitfloat.w << endl
          << "_32bitfloat.expbits: " << _32bitfloat.expbits << endl
          << "_32bitfloat.mantbits: " << _32bitfloat.mantbits << endl
          << "_32bitfloat.signbits: " << _32bitfloat.signbits << endl;
}

void loop() {
  digitalWrite(13, millis()%500>0);
}
