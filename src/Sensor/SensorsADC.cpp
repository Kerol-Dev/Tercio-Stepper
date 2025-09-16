#include "SensorsADC.h"
#include <math.h>

bool SensorsADC::begin(float vref, uint8_t adcBits) {
  _vref = vref;
  _adcMax = (1u << adcBits) - 1u;
  analogReadResolution(adcBits);
  _vbus_V = 0.0f;
  _temp_C = 25.0f;
  return true;
}

void SensorsADC::setThermistor(const ThermistorCfg& c) { _th = c; }
void SensorsADC::setVbusDivider(const VbusCfg& c)      { _vb = c; }
void SensorsADC::setLpfAlpha(float a) {
  if (a < 0.0f) a = 0.0f;
  if (a > 1.0f) a = 1.0f;
  _alpha = a;
}
void SensorsADC::setVref(float vref) { _vref = vref; }

void SensorsADC::update() {
  // 1) Raw
  _rawPB0 = analogRead(PB0); // thermistor node
  _rawPB1 = analogRead(PB1); // VBUS divider node

  // 2) Node voltages
  const float Vtemp = (float)_rawPB0 * (_vref / (float)_adcMax);
  const float Vnode = (float)_rawPB1 * (_vref / (float)_adcMax);

  // 3) VBUS back-calc from divider
  const float vbus = Vnode * ((_vb.Rtop + _vb.Rbot) / _vb.Rbot);

  // 4) Thermistor resistance from divider
  float Rntc;
  if (_th.pullup) {
    const float denom = (_vref - Vtemp);
    Rntc = (denom > 1e-6f) ? (_th.Rfixed * Vtemp / denom) : 1e9f;
  } else {
    const float denom = (Vtemp > 1e-6f) ? Vtemp : 1e-6f;
    Rntc = _th.Rfixed * (_vref - Vtemp) / denom;
  }

  // 5) Beta equation → °C
  const float T0_K = _th.T0_C + 273.15f;
  const float lnRR = logf(Rntc / _th.R0);
  const float invT = (1.0f / T0_K) + (lnRR / _th.Beta);
  const float T_K  = 1.0f / invT;
  const float tC   = T_K - 273.15f;

  // 6) LPF
  _vbus_V += _alpha * (vbus - _vbus_V);
  _temp_C += _alpha * (tC   - _temp_C);
}

float SensorsADC::vbus_V() const { return _vbus_V; }
float SensorsADC::temperatureC() const { return _temp_C; }

uint32_t SensorsADC::rawPB0() const { return _rawPB0; }
uint32_t SensorsADC::rawPB1() const { return _rawPB1; }

float SensorsADC::nodeVoltagePB1() const {
  return (float)_rawPB1 * (_vref / (float)_adcMax);
}