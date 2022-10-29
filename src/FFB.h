#ifndef FFB_H

#include <Arduino.h>

#include "FFB/PIDReportHandler.h"
#include "FFB/PIDReportType.h"

#define FORCE_FEEDBACK_MAXGAIN 100

struct Gains {
  uint8_t totalGain = FORCE_FEEDBACK_MAXGAIN;
  uint8_t constantGain = FORCE_FEEDBACK_MAXGAIN;
  uint8_t rampGain = FORCE_FEEDBACK_MAXGAIN;
  uint8_t squareGain = FORCE_FEEDBACK_MAXGAIN;
  uint8_t sineGain = FORCE_FEEDBACK_MAXGAIN;
  uint8_t triangleGain = FORCE_FEEDBACK_MAXGAIN;
  uint8_t sawtoothdownGain = FORCE_FEEDBACK_MAXGAIN;
  uint8_t sawtoothupGain = FORCE_FEEDBACK_MAXGAIN;
  uint8_t springGain = FORCE_FEEDBACK_MAXGAIN;
  uint8_t damperGain = FORCE_FEEDBACK_MAXGAIN;
  uint8_t inertiaGain = FORCE_FEEDBACK_MAXGAIN;
  uint8_t frictionGain = FORCE_FEEDBACK_MAXGAIN;
  uint8_t customGain = FORCE_FEEDBACK_MAXGAIN;
};

struct EffectParams {
  int32_t springMaxPosition = 0;
  int32_t springPosition = 0;

  int32_t damperMaxVelocity = 0;
  int32_t damperVelocity = 0;

  int32_t inertiaMaxAcceleration = 0;
  int32_t inertiaAcceleration = 0;

  int32_t frictionMaxPositionChange = 0;
  int32_t frictionPositionChange = 0;
};

class FFB : public EffectModule {
 private:
  // force feedback gain
  Gains* m_gains;

  // force feedback effect params
  EffectParams* m_effect_params;

  /// force calculate funtion
  float NormalizeRange(int32_t x, int32_t maxValue);
  int32_t ApplyEnvelope(volatile TEffectState& effect, int32_t value);
  int32_t ApplyGain(int16_t value, uint8_t gain);
  int32_t ConstantForceCalculator(volatile TEffectState& effect);
  int32_t RampForceCalculator(volatile TEffectState& effect);
  int32_t SquareForceCalculator(volatile TEffectState& effect);
  int32_t SinForceCalculator(volatile TEffectState& effect);
  int32_t TriangleForceCalculator(volatile TEffectState& effect);
  int32_t SawtoothDownForceCalculator(volatile TEffectState& effect);
  int32_t SawtoothUpForceCalculator(volatile TEffectState& effect);
  int32_t ConditionForceCalculator(volatile TEffectState& effect, float metric,
                                   uint8_t axis);
  void forceCalculator(int32_t* forces);
  int32_t getEffectForce(volatile TEffectState& effect, Gains _gains,
                         EffectParams _effect_params, uint8_t axis);

  // set gain functions
  int8_t setGains(Gains* _gains);

  // set effect params funtions
  int8_t setEffectParams(EffectParams* _effect_params);

 public:
  // force feedback Interfaces
  virtual void ProcessData(uint8_t* data, uint16_t len);

  void getForce(int32_t* forces);
};
#endif