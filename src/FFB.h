#ifndef FFB_H

#include <Arduino.h>

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

  // effect blocks
  volatile TEffectState effectStates[MAX_EFFECTS + 1];

  // device control
  volatile uint8_t devicePaused;

  volatile uint8_t actuatorEnabled;

  volatile uint8_t deviceGain;

  // effect control
  void SetEffect(USB_FFBReport_SetEffect_Output_Data_t* data);
  void StartEffect(uint8_t id);
  void StopEffect(uint8_t id);
  void StopAllEffects(void);
  void FreeEffect(uint8_t id);
  void FreeAllEffects(void);
  void EffectOperation(USB_FFBReport_EffectOperation_Output_Data_t* data);
  void BlockFree(USB_FFBReport_BlockFree_Output_Data_t* data);
  void DeviceControl(USB_FFBReport_DeviceControl_Output_Data_t* data);
  void DeviceGain(USB_FFBReport_DeviceGain_Output_Data_t* data);

  // set effect parameters
  void SetEnvelope(USB_FFBReport_SetEnvelope_Output_Data_t* data);
  void SetCondition(USB_FFBReport_SetCondition_Output_Data_t* data);
  void SetPeriodic(USB_FFBReport_SetPeriodic_Output_Data_t* data);
  void SetConstantForce(USB_FFBReport_SetConstantForce_Output_Data_t* data);
  void SetRampForce(USB_FFBReport_SetRampForce_Output_Data_t* data);
  void SetCustomForce(USB_FFBReport_SetCustomForce_Output_Data_t* data);
  void SetCustomForceData(USB_FFBReport_SetCustomForceData_Output_Data_t* data);
  void SetDownloadForceSample(
      USB_FFBReport_SetDownloadForceSample_Output_Data_t* data);

  /// effect simulatiopn
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
  int32_t getEffectForce(volatile TEffectState& effect, Gains _gains,
                         EffectParams _effect_params, uint8_t axis);

 public:
  // force feedback Interfaces
  virtual void ProcessData(uint8_t* data, uint16_t len);

  // set gain functions
  void setGains(Gains* _gains) { m_gains = _gains; };

  // set effect params funtions
  void setEffectParams(EffectParams* _effect_params) {
    m_effect_params = _effect_params;
  };

  // get the total effect force
  void getForce(int32_t* forces);
};
#endif