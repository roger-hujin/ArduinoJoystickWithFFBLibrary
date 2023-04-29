#include "FFB.h"

#include "FFB/Utils.h"

void dumpEffect(volatile TEffectState& effect) {
  Serial.print(" state=");
  Serial.print(effect.state);
  Serial.print(" type = ");
  Serial.print(effect.effectType);
  Serial.print(" elapsedTime=");
  Serial.print(effect.elapsedTime);
  Serial.print(" duration=");
  Serial.println((unsigned int)effect.duration);
}

void FFB::ProcessData(uint8_t* data, uint16_t len) {
  LogUsbData(data, len);

  switch (data[0])  // reportID
  {
    case 1:
      SetEffect((USB_FFBReport_SetEffect_Output_Data_t*)data);
      break;
    case 2:
      SetEnvelope((USB_FFBReport_SetEnvelope_Output_Data_t*)data);
      break;
    case 3:
      SetCondition((USB_FFBReport_SetCondition_Output_Data_t*)data);
      break;
    case 4:
      SetPeriodic((USB_FFBReport_SetPeriodic_Output_Data_t*)data);
      break;
    case 5:
      SetConstantForce((USB_FFBReport_SetConstantForce_Output_Data_t*)data);
      break;
    case 6:
      SetRampForce((USB_FFBReport_SetRampForce_Output_Data_t*)data);
      break;
    case 7:
      SetCustomForceData((USB_FFBReport_SetCustomForceData_Output_Data_t*)data);
      break;
    case 8:
      SetDownloadForceSample(
          (USB_FFBReport_SetDownloadForceSample_Output_Data_t*)data);
      break;
    case 9:
      break;
    case 10:
      EffectOperation((USB_FFBReport_EffectOperation_Output_Data_t*)data);
      break;
    case 11:
      BlockFree((USB_FFBReport_BlockFree_Output_Data_t*)data);
      break;
    case 12:
      DeviceControl((USB_FFBReport_DeviceControl_Output_Data_t*)data);
      break;
    case 13:
      DeviceGain((USB_FFBReport_DeviceGain_Output_Data_t*)data);
      break;
    case 14:
      SetCustomForce((USB_FFBReport_SetCustomForce_Output_Data_t*)data);
      break;
    default:
      break;
  }
}

/************************************************************************/
/*                Effect Management                                     */
/************************************************************************/

void FFB::SetEffect(USB_FFBReport_SetEffect_Output_Data_t* data) {
  volatile TEffectState* effect = &effectStates[data->effectBlockIndex];
  effect->state |= MEFFECTSTATE_ALLOCATED;
  effect->duration = data->duration;
  effect->directionX = data->directionX;
  effect->directionY = data->directionY;
  effect->effectType = data->effectType;
  effect->gain = data->gain;
  effect->enableAxis = data->enableAxis;

  Serial.print("Set Effect ");
  Serial.print(data->effectBlockIndex);
  Serial.print(" data->duiration=");
  Serial.print(data->duration);
  dumpEffect(effectStates[data->effectBlockIndex]);
}

void FFB::StartEffect(uint8_t id) {
  if (id > MAX_EFFECTS) return;
  effectStates[id].state |= MEFFECTSTATE_PLAYING;
  effectStates[id].elapsedTime = 0;
  effectStates[id].startTime = (uint64_t)millis();
  Serial.print("Start Effect ");
  Serial.print(id);
  dumpEffect(effectStates[id]);
}

void FFB::StopEffect(uint8_t id) {
  if (id > MAX_EFFECTS) return;
  effectStates[id].state &= ~MEFFECTSTATE_PLAYING;
}

void FFB::StopAllEffects(void) {
  for (uint8_t id = 0; id <= MAX_EFFECTS; id++) {
    StopEffect(id);
  }
}

void FFB::FreeEffect(uint8_t id) {
  if (id > MAX_EFFECTS) return;
  effectStates[id].state = MEFFECTSTATE_FREE;
}

void FFB::FreeAllEffects(void) {
  memset((void*)&effectStates, 0, sizeof(effectStates));
}

void FFB::EffectOperation(USB_FFBReport_EffectOperation_Output_Data_t* data) {
  if (data->operation == 1) {  // Start
    if (data->loopCount > 0)
      effectStates[data->effectBlockIndex].duration *= data->loopCount;
    if (data->loopCount == 0xFF)
      effectStates[data->effectBlockIndex].duration = USB_DURATION_INFINITE;
    StartEffect(data->effectBlockIndex);
  } else if (data->operation == 2) {  // StartSolo
    // Stop all first
    StopAllEffects();
    // Then start the given effect
    StartEffect(data->effectBlockIndex);
  } else if (data->operation == 3) {  // Stop
    StopEffect(data->effectBlockIndex);
  }
}

void FFB::BlockFree(USB_FFBReport_BlockFree_Output_Data_t* data) {
  uint8_t eid = data->effectBlockIndex;

  if (eid == 0xFF) {  // all effects
    FreeAllEffects();
  } else {
    FreeEffect(eid);
  }
}

void FFB::DeviceControl(USB_FFBReport_DeviceControl_Output_Data_t* data) {
  uint8_t control = data->control;

  if (control == 0x01) {  // 1=Enable Actuators
    actuatorEnabled = 1;
  } else if (control == 0x02) {  // 2=Disable Actuators
    actuatorEnabled = 0;
  } else if (control == 0x03) {  // 3=Stop All Effects
    StopAllEffects();
  } else if (control == 0x04) {  //  4=Reset
    FreeAllEffects();
  } else if (control == 0x05) {  // 5=Pause
    devicePaused = 1;
  } else if (control == 0x06) {  // 6=Continue
    devicePaused = 0;
  } else if (control & (0xFF - 0x3F)) {
  }
}

void FFB::DeviceGain(USB_FFBReport_DeviceGain_Output_Data_t* data) {
  deviceGain = data->gain;
}

void FFB::SetEnvelope(USB_FFBReport_SetEnvelope_Output_Data_t* data) {
  volatile TEffectState* effect = &effectStates[data->effectBlockIndex];
  effect->attackLevel = data->attackLevel;
  effect->fadeLevel = data->fadeLevel;
  effect->attackTime = data->attackTime;
  effect->fadeTime = data->fadeTime;
}

void FFB::SetCondition(USB_FFBReport_SetCondition_Output_Data_t* data) {
  volatile TEffectState* effect = &effectStates[data->effectBlockIndex];
  uint8_t axis = data->parameterBlockOffset;
  effect->conditions[axis].cpOffset = data->cpOffset;
  effect->conditions[axis].positiveCoefficient = data->positiveCoefficient;
  effect->conditions[axis].negativeCoefficient = data->negativeCoefficient;
  effect->conditions[axis].positiveSaturation = data->positiveSaturation;
  effect->conditions[axis].negativeSaturation = data->negativeSaturation;
  effect->conditions[axis].deadBand = data->deadBand;
  effect->conditionBlocksCount++;
}

void FFB::SetPeriodic(USB_FFBReport_SetPeriodic_Output_Data_t* data) {
  volatile TEffectState* effect = &effectStates[data->effectBlockIndex];
  effect->magnitude = data->magnitude;
  effect->offset = data->offset;
  effect->phase = data->phase;
  effect->period = data->period;
}

void FFB::SetConstantForce(USB_FFBReport_SetConstantForce_Output_Data_t* data) {
  volatile TEffectState* effect = &effectStates[data->effectBlockIndex];
  effect->magnitude = data->magnitude;
}

void FFB::SetRampForce(USB_FFBReport_SetRampForce_Output_Data_t* data) {
  volatile TEffectState* effect = &effectStates[data->effectBlockIndex];
  effect->startMagnitude = data->startMagnitude;
  effect->endMagnitude = data->endMagnitude;
}

// NOT IMPLEMENTED
void FFB::SetCustomForce(USB_FFBReport_SetCustomForce_Output_Data_t* data) {}

void FFB::SetCustomForceData(
    USB_FFBReport_SetCustomForceData_Output_Data_t* data) {}

void FFB::SetDownloadForceSample(
    USB_FFBReport_SetDownloadForceSample_Output_Data_t* data) {}

/************************************************************************/
/*                Effect Simulation                                     */
/************************************************************************/

void FFB::getForce(int32_t* forces) {
  forces[0] = 0;
  forces[1] = 0;

  // go through each effect block
  for (int id = 0; id < MAX_EFFECTS; id++) {
    volatile TEffectState& effect = effectStates[id];
    // if (effect.state != 0) {
    //   Serial.print(id);
    //   Serial.print(" pause=");
    //   Serial.print(devicePaused);
    //   dumpEffect(effect);
    // }
    bool effectPlaying = ((effect.state & MEFFECTSTATE_PLAYING) != 0) &&
                         ((effect.elapsedTime <= effect.duration) ||
                          (effect.duration == USB_DURATION_INFINITE));
    if (!devicePaused && effectPlaying) {
      // Serial.print(id);
      // dumpEffect(effect);
      forces[0] += getEffectForce(effect, m_gains[0], m_effect_params[0], 0);
      forces[1] += getEffectForce(effect, m_gains[1], m_effect_params[1], 1);
    }
  }
  forces[0] = (int32_t)((float)1.0 * forces[0] * m_gains[0].totalGain /
                        10000);  // each effect gain * total effect gain = 10000
  forces[1] = (int32_t)((float)1.0 * forces[1] * m_gains[1].totalGain /
                        10000);  // each effect gain * total effect gain = 10000
  forces[0] = map(forces[0], -10000, 10000, -255, 255);
  forces[1] = map(forces[1], -10000, 10000, -255, 255);
}

int32_t FFB::getEffectForce(volatile TEffectState& effect, Gains _gains,
                            EffectParams _effect_params, uint8_t axis) {
  uint8_t direction;
  uint8_t condition;
  bool useForceDirectionForConditionEffect =
      (effect.enableAxis == DIRECTION_ENABLE &&
       effect.conditionBlocksCount == 1);

  if (effect.enableAxis == DIRECTION_ENABLE) {
    direction = effect.directionX;
    if (effect.conditionBlocksCount > 1) {
      condition = axis;
    } else {
      condition = 0;  // only one Condition Parameter Block is defined
    }
  } else {
    direction = axis == 0 ? effect.directionX : effect.directionY;
    condition = axis;
  }

  float angle = (direction * 360.0 / 255.0) * DEG_TO_RAD;
  float angle_ratio = axis == 0 ? sin(angle) : -1 * cos(angle);
  int32_t force = 0;
  switch (effect.effectType) {
    case USB_EFFECT_CONSTANT:  // 1
      force =
          ConstantForceCalculator(effect) * _gains.constantGain * angle_ratio;
      break;
    case USB_EFFECT_RAMP:  // 2
      force = RampForceCalculator(effect) * _gains.rampGain * angle_ratio;
      break;
    case USB_EFFECT_SQUARE:  // 3
      force = SquareForceCalculator(effect) * _gains.squareGain * angle_ratio;
      break;
    case USB_EFFECT_SINE:  // 4
      force = SinForceCalculator(effect) * _gains.sineGain * angle_ratio;
      break;
    case USB_EFFECT_TRIANGLE:  // 5
      force =
          TriangleForceCalculator(effect) * _gains.triangleGain * angle_ratio;
      break;
    case USB_EFFECT_SAWTOOTHDOWN:  // 6
      force = SawtoothDownForceCalculator(effect) * _gains.sawtoothdownGain *
              angle_ratio;
      break;
    case USB_EFFECT_SAWTOOTHUP:  // 7
      force = SawtoothUpForceCalculator(effect) * _gains.sawtoothupGain *
              angle_ratio;
      break;
    case USB_EFFECT_SPRING:  // 8
      force = ConditionForceCalculator(
                  effect,
                  NormalizeRange(_effect_params.springPosition,
                                 _effect_params.springMaxPosition),
                  condition) *
              _gains.springGain;
      if (useForceDirectionForConditionEffect) {
        force *= angle_ratio;
      }
      break;
    case USB_EFFECT_DAMPER:  // 9
      force = ConditionForceCalculator(
                  effect,
                  NormalizeRange(_effect_params.damperVelocity,
                                 _effect_params.damperMaxVelocity),
                  condition) *
              _gains.damperGain;
      if (useForceDirectionForConditionEffect) {
        force *= angle_ratio;
      }
      break;
    case USB_EFFECT_INERTIA:  // 10
      if (_effect_params.inertiaAcceleration < 0 &&
          _effect_params.frictionPositionChange < 0) {
        force = ConditionForceCalculator(
                    effect,
                    abs(NormalizeRange(_effect_params.inertiaAcceleration,
                                       _effect_params.inertiaMaxAcceleration)),
                    condition) *
                _gains.inertiaGain;
      } else if (_effect_params.inertiaAcceleration < 0 &&
                 _effect_params.frictionPositionChange > 0) {
        force = -1 *
                ConditionForceCalculator(
                    effect,
                    abs(NormalizeRange(_effect_params.inertiaAcceleration,
                                       _effect_params.inertiaMaxAcceleration)),
                    condition) *
                _gains.inertiaGain;
      }
      if (useForceDirectionForConditionEffect) {
        force *= angle_ratio;
      }
      break;
    case USB_EFFECT_FRICTION:  // 11
      force = ConditionForceCalculator(
                  effect,
                  NormalizeRange(_effect_params.frictionPositionChange,
                                 _effect_params.frictionMaxPositionChange),
                  condition) *
              _gains.frictionGain;
      if (useForceDirectionForConditionEffect) {
        force *= angle_ratio;
      }
      break;
    case USB_EFFECT_CUSTOM:  // 12
      break;
  }
  effect.elapsedTime = (uint64_t)millis() - effect.startTime;
  return force;
}

int32_t FFB::ConstantForceCalculator(volatile TEffectState& effect) {
  return ApplyEnvelope(effect, (int32_t)effect.magnitude);
}

int32_t FFB::RampForceCalculator(volatile TEffectState& effect) {
  int32_t tempforce =
      (int32_t)(effect.startMagnitude +
                effect.elapsedTime * 1.0 *
                    (effect.endMagnitude - effect.startMagnitude) /
                    effect.duration);
  return ApplyEnvelope(effect, tempforce);
}

int32_t FFB::SquareForceCalculator(volatile TEffectState& effect) {
  int16_t offset = effect.offset * 2;
  int16_t magnitude = effect.magnitude;
  uint16_t phase = effect.phase;
  uint16_t elapsedTime = effect.elapsedTime;
  uint16_t period = effect.period;

  int32_t maxMagnitude = offset + magnitude;
  int32_t minMagnitude = offset - magnitude;
  uint32_t phasetime = (phase * period) / 255;
  uint32_t timeTemp = elapsedTime + phasetime;
  int32_t tempforce = 0;
  if (period > 0) {
    uint32_t reminder = timeTemp % period;
    if (reminder > (period / 2))
      tempforce = minMagnitude;
    else
      tempforce = maxMagnitude;
  }
  return ApplyEnvelope(effect, tempforce);
}

int32_t FFB::SinForceCalculator(volatile TEffectState& effect) {
  int16_t offset = effect.offset * 2;
  int16_t magnitude = effect.magnitude;
  uint16_t phase = effect.phase;
  uint16_t timeTemp = effect.elapsedTime;
  uint16_t period = effect.period;
  float angle = 0.0;
  if (period != 0)
    angle = ((timeTemp * 1.0 / period) * 2 * PI + (phase / 36000.0));
  float sine = sin(angle);
  int32_t tempforce = (int32_t)(sine * magnitude);
  tempforce += offset;
  return ApplyEnvelope(effect, tempforce);
}

int32_t FFB::TriangleForceCalculator(volatile TEffectState& effect) {
  int16_t offset = effect.offset * 2;
  int16_t magnitude = effect.magnitude;
  uint16_t elapsedTime = effect.elapsedTime;
  uint16_t phase = effect.phase;
  uint16_t period = effect.period;
  uint16_t periodF = effect.period;

  int16_t maxMagnitude = offset + magnitude;
  int16_t minMagnitude = offset - magnitude;
  int32_t phasetime = (phase * period) / 255;
  uint32_t timeTemp = elapsedTime + phasetime;
  int32_t reminder = timeTemp % period;
  int32_t slope = ((maxMagnitude - minMagnitude) * 2) / periodF;
  int32_t tempforce = 0;
  if (reminder > (periodF / 2))
    tempforce = slope * (periodF - reminder);
  else
    tempforce = slope * reminder;
  tempforce += minMagnitude;
  return ApplyEnvelope(effect, tempforce);
}

int32_t FFB::SawtoothDownForceCalculator(volatile TEffectState& effect) {
  int16_t offset = effect.offset * 2;
  int16_t magnitude = effect.magnitude;
  uint16_t elapsedTime = effect.elapsedTime;
  uint16_t phase = effect.phase;
  uint16_t period = effect.period;
  uint16_t periodF = effect.period;

  int16_t maxMagnitude = offset + magnitude;
  int16_t minMagnitude = offset - magnitude;
  int32_t phasetime = (phase * period) / 255;
  uint32_t timeTemp = elapsedTime + phasetime;
  int32_t reminder = timeTemp % period;
  int32_t slope = (maxMagnitude - minMagnitude) / periodF;
  int32_t tempforce = 0;
  tempforce = slope * (period - reminder);
  tempforce += minMagnitude;
  return ApplyEnvelope(effect, tempforce);
}

int32_t FFB::SawtoothUpForceCalculator(volatile TEffectState& effect) {
  int16_t offset = effect.offset * 2;
  int16_t magnitude = effect.magnitude;
  uint16_t elapsedTime = effect.elapsedTime;
  uint16_t phase = effect.phase;
  uint16_t period = effect.period;
  uint16_t periodF = effect.period;

  int16_t maxMagnitude = offset + magnitude;
  int16_t minMagnitude = offset - magnitude;
  int32_t phasetime = (phase * period) / 255;
  uint32_t timeTemp = elapsedTime + phasetime;
  int32_t reminder = timeTemp % period;
  int32_t slope = (maxMagnitude - minMagnitude) / periodF;
  int32_t tempforce = 0;
  tempforce = slope * reminder;
  tempforce += minMagnitude;
  return ApplyEnvelope(effect, tempforce);
}

int32_t FFB::ConditionForceCalculator(volatile TEffectState& effect,
                                      float metric, uint8_t axis) {
  float deadBand;
  float cpOffset;
  float positiveCoefficient;
  float negativeCoefficient;
  float positiveSaturation;
  float negativeSaturation;

  deadBand = effect.conditions[axis].deadBand;
  cpOffset = effect.conditions[axis].cpOffset;
  negativeCoefficient = effect.conditions[axis].negativeCoefficient;
  negativeSaturation = effect.conditions[axis].negativeSaturation;
  positiveSaturation = effect.conditions[axis].positiveSaturation;
  positiveCoefficient = effect.conditions[axis].positiveCoefficient;

  float tempForce = 0;
  if (metric < (cpOffset - deadBand)) {
    tempForce = (metric - 1.00f * (cpOffset - deadBand)) *
                (negativeCoefficient / 10000.0f);
    tempForce =
        (tempForce < -negativeSaturation ? -negativeSaturation : tempForce);
  } else if (metric > (cpOffset + deadBand)) {
    tempForce = (metric - 1.0f * (cpOffset + deadBand)) *
                (positiveCoefficient / 10000.0f);
    tempForce =
        (tempForce > positiveSaturation ? positiveSaturation : tempForce);
  } else
    return 0;
  tempForce = -tempForce * effect.gain / 255.0f;
  switch (effect.effectType) {
    case USB_EFFECT_DAMPER:
      // tempForce = damperFilter.filterIn(tempForce);
      break;
    case USB_EFFECT_INERTIA:
      // tempForce = interiaFilter.filterIn(tempForce);
      break;
    case USB_EFFECT_FRICTION:
      // tempForce = frictionFilter.filterIn(tempForce);
      break;
    default:
      break;
  }
  return (int32_t)tempForce;
}

float FFB::NormalizeRange(int32_t x, int32_t maxValue) {
  return ((float)x * 1.00 / maxValue) * 20000.0f - 10000.0f;
}

int32_t FFB::ApplyGain(int16_t value, uint8_t gain) {
  int32_t value_32 = value;
  return ((value_32 * gain) / 255);
}

int32_t FFB::ApplyEnvelope(volatile TEffectState& effect, int32_t value) {
  int32_t magnitude = ApplyGain(effect.magnitude, effect.gain);
  int32_t attackLevel = ApplyGain(effect.attackLevel, effect.gain);
  int32_t fadeLevel = ApplyGain(effect.fadeLevel, effect.gain);
  int32_t newValue = magnitude;
  int32_t attackTime = effect.attackTime;
  int32_t fadeTime = effect.fadeTime;
  int32_t elapsedTime = effect.elapsedTime;
  int32_t duration = effect.duration;

  if (attackTime != 0 && elapsedTime < attackTime) {
    newValue = (magnitude - attackLevel) * elapsedTime / attackTime;
    newValue += attackLevel;
  }
  if (fadeTime != 0 && elapsedTime > (duration - fadeTime)) {
    newValue = (magnitude - fadeLevel) * (duration - elapsedTime);
    newValue /= fadeTime;
    newValue += fadeLevel;
  }
  newValue = (magnitude != 0) ? (newValue * value / magnitude) : 0;
  return newValue;
}