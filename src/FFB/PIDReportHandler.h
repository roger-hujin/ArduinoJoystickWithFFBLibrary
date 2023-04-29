#ifndef _PIDREPORTHANDLER_H
#define _PIDREPORTHANDLER_H
#include <Arduino.h>

#include "PIDReportType.h"
#include "PacketSerial.h"

class PIDReportHandler {
 private:
  // effect blocks
  volatile uint8_t nextEffectID;  // FFB effect indexes starts from 1
  volatile uint8_t effectBlocks[MAX_EFFECTS + 1];

  // device report
  volatile USB_FFBReport_PIDStatus_Input_Data_t pidState = {2, 30, 0};
  volatile USB_FFBReport_PIDBlockLoad_Feature_Data_t pidBlockLoad;
  volatile USB_FFBReport_PIDPool_Feature_Data_t pidPoolReport = {
      7, MEMORY_SIZE, MAX_EFFECTS, 3};

  // effect blocks management
  uint8_t GetNextFreeEffect();
  void FreeEffect(uint8_t id);
  void FreeAllEffects();
  void StopAllEffects();
  void BlockFree(USB_FFBReport_BlockFree_Output_Data_t* data);
  void DeviceControl(USB_FFBReport_DeviceControl_Output_Data_t* data);

  // effect module interface
  EffectModule* effectModule = nullptr;
  PacketSerial* effectChannel = nullptr;
  void SendToEffectModule(uint8_t* data, uint16_t len);
  void ResetEffectModule();

 public:
  PIDReportHandler();
  ~PIDReportHandler();

  // interface to DynamicHID
  void CreateNewEffect(USB_FFBReport_CreateNewEffect_Feature_Data_t* inData);
  void ProcessUsbData(uint8_t* data, uint16_t len);

  void SetEffectModule(EffectModule* ffb) { effectModule = ffb; };
  void SetEffectChannel(PacketSerial* channel);

  uint8_t* getPIDPool();
  uint8_t* getPIDStatus();
  uint8_t* getPIDBlockLoad();
  void clearPIDBlockLoadFlag();

  uint16_t commandSentThisUpdate = 0;
};
#endif
