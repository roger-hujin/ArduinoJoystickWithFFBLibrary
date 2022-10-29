#include "PIDReportHandler.h"

#include "Utils.h"

PIDReportHandler::PIDReportHandler() {
  nextEffectID = 1;
  ResetEffectModule();
}

PIDReportHandler::~PIDReportHandler() {
  FreeAllEffects();
  ResetEffectModule();
}

// find next available effect block and return its id
uint8_t PIDReportHandler::GetNextFreeEffect(void) {
  while (effectBlocks[nextEffectID] != 0) {
    nextEffectID++;
    if (nextEffectID >= MAX_EFFECTS) return 0;
  }
  effectBlocks[nextEffectID] = MEFFECTSTATE_ALLOCATED;
  return nextEffectID;
}

void PIDReportHandler::CreateNewEffect(
    USB_FFBReport_CreateNewEffect_Feature_Data_t* inData) {
  Serial.println("Creating New Effect");
  pidBlockLoad.reportId = 6;
  pidBlockLoad.effectBlockIndex = GetNextFreeEffect();

  if (pidBlockLoad.effectBlockIndex == 0) {
    // allocation failed, set status to FULL
    pidBlockLoad.loadStatus = 2;  // 1=Success,2=Full,3=Error
  } else {
    // allocated successfully, set status to SUCCESS
    pidBlockLoad.loadStatus = 1;  // 1=Success,2=Full,3=Error

    // reduce RAM pool size
    pidBlockLoad.ramPoolAvailable -= SIZE_EFFECT;
  }

  Serial.print(pidBlockLoad.effectBlockIndex);
  Serial.print(",");
  Serial.print(pidBlockLoad.loadStatus);
  Serial.print(",");
  Serial.println(pidBlockLoad.ramPoolAvailable);
}

void PIDReportHandler::ProcessUsbData(uint8_t* data, uint16_t len) {
  // dump the received usb data to Serial
  LogUsbData(data, len);

  switch (data[0])  // reportID
  {
    case 11:
      BlockFree((USB_FFBReport_BlockFree_Output_Data_t*)data);
      break;
    case 12:
      DeviceControl((USB_FFBReport_DeviceControl_Output_Data_t*)data);
      break;
    default:
      break;
  }

  // Send to effect manager
  SendToEffectModule(data, len);
}

uint8_t* PIDReportHandler::getPIDBlockLoad() { return (uint8_t*)&pidBlockLoad; }

void PIDReportHandler::clearPIDBlockLoadFlag() { pidBlockLoad.reportId = 0; }

uint8_t* PIDReportHandler::getPIDStatus() { return (uint8_t*)&pidState; }

uint8_t* PIDReportHandler::getPIDPool() {
  FreeAllEffects();

  pidPoolReport.reportId = 7;
  pidPoolReport.ramPoolSize = MEMORY_SIZE;
  pidPoolReport.maxSimultaneousEffects = MAX_EFFECTS;
  pidPoolReport.memoryManagement = 3;
  return (uint8_t*)&pidPoolReport;
}

// device control
void PIDReportHandler::DeviceControl(
    USB_FFBReport_DeviceControl_Output_Data_t* data) {
  uint8_t control = data->control;

  if (control == 0x01) {  // 1=Enable Actuators
    pidState.status |= 2;
  } else if (control == 0x02) {  // 2=Disable Actuators
    pidState.status &= ~(0x02);
  } else if (control == 0x03) {  // 3=Stop All Effects
    // process in effect manager device control
  } else if (control == 0x04) {  //  4=Reset
    FreeAllEffects();
  } else if (control == 0x05) {  // 5=Pause
    // process in effect manager device control
  } else if (control == 0x06) {  // 6=Continue
    // process in effect manager device control
  } else if (control & (0xFF - 0x3F)) {
  }
}

void PIDReportHandler::FreeEffect(uint8_t id) {
  if (id > MAX_EFFECTS) return;
  effectBlocks[id] = 0;
  nextEffectID = (id < nextEffectID) ? id : nextEffectID;
  pidBlockLoad.ramPoolAvailable += SIZE_EFFECT;
}

void PIDReportHandler::FreeAllEffects(void) {
  nextEffectID = 1;
  memset((void*)&effectBlocks, 0, sizeof(effectBlocks));
  pidBlockLoad.ramPoolAvailable = MEMORY_SIZE;
}

void PIDReportHandler::BlockFree(USB_FFBReport_BlockFree_Output_Data_t* data) {
  uint8_t effectID = data->effectBlockIndex;

  if (effectID == 0xFF) {  // all effects
    FreeAllEffects();
  } else {
    FreeEffect(effectID);
  }
}

void PIDReportHandler::SendToEffectModule(uint8_t* data, uint16_t len) {
  if (effectChannel) {
    effectChannel->send(data, len);
  } else if (effectModule) {
    effectModule->ProcessData(data, len);
  }
}

void PIDReportHandler::ResetEffectModule() {
  uint8_t data[] = {12, 04};  // Device Control - Reset
  SendToEffectModule(data, sizeof(data));
}

void PIDReportHandler::SetEffectChannel(PacketSerial* channel) {
  effectChannel = channel;
}