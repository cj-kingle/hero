#include "yaw_calibration.hpp"
#include "main.h"
extern "C" uint8_t _yaw_calibration_start[],_yaw_calibration_end[];
namespace chassis {
namespace {
uintptr_t Base(){return reinterpret_cast<uintptr_t>(_yaw_calibration_start);}
uint32_t Read(size_t word){return *reinterpret_cast<volatile const uint32_t*>(Base()+word*4);}
bool Program(size_t word,uint32_t value) {
  const uintptr_t address=Base()+word*4;
  if(address>=reinterpret_cast<uintptr_t>(_yaw_calibration_end) || Read(word)!=0xffffffff)return false;
  return HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,uint32_t(address),value)==HAL_OK;
}
CalibrationJournal Journal(){return {(reinterpret_cast<uintptr_t>(_yaw_calibration_end)-Base())/4,Read,Program};}
}
bool LoadYawCalibration(float& value){return Journal().load(value);}
CalibrationSave SaveYawCalibration(float value) {
  if(HAL_FLASH_Unlock()!=HAL_OK)return CalibrationSave::FlashError;
  // Never erase a sector with the watchdog/control running. An exhausted journal
  // reports Full and preserves its old calibration. 128 KiB holds 8192 records.
  const auto result=Journal().save(value);
  HAL_FLASH_Lock();return result;
}
}
