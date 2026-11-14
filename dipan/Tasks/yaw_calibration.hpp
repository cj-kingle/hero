#pragma once
#include <cstddef>
#include <cstdint>
namespace chassis {
enum class CalibrationSave : uint32_t { Saved=1, FlashError=3, Full=4 };
// Append-only records: interrupted writes never replace the last valid record.
struct CalibrationJournal {
  size_t words=0;
  uint32_t (*read)(size_t)=nullptr;
  bool (*program)(size_t,uint32_t)=nullptr;
  bool load(float& offset) const;
  CalibrationSave save(float offset) const;
};
bool LoadYawCalibration(float& offset);
CalibrationSave SaveYawCalibration(float offset);
}
