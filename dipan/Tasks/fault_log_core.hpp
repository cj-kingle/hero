#pragma once
#include "fault_log.h"
#include <cstddef>
#include <cstdint>
namespace faultlog {
constexpr uint32_t kMagic=0x464c4731;
struct Record {
  uint32_t magic,version,sequence;
  FaultLogSnapshot snapshot;
  uint32_t crc,commit;
};
static_assert(sizeof(Record)%4==0);
uint32_t Crc(const void* data,size_t bytes);
bool Valid(const Record& record);
Record Make(const FaultLogSnapshot&,uint32_t sequence);
struct Journal {
  size_t words;
  uint32_t (*read)(size_t);
  bool (*program)(size_t,uint32_t);
  bool load(Record&) const;
  // Commit is programmed last. Torn/dirty slots are never reused.
  uint32_t append(Record&) const;
};
// Called by hardware adapter, also exercised by host tests.
void Restore(Journal&,uint32_t board,uint32_t reset_flags,bool ram_retained);
void Flush(Journal&);
bool Pending();
bool OutputsStopped();
}
