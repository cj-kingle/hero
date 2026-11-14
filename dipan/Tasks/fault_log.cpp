#include "fault_log_core.hpp"
#include <cstring>

volatile FaultLogSnapshot dbg_fault_log{};
volatile uint32_t dbg_diag_version=2026111303u,dbg_stop_reason=0,dbg_stop_event=0,dbg_stop_tick_ms=0;
volatile uint32_t dbg_fault_log_valid=0,dbg_fault_log_sequence=0;
volatile uint32_t dbg_fault_log_storage=0,dbg_fault_log_reset_flags=0;
namespace {
// Outside startup .bss clearing; still validated before every recovery.
struct Retained {faultlog::Record checkpoint[2],event;};
#if defined(__arm__) || defined(__thumb__)
__attribute__((section(".fault_log_noinit"),used)) Retained retained;
#else
Retained retained{};
#endif
bool working_seen=false,frozen=false,pending=false;
bool output_working=false;
uint32_t last_checkpoint=0,board_id=0;
unsigned checkpoint_slot=0;
const faultlog::Record* Checkpoint() {
  const auto& a=retained.checkpoint[0];const auto& b=retained.checkpoint[1];
  const bool av=faultlog::Valid(a),bv=faultlog::Valid(b);
  if(!av)return bv?&b:nullptr;
  if(!bv)return &a;
  return int32_t(b.snapshot.tick_ms-a.snapshot.tick_ms)>0 ? &b : &a;
}
void Publish(const faultlog::Record& r) {
  dbg_fault_log_valid=0;
  const auto* src=reinterpret_cast<const unsigned char*>(&r.snapshot);
  auto* dst=reinterpret_cast<volatile unsigned char*>(&dbg_fault_log);
  for(size_t i=0;i<sizeof(r.snapshot);++i)dst[i]=src[i];
  dbg_stop_reason=r.snapshot.reasons;dbg_stop_event=r.snapshot.event;dbg_stop_tick_ms=r.snapshot.tick_ms;
  dbg_fault_log_sequence=r.sequence;dbg_fault_log_valid=1;
}
void Capture(FaultLogSnapshot s) {
  s.board=board_id;
  retained.event=faultlog::Make(s,dbg_fault_log_sequence+1);
  frozen=true;pending=true;Publish(retained.event);
}
}
namespace faultlog {
uint32_t Crc(const void* data,size_t bytes) {
  uint32_t crc=0xffffffff;
  const auto* p=static_cast<const unsigned char*>(data);
  for(size_t i=0;i<bytes;++i) {
    crc^=p[i];
    for(unsigned bit=0;bit<8;++bit)crc=(crc>>1)^(0xedb88320u & (0u-(crc&1u)));
  }
  return ~crc;
}
bool Valid(const Record& r) {
  return r.magic==kMagic && r.version==1 && r.commit==~kMagic &&
      r.crc==Crc(&r,offsetof(Record,crc));
}
Record Make(const FaultLogSnapshot& s,uint32_t sequence) {
  Record r{};r.magic=kMagic;r.version=1;r.sequence=sequence;r.snapshot=s;
  r.crc=Crc(&r,offsetof(Record,crc));r.commit=~kMagic;return r;
}
bool Journal::load(Record& out) const {
  bool found=false;
  constexpr size_t n=sizeof(Record)/4;
  for(size_t at=0;at+n<=words;at+=n) {
    Record r{};
    for(size_t i=0;i<n;++i) {const uint32_t v=read(at+i);std::memcpy(reinterpret_cast<unsigned char*>(&r)+4*i,&v,4);}
    if(Valid(r)) {out=r;found=true;}
  }
  return found;
}
uint32_t Journal::append(Record& r) const {
  constexpr size_t n=sizeof(Record)/4;
  size_t next=0;
  for(size_t at=0;at+n<=words;at+=n) {
    bool dirty=false;for(size_t i=0;i<n;++i)dirty|=read(at+i)!=0xffffffff;
    if(dirty)next=at+n;
  }
  if(next+n>words)return 2;
  for(size_t i=0;i<n;++i) {
    uint32_t v;std::memcpy(&v,reinterpret_cast<const unsigned char*>(&r)+i*4,4);
    if(!program(next+i,v))return 3;
  }
  Record check{};
  for(size_t i=0;i<n;++i) {const uint32_t v=read(next+i);std::memcpy(reinterpret_cast<unsigned char*>(&check)+i*4,&v,4);}
  return Valid(check)?1:3;
}
void Restore(Journal& j,uint32_t board,uint32_t reset_flags,bool ram_retained) {
  dbg_diag_version=2026111303u; // a live reference keeps the marker in the ELF
  board_id=board;working_seen=frozen=pending=output_working=false;last_checkpoint=0;
  dbg_fault_log_valid=0;dbg_fault_log_sequence=0;dbg_fault_log_storage=0;
  dbg_stop_reason=0;dbg_stop_event=0;dbg_stop_tick_ms=0;
  dbg_fault_log_reset_flags=reset_flags;
  Record saved{};const bool have=j.load(saved);
  if(have)Publish(saved);
  // A completed journal write is never duplicated after a reset.
  if(ram_retained && Valid(retained.event) && retained.event.snapshot.board==board &&
      (!have || retained.event.sequence>saved.sequence)) {
    pending=frozen=true;Publish(retained.event);
  } else if(ram_retained && !(have && Valid(retained.event) && retained.event.sequence==saved.sequence) && Checkpoint() &&
      Checkpoint()->snapshot.board==board) {
    auto s=Checkpoint()->snapshot;s.event=2;s.reset_flags=reset_flags;Capture(s);
  }
  retained.checkpoint[0]={};retained.checkpoint[1]={};
  if(!pending)retained.event={};
}
bool Pending(){return pending;}
bool OutputsStopped(){return !output_working;}
void Flush(Journal& j) {
  if(!pending)return;
  dbg_fault_log_storage=j.append(retained.event);
  // Keep pending record in noinit even on write failure, for next boot recovery.
  if(dbg_fault_log_storage==1) {pending=false;retained.checkpoint[0]={};retained.checkpoint[1]={};}
}
}
void FaultLogObserve(const FaultLogSnapshot* snapshot,int working) {
  if(!snapshot)return;
  output_working=working!=0 || (snapshot->flags&256u)!=0;
  if(working) {
    // A new run may replace a saved event, but never an unsaved first fault.
    if(pending)return;
    const bool first=!working_seen;
    if(first) {frozen=false;retained.event={};}
    working_seen=true;
    if(first || uint32_t(snapshot->tick_ms-last_checkpoint)>=10u) {
      auto s=*snapshot;s.board=board_id;s.event=0;
      checkpoint_slot^=1u;
      retained.checkpoint[checkpoint_slot]=faultlog::Make(s,dbg_fault_log_sequence);
      last_checkpoint=s.tick_ms;
    }
  } else if(working_seen && !frozen) {
    auto s=*snapshot;s.event=1;s.reset_flags=dbg_fault_log_reset_flags;
    Capture(s);retained.checkpoint[0]={};retained.checkpoint[1]={};working_seen=false;
  }
}
void FaultLogCpuFault(uint32_t cfsr,uint32_t hfsr,uint32_t bfar,uint32_t mmfar) {
  if(frozen)return;
  FaultLogSnapshot s{};
  if(const auto* checkpoint=Checkpoint())s=checkpoint->snapshot;
  s.event=3;s.cfsr=cfsr;s.hfsr=hfsr;s.bfar=bfar;s.mmfar=mmfar;
  Capture(s);
}
