#include "fault_log_core.hpp"
#include <array>
#include <cassert>
#include <iostream>
namespace {
constexpr size_t n=sizeof(faultlog::Record)/4;
std::array<uint32_t,n*8> flash;
int writes_left=-1;
uint32_t Read(size_t i){return flash.at(i);}
bool Program(size_t i,uint32_t v) {
  if(writes_left==0)return false;
  if(writes_left>0)--writes_left;
  if(flash.at(i)!=0xffffffff)return false;
  flash[i]=v;return true;
}
}
int main() {
  using namespace faultlog;
  flash.fill(0xffffffff);Journal j{flash.size(),Read,Program};Record r{};
  assert(!j.load(r));Restore(j,1,123,false);
  FaultLogSnapshot s{};s.tick_ms=10;s.mode=2;s.reasons=16;
  FaultLogObserve(&s,0);assert(!Pending() && !dbg_fault_log_valid); // preparation is not a fault
  s.flags=256;FaultLogObserve(&s,0);assert(!OutputsStopped() && !Pending());
  s.flags=0;s.reasons=0;FaultLogObserve(&s,1);assert(!OutputsStopped());
  s.tick_ms=20;s.yaw_rate=3.5f;FaultLogObserve(&s,1);
  s.tick_ms=21;s.reasons=1024;FaultLogObserve(&s,0);
  assert(Pending() && OutputsStopped() && dbg_fault_log_valid && dbg_fault_log.event==1);
  assert(dbg_fault_log.reasons==1024 && dbg_fault_log.yaw_rate==3.5f);
  s.tick_ms=22;s.reasons=8192;FaultLogObserve(&s,0);assert(dbg_fault_log.reasons==1024);
  Flush(j);assert(!Pending() && dbg_fault_log_storage==1 && j.load(r));
  assert(r.sequence==1 && r.snapshot.tick_ms==21);
  auto saved=flash;Restore(j,1,456,true);assert(!Pending() && flash==saved); // no duplicate
  s.tick_ms=100;s.reasons=0;FaultLogObserve(&s,1);
  s.tick_ms=110;s.yaw_rate=4.1f;FaultLogObserve(&s,1);
  Restore(j,1,789,true);assert(Pending() && dbg_fault_log.event==2);
  assert(dbg_fault_log.reset_flags==789 && dbg_fault_log.tick_ms==110);
  Flush(j);assert(j.load(r) && r.sequence==2);
  Restore(j,1,12,true);s.tick_ms=200;FaultLogObserve(&s,1);
  FaultLogCpuFault(1,2,3,4);assert(Pending() && dbg_fault_log.event==3 && dbg_fault_log.cfsr==1);
  FaultLogCpuFault(9,9,9,9);assert(dbg_fault_log.cfsr==1);
  // A reset before the stopped-loop flush recovers the first CPU exception.
  writes_left=2;Flush(j);assert(Pending() && dbg_fault_log_storage==3);
  Restore(j,1,99,true);assert(Pending() && dbg_fault_log.event==3);
  writes_left=-1;Flush(j);
  Restore(j,1,100,false);assert(!Pending() && dbg_fault_log.event==3); // power cycle reads Flash

  // Every possible torn word retains the previous committed event, then allows append.
  saved=flash;const auto next=Make(s,4);
  for(size_t cut=0;cut<n;++cut) {
    flash=saved;writes_left=int(cut);auto attempt=next;assert(j.append(attempt)==3);
    assert(j.load(r) && r.sequence==3);
    writes_left=-1;assert(j.append(attempt)==1 && j.load(r) && r.sequence==4);
  }
  flash=saved;writes_left=-1;
  Journal small{n,Read,Program};auto attempt=next;
  assert(small.append(attempt)==2 && flash==saved); // full never erases
  flash[3]^=1;assert(j.load(r) && r.sequence==3); // damaged earlier record ignored
  std::cout<<"Fault journal: first fault, reset/power recovery, CPU fault, torn writes, CRC and full storage passed\n";
}
