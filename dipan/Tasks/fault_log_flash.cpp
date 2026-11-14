#include "fault_log_core.hpp"
#include "stm32f4xx_hal.h"
extern "C" uint8_t _fault_log_start[],_fault_log_end[];
namespace {
uintptr_t Base(){return reinterpret_cast<uintptr_t>(_fault_log_start);}
uint32_t Read(size_t i){return *reinterpret_cast<volatile const uint32_t*>(Base()+i*4);}
bool Program(size_t i,uint32_t v) {
  const auto address=Base()+i*4;
  if(address>=reinterpret_cast<uintptr_t>(_fault_log_end) || Read(i)!=0xffffffff)return false;
  return HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,uint32_t(address),v)==HAL_OK;
}
faultlog::Journal Journal(){return {(reinterpret_cast<uintptr_t>(_fault_log_end)-Base())/4,Read,Program};}
bool waiting=false,attempted=false;
uint32_t pending_since=0;
void Save(){
  auto j=Journal();
  if(HAL_FLASH_Unlock()!=HAL_OK){dbg_fault_log_storage=3;return;}
  faultlog::Flush(j);HAL_FLASH_Lock();
}
}
void FaultLogBoot(uint32_t board) {
  const uint32_t flags=RCC->CSR;
  auto j=Journal();
  // POR/BOR do not guarantee SRAM retention. Flash records remain valid.
  faultlog::Restore(j,board,flags,(flags&(RCC_CSR_PORRSTF|RCC_CSR_BORRSTF))==0);
  __HAL_RCC_CLEAR_RESET_FLAGS();
  // Boot recovery happens before motor/CAN initialization and watchdog startup.
  if(faultlog::Pending())Save();
}
void FaultLogPoll(void) {
  if(!faultlog::OutputsStopped()){waiting=false;return;}
  if(!faultlog::Pending()){waiting=attempted=false;return;}
  if(attempted)return;
  const uint32_t now=HAL_GetTick();
  if(!waiting){waiting=true;pending_since=now;return;}
  // Caller runs this only after outputs have stopped. Allow stop CAN frames time.
  if(uint32_t(now-pending_since)<50u)return;
  attempted=true;Save();
}
