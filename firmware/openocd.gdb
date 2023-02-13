target extended-remote :3333

# print demangled symbols
set print asm-demangle on

# set backtrace limit to not have infinite backtrace loops
set backtrace limit 32

monitor arm semihosting enable

monitor stm32g4x.tpiu disable
monitor stm32g4x.tpiu configure -protocol uart -traceclk 16000000 -pin-freq 2000000 -output itm.txt -formatter off
monitor stm32g4x.tpiu enable
monitor itm port 0 on

load

continue

define hook-quit
    set confirm off
end
