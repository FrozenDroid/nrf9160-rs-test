target extended-remote :2331

# print demangled symbols
set print asm-demangle on

# detect unhandled exceptions, hard faults and panics
break DefaultHandler
break UserHardFault
break rust_begin_unwind

# monitor arm semihosting enable

load

# start the process but immediately halt the processor
stepi
