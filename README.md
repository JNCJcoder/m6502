# M6502 - MOS 6502 Emulator

A 6502 Emulator written in C.

## 🏗️ How-To (Example)


```
#include <stdint.h>
#include "m6502.h"

uint8_t memory[0x10000];

uint8_t M6502_ExternalReadMemory(uint16_t address)
{
    return memory[address];
}

void M6502_ExternalWriteMemory(uint16_t address, uint8_t value)
{
    memory[address] = value;
}

int main()
{
    M6502_t cpu;
    M6502_Init(&cpu, 0x0000);

    while(1)
    {
        M6502_Step(&cpu);
    }

    return 0;
}

```