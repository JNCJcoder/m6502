# M6502 - MOS 6502 Emulator

A NMOS 6502 Emulator written in C.

Tests:

- [x] [6502_functional_test](https://github.com/Klaus2m5/6502_65C02_functional_tests/blob/master/6502_functional_test.a65)
- [x] [6502_decimal_test](https://github.com/Klaus2m5/6502_65C02_functional_tests/blob/master/6502_decimal_test.a65)
- [x] [6502_interrupt_test](https://github.com/Klaus2m5/6502_65C02_functional_tests/blob/master/6502_interrupt_test.a65)

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

    /* M6502_IRQ(&cpu); /* IRQ */ */
    /* M6502_NMI(&cpu); /* NMI */ */
    /* M6502_Reset(&cpu); /* Reset */ */

    while(1)
    {
        M6502_Step(&cpu);
    }

    return 0;
}

```