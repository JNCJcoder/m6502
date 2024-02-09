#ifndef __M6502_H__
#define __M6502_H__

#include <stdint.h>

typedef struct
{
    uint16_t    programCounter;
    uint8_t     xRegister;
    uint8_t     yRegister;
    uint8_t     accumulator;
    uint8_t     stackPointer;
    uint8_t     statusRegister;
    /**/
    uint8_t     cycles;
    uint8_t     opcode;
    uint8_t     interruptFlags;
    uint8_t     pendingInterrupts;
    uint16_t    address;
    uint16_t    target;
} M6502_t;

void M6502_Init(M6502_t *cpu, uint16_t startAddress);
void M6502_Reset(M6502_t *cpu);
void M6502_Step(M6502_t *cpu);
void M6502_IRQ(M6502_t *cpu);
void M6502_NMI(M6502_t *cpu);

uint8_t  M6502_ExternalReadMemory(uint16_t address);
void     M6502_ExternalWriteMemory(uint16_t address, uint8_t value);

#endif /* __M6502_H__ */