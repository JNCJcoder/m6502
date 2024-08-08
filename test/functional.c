#define PROGRAM_FILE "6502_functional_test.bin"
#define PROGRAM_FILE_START 0x0000
#define PROGRAM_START 0x0400
#define SUCCESS_PC 0x3469

#include "test.h"

int main(void)
{
    ClearMemory();

    M6502_t cpu;

    if(!OpenFileTest())
    {
        return 1;
    }

    M6502_Init(&cpu, PROGRAM_START);

    uint16_t previousProgramCounter = 0x0000;

    do
    {
        cpu.cycles = 0;
        
        if(cpu.programCounter == SUCCESS_PC) break;

        if(cpu.programCounter == previousProgramCounter)
        {
            printf("[Functional] Trap! - PC: 0x%04x\n", cpu.programCounter);
            exit(1);
        }

        previousProgramCounter = cpu.programCounter;
        
        M6502_Step(&cpu);

    } while (1);
    
    printf("[Functional] Passed!\n");

    return 0;
}
