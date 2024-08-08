#define PROGRAM_FILE "6502_decimal_test.bin"
#define PROGRAM_FILE_START 0x0200
#define PROGRAM_FILE_SIZE 234
#define PROGRAM_START 0x0200
#define SUCCESS_PC 0x024B

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
            printf("[Decimal] Trap! - PC: 0x%04x\n", cpu.programCounter);
            exit(1);
        }
        
        previousProgramCounter = cpu.programCounter; 

        M6502_Step(&cpu);

    } while (1);
    
    printf("[Decimal] Passed!\n");

    return 0;
}
