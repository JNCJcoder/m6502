#define PROGRAM_FILE "6502_interrupt_test.bin"
#define PROGRAM_FILE_START 0x000a
#define PROGRAM_START 0x0400
#define SUCCESS_PC 0x06F5

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

    const uint8_t IRQ_BIT = (1 << 0);
    const uint8_t NMI_BIT = (1 << 1);

    uint16_t previousProgramCounter = 0x0000;
    uint8_t previousFeedback = 0x00;
    uint8_t feedback = 0x00;

    previousFeedback = M6502_ExternalReadMemory(0xBFFC);

    do
    {
        cpu.cycles = 0;

        feedback = M6502_ExternalReadMemory(0xBFFC);

        if ((feedback & NMI_BIT) && !(previousFeedback & NMI_BIT))
        {
            M6502_NMI(&cpu);
            previousFeedback |= NMI_BIT;
        }
        else if ((feedback & IRQ_BIT) && !(previousFeedback & IRQ_BIT))
        {
            M6502_IRQ(&cpu);
	        previousFeedback |= IRQ_BIT;
        }
        else if ((previousFeedback & NMI_BIT) && !(feedback & NMI_BIT))
        {
            previousFeedback &= ~NMI_BIT;
        }
        else if ((previousFeedback & IRQ_BIT) && !(feedback & IRQ_BIT))
        {
            previousFeedback &= ~IRQ_BIT;
        }

        if(cpu.programCounter == SUCCESS_PC) break;

        if(cpu.programCounter == previousProgramCounter)
        {
            printf("[Interrupt] Trap! - PC: 0x%04x\n", cpu.programCounter);
            exit(1);
        }
        
        previousProgramCounter = cpu.programCounter; 
        
        M6502_Step(&cpu);

    } while (1);
    
    printf("[Interrupt] Passed!\n");

    return 0;
}
