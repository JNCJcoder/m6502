#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "../m6502.h"

#ifndef PROGRAM_FILE
    #define PROGRAM_FILE ""
    #error "MISSING PROGRAM_FILE"
#endif

#ifndef PROGRAM_FILE_START
    #define PROGRAM_FILE_START 0x0000
#endif

#ifndef PROGRAM_FILE_SIZE
    #define PROGRAM_FILE_SIZE 0x10000
#endif

#ifndef PROGRAM_START
    #define PROGRAM_START 0x0000
    #error "MISSING PROGRAM_START"
#endif

#ifndef SUCCESS_PC
    #define SUCCESS_PC 0x0000
    #error "MISSING SUCCESS_PC"
#endif

#define MEMORY_SIZE 0x10000

uint8_t memory[MEMORY_SIZE];

uint8_t M6502_ExternalReadMemory(uint16_t address)
{
    return memory[address];
}

void M6502_ExternalWriteMemory(uint16_t address, uint8_t value)
{
    memory[address] = value;
}

uint8_t GetFlag(uint8_t status, uint8_t flag)
{
    return ((status & flag) > 0) ? 1 : 0;
}

void ClearMemory()
{
    for (size_t index = 0; index < MEMORY_SIZE; ++index)
    {
        memory[index] = 0x00;
    }
}

uint8_t OpenFileTest()
{
    FILE *fp = fopen(PROGRAM_FILE, "rb");
    
    if(fp == NULL)
    {
        printf(PROGRAM_FILE);
        printf(" not found!\n");
        return 0;
    }

    size_t result = fread(memory + PROGRAM_FILE_START, 1, MEMORY_SIZE, fp);

    fclose(fp);

    return 1;
}