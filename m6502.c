#include "m6502.h"

static const uint16_t M6502_NMIVECTOR_ADDRESS   = 0xFFFA;
static const uint16_t M6502_RESETVECTOR_ADDRESS = 0xFFFC;
static const uint16_t M6502_IRQVECTOR_ADDRESS   = 0xFFFE;

static const uint16_t M6502_STACK_ADDRESS       = 0x0100;
static const uint16_t M6502_STACK_START_ADDRESS = 0x00FD;

static const uint8_t M6502_FLAG_CARRY       = 0x01;
static const uint8_t M6502_FLAG_ZERO        = 0x02;
static const uint8_t M6502_FLAG_INTERRUPT   = 0x04;
static const uint8_t M6502_FLAG_DECIMAL     = 0x08;
static const uint8_t M6502_FLAG_BREAK       = 0x10;
static const uint8_t M6502_FLAG_UNUSED      = 0x20;
static const uint8_t M6502_FLAG_OVERFLOW    = 0x40;
static const uint8_t M6502_FLAG_NEGATIVE    = 0x80;

static const uint8_t M6502_MAGIC_CONSTANT   = 0x00;

static const uint8_t M6502_OPCODE_CYCLES[0x100] = {
    7, 6, 2, 8, 3, 3, 5, 5, 3, 2, 2, 2, 4, 4, 6, 6,
    2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
    6, 6, 2, 8, 3, 3, 5, 5, 4, 2, 2, 2, 4, 4, 6, 6,
    2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
    6, 6, 2, 8, 3, 3, 5, 5, 3, 2, 2, 2, 3, 4, 6, 6,
    2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
    6, 6, 2, 8, 3, 3, 5, 5, 4, 2, 2, 2, 5, 4, 6, 6,
    2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
    2, 6, 2, 6, 3, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 4,
    2, 6, 2, 6, 4, 4, 4, 4, 2, 5, 2, 5, 5, 5, 5, 5,
    2, 6, 2, 6, 3, 3, 3, 3, 2, 2, 2, 2, 4, 4, 4, 4,
    2, 5, 2, 5, 4, 4, 4, 4, 2, 4, 2, 4, 4, 4, 4, 4,
    2, 6, 2, 8, 3, 3, 5, 5, 2, 2, 2, 2, 4, 4, 6, 6,
    2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7,
    2, 6, 2, 8, 3, 3, 5, 5, 2, 2, 2, 2, 4, 4, 6, 6,
    2, 5, 2, 8, 4, 4, 6, 6, 2, 4, 2, 7, 4, 4, 7, 7 
};

static inline uint8_t   M6502_ReadMemoryByte(const uint16_t address);
static inline uint16_t  M6502_ReadMemoryWord(const uint16_t address);
static inline void      M6502_WriteMemoryByte(const uint16_t address, const uint8_t value);
static inline void      M6502_WriteMemoryWord(const uint16_t address, const uint16_t value);

static inline void      M6502_SetFlag(M6502_t* cpu, const uint8_t flag, const uint8_t value);
static inline uint8_t   M6502_GetFlag(M6502_t* cpu, const uint8_t flag);

static inline void      M6502_PushByte(M6502_t* cpu, const uint8_t value);
static inline void      M6502_PushWord(M6502_t* cpu, const uint16_t value);
static inline uint8_t   M6502_PullByte(M6502_t* cpu);
static inline uint16_t  M6502_PullWord(M6502_t* cpu);

static inline void M6502_CarryTest(M6502_t* cpu, const uint16_t value);
static inline void M6502_ZeroTest(M6502_t* cpu, const uint16_t value);
static inline void M6502_OverFlowTest(M6502_t* cpu, const uint16_t value, const uint16_t result);
static inline void M6502_NegativeTest(M6502_t* cpu, const uint16_t value);

static inline void M6502_Address_Accumulator(M6502_t* cpu);
static inline void M6502_Address_Immediate(M6502_t* cpu);
static inline void M6502_Address_Relative(M6502_t* cpu);
static inline void M6502_Address_Absolute(M6502_t* cpu);
static inline void M6502_Address_AbsoluteX(M6502_t* cpu);
static inline void M6502_Address_AbsoluteY(M6502_t* cpu);
static inline void M6502_Address_ZeroPage(M6502_t* cpu);
static inline void M6502_Address_ZeroPageX(M6502_t* cpu);
static inline void M6502_Address_ZeroPageY(M6502_t* cpu);
static inline void M6502_Address_Indirect(M6502_t* cpu);
static inline void M6502_Address_IndirectX(M6502_t* cpu);
static inline void M6502_Address_IndirectY(M6502_t* cpu);

static inline void M6502_Opcode_ADC(M6502_t* cpu);
static inline void M6502_Opcode_AND(M6502_t* cpu);
static inline void M6502_Opcode_ASL(M6502_t* cpu);
static inline void M6502_Opcode_BCC(M6502_t* cpu);
static inline void M6502_Opcode_BCS(M6502_t* cpu);
static inline void M6502_Opcode_BEQ(M6502_t* cpu);
static inline void M6502_Opcode_BIT(M6502_t* cpu);
static inline void M6502_Opcode_BMI(M6502_t* cpu);
static inline void M6502_Opcode_BNE(M6502_t* cpu);
static inline void M6502_Opcode_BPL(M6502_t* cpu);
static inline void M6502_Opcode_BRK(M6502_t* cpu);
static inline void M6502_Opcode_BVC(M6502_t* cpu);
static inline void M6502_Opcode_BVS(M6502_t* cpu);
static inline void M6502_Opcode_CLC(M6502_t* cpu);
static inline void M6502_Opcode_CLD(M6502_t* cpu);
static inline void M6502_Opcode_CLI(M6502_t* cpu);
static inline void M6502_Opcode_CLV(M6502_t* cpu);
static inline void M6502_Opcode_CMP(M6502_t* cpu);
static inline void M6502_Opcode_CPX(M6502_t* cpu);
static inline void M6502_Opcode_CPY(M6502_t* cpu);
static inline void M6502_Opcode_DEC(M6502_t* cpu);
static inline void M6502_Opcode_DEX(M6502_t* cpu);
static inline void M6502_Opcode_DEY(M6502_t* cpu);
static inline void M6502_Opcode_EOR(M6502_t* cpu);
static inline void M6502_Opcode_INC(M6502_t* cpu);
static inline void M6502_Opcode_INX(M6502_t* cpu);
static inline void M6502_Opcode_INY(M6502_t* cpu);
static inline void M6502_Opcode_JMP(M6502_t* cpu);
static inline void M6502_Opcode_JSR(M6502_t* cpu);
static inline void M6502_Opcode_LDA(M6502_t* cpu);
static inline void M6502_Opcode_LDX(M6502_t* cpu);
static inline void M6502_Opcode_LDY(M6502_t* cpu);
static inline void M6502_Opcode_LSR(M6502_t* cpu);
static inline void M6502_Opcode_NOP(M6502_t* cpu);
static inline void M6502_Opcode_ORA(M6502_t* cpu);
static inline void M6502_Opcode_PHA(M6502_t* cpu);
static inline void M6502_Opcode_PHP(M6502_t* cpu);
static inline void M6502_Opcode_PLA(M6502_t* cpu);
static inline void M6502_Opcode_PLP(M6502_t* cpu);
static inline void M6502_Opcode_ROL(M6502_t* cpu);
static inline void M6502_Opcode_ROR(M6502_t* cpu);
static inline void M6502_Opcode_RTI(M6502_t* cpu);
static inline void M6502_Opcode_RTS(M6502_t* cpu);
static inline void M6502_Opcode_SBC(M6502_t* cpu);
static inline void M6502_Opcode_SEC(M6502_t* cpu);
static inline void M6502_Opcode_SED(M6502_t* cpu);
static inline void M6502_Opcode_SEI(M6502_t* cpu);
static inline void M6502_Opcode_STA(M6502_t* cpu);
static inline void M6502_Opcode_STX(M6502_t* cpu);
static inline void M6502_Opcode_STY(M6502_t* cpu);
static inline void M6502_Opcode_TAX(M6502_t* cpu);
static inline void M6502_Opcode_TAY(M6502_t* cpu);
static inline void M6502_Opcode_TSX(M6502_t* cpu);
static inline void M6502_Opcode_TXA(M6502_t* cpu);
static inline void M6502_Opcode_TXS(M6502_t* cpu);
static inline void M6502_Opcode_TYA(M6502_t* cpu);

static inline void M6502_Opcode_ALR(M6502_t* cpu);
static inline void M6502_Opcode_ANC(M6502_t* cpu);
static inline void M6502_Opcode_ANE(M6502_t* cpu);
static inline void M6502_Opcode_ARR(M6502_t* cpu);
static inline void M6502_Opcode_DCP(M6502_t* cpu);
static inline void M6502_Opcode_ISC(M6502_t* cpu);
static inline void M6502_Opcode_LAS(M6502_t* cpu);
static inline void M6502_Opcode_LAX(M6502_t* cpu);
static inline void M6502_Opcode_LXA(M6502_t* cpu);
static inline void M6502_Opcode_RLA(M6502_t* cpu);
static inline void M6502_Opcode_RRA(M6502_t* cpu);
static inline void M6502_Opcode_SAX(M6502_t* cpu);
static inline void M6502_Opcode_SBX(M6502_t* cpu);
static inline void M6502_Opcode_SHA(M6502_t* cpu);
static inline void M6502_Opcode_SHX(M6502_t* cpu);
static inline void M6502_Opcode_SHY(M6502_t* cpu);
static inline void M6502_Opcode_SLO(M6502_t* cpu);
static inline void M6502_Opcode_SRE(M6502_t* cpu);
static inline void M6502_Opcode_TAS(M6502_t* cpu);
static inline void M6502_Opcode_USBC(M6502_t* cpu);
static inline void M6502_Opcode_JAM(M6502_t* cpu);

static inline uint8_t M6502_ReadMemoryByte(const uint16_t address)
{
    return M6502_ExternalReadMemory(address);
}

static inline uint16_t M6502_ReadMemoryWord(const uint16_t address)
{
    const uint16_t low  = (uint16_t)M6502_ExternalReadMemory(address);
    const uint16_t high = (uint16_t)M6502_ExternalReadMemory(address + 1) << 8;

    return (high | low);
}

static inline void M6502_WriteMemoryByte(const uint16_t address, const uint8_t value)
{
    M6502_ExternalWriteMemory(address, value);
}

static inline void M6502_WriteMemoryWord(const uint16_t address, const uint16_t value)
{
    const uint8_t low  = (uint8_t)(value & 0xFF);
    const uint8_t high = (uint8_t)((value >> 8) & 0xFF);

    M6502_ExternalWriteMemory(address,      high);
    M6502_ExternalWriteMemory(address + 1,  low);
}

static inline void M6502_SetFlag(M6502_t* cpu, const uint8_t flag, const uint8_t value)
{
    if (value)  cpu->statusRegister |= flag;
	else        cpu->statusRegister &= ~flag;
}

static inline uint8_t M6502_GetFlag(M6502_t* cpu, const uint8_t flag)
{
    return ((cpu->statusRegister & flag) > 0) ? 1 : 0;
}

static inline void M6502_PushByte(M6502_t* cpu, const uint8_t value)
{
    M6502_ExternalWriteMemory(M6502_STACK_ADDRESS + (cpu->stackPointer-- & 0xFF), value);
}

static inline void M6502_PushWord(M6502_t* cpu, const uint16_t value)
{
    const uint8_t high = (uint8_t)((value >> 8) & 0xFF);
    const uint8_t low  = (uint8_t)(value & 0xFF);

    M6502_PushByte(cpu, high);
    M6502_PushByte(cpu, low);
}

static inline uint8_t M6502_PullByte(M6502_t* cpu)
{
    return M6502_ExternalReadMemory(M6502_STACK_ADDRESS + (++cpu->stackPointer & 0xFF));
}

static inline uint16_t M6502_PullWord(M6502_t* cpu)
{
    const uint16_t low  = (uint16_t)M6502_PullByte(cpu); 
    const uint16_t high = (uint16_t)M6502_PullByte(cpu) << 8; 
    
    return (high | low);
}


static inline void M6502_CarryTest(M6502_t* cpu, const uint16_t value)
{
    if(value & 0xFF00)  cpu->statusRegister |= M6502_FLAG_CARRY; 
    else                cpu->statusRegister &= (~M6502_FLAG_CARRY);
}

static inline void M6502_ZeroTest(M6502_t* cpu, const uint16_t value)
{
    if(value & 0x00FF)  cpu->statusRegister &= (~M6502_FLAG_ZERO); 
    else                cpu->statusRegister |= M6502_FLAG_ZERO;
}

static inline void M6502_OverFlowTest(M6502_t* cpu, const uint16_t value, const uint16_t result)
{
    const uint16_t temporaryA = (result ^ (uint16_t)cpu->accumulator);
    const uint16_t temporaryB = (result ^ value);
    const uint16_t temporaryResult = temporaryA & temporaryB & 0x0080;

    if(temporaryResult) cpu->statusRegister |= M6502_FLAG_OVERFLOW; 
    else                cpu->statusRegister &= (~M6502_FLAG_OVERFLOW);
}

static inline void M6502_NegativeTest(M6502_t* cpu, const uint16_t value)
{
    if(value & 0x0080)  cpu->statusRegister |= M6502_FLAG_NEGATIVE; 
    else                cpu->statusRegister &= (~M6502_FLAG_NEGATIVE);
}


static inline void M6502_Address_Accumulator(M6502_t* cpu)
{
    cpu->address = cpu->accumulator;
    cpu->target = cpu->accumulator;
}

static inline void M6502_Address_Immediate(M6502_t* cpu)
{
    cpu->address    = cpu->programCounter++;
    cpu->target     = M6502_ExternalReadMemory(cpu->address);
}

static inline void M6502_Address_Relative(M6502_t* cpu)
{
    cpu->address = (uint16_t)M6502_ExternalReadMemory(cpu->programCounter++);

    if (cpu->address & 0x80)
    {
		cpu->address = cpu->address | 0xFF00;
    }
}

static inline void M6502_Address_Absolute(M6502_t* cpu)
{
    cpu->address = M6502_ReadMemoryWord(cpu->programCounter);
    cpu->target = M6502_ReadMemoryByte(cpu->address);

    cpu->programCounter += 2;
}

static inline void M6502_Address_AbsoluteX(M6502_t* cpu)
{
    cpu->address = M6502_ReadMemoryWord(cpu->programCounter);
    cpu->programCounter += 2;
    const uint16_t pageTest = cpu->address & 0xFF00;

    cpu->address += (uint16_t)cpu->xRegister;

    if(pageTest != (cpu->address & 0xFF00))
    {
        cpu->cycles++;
    }

    cpu->target = M6502_ReadMemoryByte(cpu->address);
}

static inline void M6502_Address_AbsoluteY(M6502_t* cpu)
{
    cpu->address = M6502_ReadMemoryWord(cpu->programCounter);
    cpu->programCounter += 2;
    const uint16_t pageTest = cpu->address & 0xFF00;

    cpu->address += (uint16_t)cpu->yRegister;

    if(pageTest != (cpu->address & 0xFF00))
    {
        cpu->cycles++;
    }

    cpu->target = M6502_ReadMemoryByte(cpu->address);
}

static inline void M6502_Address_ZeroPage(M6502_t* cpu)
{
    cpu->address = (uint16_t)M6502_ReadMemoryByte(cpu->programCounter++);
    cpu->target = M6502_ReadMemoryByte(cpu->address);
}

static inline void M6502_Address_ZeroPageX(M6502_t* cpu)
{
    uint16_t temporary = (uint16_t)M6502_ReadMemoryByte(cpu->programCounter++);
    temporary += (uint16_t)cpu->xRegister;
    temporary &= 0x00FF;

    cpu->address = temporary;
    cpu->target = M6502_ReadMemoryByte(temporary);
}

static inline void M6502_Address_ZeroPageY(M6502_t* cpu)
{
    uint16_t temporary = (uint16_t)M6502_ReadMemoryByte(cpu->programCounter++);
    temporary += (uint16_t)cpu->yRegister;
    temporary &= 0x00FF;

    cpu->address = temporary;
    cpu->target = M6502_ReadMemoryByte(temporary);
}

static inline void M6502_Address_Indirect(M6502_t* cpu)
{
    const uint16_t temporary = M6502_ReadMemoryWord(cpu->programCounter);
    cpu->programCounter += 2;

    const uint16_t temporary2 = (temporary & 0xFF00) | ((temporary + 1) & 0x00FF);

    const uint16_t low  = (uint16_t)M6502_ReadMemoryByte(temporary);
    const uint16_t high = (uint16_t)M6502_ReadMemoryByte(temporary2) << 8;

    cpu->address = (high | low);
}

static inline void M6502_Address_IndirectX(M6502_t* cpu)
{
    uint16_t pointer = (uint16_t)M6502_ReadMemoryByte(cpu->programCounter++);
    pointer += (uint16_t)cpu->xRegister;
    pointer &= 0xFF;

    uint16_t low    = (uint16_t)M6502_ReadMemoryByte(pointer & 0xFF);
	uint16_t high   = (uint16_t)M6502_ReadMemoryByte((pointer + 1) & 0xFF);

	cpu->address    = (uint16_t)((high << 8) | low);
	cpu->target     = M6502_ReadMemoryByte(cpu->address);
}

static inline void M6502_Address_IndirectY(M6502_t* cpu)
{
    const uint16_t pointer = (uint16_t)M6502_ReadMemoryByte(cpu->programCounter++);

    const uint16_t pointer2 = (pointer & 0xFF00) | ((pointer + 1) & 0x00FF);

    const uint16_t low  = (uint16_t)M6502_ReadMemoryByte(pointer);
	const uint16_t high = (uint16_t)M6502_ReadMemoryByte(pointer2) << 8;

    const uint16_t pointerResult = (high | low);
    cpu->address = pointerResult + (uint16_t)cpu->yRegister;
	
	if ((pointerResult & 0xFF00) != (cpu->address & 0xFF00))
    {
        cpu->cycles++;
    }

    cpu->target = M6502_ReadMemoryByte(cpu->address);
}


static inline void M6502_Util_WriteResult(M6502_t* cpu, const uint16_t result)
{
    const uint8_t group     = (cpu->opcode & 0x3);
    const uint8_t address   = ((cpu->opcode & 0x1C) >> 2);

    const uint8_t resultToSave = (uint8_t)(result & 0x00FF);

    if(group == 0x02 && address == 0x02)
    {
        cpu->accumulator = resultToSave;
        return;
    }

    M6502_ExternalWriteMemory(cpu->address, resultToSave);
}

static inline void M6502_Util_BRANCH(M6502_t* cpu)
{
    uint16_t address = cpu->programCounter + (int8_t)cpu->address;

    if((address & 0xFF00) != (cpu->programCounter & 0xFF00))
    {
        cpu->cycles++;
    }
    cpu->cycles++;

    cpu->programCounter = address;
}


static inline void M6502_Opcode_Group01(M6502_t* cpu);
static inline void M6502_Opcode_Group10(M6502_t* cpu);
static inline void M6502_Opcode_Group11(M6502_t* cpu);
static inline void M6502_Opcode_Group00(M6502_t* cpu);
static inline void M6502_Opcode_Group00_Branch(M6502_t* cpu);


void M6502_Init(M6502_t* cpu, uint16_t startAddress)
{
    cpu->programCounter = 0x0000;
    cpu->xRegister      = 0x00;
    cpu->yRegister      = 0x00;
    cpu->accumulator    = 0x00;
    cpu->stackPointer   = 0x00;
    cpu->statusRegister = 0x00;
    cpu->cycles         = 0;
    cpu->opcode         = 0x00;
    cpu->address        = 0x0000;
    cpu->target         = 0x0000;

    M6502_Reset(cpu);

    cpu->programCounter = startAddress ? startAddress : 0x0000;
}

void M6502_Reset(M6502_t* cpu)
{
    cpu->programCounter = M6502_ReadMemoryWord(M6502_RESETVECTOR_ADDRESS);
    cpu->stackPointer   = M6502_STACK_START_ADDRESS;
    cpu->statusRegister |= M6502_FLAG_INTERRUPT | M6502_FLAG_UNUSED;
    cpu->cycles = 8;
}

void M6502_IRQ(M6502_t* cpu)
{
    if(M6502_GetFlag(cpu, M6502_FLAG_INTERRUPT) == 1)
    {
        return;
    }

    M6502_PushWord(cpu, cpu->programCounter);
    M6502_PushByte(cpu, cpu->statusRegister & ~M6502_FLAG_BREAK);

    cpu->statusRegister |= M6502_FLAG_INTERRUPT;
    cpu->programCounter = M6502_ReadMemoryWord(M6502_IRQVECTOR_ADDRESS);

    cpu->cycles = 7;
}

void M6502_NMI(M6502_t* cpu)
{
    M6502_PushWord(cpu, cpu->programCounter);
    M6502_PushByte(cpu, cpu->statusRegister & ~M6502_FLAG_BREAK);
    cpu->statusRegister |= M6502_FLAG_INTERRUPT;

    cpu->programCounter = M6502_ReadMemoryWord(M6502_NMIVECTOR_ADDRESS);
    
    cpu->cycles = 8;
}

void M6502_Step(M6502_t* cpu)
{
    if(cpu->cycles > 0)
    {
        cpu->cycles--;
        return;
    }

    cpu->statusRegister |= M6502_FLAG_UNUSED;

    cpu->opcode = M6502_ExternalReadMemory(cpu->programCounter++);
    cpu->cycles = M6502_OPCODE_CYCLES[cpu->opcode];

    switch (cpu->opcode)
    {
        case 0x00: M6502_Opcode_BRK(cpu);   return;
        case 0x20: M6502_Opcode_JSR(cpu);   return;
        case 0x40: M6502_Opcode_RTI(cpu);   return;
        case 0x60: M6502_Opcode_RTS(cpu);   return;

        case 0x08: M6502_Opcode_PHP(cpu);   return;
        case 0x18: M6502_Opcode_CLC(cpu);   return;
        case 0x28: M6502_Opcode_PLP(cpu);   return;
        case 0x38: M6502_Opcode_SEC(cpu);   return;
        case 0x48: M6502_Opcode_PHA(cpu);   return;
        case 0x58: M6502_Opcode_CLI(cpu);   return;
        case 0x68: M6502_Opcode_PLA(cpu);   return;
        case 0x78: M6502_Opcode_SEI(cpu);   return;
        case 0x88: M6502_Opcode_DEY(cpu);   return;
        case 0x8A: M6502_Opcode_TXA(cpu);   return;
        case 0x98: M6502_Opcode_TYA(cpu);   return;
        case 0x9A: M6502_Opcode_TXS(cpu);   return;
        case 0xA8: M6502_Opcode_TAY(cpu);   return;
        case 0xAA: M6502_Opcode_TAX(cpu);   return;
        case 0xB8: M6502_Opcode_CLV(cpu);   return;
        case 0xBA: M6502_Opcode_TSX(cpu);   return;
        case 0xC8: M6502_Opcode_INY(cpu);   return;
        case 0xCA: M6502_Opcode_DEX(cpu);   return;
        case 0xD8: M6502_Opcode_CLD(cpu);   return;
        case 0xE8: M6502_Opcode_INX(cpu);   return;
        case 0xF8: M6502_Opcode_SED(cpu);   return;

        case 0xEA: M6502_Opcode_NOP(cpu);   return;

        case 0x1A: M6502_Opcode_NOP(cpu);   return;
        case 0x3A: M6502_Opcode_NOP(cpu);   return;
        case 0x5A: M6502_Opcode_NOP(cpu);   return;
        case 0x7A: M6502_Opcode_NOP(cpu);   return;
        case 0xDA: M6502_Opcode_NOP(cpu);   return;
        case 0xFA: M6502_Opcode_NOP(cpu);   return;
        case 0x80: M6502_Opcode_NOP(cpu);   return;
        case 0x82: M6502_Opcode_NOP(cpu);   return;
        case 0x89: M6502_Opcode_NOP(cpu);   return;
        case 0xC2: M6502_Opcode_NOP(cpu);   return;
        case 0xE2: M6502_Opcode_NOP(cpu);   return;
        case 0x04: M6502_Opcode_NOP(cpu);   return;
        case 0x44: M6502_Opcode_NOP(cpu);   return;
        case 0x64: M6502_Opcode_NOP(cpu);   return;
        case 0x14: M6502_Opcode_NOP(cpu);   return;
        case 0x34: M6502_Opcode_NOP(cpu);   return;
        case 0x54: M6502_Opcode_NOP(cpu);   return;
        case 0x74: M6502_Opcode_NOP(cpu);   return;
        case 0xD4: M6502_Opcode_NOP(cpu);   return;
        case 0xF4: M6502_Opcode_NOP(cpu);   return;
        case 0x0C: M6502_Opcode_NOP(cpu);   return;
        case 0x1C: M6502_Opcode_NOP(cpu);   return;
        case 0x3C: M6502_Opcode_NOP(cpu);   return;
        case 0x5C: M6502_Opcode_NOP(cpu);   return;
        case 0x7C: M6502_Opcode_NOP(cpu);   return;
        case 0xDC: M6502_Opcode_NOP(cpu);   return;
        case 0xFC: M6502_Opcode_NOP(cpu);   return;

        case 0x02: M6502_Opcode_JAM(cpu);   return;
        case 0x12: M6502_Opcode_JAM(cpu);   return;
        case 0x22: M6502_Opcode_JAM(cpu);   return;
        case 0x32: M6502_Opcode_JAM(cpu);   return;
        case 0x42: M6502_Opcode_JAM(cpu);   return;
        case 0x52: M6502_Opcode_JAM(cpu);   return;
        case 0x62: M6502_Opcode_JAM(cpu);   return;
        case 0x72: M6502_Opcode_JAM(cpu);   return;
        case 0x92: M6502_Opcode_JAM(cpu);   return;
        case 0xB2: M6502_Opcode_JAM(cpu);   return;
        case 0xD2: M6502_Opcode_JAM(cpu);   return;
        case 0xF2: M6502_Opcode_JAM(cpu);   return;

        case 0x4B:
        {
            M6502_Address_Immediate(cpu);
            M6502_Opcode_ALR(cpu);
            return;
        }

        case 0x0B:
        case 0x2B:
        {
            M6502_Address_Immediate(cpu);
            M6502_Opcode_ANC(cpu);
            return;
        }


        case 0x8B:
        {
            M6502_Address_Immediate(cpu);
            M6502_Opcode_ANE(cpu);
            return;
        }

        case 0xB6:
        {
            M6502_Address_Immediate(cpu);
            M6502_Opcode_ARR(cpu);
            return;
        }

        case 0xBB:
        {
            M6502_Address_AbsoluteY(cpu);
            M6502_Opcode_LAS(cpu);
            return;
        }

        case 0xAB:
        {
            M6502_Address_Immediate(cpu);
            M6502_Opcode_LXA(cpu);
            return;
        }

        case 0xCB:
        {
            M6502_Address_Immediate(cpu);
            M6502_Opcode_SBX(cpu);
            return;
        }

        case 0x9F:
        {
            M6502_Address_AbsoluteY(cpu);
            M6502_Opcode_SHA(cpu);
            return;
        }

        case 0x93:
        {
            M6502_Address_IndirectY(cpu);
            M6502_Opcode_SHA(cpu);
            return;
        }

        case 0x9C: 
        {
            M6502_Address_AbsoluteX(cpu);
            M6502_Opcode_SHY(cpu);
            return;
        }
        case 0x9E:
        {
            M6502_Address_AbsoluteY(cpu);
            M6502_Opcode_SHX(cpu);
            return;
        }

        case 0x9B:
        {
            M6502_Address_AbsoluteY(cpu);
            M6502_Opcode_TAS(cpu);
            return;
        }

        case 0xEB:
        {
            M6502_Address_Immediate(cpu);
            M6502_Opcode_USBC(cpu);
            return;
        }
    
        default:                            break;
    }

    switch(cpu->opcode & 0x3)
    {
        case 0x01:  M6502_Opcode_Group01(cpu);  break;
        case 0x02:  M6502_Opcode_Group10(cpu);  break;
        case 0x03:  M6502_Opcode_Group11(cpu);  break;
        case 0x00:  M6502_Opcode_Group00(cpu);  break;
        default:                                break;
    }
}

static inline void M6502_Opcode_Group01(M6502_t* cpu)
{
    const uint8_t instruction = (cpu->opcode & 0xE0) >> 5;
    const uint8_t addressMode = (cpu->opcode & 0x1C) >> 2;

    switch(addressMode)
    {
        case 0x00:  M6502_Address_IndirectX(cpu);    break;
        case 0x01:  M6502_Address_ZeroPage(cpu);     break;
        case 0x02:  M6502_Address_Immediate(cpu);    break;
        case 0x03:  M6502_Address_Absolute(cpu);     break;
        case 0x04:  M6502_Address_IndirectY(cpu);    break;
        case 0x05:  M6502_Address_ZeroPageX(cpu);    break;
        case 0x06:  M6502_Address_AbsoluteY(cpu);    break;
        case 0x07:  M6502_Address_AbsoluteX(cpu);    break;
    }

    switch (instruction)
    {
        case 0x00:  M6502_Opcode_ORA(cpu);  break;
        case 0x01:  M6502_Opcode_AND(cpu);  break;
        case 0x02:  M6502_Opcode_EOR(cpu);  break;
        case 0x03:  M6502_Opcode_ADC(cpu);  break;
        case 0x04:  M6502_Opcode_STA(cpu);  break;
        case 0x05:  M6502_Opcode_LDA(cpu);  break;
        case 0x06:  M6502_Opcode_CMP(cpu);  break;
        case 0x07:  M6502_Opcode_SBC(cpu);  break;
    }
}

static inline void M6502_Opcode_Group10(M6502_t* cpu)
{
    const uint8_t instruction = (cpu->opcode & 0xE0) >> 5;
    const uint8_t addressMode = (cpu->opcode & 0x1C) >> 2;

    switch(addressMode)
    {
        case 0x00:  M6502_Address_Immediate(cpu);   break;
        case 0x01:  M6502_Address_ZeroPage(cpu);    break;
        case 0x02:  M6502_Address_Accumulator(cpu); break;
        case 0x03:  M6502_Address_Absolute(cpu);    break;
        case 0x05:  
        {
            if(instruction == 4 || instruction == 5)
            {
                M6502_Address_ZeroPageY(cpu);
                break;
            }
            M6502_Address_ZeroPageX(cpu);
            break;
        }
        case 0x07:
        {
            if(instruction == 5)
            {
                M6502_Address_AbsoluteY(cpu);
                break;
            }
            M6502_Address_AbsoluteX(cpu);
            break;
        }
    }

    switch (instruction)
    {
        case 0x00:  M6502_Opcode_ASL(cpu);  break;
        case 0x01:  M6502_Opcode_ROL(cpu);  break;
        case 0x02:  M6502_Opcode_LSR(cpu);  break;
        case 0x03:  M6502_Opcode_ROR(cpu);  break;
        case 0x04:  M6502_Opcode_STX(cpu);  break;
        case 0x05:  M6502_Opcode_LDX(cpu);  break;
        case 0x06:  M6502_Opcode_DEC(cpu);  break;
        case 0x07:  M6502_Opcode_INC(cpu);  break;
    }
}

static inline void M6502_Opcode_Group11(M6502_t* cpu)
{
    const uint8_t instruction = (cpu->opcode & 0xE0) >> 5;
    const uint8_t addressMode = (cpu->opcode & 0x1C) >> 2;

    switch(addressMode)
    {
        case 0x00:  M6502_Address_IndirectX(cpu);   break;
        case 0x01:  M6502_Address_ZeroPage(cpu);    break;
        case 0x02:  M6502_Address_Immediate(cpu);   break;
        case 0x03:  M6502_Address_Absolute(cpu);    break;
        case 0x04:  M6502_Address_IndirectY(cpu);   break;
        case 0x05:  M6502_Address_ZeroPageX(cpu);   break;
        case 0x06:  M6502_Address_AbsoluteY(cpu);   break;
        case 0x07:  M6502_Address_AbsoluteX(cpu);   break;
    }

    switch (instruction)
    {
        case 0x00:  M6502_Opcode_SLO(cpu);  break;
        case 0x01:  M6502_Opcode_RLA(cpu);  break;
        case 0x02:  M6502_Opcode_SRE(cpu);  break;
        case 0x03:  M6502_Opcode_RRA(cpu);  break;
        case 0x04:  M6502_Opcode_SAX(cpu);  break;
        case 0x05:  M6502_Opcode_LAX(cpu);  break;
        case 0x06:  M6502_Opcode_DCP(cpu);  break;
        case 0x07:  M6502_Opcode_ISC(cpu);  break;
    }
}

static inline void M6502_Opcode_Group00(M6502_t* cpu)
{
    const uint8_t instruction = (cpu->opcode & 0xE0) >> 5;
    const uint8_t addressMode = (cpu->opcode & 0x1C) >> 2;

    switch(addressMode)
    {
        case 0x00:  M6502_Address_Immediate(cpu);       break;
        case 0x01:  M6502_Address_ZeroPage(cpu);        break;
        case 0x03: 
        {
            if(instruction == 0x03)
            {
                M6502_Address_Indirect(cpu);
                break;
            }
            M6502_Address_Absolute(cpu);
            break;
        }
        case 0x04:  M6502_Opcode_Group00_Branch(cpu);   return;
        case 0x05:  M6502_Address_ZeroPageX(cpu);       break;
        case 0x07:  M6502_Address_AbsoluteX(cpu);       break;
    }

    switch (instruction)
    {
        case 0x01:  M6502_Opcode_BIT(cpu);  break;
        case 0x02:  M6502_Opcode_JMP(cpu);  break;
        case 0x03:  M6502_Opcode_JMP(cpu);  break;
        case 0x04:  M6502_Opcode_STY(cpu);  break;
        case 0x05:  M6502_Opcode_LDY(cpu);  break;
        case 0x06:  M6502_Opcode_CPY(cpu);  break;
        case 0x07:  M6502_Opcode_CPX(cpu);  break;
    }
}

static inline void M6502_Opcode_Group00_Branch(M6502_t* cpu)
{
    M6502_Address_Relative(cpu);

    const uint8_t branch   = (cpu->opcode >> 5);

    switch (branch)
    {
        case 0x00: M6502_Opcode_BPL(cpu); break;
        case 0x01: M6502_Opcode_BMI(cpu); break;
        case 0x02: M6502_Opcode_BVC(cpu); break;
        case 0x03: M6502_Opcode_BVS(cpu); break;
        case 0x04: M6502_Opcode_BCC(cpu); break;
        case 0x05: M6502_Opcode_BCS(cpu); break;
        case 0x06: M6502_Opcode_BNE(cpu); break;
        case 0x07: M6502_Opcode_BEQ(cpu); break;
    
        default: break;
    }
}

static inline void M6502_Opcode_ADC(M6502_t* cpu)
{
    uint16_t temporary = (uint16_t)cpu->accumulator + cpu->target;
    temporary += (uint16_t)(cpu->statusRegister & M6502_FLAG_CARRY);

#ifndef M6502_NES_CPU
    if (M6502_GetFlag(cpu, M6502_FLAG_DECIMAL))
    {
        uint16_t result_bcd = 0x00;

        result_bcd = (cpu->accumulator & 0x0F) + (cpu->target & 0x0F);
        result_bcd += (uint16_t)(cpu->statusRegister & M6502_FLAG_CARRY);

        if(result_bcd >= 0xA)
        {
            result_bcd = ((result_bcd + 0x06) & 0x0F) + 0x10;
        }
        
        result_bcd += (cpu->accumulator & 0xF0) + (cpu->target & 0xF0);
        
        M6502_SetFlag(cpu, M6502_FLAG_NEGATIVE, (uint8_t)(result_bcd & M6502_FLAG_NEGATIVE));
        
        if(result_bcd >= 0xA0)
        {
            result_bcd += 0x60;
        }

        M6502_SetFlag(cpu, M6502_FLAG_CARRY, result_bcd >= 0x100);
        M6502_SetFlag(cpu, M6502_FLAG_OVERFLOW, result_bcd & 0xFF80);

        M6502_ZeroTest(cpu, temporary);

        cpu->accumulator = (uint8_t)(result_bcd & 0x00FF);
    }
    else
#endif
    {
        M6502_CarryTest(cpu, temporary);
        M6502_ZeroTest(cpu, temporary);
        M6502_NegativeTest(cpu, temporary);
        M6502_OverFlowTest(cpu, cpu->target, temporary);

        cpu->accumulator = (uint8_t)(temporary & 0x00FF);
    }
}

static inline void M6502_Opcode_AND(M6502_t* cpu)
{
    const uint16_t temporary = (uint16_t)cpu->accumulator & cpu->target;

    cpu->accumulator = (uint8_t)(temporary & 0x00FF);

    M6502_ZeroTest(cpu, temporary);
    M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_ASL(M6502_t* cpu)
{
    const uint16_t temporary = cpu->target << 1;
    
    M6502_Util_WriteResult(cpu, temporary);

    M6502_CarryTest(cpu, temporary);
    M6502_ZeroTest(cpu, temporary);
    M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_BCC(M6502_t* cpu)
{
    if((cpu->statusRegister & M6502_FLAG_CARRY) == 0)
    {
        M6502_Util_BRANCH(cpu);
    }
}

static inline void M6502_Opcode_BCS(M6502_t* cpu)
{
    if((cpu->statusRegister & M6502_FLAG_CARRY) == M6502_FLAG_CARRY)
    {
        M6502_Util_BRANCH(cpu);
    }
}

static inline void M6502_Opcode_BEQ(M6502_t* cpu)
{
    if((cpu->statusRegister & M6502_FLAG_ZERO) == M6502_FLAG_ZERO)
    {
        M6502_Util_BRANCH(cpu);
    }
}

static inline void M6502_Opcode_BIT(M6502_t* cpu)
{
    const uint16_t temporary = (uint16_t)cpu->accumulator & cpu->target;

    M6502_ZeroTest(cpu, temporary);
    cpu->statusRegister = (cpu->statusRegister & 0x3F) | (uint8_t)(cpu->target & 0xC0);
}

static inline void M6502_Opcode_BMI(M6502_t* cpu)
{
    if((cpu->statusRegister & M6502_FLAG_NEGATIVE) == M6502_FLAG_NEGATIVE)
    {
        M6502_Util_BRANCH(cpu);
    }
}

static inline void M6502_Opcode_BNE(M6502_t* cpu)
{
    if((cpu->statusRegister & M6502_FLAG_ZERO) == 0)
    {
        M6502_Util_BRANCH(cpu);
    }
}

static inline void M6502_Opcode_BPL(M6502_t* cpu)
{
    if((cpu->statusRegister & M6502_FLAG_NEGATIVE) == 0)
    {
        M6502_Util_BRANCH(cpu);
    }
}

static inline void M6502_Opcode_BRK(M6502_t* cpu)
{
    M6502_PushWord(cpu, ++cpu->programCounter);
    M6502_PushByte(cpu, cpu->statusRegister | M6502_FLAG_BREAK);
    cpu->statusRegister |= M6502_FLAG_INTERRUPT;

    cpu->programCounter = M6502_ReadMemoryWord(M6502_IRQVECTOR_ADDRESS);
}

static inline void M6502_Opcode_BVC(M6502_t* cpu)
{
    if((cpu->statusRegister & M6502_FLAG_OVERFLOW) == 0)
    {
        M6502_Util_BRANCH(cpu);
    }
}

static inline void M6502_Opcode_BVS(M6502_t* cpu)
{
    if((cpu->statusRegister & M6502_FLAG_OVERFLOW) == M6502_FLAG_OVERFLOW)
    {
        M6502_Util_BRANCH(cpu);
    }
}

static inline void M6502_Opcode_CLC(M6502_t* cpu)
{
    cpu->statusRegister &= ~M6502_FLAG_CARRY;
}

static inline void M6502_Opcode_CLD(M6502_t* cpu)
{
    cpu->statusRegister &= ~M6502_FLAG_DECIMAL;
}

static inline void M6502_Opcode_CLI(M6502_t* cpu)
{
    cpu->statusRegister &= ~M6502_FLAG_INTERRUPT;
}

static inline void M6502_Opcode_CLV(M6502_t* cpu)
{
    cpu->statusRegister &= ~M6502_FLAG_OVERFLOW;
}

static inline void M6502_Opcode_CMP(M6502_t* cpu)
{
    const uint16_t temporary = (uint16_t)cpu->accumulator - cpu->target;

    M6502_SetFlag(cpu, M6502_FLAG_CARRY, cpu->accumulator >= (uint8_t)(cpu->target & 0x00FF));
    M6502_SetFlag(cpu, M6502_FLAG_ZERO, cpu->accumulator == (uint8_t)(cpu->target & 0x00FF));
    M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_CPX(M6502_t* cpu)
{
    const uint16_t temporary = (uint16_t)cpu->xRegister - cpu->target;
    
    M6502_SetFlag(cpu, M6502_FLAG_CARRY, cpu->xRegister >= (uint8_t)(cpu->target & 0x00FF));
    M6502_SetFlag(cpu, M6502_FLAG_ZERO, cpu->xRegister == (uint8_t)(cpu->target & 0x00FF));
    M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_CPY(M6502_t* cpu)
{
    const uint16_t temporary = (uint16_t)cpu->yRegister - cpu->target;
    
    M6502_SetFlag(cpu, M6502_FLAG_CARRY, cpu->yRegister >= (uint8_t)(cpu->target & 0x00FF));
    M6502_SetFlag(cpu, M6502_FLAG_ZERO, cpu->yRegister == (uint8_t)(cpu->target & 0x00FF));
    M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_DEC(M6502_t* cpu)
{
    const uint16_t temporary = cpu->target - 1;

    M6502_Util_WriteResult(cpu, temporary);

    M6502_ZeroTest(cpu, temporary);
	M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_DEX(M6502_t* cpu)
{
    cpu->xRegister = cpu->xRegister - 1;
    
    M6502_ZeroTest(cpu, cpu->xRegister);
	M6502_NegativeTest(cpu, cpu->xRegister);
}

static inline void M6502_Opcode_DEY(M6502_t* cpu)
{
    cpu->yRegister = cpu->yRegister - 1;
    
    M6502_ZeroTest(cpu, cpu->yRegister);
	M6502_NegativeTest(cpu, cpu->yRegister);
}

static inline void M6502_Opcode_EOR(M6502_t* cpu)
{
    const uint16_t temporary = (uint16_t)cpu->accumulator ^ cpu->target;

    cpu->accumulator = (uint8_t)(temporary & 0x00FF);

	M6502_ZeroTest(cpu, temporary);
	M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_INC(M6502_t* cpu)
{
    const uint16_t temporary = cpu->target + 1;

	M6502_Util_WriteResult(cpu, temporary);
	
    M6502_ZeroTest(cpu, temporary);
	M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_INX(M6502_t* cpu)
{
    cpu->xRegister = cpu->xRegister + 1;
    
    M6502_ZeroTest(cpu, cpu->xRegister);
	M6502_NegativeTest(cpu, cpu->xRegister);
}

static inline void M6502_Opcode_INY(M6502_t* cpu)
{
    cpu->yRegister = cpu->yRegister + 1;
    
    M6502_ZeroTest(cpu, cpu->yRegister);
	M6502_NegativeTest(cpu, cpu->yRegister);
}

static inline void M6502_Opcode_JMP(M6502_t* cpu)
{
    cpu->programCounter = cpu->address;
}

static inline void M6502_Opcode_JSR(M6502_t* cpu)
{
    M6502_Address_Absolute(cpu);
    M6502_PushWord(cpu, cpu->programCounter - 1);

    cpu->programCounter = cpu->address;
}

static inline void M6502_Opcode_LDA(M6502_t* cpu)
{
    cpu->accumulator = (uint8_t)(cpu->target & 0x00FF);

	M6502_ZeroTest(cpu, cpu->accumulator);
	M6502_NegativeTest(cpu, cpu->accumulator);
}

static inline void M6502_Opcode_LDX(M6502_t* cpu)
{
    cpu->xRegister = (uint8_t)(cpu->target & 0x00FF);

	M6502_ZeroTest(cpu, cpu->xRegister);
	M6502_NegativeTest(cpu, cpu->xRegister);
}

static inline void M6502_Opcode_LDY(M6502_t* cpu)
{
    cpu->yRegister = (uint8_t)(cpu->target & 0x00FF);

	M6502_ZeroTest(cpu, cpu->yRegister);
	M6502_NegativeTest(cpu, cpu->yRegister);
}

static inline void M6502_Opcode_LSR(M6502_t* cpu)
{
	const uint16_t temporary = cpu->target >> 1;

    M6502_Util_WriteResult(cpu, temporary);

    M6502_SetFlag(cpu, M6502_FLAG_CARRY, (uint8_t)(cpu->target & 0x1));
	M6502_ZeroTest(cpu, temporary);
    M6502_SetFlag(cpu, M6502_FLAG_NEGATIVE, 0);
}

static inline void M6502_Opcode_NOP(M6502_t* cpu)
{
    /* Nothing */
}

static inline void M6502_Opcode_ORA(M6502_t* cpu)
{
    const uint16_t temporary = (uint16_t)cpu->accumulator | cpu->target;

    cpu->accumulator = (uint8_t)(temporary & 0x00FF);

    M6502_ZeroTest(cpu, temporary);
    M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_PHA(M6502_t* cpu)
{
    M6502_PushByte(cpu, cpu->accumulator);
}

static inline void M6502_Opcode_PHP(M6502_t* cpu)
{
	M6502_PushByte(cpu, cpu->statusRegister | M6502_FLAG_BREAK);
}

static inline void M6502_Opcode_PLA(M6502_t* cpu)
{
	cpu->accumulator = M6502_PullByte(cpu);

    M6502_ZeroTest(cpu, cpu->accumulator);
    M6502_NegativeTest(cpu, cpu->accumulator);
}

static inline void M6502_Opcode_PLP(M6502_t* cpu)
{
	cpu->statusRegister = M6502_PullByte(cpu) | M6502_FLAG_UNUSED;
}

static inline void M6502_Opcode_ROL(M6502_t* cpu)
{
    uint16_t temporary = cpu->target << 1;
    temporary |= (cpu->statusRegister & M6502_FLAG_CARRY);
	
    M6502_Util_WriteResult(cpu, temporary);

    M6502_CarryTest(cpu, temporary);
	M6502_ZeroTest(cpu, temporary);
	M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_ROR(M6502_t* cpu)
{
    uint16_t temporary = cpu->target >> 1;
    temporary |= (cpu->statusRegister & M6502_FLAG_CARRY) << 7;

    M6502_Util_WriteResult(cpu, temporary);

    M6502_SetFlag(cpu, M6502_FLAG_CARRY, (uint8_t)(cpu->target & 0x1));
	M6502_ZeroTest(cpu, temporary);
	M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_RTI(M6502_t* cpu)
{
    cpu->statusRegister = M6502_PullByte(cpu);
    cpu->programCounter = M6502_PullWord(cpu);
}

static inline void M6502_Opcode_RTS(M6502_t* cpu)
{
    cpu->programCounter = M6502_PullWord(cpu) + 1;
}

static inline void M6502_Opcode_SBC(M6502_t* cpu)
{
    uint16_t temporary = (uint16_t)cpu->accumulator + (cpu->target ^ 0x00FF);
    temporary += (uint16_t)M6502_GetFlag(cpu, M6502_FLAG_CARRY);

    M6502_ZeroTest(cpu, temporary);
    M6502_CarryTest(cpu, temporary);
    M6502_NegativeTest(cpu, temporary);
    M6502_OverFlowTest(cpu, (cpu->target ^ 0x00FF), temporary);

#ifndef M6502_NES_CPU
    if (M6502_GetFlag(cpu, M6502_FLAG_DECIMAL))
    {
        uint16_t temporaryC = (uint16_t)(cpu->statusRegister & M6502_FLAG_CARRY);
        uint16_t result_bcd = 0x0000;

        result_bcd = (cpu->accumulator & 0x0F) - (cpu->target & 0x0F) + temporaryC -1;

        if(result_bcd & 0x8000)
        {
            result_bcd = ((result_bcd - 0x06) & 0x0F) - 0x10;
        }

        result_bcd = (cpu->accumulator & 0xF0) - (cpu->target & 0xF0) + result_bcd;

        if(result_bcd & 0x8000)
        {
            result_bcd = result_bcd - 0x60;
        }

        cpu->accumulator = (uint8_t)(result_bcd & 0x00FF);
    }
    else
#endif
    { 
        cpu->accumulator = (uint8_t)(temporary & 0x00FF);
    }
}

static inline void M6502_Opcode_SEC(M6502_t* cpu)
{
    cpu->statusRegister |= M6502_FLAG_CARRY;
}

static inline void M6502_Opcode_SED(M6502_t* cpu)
{
    cpu->statusRegister |= M6502_FLAG_DECIMAL;
}

static inline void M6502_Opcode_SEI(M6502_t* cpu)
{
    cpu->statusRegister |= M6502_FLAG_INTERRUPT;
}

static inline void M6502_Opcode_STA(M6502_t* cpu)
{
    M6502_Util_WriteResult(cpu, cpu->accumulator);
}

static inline void M6502_Opcode_STX(M6502_t* cpu)
{
    M6502_Util_WriteResult(cpu, cpu->xRegister);
}

static inline void M6502_Opcode_STY(M6502_t* cpu)
{
    M6502_Util_WriteResult(cpu, cpu->yRegister);
}

static inline void M6502_Opcode_TAX(M6502_t* cpu)
{
    cpu->xRegister = cpu->accumulator;

    M6502_ZeroTest(cpu, cpu->xRegister);
    M6502_NegativeTest(cpu, cpu->xRegister);
}

static inline void M6502_Opcode_TAY(M6502_t* cpu)
{
    cpu->yRegister = cpu->accumulator;

    M6502_ZeroTest(cpu, cpu->yRegister);
    M6502_NegativeTest(cpu, cpu->yRegister);
}

static inline void M6502_Opcode_TSX(M6502_t* cpu)
{
    cpu->xRegister = cpu->stackPointer;

    M6502_ZeroTest(cpu, cpu->xRegister);
    M6502_NegativeTest(cpu, cpu->xRegister);
}

static inline void M6502_Opcode_TXA(M6502_t* cpu)
{
    cpu->accumulator = cpu->xRegister;
    
    M6502_ZeroTest(cpu, cpu->accumulator);
    M6502_NegativeTest(cpu, cpu->accumulator);
}

static inline void M6502_Opcode_TXS(M6502_t* cpu)
{
    cpu->stackPointer = cpu->xRegister;
}

static inline void M6502_Opcode_TYA(M6502_t* cpu)
{
    cpu->accumulator = cpu->yRegister;
    
    M6502_ZeroTest(cpu, cpu->accumulator);
    M6502_NegativeTest(cpu, cpu->accumulator);
}

static inline void M6502_Opcode_ALR(M6502_t* cpu)
{
    uint16_t temporary = (uint16_t)cpu->accumulator & cpu->target;

    M6502_SetFlag(cpu, M6502_FLAG_CARRY, (uint8_t)(temporary & 0x1));
    
    temporary = (temporary >> 1);

    M6502_Util_WriteResult(cpu, temporary);

	M6502_ZeroTest(cpu, temporary);
	M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_ANC(M6502_t* cpu)
{
    const uint16_t temporary = (uint16_t)cpu->accumulator & cpu->target;

    cpu->accumulator = (uint8_t)(temporary & 0x00FF);

    M6502_CarryTest(cpu, temporary);
    M6502_ZeroTest(cpu, temporary);
    M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_ANE(M6502_t* cpu)
{
    uint16_t temporary = (uint16_t)(cpu->accumulator | M6502_MAGIC_CONSTANT);
    temporary &= (uint16_t)cpu->xRegister;
    temporary &= cpu->target;

    cpu->accumulator = (uint8_t)(temporary & 0x00FF);

    M6502_ZeroTest(cpu, temporary);
    M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_ARR(M6502_t* cpu)
{
    uint16_t temporary = (uint16_t)cpu->accumulator & cpu->target;
    temporary = temporary >> 1;
    temporary |= (cpu->statusRegister & M6502_FLAG_CARRY) << 7;

    cpu->accumulator = (uint8_t)(temporary & 0x00FF);

    const uint8_t bit5 = ((cpu->accumulator >> 5) & 1);
    const uint8_t bit6 = ((cpu->accumulator >> 6) & 1);

    if(bit5 == 1 && bit6 == 1)
    {
        cpu->statusRegister |= M6502_FLAG_CARRY;
        cpu->statusRegister &= ~M6502_FLAG_OVERFLOW;
    }
    else if(bit5 == 0 && bit6 == 0)
    {
        cpu->statusRegister &= ~M6502_FLAG_CARRY;        
        cpu->statusRegister &= ~M6502_FLAG_OVERFLOW;        
    }
    else if(bit5 == 1 && bit6 == 0)
    {
        cpu->statusRegister &= ~M6502_FLAG_CARRY;
        cpu->statusRegister |= M6502_FLAG_OVERFLOW;
    }
    else if(bit5 == 0 && bit6 == 1)
    {
        cpu->statusRegister |= M6502_FLAG_CARRY;
        cpu->statusRegister |= M6502_FLAG_OVERFLOW;
    }

	M6502_ZeroTest(cpu, temporary);
	M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_DCP(M6502_t* cpu)
{
    const uint16_t temporary = cpu->target - 1;
    const uint16_t compare = (uint16_t)cpu->accumulator - temporary;

    M6502_WriteMemoryByte(cpu->address, (uint8_t)(temporary & 0x00FF));

    M6502_SetFlag(cpu, M6502_FLAG_CARRY, cpu->accumulator >= (uint8_t)(temporary & 0x00FF));
    M6502_SetFlag(cpu, M6502_FLAG_ZERO, cpu->accumulator == (uint8_t)(temporary & 0x00FF));
    M6502_NegativeTest(cpu, compare);
}

static inline void M6502_Opcode_ISC(M6502_t* cpu)
{
    const uint16_t temporary = ++cpu->target;

    M6502_WriteMemoryByte(cpu->address, (uint8_t)(temporary & 0x00FF));

    M6502_Opcode_SBC(cpu);
}

static inline void M6502_Opcode_LAS(M6502_t* cpu)
{
    const uint8_t temporary = cpu->target & cpu->stackPointer;

    cpu->accumulator = temporary;
    cpu->xRegister = temporary;
    cpu->stackPointer = temporary;

    M6502_ZeroTest(cpu, (uint16_t)(temporary));
    M6502_NegativeTest(cpu, (uint16_t)(temporary));
}

static inline void M6502_Opcode_LAX(M6502_t* cpu)
{
    cpu->accumulator = cpu->target;
    cpu->xRegister = cpu->target;

    M6502_ZeroTest(cpu, cpu->xRegister);
    M6502_NegativeTest(cpu, cpu->xRegister);
}

static inline void M6502_Opcode_LXA(M6502_t* cpu)
{
    uint16_t temporary = ((uint16_t)cpu->accumulator | M6502_MAGIC_CONSTANT);
    temporary &= cpu->target;

    cpu->accumulator = (uint8_t)(temporary & 0x00FF);
    cpu->xRegister = (uint8_t)(temporary & 0x00FF);

    M6502_ZeroTest(cpu, cpu->xRegister);
    M6502_NegativeTest(cpu, cpu->xRegister);
}

static inline void M6502_Opcode_RLA(M6502_t* cpu)
{
    uint16_t temporary = cpu->target >> 1;
    temporary |= (cpu->statusRegister & M6502_FLAG_CARRY) << 7;

    M6502_WriteMemoryByte(cpu->address, (uint8_t)(temporary & 0x00FF));

    M6502_SetFlag(cpu, M6502_FLAG_CARRY, (uint8_t)(cpu->target & 0x1));

    M6502_Opcode_AND(cpu);
}

static inline void M6502_Opcode_RRA(M6502_t* cpu)
{
    uint16_t temporary = cpu->target >> 1;
    temporary |= (cpu->statusRegister & M6502_FLAG_CARRY) << 7;

    M6502_WriteMemoryByte(cpu->address, (uint8_t)(temporary & 0x00FF));

    M6502_Opcode_ADC(cpu);
}

static inline void M6502_Opcode_SAX(M6502_t* cpu)
{
    M6502_WriteMemoryByte(cpu->address, cpu->accumulator & cpu->xRegister);
}

static inline void M6502_Opcode_SBX(M6502_t* cpu)
{
    uint16_t temporary = (uint16_t)cpu->accumulator & (uint16_t)cpu->xRegister;
    temporary -= cpu->target;

    cpu->xRegister = (uint8_t)(temporary & 0x00FF);

    M6502_SetFlag(cpu, M6502_FLAG_CARRY, cpu->accumulator >= cpu->xRegister);
    M6502_SetFlag(cpu, M6502_FLAG_ZERO, cpu->accumulator == cpu->xRegister);
    M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_SHA(M6502_t* cpu)
{
    uint16_t temporary = (uint16_t)cpu->xRegister & (uint16_t)cpu->accumulator;
    temporary &= ((cpu->address >> 7) & 1) + 1;

    M6502_WriteMemoryByte(cpu->address, (uint8_t)(temporary & 0x00FF));
}

static inline void M6502_Opcode_SHX(M6502_t* cpu)
{
    uint16_t temporary = (uint16_t)cpu->xRegister & ((cpu->address >> 7) & 1);
    temporary += 1;

    M6502_WriteMemoryByte(cpu->address, (uint8_t)(temporary & 0x00FF));
}

static inline void M6502_Opcode_SHY(M6502_t* cpu)
{
    uint16_t temporary = (uint16_t)cpu->yRegister & ((cpu->address >> 7) & 1);
    temporary += 1;

    M6502_WriteMemoryByte(cpu->address, (uint8_t)(temporary & 0x00FF));
}

static inline void M6502_Opcode_SLO(M6502_t* cpu)
{
    uint16_t temporary = (cpu->target << 1);

    M6502_WriteMemoryByte(cpu->address, (uint8_t)(temporary & 0x00FF));
    M6502_CarryTest(cpu, temporary);

    temporary |= (uint16_t)cpu->accumulator;

    cpu->accumulator = (uint8_t)(temporary & 0x00FF);

    M6502_ZeroTest(cpu, temporary);
    M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_SRE(M6502_t* cpu)
{
    uint16_t temporary = (cpu->target >> 1);

    M6502_WriteMemoryByte(cpu->address, (uint8_t)(temporary & 0x00FF));
    M6502_CarryTest(cpu, temporary);

    temporary ^= (uint16_t)cpu->accumulator;

    cpu->accumulator = (uint8_t)(temporary & 0x00FF);

    M6502_ZeroTest(cpu, temporary);
    M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_TAS(M6502_t* cpu)
{
    cpu->stackPointer = cpu->xRegister & cpu->accumulator;

    uint16_t temporary = cpu->stackPointer & ((cpu->address >> 7) & 1);
    temporary += 1;

    M6502_WriteMemoryByte(cpu->address, (uint8_t)(temporary & 0x00FF));
}

static inline void M6502_Opcode_USBC(M6502_t* cpu)
{
    M6502_Opcode_SBC(cpu);
    M6502_Opcode_NOP(cpu);
}

static inline void M6502_Opcode_JAM(M6502_t* cpu)
{
    while(1)
    {
        cpu->target = 0xFF;
    }
}