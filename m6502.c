#include "m6502.h"

static const uint16_t M6502_NMIVECTOR_ADDRESS   = 0xFFFAu;
static const uint16_t M6502_RESETVECTOR_ADDRESS = 0xFFFCu;
static const uint16_t M6502_IRQVECTOR_ADDRESS   = 0xFFFEu;

static const uint16_t M6502_STACK_ADDRESS       = 0x0100u;
static const uint16_t M6502_STACK_START_ADDRESS = 0x00FDu;

static const uint8_t M6502_FLAG_CARRY       = 0x01u;
static const uint8_t M6502_FLAG_ZERO        = 0x02u;
static const uint8_t M6502_FLAG_INTERRUPT   = 0x04u;
static const uint8_t M6502_FLAG_DECIMAL     = 0x08u;
static const uint8_t M6502_FLAG_BREAK       = 0x10u;
static const uint8_t M6502_FLAG_UNUSED      = 0x20u;
static const uint8_t M6502_FLAG_OVERFLOW    = 0x40u;
static const uint8_t M6502_FLAG_NEGATIVE    = 0x80u;

static const uint8_t M6502_INTERRUPT_NMI    = 0xF0u;
static const uint8_t M6502_INTERRUPT_IRQ    = 0x0Fu;

static const uint8_t M6502_MAGIC_CONSTANT   = 0x00u;

static const uint8_t M6502_OPCODE_CYCLES[0x100] = {
    7u, 6u, 2u, 8u, 3u, 3u, 5u, 5u, 3u, 2u, 2u, 2u, 4u, 4u, 6u, 6u,
    2u, 5u, 2u, 8u, 4u, 4u, 6u, 6u, 2u, 4u, 2u, 7u, 4u, 4u, 7u, 7u,
    6u, 6u, 2u, 8u, 3u, 3u, 5u, 5u, 4u, 2u, 2u, 2u, 4u, 4u, 6u, 6u,
    2u, 5u, 2u, 8u, 4u, 4u, 6u, 6u, 2u, 4u, 2u, 7u, 4u, 4u, 7u, 7u,
    6u, 6u, 2u, 8u, 3u, 3u, 5u, 5u, 3u, 2u, 2u, 2u, 3u, 4u, 6u, 6u,
    2u, 5u, 2u, 8u, 4u, 4u, 6u, 6u, 2u, 4u, 2u, 7u, 4u, 4u, 7u, 7u,
    6u, 6u, 2u, 8u, 3u, 3u, 5u, 5u, 4u, 2u, 2u, 2u, 5u, 4u, 6u, 6u,
    2u, 5u, 2u, 8u, 4u, 4u, 6u, 6u, 2u, 4u, 2u, 7u, 4u, 4u, 7u, 7u,
    2u, 6u, 2u, 6u, 3u, 3u, 3u, 3u, 2u, 2u, 2u, 2u, 4u, 4u, 4u, 4u,
    2u, 6u, 2u, 6u, 4u, 4u, 4u, 4u, 2u, 5u, 2u, 5u, 5u, 5u, 5u, 5u,
    2u, 6u, 2u, 6u, 3u, 3u, 3u, 3u, 2u, 2u, 2u, 2u, 4u, 4u, 4u, 4u,
    2u, 5u, 2u, 5u, 4u, 4u, 4u, 4u, 2u, 4u, 2u, 4u, 4u, 4u, 4u, 4u,
    2u, 6u, 2u, 8u, 3u, 3u, 5u, 5u, 2u, 2u, 2u, 2u, 4u, 4u, 6u, 6u,
    2u, 5u, 2u, 8u, 4u, 4u, 6u, 6u, 2u, 4u, 2u, 7u, 4u, 4u, 7u, 7u,
    2u, 6u, 2u, 8u, 3u, 3u, 5u, 5u, 2u, 2u, 2u, 2u, 4u, 4u, 6u, 6u,
    2u, 5u, 2u, 8u, 4u, 4u, 6u, 6u, 2u, 4u, 2u, 7u, 4u, 4u, 7u, 7u 
};

static inline uint8_t   M6502_ReadMemoryByte(const uint16_t address);
static inline uint16_t  M6502_ReadMemoryWord(const uint16_t address);
static inline void      M6502_WriteMemoryByte(const uint16_t address, const uint8_t value);
static inline void      M6502_WriteMemoryWord(const uint16_t address, const uint16_t value);

static inline void      M6502_SetFlag(M6502_t *cpu, const uint8_t flag, const uint8_t value);
static inline uint8_t   M6502_GetFlag(M6502_t *cpu, const uint8_t flag);

static inline void      M6502_PushByte(M6502_t *cpu, const uint8_t value);
static inline void      M6502_PushWord(M6502_t *cpu, const uint16_t value);
static inline uint8_t   M6502_PullByte(M6502_t *cpu);
static inline uint16_t  M6502_PullWord(M6502_t *cpu);

static inline void M6502_CarryTest(M6502_t *cpu, const uint16_t value);
static inline void M6502_ZeroTest(M6502_t *cpu, const uint16_t value);
static inline void M6502_OverFlowTest(M6502_t *cpu, const uint16_t value, const uint16_t result);
static inline void M6502_NegativeTest(M6502_t *cpu, const uint16_t value);

static inline void M6502_Address_Accumulator(M6502_t *cpu);
static inline void M6502_Address_Immediate(M6502_t *cpu);
static inline void M6502_Address_Relative(M6502_t *cpu);
static inline void M6502_Address_Absolute(M6502_t *cpu);
static inline void M6502_Address_AbsoluteX(M6502_t *cpu);
static inline void M6502_Address_AbsoluteY(M6502_t *cpu);
static inline void M6502_Address_ZeroPage(M6502_t *cpu);
static inline void M6502_Address_ZeroPageX(M6502_t *cpu);
static inline void M6502_Address_ZeroPageY(M6502_t *cpu);
static inline void M6502_Address_Indirect(M6502_t *cpu);
static inline void M6502_Address_IndirectX(M6502_t *cpu);
static inline void M6502_Address_IndirectY(M6502_t *cpu);

static inline void M6502_Util_WriteResult(M6502_t *cpu, const uint16_t result);
static inline void M6502_Util_Branch(M6502_t *cpu);
static inline void M6502_Util_Interrupt(M6502_t *cpu);

static inline void M6502_Opcode_Group01(M6502_t *cpu);
static inline void M6502_Opcode_Group10(M6502_t *cpu);
static inline void M6502_Opcode_Group11(M6502_t *cpu);
static inline void M6502_Opcode_Group00(M6502_t *cpu);
static inline void M6502_Opcode_Group00_Branch(M6502_t *cpu);

static inline void M6502_Opcode_ADC(M6502_t *cpu);
static inline void M6502_Opcode_AND(M6502_t *cpu);
static inline void M6502_Opcode_ASL(M6502_t *cpu);
static inline void M6502_Opcode_BCC(M6502_t *cpu);
static inline void M6502_Opcode_BCS(M6502_t *cpu);
static inline void M6502_Opcode_BEQ(M6502_t *cpu);
static inline void M6502_Opcode_BIT(M6502_t *cpu);
static inline void M6502_Opcode_BMI(M6502_t *cpu);
static inline void M6502_Opcode_BNE(M6502_t *cpu);
static inline void M6502_Opcode_BPL(M6502_t *cpu);
static inline void M6502_Opcode_BRK(M6502_t *cpu);
static inline void M6502_Opcode_BVC(M6502_t *cpu);
static inline void M6502_Opcode_BVS(M6502_t *cpu);
static inline void M6502_Opcode_CLC(M6502_t *cpu);
static inline void M6502_Opcode_CLD(M6502_t *cpu);
static inline void M6502_Opcode_CLI(M6502_t *cpu);
static inline void M6502_Opcode_CLV(M6502_t *cpu);
static inline void M6502_Opcode_CMP(M6502_t *cpu);
static inline void M6502_Opcode_CPX(M6502_t *cpu);
static inline void M6502_Opcode_CPY(M6502_t *cpu);
static inline void M6502_Opcode_DEC(M6502_t *cpu);
static inline void M6502_Opcode_DEX(M6502_t *cpu);
static inline void M6502_Opcode_DEY(M6502_t *cpu);
static inline void M6502_Opcode_EOR(M6502_t *cpu);
static inline void M6502_Opcode_INC(M6502_t *cpu);
static inline void M6502_Opcode_INX(M6502_t *cpu);
static inline void M6502_Opcode_INY(M6502_t *cpu);
static inline void M6502_Opcode_JMP(M6502_t *cpu);
static inline void M6502_Opcode_JSR(M6502_t *cpu);
static inline void M6502_Opcode_LDA(M6502_t *cpu);
static inline void M6502_Opcode_LDX(M6502_t *cpu);
static inline void M6502_Opcode_LDY(M6502_t *cpu);
static inline void M6502_Opcode_LSR(M6502_t *cpu);
static inline void M6502_Opcode_NOP(M6502_t *cpu);
static inline void M6502_Opcode_ORA(M6502_t *cpu);
static inline void M6502_Opcode_PHA(M6502_t *cpu);
static inline void M6502_Opcode_PHP(M6502_t *cpu);
static inline void M6502_Opcode_PLA(M6502_t *cpu);
static inline void M6502_Opcode_PLP(M6502_t *cpu);
static inline void M6502_Opcode_ROL(M6502_t *cpu);
static inline void M6502_Opcode_ROR(M6502_t *cpu);
static inline void M6502_Opcode_RTI(M6502_t *cpu);
static inline void M6502_Opcode_RTS(M6502_t *cpu);
static inline void M6502_Opcode_SBC(M6502_t *cpu);
static inline void M6502_Opcode_SEC(M6502_t *cpu);
static inline void M6502_Opcode_SED(M6502_t *cpu);
static inline void M6502_Opcode_SEI(M6502_t *cpu);
static inline void M6502_Opcode_STA(M6502_t *cpu);
static inline void M6502_Opcode_STX(M6502_t *cpu);
static inline void M6502_Opcode_STY(M6502_t *cpu);
static inline void M6502_Opcode_TAX(M6502_t *cpu);
static inline void M6502_Opcode_TAY(M6502_t *cpu);
static inline void M6502_Opcode_TSX(M6502_t *cpu);
static inline void M6502_Opcode_TXA(M6502_t *cpu);
static inline void M6502_Opcode_TXS(M6502_t *cpu);
static inline void M6502_Opcode_TYA(M6502_t *cpu);

static inline void M6502_Opcode_ALR(M6502_t *cpu);
static inline void M6502_Opcode_ANC(M6502_t *cpu);
static inline void M6502_Opcode_ANE(M6502_t *cpu);
static inline void M6502_Opcode_ARR(M6502_t *cpu);
static inline void M6502_Opcode_DCP(M6502_t *cpu);
static inline void M6502_Opcode_ISC(M6502_t *cpu);
static inline void M6502_Opcode_LAS(M6502_t *cpu);
static inline void M6502_Opcode_LAX(M6502_t *cpu);
static inline void M6502_Opcode_LXA(M6502_t *cpu);
static inline void M6502_Opcode_RLA(M6502_t *cpu);
static inline void M6502_Opcode_RRA(M6502_t *cpu);
static inline void M6502_Opcode_SAX(M6502_t *cpu);
static inline void M6502_Opcode_SBX(M6502_t *cpu);
static inline void M6502_Opcode_SHA(M6502_t *cpu);
static inline void M6502_Opcode_SHX(M6502_t *cpu);
static inline void M6502_Opcode_SHY(M6502_t *cpu);
static inline void M6502_Opcode_SLO(M6502_t *cpu);
static inline void M6502_Opcode_SRE(M6502_t *cpu);
static inline void M6502_Opcode_TAS(M6502_t *cpu);
static inline void M6502_Opcode_USBC(M6502_t *cpu);
static inline void M6502_Opcode_JAM(M6502_t *cpu);

static inline uint8_t M6502_ReadMemoryByte(const uint16_t address)
{
    return M6502_ExternalReadMemory(address);
}

static inline uint16_t M6502_ReadMemoryWord(const uint16_t address)
{
    const uint16_t low  = (uint16_t)M6502_ExternalReadMemory(address);
    const uint16_t high = (uint16_t)M6502_ExternalReadMemory(address + 1u) << 8u;

    return (high | low);
}

static inline void M6502_WriteMemoryByte(const uint16_t address, const uint8_t value)
{
    M6502_ExternalWriteMemory(address, value);
}

static inline void M6502_WriteMemoryWord(const uint16_t address, const uint16_t value)
{
    const uint8_t low  = (uint8_t)(value & 0xFFu);
    const uint8_t high = (uint8_t)((value >> 8u) & 0xFFu);

    M6502_ExternalWriteMemory(address,      high);
    M6502_ExternalWriteMemory(address + 1u,  low);
}

static inline void M6502_SetFlag(M6502_t *cpu, const uint8_t flag, const uint8_t value)
{
    if (value)  cpu->statusRegister |= flag;
	else        cpu->statusRegister &= ~flag;
}

static inline uint8_t M6502_GetFlag(M6502_t *cpu, const uint8_t flag)
{
    return ((cpu->statusRegister & flag) > 0u) ? 1u : 0u;
}

static inline void M6502_PushByte(M6502_t *cpu, const uint8_t value)
{
    M6502_ExternalWriteMemory(M6502_STACK_ADDRESS + cpu->stackPointer, value);
    
    cpu->stackPointer = (cpu->stackPointer - 1u) & 0xFFu;
}

static inline void M6502_PushWord(M6502_t *cpu, const uint16_t value)
{
    const uint8_t high = (uint8_t)((value >> 8u) & 0xFFu);
    const uint8_t low  = (uint8_t)(value & 0xFFu);

    M6502_PushByte(cpu, high);
    M6502_PushByte(cpu, low);
}

static inline uint8_t M6502_PullByte(M6502_t *cpu)
{
    cpu->stackPointer = (cpu->stackPointer + 1u) & 0xFFu;

    return M6502_ExternalReadMemory(M6502_STACK_ADDRESS + cpu->stackPointer);
}

static inline uint16_t M6502_PullWord(M6502_t *cpu)
{
    const uint16_t low  = (uint16_t)M6502_PullByte(cpu); 
    const uint16_t high = (uint16_t)M6502_PullByte(cpu) << 8u; 
    
    return (high | low);
}


static inline void M6502_CarryTest(M6502_t *cpu, const uint16_t value)
{
    M6502_SetFlag(cpu, M6502_FLAG_CARRY, !!(value & 0xFF00u));
}

static inline void M6502_ZeroTest(M6502_t *cpu, const uint16_t value)
{
    M6502_SetFlag(cpu, M6502_FLAG_ZERO, !(value & 0x00FFu));
}

static inline void M6502_OverFlowTest(M6502_t *cpu, const uint16_t value, const uint16_t result)
{
    const uint16_t temporaryA = (result ^ (uint16_t)cpu->accumulator);
    const uint16_t temporaryB = (result ^ value);
    const uint16_t temporaryResult = temporaryA & temporaryB & 0x0080u;

    M6502_SetFlag(cpu, M6502_FLAG_OVERFLOW, temporaryResult);
}

static inline void M6502_NegativeTest(M6502_t *cpu, const uint16_t value)
{
    M6502_SetFlag(cpu, M6502_FLAG_NEGATIVE, (value & 0x0080u));
}


static inline void M6502_Address_Accumulator(M6502_t *cpu)
{
    cpu->address    = cpu->accumulator;
    cpu->target     = cpu->accumulator;
}

static inline void M6502_Address_Immediate(M6502_t *cpu)
{
    cpu->address    = cpu->programCounter++;
    cpu->target     = M6502_ExternalReadMemory(cpu->address);
}

static inline void M6502_Address_Relative(M6502_t *cpu)
{
    cpu->address = (uint16_t)M6502_ExternalReadMemory(cpu->programCounter++);

    if (cpu->address & 0x80u)
    {
		cpu->address = cpu->address | 0xFF00u;
    }
}

static inline void M6502_Address_Absolute(M6502_t *cpu)
{
    cpu->address    = M6502_ReadMemoryWord(cpu->programCounter);
    cpu->target     = M6502_ReadMemoryByte(cpu->address);

    cpu->programCounter += 2u;
}

static inline void M6502_Address_AbsoluteX(M6502_t *cpu)
{
    cpu->address = M6502_ReadMemoryWord(cpu->programCounter);
    cpu->programCounter += 2u;

    const uint16_t pageTest = cpu->address & 0xFF00u;

    cpu->address += (uint16_t)cpu->xRegister;

    if(pageTest != (cpu->address & 0xFF00u))
    {
        cpu->cycles++;
    }

    cpu->target = M6502_ReadMemoryByte(cpu->address);
}

static inline void M6502_Address_AbsoluteY(M6502_t *cpu)
{
    cpu->address = M6502_ReadMemoryWord(cpu->programCounter);
    cpu->programCounter += 2u;

    const uint16_t pageTest = cpu->address & 0xFF00u;

    cpu->address += (uint16_t)cpu->yRegister;

    if(pageTest != (cpu->address & 0xFF00u))
    {
        cpu->cycles++;
    }

    cpu->target = M6502_ReadMemoryByte(cpu->address);
}

static inline void M6502_Address_ZeroPage(M6502_t *cpu)
{
    cpu->address    = (uint16_t)M6502_ReadMemoryByte(cpu->programCounter++);
    cpu->target     = M6502_ReadMemoryByte(cpu->address);
}

static inline void M6502_Address_ZeroPageX(M6502_t *cpu)
{
    uint16_t temporary = (uint16_t)M6502_ReadMemoryByte(cpu->programCounter++);
    temporary += (uint16_t)cpu->xRegister;
    temporary &= 0x00FFu;

    cpu->address    = temporary;
    cpu->target     = M6502_ReadMemoryByte(temporary);
}

static inline void M6502_Address_ZeroPageY(M6502_t *cpu)
{
    uint16_t temporary = (uint16_t)M6502_ReadMemoryByte(cpu->programCounter++);
    temporary += (uint16_t)cpu->yRegister;
    temporary &= 0x00FFu;

    cpu->address    = temporary;
    cpu->target     = M6502_ReadMemoryByte(temporary);
}

static inline void M6502_Address_Indirect(M6502_t *cpu)
{
    const uint16_t temporary = M6502_ReadMemoryWord(cpu->programCounter);
    cpu->programCounter += 2u;

    const uint16_t temporary2 = (temporary & 0xFF00u) | ((temporary + 1) & 0x00FFu);

    const uint16_t low  = (uint16_t)M6502_ReadMemoryByte(temporary);
    const uint16_t high = (uint16_t)M6502_ReadMemoryByte(temporary2) << 8u;

    cpu->address = (high | low);
}

static inline void M6502_Address_IndirectX(M6502_t *cpu)
{
    uint16_t pointer = (uint16_t)M6502_ReadMemoryByte(cpu->programCounter++);
    pointer += (uint16_t)cpu->xRegister;
    pointer &= 0xFFu;

    uint16_t low    = (uint16_t)M6502_ReadMemoryByte(pointer & 0xFFu);
	uint16_t high   = (uint16_t)M6502_ReadMemoryByte((pointer + 1u) & 0xFFu);

	cpu->address    = (uint16_t)((high << 8u) | low);
	cpu->target     = M6502_ReadMemoryByte(cpu->address);
}

static inline void M6502_Address_IndirectY(M6502_t *cpu)
{
    const uint16_t pointer = (uint16_t)M6502_ReadMemoryByte(cpu->programCounter++);

    const uint16_t pointer2 = (pointer & 0xFF00u) | ((pointer + 1u) & 0x00FFu);

    const uint16_t low  = (uint16_t)M6502_ReadMemoryByte(pointer);
	const uint16_t high = (uint16_t)M6502_ReadMemoryByte(pointer2) << 8u;

    const uint16_t pointerResult = (high | low);

    cpu->address = pointerResult + (uint16_t)cpu->yRegister;
	
	if ((pointerResult & 0xFF00u) != (cpu->address & 0xFF00u))
    {
        cpu->cycles++;
    }

    cpu->target = M6502_ReadMemoryByte(cpu->address);
}


static inline void M6502_Util_WriteResult(M6502_t *cpu, const uint16_t result)
{
    const uint8_t group     = (cpu->opcode & 0x3u);
    const uint8_t address   = ((cpu->opcode & 0x1Cu) >> 2u);

    const uint8_t resultToSave = (uint8_t)(result & 0x00FFu);

    if(group == 0x02u && address == 0x02u)
    {
        cpu->accumulator = resultToSave;
        return;
    }

    M6502_ExternalWriteMemory(cpu->address, resultToSave);
}

static inline void M6502_Util_Branch(M6502_t *cpu)
{
    uint16_t address = cpu->programCounter + (int8_t)cpu->address;

    if((address & 0xFF00u) != (cpu->programCounter & 0xFF00u))
    {
        cpu->cycles++;
    }
    cpu->cycles++;

    cpu->programCounter = address;
}

static inline void M6502_Util_Interrupt(M6502_t *cpu)
{
    M6502_PushWord(cpu, cpu->programCounter);
    M6502_PushByte(cpu, (cpu->statusRegister & ~M6502_FLAG_BREAK));
    M6502_SetFlag(cpu, M6502_FLAG_INTERRUPT, 1u);

    if ((cpu->pendingInterrupts & M6502_INTERRUPT_NMI) != 0u)
    {
        cpu->programCounter = M6502_ReadMemoryWord(M6502_NMIVECTOR_ADDRESS);

        cpu->pendingInterrupts &= ~M6502_INTERRUPT_NMI;
        cpu->interruptFlags |= M6502_INTERRUPT_NMI;

        cpu->cycles = 8u;
    }
    else if ((cpu->pendingInterrupts & M6502_INTERRUPT_IRQ) != 0u)
    {
        cpu->programCounter = M6502_ReadMemoryWord(M6502_IRQVECTOR_ADDRESS);

        cpu->pendingInterrupts &= ~M6502_INTERRUPT_IRQ;
        cpu->interruptFlags |= M6502_INTERRUPT_IRQ;

        cpu->cycles = 7u;
    }
}


void M6502_Init(M6502_t *cpu)
{
    cpu->programCounter = 0x0000u;
    cpu->xRegister      = 0x00u;
    cpu->yRegister      = 0x00u;
    cpu->accumulator    = 0x00u;
    cpu->stackPointer   = 0x00u;
    cpu->statusRegister = 0x00u;
    cpu->cycles         = 0u;
    cpu->opcode         = 0x00u;
    cpu->address        = 0x0000u;
    cpu->target         = 0x0000u;

    M6502_Reset(cpu);
}

void M6502_Reset(M6502_t *cpu)
{
    cpu->programCounter     = M6502_ReadMemoryWord(M6502_RESETVECTOR_ADDRESS);
    cpu->stackPointer       = M6502_STACK_START_ADDRESS;
    cpu->interruptFlags     = 0x00u;
    cpu->pendingInterrupts  = 0x00u;
    cpu->jammed             = 0x00u;
    
    M6502_SetFlag(cpu, M6502_FLAG_INTERRUPT, 1u);
    M6502_SetFlag(cpu, M6502_FLAG_UNUSED, 1u);

    cpu->cycles = 8u;
}

void M6502_IRQ(M6502_t *cpu)
{
    cpu->pendingInterrupts |= M6502_INTERRUPT_IRQ;
}

void M6502_NMI(M6502_t *cpu)
{
    cpu->pendingInterrupts |= M6502_INTERRUPT_NMI;
}

void M6502_Step(M6502_t *cpu)
{
    if(cpu->cycles > 0u)
    {
        cpu->cycles--;
        return;
    }

    if (cpu->jammed == 0xFFu)
    {
        return;
    }

    if ((cpu->pendingInterrupts & M6502_INTERRUPT_NMI) != 0u)
    {
        if ((cpu->interruptFlags & M6502_INTERRUPT_NMI) == 0u)
        {
            M6502_Util_Interrupt(cpu);
            return;
        }
    }
    else if((cpu->pendingInterrupts & M6502_INTERRUPT_IRQ) != 0u)
    {
        if ((cpu->interruptFlags == 0u)
        && (M6502_GetFlag(cpu, M6502_FLAG_INTERRUPT) == 0u))
        {
            M6502_Util_Interrupt(cpu);
            return;
        }
    }

    M6502_SetFlag(cpu, M6502_FLAG_UNUSED, 1u);

    cpu->opcode = M6502_ExternalReadMemory(cpu->programCounter++);
    cpu->cycles = M6502_OPCODE_CYCLES[cpu->opcode];

    switch (cpu->opcode)
    {
        case 0x00u: M6502_Opcode_BRK(cpu);   return;
        case 0x20u: M6502_Opcode_JSR(cpu);   return;
        case 0x40u: M6502_Opcode_RTI(cpu);   return;
        case 0x60u: M6502_Opcode_RTS(cpu);   return;

        case 0x08u: M6502_Opcode_PHP(cpu);   return;
        case 0x18u: M6502_Opcode_CLC(cpu);   return;
        case 0x28u: M6502_Opcode_PLP(cpu);   return;
        case 0x38u: M6502_Opcode_SEC(cpu);   return;
        case 0x48u: M6502_Opcode_PHA(cpu);   return;
        case 0x58u: M6502_Opcode_CLI(cpu);   return;
        case 0x68u: M6502_Opcode_PLA(cpu);   return;
        case 0x78u: M6502_Opcode_SEI(cpu);   return;
        case 0x88u: M6502_Opcode_DEY(cpu);   return;
        case 0x8Au: M6502_Opcode_TXA(cpu);   return;
        case 0x98u: M6502_Opcode_TYA(cpu);   return;
        case 0x9Au: M6502_Opcode_TXS(cpu);   return;
        case 0xA8u: M6502_Opcode_TAY(cpu);   return;
        case 0xAAu: M6502_Opcode_TAX(cpu);   return;
        case 0xB8u: M6502_Opcode_CLV(cpu);   return;
        case 0xBAu: M6502_Opcode_TSX(cpu);   return;
        case 0xC8u: M6502_Opcode_INY(cpu);   return;
        case 0xCAu: M6502_Opcode_DEX(cpu);   return;
        case 0xD8u: M6502_Opcode_CLD(cpu);   return;
        case 0xE8u: M6502_Opcode_INX(cpu);   return;
        case 0xF8u: M6502_Opcode_SED(cpu);   return;

        case 0xEAu: M6502_Opcode_NOP(cpu);   return;

        case 0x1Au: M6502_Opcode_NOP(cpu);   return;
        case 0x3Au: M6502_Opcode_NOP(cpu);   return;
        case 0x5Au: M6502_Opcode_NOP(cpu);   return;
        case 0x7Au: M6502_Opcode_NOP(cpu);   return;
        case 0xDAu: M6502_Opcode_NOP(cpu);   return;
        case 0xFAu: M6502_Opcode_NOP(cpu);   return;
        case 0x80u:
        {
            M6502_Address_Immediate(cpu);
            M6502_Opcode_NOP(cpu);
            return;
        }
        case 0x82u:
        {
            M6502_Address_Immediate(cpu);
            M6502_Opcode_NOP(cpu);
            return;
        }
        case 0x89u:
        {
            M6502_Address_Immediate(cpu);
            M6502_Opcode_NOP(cpu);
            return;
        }
        case 0xC2u:
        {
            M6502_Address_Immediate(cpu);
            M6502_Opcode_NOP(cpu);
            return;
        }
        case 0xE2u:
        {
            M6502_Address_Immediate(cpu);
            M6502_Opcode_NOP(cpu);
            return;
        }
        case 0x04u:
        {
            M6502_Address_ZeroPage(cpu);
            M6502_Opcode_NOP(cpu);
            return;
        }
        case 0x44u:
        {
            M6502_Address_ZeroPage(cpu);
            M6502_Opcode_NOP(cpu);
            return;
        }
        case 0x64u:
        {
            M6502_Address_ZeroPage(cpu);
            M6502_Opcode_NOP(cpu);
            return;
        }
        case 0x14u:
        {
            M6502_Address_ZeroPageX(cpu);
            M6502_Opcode_NOP(cpu);
            return;
        }
        case 0x34u:
        {
            M6502_Address_ZeroPageX(cpu);
            M6502_Opcode_NOP(cpu);
            return;
        }
        case 0x54u:
        {
            M6502_Address_ZeroPageX(cpu);
            M6502_Opcode_NOP(cpu);
            return;
        }
        case 0x74u:
        {
            M6502_Address_ZeroPageX(cpu);
            M6502_Opcode_NOP(cpu);
            return;
        }
        case 0xD4u:
        {
            M6502_Address_ZeroPageX(cpu);
            M6502_Opcode_NOP(cpu);
            return;
        }
        case 0xF4u:
        {
            M6502_Address_ZeroPageX(cpu);
            M6502_Opcode_NOP(cpu);
            return;
        }
        case 0x0Cu:
        {
            M6502_Address_Absolute(cpu);
            M6502_Opcode_NOP(cpu);
            return;
        }
        case 0x1Cu: 
        {
            M6502_Address_AbsoluteX(cpu);
            M6502_Opcode_NOP(cpu);
            return;
        }
        case 0x3Cu:
        {
            M6502_Address_AbsoluteX(cpu);
            M6502_Opcode_NOP(cpu);
            return;
        }
        case 0x5Cu:
        {
            M6502_Address_AbsoluteX(cpu);
            M6502_Opcode_NOP(cpu);
            return;
        }
        case 0x7Cu:
        {
            M6502_Address_AbsoluteX(cpu);
            M6502_Opcode_NOP(cpu);
            return;
        }
        case 0xDCu:
        {
            M6502_Address_AbsoluteX(cpu);
            M6502_Opcode_NOP(cpu);
            return;
        }
        case 0xFCu:
        {
            M6502_Address_AbsoluteX(cpu);
            M6502_Opcode_NOP(cpu);
            return;
        }

        case 0x02u: M6502_Opcode_JAM(cpu);   return;
        case 0x12u: M6502_Opcode_JAM(cpu);   return;
        case 0x22u: M6502_Opcode_JAM(cpu);   return;
        case 0x32u: M6502_Opcode_JAM(cpu);   return;
        case 0x42u: M6502_Opcode_JAM(cpu);   return;
        case 0x52u: M6502_Opcode_JAM(cpu);   return;
        case 0x62u: M6502_Opcode_JAM(cpu);   return;
        case 0x72u: M6502_Opcode_JAM(cpu);   return;
        case 0x92u: M6502_Opcode_JAM(cpu);   return;
        case 0xB2u: M6502_Opcode_JAM(cpu);   return;
        case 0xD2u: M6502_Opcode_JAM(cpu);   return;
        case 0xF2u: M6502_Opcode_JAM(cpu);   return;

        case 0x4Bu:
        {
            M6502_Address_Immediate(cpu);
            M6502_Opcode_ALR(cpu);
            return;
        }

        case 0x0Bu:
        case 0x2Bu:
        {
            M6502_Address_Immediate(cpu);
            M6502_Opcode_ANC(cpu);
            return;
        }


        case 0x8Bu:
        {
            M6502_Address_Immediate(cpu);
            M6502_Opcode_ANE(cpu);
            return;
        }

        case 0x6Bu:
        {
            M6502_Address_Immediate(cpu);
            M6502_Opcode_ARR(cpu);
            return;
        }

        case 0xBBu:
        {
            M6502_Address_AbsoluteY(cpu);
            M6502_Opcode_LAS(cpu);
            return;
        }

        case 0xABu:
        {
            M6502_Address_Immediate(cpu);
            M6502_Opcode_LXA(cpu);
            return;
        }

        case 0xCBu:
        {
            M6502_Address_Immediate(cpu);
            M6502_Opcode_SBX(cpu);
            return;
        }

        case 0x9Fu:
        {
            M6502_Address_AbsoluteY(cpu);
            M6502_Opcode_SHA(cpu);
            return;
        }

        case 0x93u:
        {
            M6502_Address_IndirectY(cpu);
            M6502_Opcode_SHA(cpu);
            return;
        }

        case 0x9Cu: 
        {
            M6502_Address_AbsoluteX(cpu);
            M6502_Opcode_SHY(cpu);
            return;
        }
        case 0x9Eu:
        {
            M6502_Address_AbsoluteY(cpu);
            M6502_Opcode_SHX(cpu);
            return;
        }

        case 0x9Bu:
        {
            M6502_Address_AbsoluteY(cpu);
            M6502_Opcode_TAS(cpu);
            return;
        }

        case 0xEBu:
        {
            M6502_Address_Immediate(cpu);
            M6502_Opcode_USBC(cpu);
            return;
        }
    
        default:                            break;
    }

    switch(cpu->opcode & 0x3u)
    {
        case 0x01u:  M6502_Opcode_Group01(cpu);  break;
        case 0x02u:  M6502_Opcode_Group10(cpu);  break;
        case 0x03u:  M6502_Opcode_Group11(cpu);  break;
        case 0x00u:  M6502_Opcode_Group00(cpu);  break;
        default:                                break;
    }
}

static inline void M6502_Opcode_Group01(M6502_t *cpu)
{
    const uint8_t instruction = (cpu->opcode & 0xE0u) >> 5u;
    const uint8_t addressMode = (cpu->opcode & 0x1Cu) >> 2u;

    switch(addressMode)
    {
        case 0x00u:  M6502_Address_IndirectX(cpu);    break;
        case 0x01u:  M6502_Address_ZeroPage(cpu);     break;
        case 0x02u:  M6502_Address_Immediate(cpu);    break;
        case 0x03u:  M6502_Address_Absolute(cpu);     break;
        case 0x04u:  M6502_Address_IndirectY(cpu);    break;
        case 0x05u:  M6502_Address_ZeroPageX(cpu);    break;
        case 0x06u:  M6502_Address_AbsoluteY(cpu);    break;
        case 0x07u:  M6502_Address_AbsoluteX(cpu);    break;
    }

    switch (instruction)
    {
        case 0x00u:  M6502_Opcode_ORA(cpu);  break;
        case 0x01u:  M6502_Opcode_AND(cpu);  break;
        case 0x02u:  M6502_Opcode_EOR(cpu);  break;
        case 0x03u:  M6502_Opcode_ADC(cpu);  break;
        case 0x04u:  M6502_Opcode_STA(cpu);  break;
        case 0x05u:  M6502_Opcode_LDA(cpu);  break;
        case 0x06u:  M6502_Opcode_CMP(cpu);  break;
        case 0x07u:  M6502_Opcode_SBC(cpu);  break;
    }
}

static inline void M6502_Opcode_Group10(M6502_t *cpu)
{
    const uint8_t instruction = (cpu->opcode & 0xE0u) >> 5u;
    const uint8_t addressMode = (cpu->opcode & 0x1Cu) >> 2u;

    switch(addressMode)
    {
        case 0x00u:  M6502_Address_Immediate(cpu);   break;
        case 0x01u:  M6502_Address_ZeroPage(cpu);    break;
        case 0x02u:  M6502_Address_Accumulator(cpu); break;
        case 0x03u:  M6502_Address_Absolute(cpu);    break;
        case 0x05u:  
        {
            if(instruction == 4u || instruction == 5u)
            {
                M6502_Address_ZeroPageY(cpu);
                break;
            }
            M6502_Address_ZeroPageX(cpu);
            break;
        }
        case 0x07u:
        {
            if(instruction == 5u)
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
        case 0x00u:  M6502_Opcode_ASL(cpu);  break;
        case 0x01u:  M6502_Opcode_ROL(cpu);  break;
        case 0x02u:  M6502_Opcode_LSR(cpu);  break;
        case 0x03u:  M6502_Opcode_ROR(cpu);  break;
        case 0x04u:  M6502_Opcode_STX(cpu);  break;
        case 0x05u:  M6502_Opcode_LDX(cpu);  break;
        case 0x06u:  M6502_Opcode_DEC(cpu);  break;
        case 0x07u:  M6502_Opcode_INC(cpu);  break;
    }
}

static inline void M6502_Opcode_Group11(M6502_t *cpu)
{
    const uint8_t instruction = (cpu->opcode & 0xE0u) >> 5u;
    const uint8_t addressMode = (cpu->opcode & 0x1Cu) >> 2u;

    switch(addressMode)
    {
        case 0x00u:  M6502_Address_IndirectX(cpu);   break;
        case 0x01u:  M6502_Address_ZeroPage(cpu);    break;
        case 0x02u:  M6502_Address_Immediate(cpu);   break;
        case 0x03u:  M6502_Address_Absolute(cpu);    break;
        case 0x04u:  M6502_Address_IndirectY(cpu);   break;
        case 0x05u:
        {
            if(instruction == 4u || instruction == 5u)
            {
                M6502_Address_ZeroPageY(cpu);
                break;
            }
            M6502_Address_ZeroPageX(cpu);
            break;
        }
        case 0x06u:  M6502_Address_AbsoluteY(cpu);   break;
        case 0x07u:
        {
            if(instruction == 5u)
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
        case 0x00u:  M6502_Opcode_SLO(cpu);  break;
        case 0x01u:  M6502_Opcode_RLA(cpu);  break;
        case 0x02u:  M6502_Opcode_SRE(cpu);  break;
        case 0x03u:  M6502_Opcode_RRA(cpu);  break;
        case 0x04u:  M6502_Opcode_SAX(cpu);  break;
        case 0x05u:  M6502_Opcode_LAX(cpu);  break;
        case 0x06u:  M6502_Opcode_DCP(cpu);  break;
        case 0x07u:  M6502_Opcode_ISC(cpu);  break;
    }
}

static inline void M6502_Opcode_Group00(M6502_t *cpu)
{
    const uint8_t instruction = (cpu->opcode & 0xE0u) >> 5u;
    const uint8_t addressMode = (cpu->opcode & 0x1Cu) >> 2u;

    switch(addressMode)
    {
        case 0x00u:  M6502_Address_Immediate(cpu);       break;
        case 0x01u:  M6502_Address_ZeroPage(cpu);        break;
        case 0x03u: 
        {
            if(instruction == 0x03u)
            {
                M6502_Address_Indirect(cpu);
                break;
            }
            M6502_Address_Absolute(cpu);
            break;
        }
        case 0x04u:  M6502_Opcode_Group00_Branch(cpu);   return;
        case 0x05u:  M6502_Address_ZeroPageX(cpu);       break;
        case 0x07u:  M6502_Address_AbsoluteX(cpu);       break;
    }

    switch (instruction)
    {
        case 0x01u:  M6502_Opcode_BIT(cpu);  break;
        case 0x02u:  M6502_Opcode_JMP(cpu);  break;
        case 0x03u:  M6502_Opcode_JMP(cpu);  break;
        case 0x04u:  M6502_Opcode_STY(cpu);  break;
        case 0x05u:  M6502_Opcode_LDY(cpu);  break;
        case 0x06u:  M6502_Opcode_CPY(cpu);  break;
        case 0x07u:  M6502_Opcode_CPX(cpu);  break;
    }
}

static inline void M6502_Opcode_Group00_Branch(M6502_t *cpu)
{
    M6502_Address_Relative(cpu);

    const uint8_t branch = (cpu->opcode >> 5u);

    switch (branch)
    {
        case 0x00u: M6502_Opcode_BPL(cpu); break;
        case 0x01u: M6502_Opcode_BMI(cpu); break;
        case 0x02u: M6502_Opcode_BVC(cpu); break;
        case 0x03u: M6502_Opcode_BVS(cpu); break;
        case 0x04u: M6502_Opcode_BCC(cpu); break;
        case 0x05u: M6502_Opcode_BCS(cpu); break;
        case 0x06u: M6502_Opcode_BNE(cpu); break;
        case 0x07u: M6502_Opcode_BEQ(cpu); break;
    
        default: break;
    }
}

static inline void M6502_Opcode_ADC(M6502_t *cpu)
{
    uint16_t temporary = (uint16_t)cpu->accumulator + cpu->target;
    temporary += M6502_GetFlag(cpu, M6502_FLAG_CARRY);

#ifndef M6502_NES_CPU
    if (M6502_GetFlag(cpu, M6502_FLAG_DECIMAL))
    {
        uint16_t high = (cpu->accumulator & 0xF0u) + (cpu->target & 0xF0u);
        uint16_t low = (cpu->accumulator & 0x0Fu) + (cpu->target & 0x0Fu);
        low += M6502_GetFlag(cpu, M6502_FLAG_CARRY);

        if(low >= 0xAu)
        {
            low = ((low + 0x06u) & 0x0Fu) + 0x10u;
        }

        temporary = high + low;
        M6502_SetFlag(cpu, M6502_FLAG_NEGATIVE, (temporary & 0x80u));

        if(temporary >= 0xA0u)
        {
            temporary += 0x60u;
        }

        M6502_SetFlag(cpu, M6502_FLAG_OVERFLOW, (temporary & 0xFF80u));
        M6502_SetFlag(cpu, M6502_FLAG_CARRY, (temporary >= 0x100u));
    }
    else
#endif
    {
        M6502_CarryTest(cpu, temporary);
        M6502_NegativeTest(cpu, temporary);
        M6502_OverFlowTest(cpu, cpu->target, temporary);
    }
    M6502_ZeroTest(cpu, temporary);

    cpu->accumulator = (uint8_t)(temporary & 0x00FFu);
}

static inline void M6502_Opcode_AND(M6502_t *cpu)
{
    const uint16_t temporary = ((uint16_t)cpu->accumulator & cpu->target);

    cpu->accumulator = (uint8_t)(temporary & 0x00FFu);

    M6502_ZeroTest(cpu, temporary);
    M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_ASL(M6502_t *cpu)
{
    const uint16_t temporary = (cpu->target << 1u);
    
    M6502_Util_WriteResult(cpu, temporary);

    M6502_CarryTest(cpu, temporary);
    M6502_ZeroTest(cpu, temporary);
    M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_BCC(M6502_t *cpu)
{
    if(M6502_GetFlag(cpu, M6502_FLAG_CARRY) == 0u)
    {
        M6502_Util_Branch(cpu);
    }
}

static inline void M6502_Opcode_BCS(M6502_t *cpu)
{
    if(M6502_GetFlag(cpu, M6502_FLAG_CARRY) == 1u)
    {
        M6502_Util_Branch(cpu);
    }
}

static inline void M6502_Opcode_BEQ(M6502_t *cpu)
{
    if(M6502_GetFlag(cpu, M6502_FLAG_ZERO) == 1u)
    {
        M6502_Util_Branch(cpu);
    }
}

static inline void M6502_Opcode_BIT(M6502_t *cpu)
{
    const uint16_t temporary = ((uint16_t)cpu->accumulator & cpu->target);

    M6502_ZeroTest(cpu, temporary);
    cpu->statusRegister = (cpu->statusRegister & 0x3Fu);
    cpu->statusRegister |= (uint8_t)(cpu->target & 0xC0u);
}

static inline void M6502_Opcode_BMI(M6502_t *cpu)
{
    if(M6502_GetFlag(cpu, M6502_FLAG_NEGATIVE) == 1u)
    {
        M6502_Util_Branch(cpu);
    }
}

static inline void M6502_Opcode_BNE(M6502_t *cpu)
{
    if(M6502_GetFlag(cpu, M6502_FLAG_ZERO) == 0u)
    {
        M6502_Util_Branch(cpu);
    }
}

static inline void M6502_Opcode_BPL(M6502_t *cpu)
{
    if(M6502_GetFlag(cpu, M6502_FLAG_NEGATIVE) == 0u)
    {
        M6502_Util_Branch(cpu);
    }
}

static inline void M6502_Opcode_BRK(M6502_t *cpu)
{
    M6502_PushWord(cpu, ++cpu->programCounter);
    M6502_PushByte(cpu, (cpu->statusRegister | M6502_FLAG_BREAK));
    M6502_SetFlag(cpu, M6502_FLAG_INTERRUPT, 1u);

    cpu->programCounter = M6502_ReadMemoryWord(M6502_IRQVECTOR_ADDRESS);
}

static inline void M6502_Opcode_BVC(M6502_t *cpu)
{
    if(M6502_GetFlag(cpu, M6502_FLAG_OVERFLOW) == 0u)
    {
        M6502_Util_Branch(cpu);
    }
}

static inline void M6502_Opcode_BVS(M6502_t *cpu)
{
    if(M6502_GetFlag(cpu, M6502_FLAG_OVERFLOW) == 1u)
    {
        M6502_Util_Branch(cpu);
    }
}

static inline void M6502_Opcode_CLC(M6502_t *cpu)
{
    M6502_SetFlag(cpu, M6502_FLAG_CARRY, 0u);
}

static inline void M6502_Opcode_CLD(M6502_t *cpu)
{
    M6502_SetFlag(cpu, M6502_FLAG_DECIMAL, 0u);
}

static inline void M6502_Opcode_CLI(M6502_t *cpu)
{
    M6502_SetFlag(cpu, M6502_FLAG_INTERRUPT, 0u);
}

static inline void M6502_Opcode_CLV(M6502_t *cpu)
{
    M6502_SetFlag(cpu, M6502_FLAG_OVERFLOW, 0u);
}

static inline void M6502_Opcode_CMP(M6502_t *cpu)
{
    const uint16_t temporary = (uint16_t)cpu->accumulator - cpu->target;

    M6502_SetFlag(cpu, M6502_FLAG_CARRY, cpu->accumulator >= (uint8_t)(cpu->target & 0x00FFu));
    M6502_SetFlag(cpu, M6502_FLAG_ZERO, cpu->accumulator == (uint8_t)(cpu->target & 0x00FFu));
    M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_CPX(M6502_t *cpu)
{
    const uint16_t temporary = (uint16_t)cpu->xRegister - cpu->target;
    
    M6502_SetFlag(cpu, M6502_FLAG_CARRY, cpu->xRegister >= (uint8_t)(cpu->target & 0x00FFu));
    M6502_SetFlag(cpu, M6502_FLAG_ZERO, cpu->xRegister == (uint8_t)(cpu->target & 0x00FFu));
    M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_CPY(M6502_t *cpu)
{
    const uint16_t temporary = (uint16_t)cpu->yRegister - cpu->target;
    
    M6502_SetFlag(cpu, M6502_FLAG_CARRY, cpu->yRegister >= (uint8_t)(cpu->target & 0x00FFu));
    M6502_SetFlag(cpu, M6502_FLAG_ZERO, cpu->yRegister == (uint8_t)(cpu->target & 0x00FFu));
    M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_DEC(M6502_t *cpu)
{
    const uint16_t temporary = cpu->target - 1u;

    M6502_Util_WriteResult(cpu, temporary);

    M6502_ZeroTest(cpu, temporary);
	M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_DEX(M6502_t *cpu)
{
    cpu->xRegister = cpu->xRegister - 1u;
    
    M6502_ZeroTest(cpu, (uint16_t)(cpu->xRegister));
	M6502_NegativeTest(cpu, (uint16_t)(cpu->xRegister));
}

static inline void M6502_Opcode_DEY(M6502_t *cpu)
{
    cpu->yRegister = cpu->yRegister - 1u;
    
    M6502_ZeroTest(cpu, (uint16_t)(cpu->yRegister));
	M6502_NegativeTest(cpu, (uint16_t)(cpu->yRegister));
}

static inline void M6502_Opcode_EOR(M6502_t *cpu)
{
    const uint16_t temporary = ((uint16_t)cpu->accumulator ^ cpu->target);

    cpu->accumulator = (uint8_t)(temporary & 0x00FFu);

	M6502_ZeroTest(cpu, temporary);
	M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_INC(M6502_t *cpu)
{
    const uint16_t temporary = cpu->target + 1u;

	M6502_Util_WriteResult(cpu, temporary);
	
    M6502_ZeroTest(cpu, temporary);
	M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_INX(M6502_t *cpu)
{
    cpu->xRegister = cpu->xRegister + 1u;
    
    M6502_ZeroTest(cpu, (uint16_t)(cpu->xRegister));
	M6502_NegativeTest(cpu, (uint16_t)(cpu->xRegister));
}

static inline void M6502_Opcode_INY(M6502_t *cpu)
{
    cpu->yRegister = cpu->yRegister + 1u;
    
    M6502_ZeroTest(cpu, (uint16_t)(cpu->yRegister));
	M6502_NegativeTest(cpu, (uint16_t)(cpu->yRegister));
}

static inline void M6502_Opcode_JMP(M6502_t *cpu)
{
    cpu->programCounter = cpu->address;
}

static inline void M6502_Opcode_JSR(M6502_t *cpu)
{
    M6502_Address_Absolute(cpu);
    M6502_PushWord(cpu, cpu->programCounter - 1u);

    cpu->programCounter = cpu->address;
}

static inline void M6502_Opcode_LDA(M6502_t *cpu)
{
    cpu->accumulator = (uint8_t)(cpu->target & 0x00FFu);

	M6502_ZeroTest(cpu, (uint16_t)(cpu->accumulator));
	M6502_NegativeTest(cpu, (uint16_t)(cpu->accumulator));
}

static inline void M6502_Opcode_LDX(M6502_t *cpu)
{
    cpu->xRegister = (uint8_t)(cpu->target & 0x00FFu);

	M6502_ZeroTest(cpu, (uint16_t)(cpu->xRegister));
	M6502_NegativeTest(cpu, (uint16_t)(cpu->xRegister));
}

static inline void M6502_Opcode_LDY(M6502_t *cpu)
{
    cpu->yRegister = (uint8_t)(cpu->target & 0x00FFu);

	M6502_ZeroTest(cpu, (uint16_t)(cpu->yRegister));
	M6502_NegativeTest(cpu, (uint16_t)(cpu->yRegister));
}

static inline void M6502_Opcode_LSR(M6502_t *cpu)
{
	const uint16_t temporary = (cpu->target >> 1u);

    M6502_Util_WriteResult(cpu, temporary);

    M6502_SetFlag(cpu, M6502_FLAG_CARRY, (uint8_t)(cpu->target & 0x1u));
	M6502_ZeroTest(cpu, temporary);
    M6502_SetFlag(cpu, M6502_FLAG_NEGATIVE, 0u);
}

static inline void M6502_Opcode_NOP(M6502_t *cpu)
{
    /* Nothing */
}

static inline void M6502_Opcode_ORA(M6502_t *cpu)
{
    const uint16_t temporary = ((uint16_t)cpu->accumulator | cpu->target);

    cpu->accumulator = (uint8_t)(temporary & 0x00FFu);

    M6502_ZeroTest(cpu, temporary);
    M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_PHA(M6502_t *cpu)
{
    M6502_PushByte(cpu, cpu->accumulator);
}

static inline void M6502_Opcode_PHP(M6502_t *cpu)
{
	M6502_PushByte(cpu, (cpu->statusRegister | M6502_FLAG_BREAK));
}

static inline void M6502_Opcode_PLA(M6502_t *cpu)
{
	cpu->accumulator = M6502_PullByte(cpu);

    M6502_ZeroTest(cpu, (uint16_t)(cpu->accumulator));
    M6502_NegativeTest(cpu, (uint16_t)(cpu->accumulator));
}

static inline void M6502_Opcode_PLP(M6502_t *cpu)
{
	cpu->statusRegister = (M6502_PullByte(cpu) | M6502_FLAG_UNUSED);
}

static inline void M6502_Opcode_ROL(M6502_t *cpu)
{
    uint16_t temporary = (cpu->target << 1u);
    temporary |= (uint8_t)(M6502_GetFlag(cpu, M6502_FLAG_CARRY));
	
    M6502_Util_WriteResult(cpu, temporary);

    M6502_CarryTest(cpu, temporary);
	M6502_ZeroTest(cpu, temporary);
	M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_ROR(M6502_t *cpu)
{
    uint16_t temporary = (cpu->target >> 1u);
    temporary |= M6502_GetFlag(cpu, M6502_FLAG_CARRY) << 7u;

    M6502_Util_WriteResult(cpu, temporary);

    M6502_SetFlag(cpu, M6502_FLAG_CARRY, (uint8_t)(cpu->target & 0x1u));
	M6502_ZeroTest(cpu, temporary);
	M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_RTI(M6502_t *cpu)
{
    if ((cpu->interruptFlags & M6502_INTERRUPT_NMI) != 0u)
    {
        cpu->interruptFlags &= ~M6502_INTERRUPT_NMI;
    }
    else if ((cpu->interruptFlags & M6502_INTERRUPT_IRQ) != 0u)
    {
        cpu->interruptFlags &= ~M6502_INTERRUPT_IRQ;
    }

    cpu->statusRegister = M6502_PullByte(cpu);
    cpu->programCounter = M6502_PullWord(cpu);
}

static inline void M6502_Opcode_RTS(M6502_t *cpu)
{
    cpu->programCounter = M6502_PullWord(cpu) + 1u;
}

static inline void M6502_Opcode_SBC(M6502_t *cpu)
{
    const uint8_t carry = M6502_GetFlag(cpu, M6502_FLAG_CARRY);
    uint16_t temporary = (uint16_t)cpu->accumulator + (cpu->target ^ 0x00FFu);
    temporary += (uint16_t)carry;

    M6502_ZeroTest(cpu, temporary);
    M6502_CarryTest(cpu, temporary);
    M6502_OverFlowTest(cpu, (cpu->target ^ 0x00FFu), temporary);
    M6502_NegativeTest(cpu, (temporary & 0x00FFu));

#ifndef M6502_NES_CPU
    if (M6502_GetFlag(cpu, M6502_FLAG_DECIMAL))
    {
        uint16_t high = (cpu->accumulator & 0xF0u) - (cpu->target & 0xF0u);
        uint16_t low = (cpu->accumulator & 0x0Fu) - (cpu->target & 0x0Fu);
        low += (uint16_t)carry - 1u;

        if(low & 0x8000u)
        {
            low = ((low - 0x06u) & 0x0Fu) - 0x10u;
        }

        temporary = high + low;

        if(temporary & 0x8000u)
        {
            temporary = temporary - 0x60u;
        }
    }
#endif

    cpu->accumulator = (uint8_t)(temporary & 0x00FFu);
}

static inline void M6502_Opcode_SEC(M6502_t *cpu)
{
    M6502_SetFlag(cpu, M6502_FLAG_CARRY, 1u);
}

static inline void M6502_Opcode_SED(M6502_t *cpu)
{
    M6502_SetFlag(cpu, M6502_FLAG_DECIMAL, 1u);
}

static inline void M6502_Opcode_SEI(M6502_t *cpu)
{
    M6502_SetFlag(cpu, M6502_FLAG_INTERRUPT, 1u);
}

static inline void M6502_Opcode_STA(M6502_t *cpu)
{
    M6502_Util_WriteResult(cpu, (uint16_t)(cpu->accumulator));
}

static inline void M6502_Opcode_STX(M6502_t *cpu)
{
    M6502_Util_WriteResult(cpu, (uint16_t)(cpu->xRegister));
}

static inline void M6502_Opcode_STY(M6502_t *cpu)
{
    M6502_Util_WriteResult(cpu, (uint16_t)(cpu->yRegister));
}

static inline void M6502_Opcode_TAX(M6502_t *cpu)
{
    cpu->xRegister = cpu->accumulator;

    M6502_ZeroTest(cpu, (uint16_t)(cpu->xRegister));
    M6502_NegativeTest(cpu, (uint16_t)(cpu->xRegister));
}

static inline void M6502_Opcode_TAY(M6502_t *cpu)
{
    cpu->yRegister = cpu->accumulator;

    M6502_ZeroTest(cpu, (uint16_t)(cpu->yRegister));
    M6502_NegativeTest(cpu, (uint16_t)(cpu->yRegister));
}

static inline void M6502_Opcode_TSX(M6502_t *cpu)
{
    cpu->xRegister = cpu->stackPointer;

    M6502_ZeroTest(cpu, (uint16_t)(cpu->xRegister));
    M6502_NegativeTest(cpu, (uint16_t)(cpu->xRegister));
}

static inline void M6502_Opcode_TXA(M6502_t *cpu)
{
    cpu->accumulator = cpu->xRegister;
    
    M6502_ZeroTest(cpu, (uint16_t)(cpu->accumulator));
    M6502_NegativeTest(cpu, (uint16_t)(cpu->accumulator));
}

static inline void M6502_Opcode_TXS(M6502_t *cpu)
{
    cpu->stackPointer = cpu->xRegister;
}

static inline void M6502_Opcode_TYA(M6502_t *cpu)
{
    cpu->accumulator = cpu->yRegister;
    
    M6502_ZeroTest(cpu, (uint16_t)(cpu->accumulator));
    M6502_NegativeTest(cpu, (uint16_t)(cpu->accumulator));
}

static inline void M6502_Opcode_ALR(M6502_t *cpu)
{
    uint16_t temporary = ((uint16_t)cpu->accumulator & cpu->target);

    M6502_SetFlag(cpu, M6502_FLAG_CARRY, (uint8_t)(temporary & 0x1u));
    
    temporary = (temporary >> 1u);

    cpu->accumulator = (uint8_t)(temporary & 0x00FFu);

	M6502_ZeroTest(cpu, temporary);
	M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_ANC(M6502_t *cpu)
{
    const uint16_t temporary = ((uint16_t)cpu->accumulator & cpu->target);

    cpu->accumulator = (uint8_t)(temporary & 0x00FFu);

    M6502_ZeroTest(cpu, temporary);
    M6502_NegativeTest(cpu, temporary);
    M6502_SetFlag(cpu, M6502_FLAG_CARRY, (uint8_t)(M6502_GetFlag(cpu, M6502_FLAG_NEGATIVE)));
}

static inline void M6502_Opcode_ANE(M6502_t *cpu)
{
    uint16_t temporary = (uint16_t)(cpu->accumulator | M6502_MAGIC_CONSTANT);
    temporary &= (uint16_t)cpu->xRegister;
    temporary &= cpu->target;

    cpu->accumulator = (uint8_t)(temporary & 0x00FFu);

    M6502_ZeroTest(cpu, temporary);
    M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_ARR(M6502_t *cpu)
{
    uint16_t temporary = ((uint16_t)cpu->accumulator & cpu->target);
    temporary = (temporary >> 1u);
    temporary |= (M6502_GetFlag(cpu, M6502_FLAG_CARRY) << 7u);

    cpu->accumulator = (uint8_t)(temporary & 0x00FFu);

    const uint8_t bit5 = ((cpu->accumulator >> 5u) & 1u);
    const uint8_t bit6 = ((cpu->accumulator >> 6u) & 1u);

    M6502_SetFlag(cpu, M6502_FLAG_CARRY, bit6);
    M6502_SetFlag(cpu, M6502_FLAG_OVERFLOW, (bit5 ^ bit6));

	M6502_ZeroTest(cpu, temporary);
	M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_DCP(M6502_t *cpu)
{
    const uint16_t temporary = cpu->target - 1u;
    const uint16_t compare = (uint16_t)cpu->accumulator - temporary;

    M6502_WriteMemoryByte(cpu->address, (uint8_t)(temporary & 0x00FFu));

    M6502_SetFlag(cpu, M6502_FLAG_CARRY, cpu->accumulator >= (uint8_t)(temporary & 0x00FFu));
    M6502_SetFlag(cpu, M6502_FLAG_ZERO, cpu->accumulator == (uint8_t)(temporary & 0x00FFu));
    M6502_NegativeTest(cpu, compare);
}

static inline void M6502_Opcode_ISC(M6502_t *cpu)
{
    const uint16_t temporary = ++cpu->target;

    M6502_WriteMemoryByte(cpu->address, (uint8_t)(temporary & 0x00FFu));

    M6502_Opcode_SBC(cpu);
}

static inline void M6502_Opcode_LAS(M6502_t *cpu)
{
    const uint8_t temporary = ((uint8_t)cpu->target & cpu->stackPointer);

    cpu->accumulator = temporary;
    cpu->xRegister = temporary;
    cpu->stackPointer = temporary;

    M6502_ZeroTest(cpu, (uint16_t)(temporary));
    M6502_NegativeTest(cpu, (uint16_t)(temporary));
}

static inline void M6502_Opcode_LAX(M6502_t *cpu)
{
    cpu->accumulator = (uint8_t)(cpu->target & 0x00FFu);
    cpu->xRegister = (uint8_t)(cpu->target & 0x00FFu);

    M6502_ZeroTest(cpu, cpu->target);
    M6502_NegativeTest(cpu, cpu->target);
}

static inline void M6502_Opcode_LXA(M6502_t *cpu)
{
    uint16_t temporary = ((uint16_t)cpu->accumulator | M6502_MAGIC_CONSTANT);
    temporary &= cpu->target;

    cpu->accumulator = (uint8_t)(temporary & 0x00FFu);
    cpu->xRegister = (uint8_t)(temporary & 0x00FFu);

    M6502_ZeroTest(cpu, temporary);
    M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_RLA(M6502_t *cpu)
{
    uint16_t temporary = (cpu->target << 1u);
    temporary |= M6502_GetFlag(cpu, M6502_FLAG_CARRY);

    M6502_WriteMemoryByte(cpu->address, (uint8_t)(temporary & 0x00FFu));

    M6502_SetFlag(cpu, M6502_FLAG_CARRY, (uint8_t)(cpu->target >> 7u));

    cpu->target = temporary;

    M6502_Opcode_AND(cpu);
}

static inline void M6502_Opcode_RRA(M6502_t *cpu)
{
    uint16_t temporary = (cpu->target >> 1u);
    temporary |= (M6502_GetFlag(cpu, M6502_FLAG_CARRY) << 7u);

    M6502_SetFlag(cpu, M6502_FLAG_CARRY, (uint8_t)(cpu->target & 0x0001u));

    M6502_WriteMemoryByte(cpu->address, (uint8_t)(temporary & 0x00FFu));

    cpu->target = temporary;

    M6502_Opcode_ADC(cpu);
}

static inline void M6502_Opcode_SAX(M6502_t *cpu)
{
    const uint8_t temporary = (cpu->accumulator & cpu->xRegister);

    M6502_WriteMemoryByte(cpu->address, temporary);
}

static inline void M6502_Opcode_SBX(M6502_t *cpu)
{
    uint16_t temporary = ((uint16_t)cpu->accumulator & (uint16_t)cpu->xRegister);
    
    M6502_SetFlag(cpu, M6502_FLAG_CARRY, (uint8_t)(temporary) >= (uint8_t)(cpu->target & 0x00FFu));
    
    temporary -= cpu->target;

    cpu->xRegister = (uint8_t)(temporary & 0x00FFu);

    M6502_NegativeTest(cpu, temporary);
    M6502_ZeroTest(cpu, temporary);
}

static inline void M6502_Opcode_SHA(M6502_t *cpu)
{
    uint16_t temporary = ((uint16_t)cpu->xRegister & (uint16_t)cpu->accumulator);
    temporary &= ((cpu->address >> 8u) + 1u);

    M6502_WriteMemoryByte(cpu->address, (uint8_t)(temporary & 0x00FFu));
}

static inline void M6502_Opcode_SHX(M6502_t *cpu)
{
    uint16_t temporary = (uint16_t)cpu->xRegister & ((cpu->address >> 8u) + 1u);

    M6502_WriteMemoryByte(cpu->address, (uint8_t)(temporary & 0x00FFu));
}

static inline void M6502_Opcode_SHY(M6502_t *cpu)
{
    uint16_t temporary = ((uint16_t)cpu->yRegister & ((cpu->address >> 8u) + 1u));

    M6502_WriteMemoryByte(cpu->address, (uint8_t)(temporary & 0x00FFu));
}

static inline void M6502_Opcode_SLO(M6502_t *cpu)
{
    uint16_t temporary = (cpu->target << 1u);

    M6502_WriteMemoryByte(cpu->address, (uint8_t)(temporary & 0x00FFu));
    M6502_CarryTest(cpu, temporary);

    temporary |= (uint16_t)cpu->accumulator;

    cpu->accumulator = (uint8_t)(temporary & 0x00FFu);

    M6502_ZeroTest(cpu, temporary);
    M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_SRE(M6502_t *cpu)
{
    uint16_t temporary = (cpu->target >> 1u);

    M6502_SetFlag(cpu, M6502_FLAG_CARRY, (uint8_t)(cpu->target & 0x0001u));
    M6502_WriteMemoryByte(cpu->address, (uint8_t)(temporary & 0x00FFu));

    temporary ^= (uint16_t)cpu->accumulator;

    cpu->accumulator = (uint8_t)(temporary & 0x00FFu);

    M6502_ZeroTest(cpu, temporary);
    M6502_NegativeTest(cpu, temporary);
}

static inline void M6502_Opcode_TAS(M6502_t *cpu)
{
    cpu->stackPointer = (cpu->xRegister & cpu->accumulator);

    uint16_t temporary = (cpu->stackPointer & ((cpu->address >> 8u) + 1u));

    M6502_WriteMemoryByte(cpu->address, (uint8_t)(temporary & 0x00FFu));
}

static inline void M6502_Opcode_USBC(M6502_t *cpu)
{
    M6502_Opcode_SBC(cpu);
    M6502_Opcode_NOP(cpu);
}

static inline void M6502_Opcode_JAM(M6502_t *cpu)
{
    cpu->jammed = 0xFFu;
}