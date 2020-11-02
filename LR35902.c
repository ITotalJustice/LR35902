#include "LR35902.h"

static const unsigned char CYCLE_TABLE[0x100] = {
	4,12,8,8,4,4,8,4,20,8,8,8,4,4,8,4,4,
	12,8,8,4,4,8,4,12,8,8,8,4,4,8,4,8,
	12,8,8,4,4,8,4,8,8,8,8,4,4,8,4,8,
	12,8,8,12,12,12,4,8,8,8,8,4,4,8,4,4,
	4,4,4,4,4,8,4,4,4,4,4,4,4,8,4,4,
	4,4,4,4,4,8,4,4,4,4,4,4,4,8,4,4,
	4,4,4,4,4,8,4,4,4,4,4,4,4,8,4,8,
	8,8,8,8,8,4,8,4,4,4,4,4,4,8,4,4,
	4,4,4,4,4,8,4,4,4,4,4,4,4,8,4,4,
	4,4,4,4,4,8,4,4,4,4,4,4,4,8,4,4,
	4,4,4,4,4,8,4,4,4,4,4,4,4,8,4,4,
	4,4,4,4,4,8,4,4,4,4,4,4,4,8,4,8,
	12,12,16,12,16,8,16,8,16,12,4,12,24,8,16,8,
	12,12,0,12,16,8,16,8,16,12,0,12,0,8,16,12,
	12,8,0,0,16,8,16,16,4,16,0,0,0,8,16,12,
	12,8,4,0,16,8,16,12,8,16,4,0,0,8,16,
};

static const unsigned char CYCLE_TABLE_CB[0x100] = {
	8,8,8,8,8,8,16,8,8,8,8,8,8,8,16,8,8,
	8,8,8,8,8,16,8,8,8,8,8,8,8,16,8,8,
	8,8,8,8,8,16,8,8,8,8,8,8,8,16,8,8,
	8,8,8,8,8,16,8,8,8,8,8,8,8,16,8,8,
	8,8,8,8,8,12,8,8,8,8,8,8,8,12,8,8,
	8,8,8,8,8,12,8,8,8,8,8,8,8,12,8,8,
	8,8,8,8,8,12,8,8,8,8,8,8,8,12,8,8,
	8,8,8,8,8,12,8,8,8,8,8,8,8,12,8,8,
	8,8,8,8,8,16,8,8,8,8,8,8,8,16,8,8,
	8,8,8,8,8,16,8,8,8,8,8,8,8,16,8,8,
	8,8,8,8,8,16,8,8,8,8,8,8,8,16,8,8,
	8,8,8,8,8,16,8,8,8,8,8,8,8,16,8,8,
	8,8,8,8,8,16,8,8,8,8,8,8,8,16,8,8,
	8,8,8,8,8,16,8,8,8,8,8,8,8,16,8,8,
	8,8,8,8,8,16,8,8,8,8,8,8,8,16,8,8,
	8,8,8,8,8,16,8,8,8,8,8,8,8,16,8,
};

#define REG_B cpu->registers[0]
#define REG_C cpu->registers[1]
#define REG_D cpu->registers[2]
#define REG_E cpu->registers[3]
#define REG_H cpu->registers[4]
#define REG_L cpu->registers[5]
#define REG_F cpu->registers[6]
#define REG_A cpu->registers[7]
#define REG(v) cpu->registers[(v) & 0x7]
#define REG_SP cpu->SP
#define REG_PC cpu->PC

#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
#define REG_BC ((REG_B << 8) | REG_C)
#define REG_DE ((REG_D << 8) | REG_E)
#define REG_HL ((REG_H << 8) | REG_L)
#define REG_AF ((REG_A << 8) | (REG_F & 0xF0))
#define SET_REG_BC(v) REG_B = (((v) >> 8) & 0xFF); REG_C = ((v) & 0xFF)
#define SET_REG_DE(v) REG_D = (((v) >> 8) & 0xFF); REG_E = ((v) & 0xFF)
#define SET_REG_HL(v) REG_H = (((v) >> 8) & 0xFF); REG_L = ((v) & 0xFF)
#define SET_REG_AF(v) REG_A = (((v) >> 8) & 0xFF); REG_F = ((v) & 0xF0)
#else
#define REG_BC ((REG_C << 8) | REG_B)
#define REG_DE ((REG_E << 8) | REG_D)
#define REG_HL ((REG_L << 8) | REG_H)
#define REG_AF ((REG_F << 8) | REG_A)
#define SET_REG_BC(v) REG_C = (((v) >> 8) & 0xFF); REG_B = ((v) & 0xFF)
#define SET_REG_DE(v) REG_E = (((v) >> 8) & 0xFF); REG_D = ((v) & 0xFF)
#define SET_REG_HL(v) REG_L = (((v) >> 8) & 0xFF); REG_H = ((v) & 0xFF)
#define SET_REG_AF(v) REG_F = (((v) >> 8) & 0xFF); REG_A = ((v) & 0xFF)
#endif

#define FLAG_C (!!(REG_F & 0x10))
#define FLAG_H (!!(REG_F & 0x20))
#define FLAG_N (!!(REG_F & 0x40))
#define FLAG_Z (!!(REG_F & 0x80))
#define SET_FLAG_C(n) do { REG_F ^= (-(!!(n)) ^ REG_F) & 0x10; } while(0)
#define SET_FLAG_H(n) do { REG_F ^= (-(!!(n)) ^ REG_F) & 0x20; } while(0)
#define SET_FLAG_N(n) do { REG_F ^= (-(!!(n)) ^ REG_F) & 0x40; } while(0)
#define SET_FLAG_Z(n) do { REG_F ^= (-(!!(n)) ^ REG_F) & 0x80; } while(0)
#define SET_FLAGS_HN(h,n) do { SET_FLAG_H(h); SET_FLAG_N(n); } while(0)
#define SET_FLAGS_HZ(h,z) do { SET_FLAG_H(h); SET_FLAG_Z(z); } while(0)
#define SET_FLAGS_HNZ(h,n,z) do { SET_FLAGS_HN(h,n); SET_FLAG_Z(z); } while(0)
#define SET_FLAGS_CHN(c,h,n) do { SET_FLAG_C(c); SET_FLAGS_HN(h,n); } while(0)
#define SET_ALL_FLAGS(c,h,n,z) do { SET_FLAGS_CHN(c,h,n); SET_FLAG_Z(z); } while(0)

#define read8(addr) LR35902_read(cpu->userdata, addr)
#define read16(addr) (read8(addr) | (read8(addr + 1) << 8))
#define write8(addr, value) LR35902_write(cpu->userdata, addr, value)
#define write16(addr, value) write8(addr, value & 0xFF); write8(addr + 1, (value >> 8) & 0xFF);

static void _LR35902_push(struct LR35902* cpu, unsigned short value) {
	write8(--REG_SP, (value >> 8) & 0xFF);
	write8(--REG_SP, value & 0xFF);
}
static unsigned short _LR35902_pop(struct LR35902* cpu) {
	const unsigned short result = read16(REG_SP);
	REG_SP += 2;
	return result;
}
#define PUSH(value) _LR35902_push(cpu, value)
#define POP() _LR35902_pop(cpu)

#define CALL() do { \
	const unsigned short result = read16(REG_PC); \
	PUSH(REG_PC + 2); \
	REG_PC = result; \
} while(0)
#define CALL_COND(cond) do { \
	if (cond) { \
		CALL(); \
		cpu->cycles += 12; \
	} else { \
		REG_PC += 2; \
	} \
} while(0)
#define CALL_NZ() do { CALL_COND(!FLAG_Z); } while(0)
#define CALL_Z() do { CALL_COND(FLAG_Z); } while(0)
#define CALL_NC() do { CALL_COND(!FLAG_C); } while(0)
#define CALL_C() do { CALL_COND(FLAG_C); } while(0)

#define JP() do { REG_PC = read16(REG_PC); } while(0)
#define JP_HL() do { REG_PC = REG_HL; } while(0)
#define JP_COND(cond) do { \
	if (cond) { \
		JP(); \
		cpu->cycles += 4; \
	} else { \
		REG_PC += 2; \
	} \
} while(0)
#define JP_NZ() do { JP_COND(!FLAG_Z); } while(0)
#define JP_Z() do { JP_COND(FLAG_Z); } while(0)
#define JP_NC() do { JP_COND(!FLAG_C); } while(0)
#define JP_C() do { JP_COND(FLAG_C); } while(0)

#define JR() do { REG_PC += ((signed char)read8(REG_PC)) + 1; } while(0)
#define JR_COND(cond) do { \
	if (cond) { \
		JR(); \
		cpu->cycles += 4; \
	} else { \
		REG_PC++; \
	} \
} while(0)
#define JR_NZ() do { JR_COND(!FLAG_Z); } while(0)
#define JR_Z() do { JR_COND(FLAG_Z); } while(0)
#define JR_NC() do { JR_COND(!FLAG_C); } while(0)
#define JR_C() do { JR_COND(FLAG_C); } while(0)

#define RET() do { REG_PC = POP(); } while(0)
#define RET_COND(cond) do { \
	if (cond) { \
		RET(); \
		cpu->cycles += 12; \
	} \
} while(0)
#define RET_NZ() do { RET_COND(!FLAG_Z); } while(0)
#define RET_Z() do { RET_COND(FLAG_Z); } while(0)
#define RET_NC() do { RET_COND(!FLAG_C); } while(0)
#define RET_C() do { RET_COND(FLAG_C); } while(0)

#define INC_r() do { \
	REG((opcode >> 3))++; \
	SET_FLAGS_HNZ(((REG((opcode >> 3)) & 0xF) == 0), 0, REG((opcode >> 3)) == 0); \
} while(0)

#define INC_HLa() do { \
	const unsigned char result = read8(REG_HL) + 1; \
	write8(REG_HL, result); \
	SET_FLAGS_HNZ((result & 0xF) == 0, 0, result == 0); \
} while (0)

#define DEC_r() do { \
	REG((opcode >> 3))--; \
	SET_FLAGS_HNZ(((REG((opcode >> 3)) & 0xF) == 0xF), 1, REG((opcode >> 3)) == 0); \
} while(0)

#define DEC_HLa() do { \
	const unsigned char result = read8(REG_HL) - 1; \
	write8(REG_HL, result); \
	SET_FLAGS_HNZ((result & 0xF) == 0xF, 1, result == 0); \
} while(0)

#define INC_BC() do { SET_REG_BC(REG_BC + 1); } while(0)
#define INC_DE() do { SET_REG_DE(REG_DE + 1); } while(0)
#define INC_HL() do { SET_REG_HL(REG_HL + 1); } while(0)
#define INC_SP() do { REG_SP++; } while(0)
#define DEC_BC() do { SET_REG_BC(REG_BC - 1); } while(0)
#define DEC_DE() do { SET_REG_DE(REG_DE - 1); } while(0)
#define DEC_HL() do { SET_REG_HL(REG_HL - 1); } while(0)
#define DEC_SP() do { REG_SP--; } while(0)

#define CP() do { \
	const unsigned char result = REG_A - REG(opcode); \
	SET_ALL_FLAGS(REG(opcode) < REG_A, (REG_A & 0xF) < (REG(opcode) & 0xF), 1, result == 0); \
} while(0)

#define LD_r_r() do { REG(opcode >> 3) = REG(opcode); } while(0)
#define LD_r_u8() do { REG(opcode >> 3) = read8(REG_PC++); } while(0)
#define LD_HLa_r() do { write8(REG_HL, REG(opcode)); } while(0)
#define LD_HLa_u8() do { write8(REG_HL, read8(REG_PC++)); } while(0)
#define LD_r_HLa() do { REG(opcode >> 3) = read8(REG_HL); } while(0)
#define LD_SP_u16() do { REG_SP = read16(REG_PC); REG_PC+=2; } while(0)
#define LD_A_u16() do { REG_A = read8(read16(REG_PC)); REG_PC+=2; } while(0)
#define LD_u16_A() do { write8(read16(REG_PC), REG_A); REG_PC+=2; } while(0)

#define LD_HLi_A() do { write8(REG_HL, REG_A); INC_HL(); } while(0)
#define LD_A_HLi() do { REG_A = read8(REG_HL); INC_HL(); } while(0)
#define LD_HLd_A() do { write8(REG_HL, REG_A); DEC_HL(); } while(0)
#define LD_A_HLd() do { REG_A = read8(REG_HL); DEC_HL(); } while(0)

#define LD_A_BCa() do { REG_A = read8(REG_BC); } while(0)
#define LD_A_DEa() do { REG_A = read8(REG_DE); } while(0)
#define LD_A_HLa() do { REG_A = read8(REG_HL); } while(0)
#define LD_A_AFa() do { REG_A = read8(REG_AF); } while(0)
#define LD_BCa_A() do { write8(REG_BC, REG_A); } while(0)
#define LD_DEa_A() do { write8(REG_DE, REG_A); } while(0)
#define LD_HLa_A() do { write8(REG_HL, REG_A); } while(0)
#define LD_AFa_A() do { write8(REG_AF, REG_A); } while(0)

#define LD_FFRC_A() do { write8(0xFF00 | REG_C, REG_A); } while(0)
#define LD_A_FFRC() do { REG_A = read8(0xFF00 | REG_C); } while(0)

#define LD_BC_u16() do { \
	const unsigned short result = read16(REG_PC); \
	SET_REG_BC(result); \
	REG_PC += 2; \
} while(0)
#define LD_DE_u16() do { \
	const unsigned short result = read16(REG_PC); \
	SET_REG_DE(result); \
	REG_PC += 2; \
} while(0)
#define LD_HL_u16() do { \
	const unsigned short result = read16(REG_PC); \
	SET_REG_HL(result); \
	REG_PC += 2; \
} while(0)

#define LD_SP_u16() do { REG_SP = read16(REG_PC); REG_PC+=2; } while(0)
#define LD_u16_BC() do { write16(read16(REG_PC), REG_BC); REG_PC+=2; } while(0)
#define LD_u16_DE() do { write16(read16(REG_PC), REG_DE); REG_PC+=2; } while(0)
#define LD_u16_HL() do { write16(read16(REG_PC), REG_HL); REG_PC+=2; } while(0)
#define LD_u16_SP() do { write16(read16(REG_PC), REG_SP); REG_PC+=2; } while(0)
#define LD_FFu8_A() do { write8((0xFF00 | read8(REG_PC++)), REG_A); } while(0)
#define LD_A_FFu8() do { REG_A = read8(0xFF00 | read8(REG_PC++)); } while(0)
#define LD_SP_HL() do { REG_SP = REG_HL; } while(0)

#define CP_r() do { \
	const unsigned char value = REG(opcode); \
	const unsigned char result = REG_A - value; \
	SET_ALL_FLAGS(value > REG_A, (REG_A & 0xF) < (value & 0xF), 1, result == 0); \
} while(0)

#define CP_u8() do { \
	const unsigned char value = read8(REG_PC++); \
	const unsigned char result = REG_A - value; \
	SET_ALL_FLAGS(value > REG_A, (REG_A & 0xF) < (value & 0xF), 1, result == 0); \
} while(0)

#define CP_HLa() do { \
	const unsigned char value = read8(REG_HL); \
	const unsigned char result = REG_A - value; \
	SET_ALL_FLAGS(value > REG_A, (REG_A & 0xF) < (value & 0xF), 1, result == 0); \
} while(0)

#define __ADD(value, carry) do { \
	const unsigned char result = REG_A + value + carry; \
    SET_ALL_FLAGS((REG_A + value + carry) > 0xFF, ((REG_A & 0xF) + (value & 0xF) + carry) > 0xF, 0, result == 0); \
    REG_A = result; \
} while (0)
#define ADD_r() do { __ADD(REG(opcode), 0); } while(0)
#define ADD_u8() do { const unsigned char value = read8(REG_PC++); __ADD(value, 0); } while(0)
#define ADD_HLa() do { const unsigned char value = read8(REG_HL); __ADD(value, 0); } while(0)
#define __ADD_HL(value) do { \
	const unsigned short result = REG_HL + value; \
	SET_FLAGS_CHN((REG_HL + value) > 0xFFFF, (REG_HL & 0xFFF) + (value & 0xFFF) > 0xFFF, 0); \
	SET_REG_HL(result); \
} while(0)

#define ADD_HL_BC() do { __ADD_HL(REG_BC); } while(0)
#define ADD_HL_DE() do { __ADD_HL(REG_DE); } while(0)
#define ADD_HL_HL() do { __ADD_HL(REG_HL); } while(0)
#define ADD_HL_SP() do { __ADD_HL(REG_SP); } while(0)
#define ADD_SP_i8() do { \
	const unsigned char value = read8(REG_PC++); \
    const unsigned short result = REG_SP + (signed char)value; \
    SET_ALL_FLAGS(((REG_SP & 0xFF) + value) > 0xFF, ((REG_SP & 0xF) + (value & 0xF)) > 0xF, 0, 0); \
	REG_SP = result; \
} while (0)

#define LD_HL_SP_i8() do { \
	const unsigned char value = read8(REG_PC++); \
    const unsigned short result = REG_SP + (signed char)value; \
    SET_ALL_FLAGS(((REG_SP & 0xFF) + value) > 0xFF, ((REG_SP & 0xF) + (value & 0xF)) > 0xF, 0, 0); \
	SET_REG_HL(result); \
} while (0)

#define ADC_r() do { \
	const unsigned char fc = FLAG_C; \
	__ADD(REG(opcode), fc); \
} while(0)

#define ADC_u8() do { \
	const unsigned char value = read8(REG_PC++); \
	const unsigned char fc = FLAG_C; \
	__ADD(value, fc); \
} while(0)

#define ADC_HLa() do { \
	const unsigned char value = read8(REG_HL); \
	const unsigned char fc = FLAG_C; \
	__ADD(value, fc); \
} while(0)

#define __SUB(value, carry) do { \
	const unsigned char result = REG_A - value - carry; \
	SET_ALL_FLAGS((value + carry) > REG_A, (REG_A & 0xF) < ((value & 0xF) + carry), 1, result == 0); \
    REG_A = result; \
} while (0)
#define SUB_r() do { __SUB(REG(opcode), 0); } while(0)
#define SUB_u8() do { const unsigned char value = read8(REG_PC++); __SUB(value, 0); } while(0)
#define SUB_HLa() do { const unsigned char value = read8(REG_HL); __SUB(value, 0); } while(0)
#define SBC_r() do { const unsigned char fc = FLAG_C; __SUB(REG(opcode), fc); } while(0)
#define SBC_u8() do { \
	const unsigned char value = read8(REG_PC++); \
	const unsigned char fc = FLAG_C; \
	__SUB(value, fc); \
} while(0)
#define SBC_HLa() do { \
	const unsigned char value = read8(REG_HL); \
	const unsigned char fc = FLAG_C; \
	__SUB(value, fc); \
} while(0)

#define AND_r() do { REG_A &= REG(opcode); SET_ALL_FLAGS(0, 1, 0, REG_A == 0); } while(0)
#define AND_u8() do { REG_A &= read8(REG_PC++); SET_ALL_FLAGS(0, 1, 0, REG_A == 0); } while(0)
#define AND_HLa() do { REG_A &= read8(REG_HL); SET_ALL_FLAGS(0, 1, 0, REG_A == 0); } while(0)

#define XOR_r() do { REG_A ^= REG(opcode); SET_ALL_FLAGS(0, 0, 0, REG_A == 0); } while(0)
#define XOR_u8() do { REG_A ^= read8(REG_PC++); SET_ALL_FLAGS(0, 0, 0, REG_A == 0); } while(0)
#define XOR_HLa() do { REG_A ^= read8(REG_HL); SET_ALL_FLAGS(0, 0, 0, REG_A == 0); } while(0)

#define OR_r() do { REG_A |= REG(opcode); SET_ALL_FLAGS(0, 0, 0, REG_A == 0); } while(0)
#define OR_u8() do { REG_A |= read8(REG_PC++); SET_ALL_FLAGS(0, 0, 0, REG_A == 0); } while(0)
#define OR_HLa() do { REG_A |= read8(REG_HL); SET_ALL_FLAGS(0, 0, 0, REG_A == 0); } while(0)

#define DI() do { cpu->IME = 0; } while(0)
#define EI() do { cpu->IME = 1; } while(0)

#define POP_BC() do { const unsigned short result = POP(); SET_REG_BC(result); } while(0)
#define POP_DE() do { const unsigned short result = POP(); SET_REG_DE(result); } while(0)
#define POP_HL() do { const unsigned short result = POP(); SET_REG_HL(result); } while(0)
#define POP_AF() do { const unsigned short result = POP(); SET_REG_AF(result); } while(0)

#define RL_r() do { \
	const unsigned char value = REG(opcode); \
	REG(opcode) = (REG(opcode) << 1) | (FLAG_C); \
	SET_ALL_FLAGS(value >> 7, 0, 0, REG(opcode) == 0); \
} while(0)

#define RLA() do { \
	const unsigned char value = REG(opcode); \
	REG(opcode) = (REG(opcode) << 1) | (FLAG_C); \
	SET_ALL_FLAGS(value >> 7, 0, 0, 0); \
} while(0)

#define RL_HLa() do { \
	const unsigned char value = read8(REG_HL); \
	const unsigned char result = (value << 1) | (FLAG_C); \
	write8(REG_HL, result); \
	SET_ALL_FLAGS(value >> 7, 0, 0, result == 0); \
} while (0)

#define RLC_r() do { \
	const unsigned char value = REG(opcode); \
	REG(opcode) = (REG(opcode) << 1) | ((REG(opcode) >> 7) & 1); \
	SET_ALL_FLAGS(value >> 7, 0, 0, REG(opcode) == 0); \
} while(0)

#define RLC_HLa() do { \
	const unsigned char value = read8(REG_HL); \
	const unsigned char result = (value << 1) | ((value >> 7) & 1); \
	write8(REG_HL, result); \
	SET_ALL_FLAGS(value >> 7, 0, 0, result == 0); \
} while(0)

#define RLCA() do { \
	const unsigned char value = REG(opcode); \
	REG(opcode) = (REG(opcode) << 1) | ((REG(opcode) >> 7) & 1); \
	SET_ALL_FLAGS(value >> 7, 0, 0, 0); \
} while(0)

#define RR_r() do { \
	const unsigned char value = REG(opcode); \
	REG(opcode) = (REG(opcode) >> 1) | (FLAG_C << 7); \
	SET_ALL_FLAGS(value & 1, 0, 0, REG(opcode) == 0); \
} while(0)

#define RR_HLa() do { \
	const unsigned char value = read8(REG_HL); \
	const unsigned char result = (value >> 1) | (FLAG_C << 7); \
	write8(REG_HL, result); \
	SET_ALL_FLAGS(value & 1, 0, 0, result == 0); \
} while(0)

#define RRA() do { \
	const unsigned char value = REG(opcode); \
	REG(opcode) = (REG(opcode) >> 1) | (FLAG_C << 7); \
	SET_ALL_FLAGS(value & 1, 0, 0, 0); \
} while(0)

#define RRC_r() do { \
	const unsigned char value = REG(opcode); \
	REG(opcode) = (REG(opcode) >> 1) | (REG(opcode) << 7); \
	SET_ALL_FLAGS(value & 1, 0, 0, REG(opcode) == 0); \
} while(0)

#define RRCA() do { \
	const unsigned char value = REG(opcode); \
	REG(opcode) = (REG(opcode) >> 1) | (REG(opcode) << 7); \
	SET_ALL_FLAGS(value & 1, 0, 0, 0); \
} while(0)

#define RRC_HLa() do { \
	const unsigned char value = read8(REG_HL); \
    const unsigned char result = (value >> 1) | (value << 7); \
	write8(REG_HL, result); \
	SET_ALL_FLAGS(value & 1, 0, 0, result == 0); \
} while (0)

#define SLA_r() do { \
	const unsigned char value = REG(opcode); \
	REG(opcode) <<= 1; \
	SET_ALL_FLAGS(value >> 7, 0, 0, REG(opcode) == 0); \
} while(0)

#define SLA_HLa() do { \
	const unsigned char value = read8(REG_HL); \
    const unsigned char result = value << 1; \
	write8(REG_HL, result); \
	SET_ALL_FLAGS(value >> 7, 0, 0, result == 0); \
} while(0)

#define SRA_r() do { \
	const unsigned char value = REG(opcode); \
	REG(opcode) = (REG(opcode) >> 1) | (REG(opcode) & 0x80); \
	SET_ALL_FLAGS(value & 1, 0, 0, REG(opcode) == 0); \
} while(0)

#define SRA_HLa() do { \
	const unsigned char value = read8(REG_HL); \
    const unsigned char result = (value >> 1) | (value & 0x80); \
	write8(REG_HL, result); \
	SET_ALL_FLAGS(value & 1, 0, 0, result == 0); \
} while(0)

#define SRL_r() do { \
	const unsigned char value = REG(opcode); \
	REG(opcode) >>= 1; \
	SET_ALL_FLAGS(value & 1, 0, 0, REG(opcode) == 0); \
} while(0)

#define SRL_HLa() do { \
	const unsigned char value = read8(REG_HL); \
    const unsigned char result = (value >> 1); \
	write8(REG_HL, result); \
	SET_ALL_FLAGS(value & 1, 0, 0, result == 0); \
} while(0)

#define SWAP_r() do { \
    REG(opcode) = (REG(opcode) << 4) | (REG(opcode) >> 4); \
	SET_ALL_FLAGS(0, 0, 0, REG(opcode) == 0); \
} while(0)

#define SWAP_HLa() do { \
	const unsigned char value = read8(REG_HL); \
    const unsigned char result = (value << 4) | (value >> 4); \
	write8(REG_HL, result); \
	SET_ALL_FLAGS(0, 0, 0, result == 0); \
} while(0)

#define BIT_r() do { SET_FLAGS_HNZ(1, 0, (REG(opcode) & (1 << ((opcode >> 3) & 0x7))) == 0); } while(0)
#define BIT_HLa() do { SET_FLAGS_HNZ(1, 0, (read8(REG_HL) & (1 << ((opcode >> 3) & 0x7))) == 0); } while(0)

#define RES_r() do { REG(opcode) &= ~(1 << ((opcode >> 3) & 0x7)); } while(0)
#define RES_HLa() do { write8(REG_HL, (read8(REG_HL)) & ~(1 << ((opcode >> 3) & 0x7))); } while(0)

#define SET_r() do { REG(opcode) |= (1 << ((opcode >> 3) & 0x7)); } while(0)
#define SET_HLa() do { write8(REG_HL, (read8(REG_HL)) | (1 << ((opcode >> 3) & 0x7))); } while(0)

#define DAA() do { \
	if (FLAG_N) { \
        if (FLAG_C) { \
            REG_A -= 0x60; \
            SET_FLAG_C(1); \
        } \
        if (FLAG_H) { \
            REG_A -= 0x6; \
        } \
    } else { \
        if (FLAG_C || REG_A > 0x99) { \
            REG_A += 0x60; \
            SET_FLAG_C(1); \
        } \
        if (FLAG_H || (REG_A & 0x0F) > 0x09) { \
            REG_A += 0x6; \
        } \
    } \
	SET_FLAGS_HZ(0, REG_A == 0); \
} while(0)

#define RETI() do { REG_PC = POP(); EI(); } while(0)
#define RST(value) do { PUSH(REG_PC); REG_PC = value; } while(0)

#define CPL() do { REG_A = ~REG_A; SET_FLAGS_HN(1, 1); } while(0)
#define SCF() do { SET_FLAGS_CHN(1, 0, 0); } while(0)
#define CCF() do { SET_FLAGS_CHN(FLAG_C ^ 1, 0, 0); } while(0)
#define HALT() do { cpu->HALT = 1; } while(0)

static void _LR35902_execute(struct LR35902* cpu);
static void _LR35902_execute_cb(struct LR35902* cpu);
static void _LR35902_interrupt_handler(struct LR35902* cpu);

void LR35902_HLE_DMG_bios(struct LR35902* cpu) {
	SET_REG_AF(0x01B0);
	SET_REG_BC(0x0013);
	SET_REG_DE(0x00D8);
	SET_REG_HL(0x014D);
	REG_SP = 0xFFFE;
	REG_PC = 0x0100;
}

void LR35902_run(struct LR35902* cpu) {
	cpu->cycles = 0;

	_LR35902_interrupt_handler(cpu);
		if (cpu->HALT) { /* this is slow, use gcc builtin likely */
		cpu->cycles += 8;
		return;
	}
	_LR35902_execute(cpu);
}

#ifdef LR35902_BUILTIN_INTERRUTS
#define LR35902_get_interrupts(a) cpu->IE & cpu->IF
#define LR35902_handle_interrupt(a,i) cpu->IF &= ~(i)
#endif

static void _LR35902_interrupt_handler(struct LR35902* cpu) {
	unsigned char live_interrupts;

	if (!cpu->IME && !cpu->HALT) {
		return;
	}
	
	live_interrupts = LR35902_get_interrupts(cpu->userdata);
	if (!live_interrupts) {
		return;
	}

	cpu->HALT = 0;
	if (!cpu->IME) {
		return;
	}
	cpu->IME = 0;

	if (live_interrupts & 0x01) { /* vblank */
        RST(64);
		LR35902_handle_interrupt(cpu->userdata, 0x01);
    } else if (live_interrupts & 0x02) { /* stat */
        RST(72);
		LR35902_handle_interrupt(cpu->userdata, 0x02);
    } else if (live_interrupts & 0x04) { /* timer */
        RST(80);
		LR35902_handle_interrupt(cpu->userdata, 0x04);
    } else if (live_interrupts & 0x08) { /* serial */
        RST(88);
		LR35902_handle_interrupt(cpu->userdata, 0x08);
    } else if (live_interrupts & 0x10) { /* joypad */
        RST(96);
		LR35902_handle_interrupt(cpu->userdata, 0x10);
    }

	cpu->cycles += 20;
}

static void _LR35902_execute(struct LR35902* cpu) {
	register const unsigned char opcode = read8(REG_PC++);

	switch (opcode) {
	case 0x00: /* NOP() */ break;
	case 0x01: LD_BC_u16(); break;
	case 0x02: LD_BCa_A(); break;
	case 0x03: INC_BC(); break;
	case 0x04: INC_r(); break;
	case 0x05: DEC_r(); break;
	case 0x06: LD_r_u8(); break;
	case 0x07: RLCA(); break;
	case 0x08: LD_u16_SP(); break;
	case 0x0A: LD_A_BCa(); break;
	case 0x09: ADD_HL_BC(); break;
	case 0x0B: DEC_BC(); break;
	case 0x0C: INC_r(); break;
	case 0x0D: DEC_r(); break;
	case 0x0E: LD_r_u8(); break;
	case 0x0F: RRCA(); break;
	case 0x11: LD_DE_u16(); break;
	case 0x12: LD_DEa_A(); break;
	case 0x13: INC_DE(); break;
	case 0x14: INC_r(); break;
	case 0x15: DEC_r(); break;
	case 0x16: LD_r_u8(); break;
	case 0x17: RLA(); break;
	case 0x18: JR(); break;
	case 0x19: ADD_HL_DE(); break;
	case 0x1A: LD_A_DEa(); break;
	case 0x1B: DEC_DE(); break;
	case 0x1C: INC_r(); break;
	case 0x1D: DEC_r(); break;
	case 0x1E: LD_r_u8(); break;
	case 0x1F: RRA(); break;
	case 0x20: JR_NZ(); break;
	case 0x21: LD_HL_u16(); break;
	case 0x22: LD_HLi_A(); break;
	case 0x23: INC_HL(); break;
	case 0x24: INC_r(); break;
	case 0x25: DEC_r(); break;
	case 0x26: LD_r_u8(); break;
	case 0x27: DAA(); break;
	case 0x28: JR_Z(); break;
	case 0x29: ADD_HL_HL(); break;
	case 0x2A: LD_A_HLi(); break;
	case 0x2B: DEC_HL(); break;
	case 0x2C: INC_r(); break;
	case 0x2D: DEC_r(); break;
	case 0x2E: LD_r_u8(); break;
	case 0x2F: CPL(); break;
	case 0x30: JR_NC(); break;
	case 0x31: LD_SP_u16(); break;
	case 0x32: LD_HLd_A(); break;
	case 0x33: INC_SP(); break;
	case 0x34: INC_HLa(); break;
	case 0x35: DEC_HLa(); break;
	case 0x36: LD_HLa_u8(); break;
	case 0x37: SCF(); break;
	case 0x38: JR_C(); break;
	case 0x39: ADD_HL_SP(); break;
	case 0x3A: LD_A_HLd(); break;
	case 0x3B: DEC_SP(); break;
	case 0x3C: INC_r(); break;
	case 0x3D: DEC_r(); break;
	case 0x3E: LD_r_u8(); break;
	case 0x3F: CCF(); break;
	case 0x40: /* nop b,b */ break;
	case 0x41: /* FALLTHROUGH */
	case 0x42: /* FALLTHROUGH */
	case 0x43: /* FALLTHROUGH */
	case 0x44: /* FALLTHROUGH */
	case 0x45: LD_r_r(); break;
	case 0x46: LD_r_HLa(); break;
	case 0x47: /* FALLTHROUGH */
	case 0x48: LD_r_r(); break;
	case 0x49: /* nop c,c */ break;
	case 0x4A: /* FALLTHROUGH */
	case 0x4B: /* FALLTHROUGH */
	case 0x4C: /* FALLTHROUGH */
	case 0x4D: LD_r_r(); break;
	case 0x4E: LD_r_HLa(); break;
	case 0x4F: /* FALLTHROUGH */
	case 0x50: /* FALLTHROUGH */
	case 0x51: LD_r_r(); break;
	case 0x52: /* nop d,d */ break;
	case 0x53: /* FALLTHROUGH */
	case 0x54: /* FALLTHROUGH */
	case 0x55: LD_r_r(); break;
	case 0x56: LD_r_HLa(); break;
	case 0x57: /* FALLTHROUGH */
	case 0x58: /* FALLTHROUGH */
	case 0x59: /* FALLTHROUGH */
	case 0x5A: LD_r_r(); break;
	case 0x5B: /* nop e,e */ break;
	case 0x5C: /* FALLTHROUGH */
	case 0x5D: LD_r_r(); break;
	case 0x5E: LD_r_HLa(); break;
	case 0x5F: /* FALLTHROUGH */
	case 0x60: /* FALLTHROUGH */
	case 0x61: /* FALLTHROUGH */
	case 0x62: /* FALLTHROUGH */
	case 0x63: LD_r_r(); break;
	case 0x64: /* nop h,h */ break;
	case 0x65: LD_r_r(); break;
	case 0x66: LD_r_HLa(); break;
	case 0x67: /* FALLTHROUGH */
	case 0x68: /* FALLTHROUGH */
	case 0x69: /* FALLTHROUGH */
	case 0x6A: /* FALLTHROUGH */
	case 0x6B: /* FALLTHROUGH */
	case 0x6C: LD_r_r(); break;
	case 0x6D: /* nop l,l */ break;
	case 0x6E: LD_r_HLa(); break;
	case 0x6F: LD_r_r(); break;
	case 0x70: /* FALLTHROUGH */
	case 0x71: /* FALLTHROUGH */
	case 0x72: /* FALLTHROUGH */
	case 0x73: /* FALLTHROUGH */
	case 0x74: /* FALLTHROUGH */
	case 0x75: LD_HLa_r(); break;
	case 0x76: HALT(); break;
	case 0x77: LD_HLa_r(); break;
	case 0x78: /* FALLTHROUGH */
	case 0x79: /* FALLTHROUGH */
	case 0x7A: /* FALLTHROUGH */
	case 0x7B: /* FALLTHROUGH */
	case 0x7C: /* FALLTHROUGH */
	case 0x7D: LD_r_r(); break;
	case 0x7E: LD_A_HLa(); break;
	case 0x7F: /* nop a,a */ break;
	case 0x80: /* FALLTHROUGH */
	case 0x81: /* FALLTHROUGH */
	case 0x82: /* FALLTHROUGH */
	case 0x83: /* FALLTHROUGH */
	case 0x84: /* FALLTHROUGH */
	case 0x85: ADD_r(); break;
	case 0x86: ADD_HLa(); break;
	case 0x87: ADD_r(); break;
	case 0x88: /* FALLTHROUGH */
	case 0x89: /* FALLTHROUGH */
	case 0x8A: /* FALLTHROUGH */
	case 0x8B: /* FALLTHROUGH */
	case 0x8C: /* FALLTHROUGH */
	case 0x8D: ADC_r(); break;
	case 0x8E: ADC_HLa(); break;
	case 0x8F: ADC_r(); break;
	case 0x90: /* FALLTHROUGH */
	case 0x91: /* FALLTHROUGH */
	case 0x92: /* FALLTHROUGH */
	case 0x93: /* FALLTHROUGH */
	case 0x94: /* FALLTHROUGH */
	case 0x95: SUB_r(); break;
	case 0x96: SUB_HLa(); break;
	case 0x97: SUB_r(); break;
	case 0x98: /* FALLTHROUGH */
	case 0x99: /* FALLTHROUGH */
	case 0x9A: /* FALLTHROUGH */
	case 0x9B: /* FALLTHROUGH */
	case 0x9C: /* FALLTHROUGH */
	case 0x9D: SBC_r(); break;
	case 0x9E: SBC_HLa(); break;
	case 0x9F: SBC_r(); break;
	case 0xA0: /* FALLTHROUGH */
	case 0xA1: /* FALLTHROUGH */
	case 0xA2: /* FALLTHROUGH */
	case 0xA3: /* FALLTHROUGH */
	case 0xA4: /* FALLTHROUGH */
	case 0xA5: AND_r(); break;
	case 0xA6: AND_HLa(); break;
	case 0xA7: AND_r(); break;
	case 0xA8: /* FALLTHROUGH */
	case 0xA9: /* FALLTHROUGH */
	case 0xAA: /* FALLTHROUGH */
	case 0xAB: /* FALLTHROUGH */
	case 0xAC: /* FALLTHROUGH */
	case 0xAD: XOR_r(); break;
	case 0xAE: XOR_HLa(); break;
	case 0xAF: XOR_r(); break;
	case 0xB0: /* FALLTHROUGH */
	case 0xB1: /* FALLTHROUGH */
	case 0xB2: /* FALLTHROUGH */
	case 0xB3: /* FALLTHROUGH */
	case 0xB4: /* FALLTHROUGH */
	case 0xB5: OR_r(); break;
	case 0xB6: OR_HLa(); break;
	case 0xB7: OR_r(); break;
	case 0xB8: /* FALLTHROUGH */
	case 0xB9: /* FALLTHROUGH */
	case 0xBA: /* FALLTHROUGH */
	case 0xBB: /* FALLTHROUGH */
	case 0xBC: /* FALLTHROUGH */
	case 0xBD: CP_r(); break;
	case 0xBE: CP_HLa(); break;
	case 0xBF: CP_r(); break;
	case 0xC0: RET_NZ(); break;
	case 0xC1: POP_BC(); break;
	case 0xC2: JP_NZ(); break;
	case 0xC3: JP();  break;
	case 0xC4: CALL_NZ(); break;
	case 0xC5: PUSH(REG_BC); break;
	case 0xC6: ADD_u8(); break;
	case 0xC7: RST(0x00); break;
	case 0xC8: RET_Z(); break;
	case 0xC9: RET(); break;
	case 0xCA: JP_Z(); break;
	case 0xCB: _LR35902_execute_cb(cpu); return;
	case 0xCC: CALL_Z(); break;
	case 0xCD: CALL(); break;
	case 0xCE: ADC_u8(); break;
	case 0xCF: RST(0x08); break;
	case 0xD0: RET_NC(); break;
	case 0xD1: POP_DE();  break;
	case 0xD2: JP_NC(); break;
	case 0xD4: CALL_NC(); break;
	case 0xD5: PUSH(REG_DE); break;
	case 0xD6: SUB_u8(); break;
	case 0xD7: RST(0x10); break;
	case 0xD8: RET_C(); break;
	case 0xD9: RETI(); break;
	case 0xDA: JP_C(); break;
	case 0xDC: CALL_C(); break;
	case 0xDE: SBC_u8(); break;
	case 0xDF: RST(0x18); break;
	case 0xE0: LD_FFu8_A(); break;
	case 0xE1: POP_HL(); break;
	case 0xE2: LD_FFRC_A(); break;
	case 0xE5: PUSH(REG_HL); break;
	case 0xE6: AND_u8(); break;
	case 0xE7: RST(0x20); break;
	case 0xE8: ADD_SP_i8(); break;
	case 0xE9: JP_HL(); break;
	case 0xEA: LD_u16_A(); break;
	case 0xEE: XOR_u8(); break;
	case 0xEF: RST(0x28); break;
	case 0xF0: LD_A_FFu8(); break;
	case 0xF1: POP_AF(); break;
	case 0xF2: LD_A_FFRC(); break;
	case 0xF3: DI(); break;
	case 0xF5: PUSH(REG_AF); break;
	case 0xF6: OR_u8(); break;
	case 0xF7: RST(0x30); break;
	case 0xF8: LD_HL_SP_i8(); break;
	case 0xF9: LD_SP_HL(); break;
	case 0xFA: LD_A_u16(); break;
	case 0xFB: EI(); break;
	case 0xFE: CP_u8(); break;
	case 0xFF: RST(0x38); break;
	}

	cpu->cycles += CYCLE_TABLE[opcode];
}

static void _LR35902_execute_cb(struct LR35902* cpu) {
	register const unsigned char opcode = read8(REG_PC++);

	switch (opcode) {
	case 0x00: /* FALLTHROUGH */
	case 0x01: /* FALLTHROUGH */
	case 0x02: /* FALLTHROUGH */
	case 0x03: /* FALLTHROUGH */
	case 0x04: /* FALLTHROUGH */
	case 0x05: RLC_r(); break;
	case 0x06: RLC_HLa(); break;
	case 0x07: RLC_r(); break;
	case 0x08: /* FALLTHROUGH */
	case 0x09: /* FALLTHROUGH */
	case 0x0A: /* FALLTHROUGH */
	case 0x0B: /* FALLTHROUGH */
	case 0x0C: /* FALLTHROUGH */
	case 0x0D: RRC_r(); break;
	case 0x0E: RRC_HLa(); break;
	case 0x0F: RRC_r(); break;
	case 0x10: /* FALLTHROUGH */
	case 0x11: /* FALLTHROUGH */
	case 0x12: /* FALLTHROUGH */
	case 0x13: /* FALLTHROUGH */
	case 0x14: /* FALLTHROUGH */
	case 0x15: RL_r(); break;
	case 0x16: RL_HLa(); break;
	case 0x17: RL_r(); break;
	case 0x18: /* FALLTHROUGH */
	case 0x19: /* FALLTHROUGH */
	case 0x1A: /* FALLTHROUGH */
	case 0x1B: /* FALLTHROUGH */
	case 0x1C: /* FALLTHROUGH */
	case 0x1D: RR_r(); break;
	case 0x1E: RR_HLa(); break;
	case 0x1F: RR_r(); break;
	case 0x20: /* FALLTHROUGH */
	case 0x21: /* FALLTHROUGH */
	case 0x22: /* FALLTHROUGH */
	case 0x23: /* FALLTHROUGH */
	case 0x24: /* FALLTHROUGH */
	case 0x25: SLA_r(); break;
	case 0x26: SLA_HLa(); break;
	case 0x27: SLA_r(); break;
	case 0x28: /* FALLTHROUGH */
	case 0x29: /* FALLTHROUGH */
	case 0x2A: /* FALLTHROUGH */
	case 0x2B: /* FALLTHROUGH */
	case 0x2C: /* FALLTHROUGH */
	case 0x2D: SRA_r(); break;
	case 0x2E: SRA_HLa(); break;
	case 0x2F: SRA_r(); break;
	case 0x30: /* FALLTHROUGH */
	case 0x31: /* FALLTHROUGH */
	case 0x32: /* FALLTHROUGH */
	case 0x33: /* FALLTHROUGH */
	case 0x34: /* FALLTHROUGH */
	case 0x35: SWAP_r(); break;
	case 0x36: SWAP_HLa(); break;
	case 0x37: SWAP_r(); break;
	case 0x38: /* FALLTHROUGH */
	case 0x39: /* FALLTHROUGH */
	case 0x3A: /* FALLTHROUGH */
	case 0x3B: /* FALLTHROUGH */
	case 0x3C: /* FALLTHROUGH */
	case 0x3D: SRL_r(); break;
	case 0x3E: SRL_HLa(); break;
	case 0x3F: SRL_r(); break;
	case 0x40: /* FALLTHROUGH */
	case 0x41: /* FALLTHROUGH */
	case 0x42: /* FALLTHROUGH */
	case 0x43: /* FALLTHROUGH */
	case 0x44: /* FALLTHROUGH */
	case 0x45: BIT_r(); break;
	case 0x46: BIT_HLa(); break;
	case 0x47: /* FALLTHROUGH */
	case 0x48: /* FALLTHROUGH */
	case 0x49: /* FALLTHROUGH */
	case 0x4A: /* FALLTHROUGH */
	case 0x4B: /* FALLTHROUGH */
	case 0x4C: /* FALLTHROUGH */
	case 0x4D: BIT_r(); break;
	case 0x4E: BIT_HLa(); break;
	case 0x4F: /* FALLTHROUGH */
	case 0x50: /* FALLTHROUGH */
	case 0x51: /* FALLTHROUGH */
	case 0x52: /* FALLTHROUGH */
	case 0x53: /* FALLTHROUGH */
	case 0x54: /* FALLTHROUGH */
	case 0x55: BIT_r(); break;
	case 0x56: BIT_HLa(); break;
	case 0x57: /* FALLTHROUGH */
	case 0x58: /* FALLTHROUGH */
	case 0x59: /* FALLTHROUGH */
	case 0x5A: /* FALLTHROUGH */
	case 0x5B: /* FALLTHROUGH */
	case 0x5C: /* FALLTHROUGH */
	case 0x5D: BIT_r(); break;
	case 0x5E: BIT_HLa(); break;
	case 0x5F: /* FALLTHROUGH */
	case 0x60: /* FALLTHROUGH */
	case 0x61: /* FALLTHROUGH */
	case 0x62: /* FALLTHROUGH */
	case 0x63: /* FALLTHROUGH */
	case 0x64: /* FALLTHROUGH */
	case 0x65: BIT_r(); break;
	case 0x66: BIT_HLa(); break;
	case 0x67: /* FALLTHROUGH */
	case 0x68: /* FALLTHROUGH */
	case 0x69: /* FALLTHROUGH */
	case 0x6A: /* FALLTHROUGH */
	case 0x6B: /* FALLTHROUGH */
	case 0x6C: /* FALLTHROUGH */
	case 0x6D: BIT_r(); break;
	case 0x6E: BIT_HLa(); break;
	case 0x6F: /* FALLTHROUGH */
	case 0x70: /* FALLTHROUGH */
	case 0x71: /* FALLTHROUGH */
	case 0x72: /* FALLTHROUGH */
	case 0x73: /* FALLTHROUGH */
	case 0x74: /* FALLTHROUGH */
	case 0x75: BIT_r(); break;
	case 0x76: BIT_HLa(); break;
	case 0x77: /* FALLTHROUGH */
	case 0x78: /* FALLTHROUGH */
	case 0x79: /* FALLTHROUGH */
	case 0x7A: /* FALLTHROUGH */
	case 0x7B: /* FALLTHROUGH */
	case 0x7C: /* FALLTHROUGH */
	case 0x7D: BIT_r(); break;
	case 0x7E: BIT_HLa(); break;
	case 0x7F: BIT_r(); break;
	case 0x80: /* FALLTHROUGH */
	case 0x81: /* FALLTHROUGH */
	case 0x82: /* FALLTHROUGH */
	case 0x83: /* FALLTHROUGH */
	case 0x84: /* FALLTHROUGH */
	case 0x85: RES_r(); break;
	case 0x86: RES_HLa(); break;
	case 0x87: /* FALLTHROUGH */
	case 0x88: /* FALLTHROUGH */
	case 0x89: /* FALLTHROUGH */
	case 0x8A: /* FALLTHROUGH */
	case 0x8B: /* FALLTHROUGH */
	case 0x8C: /* FALLTHROUGH */
	case 0x8D: RES_r(); break;
	case 0x8E: RES_HLa(); break;
	case 0x8F: /* FALLTHROUGH */
	case 0x90: /* FALLTHROUGH */
	case 0x91: /* FALLTHROUGH */
	case 0x92: /* FALLTHROUGH */
	case 0x93: /* FALLTHROUGH */
	case 0x94: /* FALLTHROUGH */
	case 0x95: RES_r(); break;
	case 0x96: RES_HLa(); break;
	case 0x97: /* FALLTHROUGH */
	case 0x98: /* FALLTHROUGH */
	case 0x99: /* FALLTHROUGH */
	case 0x9A: /* FALLTHROUGH */
	case 0x9B: /* FALLTHROUGH */
	case 0x9C: /* FALLTHROUGH */
	case 0x9D: RES_r(); break;
	case 0x9E: RES_HLa(); break;
	case 0x9F: /* FALLTHROUGH */
	case 0xA0: /* FALLTHROUGH */
	case 0xA1: /* FALLTHROUGH */
	case 0xA2: /* FALLTHROUGH */
	case 0xA3: /* FALLTHROUGH */
	case 0xA4: /* FALLTHROUGH */
	case 0xA5: RES_r(); break;
	case 0xA6: RES_HLa(); break;
	case 0xA7: /* FALLTHROUGH */
	case 0xA8: /* FALLTHROUGH */
	case 0xA9: /* FALLTHROUGH */
	case 0xAA: /* FALLTHROUGH */
	case 0xAB: /* FALLTHROUGH */
	case 0xAC: /* FALLTHROUGH */
	case 0xAD: RES_r(); break;
	case 0xAE: RES_HLa(); break;
	case 0xAF: /* FALLTHROUGH */
	case 0xB0: /* FALLTHROUGH */
	case 0xB1: /* FALLTHROUGH */
	case 0xB2: /* FALLTHROUGH */
	case 0xB3: /* FALLTHROUGH */
	case 0xB4: /* FALLTHROUGH */
	case 0xB5: RES_r(); break;
	case 0xB6: RES_HLa(); break;
	case 0xB7: /* FALLTHROUGH */
	case 0xB8: /* FALLTHROUGH */
	case 0xB9: /* FALLTHROUGH */
	case 0xBA: /* FALLTHROUGH */
	case 0xBB: /* FALLTHROUGH */
	case 0xBC: /* FALLTHROUGH */
	case 0xBD: RES_r(); break;
	case 0xBE: RES_HLa(); break;
	case 0xBF: RES_r(); break;
	case 0xC0: /* FALLTHROUGH */
	case 0xC1: /* FALLTHROUGH */
	case 0xC2: /* FALLTHROUGH */
	case 0xC3: /* FALLTHROUGH */
	case 0xC4: /* FALLTHROUGH */
	case 0xC5: SET_r(); break;
	case 0xC6: SET_HLa(); break;
	case 0xC7: /* FALLTHROUGH */
	case 0xC8: /* FALLTHROUGH */
	case 0xC9: /* FALLTHROUGH */
	case 0xCA: /* FALLTHROUGH */
	case 0xCB: /* FALLTHROUGH */
	case 0xCC: /* FALLTHROUGH */
	case 0xCD: SET_r(); break;
	case 0xCE: SET_HLa(); break;
	case 0xCF: /* FALLTHROUGH */
	case 0xD0: /* FALLTHROUGH */
	case 0xD1: /* FALLTHROUGH */
	case 0xD2: /* FALLTHROUGH */
	case 0xD3: /* FALLTHROUGH */
	case 0xD4: /* FALLTHROUGH */
	case 0xD5: SET_r(); break;
	case 0xD6: SET_HLa(); break;
	case 0xD7: /* FALLTHROUGH */
	case 0xD8: /* FALLTHROUGH */
	case 0xD9: /* FALLTHROUGH */
	case 0xDA: /* FALLTHROUGH */
	case 0xDB: /* FALLTHROUGH */
	case 0xDC: /* FALLTHROUGH */
	case 0xDD: SET_r(); break;
	case 0xDE: SET_HLa(); break;
	case 0xDF: /* FALLTHROUGH */
	case 0xE0: /* FALLTHROUGH */
	case 0xE1: /* FALLTHROUGH */
	case 0xE2: /* FALLTHROUGH */
	case 0xE3: /* FALLTHROUGH */
	case 0xE4: /* FALLTHROUGH */
	case 0xE5: SET_r(); break;
	case 0xE6: SET_HLa(); break;
	case 0xE7: /* FALLTHROUGH */
	case 0xE8: /* FALLTHROUGH */
	case 0xE9: /* FALLTHROUGH */
	case 0xEA: /* FALLTHROUGH */
	case 0xEB: /* FALLTHROUGH */
	case 0xEC: /* FALLTHROUGH */
	case 0xED: SET_r(); break;
	case 0xEE: SET_HLa(); break;
	case 0xEF: /* FALLTHROUGH */
	case 0xF0: /* FALLTHROUGH */
	case 0xF1: /* FALLTHROUGH */
	case 0xF2: /* FALLTHROUGH */
	case 0xF3: /* FALLTHROUGH */
	case 0xF4: /* FALLTHROUGH */
	case 0xF5: SET_r(); break;
	case 0xF6: SET_HLa(); break;
	case 0xF7: /* FALLTHROUGH */
	case 0xF8: /* FALLTHROUGH */
	case 0xF9: /* FALLTHROUGH */
	case 0xFA: /* FALLTHROUGH */
	case 0xFB: /* FALLTHROUGH */
	case 0xFC: /* FALLTHROUGH */
	case 0xFD: SET_r(); break;
	case 0xFE: SET_HLa(); break;
	case 0xFF: SET_r(); break;
	}

	cpu->cycles += CYCLE_TABLE_CB[opcode];
}
