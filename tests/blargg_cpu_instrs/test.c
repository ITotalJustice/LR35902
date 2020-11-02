/* just enough gb emu to pass the tests. */

/* test.c ../../LR35902.c -o test -std=c89 -pedantic -Wall -Wextra -Werror */

#include "../../LR35902.h"
#include "cpu_instrs.h"
#include <stdio.h>

#define UNUSED(a) (void)a

struct DummyMBC1 {
    unsigned char* rom;
    unsigned char rom_bank;
    /* nothing else is needed for these tests */
};

struct DummyGB {
    const unsigned char* mmap[0x10];
    struct LR35902 cpu;
    unsigned char io[0x80];
    unsigned char oam[0xA0];
    unsigned char hram[0x80];
    unsigned char wram[2][0x1000];
    unsigned char vram[0x2000];
    unsigned short timer_cycles;
    void (*cart_write)(struct DummyGB*, unsigned short, unsigned char);
    unsigned char* (*update_rom_banks)(struct DummyGB*, int);
    union { /* might add more mbc for tests */
        struct DummyMBC1 _1;
    } mbc;
};

static void update_rom_banks(struct DummyGB* gb) {
    unsigned char* a;
    unsigned char* b;
    a = gb->update_rom_banks(gb, 0);
    b = gb->update_rom_banks(gb, 1);
    gb->mmap[0x0] = a + 0x0000;
    gb->mmap[0x1] = a + 0x1000;
    gb->mmap[0x2] = a + 0x2000;
    gb->mmap[0x3] = a + 0x3000;
    gb->mmap[0x4] = b + 0x0000;
    gb->mmap[0x5] = b + 0x1000;
    gb->mmap[0x6] = b + 0x2000;
    gb->mmap[0x7] = b + 0x3000;
}

static void dummy_mbc1_write(struct DummyGB* gb, unsigned short addr, unsigned char value) {
    switch ((addr >> 12) & 0xF) {
    case 0x2: case 0x3:
        gb->mbc._1.rom_bank = (value & 0x1F) + (!value);
        update_rom_banks(gb);
        break;
    case 0x4: case 0x5:
        gb->mbc._1.rom_bank = (gb->mbc._1.rom_bank & 0x1F) | ((value & 224));
        update_rom_banks(gb);
		break;
	}
}

static unsigned char* dummy_mbc1_update_rom_banks(struct DummyGB* gb, int bank) {
    if (bank == 0) return gb->mbc._1.rom;
    return gb->mbc._1.rom + (gb->mbc._1.rom_bank * 0x4000);
}

/* this is inaccurate, do not copy this. */
static void dummy_timmer(struct DummyGB* gb, unsigned short cycles) {
    static const unsigned short TAC_FREQ[4] = { 1024, 16, 64, 256 };
    #define IO_DIV gb->io[0x04]
    #define IO_TIMA gb->io[0x05]
    #define IO_TMA gb->io[0x06]
    #define IO_TAC gb->io[0x07]

    IO_DIV += cycles;
    if (IO_TAC & 0x04) {
        gb->timer_cycles += cycles;
        while ((gb->timer_cycles) >= TAC_FREQ[IO_TAC & 0x03]) {
            gb->timer_cycles -= TAC_FREQ[IO_TAC & 0x03];
            if (IO_TIMA == 0xFF) {
                IO_TIMA = IO_TMA;
                gb->io[0xF] |= 4;
            } else {
                IO_TIMA++;
            }
        }
	}
}

unsigned char LR35902_read(void* user, unsigned short addr) {
    if (addr < 0xFE00) {
        return ((struct DummyGB*)user)->mmap[(addr >> 12)][addr & 0x0FFF];
	} else if (addr <= 0xFE9F) {
        return ((struct DummyGB*)user)->oam[addr & 0x9F];
    } else if (addr >= 0xFF00 && addr <= 0xFF7F) {
        return ((struct DummyGB*)user)->io[addr & 0x7F];
    } else if (addr >= 0xFF80) {
        return ((struct DummyGB*)user)->hram[addr & 0x7F];
    }
    return 0xFF;
}

void LR35902_write(void* user, unsigned short addr, unsigned char value) {
    if (addr < 0xFE00) {
        switch ((addr >> 12) & 0xF) {
        case 0: case 1: case 2: case 3: case 4: case 5: case 6: case 7:
            ((struct DummyGB*)user)->cart_write(((struct DummyGB*)user), addr, value);
            break;
        case 0x8: case 0x9:
            ((struct DummyGB*)user)->vram[addr & 0x1FFF] = value;
            break;
        case 0xA: case 0xB:
            break;
        case 0xC:
            ((struct DummyGB*)user)->wram[0][addr & 0x0FFF] = value;
            break;
        case 0xD:
            ((struct DummyGB*)user)->wram[1][addr & 0x0FFF] = value;
            break;
        case 0xE:
            ((struct DummyGB*)user)->wram[0][addr & 0x0FFF] = value;
            break;
        case 0xF:
            ((struct DummyGB*)user)->wram[1][addr & 0x0FFF] = value;
            break;
        }
    } else if (addr <= 0xFE9F) {
        ((struct DummyGB*)user)->oam[addr & 0x9F] = value;
    } else if (addr >= 0xFF00 && addr <= 0xFF7F) {
        ((struct DummyGB*)user)->io[addr & 0x7F] = value;
        if (addr == 0xFF01) putchar(value);
    } else if (addr >= 0xFF80) {
        ((struct DummyGB*)user)->hram[addr & 0x7F] = value;
    }
}

unsigned char LR35902_get_interrupts(void* user) {
    return (((struct DummyGB*)user)->hram[0x7F]) & (((struct DummyGB*)user)->io[0xF]);
}

void LR35902_handle_interrupt(void* user, unsigned char interrupt) {
    ((struct DummyGB*)user)->io[0xF] &= ~(interrupt);
}

int main(int argc, char** argv) {
    static struct DummyGB gb = {0};

    LR35902_HLE_DMG_bios(&gb.cpu);
    gb.cpu.userdata = &gb;
    gb.cart_write = dummy_mbc1_write;
    gb.update_rom_banks = dummy_mbc1_update_rom_banks;
    gb.mbc._1.rom = CPU_INSTRS_ROM;
    gb.mbc._1.rom_bank = 1;

    UNUSED(argc); UNUSED(argv);

    update_rom_banks(&gb);
    gb.mmap[0x8] = gb.vram + 0x0000;
    gb.mmap[0x9] = gb.vram + 0x1000;
    gb.mmap[0xC] = gb.wram[0];
    gb.mmap[0xD] = gb.wram[1];
    gb.mmap[0xE] = gb.wram[0];
    gb.mmap[0xF] = gb.wram[1];

    for (;;) {
        LR35902_run(&gb.cpu);
        dummy_timmer(&gb, gb.cpu.cycles);
    }

    return 0;
}
