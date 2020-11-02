#ifndef LR35902_H
#define LR35902_H

struct LR35902 {
    void* userdata;
    unsigned short cycles;
    unsigned short SP;
    unsigned short PC;
    unsigned char registers[0x8];
    unsigned char IME;
    unsigned char HALT;
#ifdef LR35902_BUILTIN_INTERRUTS
    unsigned char IF;
    unsigned char IE;
#endif
};

void LR35902_run(struct LR35902*);
void LR35902_HLE_DMG_bios(struct LR35902*);
void LR35902_HLE_GBC_bios(struct LR35902*);

/* need to be defined */
unsigned char LR35902_read(void* user, unsigned short addr);
void LR35902_write(void* user, unsigned short addr, unsigned char value);

#ifndef LR35902_BUILTIN_INTERRUTS
unsigned char LR35902_get_interrupts(void* user);
void LR35902_handle_interrupt(void* user, unsigned char interrupt);
#endif

#endif /* LR35902 */
