#include <stdio.h>

int main(int argc, char** argv) {
    int i;
    unsigned char buffer[0x10000] = {0};
    FILE* file;
    (void)argc; (void)argv;

    file = fopen("cpu_instrs.gb", "rb");
    if (!file) {
        perror("failed to open cpu_instrs.gb\n");
        return -1;
    }

    fread(buffer, sizeof(buffer), 1, file);
    fclose(file);

    file = fopen("cpu_instrs.h", "wb");
    if (!file) {
        perror("failed to open cpu_instrs.h\n");
        return -1;
    }

    fputs("#ifndef CPU_INSTRS_ROM_H\n#define CPU_INSTRS_ROM_H\n", file);
    fputs("\nstatic unsigned char CPU_INSTRS_ROM[] = {", file);
    for (i = 0; i < sizeof(buffer); i++) {
        if ((i % 0x10) == 0) {
            fputs("\n\t", file);
        }
        fprintf(file, "0x%02X,", buffer[i]);
    }
    fputs("\n};\n", file);

    fputs("\n#endif /* CPU_INSTRS_ROM_H */\n", file);
    fclose(file);

    return 0;
}
