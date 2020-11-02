# LR35902

gameboy cpu written in c89 with zero dependencies (no includes).

## requirements

- char to equal 1 byte.

- short to equal 2 bytes.

## what works

- passes all of blarggs cpu_instrs tests.

- interrupts*

- accurate* cycle timings.

## what doesn't work

- interrupts are meant to be delayed, this is out of scope for this as i don't (currently) care for super accurate emulation, although i would happily accept a pr to fix this! you can read more about the delayed interrupts here as well as an example of 2 games that requires this <https://mgba.io/2018/03/09/holy-grail-bugs-revisited/>

- probably more stuff here...

## credit

sources for opcode:

- <https://github.com/izik1/gbops>

- <https://gbdev.io/pandocs/>

## final

please consider giving credit if you use any code from this repo, even as a refrence (this is not required).
