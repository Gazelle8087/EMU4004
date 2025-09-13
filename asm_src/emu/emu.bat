asl -L emu.asm
p2bin emu.p emu.bin
python bin2hexsrc.py emu.bin > emu.txt