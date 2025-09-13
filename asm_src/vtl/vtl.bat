asl -L vtl.asm
p2bin vtl.p vtl.bin
python bin2hexsrc.py vtl.bin > vtl.txt
