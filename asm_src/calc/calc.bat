asl -L calc.asm
p2bin calc.p calc.bin
python bin2hexsrc.py calc.bin > calc.txt