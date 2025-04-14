#!/bin/sh
#This script calls the required python scripts to convert 
#.srec files to .dat files for use as memory inits in FPGA's. 
#The resulting files are then hashed and/or encrypted. 
#Encryption will take a while (A minute or so for 1024 lines).

#Convert srec file to code and data .dat files
#The base addresses and sizes of the text and data sections 
#are given as arguments. -f is needed to ignore size errors, 
#caused by overriding the sizes for both files.
python python_scripts/srec_to_dat.py ./GPIO_LedBlink.srec -o code128.dat -b 0x8000 -s 0x8000 -f &
python python_scripts/srec_to_dat.py ./GPIO_LedBlink.srec -o data128.dat -b 0x100000 -s 0x8000 -f

python python_scripts/srec_to_dat.py ./Release/RISCKY.srec -o rom_date.dat -b 0x00000 -s 0x400

python python_scripts/srec_to_sram.py ./Release/RISCKY.srec -o sram_AES.dat -b 0x0 -s 0x4000 -b2 0x104000 -s2 0x4000 -f


#Add hashes to 128 bit files
python python_scripts/128_to_siphash_D.py ./code128.dat ./code128_siphash_D.dat &
python python_scripts/128_to_siphash_D.py ./data128.dat ./data128_siphash_D.dat 
python python_scripts/128_to_siphash_S.py ./code128.dat ./code128_siphash_S.dat &
python python_scripts/128_to_siphash_S.py ./data128.dat ./data128_siphash_S.dat
#Encrypt 128 bit files (Prince function requires python 2)
python python_scripts/128_to_prince.py ./code128.dat ./code128_prince.dat &
python python_scripts/128_to_prince.py ./data128.dat ./data128_prince.dat
#Add hashes to encrypted 128 bit files
python python_scripts/128_to_siphash_D.py ./code128_prince.dat ./code128_prince_siphash_D.dat &
python python_scripts/128_to_siphash_D.py ./data128_prince.dat ./data128_prince_siphash_D.dat 
python python_scripts/128_to_siphash_S.py ./code128_prince.dat ./code128_prince_siphash_S.dat &
python python_scripts/128_to_siphash_S.py ./data128_prince.dat ./data128_prince_siphash_S.dat


-march=rv32imc -mabi=ilp32 



