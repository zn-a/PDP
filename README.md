# PDP-Project



## Getting started

Required software:

* Jupyer Notebooks
* Vivado 2024.2: Free version is enough for the FPGA we target.
* [LLVM](https://llvm.org/docs/GettingStarted.html#getting-the-source-code-and-building-llvm): see [install llvm](#install-llvm)
* [RISCV GCC](https://github.com/riscv-collab/riscv-gnu-toolchain): see [install gcc](#install-gcc)

## Hardware: Vivado project and RTL

### Sources:

The source code of the riscy core can be found under: `pdp-project/hardware/src/design/riscy`.

The simulation testbench can be found under: `pdp-project/hardware/src/simulation/zynq_tb.sv`.

The system being simulated and implemented on the fpga is generated out of a tcl script (`pdp-project/hardware/scripts/generate_fpga_bd.tcl`), the format is not very user friendly, so it is recommended to open the bd directly in vivado when trying to understand it.

### Scripts:

A few scripts to perform all the basic functions are provided under `pdp-project/hardware/scripts`, they all require to be under `pdp-project/hardware/` when executing them.

Open vivado, and from the tcl console within it:

```
cd ./pdp-project/hardware

# To create the base project:
source ./scripts/create_project.tcl

# To run simulation and select some waveforms (run after create_project.tcl)
source ./run_simulation.tcl

# To generate the bitstream from the open project (run after create_project.tcl):
source ./run_synth_impl.tcl

# To create the base project and generate the bitstream (run from a clean open vivado):
source ./scripts/gen_bitstream.tcl

# To run Out of Context (OOC) synthesis of the riscy core and timing and utilization results (run from a clean open vivado):
source ./create_project_ooc_synth.tcl
```

### Generated files:

The generated bitstream and other products of the full project can be found under the generated folder: `pdp-project/hardware/vivado/riscy/riscy.runs/impl_1`.

The OOC synthesis results can be found under the generated folder: `pdp-project/hardware/vivado/ooc_riscy/ooc_riscy.runs/ooc_synth`.
The constraints file used for the OOC synthesis should not be modified unless it is for increasing the clock frequency, this is how we will verify that your design closes timing.

The hardware hand off file can be found under the generated directory:
`pdp-project/hardware/vivado/riscy/riscy.gen/sources_1/bd/riscv/hw_handoff/riscv.hwh`.
This file is used as part of the Overlay to write the bitstream to the fpga via Jupyter Notebooks.

## Hardware: Running on the FPGA

It is possible to connect the PYNQ board via ethernet cable to your pc, that way you can run Jupyter Notebooks in the hardened ARM core (processing_system) of the FPGA.

Using this conexion method we can read and write the instructions and data memories connected to the riscy core, start and stop the execution (fetch) of the core or perform reads and writes to any other IP connected to the processing_system.

### Setup:

* The SD card in the PYNQ board already has all the software required for this, you will only need to set up your PC to be able to access it via ethernet.

* Make sure the board is correctly setup, this is explained [here](https://pynq.readthedocs.io/en/latest/getting_started/pynq_z1_setup.html).

* You will have to set up the IP address of your laptop, instructions [here](https://pynq.readthedocs.io/en/latest/appendix/assign_a_static_ip.html#assign-a-static-ip-address).

* Then you can connect via web browser by typing the following broswer address: `http://192.168.2.99/`; or ssh into it with: `ssh xilinx@192.168.2.99`. We recommend you to use the browser.
If you are asked for a username or a password, it is `xilinx` for both.

### Use:

Once logged in, you will be able to browse its contents and run Jupyter Notebooks, this is how the home screen looks, where there should be a copy of the base riscy fpga directory:

![AAAAAA](./images/home_jupyter.png)

If the initial copy of the riscy fpga directory is not present, or you just want a fresh one, you can find it under: `pdp-project/hardware/src/sw/fpga`. This base directory contains a bitstream, memory initialization files and a jupyter notebook generated to run the base software AES implementation in the riscy core on the FPGA.

Which contains:

![AAAAAA](./images/base_riscy_folder.png)

-`base_riscy.ipynb`: base notebook to control the riscv core execution, showing how to write the bitstream to the fpga, perform reads/writes, control the core, and check the results of the AES software binaries running on the core.

-`mem_files`: should contain: `data.coe` and `code.coe`, those are the memory initialization files generated from the software AES C code and used by the notebook to load the program.

-`overlays`: should contain: `base_riscy.bit`, `base_riscy.hwh` and `base_riscy.tcl`. These are the files required to write the bitstream to the rpgrammable logic (PL) and setup the processing system to perform reads/writes. If you generate a new bitstream, you will have to copy and rename both `riscv_wrapper.bit` and `riscv_wrapper.tcl` from the implementation directory, and the hardware hand off (`.hwh`) file from the gen directory (`pdp-project/hardware/vivado/riscy/riscy.gen/sources_1/bd/riscv/hw_handoff/riscv.hwh`).



## Software

The source C code of the software AES implementation can be found under: `pdp-project/software/main.c`.

First you will need to update the paths (RISCV_GCC and LLVM) found in the configuration file `pdp-project/software/config/rv32-standard.conf` to point to your specific install of the riscv gcc and llvm.

To generate the binaries and the memory initialization files used for vivado simulation and fpga runs just:

```
cd pdp-project/software

make soft
```

You can find the generated binaries under `pdp-project/software/output` and the memory initialization files under `pdp-project/software/bin_files`.

You are encouraged to inspect and modify the Makefile compilation commands.

## Install LLVM

Source: [instructions](https://llvm.org/docs/GettingStarted.html#getting-the-source-code-and-building-llvm).
```
git clone https://github.com/llvm/llvm-project.git

cd llvm-project

mkdir build-release

cd build-release

cmake -G Ninja  -DLLVM_TARGETS_TO_BUILD="RISCV" -DLLVM_ENABLE_PROJECTS="clang;lld" -DCMAKE_BUILD_TYPE=Release -DLLVM_BUILD_TESTS=OFF -DLLVM_INCLUDE_TESTS=OFF "../llvm/"

ninja -j6
```

## Install GCC

Source: [instructions](https://github.com/riscv-collab/riscv-gnu-toolchain).
```
git clone https://github.com/riscv/riscv-gnu-toolchain

cd riscv-gnu-toolchain

#On ubuntu (for other check link):
sudo apt-get install autoconf automake autotools-dev curl python3 python3-pip python3-tomli libmpc-dev libmpfr-dev libgmp-dev gawk build-essential bison flex texinfo gperf libtool patchutils bc zlib1g-dev libexpat-dev ninja-build git cmake libglib2.0-dev libslirp-dev

./configure --prefix=./riscv --with-arch=rv32imafdcbk --with-abi=ilp32d

make
```