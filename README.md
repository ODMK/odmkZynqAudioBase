# odmkZynqAudioBase
__(((Zynq SoC audio synth and processing platform)))__

This project has been created to run on the Zedboard:
http://zedboard.org/product/zedboard

The current tool version used is Xilinx Vivado 2017.1, Vivado HLS 2017.1, and Xilinx SDK 2017.1
The design primarily consists of C, C++, VHDL code.
Python and Psipy were also used for verification, filter coefficient design and other tasks, but these tools are not required to reproduce the design.

The purpose of this design is to demonstrate the functionality of Zotech's free IP, and to demonstrate how HLS based designs can be incorporated into a SoC embedded system. Many of the functions in the system have been developed with synthesizable C++ using Xilinx's Vivado HLS tool. These functions are then packaged as IP and used together with an ARM processor in the Vivado IP Integrator design flow. Finally, Xilinx's SDK tool is used for the software portion of the design to control the integration of the peripherals with the functions running in FPGA fabric.

The design currently implements a stereo audio interface (1 stereo in, 1 stereo out), a basic audio synthesizer with sine, square and pulse code modulated square waveforms, and single sideband modulation using a hilbert transform for frequency shifting. Rotary encoders are used to control oscillator frequency, volume, etc. There is a 96x64 pixel OLED RGB display for visual feedback, and also a USB-UART for additional communication and feedback to and from a computer.





