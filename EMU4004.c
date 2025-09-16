/*!
 * Copyright (c) 2025 by Gazelle
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *-----------------------------------------------------------------------
 * EMU4004 ROM, RAM, UART emulation firmware
 *
 * Target: EMU4004 with 4004/4040 and PIC18F47Q84
 * IDE: MPLAB X v6.25
 * Compiler: MPLAB XC8 v3.0
 * 
 * Repsitory https://github.com/Gazelle8087/EMU4004
 *
 * 2025/09/13  Rev. 1.00 Initial release
 * 2025/09/14  Rev. 1.01 bug fix(CM-RAM calculation fixed)
 * 2025/09/16  Rev. 1.02 BBS instruction implemented(needed for SRC receive)
 */

// CONFIG1
#pragma config FEXTOSC = OFF	// External Oscillator Selection (Oscillator not enabled)
#pragma config RSTOSC = HFINTOSC_64MHZ// Reset Oscillator Selection (HFINTOSC with HFFRQ = 64 MHz and CDIV = 1:1)

// CONFIG2
#pragma config CLKOUTEN = OFF	// Clock out Enable bit (CLKOUT function is disabled)
#pragma config PR1WAY = ON		// PRLOCKED One-Way Set Enable bit (PRLOCKED bit can be cleared and set only once)
#pragma config CSWEN = ON		// Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON		// Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#ifndef _18F47Q43
#pragma config JTAGEN = OFF
#pragma config FCMENP = OFF
#pragma config FCMENS = OFF
#endif

// CONFIG3
#pragma config MCLRE = EXTMCLR	// MCLR Enable bit (If LVP = 0, MCLR pin is MCLR; If LVP = 1, RE3 pin function is MCLR )
#pragma config PWRTS = PWRT_OFF // Power-up timer selection bits (PWRT is disabled)
#pragma config MVECEN = ON		// Multi-vector enable bit (Multi-vector enabled, Vector table used for interrupts)
#pragma config IVT1WAY = ON		// IVTLOCK bit One-way set enable bit (IVTLOCKED bit can be cleared and set only once)
#pragma config LPBOREN = OFF	// Low Power BOR Enable bit (Low-Power BOR disabled)
#pragma config BOREN = SBORDIS	// Brown-out Reset Enable bits (Brown-out Reset enabled , SBOREN bit is ignored)

// CONFIG4
#pragma config BORV = VBOR_1P9	// Brown-out Reset Voltage Selection bits (Brown-out Reset Voltage (VBOR) set to 1.9V)
#pragma config ZCD = OFF		// ZCD Disable bit (ZCD module is disabled. ZCD can be enabled by setting the ZCDSEN bit of ZCDCON)
#pragma config PPS1WAY = OFF	// PPSLOCK bit One-Way Set Enable bit (PPSLOCKED bit can be set and cleared repeatedly (subject to the unlock sequence))
#pragma config STVREN = ON		// Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON			// Low Voltage Programming Enable bit (Low voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored)
#pragma config XINST = OFF		// Extended Instruction Set Enable bit (Extended Instruction Set and Indexed Addressing Mode disabled)

// CONFIG5
#pragma config WDTCPS = WDTCPS_31// WDT Period selection bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF		// WDT operating mode (WDT Disabled; SWDTEN is ignored)

// CONFIG6
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC		// WDT input clock selector (Software Control)

// CONFIG7
#pragma config BBSIZE = BBSIZE_512// Boot Block Size selection bits (Boot Block size is 512 words)
#pragma config BBEN = OFF		// Boot Block enable bit (Boot block disabled)
#pragma config SAFEN = OFF		// Storage Area Flash enable bit (SAF disabled)
#ifdef _18F47Q43
#pragma config DEBUG = OFF		// Background Debugger (Background Debugger disabled)	
#endif

// CONFIG8
#pragma config WRTB = OFF		// Boot Block Write Protection bit (Boot Block not Write protected)
#pragma config WRTC = OFF		// Configuration Register Write Protection bit (Configuration registers not Write protected)
#pragma config WRTD = OFF		// Data EEPROM Write Protection bit (Data EEPROM not Write protected)
#pragma config WRTSAF = OFF		// SAF Write protection bit (SAF not Write Protected)
#pragma config WRTAPP = OFF		// Application Block write protection bit (Application Block not write protected)

// CONFIG10
#pragma config CP = OFF		 // PFM and Data EEPROM Code Protection bit (PFM and Data EEPROM code protection disabled)

#ifndef _18F47Q43
// CONFIG9
#pragma config BOOTPINSEL = RC5 // CRC on boot output pin selection (CRC on boot output pin is RC5)
#pragma config BPEN = OFF       // CRC on boot output pin enable bit (CRC on boot output pin disabled)
#pragma config ODCON = OFF      // CRC on boot output pin open drain bit (Pin drives both high-going and low-going signals)

// CONFIG11
#pragma config BOOTSCEN = OFF   // CRC on boot scan enable for boot area (CRC on boot will not include the boot area of program memory in its calculation)
#pragma config BOOTCOE = HALT   // CRC on boot Continue on Error for boot areas bit (CRC on boot will stop device if error is detected in boot areas)
#pragma config APPSCEN = OFF    // CRC on boot application code scan enable (CRC on boot will not include the application area of program memory in its calculation)
#pragma config SAFSCEN = OFF    // CRC on boot SAF area scan enable (CRC on boot will not include the SAF area of program memory in its calculation)
#pragma config DATASCEN = OFF   // CRC on boot Data EEPROM scan enable (CRC on boot will not include data EEPROM in its calculation)
#pragma config CFGSCEN = OFF    // CRC on boot Config fuses scan enable (CRC on boot will not include the configuration fuses in its calculation)
#pragma config COE = HALT       // CRC on boot Continue on Error for non-boot areas bit (CRC on boot will stop device if error is detected in non-boot areas)
#pragma config BOOTPOR = OFF    // Boot on CRC Enable bit (CRC on boot will not run)

// CONFIG12
#pragma config BCRCPOLT = hFF   // Boot Sector Polynomial for CRC on boot bits 31-24 (Bits 31:24 of BCRCPOL are 0xFF)

// CONFIG13
#pragma config BCRCPOLU = hFF   // Boot Sector Polynomial for CRC on boot bits 23-16 (Bits 23:16 of BCRCPOL are 0xFF)

// CONFIG14
#pragma config BCRCPOLH = hFF   // Boot Sector Polynomial for CRC on boot bits 15-8 (Bits 15:8 of BCRCPOL are 0xFF)

// CONFIG15
#pragma config BCRCPOLL = hFF   // Boot Sector Polynomial for CRC on boot bits 7-0 (Bits 7:0 of BCRCPOL are 0xFF)

// CONFIG16
#pragma config BCRCSEEDT = hFF  // Boot Sector Seed for CRC on boot bits 31-24 (Bits 31:24 of BCRCSEED are 0xFF)

// CONFIG17
#pragma config BCRCSEEDU = hFF  // Boot Sector Seed for CRC on boot bits 23-16 (Bits 23:16 of BCRCSEED are 0xFF)

// CONFIG18
#pragma config BCRCSEEDH = hFF  // Boot Sector Seed for CRC on boot bits 15-8 (Bits 15:8 of BCRCSEED are 0xFF)

// CONFIG19
#pragma config BCRCSEEDL = hFF  // Boot Sector Seed for CRC on boot bits 7-0 (Bits 7:0 of BCRCSEED are 0xFF)

// CONFIG20
#pragma config BCRCEREST = hFF  // Boot Sector Expected Result for CRC on boot bits 31-24 (Bits 31:24 of BCRCERES are 0xFF)

// CONFIG21
#pragma config BCRCERESU = hFF  // Boot Sector Expected Result for CRC on boot bits 23-16 (Bits 23:16 of BCRCERES are 0xFF)

// CONFIG22
#pragma config BCRCERESH = hFF  // Boot Sector Expected Result for CRC on boot bits 15-8 (Bits 15:8 of BCRCERES are 0xFF)

// CONFIG23
#pragma config BCRCERESL = hFF  // Boot Sector Expected Result for CRC on boot bits 7-0 (Bits 7:0 of BCRCERES are 0xFF)

// CONFIG24
#pragma config CRCPOLT = hFF    // Non-Boot Sector Polynomial for CRC on boot bits 31-24 (Bits 31:24 of CRCPOL are 0xFF)

// CONFIG25
#pragma config CRCPOLU = hFF    // Non-Boot Sector Polynomial for CRC on boot bits 23-16 (Bits 23:16 of CRCPOL are 0xFF)

// CONFIG26
#pragma config CRCPOLH = hFF    // Non-Boot Sector Polynomial for CRC on boot bits 15-8 (Bits 15:8 of CRCPOL are 0xFF)

// CONFIG27
#pragma config CRCPOLL = hFF    // Non-Boot Sector Polynomial for CRC on boot bits 7-0 (Bits 7:0 of CRCPOL are 0xFF)

// CONFIG28
#pragma config CRCSEEDT = hFF   // Non-Boot Sector Seed for CRC on boot bits 31-24 (Bits 31:24 of CRCSEED are 0xFF)

// CONFIG29
#pragma config CRCSEEDU = hFF   // Non-Boot Sector Seed for CRC on boot bits 23-16 (Bits 23:16 of CRCSEED are 0xFF)

// CONFIG30
#pragma config CRCSEEDH = hFF   // Non-Boot Sector Seed for CRC on boot bits 15-8 (Bits 15:8 of CRCSEED are 0xFF)

// CONFIG31
#pragma config CRCSEEDL = hFF   // Non-Boot Sector Seed for CRC on boot bits 7-0 (Bits 7:0 of CRCSEED are 0xFF)

// CONFIG32
#pragma config CRCEREST = hFF   // Non-Boot Sector Expected Result for CRC on boot bits 31-24 (Bits 31:24 of CRCERES are 0xFF)

// CONFIG33
#pragma config CRCERESU = hFF   // Non-Boot Sector Expected Result for CRC on boot bits 23-16 (Bits 23:16 of CRCERES are 0xFF)

// CONFIG34
#pragma config CRCERESH = hFF   // Non-Boot Sector Expected Result for CRC on boot bits 15-8 (Bits 15:8 of CRCERES are 0xFF)

// CONFIG35
#pragma config CRCERESL = hFF   // Non-Boot Sector Expected Result for CRC on boot bits 7-0 (Bits 7:0 of CRCERES are 0xFF)
#endif

#include <xc.h>
#include <stdio.h>

#define _XTAL_FREQ 64000000UL

// Never called, logically
void __interrupt(irq(default),base(8)) Default_ISR(){}

// UART3 Transmit
void putch(char c) {
	while(!U3TXIF);		// Wait or Tx interrupt flag set
	U3TXB = c;			// Write data
}

// UART3 Recive
int getch(void) {
	while(!U3RXIF);		// Wait for Rx interrupt flag set
	return U3RXB;		// Read data
}
// ======================  wiring define  ============================

#define		SYNC	RC3
#define		TEST	RD0
#define		PHI2	RD2
#define		PHI1	RD3

// ======================  Global ram and rom  =======================
#define ROM_SIZE	0x0f00		// program  ROM Must allocate contiguously in this order and size
#define B_MEM_SIZE	0x1000		// bank ROM     so that Program memory bank functional.
#define M_RAM_SIZE	0x800		// main RAM
#define S_RAM_SIZE	0x800		// status RAM

#define P_MEM_start	0x06		// program memory 0600-14FF
#define B_MEM_start	0x15		// bank ram 1500-24FF
#define M_RAM_start	0x25		// main ram 2500-2CFF
#define S_RAM_start	0x2D		// status ram 2D00-34FF
#define SRC_start	0x35		// SRC array 3500-350F

// willing to place work area into access bank(0x500-0x55f)

static unsigned char	P_MEM[ROM_SIZE]   __at(P_MEM_start * 0x100);	// Program memory
static unsigned char	B_MEM[B_MEM_SIZE] __at(B_MEM_start * 0x100);	// bank memory
static unsigned char	M_RAM[M_RAM_SIZE] __at(M_RAM_start * 0x100);	// main data memory
static unsigned char	S_RAM[S_RAM_SIZE] __at(S_RAM_start * 0x100);	// status memory
static unsigned char	SRC_ARRAY[8] __at(SRC_start * 0x100);			// SRC for 8RAMs

static unsigned char	BANK_00	__at(0x500);	// Port 0 out data (Program memory bank number)
static unsigned char	PORT_01	__at(0x501);	// Port 1 out data
static unsigned char	PORT_02 __at(0x502);	// Port 2 out data
static unsigned char	PORT_03	__at(0x503);	// Port 3 out data
static unsigned char	WPM_01	__at(0x504);	// WPM high or low nibble
static unsigned char	RPM_01	__at(0x505);	// RPM high or low nibble
static unsigned char	C_M1	__at(0x506);	// CM-ROM0 in M1 state(first cycle of 4040)
static unsigned char	OPR		__at(0x507);	// OPR for M1 cycle
static unsigned char	OPA		__at(0x508);	// OPA for M2 cycle
static unsigned char	OPAC	__at(0x509);	// complemented OPA
static unsigned char	OPA2	__at(0x50a);	// lower 2 bit of OPAC
static unsigned char	CMRAM0	__at(0x50b);	// CM-RAM data shown in A3 state
static unsigned char	CMRAM1	__at(0x50c);	// CMRAM1 = CMRAM0>>1
static unsigned char	SRC		__at(0x50d);	// SRC data shown in X2-3 state
static unsigned char	ROM_01	__at(0x50e);	// ROM_01 0:lower nibble 1:higher nibble
static unsigned char	ROMIN1	__at(0x50f);	// Console received data
static unsigned char	ROMOUT1	__at(0x510);	// Console out data
static unsigned char	PC1		__at(0x511);	// saved PCL
static unsigned char	PC2		__at(0x512);	// saved PCM
static unsigned char	PC3		__at(0x513);	// saved PCH
static unsigned char	ACC		__at(0x514);	// saved ACC
static unsigned char	NUM0F	__at(0x515);	// constant 0x0f

static unsigned int		PCX		__at(0x520);
static unsigned char	PC2X	__at(0x520);
static unsigned char	PC3X	__at(0x521);
static unsigned char	OPRX	__at(0x523);
static unsigned char	OPAX	__at(0x524);
static unsigned char	SRCX	__at(0x525);
static unsigned char	ACCX	__at(0x526);
static unsigned char	CMRAMX	__at(0x527);

static unsigned int	i,j;

#define	running_monitor
const unsigned char rom[] __at(0x10000) = {
//#include "loop4004.txt"
#include "vtl.txt"
//#include "calc.txt"
//#include "emu.txt"
};

// ======================  main routine  =======================================
void main(void) {

	// System initialize
	OSCFRQ		= 0x08;			// 64MHz internal OSC

#define	PORT_IN	  0b10111111;
#define PORT_OUT  0b10110000;
	ANSELA		= 0;			// Disable analog function
	LATA		= 0b11111111;
	TRISA		= PORT_IN;		// UART Tx will be OUT
	WPUA		= 0xff;			// weak pull up

	ANSELB		= 0;			// Disable analog function
	TRISB		= 0xff;			// Set as input
	WPUB		= 0xff;			// weak pull up

	ANSELC		= 0;			// Disable analog function
	LATC		= 0xff;
	TRISC		= 0xfe;			// RC0 output others input
	WPUC		= 0xff;			// weak pull up

	ANSELD		= 0;			// Disable analog function
	LATD		= 0b11100011;
	TRISD		= 0b11100011;	// RESET PHI1 PHI2 will be OUT
	WPUD		= 0xff;			// weak pull up
	LATD		= 0;

	ANSELE		= 0;			// Disable analog function
	TRISE		= 0xff;			// Set as input
	WPUE		= 0xff;			// weak pull up

//======== Generate 2 pahse CPU clock using PWM ================================
	ANSELD3		= 0;		// Disable analog function
	TRISD3		= 0;		// PWM output pin
	RD3PPS		= 0x1C;		// PWM3S1P1_OUT => RD3 => PHI1

	ANSELD2		= 0;		// Disable analog function
	TRISD2		= 0;		// PWM output pin
	RD2PPS		= 0x1A;		// PWM2S1P1_OUT => RD2 => PHI2

	PWM3CON		= 0x00;		// EN=0, LD=0
	PWM2CON		= 0x00;		// EN=0, LD=0
	PWM3CLK		= 0x02;		// Clock source Fsoc
	PWM2CLK		= 0x02;		// Clock source Fsoc
	PWM3GIE		= 0x00;		// interrupt disable
	PWM2GIE		= 0x00;		// interrupt disable
	PWM3S1CFG	= 0x00;		// (POL1, POL2)= 0, PPEN = 0 MODE = 0 (Left Aligned mode)
	PWM2S1CFG	= 0x00;		// (POL1, POL2)= 0, PPEN = 0 MODE = 0 (Left Aligned mode)

	PWM3PR		= 86;		// period 1359.375 nsec ((86+1)*15.625) 736kHz
	PWM2PR		= PWM3PR;	// same as PHI1
	PWM3S1P1	= 25;		// PHI1 width  390.625 nsec (25*15.625)
	PWM2S1P1	= PWM3S1P1;	// same as PHI1

	PWM3CON		= 0x84;		// EN=1, LD=1 it takes 125nsec(=2*62.5)
							// instruction cycle for delay
	NOP();					// 1 NOP(); takes 62.5nsec
	NOP();					// 2
	NOP();					// 3
	NOP();					// 4
	NOP();					// 5
	NOP();					// 6
	NOP();					// 7
	NOP();					// 8
	NOP();					// 9
	NOP();					// 10
	NOP();					// 11
	PWM2CON		= 0x84;		// 12 13 Time difference 62.5*13 = 812.5 nsec
//==============================================================================
	// UART3 initialize
	U3BRG		= 416;		// 9600bps @ 64MHz
	U3RXEN		= 1;		// Receiver enable
	U3TXEN		= 1;		// Transmitter enable

	// UART3 Receiver
	ANSELA7		= 0;		// Disable analog function
	TRISA7		= 1;		// RX set as input
	U3RXPPS		= 0x07;		// RA7->UART3:RX3;

	// UART3 Transmitter
	ANSELA6		= 0;		// Disable analog function
	LATA6		= 1;		// Default level
	TRISA6		= 0;		// TX set as output
	RA6PPS		= 0x26;		// RA6->UART3:TX3;
	U3ON		= 1;		// Serial port enable

	printf("\r\nEMU4004 clock speed %3.0fkHz\r\n",(float)64000/(PWM3PR+1));
	printf("PWM3 -> RD3 -> PHI1 width %3.0f nsec\r\n",(float)PWM3S1P1*15.625);
	printf("PWM2 -> RD2 -> PHI2 width %3.0f nsec\r\n\n",(float)PWM2S1P1*15.625);
	printf("PIC UART display runnung monitor\r\n");
	printf("4004/4040 Software Serial on another port\r\n\n");

//	================= Serial console routing ===============
//	========== CLC pin assign ===========
	CLCIN0PPS	= 0x11;		// CLCIN0PPS <- RC1 <- Serial Rx
    CLCSELECT	= 2;		// select CLC3

	CLCnSEL0	= 0;		// CLCIN1PPS <- RC1 <- Rx
	CLCnSEL1	= 127;		// NC
	CLCnSEL2	= 127;		// NC
	CLCnSEL3	= 127;		// NC

	CLCnGLS0	= 0x01;		// invert RC1
	CLCnGLS1	= 0x10;		// 1(0 inv)
	CLCnGLS2	= 0x10;		// 1(0 inv)
	CLCnGLS3	= 0x40;		// 1(0 inv)

	CLCnPOL		= 0x00;		// Not inverted
	CLCnCON		= 0x82;		// 4 input AND, no interrupt

	RD0			= 0;
	TRISD0		= 0;		// RD0 output
	RD0PPS		= 3;		// CLC3 -> RD0 ->TEST)
//==============================================
	// Unlock IVT
	IVTLOCK	= 0x55;
	IVTLOCK	= 0xAA;
	IVTLOCKbits.IVTLOCKED = 0x00;

	// Default IVT base address
	IVTBASE 	= 0x000008;

	// Lock IVT
	IVTLOCK	= 0x55;
	IVTLOCK	= 0xAA;
	IVTLOCKbits.IVTLOCKED = 0x01;
//======== set up memory ==================================
	for(i = 0; i < sizeof rom; i++) {		// transfer rom data to Program memory
		P_MEM[i] = rom[i];				// rom is slow read out
	}

	for(i = 0; i < sizeof SRC_ARRAY; i++) {				// each RAM chip have independent SRC register
		SRC_ARRAY[i] = 0;				// it sgould be cleared at reset
	}

	ROM_01	= 0;	// set console order 0
	NUM0F	= 0x0f;	// set constant 0x0f
	BANK_00	= 00;	// set bank no 00
	PORT_01	= 00;	// set port 01 00
	PORT_02	= 00;	// set port 02 00
	PORT_03	= 00;	// set port 03 00
	WPM_01	= 0x00;	// set WPM order low nibble
	RPM_01	= 0x00;	// set RPM order low nibble
	CMRAM0	= 1;	// CM-RAM set to select RAM0
	CMRAM1	= 0;	// CM-RAM set to select RAM0

// 4004 start

	while(SYNC);
	while(!SYNC);

	while(!PHI2);	// timing adjust for RESET release timing
	while(PHI2);	// timing adjust for RESET release timing

	IOCIF	= 0;
	IOCCP3	= 1;	// SYNC (RC3) positive edge intrurrupt
	IOCIE	= 1;	// Inturrut on change enable
	GIE		= 1;	// Global interrupt enable
	LATD4	= 1;	// release RESET and start 4004/4040

	while(1) {
#ifdef running_monitor
		printf("\e[H");
		for (i=0; i<23; i++) {
			for (j=0; j<4; j++) {
				asm("movff	_PC2,_PC2X");
				asm("movff	_PC3,_PC3X");
				asm("comf	_OPA,w,c");
				asm("movwf	_OPAX,c");
				asm("movf	_ACC,w,c");
				asm("andlw	0x01f");
				asm("movwf	_ACCX,c");
				asm("movff	_SRC,_SRCX");
				asm("movff	_CMRAM0,_CMRAMX");
				printf(" %03X %02X %02X %01X%02X %01X%01X%01X%01X ",PCX,OPAX,ACCX,CMRAMX,SRCX,PORT_03,PORT_02,PORT_01,BANK_00);
			}
			printf("\r\n");
		}
#endif
	}
}
//==============================================================================
void __interrupt(irq(IOC),base(8)) SYNC_ISR(){
//	--------------- A1 state common to all instruction, lower 4bit of PC -------
	IOCCF	= 0;				// clear IOC port C inturrupt flag

	/* some time space here but nothing to do  */

	while (!PHI2);				//    wait for A1 end
	asm("comf	PORTA,w,c");	// 01 complemet data bus value and store to WREG
	asm("andlw	0x0f");			// 02 WREG &= 0x0f
	FSR0L	= WREG; 			// 03 save tentitive FSR0L
	PC1		= WREG;				// 04 save PCL for running monitor
	while (PHI2);				//    wait for A2
//	--------------- A2 state common to all instruction, middle 4bit of PC ------

	/* some time space here but nothing to do  */

	while (!PHI2);				//    wait for A2 end
	asm("comf	PORTA,w,c");	// 01 complement data bus value and store to WREG
	asm("andlw	0x0f");			// 02 WREG &= 0x0f
	asm("swapf	WREG,w");		// 03 swap nibble
	asm("addwf	FSR0L,f,c");	// 04 FSR0L += WREG
	PC2		= FSR0L;			// 05 06 save PCL for running monitor
//	while (PHI2);				//    wait for A3 (no need)
//	--------------- A3 state common to all instruction, upper 4bit of PC -------
	NOP();						// 07 timing adjust *** most crirical timing ***
	NOP();						// 08 timing adjust *** have to wait A3 addres comeout ***
	NOP();						// 09 timing adjust *** also there's many process to do ***
	NOP();						// 10 timing adjust *** so waiting PHI2 leading edge will be too late ***

	asm("comf	PORTA,w,c");	// 11 complement data bus value and store to WREG
	asm("andlw	0x0f");			// 12 WREG &= 0x0f
	PC3		= WREG;				// 13 save PCH for running monitor
	asm("cpfsgt	_NUM0F,c");		// 14 is it banked area? (if (PC3<0x0f) then skip next instruction)
	asm("addwf	_BANK_00,w,c");	// 15 modify ram address in PIC
	asm("addlw	high _P_MEM");	// 16 WREG += P_MEM (it's 0x06)
	FSR0H	= WREG;				// 17 save FSR0H
	asm("comf	indf0,w,c");	// 18 complement Program memory and store to WREG
	OPA		= WREG;				// 19 save OPA (upper 4bit will be omotted)
	asm("swapf	WREG,w");		// 20 swap nibble
	OPR 	= WREG;				// 21 save OPR (upper 4bit will be omotted)
	LATA	= WREG;				// 22 prepare OPR for M1 state
//	while (!PHI2);				//    wait for A3 end

	asm("comf	PORTC,w,c");	// 01 read in CM-RAM
	asm("andlw	0xf0");			// 02 mask (CM-RAM connected to RC4-7)
	asm("swapf	wreg,w");		// 03 swap nibble
	CMRAM0	= WREG;				// 04 save CMRAM0 for IO operation
	asm("rrncf	wreg,w");		// 05 CMRAM1 = CMRAM0>>1
	asm("andlw	0x07");			// 06
	CMRAM1	= WREG;				// 07 save CMRAM1
	NOP();						// 08
	NOP();						// 09
//	while (PHI2);				//    wait for M1 (no need)
//	--------------- M1 state common to all instruction, out OPR to data bus ----
	TRISA	= PORT_OUT;			// 10 11 data bus output
	asm("comf	_OPA,w,c");		// 12 complement OPA for easier handling in FW
	asm("andlw	0x0f");			// 13 mask 4bit
	asm("movwf	_OPAC,c");		// 14 save as OPAC
	asm("andlw	0x03");			// 15 OPAC lower 2 bit for WRx RDx instruction
	asm("movwf	_OPA2,c");		// 16 save as OPA2
	FSR0L	= CMRAM1;			// 17 18
	FSR0H	= SRC_start;		// 19 20
	SRC		= INDF0;			// 21 22
//	whhile (!PHI2);				//    wait for M1 end
//	TRISA	= PORT_IN;			//    keep databus output throughout M2 end
	asm("comf	PORTB,w,c");	// 01 read CM-ROM during M1(if it's 1 first cycle of 4040)
	asm("andlw	0x02");			// 02
	C_M1	= WREG;				// 03 save it
	NOP();						// 04
//	while (PHI2);				//    wait for M2 (no need)
	LATA	= OPA;				// 05 06 out OPA for M2 state
//	--------------- M2 state common entrance and branch during M2 ------
//	TRISA	= PORT_OUT;			//    keep databus output throughout M2 end
	NOP();						// 07 wait for CM-ROM0 come out
	NOP();						// 08 wait for CM-ROM0 come out
	NOP();						// 09 wait for CM-ROM0 come out
	asm("btfsc	PORTB,1,c");	// 10 if RB1(CM-ROM0) = L then IO(read or write) instruction
	asm("bra	not_io");		// 11 (12) if RB1(CM-ROM0) = H then goto not IO instruction
	asm("btfss	_OPAC,3,c");	// 12 check OPAC bit 3 if it's 1 then IO read instruction
	asm("bra	IO_write");		// 13 (14) if it's 0 then IO write instruction
//	=============== IO read instruction ========================================
//	--------------- M2 state continue for IO read operation --------------------
	asm("IO_read: ");
	NOP();						// 14
	NOP();						// 15
	NOP();						// 16
	NOP();						// 17
	NOP();						// 18
	NOP();						// 19
	NOP();						// 20
	NOP();						// 21
//	while (!PHI2);				//    wait for M2 end
	TRISA	= PORT_IN;			// 01 02 data bus input
//	while (PHI2);				//    wait for X1 (no need)
//	--------------- X1 state for IO read instruction, branch each instruction --
	NOP();						// 03
	NOP();						// 04
	asm("movf	_OPAC,w,c");	// 05 read out OPAC
	asm("btfsc	WREG,2,c");		// 06 if bit2=0 then (SBM RDM RDR ADM)
	asm("bra	RD0123");		// 07 (08) if bit2=1 then status ram read (RD0 RD1 RD2 RD3)
	asm("xorlw	0x0a");			// 08 if it's 0xah?
	asm("bnz	M_RAM_read");	// 09 (10) it it's 0xah then main ram read (SBM RDM ADM)
//	--------------- X1 state for RDR (read from ROM IO port) code EA -----------
	asm("RDR:");
	asm("comf	PORTA,w,c");	// 10 complement PORTA(it shuld be Acc) sand store to WREG
	ACC		= WREG;				// 11 save Acc for running monitor
	asm("btfss	_SRC,7,c");		// 12 IO address decode if bit7 of SRC?
	asm("bra	UART_status");	// 13 (14) if it's 0 then UART read status register
	asm("btfss	_ROM_01,0,c");	// 14 input order ?
	asm("bra	UART_0");		// 15 (16) if it's 0 then Inout lower 4bit
//	----------------------------
	asm("UART_1:");				//    input upper 4bit
	asm("swapf	_ROMIN1,w,c");	// 16 get upper 4bit from saved received data
	asm("bcf	_ROM_01,0,c");	// 17 next time lower 4bit
	asm("bra	X1_exit");		// 18 19 goto exit
//	----------------------------
	asm("UART_status:");		//    check console status
	asm("comf	PIR9,w,c");		// 15 read UART status register
	asm("clrf	_ROM_01,c");	// 16 next time lower 4bit( both input output)
	asm("bra	X1_exit");		// 17 18 goto exit
//	----------------------------
	asm("UART_0:");				//    input lower 4bit
	asm("movlb	2");			// 17 have to set BSR
	asm("comf	U3RXB,w,b");	// 18 read UART receive buffer register
	ROMIN1	= WREG;				// 19 temporary save for read upper 4bit
	asm("bsf	_ROM_01,0,c");	// 20 next time upper 4bit

	asm("X1_exit:");
	LATA	= WREG;				// 21 prepare data bus
	while (!PHI2);				//    wait for X1 end
	asm("comf	PORTA,w,c");	//    complement PORTA(it shuld be Acc) sand store to WREG
	ACC		= WREG;				//    save Acc for running monitor
	while (PHI2);				//    wait for X2
//	--------------- X2 state for IO read instruction ---------------------------
	TRISA	= PORT_OUT;			//    data bus output
	while (!PHI2);				//    wait for X2 end
	while (PHI2);				//    wait for X2 end
	TRISA	= PORT_IN;			//    databus input ***** it seemed extend PORT OUT duration for IO instruction
	asm("retfie	1");			//    return from inturrupt
//	--------------- X1 state for SBM RDM ADM instruction code E8 E9 EB ---------
	asm("M_RAM_read:");
	FSR0L	= SRC;				// 11 12 set main RAM index
	asm("movf	_CMRAM1,w,c");	// 13
	asm("addlw	high _M_RAM");  // 14
	FSR0H	= WREG;				// 15 set main RAM address
	asm("comf	indf0,w,c");	// 16 complement main RAM and store to WREG
	asm("bra	X1_exit");		// 17 18
//	--------------- X1 state for RDx(0-3) instruction code EC ED EE EF ---------
	asm("RD0123:");				// 09
	asm("movf	_SRC,w,c");		// 10 set status RAM index
	asm("andlw	0xf0");			// 11 mask lower 4 bit
	asm("addwf	_OPA2,w,c");	// 12 modify status RAM index
	FSR0L	= WREG;				// 13 set status RAM index
	asm("movf	_CMRAM1,w,c");	// 14 get RAM bank no
	asm("addlw	high _S_RAM");  // 15 modify status RAM index
	FSR0H	= WREG;				// 16 set status RAM address
	asm("comf	indf0,w,c");	// 17 complement main RAM and store to WREG
	asm("bra	X1_exit");		// 18 19

//	=============== IO write instruction =======================================
//	--------------- M2 state continue for IO write instruction -----------------
	asm("IO_write: ");			//
	NOP();						// 15
	NOP();						// 16
	NOP();						// 17
	NOP();						// 18
	NOP();						// 19
	NOP();						// 20
	NOP();						// 21
//	while (!PHI2);				//    wait for M1 end
	TRISA	= PORT_IN;			// 01 02 data bus input
//	while (PHI2);				// wait for X1 (already exceed PHI2 falling edge)
//	--------------- X1 state for IO write instruction branch each instruction --
	asm("movf	_OPAC,w,c");	// 03 Z N flag affected
	asm("bz		WRM");			// 04 (05) if it's 0 then main RAM write
	asm("btfsc	WREG,2,c");		// 05 if bit2 of OPAC?
	asm("bra	WR0123");		// 06 (07) if it's 1 status RAM write
	asm("xorlw	0b01");			// 07 if it's 1?
	asm("bz		WMP");			// 08 (09) if it'S 1 then RAM port out
	asm("xorlw	0b11");			// 09 if it's 2? (note: there is XOR branch magic)
	asm("bz		WRR");			// 10 (11) if it's 2 then ROM port out
//	--------------- X1 state for WPM instruction Program memory write code E3 --
	asm("WPM:");				// 11 currently not implemented
	while (!PHI2);				//    wait for X1 end
	asm("comf	PORTA,w,c");	// 01 complement PORTA(it shuld be Acc) sand store to WREG
	ACC		= WREG;				// 02 save Acc for running monitor
	while (PHI2);				//    wait for X2
//	--------------- X2 state for WPM instruction Program memory write code E3 --
	FSR0L	= SRC;				// 08 09 set FSR0L
	asm("movf	_BANK_00,w,c");	// 10 WREG = BANK_00
	asm("addlw	high _B_MEM");	// 11 add start address of bank ram
	FSR0H	= WREG;				// 12 set FSR0H
	asm("movf	_WPM_01,w,c");	// 13 
	asm("bz		WPM_0");		// 14 15

	asm("WPM_1:");				// 15 write upper nibble
	asm("movf	indf0,w,c");	// 16 read in target byte
	asm("andlw	0x0f");			// 17 mask upper nibble
	asm("movwf	indf0,c");		// 18 save back
	asm("comf	PORTA,w,c");	// 19 read data bus
	asm("swapf	wreg,w");		// 20 swap nibble
	asm("andlw	0xf0");			// 21 mask lower nibble
	asm("addwf	indf0,f,c");	// 22 INDF0 += WREG
	asm("clrf	_WPM_01,c");	// 23 next time lower nibble
	asm("retfie	1");			//    return from inturrupt

	asm("WPM_0:");				// 16 write lower nibble
	asm("movf	indf0,w,c");	// 17 read in target byte
	asm("andlw	0xf0");			// 18 masl lower nibble
	asm("movwf	indf0,c");		// 19 save back
	asm("comf	PORTA,w,c");	// 20 read data bus
//	asm("swapf	wreg,w");		//    no swap
	asm("andlw	0x0f");			// 21 mask upper nibble
	asm("addwf	indf0,f,c");	// 22 INDF0 += WREG
	asm("setf	_WPM_01,c");	// 23 next time upper nibble
	asm("retfie	1");			//    return from inturrupt
//	--------------- X1 state for WMP instruction RAM port out code E1 ----------
	asm("WMP:");				// 10
	while (!PHI2);				//    wait for X1 end
	asm("comf	PORTA,w,c");	//    complement PORTA(it shuld be Acc) sand store to WREG
	ACC		= WREG;				//    save Acc for running monitor
	while (PHI2);				//    wait for X2
//	--------------- X2 state for WMP instruction RAM port out code E1 ----------
	asm("movf	_SRC,w,c");		// 07 read SRC to WREG
	asm("andlw	0xc0");			// 08 mask other than chip select
	asm("bz		bank_sw");		// 09 (10)if it's 0 then bank switch
	asm("xorlw	0xc0");			// 10 if it's 0xc0?
	asm("bz		soft_serial");	// 11 (12) then goto soft serial
	asm("xorlw	0x80");			// 12 if it's 0x40? (xor branch magic)
	asm("bz		port_01");		// 13 (14) then goto port_01
	asm("xorlw	0x40");			// 14 if it's 0x80? (xor branch magic)
	asm("bz		port_02");		// 15 (16) then goto port_02
	asm("retfie	1");			// 16 otherwise return from interrupt
//	--- out RAM #1 port
	asm("port_01:");
	asm("comf	PORTA,w,c");	// 15 complement databus and store to WREG
	asm("andlw	0x0f");			// 16
	PORT_01	= WREG;				// 17
	asm("retfie	1");			//    return from inturrupt
// -- out RAM #2 port
	asm("port_02:");
	asm("comf	PORTA,w,c");	// 17 complement databus and store to WREG
	asm("andlw	0x0f");			// 18
	PORT_02	= WREG;				// 19
	asm("retfie	1");			//    return from inturrupt
//	--- Program memory bank select (out RAM #0 port) -----
	asm("bank_sw:");			// 11
	asm("comf	PORTA,w,c");	// 12 complement databus and store to WREG
	asm("andlw	0x0f");
	BANK_00	= WREG;				// 13
	asm("retfie	1");			//    return from inturrupt
//  ---  software serial out (bit0 of RAM #3 port) -----
	asm("soft_serial:");
	asm("comf	PORTA,w,c");	// 13 complement databus and store to WREG
	asm("andlw	0x0f");
	PORT_03	= WREG;				// 14
	asm("btfss	wreg,0");		// 15 if bit0=1 then RAMOUT1
	asm("bra	RAMOUT0");		// 16 (17) if bit0=0 then RAMOUT0

	asm("RAMOUT1:");
	asm("bsf	LATC,0,c");		// 17 OUT "H"
	asm("retfie	1");			//    return from inturrupt

	asm("RAMOUT0:");
	asm("bcf	LATC,0,c");		// 18 OUT "L"
	asm("retfie	1");			//    return from inturrupt
//	--------------- X1 state for WRR instruction ROM port out code E2 ----------
	asm("WRR:");
	while (!PHI2);				//    wait for X1 end
	asm("comf	PORTA,w,c");	// 01 complement PORTA(it shuld be Acc) sand store to WREG
	ACC		= WREG;				// 02 save Acc for running monitor
	while (PHI2);				//    wait for X2
//	--------------- X2 state for WRR instruction ROM port out code E2 ----------
	asm("btfss	_SRC,7,c");		// 08 if bit 7 of SRC=1 then ROM PORT OUT operation
	asm("retfie	1");			// 09 (10) if bit 7 of SRC=0 then goto exit
	asm("btfss	_ROM_01,1,c");	// 10 if ROM_01 flag bit1=1 then upper 4bit output
	asm("bra	ROMOUT_0");		// 11 (12) if ROM_01 flag bit1=0 then upper 4bit output

	asm("ROMOUT_1:");			//    merge upper/lower 4bit and transmit
	asm("comf	PORTA,w,c");	// 12 complement databus and store to WREG
	asm("andlw	0x0f");			// 13 WREG &= 0x0f
	asm("swapf	wreg,w");		// 14 swap nibble
	asm("iorwf	_ROMOUT1,w,c");	// 15 WREG |= RAMOUT1(saved lower 4bit)
	U3TXB	= WREG;				// 16 ransmit WREG
	asm("bcf	_ROM_01,1,c");	// 17 next time lower 4bit
	asm("retfie	1");			//    return from inturrupt

	asm("ROMOUT_0:");
	asm("comf	PORTA,w,c");	// 13 complement databus and store to WREG
	asm("andlw	0x0f");			// 15 WREG &= 0x0f
	ROMOUT1	= WREG;				// 16 save lower 4bit data
	asm("bsf	_ROM_01,1,c");	// 17 next time upper 4bit
	asm("retfie	1");			//    return from inturrupt
//	--------------- X1 state for WRM instruction main memory write code E0 -----
	asm("WRM:");
	while (!PHI2);				// 06 wait for X1 end
	asm("comf	PORTA,w,c");	// 07 complement PORTA(it shuld be Acc) sand store to WREG
	ACC		= WREG;				// 08 save Acc for running monitor
	while (PHI2);				//    wait for X2
//	--------------- X2 state for WRM instruction main memory write code E0 -----
	FSR0L	= SRC;				// 08 09 set main RAM index
	asm("movf	_CMRAM1,w,c");	// 10
	asm("addlw	high _M_RAM");  // 11
	FSR0H	= WREG;				// 12 set main RAM address
	while (!PHI2);				//    wait for X1 end
	asm("comf	PORTA,w,c");	// 01 complement data bus and store to WREG
	INDF0	= WREG;				// 02 write into main RAM
	asm("retfie	1");			//    return from inturrupt
//	--------------- X1 state for WRx(0-3) instruction code E4 E5 E6 E7 ---------
	asm("WR0123:");
	while (!PHI2);				// 08 wait for X1 end
	asm("comf	PORTA,w,c");	// 09 complement PORTA(it shuld be Acc) sand store to WREG
	ACC		= WREG;				// 10 save Acc for running monitor
	while (PHI2);				//    wait for X2
//	--------------- X2 state for WRx(0-3) instruction code E4 E5 E6 E7 ---------
	asm("movf	_SRC,w,c");		// 08 set status RAM index
	asm("andlw	0xf0");			// 09 mask lower 4 bit
	asm("addwf	_OPA2,w,c");	// 10 modify status RAM index
	FSR0L	= WREG;				// 11 set status RAM index
	asm("movf	_CMRAM1,w,c");	// 12 get RAM bank no
	asm("addlw	high _S_RAM");  // 13 modify status RAM index
	FSR0H	= WREG;				// 14 set status RAM address
	while (!PHI2);				//    wait for X1 end
	asm("comf	PORTA,w,c");	// 01 complement data bus and store to WREG
	INDF0	= WREG;				// 02 write to status RAM
	asm("retfie	1");			//    return from inturrupt
//	=============== usual instruction (other than IO) ==========================
//	--------------- M2 state continue for non IO instruction -------------------
	asm("not_io:	");			// 13
	asm("movf	_C_M1,w,c");	// 13 if CM-RAM0 at M1 was 1(L)? then it's 4040 first OP code
	asm("bz		not_4040_1");	// 14 (15) otherwise it's 4004 or second OP code
	asm("comf	_OPA,w,c");		// 15 check OP code
	asm("xorlw	0x0e");			// 16 if it's 0x0e? (RPM OP code)
	asm("bz		RPM");			// 17 (18) then it's RPM instruction
	asm("xorlw	0x0c");			// 18 0x0E XOR 0x02 = 0x0C
	asm("bz		BBS");			// 19 
	asm("bra	not_4040rpm");	// 20 21
//	=============== BBS instruction ============================================
	asm("BBS:");				// 19
	NOP();						// 19
	NOP();						// 20
	NOP();						// 21
//	while (!PHI2);				//    wait for M2 end
	TRISA	= PORT_IN;			//    data bus INPUT
	while (PHI2);				//    wait for X1
	asm("bra	src_io");		// 01
//	=============== RPM instruction ============================================
	asm("RPM:");				// 19
	NOP();						// 19
	NOP();						// 20
	NOP();						// 21
//	while (!PHI2);				//    wait for M2 end
	TRISA	= PORT_IN;			//    data bus INPUT
	while (PHI2);				//    wait for X1
//	--------------- X1 state RPM instruction ---------------------------------  
	FSR0L	= SRC;				// 08 09
	asm("movf	_BANK_00,w,c");	// 10 read Bank no
	asm("addlw	high _B_MEM");	// 11 calc start address of bank MEM
	FSR0H	= WREG;				// 12 set index
	asm("comf	indf0,w,c");	// 13 read out program memory
	asm("btfsc	_RPM_01,0,c");	// 14 lower 4bit? then skip swap
	asm("swapf	wreg,w");		// 15 upper 4bit? swap wreg
	asm("comf	_RPM_01,f,c");	// 16 modify instruction order
	asm("bra	X1_exit");		// 17 18 goto exit
//	=============== other than RPM instruction =================================
	asm("not_4040_1:");	
	NOP();						// 16
	NOP();						// 17
	NOP();						// 18
	NOP();						// 19
	NOP();						// 20
	NOP();						// 21
	asm("not_4040rpm:");		// 
//	while (!PHI2);				//    wait for M2 end
	TRISA	= PORT_IN;			//    data bus INPUT
	while (PHI2);				//    wait for X1
//	--------------- X1 state for no IO instruction -----------------------------
	asm("comf	_OPA,w,c");		// 08 check OP code (it's complemented)
	asm("andlw	0xf1");			// 09 ma sk RRR field
	asm("xorlw	0x21");			// 10 if SRC command or second OP code 0b0010xxx1
	asm("bz		src_io");		// 11 (12) if it's 0x21 goto SRC operation
//	=============== CPU instruction (other than IO, SRC, RPM) ==================
//	--------------- X1 state continue for non SRC instruction-------------------
	while (!PHI2);				//    wait for X1 end
	asm("comf	PORTA,w,c");	// 12 complement PORTA(it shuld be Acc) sand store to WREG
	ACC		= WREG;				// 13 save Acc for running monitor
	asm("retfie	1");			// 14 return from inturrupt
//	=============== SRC instruction ============================================
//	--------------- X1 state continue for SRC instruction ----------------------
	asm("src_io:	");
	while (!PHI2);				// 13 wait for X1 end
	asm("comf	PORTA,w,c");	// 14 complement PORTA(it shuld be Acc) sand store to WREG
	ACC		= WREG;				// 15 save Acc for running monitor
	while (PHI2);				//    wait for X2
//	--------------- X2 state in SRC instruction --------------------------------
	NOP();						// 07 wait for CM-ROM come out
	NOP();						// 08 wait for CM-ROM come out
	NOP();						// 09 wait for CM-ROM come out
	NOP();						// 10 wait for CM-ROM come out
	asm("btfsc	PORTB,1,c");	// 11 if RB1 = L skip retfie and get SRC data
	asm("retfie	1");			// 12 if RB1 = H it's code 0010xxx1 in 2nd code, so return now
	asm("comf	PORTA,w,c");	// 13 complemet data bus and store to WREG
	asm("andlw	0x0f");			// 14 WREG &= 0x0f
	asm("swapf	WREG,w,c");		// 15 swap nibble(it's high nibble of SRC)
	SRC		= WREG;				// 16 save higher 4bit of SRC

	while (!PHI2);				// wait for X2 end
	while (PHI2);				// wait for X3
//	--------------- X3 state in SRC instruction --------------------------------
	NOP();						// 07 wait for SRC data come out
	NOP();						// 08 wait for SRC data come out
	NOP();						// 09 wait for SRC data come out
	NOP();						// 10 wait for SRC data come outs
	asm("comf	PORTA,w,c");	// 11 complement data bus and store to WREG
	asm("andlw	0x0f");			// 12 WREG &= 0x0f
	asm("iorwf	_SRC,f,c");		// 13 SRC	|= WREG;
	FSR0L	= CMRAM1;			// 14 15
	FSR0H	= SRC_start;		// 16 17
	INDF0	= SRC;				// 18 19
}
