#ifndef omap3_h
#define omap3_h
/*******************************************************************************
omap3h - Register definitions for TI BeagleBoard C4
           
THE SOFTWARE IS DELIVERED "AS IS" WITHOUT WARRANTY OR CONDITION OF ANY KIND, 
EITHER EXPRESS, IMPLIED OR STATUTORY. THIS INCLUDES WITHOUT LIMITATION ANY 
WARRANTY OR CONDITION WITH RESPECT TO MERCHANTABILITY OR FITNESS FOR ANY 
PARTICULAR PURPOSE, OR AGAINST THE INFRINGEMENTS OF INTELLECTUAL PROPERTY RIGHTS 
OF OTHERS.
           
This file may be freely used for commercial and non-commercial applications, 
including being redistributed with any tools.

If you find a problem with the file, please report it so that it can be fixed.

Created by Markos Chandras <chandram@cs.man.ac.uk>

*******************************************************************************/

extern int checkInt;
extern short channel1_flag;
extern short channel2_flag;
extern short channel3_flag;

#define REG32 (volatile unsigned int*)
#define FALSE 0
#define TRUE 1

/*##############################################################################
## MISC
##############################################################################*/


#define LINE_FEED              0xA
#define CARRIAGE_RETURN        0xD
#define PRINTABLE_START        0x20
#define PRINTABLE_END          0x7E


/* Default RAM Exception handlers */
#define E_UNDEFINED				(*(REG32 (0x4030CE24)))
#define E_SWI					(*(REG32 (0x4030CE28)))
#define E_PREFETCH				(*(REG32 (0x4030CE2C)))
#define E_DATA_ABRT				(*(REG32 (0x4030CE30)))
#define E_UNUSED				(*(REG32 (0x4030CE34)))
#define E_IRQ					(*(REG32 (0x4030CE38)))
#define E_FIQ					(*(REG32 (0x4030CE3C)))

/* modified */
/* GPTIMER REGISTERS */
//#define GPTI1					0x48040000
#define GPTI1					0x44E05000
#define GPTI2					0x49032000
#if 0
/* Timer Offsets */
#define GPTI_TIDR				0x0
#define GPTI_TIOCP_CFG 			0x10
#define GPTI_TISTAT				0x14
#define GPTI_TISR				0x18
#define GPTI_TIER				0x1C
#define GPTI_TWER				0x20
#define GPTI_TCLR				0x24
#define GPTI_TCRR				0x28
#define GPTI_TLDR				0x2C
#define GPTI_TTGR				0x30
#define GPTI_TWPS				0x34
#define GPTI_TMAR				0x38
#define GPTI_TCAR1				0x3C
#define GPTI_TSICR				0x40
#define GPTI_TCAR2				0x44
#define GPTI_TPIR				0x48
#define GPTI_TNIR				0x4C
#define GPTI_TCVR				0x50
#define GPTI_TOCR				0x54
#define GPTI_TOWR				0x58
#endif

#define RTC_BASE		0x44E3E000
#define RTC_OSC_REG		0x54
#define RTC_KICK0		0x6C
#define RTC_KICK1		0x70
#define DMTIMER2	 	0x48040000
#define DMTIMER1          	0x44E31000
#define DMTIMER1_CLKSEL		0x28
#define DMTIMER2_CLKSEL		0x8
#define DMTIMER1_32Khz		0x4
#define CM_PER                  0x44E00000
#define CM_PER_TIMER2		0x80
#define CM_WKUP_TIMER1_CLKCTRL  0xC4
#define CM_DPLL_REG		0x44E00500
#define CM_WKUP			0x44E00400
#define CM_WKUP_CLKSTCTRL	0x0
#define CM_RTC			0x44E00800
#define CM_RTC_RTC_CLKCTRL	0x0

#define CONTROL_MODULE 0x44E10000


#define GPTI_TIDR				0x0
#define GPTI_TIOCP_CFG 			0x10
#define GPTI_IRQ_EOI			0x20
#define GPTI_IRQSTATUS_RAW		0x24
#define GPTI_IRQSTATUS			0x28
#define GPTI_IRQENABLE_SET		0x2C
#define GPTI_IRQSTATUS_CLR		0x30
#define GPTI_IRQWAKEEN			0x34			
#define GPTI_TCLR				0x38	
#define GPTI_TCRR				0x3C
#define GPTI_TLDR				0x40
#define GPTI_TTGR				0x44
#define GPTI_TWPS				0x48
#define GPTI_TMAR				0x4C
#define GPTI_TCAR1				0x50
#define GPTI_TSICR				0x54
#define GPTI_TCAR2				0x58

/* modified */
/* Define MPU_INTC */
#define MPU_INTC				0x48200000

/* Interrupt Controller Offsets */
#define INTCPS_REVISION			0x0
#define INTCPS_SYSCONFIG 		0x10
#define INTCPS_SYSSTATUS 		0x14
#define INTCPS_SIR_IRQ 			0x40
#define INTCPS_SIR_FIQ 			0x44
#define INTCPS_CONTROL			0x48
#define INTCPS_PROTECTION		0x4C
#define INTCPS_IDLE				0x50
#define INTCPS_IRQ_PRIORITY		0x60
#define INTCPS_FIQ_PRIORITY		0x64
#define INTCPS_THRESHOLD		0x68
#define INTCPS_ITR0				0x80
#define INTCPS_MIR0				0x84
#define INTCPS_MIR_CLEAR0		0x88
#define INTCPS_MIR_SET0			0x8C
#define INTCPS_ISR_SET0			0x90
#define INTCPS_ISR_CLEAR0		0x94
#define INTCPS_PENDING_IRQ0		0x98
#define INTCPS_PENDING_FIQ0		0x9C
#define INTCPS_ITR1				0xA0
#define INTCPS_MIR1				0xA4
#define INTCPS_MIR_CLEAR1		0xA8
#define INTCPS_MIR_SET1			0xAC
#define INTCPS_ISR_SET1			0xB0
#define INTCPS_ISR_CLEAR1		0xB4
#define INTCPS_PENDING_IRQ1		0xB8
#define INTCPS_PENDING_FIQ1		0xBC
#define INTCPS_ITR2				0xC0
#define INTCPS_MIR2				0xC4
#define INTCPS_MIR_CLEAR2		0xC8
#define INTCPS_MIR_CLEAR3		0xE8
#define INTCPS_MIR3			0xE4
#define INTCPS_MIR_SET2			0xCC
#define INTCPS_ISR_SET2			0xD0
#define INTCPS_ISR_CLEAR2		0xD4
#define INTCPS_PENDING_IRQ2		0xD8
#define INTCPS_PENDING_FIQ2		0xDc
#define INTCPS_ILSR0			0x100
#define INTCPS_ILSR1			0x104
#define INTCPS_ILSR2			0x108
#define INTCPS_ILSR3			0x10C
#define INTCPS_ILSR4			0x110
#define INTCPS_ILSR5			0x114
#define INTCPS_ILSR6			0x118
#define INTCPS_ILSR7			0x11C
#define INTCPS_ILSR8			0x120
#define INTCPS_ILSR9			0x124
#define INTCPS_ILSR10			0x128
#define INTCPS_ILSR11			0x12C
#define INTCPS_ILSR12			0x130
#define INTCPS_ILSR13			0x134
#define INTCPS_ILSR14			0x138
#define INTCPS_ILSR15			0x13C
#define INTCPS_ILSR16			0x140
#define INTCPS_ILSR17			0x144
#define INTCPS_ILSR18			0x148
#define INTCPS_ILSR19			0x14C
#define INTCPS_ILSR20			0x150
#define INTCPS_ILSR21			0x154
#define INTCPS_ILSR22			0x158
#define INTCPS_ILSR23			0x15C
#define INTCPS_ILSR24			0x160
#define INTCPS_ILSR25			0x164
#define INTCPS_ILSR26			0x168
#define INTCPS_ILSR27			0x16C
#define INTCPS_ILSR28			0x170
#define INTCPS_ILSR29			0x174
#define INTCPS_ILSR30			0x178
#define INTCPS_ILSR31			0x17C
#define INTCPS_ILSR32			0x180
#define INTCPS_ILSR33			0x184
#define INTCPS_ILSR34			0x188
#define INTCPS_ILSR35			0x18C
#define INTCPS_ILSR36			0x190
#define INTCPS_ILSR37			0x194
#define INTCPS_ILSR38			0x198
#define INTCPS_ILSR39			0x19C
#define INTCPS_ILSR40			0x1A0
#define INTCPS_ILSR41			0x1A4
#define INTCPS_ILSR42			0x1A8
#define INTCPS_ILSR43			0x1AC
#define INTCPS_ILSR44			0x1B0
#define INTCPS_ILSR45			0x1B4
#define INTCPS_ILSR46			0x1B8
#define INTCPS_ILSR47			0x1BC
#define INTCPS_ILSR48			0x1C0
#define INTCPS_ILSR49			0x1C4
#define INTCPS_ILSR50			0x1C8
#define INTCPS_ILSR51			0x1CC
#define INTCPS_ILSR52			0x1D0
#define INTCPS_ILSR53			0x1D4
#define INTCPS_ILSR54			0x1D8
#define INTCPS_ILSR55			0x1DC
#define INTCPS_ILSR56			0x1E0
#define INTCPS_ILSR57			0x1E4
#define INTCPS_ILSR58			0x1E8
#define INTCPS_ILSR59			0x1EC
#define INTCPS_ILSR60			0x1F0
#define INTCPS_ILSR61			0x1F4
#define INTCPS_ILSR62			0x1F8
#define INTCPS_ILSR63			0x1FC
#define INTCPS_ILSR64			0x200
#define INTCPS_ILSR65			0x204
#define INTCPS_ILSR66			0x208
#define INTCPS_ILSR67			0x20C
#define INTCPS_ILSR68			0x210
#define INTCPS_ILSR69			0x214
#define INTCPS_ILSR70			0x218
#define INTCPS_ILSR71			0x21C
#define INTCPS_ILSR72			0x220
#define INTCPS_ILSR73			0x224
#define INTCPS_ILSR74			0x228
#define INTCPS_ILSR75			0x22C
#define INTCPS_ILSR76			0x230
#define INTCPS_ILSR77			0x234
#define INTCPS_ILSR78			0x238
#define INTCPS_ILSR79			0x23C
#define INTCPS_ILSR80			0x240
#define INTCPS_ILSR81			0x244
#define INTCPS_ILSR82			0x248
#define INTCPS_ILSR83			0x24C
#define INTCPS_ILSR84			0x250
#define INTCPS_ILSR85			0x254
#define INTCPS_ILSR86			0x258
#define INTCPS_ILSR87			0x25C
#define INTCPS_ILSR88			0x260
#define INTCPS_ILSR89			0x264
#define INTCPS_ILSR90			0x268
#define INTCPS_ILSR91			0x26C
#define INTCPS_ILSR92			0x270
#define INTCPS_ILSR93			0x274
#define INTCPS_ILSR94			0x278
#define INTCPS_ILSR95			0x27C
#define INTCPS_ILSR96			0x280
#define INTCPS_ILSR97			0x284
/* GPIO */
/* modified */
//#define GPIO5_BASE				0x48032000
//#define GPIO6_BASE				0x49058000

#define GPIO0_BASE				0x44E07000
#define GPIO1_BASE				0x4804C000
#define GPIO2_BASE				0x481AC000
#define GPIO3_BASE				0x481AE000

#define GPIO0_INTA			96
#define GPIO0_INTB			9

#if 0
/* GPIO Offsets */
#define GPIO_REVISION			0x0
#define GPIO_SYSCONFIG			0x10
#define GPIO_SYSSTATUS			0x14
#define GPIO_IRQSTATUS1			0x18
#define GPIO_IRQ_ENABLE1		0x1C
#define GPIO_WAKEUPENABLE		0x20
#define GPIO_IRQSTATUS2			0x28
#define GPIO_IRQENABLE2			0x2C
#define GPIO_CTRL				0x30
#define GPIO_OE					0x34
#define GPIO_DATAIN				0x38
#define GPIO_DATAOUT			0x3C
#define GPIO_LEVELDETECT0		0x40
#define GPIO_LEVELDETECT1		0x44
#define GPIO_RISINGDETECT		0x48
#define GPIO_FALLINGDETECT		0x4C
#define GPIO_DEBOUNCENABLE		0x50
#define GPIO_DEBOUNCEINGTIME	0x54
#define GPIO_CLEARIRQENABLE1	0x60
#define GPIO_SETIRQENABLE1		0x64
#define GPIO_CLEARIRQENABLE2	0x70
#define GPIO_SETIRQENABLE2		0x74
#define GPIO_CLEARWKUENA		0x80
#define GPIO_SETWKUENA			0x84
#define GPIO_CLEARDATAOUT		0x90
#define GPIO_SETDATAOUT			0x94

#endif

#define GPIO_REVISION			0x0
#define GPIO_SYSCONFIG			0x10
#define GPIO_EOI				0x20
#define GPIO_IRQSTATUS_RAW_0	0x24
#define GPIO_IRQSTATUS_RAW_1	0x28
#define GPIO_IRQSTATUS_0		0x3C
#define GPIO_IRQSTATUS_1		0x30
#define GPIO_IRQSTATUS_SET_0	0x34
#define GPIO_IRQSTATUS_SET_1 	0x38
#define GPIO_IRQSTATUS_CLR_0	0x3C
#define GPIO_IRQSTATUS_CLR_1	0x40
#define GPIO_SYSSTATUS			0x114
#define GPIO_CTRL				0x130
#define GPIO_OE					0x134
#define GPIO_DATAIN				0x138
#define GPIO_DATAOUT			0x13C
#define GPIO_LEVELDETECT0		0x140
#define GPIO_LEVELDETECT1		0x144
#define GPIO_RISINGDETECT		0x148
#define GPIO_FALLINGDETECT		0x14C
#define GPIO_DEBOUNCENABLE		0x150
#define GPIO_DEBOUNCINGTIME		0x154
#define GPIO_CLEARDATAOUT		0x190
#define GPIO_SETDATAOUT			0x194
 


/* Pin definitions */
#define PIN0					(0x1 << 0)
#define PIN1					(0x1 << 1)
#define PIN2					(0x1 << 2)
#define	PIN3					(0x1 << 3)
#define PIN4					(0x1 << 4)
#define PIN5					(0x1 << 5)
#define PIN6					(0x1 << 6)
#define PIN7					(0x1 << 7)
#define PIN8					(0x1 << 8)
#define PIN9					(0x1 << 9)
#define PIN10					(0x1 << 10)
#define PIN11					(0x1 << 11)
#define PIN12					(0x1 << 12)
#define PIN13					(0x1 << 13)
#define PIN14					(0x1 << 14)
#define PIN15					(0x1 << 15)
#define PIN16					(0x1 << 16)
#define PIN17					(0x1 << 17)
#define PIN18					(0x1 << 18)
#define PIN19					(0x1 << 19)
#define PIN20					(0x1 << 20)
#define PIN21					(0x1 << 21)
#define PIN22					(0x1 << 22)
#define PIN23					(0x1 << 23)
#define PIN24					(0x1 << 24)
#define PIN25					(0x1 << 25)
#define PIN26					(0x1 << 26)
#define PIN27					(0x1 << 27)
#define PIN28					(0x1 << 28)
#define PIN29					(0x1 << 29)
#define PIN30					(0x1 << 30)
#define PIN31					(0x1 << 31)

/* Serial Configuration (UART 0)*/
/* modified */
//#define SERIAL_BASE 		    0x44e09000
#define UART0_BASE 		    0x44E09000
#define UART4_BASE		    0x481A8000 

/* Serial Offsets */
#define DLL_REG					0x000
#define RHR_REG					0x000
#define THR_REG					0x000
#define DLH_REG					0x004
#define IER_REG					0x004
#define IIR_REG					0x008
#define FCR_REG					0x008
#define EFR_REG					0x008
#define LCR_REG					0x00C
#define MCR_REG					0x010
#define XON1_ADDR1_REG			0x010
#define LSR_REG					0x014
#define XON2_ADDR2_REG			0x014
#define MSR_REG					0x018
#define TCR_REG					0x018
#define XOFF1_REG				0x018
#define SPR_REG					0x01C
#define TLR_REG					0x01C
#define XOFF2_REG				0x01C
#define MDR1_REG				0x020
#define MDR2_REG				0x024
#define SFLSR_REG				0x028
#define TXFLL_REG				0x028
#define RESUME_REG				0x02C
#define TXFLH_REG				0x02C
#define SFREGL_REG				0x030
#define RXFLL_REG				0x030
#define SFREGH_REG				0x034
#define RXFLH_REG				0x034
#define UASR_REG				0x038
#define BLR_REG					0x038
#define ACREG_REG				0x03C
#define SCR_REG					0x040
#define SSR_REG					0x044
#define EBLR_REG				0x048
#define MVR_REG					0x050
#define SYSC_REG				0x054
#define SYSS_REG				0x058
#define WER_REG					0x05C
#define CFPS_REG				0x060

#define PRCM_REG                0x44e00000
#define CM_PER_GPIO1_CLKCTRL    0xAC
#endif /* omap3_h */
