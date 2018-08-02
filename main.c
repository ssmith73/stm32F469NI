/*  GPIO CONTROL 
   0x4002 2800 - 0x4002 2BFF GPIOK
   0x4002 2400 - 0x4002 27FF GPIOJ
   0x4002 2000 - 0x4002 23FF GPIOI
   0x4002 1C00 - 0x4002 1FFF GPIOH
   0x4002 1800 - 0x4002 1BFF GPIOG
   0x4002 1400 - 0x4002 17FF GPIOF
   0x4002 1000 - 0x4002 13FF GPIOE
   0x4002 0C00 - 0x4002 0FFF GPIOD
   0x4002 0800 - 0x4002 0BFF GPIOC
   0x4002 0400 - 0x4002 07FF GPIOB
   0x4002 0000 - 0x4002 03FF GPIOA

   led's for the stm32f469NI (disco)
   NB: They are active low and are connected as follows
    ------------------------
    LED     Port    Color
    ________________________
    LD1     PG6     Green
    LD2     PD4     Orange
    LD3     PD5     Red
    LD4     PK3     Blue
    ------------------------
  
    Base Addresses
    ------------------------------------
    Port    Address Ranges
    ____________________________________
    PG     0x4002_1800 --> 0x4002_1BFF
    PD     0x4002_0C00 --> 0x4002_0FFF
    PK     0x4002_2800 --> 0x4002_2BFF
    ------------------------------------

    LED's on the disco are configured with different registers.
    Each GPIO port has 
    
    4 32b config registers
    GPIOx_MODER 
        00: input, 01: output, 10: Alternate function mode, 11:Analog Mode
    GPIOx_OTYPER  OT[15:0], 0: push/pull, 1: open drain
    GPIOx_OSPEEDR 2 bits per i/o
        00:Low Speed, 01:Medium Speed, 10:High Speed, 11: Very high speed (see datasheet)
    GPIOx_PUPDR 
        00:No pullup/down, 01:pull-up, 10:Pull Down, 11: Reserved

    2 32b data registers
    GPIOx_IDR 16 bits [15:0] used, read only, corrospoding value of I/O port
    GPIOX_ODR 16 bits [15:0] used, r/w, corrospoding value of I/O port
        for atomic (safe) set/reset, the ODR bits can be individually set
        and/or reset by writing to GPIO_BSRR/BRR

    1 32b set/reset register  (write only)
    GPIO_x_BSRR - all 32 bits are used, for each I/O, one for set and one for reset
    active high sets or resets a bit - [15:0] is BSx, [31:16] is BRx

    All GPIO ports have a 32b locking register GPIOx_LCKR
    All GPIO ports have 2 32b Alternate-Function selection registers
    GPIOx_AFRH/AFRL

    Port G[6] has the Green LED, it's address range is 4002_1800→1BFF
    0x1BFF - 0x1800 is 1024 locations (bytes) (0x3FF), or 32, 32b registers
    GPIOG_MODER  0x4002_1800 GPIOG_OTYPER 0x4002_1804
    GPIOG_SPEEDR 0x4002_1808 GPIOG_PUPDR  0x4002_180C
    GPIOG_IDR    0x4002_1810 GPIOG_ODR    0x4002_1814
    GPIOG_BSRR   0x4002_1818 GPIOG_LCKR   0x4002_181c
    GPIOG_AFRL   0x4002_1820 GPIOG_AFRH   0x4002_1824



    ___________________________________________________________________


 */

/*  RCC Clock and Reset Control
    By defaults the clocks controlling peripherals are disabled.
    So it is necessary to enable the clock to either the a given
    I/O port, or peripheral before using it.

    This is done by accessing a group of registers belonging to 
    RCC (Reset and Clock Control)

    Base Address of RCC 0x4002_3800→3BFF 
    To enable the clock for the GPIO's, access RCC_AHB1ENR, the
    peripheral clock enable register 

    Address offset 0x30 - bits [10:0] control the GPIO ports A→K
    enables are active high

 */
#include<stdint.h>

void delayMs(int n);
#define DELAY_IN_MS 500
	
int main(void) {

    uint32_t *rccAhb1Ptr    = (uint32_t *)0x40023830;
    uint32_t *gpiogModerPtr = (uint32_t *)0x40021800;
    uint32_t *gpiogBsrrPtr  = (uint32_t *)0x40021818;

    *rccAhb1Ptr      |= 0x00000040; //enable gpioG clk
    *gpiogModerPtr   |= 0x00001000; //Set mode bit 6 to output

    while(1) {
        *gpiogBsrrPtr = 0x00000040; //Set GPIO's
        delayMs(DELAY_IN_MS);
        *gpiogBsrrPtr = 0x00400000; //Clear GPIO's
        delayMs(DELAY_IN_MS);
    }
}

// 16MHz delay
void delayMs(int n) {
    uint16_t i;
    for(;n>0;n--)
        for(i=0;i<3195;i++);
}


