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


sad
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

/* 4 USART & 4 UART ports
  

   Addressing and enabling the UART
   The serial interface USART3 is directly available as a virtual COM port of the 
   PC connected to the ST-LINK/V2-1 USB connector CN1. 
   The virtual COM port settings are configured as: 
   115200 b/s, 8 bits data, no parity, 1 stop bit, no flow control.

   USART3 addresses
   0x4000.4800, .4BFF

   The ST LINK uses PB10 RX, and PB11 for TX
    it also uses PC10/11
   UART3_TX PA0 Also alternative function PC10 AF8
   UART3_RX PA1 Also alternative function PC11 AF8

   Base Addresses of UART ports = all on APB1
   UART4 0x4000 4C00→4FFF
   UART5 0x4000 5000→53FF
   UART7 0x4000 7800→7BFF
   UART8 0x4000 7C00→7FFF

   Clock enable port is RCC_APB1ENR - 
   base address of RCC is 0x4002 3800 - 0x4002 3BFF
   offset for RCC_APB1ENR is 0x40
   USART3[18]

   SYSCLK is 16MHZ - there is an AHB  Prescaler and an APB prescaler

    16MHz → AHB PS 1/2/4...512 → APB PS 1/2/4/8/16
   set in the RCC  CFGR register 

   NB AHB clock is HCLK, also the CPU clock - max frequency 180MHZ
   AHB controlled in RCC_CFG[7:4]    - msb @ 0 => no division
   APB1 controlled in RCC_CFG[12:10] - msb @ 0 => no division

   Setup Baud Rate
   USART_BRR - @ 4000.4800 base address for USART3
   offset for USART_BRR 0x08
   USART_BRR[31:16] are reserved
   USART_BRR[15:4] are DIV Mantissa[11:0]
   USART_BRR[3:0] are DIV Fraction[3:0] - for oversampling 16
   USART_BRR[2:0] are DIV Fraction[3:0] - for oversampling 8, [3] should be forced low 

USART_DIV = fck/16*baud - let fck be 16MHZ, OSR=16 (default)
USART_DIV = fck/8*baud - let fck be 16MHZ, OSR=8
OSR = 16 examples
BaudRate=9600, 16MHz/16*9600=104.166 - 104 = 0x68 for [15:5],
               [3:0] take Ceil(0.166*16)=2.656=3, so 0x3 in [3:0]
               so load USART_DIV with 0x0000.0683
Using the same method. 
    19600 → USART_DIV = 0x0000.0341
    57600 → USART_DIV = 0x0000.0116
   115200 → USART_DIV = 0x0000.008B

   If OSR = 8
     9600 → USART_DIV = 0x0000.1183
    19600 → USART_DIV = 0x0000.0681
    57600 → USART_DIV = 0x0000.0236
   115200 → USART_DIV = 0x0000.0113

   Alternative Function of GPIO's
   USART3 is connected to ST LINK
   The ST LINK uses PB10 RX, and PB11 for TX
    it also uses PC10/11
   
   PB10/11 need to be configured in the GPIO mode reg to use AF7 
   See page 75 of datasheet (not reference manual)
   After the mode of the GPIO's is set to AF (0x2), the GPIO_AFRH/L
   must be set to alternative function '0x7'

STEPS TO TAKE TO USE UART3 ON DISCO BOARD (for write)
    1: Enable the clock on PortB
    2: Enable the clock to USART3
    3: Select the peripheral function AF7 for PB10 USART3_RxD pin
    4: Set the baud rate for USART3 use USART3_BRR register
    5: Configure CR1 {OSR, tx size 8/9 bit, and enabling transmit}
    6: Configure CR2 {stop-bits etc}
    7: Configure CR3 {h/w flow etc}
    8: Enable hardware flow after configuration is complete
    9: Wait until TXE (Transmit Empty) bit of USART_SR is set
   10: Write a byte of data to the DR register 
   11: Repeat from step 9 to write more characters.

   Observe in a terminal

 */


#include "functions.h"

#define DELAY_IN_MS 250
	

 int main(void) {

     /*
      * Using the User button (blue) on the discovery board
      * This button has a pull-down resistor, so pressing it
      * causes a logic-1 to be read - connected to PA0, shared
      * with the gpio, and wakeup function
      *
      * We need to detect the rising edge, this will trigger an
      * interrupt on EXTI (the external interrupt controller)
      * There are 16 external interupts assigned to the GPIO pins
      * Each pin from each port can be attached with an EXTIx, 
      * for each of the 16-EXTI's, 4 bits are assigned to connect
      * one of the GPIO ports to the specific EXTI
      *
      * for example: To connect PC13, EXTI13 must be configured for
      * portC - EXTI13 controls this mux function - one of several 
      * pins can be connected to each EXTI
      *  SYSCFG_EXTICRx is used for this.
      *
      * Here simply toggle the LED's when the button is pressed.
      *
      *
      */

     __disable_irq();

     uint32_t *ptr;
     initLeds(); //This enables the clk for ports, d,g and K

     ptr  = (uint32_t *)RCC_AHB1ENR_ADDR;
     *ptr |= 0x00000001; //enable gpioA clk

     ptr  = (uint32_t *)(GPIO_BASE + GPIOA_OFS + MODER_OFS);
     *ptr &= 0x00000003; //enable gpioA[0] as input source
     

     ptr = (uint32_t *)(RCC_BASE + RCC_APB2ENR_OFS);
     *ptr |= 0x00004000; //Enable SYSCFG clock [14], (for EXTIx)

     ptr  = (uint32_t *)SYSCFG_EXTICR1;
     *ptr &= 0x0000000F; //clear EXTI0 bits
     *ptr |= 0x00000000; //Configure PA0 as Line0 EXTI

     //Enable the mask bit of the interrupt line in EXTI_IMR
     ptr  = (uint32_t *)(EXTI_BASE + EXTI_IMR);
     *ptr |= 0x00000001; //unmask PA0

     //Select the trigger, rising edge detect EXTI_RTSR
     ptr  = (uint32_t *)(EXTI_BASE + EXTI_RTSR);
     *ptr |= 0x00000001; //rising edge detect on PA0

     //Enable the EXTI0 interrupt in the NVIC
     //The number of the interrupt is found in the Vector table
     //page 289 of the Reference Manual - here Position 6 is for 
     //EXTI0  - set in ISER0[6] 
     //Also page 218 of Programmers manual
     ptr  = (uint32_t *)(NVIC_BASE + NVIC_ISER0);
     *ptr |= 0x00000040; //Enable IRQ[0]

     __enable_irq();


     while(1) { }

 }

void EXTI0_IRQHandler(void) {
    driveLed(RED,SET);
    delayMs(200);
    driveLed(RED,CLEAR);
    delayMs(200);
    driveLed(RED,SET);
    delayMs(200);
    driveLed(RED,CLEAR);
    delayMs(200);

    //Clear interrupt pending, or this keeps running!
    //EXTI0 is IRQ6
    
     uint32_t *ptr;
     ptr  = (uint32_t *)(NVIC_BASE + NVIC_IPR6);
     *ptr |= 0x00000040; //Enable IRQ[0]

}


