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
#include<stdint.h>
 

typedef enum {RED,GREEN,ORANGE,BLUE} colours_t;
typedef enum {SETBIT,CLEAR} pinState_t;

//#define RCC_BASE            (uint32_t)0x40023800
#define RCC_AHB1ENR_ADDR    (uint32_t)0x40023830

#define RCC_APB1ENR_OFS     (uint32_t)0x40
#define RCC_APB2ENR_OFS     (uint32_t)0x44

#define GPIO_BASE           (uint32_t)0x40020000 //ahb1
#define GPIOA_OFS           (uint32_t)0x0000
#define GPIOB_OFS           (uint32_t)0x0400
#define GPIOC_OFS           (uint32_t)0x0800
#define GPIOD_OFS           (uint32_t)0x0C00
#define GPIOE_OFS           (uint32_t)0x1000
#define GPIOF_OFS           (uint32_t)0x1400
#define GPIOG_OFS           (uint32_t)0x1800
#define GPIOH_OFS           (uint32_t)0x1C00
#define GPIOI_OFS           (uint32_t)0x2000
#define GPIOJ_OFS           (uint32_t)0x2400
#define GPIOK_OFS           (uint32_t)0x2800

#define MODER_OFS           (uint32_t)0x00
#define OTYPER_OFS          (uint32_t)0x04
#define OSPEEDR_OFS         (uint32_t)0x08
#define PUPDR_OFS           (uint32_t)0x0C
#define IDR_OFS             (uint32_t)0x10
#define ODR_OFS             (uint32_t)0x14
#define BSRR_OFS            (uint32_t)0x18
#define LCKR_OFS            (uint32_t)0x1C
#define AFRL_OFS            (uint32_t)0x20
#define AFRH_OFS            (uint32_t)0x24


//#define EXTI_BASE           (uint32_t)0x40013C00 //apb2
#define EXTI_IMR            (uint32_t)0x00
#define EXTI_EMR            (uint32_t)0x04
#define EXTI_RTSR           (uint32_t)0x08
#define EXTI_FTSR           (uint32_t)0x0C
#define EXTI_SWIER          (uint32_t)0x10
#define EXTI_PR             (uint32_t)0x14

#define INT_EXTI0           (uint32_t)0x00000058
#define INT_EXTI1           (uint32_t)0x0000005C
#define INT_EXTI2           (uint32_t)0x00000060
#define INT_EXTI3           (uint32_t)0x00000064
#define INT_EXTI4           (uint32_t)0x00000068
#define INT_EXTI9_5         (uint32_t)0x0000009C
#define INT_EXTI15_10       (uint32_t)0x000000E0

#define SYSCFG_EXTICR1      (uint32_t)0x00000008
#define SYSCFG_EXTICR2      (uint32_t)0x0000000C
#define SYSCFG_EXTICR3      (uint32_t)0x00000010
#define SYSCFG_EXTICR4      (uint32_t)0x00000014
#define SYSCFG_MEMRMP       (uint32_t)0x00000020


//NVIC

//#define NVIC_BASE       (uint32_t)0xE000E100
#define NVIC_ISER0      (uint32_t)0x000
#define NVIC_ISER1      (uint32_t)0x004
#define NVIC_ISER2      (uint32_t)0x008
#define NVIC_ICER0      (uint32_t)0x080
#define NVIC_ICER1      (uint32_t)0x084
#define NVIC_ICER2      (uint32_t)0x088
#define NVIC_ISPR0      (uint32_t)0x100
#define NVIC_ISPR1      (uint32_t)0x104
#define NVIC_ISPR2      (uint32_t)0x108
#define NVIC_ICPR0      (uint32_t)0x180
#define NVIC_ICPR1      (uint32_t)0x184
#define NVIC_ICPR2      (uint32_t)0x188
#define NVIC_IABR0      (uint32_t)0x200
#define NVIC_IABR1      (uint32_t)0x204
#define NVIC_IABR2      (uint32_t)0x208

#define NVIC_IPR0       (uint32_t)0x300
#define NVIC_IPR1       (uint32_t)0x304
#define NVIC_IPR2       (uint32_t)0x308
#define NVIC_IPR3       (uint32_t)0x30C
#define NVIC_IPR4       (uint32_t)0x310
#define NVIC_IPR5       (uint32_t)0x314
#define NVIC_IPR6       (uint32_t)0x318
#define NVIC_IPR7       (uint32_t)0x31C
#define NVIC_IPR8       (uint32_t)0x320
#define NVIC_IPR9       (uint32_t)0x324
#define NVIC_IPR10      (uint32_t)0x328
#define NVIC_IPR11      (uint32_t)0x32C
#define NVIC_IPR12      (uint32_t)0x330
#define NVIC_IPR13      (uint32_t)0x334
#define NVIC_IPR14      (uint32_t)0x338
#define NVIC_IPR15      (uint32_t)0x33C
#define NVIC_IPR16      (uint32_t)0x340
#define NVIC_IPR17      (uint32_t)0x344
#define NVIC_IPR18      (uint32_t)0x348
#define NVIC_IPR19      (uint32_t)0x34C
#define NVIC_IPR20      (uint32_t)0x350
#define NVIC_IPR21      (uint32_t)0x354
#define NVIC_IPR22      (uint32_t)0x358
#define NVIC_IPR23      (uint32_t)0x35C
#define NVIC_IPR24      (uint32_t)0x360
#define NVIC_IPR25      (uint32_t)0x364



#define NVIC_ISPR       (uint32_t)0x200
#define NVIC_ICPR       (uint32_t)0x300
#define NVIC_IPR        (uint32_t)0x400
#define NVIC_STIR       (uint32_t)0xF00




//SysTick timer
#define SYSTICK_BASE    (uint32_t)0xE000E010
#define STK_CTRL        (uint32_t)0x00
#define STK_LOAD        (uint32_t)0x04
#define STK_VAL         (uint32_t)0x08
#define STK_CALIB       (uint32_t)0x0C

//Main Timers base addresses
/*#define TIM1_BASE       (uint32_t)0x40010000 //APB2*/
/*#define TIM2_BASE       (uint32_t)0x40000000 //APB1*/
/*#define TIM3_BASE       (uint32_t)0x40000400 //APB1*/
/*#define TIM4_BASE       (uint32_t)0x40000800 //APB1*/
/*#define TIM5_BASE       (uint32_t)0x40000C00 //APB1*/
/*#define TIM6_BASE       (uint32_t)0x40001000 //APB1*/
/*#define TIM7_BASE       (uint32_t)0x40001400 //APB1*/
/*#define TIM8_BASE       (uint32_t)0x40010400 //APB2*/
/*#define TIM9_BASE       (uint32_t)0x40014000 //APB2*/
/*#define TIM10_BASE      (uint32_t)0x40014400 //APB2*/
/*#define TIMX11_BASE     (uint32_t)0x40014800 //APB2*/
/*#define TIMX12_BASE     (uint32_t)0x40001800 //APB1*/
/*#define TIMX13_BASE     (uint32_t)0x40001C00 //APB1*/
/*#define TIMX14_BASE     (uint32_t)0x40002000 //APB1*/

//Timer register offsets
#define TIMX_CR1        (uint32_t)0x00
#define TIMX_CR2        (uint32_t)0x04
#define TIMX_SR         (uint32_t)0x10
#define TIMX_CCMR1      (uint32_t)0x18
#define TIMX_CCMR2      (uint32_t)0x1C
#define TIMX_CCER       (uint32_t)0x20
#define TIMX_CNT        (uint32_t)0x24
#define TIMX_PSC        (uint32_t)0x28
#define TIMX_ARR        (uint32_t)0x2C
#define TIMX_CCR1       (uint32_t)0x34
#define TIMX_CCR2       (uint32_t)0x38
#define TIMX_CCR3       (uint32_t)0x3C
#define TIMX_CCR4       (uint32_t)0x40

void    delayMs(int n);
void    driveLed(colours_t colour, pinState_t state);
void    initLeds(void);
uint8_t usart2_write(uint32_t c);
uint8_t usart3_read(void);
void    usart3_init(void);
void    dlyMs(uint16_t numMs); //This uses the SysTick
	

uint8_t usart3_read(void) {
    
    uint32_t *ptr = (uint32_t *)0x40004800; //point to USART_SR
    uint32_t *usartDrPtr =(uint32_t *)0x40004804; //point to USART_DR
    
    while(!(*ptr & 0x00000020)) {} //wait until char arrives
    return *usartDrPtr;

}

uint8_t usart3_write(uint32_t ch) {
    uint32_t *ptr = (uint32_t *)0x40004800; //point to USART_SR
    uint32_t *usartDrPtr =(uint32_t *)0x40004804; //point to USART_DR
    
    while(!(*ptr & 0x00000080)) {} //wait until Tx buffer is empty

    return *usartDrPtr = (ch & 0xff); 
}

// (Rough) delay in mS, off a 16MHz delay
void delayMs(int n) {
    uint16_t i;
    for(;n>0;n--)
        for(i=0;i<3195;i++);
}

void initLeds() {

    uint32_t *ptr;
   //setup the AHB1 clock
    ptr  = (uint32_t *)RCC_AHB1ENR_ADDR;
    *ptr |= 0x00000448; //enable gpioD,G, K clk

   //Enable PD4 & 5 as outputs 
    ptr   =   (uint32_t *)0x40020C00;
    *ptr  |= 0x00000500; 

   //Enable PG6 Green LED as output
    ptr   =  (uint32_t *)0x40021800;
    *ptr |= 0x00001000; 

   //Enable PK3 Blue LED as output
    ptr   = (uint32_t *)0x40022800;
    *ptr |= 0x00000040; 
}

void driveLed(colours_t colour, pinState_t state) {
    uint32_t *ptr;

    /*
      LED's are active LOW - see 6.15 Discovery user Manual
      Four LEDs located top side are available for the user. 
      Refer to the Figure 4: STM32F469IDISCO top side layout .
      The LEDs are LD1, LD2, LD3, LD4 from left to right with 
      colors green, orange, red, blue respectively. To light a
      LED a low logic state 0 should be written in the corresponding GPIO.
    */
    switch(colour){
        case RED:
        ptr =  (uint32_t *)0x40020C18; //addr of GPIOD RED (PD5) BSRR reg
        *ptr = (state == SETBIT) ? 
            0x00200000 :  //Set RED LED
            0x00000020;   //Clear RED LED 
        break;
        case GREEN:
        ptr = (uint32_t *)0x40021818;//addr of GPIOG GREEN (PG6) BSRR reg
        *ptr = (state == SETBIT) ? 
            0x00400000 : //Set Green LED
            0x00000040;  //Clear Green LED 
        break;
        case ORANGE:
        ptr = (uint32_t *)0x40020C18;//addr of GPIOD ORANGE (PD4) BSRR reg
        *ptr = (state == SETBIT) ? 
            0x00100000 : //Set Green LED
            0x00000010;  //Clear Green LED 
        break;
        case BLUE:
        ptr = (uint32_t *)0x40022818;//addr of GPIOK Blue (PK3) BSRR reg
        *ptr = (state == SETBIT) ? 
            0x00080000 : //Set Blue LED
            0x00000008;  //Clear Blue LED 
        break;
    }
}

void usart3_init(){

    uint32_t *ptr;
    //Enable AHB1 clock for GPIO (port B)
    //Set RCC_AHB1ENR[1]
    ptr  = (uint32_t *)RCC_AHB1ENR_ADDR;
    *ptr |= 0x00000002; 

    //Enable GPIOB clock - APB1
    //Addresses of RCC 0x4002_3800→3BFF 
    //set RCC_APB1ENR[18]
    ptr = (uint32_t *) 0x40023840;
    *ptr |= 0x00040000;

    //Configure B11 for USART3 Tx GPIOx_AFH[11]
    //Configure B10 for USART3 Rx GPIOx_AFH[10]
    //0x4002.0400 - 0x4002 07FF GPIOB
    //GPIOB 4002.0400, offset for AFH 0x24
    ptr =  (uint32_t *)0x40020424;
    *ptr |= 0x00007700;

    //Mode bits for P10 to AF, offset for GPIOx_MODER 0x00
    //Mode bits for P11 to AF, offset for GPIOx_MODER 0x00
    ptr = (uint32_t *) 0x40020400;
    *ptr |= 0x00A00000;

    //Set baud rate = 11500 to match ST LINK
    //USART3 base addr = 0x4000.4800, offset for USART_BRR 0x8
    //baud rate: 115200, OSR 16, 8bits, no parity,no flow control
    ptr =  (uint32_t *)0x40004808;
    *ptr |= 0x0000008B;

    //USART3_CR1 - set transmit enable, TE(bit 3), offset 0x0c
    //USART3_CR1 - set receive  enable, TE(bit 2), offset 0x0c
    ptr   =  (uint32_t *)0x4000480C;
    *ptr |= 0x0000000C;

    //USART3_CR2 - 1 stop bit[13:12], offset 0x10
    ptr   =  (uint32_t *)0x40004810;
    *ptr |= 0x00000000;

    //USART3_CR3 - no flow-control, offset 0x14
    ptr   =  (uint32_t *)0x40004814;
    *ptr |= 0x00000000;

    //USART3_CR1 - enable USART3 - [13]), offset 0x0c
    ptr   =  (uint32_t *)0x4000480C;
    *ptr |= 0x00002000;

}

void dlyMs(uint16_t numMs) {

    // delay = N / SYSCLK
    // N = delay * SYSCLK = 0.001Sec * 16MHz) = 16000 = 0x3E80
    
    uint32_t *ptr;
    
    ptr = (uint32_t *)(SYSTICK_BASE +  STK_VAL);
    *ptr = 0x0; //clear the current value register

    ptr = (uint32_t *)(SYSTICK_BASE +  STK_LOAD);
    *ptr = 0x3E80; //set reload value for 1ms

    ptr = (uint32_t *)SYSTICK_BASE + STK_CTRL;
    *ptr = 0x5; //Enable timer, no interupt, clkSource=ProcessorClk (AHB)

    
    //check STK_CTRL[16] - countFlag, which returns 1 if timer counted to 
    //zero since the last time it was read
    for(uint8_t i=0;i<numMs;i++)
        while(!(*ptr & 0x00010000)){};

    *ptr = 0x0;

}
