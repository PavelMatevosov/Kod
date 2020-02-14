#define SRAM_BASE       0x20000000
#define STACK_TOP       (SRAM_BASE + 0x800)

#define PERIPH_BASE     0x40000000 /*!< Peripheral base address in the bit-band region */

#define RCC_BASE        0x40023800
#define GPIOB_BASE      0x40020400
#define TIM6_BASE       0x40001000
#define TIM7_BASE       0x40001400
#define NVIC_BASE		0xE000E100

#define RCC_AHBENR      (*(volatile unsigned long*)(RCC_BASE + 0x1C))
#define RCC_APB1ENR		(*(volatile unsigned long*)(RCC_BASE + 0x24))
#define GPIOB_MODER     (*(volatile unsigned long*)(GPIOB_BASE + 0x00))
#define GPIOB_ODR 		(*(volatile unsigned long*)(GPIOB_BASE + 0x14))
#define GPIOB_IDR 		(*(volatile unsigned long*)(GPIOB_BASE + 0x10))
#define NVIC_ISER1      (*(volatile unsigned long*)(NVIC_BASE + 0x004))

#define TIM6_CR1        (*(volatile unsigned long*)(TIM6_BASE + 0x00))
#define TIM6_SR         (*(volatile unsigned long*)(TIM6_BASE + 0x10))
#define TIM6_CNT        (*(volatile unsigned long*)(TIM6_BASE + 0x24))
#define TIM6_PSC        (*(volatile unsigned long*)(TIM6_BASE + 0x28))
#define TIM6_ARR        (*(volatile unsigned long*)(TIM6_BASE + 0x2C))
#define TIM6_DIER       (*(volatile unsigned long*)(TIM6_BASE + 0x0C))

#define TIM7_CR1        (*(volatile unsigned long*)(TIM7_BASE + 0x00))
#define TIM7_SR         (*(volatile unsigned long*)(TIM7_BASE + 0x10))
#define TIM7_CNT        (*(volatile unsigned long*)(TIM7_BASE + 0x24))
#define TIM7_PSC        (*(volatile unsigned long*)(TIM7_BASE + 0x28))
#define TIM7_ARR        (*(volatile unsigned long*)(TIM7_BASE + 0x2C))
#define TIM7_DIER       (*(volatile unsigned long*)(TIM7_BASE + 0x0C))

#define APB1PERIPH_BASE                 PERIPH_BASE
#define USART3_BASE                     (APB1PERIPH_BASE + 0x4800)
#define USART3_SR                       (*(volatile unsigned long*)(USART3_BASE + 0x00))
#define USART3_DR                      (*(volatile unsigned long*)(USART3_BASE + 0x02))
#define USART3_BRR                      (*(volatile unsigned long*)(USART3_BASE + 0x04))
#define USART3_CR1                      (*(volatile unsigned long*)(USART3_BASE + 0x06))
#define USART3_CR2                      (*(volatile unsigned long*)(USART3_BASE + 0x08))
#define USART3_CR3                      (*(volatile unsigned long*)(USART3_BASE + 0x0A))


/* CR1 register Mask */
#define CR1_CLEAR_Mask                  ((volatile unsigned long)0xFFF0FEFF)
#define CR2_STOP_CLEAR_Mask             ((volatile unsigned short)0xCFFF)  /*!< USART CR2 STOP Bits Mask */
#define CR3_CLEAR_Mask                  ((volatile unsigned short)0xFCFF)  /*!< USART CR3 Mask */
#define CR1_OVER8_Set                   ((volatile unsigned short)0x8000)       /* USART OVER8 mode Enable Mask */

#define USART_WordLength_8b             ((volatile unsigned short)0x0000)
#define USART_Parity_No                 ((volatile unsigned short)0x0000)
#define USART_Mode_Tx                   ((volatile unsigned short)0x0008)
#define USART_Mode_Rx                   ((volatile unsigned short)0x0004)
#define USART_HardwareFlowControl_None  ((volatile unsigned short)0x0000)

#define USART_IT_RXNE                   ((volatile unsigned short)0x0525)
#define IT_Mask                         ((volatile unsigned short)0x001F)  /*!< USART Interrupt Mask */
#define USART_FLAG_TC                   ((volatile unsigned short)0x0040)
#define CR1_UE_Set                      ((volatile unsigned short)0x2000)  /*!< USART Enable Mask */

#define CLK_FREQ                        (8000000UL)

char rx_buf[70] = {0};  // buf for UART stream from PC
char rx_buf_ptr = 0;

int main(void);

void delay() {
	for (volatile int i=0; i<10000; i++);
};

void change_green_led_state() {
	// GPIOB7 only
	if (GPIOB_IDR != (GPIOB_IDR | (0x80)))
	{
		GPIOB_ODR = GPIOB_ODR | (0x80); // Light up the LED
	}
	else {
		GPIOB_ODR = GPIOB_ODR & (~0x80); // Turn off the LED		
	}
}

void change_blue_led_state() {
	// GPIOB6 only
	if (GPIOB_IDR != (GPIOB_IDR | (0x40)))
	{
		GPIOB_ODR = GPIOB_ODR | (0x40); // Light up the LED
	}
	else {
		GPIOB_ODR = GPIOB_ODR & (~0x40); // Turn off the LED		
	}
}

 
void TIM6_handler() {
	TIM6_SR = TIM6_SR & (0x0); // Shows that event is handled
    change_green_led_state();
};

void TIM7_handler() {
	TIM7_SR = TIM7_SR & (0x0); // Shows that event is handled
    change_blue_led_state();
};

void USART3_IRQHandler(void)
{
	volatile unsigned long bitpos = 0x00, itmask = 0x00, usartreg = 0x00;
    unsigned char bitstatus = 0;

  
    /* Get the USART register index */
    usartreg = (((unsigned char)USART_IT_RXNE) >> 0x05);
    /* Get the interrupt position */
    itmask = USART_IT_RXNE & IT_Mask;
    itmask = (unsigned long)0x01 << itmask;
  
    if (usartreg == 0x01) /* The IT  is in CR1 register */
    {
        itmask &= USART3_CR1;
    }
    else if (usartreg == 0x02) /* The IT  is in CR2 register */
    {
        itmask &= USART3_CR2;
    }
    else /* The IT  is in CR3 register */
    {
        itmask &= USART3_CR3;
    }
  
    bitpos = USART_IT_RXNE >> 0x08;
    bitpos = (unsigned long)0x01 << bitpos;
    bitpos &= USART3_SR;
    if ((itmask != (unsigned short)0)&&(bitpos != (unsigned short)0))
    {
        bitstatus = 1;
    }
    else
    {
       bitstatus = 0;
    }
	
    if(bitstatus != 0)
    {
		unsigned short bitpos = 0x00, itmask = 0x00;
        bitpos = USART_IT_RXNE >> 0x08;
        itmask = ((unsigned short)0x01 << (unsigned short)bitpos);
        USART3_SR = (unsigned short)~itmask;
        rx_buf[rx_buf_ptr] = (unsigned char)(USART3_DR & (unsigned short)0x01FF);
        rx_buf_ptr++;
    }
}

void USART_SendData(unsigned short Data)
{
  /* Transmit Data */
  USART3_DR = (Data & (unsigned short)0x01FF);
}

void USART_Puts(volatile char *s)
{
	while(*s)
	{
		while(!(USART3_SR & USART_FLAG_TC));
		USART_SendData(*s);
		*s++;
	}
}

void default_handler() {
while(1) {};
};

/* vector table */
unsigned long *vector_table[] __attribute__((section(".vector_table"))) =
{
    (unsigned long *)STACK_TOP,  // initial stack pointer 
    (unsigned long *)main,        // main as Reset_Handler
    // 1 (upper)
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    // 11 (upper)
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    // 21
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    // 31
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    // 41
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    // 51
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)USART3_IRQHandler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)default_handler,
    (unsigned long *)TIM6_handler,
    (unsigned long *)TIM7_handler,
    (unsigned long *)default_handler,
};

int main() {
    RCC_AHBENR |= 0x2; // Enable GPIOB
    RCC_APB1ENR |= 0x10; // Enable TIM6
    RCC_APB1ENR |= 0x20; // Enable TIM7
    volatile unsigned long i=0;
	volatile unsigned long tmpreg = 0x00, apbclock = 0x00;
    volatile unsigned long integerdivider = 0x00;
    volatile unsigned long fractionaldivider = 0x00;


    // GPIOB_MODER = (GPIOB_MODER & (~0x0000F000)) | (0x1000); // GPIOB6 mode GP output
    GPIOB_MODER = (GPIOB_MODER & (~0x0000F000)) | (0x5000); //GPIOB6 and GPIOB7 GP output
	// USART3 TX and RX config
	GPIOB_MODER = (GPIOB_MODER & (~0x00F00000)) | (0x200000);
    // enable USART3
    RCC_APB1ENR |= ((unsigned long)0x00040000);

/*---------------------------- USART CR2 Configuration -----------------------*/
  
    tmpreg = USART3_CR2;
    tmpreg &= CR2_STOP_CLEAR_Mask;
    tmpreg |= ((unsigned short)0x0000);
    /* Write to USART CR2 */
    USART3_CR2 = (unsigned short)tmpreg;

/*---------------------------- USART CR1 Configuration -----------------------*/
    tmpreg = USART3_CR1;
    tmpreg &= CR1_CLEAR_Mask;
    tmpreg |= (unsigned long)USART_WordLength_8b | USART_Parity_No | USART_Mode_Tx | USART_Mode_Rx;
    /* Write to USART CR1 */
    USART3_CR1 = (unsigned short)tmpreg;

/*---------------------------- USART CR3 Configuration -----------------------*/  
    tmpreg = USART3_CR3;
    tmpreg &= CR3_CLEAR_Mask;
    tmpreg |= USART_HardwareFlowControl_None;
    /* Write to USART CR3 */
    USART3_CR3 = (unsigned short)tmpreg;

/*---------------------------- USART BRR Configuration -----------------------*/
    /* Configure the USART Baud Rate -------------------------------------------*/
    apbclock = CLK_FREQ;
 
  
    /* Determine the integer part */
    if ((USART3_CR1 & CR1_OVER8_Set) != 0)
    {
        /* Integer part computing in case Oversampling mode is 8 Samples */
        integerdivider = ((25 * apbclock) / (2 * (9600)));    
    }
    else
    {
        /* Integer part computing in case Oversampling mode is 16 Samples */
        integerdivider = ((25 * apbclock) / (4 * (9600)));    
    }
    tmpreg = (integerdivider / 100) << 4;

    /* Determine the fractional part */
    fractionaldivider = integerdivider - (100 * (tmpreg >> 4));

    /* Implement the fractional part in the register */
    if ((USART3_CR1 & CR1_OVER8_Set) != 0)
    {
        tmpreg |= ((((fractionaldivider * 8) + 50) / 100)) & ((char)0x07);
    }
    else
    {
        tmpreg |= ((((fractionaldivider * 16) + 50) / 100)) & ((char)0x0F);
    }
    /* Write to USART BRR */
    USART3_BRR = (unsigned short)tmpreg;
  
    /* Enable the selected USART by setting the UE bit in the CR1 register */
    USART3_CR1 |= CR1_UE_Set;

	
    TIM6_DIER = TIM6_DIER | (0x1);
    TIM6_PSC = (TIM6_PSC & (~0xFFFF)) | (0x1B57);
    TIM6_ARR = (TIM6_ARR & (~0xFFFF)) | (0x14D);
    TIM6_CR1 = TIM6_CR1 | (0x1);
    
    TIM7_DIER = TIM7_DIER | (0x1);
    TIM7_PSC = (TIM7_PSC & (~0xFFFF)) | (0x1B57);
    TIM7_ARR = (TIM7_ARR & (~0xFFFF)) | (0xFA);
    TIM7_CR1 = TIM7_CR1 | (0x1);
    NVIC_ISER1 = NVIC_ISER1 | (0x800); // Enable NVIC_ISER1 11th intrrupt (43)
    NVIC_ISER1 = NVIC_ISER1 | (0x1000); // Enable NVIC_ISER1 11th intrrupt (44)
	
	USART_Puts("Pavel");
    while (1)
	{
		if((rx_buf[rx_buf_ptr-2] == 0x0D) && (rx_buf[rx_buf_ptr-1] == 0x0A))
        {
            rx_buf[rx_buf_ptr] = '\0';
			if ((rx_buf[rx_buf_ptr-4] == 'O') && (rx_buf[rx_buf_ptr-3] == 'K'))
			{
			    USART_Puts("Svetlana");	
			}
            else
			{
				USART_Puts("Error");
			}
	    }
	}
}
