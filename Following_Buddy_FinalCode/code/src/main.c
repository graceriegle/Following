#include "stm32f0xx.h"
#include <stdint.h>
#include <math.h>   // for M_PI

//void internal_clock();

void internal_clock()
{
    /* Disable HSE to allow use of the GPIOs */
    RCC->CR &= ~RCC_CR_HSEON;

    /* Enable Prefetch Buffer and set Flash Latency */
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

    /* HCLK = SYSCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;

    /* PCLK = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE_DIV1;

    /* PLL configuration = (HSI/2) * 12 = ~48 MHz */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSI_Div2 | RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLMULL12);

    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;

    /* Wait till PLL is ready */
    while((RCC->CR & RCC_CR_PLLRDY) == 0)
    {
    }

    /* Select PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;

    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL)
    {
    }
}

void input_gpio(){

    RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB -> MODER &= ~0xf000; //clears PB6 PB7, set to input mode
    
}

void init_usart5() {

    RCC -> AHBENR |= RCC_AHBENR_GPIOCEN;
    RCC -> AHBENR |= RCC_AHBENR_GPIODEN;

    //configure pin PC12 to be routed to USART_TX
    GPIOC -> MODER &= ~GPIO_MODER_MODER12;
    GPIOC -> MODER |= GPIO_MODER_MODER12_1;
    GPIOC -> AFR[1] &= ~0xf0000;
    GPIOC -> AFR[1] |= 0x20000;

    //configure pin PD2 to be routed to USART5_RX
    GPIOD -> MODER &= ~GPIO_MODER_MODER2;
    GPIOD -> MODER |= GPIO_MODER_MODER2_1;
    GPIOD -> AFR[0] &= ~0xf00;
    GPIOD -> AFR[0] |= 0x200;

    RCC -> APB1ENR |= RCC_APB1ENR_USART5EN;
    USART5 -> CR1 &= ~USART_CR1_UE; //disable UART
    USART5 -> CR1 &= ~USART_CR1_M1; //set word length to 8 bits
    USART5 -> CR1 &= ~USART_CR1_M0;
    USART5 -> CR2 &= ~USART_CR2_STOP; //set for one stop bit
    USART5 -> CR1 &= ~USART_CR1_PCE; //set for no parity control
    USART5 -> CR1 &= ~USART_CR1_OVER8; //oversampling by 16

    USART5 -> BRR = 0x1A1; //set baud rate

    USART5 -> CR1 |= USART_CR1_RE; //enable receiver    
    USART5 -> CR1 |= USART_CR1_TE; //enable transmitter

    USART5 -> CR1 |= USART_CR1_UE; //enable the USART

    while(!(USART5 -> ISR & USART_ISR_TEACK)); //waits for TE bit to be acknowledged
    while(!(USART5 -> ISR & USART_ISR_REACK)); //waits for RE bit to be acknowledged
}

void nano_wait(int);

void setup_tim3(void) {
    RCC -> APB1ENR |= RCC_APB1ENR_TIM3EN; //enable the clock for the TIM3 peripheral
    RCC -> AHBENR |= RCC_AHBENR_GPIOCEN; //enable the clock for the GPIOC peripheral

    GPIOC -> MODER &= ~0xf000; //set the mode to alternate function mode
    GPIOC -> MODER |= 0xa000;

    GPIOC -> AFR[0] &= ~0xff000000;

    TIM3 -> PSC = 10000 - 1; //set TIM3 auto-reload register such that the timer's frequency is 1 Hz
    
    int arr = 12 - 1;
    TIM3 -> ARR = arr; 
    
    float duty_cycle1 = 50.0 / 100.0; //type in desired duty cycle
    float duty_cycle2 = 50.0 / 100.0;

    TIM3 -> CCMR1 &= ~0x7070;
    TIM3 -> CCMR1 |= 0x6060;

    TIM3 -> CCER &= ~0xff;
    TIM3 -> CCER |= 0x11;

    TIM3 -> CR1 |= TIM_CR1_CEN; //enable the timer

    float ccr1 = (duty_cycle1 * (arr)); //do I need to add (arr + 1)
    float ccr2 = (duty_cycle2 * (arr));

    TIM3 -> CCR1 = (ccr1 + 1); //change these values to increase / decrease the duty cycle
    TIM3 -> CCR2 = (ccr2 + 1);
}

void stop_pwm(void){
    TIM3 -> CR1 &= ~TIM_CR1_CEN;
}

void init_gpio(){ //initialize PB8-PB11
    RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB -> MODER &= ~0xff0000; //clears PB8-PB11
    GPIOB -> MODER |= 0x550000; //sets PB8-11 to output mode

    GPIOB -> MODER  &= ~0x3f; //clears PB0-2
    GPIOB -> MODER |= 0x14; //sets PB1, PB2 to output
}

int main(void){
    internal_clock();
    init_usart5();
    //setup_tim3();
    init_gpio();
    stop_pwm();
    int dir = 0 ;

    int input = 0;

    //RECEIVE GPIO INPUT SIGNALS
    GPIOB -> BSRR = (1 << 24); 
    GPIOB -> BSRR = (1 << 25);
    GPIOB -> BSRR = (1 << 26);
    GPIOB -> BSRR = (1 << 27);
    nano_wait(1000000000);
    nano_wait(1000000000);
    nano_wait(1000000000);
    nano_wait(1000000000);
    nano_wait(1000000000);


    stop_pwm();
    for(;;) {
        dir = ((GPIOB -> IDR) >> 6) & (3) ; 


//  NEW SECTION
        input = ((GPIOB -> IDR) & 1);    

        if (input == 1)
        {//PB1
            GPIOB -> BSRR = (1 << 1); //turns on PB1 if input is high
        }

        if (input == 0)
        {//PB2
            GPIOB -> BSRR = (1 << 2); //turns on if PB2 input is low
        }

//  NEW SECTION

        //move forward
        if (dir == 3)
        { 
            GPIOB -> BSRR = (1 << 8); //output for PB8 left wheel(sets 1)
            GPIOB -> BSRR = (1 << 25); //output for PB9 left wheel (sets 0)

            GPIOB -> BSRR = (1 << 10); //output for PB10 right wheel (sets 1)
            GPIOB -> BSRR = (1 << 27);//output for PB11 right wheel (sets 0);
            USART5 -> TDR = 'f';
            setup_tim3(); //set up the PWM
        }

        //move right
        else if (dir == 2)
        {
            //move left wheel forward
            GPIOB -> BSRR = (1 << 8); //output for PB8 left wheel(sets 1)
            GPIOB -> BSRR = (1 << 25); //output for PB9 left wheel (sets 0)

            //move right wheel backward
            GPIOB -> BSRR = (1 << 26); //output for PB10 right wheel (sets 0)
            GPIOB -> BSRR = (1 << 11); //output for PB11 right wheel (sets 1)
            USART5 -> TDR = 'r';
            setup_tim3(); //set up the PWM
        }

        //move left
        else if (dir == 1)
        {
            //move left wheel backward
            GPIOB -> BSRR = (1 << 24); //output for PB8 left wheel (sets 0)
            GPIOB -> BSRR = (1 << 9); //output for PB9 left wheel (sets 1)

            //move right wheel forward
            GPIOB -> BSRR = (1 << 10); //output for PB10 right wheel (sets 1)
            GPIOB -> BSRR = (1 << 27); //output for PB11 right wheel (sets 0)
            USART5 -> TDR = 'l';
            setup_tim3(); //set up the PWM
        }

        //enable PWM for 0.2s, the stop PWM
        //setup_tim3(); //set up the PWM
        nano_wait(200000000); //runs the motor for 0.2s
        //nano_wait(2000000000); //runs the motor for 2s
        stop_pwm(); //stop motor after 0.2s

        //set all directional control GPIOs to 0
        GPIOB -> BSRR = (1 << 24); 
        GPIOB -> BSRR = (1 << 25);
        GPIOB -> BSRR = (1 << 26);
        GPIOB -> BSRR = (1 << 27);
    }
}