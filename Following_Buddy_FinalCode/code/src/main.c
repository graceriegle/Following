#include "stm32f0xx.h"
#include <stdint.h>
#include <math.h>   // for M_PI


//TO-DO
//* determine how long one rotation takes for different duty cycles so we can adjust the code
//* make sure we can remove the UART code
//* simplify code for turning off GPIOs into one line of code
//* change from 0.2s to a smaller time

float duty =  0.75 ; 


void internal_clock();
void search_for_flag(char *) ; 
void setup_tim3() ; 
void stop_pwm() ; 
void nano_wait(int);

void input_gpio(){

    RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB -> MODER &= ~0xf000; //clears PB6 PB7, set to input mode
    GPIOB -> PUPDR &= ~0xc003; //sets PUPDR to 0 for PB0, PB6, PB7
    GPIOB -> PUPDR = (1 << 15) | (1 << 13) | (1 << 1);
    
}

void search_for_flag(char * searched){
    duty = 0.75; 
    int flag_found = 0;
    int rotation_time = 1; //1 second rotation increment
    int total_rotation_time = 5; //5 seconds for a full 360-degree search routine
    //we might have to change total_rotation_time because i am not sure what makes it 360 degrees

    for (int elapsed_time = 0; elapsed_time < total_rotation_time; elapsed_time += rotation_time)
    {
        //start a right turn
        GPIOB -> BSRR = (1 << 8);
        GPIOB -> BSRR = (1 << 25);
        GPIOB -> BSRR = (1 << 26);
        GPIOB -> BSRR = (1 << 11);

        setup_tim3(); //enable PWM
        nano_wait(1000000000); //allows cart to rotate for 1 second
        stop_pwm(); //stop PWM after each rotation increment
        
        //deactivate GPIO signals to turn off motor
        GPIOB -> BSRR = (1 << 24); 
        GPIOB -> BSRR = (1 << 25);
        GPIOB -> BSRR = (1 << 26);
        GPIOB -> BSRR = (1 << 27);

        nano_wait(1000000000); 
        flag_found = ((GPIOB -> IDR) & 1); //checks if the signal is high from PB0 (assuming high means nothing is detected)

        if (flag_found)
        {
            duty = 0.75 ; 
            return; //exit loop if the flag is found
        }
    }

    //stops the cart if the flag is not found after search routine is finished
    if (!flag_found){
        *searched = 'T' ; 
        GPIOB -> BSRR = (1 << 24) | (1 << 25) | (1 << 26) | (1 << 27); //set GPIO to 0 to stop motor
    }
    duty = 0.75 ; 
}



void setup_tim3(void) {
    RCC -> APB1ENR |= RCC_APB1ENR_TIM3EN; //enable the clock for the TIM3 peripheral
    RCC -> AHBENR |= RCC_AHBENR_GPIOCEN; //enable the clock for the GPIOC peripheral

    GPIOC -> MODER &= ~0xf000; //set the mode to alternate function mode
    GPIOC -> MODER |= 0xa000;

    GPIOC -> AFR[0] &= ~0xff000000;

    TIM3 -> PSC = 10000 - 1; //set TIM3 auto-reload register such that the timer's frequency is 1 Hz
    
    int arr = 12 - 1;
    TIM3 -> ARR = arr; 
    
    // float duty_cycle1 = 70.0 / 100.0; //type in desired duty cycle
    // float duty_cycle2 = 70.0 / 100.0;

    TIM3 -> CCMR1 &= ~0x7070;
    TIM3 -> CCMR1 |= 0x6060;

    TIM3 -> CCER &= ~0xff;
    TIM3 -> CCER |= 0x11;

    TIM3 -> CR1 |= TIM_CR1_CEN; //enable the timer

    float ccr1 = (duty * (arr)); //do I need to add (arr + 1)
    float ccr2 = (duty * (arr));

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

    //sets pull down resistors
    GPIOB -> PUPDR &= ~0xc003; //sets PUPDR to 0 for PB0, PB6, PB7
    // GPIOB -> PUPDR = (1 << 15) | (1 << 13) | (1 << 1);
}

int main(void){
    internal_clock();
    init_gpio();
    stop_pwm();
    char searched = 'F' ; 
    int notFoundCount = 0 ;
    int dir = 0 ;
    //TURN OFF ALL GPIOS TO STOP MOTOR
    GPIOB -> BSRR = (1 << 24); 
    GPIOB -> BSRR = (1 << 25);
    GPIOB -> BSRR = (1 << 26);
    GPIOB -> BSRR = (1 << 27);

    //WAIT 5 SECONDS BEFORE ALLOWING THE CART TO MOVE
    nano_wait(1000000000);
    nano_wait(1000000000);
    nano_wait(1000000000);
    nano_wait(1000000000);
    nano_wait(1000000000);


    stop_pwm();

    while (1){

        /**************************************** */
        //CHECK BELOW LINE WITH SAM!!!!!!!!!!!!!
        //ideally, sending a 1 will mean that we see the flag
        //************************************ */
        
        int flag_detected = (GPIOB -> IDR) & 1; //check input flag status (assumes high if flag is detected) 
        if (!flag_detected)
        {
            notFoundCount += 200 ;
            if (notFoundCount >= 10000) notFoundCount = 10000 ; 
            if ((notFoundCount >= 10000) && searched == 'F') search_for_flag(&searched);
            nano_wait(200000000);
        }

        else
        {
            notFoundCount = 0 ;
            searched = 'F' ; 
            //there used to be a for loop here, but i removed it
                dir = ((GPIOB -> IDR) >> 6) & (3) ; 

                        //testing to see if third GPIO input works correctly
                        // //  NEW SECTION
                        //         input = ((GPIOB -> IDR) & 1);    

                        //         if (input == 1)
                        //         {//PB1
                        //             GPIOB -> BSRR = (1 << 1); //turns on PB1 if input is high
                        //         }

                        //         if (input == 0)
                        //         {//PB2
                        //             GPIOB -> BSRR = (1 << 2); //turns on if PB2 input is low
                        //         }

                        // //  NEW SECTION

                //move forward
                if (dir == 3) //11 for forward
                { 
                    GPIOB -> BSRR = (1 << 8); //output for PB8 left wheel(sets 1)
                    GPIOB -> BSRR = (1 << 25); //output for PB9 left wheel (sets 0)

                    GPIOB -> BSRR = (1 << 10); //output for PB10 right wheel (sets 1)
                    GPIOB -> BSRR = (1 << 27);//output for PB11 right wheel (sets 0);
                    //USART5 -> TDR = 'f';
                    setup_tim3(); //set up the PWM
                }

                //move right
                else if (dir == 2) //11 for right
                {
                    //move left wheel forward
                    GPIOB -> BSRR = (1 << 8); //output for PB8 left wheel(sets 1)
                    GPIOB -> BSRR = (1 << 25); //output for PB9 left wheel (sets 0)

                    //move right wheel backward
                    GPIOB -> BSRR = (1 << 26); //output for PB10 right wheel (sets 0)
                    GPIOB -> BSRR = (1 << 11); //output for PB11 right wheel (sets 1)
                    //USART5 -> TDR = 'r';
                    setup_tim3(); //set up the PWM
                }

                //move left
                else if (dir == 1)  //01 for left
                {
                    //move left wheel backward
                    GPIOB -> BSRR = (1 << 24); //output for PB8 left wheel (sets 0)
                    GPIOB -> BSRR = (1 << 9); //output for PB9 left wheel (sets 1)

                    //move right wheel forward
                    GPIOB -> BSRR = (1 << 10); //output for PB10 right wheel (sets 1)
                    GPIOB -> BSRR = (1 << 27); //output for PB11 right wheel (sets 0)
                    //USART5 -> TDR = 'l';
                    setup_tim3(); //set up the PWM
                }

            //enable PWM for 0.2s, the stop PWM
            //setup_tim3(); //set up the PWM
            nano_wait(200000000); //runs the motor for 0.2s
            //nano_wait(2000000000); //runs the motor for 2s
            stop_pwm(); //stop motor after 0.2s
            }

            //set all directional control GPIOs to 0
            GPIOB -> BSRR = (1 << 24); 
            GPIOB -> BSRR = (1 << 25);
            GPIOB -> BSRR = (1 << 26);
            GPIOB -> BSRR = (1 << 27);

       }
    }
