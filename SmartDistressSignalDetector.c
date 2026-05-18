//******************ECE 132*************************
//Names: Frank Tamburro and Alex Hume
//Lab section: 061
//*************************************************
//Date Started: 04/17/26
//Date of Last Modification: 05/03/26
//Project 2 - Smart Distress Signal Detector
//Lab Due Date: 05/03/26
//*************************************************
//Purpose of program: Prototype of system to detect repeated loud audio bursts to define a distress signal,
//and to respond with visual, mechanical, and UART alerts
//Program Inputs: Microphone, IR Sensor, SW2, Potentiometer, ADC
//Program Outputs: RED LED, GREEN LED, Servo motor, UART
//*************************************************
// Potentiometer Specifications:
// Minimum Resistance: 27.772 Ohms
// Maximum Resistance: 10,404 Ohms
//*************************************************


//Include statements
#include <stdbool.h>
#include <stdint.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"
#include "driverlib/pwm.h"
#include "driverlib/uart.h"
#include "driverlib/watchdog.h"

//UART pin definitions
#define GPIO_PA0_U0RX 0x00000001
#define GPIO_PA1_U0TX 0x00000401

//LED Definitions
#define RED_LED    0x02  //PE1, active alert
#define GREEN_LED  0x04  //PE2, monitoring

//Servo PWM pulse width definitions
//PWM clock = 50MHz / 64 = 781,250 Hz
//Period = 781,250 /50Hz = 15,625 ticks (20ms)
#define PWM_PERIOD  15625  //20ms period in ticks
#define PULSE_0     781    //1ms  = 0 degrees
#define PULSE_180   1562   //2ms  = 180 degrees

//SysTick reload value
#define T 0x0007A120

//audio detection window definition
#define WINDOW_SIZE 100 //number of SysTick ticks per window

//Global Variables
unsigned char cstate;    //current FSM state
unsigned char input;     //current FSM input
unsigned long adc_raw;   //raw ADC result from potentiometer (PE3, CH0)
int sound_count;         //counter for loud microphone bursts
volatile bool g_bWatchdogFeed = 1;   //watchdog feed flag
unsigned long high_tick_count = 0; //count of high PE5/mic ticks
unsigned long window_counter = 0; //total ticks in current window
unsigned long high_ticks_target = 30; //controlled by potentiometer, where number of high ticks = a loud event
//UART event variables
volatile bool loud_event_flag = false; //set by mic_isr when a burst is detected
volatile int loud_event_num = 0; //count of bursts since last reset
unsigned long prev_target =0; //last printed sensitivity threshold, for change detection
volatile bool tick_flag = false; //set by mic_isr each tick, used for delay

//pot-controlled burst count target, set as default here
unsigned long burst_target = 4;

//Reset button state variable
unsigned char button_prev = 1; //to avoid multiple uart outputs

//Moore-style FSM table
struct state{
    unsigned char out;     //output bitmask
    unsigned long delay;   //delay period set by tick_flag
    unsigned char next[4]; //next state indexed by inputs [no input, person detected, distress pattern met, reset button]
};
typedef struct state stype;
stype fsm[3]={
    {0b00, T, {0, 1, 0, 0}},  //IDLE     - both LEDs off, servo at 0 degrees
    {0b01, T, {0, 1, 2, 0}},  //MONITOR  - green LED on, servo at 0 degrees
    {0b10, T, {2, 2, 2, 0}},  //ALERT    - Red LED on, servo holds at 180 degrees
};

//Function prototype statements
void uart_print_window(unsigned long window_size);
void uart_print_loud_event(int num);
void uart_print_distress(void);
void uart_print_reset(void);
void mic_isr(void);
void update_output(unsigned char out);
int get_input(void);
void adc_read_threshold(void);
void uart_setup(void);
void adc_setup(void);
void servo_pwm_setup(void);
void servo_set_angle(int degrees);
void portF_input_setup(int pin);
void systick(int reload_val);
void portE_output_setup(int pin);
void portE_input_setup(int pin);
void WatchdogIntHandler(void);
void watchdog_setup(void);




int main(void)
{
    //setting system clock to 50MHz
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    //initializing SW2 (PF0) as an input, reset button
    portF_input_setup(0x01);

    //initializing PF1 as an input, for the IR Sensor
    portF_input_setup(0x02);

    //initializing PE5 as an input, for the Microphone
    portE_input_setup(0x20);

    //initializing PE1 as an output, Red LED
    portE_output_setup(RED_LED);
    //initializing PE2 as an output, Green LED
    portE_output_setup(GREEN_LED);

    //initializing ADC0 on PE3 (CH0) for potentiometer
    adc_setup();

    //initializing UART0 for terminal output
    uart_setup();

    //initializing PWM on PE4 (M0PWM4) for servo
    servo_pwm_setup();

    //SysTick configurations
    systick(T); //setting reload value and initializing SysTick registers
    SysTickIntRegister(mic_isr); //mic_isr acts as the SysTick ISR



    //enabling global interrupts
    IntMasterEnable();

    //initializing distress detection counters
    sound_count = 0;
    high_ticks_target= 30; //default, 30/100 ticks must be high to qualify loud event

    //start the system in IDLE state
    cstate = 0;
    //initializing watchdog
    watchdog_setup();
    while(1){
        //update output
        update_output(fsm[cstate].out);

        //Delay is measured, tick_flag set in mic_isr
        while(!tick_flag){
        }
        tick_flag = false;


        adc_read_threshold(); //read potentiometer, update threshold, print via UART
        //print loud event if flag was set by ISR
        if(loud_event_flag){
            uart_print_loud_event(loud_event_num);
            loud_event_flag = false;
        }

        //update input
        input = get_input();

        //print state change in UART based on get_input
        if(input == 2){
            uart_print_distress();    //distress just triggered
        }
        if(input == 3){
            uart_print_reset();       //reset just pressed
            loud_event_num = 0;       //reset the loud event counter too
        }
        //update state
        cstate = fsm[cstate].next[input];
    }

}

//function to print the current window sensitivity level to UART
void uart_print_window(unsigned long window_size){
    char window_msg[] = "WINDOW: ";
    int win_len= sizeof(window_msg) / sizeof(window_msg[0]);
    int i;
    for (i=0; i<win_len; i++){
       UARTCharPut(UART0_BASE, window_msg[i]);
    }

    if(window_size <= 2){
        char win_type1[] = "SHORT";
        int win_len1 = sizeof(win_type1) / sizeof(win_type1[0]);
        int j;
        for (j=0; j<win_len1; j++){
           UARTCharPut(UART0_BASE, win_type1[j]);
        }
        UARTCharPut(UART0_BASE,'\n');
        UARTCharPut(UART0_BASE,'\r');
    }
    else if(window_size <= 4){
        char win_type2[] = "MEDIUM";
        int win_len2 = sizeof(win_type2) / sizeof(win_type2[0]);
        int j;
        for (j=0; j<win_len2; j++){
           UARTCharPut(UART0_BASE, win_type2[j]);
        }
        UARTCharPut(UART0_BASE,'\n');
        UARTCharPut(UART0_BASE,'\r');
    }
    else{
        char win_type3[] = "LONG";
        int win_len3 = sizeof(win_type3) / sizeof(win_type3[0]);
        int j;
        for (j=0; j<win_len3; j++){
           UARTCharPut(UART0_BASE, win_type3[j]);
        }
        UARTCharPut(UART0_BASE,'\n');
        UARTCharPut(UART0_BASE,'\r');

    }

}
//function to print loud event notification to UART with event number (1-6)
void uart_print_loud_event(int num){
    char loud_msg[] = "LOUD EVENT #: ";
    int loud_len= sizeof(loud_msg) / sizeof(loud_msg[0]);
    int i;
    for ( i=0; i<loud_len; i++){
       UARTCharPut(UART0_BASE, loud_msg[i]);
    }
    //converting int to char in ASCII format
    int q = (num + 0x30);
    UARTCharPut(UART0_BASE, q);
    UARTCharPut(UART0_BASE,'\n');
    UARTCharPut(UART0_BASE,'\r');
}

//prints distress event confirmation when burst_target loud events are reached
void uart_print_distress(void){
    char dist_msg[] = "DISTRESS EVENT DETECTED";
    int dist_len= sizeof(dist_msg) / sizeof(dist_msg[0]);
    int i;
    for (i=0; i<dist_len; i++){
       UARTCharPut(UART0_BASE, dist_msg[i]);
    }
    UARTCharPut(UART0_BASE,'\n');
    UARTCharPut(UART0_BASE,'\r');
}

//prints system reset message when SW2 is pressed at any point
void uart_print_reset(void){
    char res_msg[] = "SYSTEM RESET";
    int res_len= sizeof(res_msg) / sizeof(res_msg[0]);
    int i;
    for (i=0; i<res_len; i++){
       UARTCharPut(UART0_BASE, res_msg[i]);
    }
    UARTCharPut(UART0_BASE,'\n');
    UARTCharPut(UART0_BASE,'\r');
}

//functions as SysTick ISR, fires quickly with set reload value
//samples the microphone inputs
void mic_isr(void){
    //used for delay, main loop must wait for microphone reading
    tick_flag = true;
    g_bWatchdogFeed = 1; //feeds the dog
    //only processes loud events if IR sensor detects a person, active low reading
    unsigned char ir_active = (0 == (GPIO_PORTF_DATA_R & 0x02));
    //skips detection if no person is present
    if(!ir_active){
        high_tick_count = 0;
        window_counter = 0;
        return;
    }

    //reads microphone high/low from PE5
    unsigned char mic_curr = (GPIO_PORTE_DATA_R & 0x20); //reads PE5 current state

    //count ticks where mic is HIGH
    if(mic_curr != 0){
        high_tick_count++;
    }

    window_counter++;

    //when window expires, evaluate count vs target
    if(window_counter >= WINDOW_SIZE){
        if(high_tick_count >= high_ticks_target){
            //enough HIGH ticks - counts as a loud event
            sound_count++;
            loud_event_num++;
            loud_event_flag = true;
        }
        //reset for next evaluation window
        high_tick_count = 0;
        window_counter = 0;
    }
}

//function to update LEDs and servo based on FSM output bitmask
void update_output(unsigned char out){
    //update Red LED (bit 1 of out)
    if(out & 0x02){
        GPIO_PORTE_DATA_R |= RED_LED;    //red LED on
    }
    else {
        GPIO_PORTE_DATA_R &= ~RED_LED;   //red LED off
    }

    //update Green LED, bit 0 of out
    if(out & 0x01){
        GPIO_PORTE_DATA_R |= GREEN_LED;  //green LED on
    }
    else {
        GPIO_PORTE_DATA_R &= ~GREEN_LED; //green LED off
    }

    //update servo based on state
    if(cstate == 0 || cstate == 1){
        servo_set_angle(0);   //IDLE and MONITOR - servo at 0 degrees
    } else {
        servo_set_angle(180); //DISTRESS and ALERT - servo at 180 degrees/raised
    }
}


//function to get current FSM input
int get_input(void){
    //detecting if SW2 is pressed to reset distress system (active low, highest priority)
    //need to detect state of button due to debouncing/clock issue, evident in UART output
    unsigned char button_curr = (GPIO_PORTF_DATA_R & 0x01);
    if( (button_prev != 0) && (button_curr ==0)){
        //resets system
        button_prev = button_curr;
        sound_count = 0;
        window_counter =0;
        high_tick_count=0;
        return 3;
    }
    button_prev = button_curr;



    //checking if distress pattern threshold has been met
    if(sound_count >= burst_target){
        sound_count = 0; //resetting count after distress detected
        high_tick_count =0;
        window_counter = 0; //clears burst window timer
        loud_event_num =0;
        return 2;
    }

    //detecting if a person is in proximity via IR sensor on PF1 (active low)
    if(0 == (GPIO_PORTF_DATA_R & 0x02)){
        return 1;
    }

    return 0;
}


//function to read ADC potentiometer value and update window length
//maps resistance range (27.772 - 10404 Ohms)
void adc_read_threshold(void){
    float voltage;
    float resistance;

    ADCProcessorTrigger(ADC0_BASE, 0);           //triggering ADC sample
    ADCSequenceDataGet(ADC0_BASE, 0, &adc_raw);  //retrieving ADC result

    //3 sensitivity levels based on potentiometer position
    //low resistance/low ADC = sensitive or fewer events needed
    //high resistance/high ADC = strict or  more events needed
    //short, 2 loud windos
    if(adc_raw < 1000){
        burst_target = 2;
    }
    //medium, 4 loud windows
    else if(adc_raw < 2800){
        burst_target = 4;
    }
    //long, 6 loud windows
    else{
        burst_target = 6;
    }

    //only prints in UART if level changes
    if(burst_target != prev_target){
        uart_print_window(burst_target);
        prev_target = burst_target;
    }


}


//UART setup function for terminal output on PA0/PA1
void uart_setup(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); //enabling UART0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); //enabling Port A clock
    GPIOPinConfigure(GPIO_PA0_U0RX);             //configuring PA0 as UART RX
    GPIOPinConfigure(GPIO_PA1_U0TX);             //configuring PA1 as UART TX
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1); //setting PA0 and PA1 as UART pins
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,  //configuring UART at 115200bps
    (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
}




//ADC setup function for potentiometer on PE3 (ADC0 Channel 0)
void adc_setup(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);          //enabling ADC0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);         //enabling Port E clock
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);         //configuring PE3 as ADC input
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0); //processor triggered, priority 0
    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_IE | ADC_CTL_END | ADC_CTL_CH0); //channel 0, PE3
    ADCSequenceEnable(ADC0_BASE, 0);                     //enabling sequencer 0
}


//PWM setup function for SG90 servo on PE4 (M0PWM4)
//50Hz signal = 20ms period; pulse width 1ms-2ms controls 0-180 degrees
void servo_pwm_setup(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);          //enabling PWM0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);         //enabling Port E clock
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));   //waiting for PWM0 to be ready
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);                 //dividing system clock by 64
    GPIOPinConfigure(GPIO_PE4_M0PWM4);                   //configuring PE4 as M0PWM4
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);         //setting PE4 as PWM output
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC); //configuring generator 2
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, PWM_PERIOD);  //setting period
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, PULSE_0);    //starting at 0 degrees
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);                  //enabling generator 2
    PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);      //enabling PWM output on PE4
}


//function to set servo angle (0-180 degrees)
//maps degrees to pulse width: 1ms (0 deg) to 2ms (180 deg)
void servo_set_angle(int degrees){
    uint32_t pulse_width;
    pulse_width = PULSE_0 + (degrees * (PULSE_180 - PULSE_0)) / 180; //linear mapping 0-180 to pulse range
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, pulse_width);
}


//Port F input function to initialize SW2 and IR Sensor as inputs
void portF_input_setup(int pin){
    SYSCTL_RCGCGPIO_R |= 0x20;           //setting system clock enable to Port F
    GPIO_PORTF_LOCK_R |= 0x4C4F434B;     //explicit command to unlock register
    GPIO_PORTF_CR_R   |= pin;            //configures the commit register
    GPIO_PORTF_DIR_R  &= ~(pin);         //setting direction to an input
    GPIO_PORTF_PUR_R  |=  pin;           //setting the PUPD resistor
    GPIO_PORTF_DEN_R  |=  pin;           //setting data enable
}


//function "systick" to set up SysTick registers
//void return, input reload_val is an int
void systick(int reload_val){
    NVIC_ST_CTRL_R    = 0x0;        //disabling the counting bit
    NVIC_ST_RELOAD_R  = reload_val; //setting reload value
    NVIC_ST_CURRENT_R = 0x0;        //clearing the counter for the clock
    NVIC_ST_CTRL_R    = 0x7;
}


//Output function specific to Port E to drive LED outputs
//void return, input "pin" is an int
void portE_output_setup(int pin){
    SYSCTL_RCGCGPIO_R |= 0x10; //setting system clock enable to Port E
    GPIO_PORTE_DIR_R  |=  pin; //setting output direction
    GPIO_PORTE_DEN_R  |=  pin; //setting data enable
}


//Input function specific to Port E - used for Microphone
void portE_input_setup(int pin){
    SYSCTL_RCGCGPIO_R |= 0x10;   //setting system clock enable to Port E
    GPIO_PORTE_DIR_R  &= ~(pin); //setting direction to an input
    GPIO_PORTE_DEN_R  |=  pin;   //setting data enable
}


/**********************************************NOTE*******************************************/
//In the demonstration of the prototype, the WatchDog implementation resulted in the system fully
//resetting, despite proper "feeding." This led to an in-depth debugging process. We first examined
//the feeding process, within the mic_isr() function and then moved onto checking our full WatchDog setup.
//To check validity of our handler, we put in temporary print checks to see if the handler was even being called
//, it was not. After using breakpoint debugging and iterating through our program we still could not define the issue.
//The system reset occurred in our setup and the system issue could not be identified. After evaluating our options
//and speaking with our professor, we were forced to not implement watchdog in our prototype.

//Watchdog ISR that feeds the dog if g_bWatchdogFeed is set
void WatchdogIntHandler(void){
    if(g_bWatchdogFeed){
        WatchdogIntClear(WATCHDOG0_BASE);   //clear interrupt = fed the dog
        g_bWatchdogFeed = 0;
    }
    else{
        GPIO_PORTE_DATA_R &= ~RED_LED;
        GPIO_PORTE_DATA_R &= ~GREEN_LED;
        servo_set_angle(0);
        sound_count =0;
        high_tick_count=0;
        window_counter=0;
        loud_event_num =0;
        g_bWatchdogFeed=1;
    }

}

//Watchdog setup
void watchdog_setup(void){
    //enables the peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);

    //waits for module to be ready
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_WDOG0)){}

    //enables the Watchdog interrupt
    IntEnable(INT_WATCHDOG);

    //checks that register access is unlocked
    if(WatchdogLockState(WATCHDOG0_BASE) == true){
        WatchdogUnlock(WATCHDOG0_BASE);
    }

    //enables the real watchdog interrupt
    WatchdogIntEnable(WATCHDOG0_BASE);
    WatchdogIntTypeSet(WATCHDOG0_BASE, WATCHDOG_INT_TYPE_INT);

    //sets period for 1 s which is 2 s total to reset
    WatchdogReloadSet(WATCHDOG0_BASE, SysCtlClockGet()/3);

    //enables resetting if not fed
    WatchdogResetEnable(WATCHDOG0_BASE);

    //locks in the configuration
    WatchdogLock(WATCHDOG0_BASE);

    //turns on the watchdog
    WatchdogEnable(WATCHDOG0_BASE);
}
