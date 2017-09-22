/**
@file main.h
@brief Header file contains functions and variables
@brief Fast Lock-In Amplifier - AC to DC converter
@brief Revision 1.0
@author Nikollao Sulollari
@Date 16/08/2017
*/

#ifndef MAIN_H
#define MAIN_H
#include "mbed.h"
#include "N5110.h"

#define DIRECTION_TOLERANCE 0.05

/**
@namespace aout
@brief Analog output DC filtered value 
*/
AnalogOut aout(DAC0_OUT);

/**
@namespace gpo
@brief Digital output set the D0 pin High/Low
*/
DigitalOut gpo(D0);

/**
@namespace ain
@brief analog input which is the signal from the bolometer (THz detector)
*/
AnalogIn ain(A5);

/**
@namespace dref
@brief event triggered interrupt used to calculate the reference frequency
*/
InterruptIn dref(D6);

/**
@namespace menu_ticker
@brief time-triggered interrupt to update the menu of the LIA
*/
Ticker menu_ticker;

/**
@namespace amp_ticker
@brief time-triggered interrupt calculates the offset of the input signal
*/
Ticker offset_ticker;

/**
@namespace sample_ticker
@brief time-triggered interrupt samples internal sine and cosine signals
*/
Ticker sample_ticker;

/**
@namespace output_ticker
@brief time-triggered interrupt updates the output of Fast LIA 
*/
Ticker output_ticker;

/**
@namespace period_timer
@brief timer is used to calculate the frequency of the input square wave
*/
Timer period_timer;


/**
@namespace period_timer
@brief timer updates the output of the LIA depending on the time constant
*/
Timer output_timer;
/**
@namespace lcd
@brief object of the N5110 class
*/
N5110 lcd(PTE26 , PTA0 , PTC4 , PTD0 , PTD2 , PTD1 , PTC3);


/**
@namespace pc
@brief serial connection between mbed and pc
*/
Serial pc(USBTX,USBRX);

/**
@namespace xPot
@brief read x-axis position from the value of the joystick
*/
AnalogIn xPot(PTB2);

/**
@namespace yPot
@brief read y-axis position from the value of the joystick
*/
AnalogIn yPot(PTB3);

/**
@namespace joyButton
@brief interrupt executes an event triggered task when joyButton is pressed
*/
InterruptIn joyButton(PTB11);

/**
@namespace swButton
@brief interrupt executes an event triggered task
*/
InterruptIn swButton(PTB18);

/**
@namespace DirectionName
@brief define joystick's direction based on its x,y values
*/
enum DirectionName {
    UP,
    DOWN,
    LEFT,
    RIGHT,
    CENTRE,
};

/**
@namespace Joystick
@brief create strcut Joystick
*/
typedef struct JoyStick Joystick;
struct JoyStick {
    double x;    /// current x value
    double x0;   /// 'centred' x value
    double y;    /// current y value
    double y0;   /// 'centred' y value
    int button; /// button state (assume pull-down used, so 1 = pressed, 0 = unpressed)
    DirectionName direction;  // current direction
};
/// create struct variable
Joystick joystick;

/**
update position of joystick
*/
void updateJoystick();

/**
set origin of joystick
*/
void calibrateJoystick();

/**
set-up serial port
*/
void init_serial();

/**
initialise lcd
*/
void lcd_intro();

/**
configure settings of the LIA, sensitivity (gain) and update output (speed) 
*/
void settings_menu();

/**
setup the position of the menu selector (circle)
*/
void setup_selector();


/**
init the LIA menu
*/
void init_LIA_menu();

/**
joystick button pressed
*/
void confirmationJoyButton();

/**
set a timeout interrupt to implement power-efficient delays
*/
void timeout_isr();

/**
set a timeout interrupt to update the menu on screen
*/
void menu_isr();

/**
Event-triggered interrupt executes ISR  when joystick button is pressed
*/
void joyButton_isr();

/**
Event-triggered interrupt executes ISR  when joystick button is pressed
*/
void swButton_isr();

/**
use MCU in High Power Mode and set Core, ADC, and BUS clocks at max
*/

void setupK64Fclocks();

/**
initialise DAC pin, otherwise AnalogOut (dac0_out) does not work
*/
void initDAC();

/**
find the amplitude of the analog bolometer signal
*/
double max(int points);

double mavg_filter(int filt_points);

/**
performs digital mixing with refX and refY components 
*/
void digitalMix(double remove_offset);

/**
ISR calculates the DigitalIn signal frequency
*/
void voltageRise();

/**
ISR calculates the offset of the bolometer signal
*/
void offset_isr();

/**
ISR updates the output of the LIA 
*/
void output_isr();

//double calculate_constant(double freq_ref);

/*!< used to assign the amplitude of the bolometer signal */
volatile double amplitude = 0; 

/*!< stores the period of the input digital signal */
volatile double ref_period = 0;

/*!< stores the frequency of the input digital signal */
volatile double ref_freq = 0;

/*!< counter is used to find take between two rises of digital signal*/
volatile int g_counter = 1; 

/*!< flag is set from ISR to find offset of the bolometer signal */
volatile int g_offset_flag = 0;

/*!< sampling period to take n samples and find bolometer signal amplitude */
volatile double amplitude_delay = 0;

/*!< flag used to update output */
volatile int g_output_flag = 0;

/*!< flag used to update menu */
volatile int g_menu_flag = 0;

/*!< flag is set when joystick button is pressed */
volatile int g_joyButton_flag = 0;

/*!< flag is set when joystick button is pressed */
volatile int g_swButton_flag = 0;

/*!< flag used to sample internal sine/cosine signals */
volatile int g_sample_flag = 0;

/*!< variable used to move the menu selector */
volatile int menu_option = 0;

/*!< variable used to move the continue selector */
volatile int save_option = 0;

/*!< var is set to exit the menu */
volatile int exit_menu = 0;

/*!< var is used to set the output gain */
volatile double var_gain = 1.0;

/*!< var is used to set the output speed */
volatile double var_speed = 1.0;

/*!< array stores the square root of X^2 and Y^2 */
volatile double R[16];

/*!< store the bolometer signal and find max value */
double bolometer_signal[8];

/*!< store the bolometer signal and find max value */
double bol_signal;
//int size = 16;

/*!< variable used if a sine/cosine of 8 discrete values is used */
int samples8 = 8;

/*!< variable used if a sine/cosine of 17 discrete values is used */
int samples16 = 16;

/*!< variable used if a sine/cosine of 32 discrete values is used */
int samples32 = 32;

/*!< sampling frequency of the internal sine/cosine waves */
double sample_freq = 0;

/*!< variable used to sample the internal sine/cosine waves */
double sample_time = 0;

/*!< number of samples for the amplitude of the bolometer signal */
int amp_points = 32;

/*!< sampling freq to find the amplitude of the bolomter signal */
double delay_freq = 0;

/*!< read the timer and if it is equal to the time constant update output */
double time_out = 0;

int filter_points = 500;

double offset = 0;

/*!< sine wave array with 32 values */
static double sin_array32[32] = {0, 0.1951, 0.3827, 0.5556, 0.7071, 0.8315, 0.9239,
                                 0.9808, 1, 0.9808, 0.9239, 0.8315, 0.7071, 0.5556,
                                 0.3827, 0.1951, 0, -0.1951, -0.3827, -0.5556, -0.7071,
                                 -0.8315, -0.9239, -0.9808, -1, -0.9808, -0.9239, -0.8315,
                                 -0.7071, -0.5556, -0.3827, -0.1951
                                };

/*!< cosine wave array with 32 values */
static double cos_array32[32] = {1, 0.9808, 0.9239, 0.8315, 0.7071, 0.5556,
                                 0.3827, 0.1951, 0, -0.1951, -0.3827, -0.5556, -0.7071,
                                 -0.8315, -0.9239, -0.9808, -1, -0.9808, -0.9239, -0.8315,
                                 -0.7071, -0.5556, -0.3827, -0.1951, 0, 0.1951, 0.3827,
                                 0.5556, 0.7071, 0.8315, 0.9239, 0.9808
                                };

/*!< sine wave array with 17 values */
static double sin_array16[16] = {0, 0.3827, 0.7071, 0.9239, 1, 0.9239, 0.7071,
                                 0.3827, 0, -0.3827, -0.7071, -0.9239, -1, -0.9239
                                 ,-0.7071, -0.3827,
                                };

/*!< sine wave array with delay and 16 values, used for testing output */
static double sin_array16delay[16] = {0.7071, 0.9239, 1, 0.9239, 0.7071,
                               0.3827, 0, -0.3827, -0.7071, -0.9239, -1, -0.9239
                               ,-0.7071, -0.3827, 0, 0.3827
                              }; 

/*!< cosine wave array with 17 values */
static double cos_array16[16] = {1, 0.9239, 0.7071,
                                 0.3827, 0, -0.3827, -0.7071, -0.9239, -1, -0.9239
                                 ,-0.7071, -0.3827, 0, 0.3827, 0.7071, 0.9239, 
                                };

/*!< sine wave array with 8 values */
static double sin_array8[8] = {0, 0.7071, 1, 0.7071, 0, -0.7071, -1, -0.7071};

/*!< cosine wave array with 8 values */
static double cos_array8[8] = {1, 0.7071, 0, -0.7071, -1, -0.7071, 0, 0.7071};

#endif