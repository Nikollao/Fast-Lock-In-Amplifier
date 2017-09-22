#include "main.h"

int main()
{
    lcd_intro();
    calibrateJoystick(); ///calibrate joystick
    settings_menu();
    lcd.clear();
    dref.rise(&voltageRise); /// set interrupt to calculate reference frequency
    setupK64Fclocks();
    //settings_menu();
    /// initialise DAC output dac0_out
    while (ref_freq < 1e2) {
        sleep();
    }
    double freq_check = ref_freq*0.001;
    double out_speed = 2*var_speed;
    
    if (freq_check < out_speed) {
        lcd.printString("RefFreq Low",0,0);
        lcd.printString("OR: ",0,1);
        lcd.printString("AnOut too Fast",0,2);
        
        while (freq_check < out_speed) {
            sleep();
        }
    }
    /// make sure frequency is read before we go to the program
    /// cancel event-triggered rise interrupt, not to interfere with program
    dref.rise(NULL);
    pc.printf("Ref_Freq is:%.2f kHz\n\r",ref_freq*0.001);
    lcd.clear();
    char refFreq_char[20]; ///create an array of chars
    sprintf(refFreq_char,"%.3f",ref_freq); ///create string
    
    lcd.printString("Ref Freq: ",0,0);        
    lcd.printString(refFreq_char,35,1);
    /// constant 6 for correct sampling time 
    /// compensates for delay caused by computations
    sample_freq = 6*samples16*ref_freq;
    sample_time = 1/sample_freq;

    initDAC();
    delay_freq = ref_freq*amp_points;
    amplitude_delay = 1/delay_freq;
    
    wait(1);
    lcd.clear();
    lcd.printString("LIA ",0,0);
    lcd.printString("In process...",0,1);
    lcd.refresh();
    /// find the offset of the signal
    offset_ticker.attach(&offset_isr,0.001);
    
    double update_value = 0.0;
    
    while (offset == 0) {
        if (g_offset_flag == 1) {
            g_offset_flag = 0;
            offset = mavg_filter(filter_points);
        }
        sleep();
    }
    offset_ticker.detach();
    /// once the offset is calculated detach the offset ticker
    /// attach the output ticker to update every x ms
    update_value = var_speed*0.001;
    output_ticker.attach(&output_isr,update_value);
    
    while (true) {
        // gpo = !gpo;
        digitalMix(offset); /// perform digital mixing 
        while (g_output_flag == 0) {sleep();} /// sleep until flag is set
        /// update output
        if (g_output_flag == 1) {
            g_output_flag = 0;
            //aout = max(samples16);
            aout = var_gain*max(samples16); 
            /// DC output by taking the maximum value of the mixed signal (R)
        }
    }
}

double max(int points)
{
    double amp = 0;

    for (int i = 0; i < points; i++) {
        if (amp < R[i])
            amp = R[i]; /// find max of R
        //wait(amplitude_delay);
    }
    return amp;
}

double mavg_filter(int filt_points)
{
    double avg = 0, signal = 0;
    //double delay = 0.9/(1*ref_freq*filter_points);
    for (int i = 0; i < filter_points; i++) {
        signal = ain.read();
        avg = avg + signal;
         wait((float)(5e-5));
    }
    avg = avg/filter_points; /// find offset of input signal
    return avg;
}

void digitalMix(double remove_offset) {
    /// perform mixing of input and reference signals
    double input = 0;
    for (int i = 0; i < samples16;i++) {
        /// remove the offset before doing the multiplication of signals
        input = ain.read()-remove_offset;
        /// find the X component by multiplying with sine 17 values array
        double refX = input*sin_array16[i];
        /// find the Y component by multiplying with cosine 17 values array
        double refY = input*cos_array16[i];
        //double XY = exp(2*log(refX))+exp(2*log(refY));
        double XY = (refX*refX)+(refY*refY); /// R square
        //double R = exp(0.5*log(XY))/4;
        R[i] = pow(XY,0.5); /// R
        //aout = (1+sin_array16[i])/4;
        //aout = R[i]/2;
        wait(sample_time); /// sample time
    }
}

void voltageRise() {
    if (g_counter == 1) { 
        /// first time function is called is the first rise
        /// start timer
        period_timer.start();
        /// increase counter so next time function is called we calculate freq.
        g_counter++; 
    }
    else if (g_counter == 2) {
        /// second time function is called is the second rise 
        /// stop timer
        period_timer.stop();  
        /// calculate the time taken between the two rises to find period
        ref_period = period_timer.read();
        /// frequency is the inverse of the signal period
        ref_freq = 1/ref_period;
        /// reset timer
        period_timer.reset();
        /// increase counter because we only want to calculate once per cycle
        /// if we want to actively track the ref_freq we should decrease counter
        g_counter++;
    }
}

void setupK64Fclocks() {
    if(1) {
        uint32_t div1=0,div2=0,busClk=0,adcClk=0;
        SystemCoreClockUpdate();
        pc.printf("SystemCoreClock= %u \r\n",SystemCoreClock);
        /// System Core Clock: 120 MHz
        div1=( (SIM->CLKDIV1) & SIM_CLKDIV1_OUTDIV1_MASK)>>SIM_CLKDIV1_OUTDIV1_SHIFT;
        div1=1+div1;
        div2=1+( (SIM->CLKDIV1) &    SIM_CLKDIV1_OUTDIV2_MASK)>>SIM_CLKDIV1_OUTDIV2_SHIFT;
        busClk=SystemCoreClock*div1/div2;
        pc.printf("Divider1== %u div2=%u \r\n",div1,div2);
        pc.printf("MCGOUTCLK= %u,  busClk = %u \r\n",SystemCoreClock*div1,busClk);
        /// MCGOUTCLK 120 MHz, Bus Clock = 120 MHz
        ADC1->SC3 &= ~ADC_SC3_AVGE_MASK;//disable averages
        ADC1->CFG1 &= ~ADC_CFG1_ADLPC_MASK;//high-power mode
        ADC1->CFG1 &= ~0x0063 ; //clears ADICLK and ADIV
        ADC1->CFG1 |= ADC_CFG1_ADIV(2); //divide clock 0=/1, 1=/2, 2=/4, 3=/8
        //ADC0->SC3 |= 0x0007;//enable 32 averages

        if (((ADC1->CFG1)& 0x03) == 0) adcClk = busClk/(0x01<<(((ADC1->CFG1)&0x60)>>5));
        if (((ADC1->SC3)& 0x04) != 0) adcClk = adcClk/(0x01<<(((ADC1->SC3)&0x03)+2));
        pc.printf("adcCLK= %u  \r\n",adcClk);
        /// ADC Clock: 60 MHz
    }   
}

void offset_isr() {
    g_offset_flag = 1;   
}

void output_isr() {
    g_output_flag = 1;   
}

void initDAC() {
        DAC0->C0 = 0;
        DAC0->C1 = 0; //reset DAC state
        DAC0->C0 = DAC_C0_DACEN_MASK | DAC_C0_DACSWTRG_MASK| DAC_C0_DACRFS_MASK;
}   
void lcd_intro()
{
    pc.baud(115200);
    joyButton.rise(&joyButton_isr); ///assign rise with ISR
    joyButton.mode(PullDown); ///use PullDown mode
    swButton.rise(&swButton_isr); ///assign rise with ISR
    swButton.mode(PullDown); ///use PullDown mode
    lcd.init();
    lcd.setBrightness(0.7); // put LED backlight on 50%
    lcd.printString("THE CIRCUIT IS",1,1);
    lcd.printString("A FAST LIA!",7,3);
    lcd.refresh();
    Timeout timeout;
    timeout.attach(&timeout_isr,3);
    sleep();
    lcd.clear();
}

void settings_menu()
{
    lcd.setBrightness(0.7); // put LED backlight on 50%
    menu_ticker.attach(&menu_isr,0.2);

    while (exit_menu == 0) {
        if (g_menu_flag == 1) {
            g_menu_flag = 0;
            
            updateJoystick();
            init_LIA_menu();
            setup_selector();
            
            lcd.refresh();
            
            
        }
        sleep();
    }
    menu_ticker.detach();
}

void init_LIA_menu() {
       lcd.clear();
            
            lcd.printString("Settings:",0,0);
            
            char gain_char[20]; ///create an array of chars
            
            sprintf(gain_char,"%.2f",var_gain); ///create string
            
             char speed_char[20]; ///create an array of chars
            sprintf(speed_char,"%.2f",var_speed); ///create string
            
            lcd.printString("Gain: ",0,1);
            lcd.printString(gain_char,35,2);
            lcd.printString("Speed (ms)",0,3);
            lcd.printString(speed_char,35,4);
            lcd.printString("Confirm",0,5);
}

void setup_selector()
{
    switch (joystick.direction) { ///check the direction of joystick
        case UP:
            menu_option--;
            break;
        case DOWN:
            menu_option++;
            break;
        case RIGHT:
            switch (menu_option) {
                case 0:
                    var_gain += 0.1;
                    if (var_gain > 3)
                        var_gain = 0.1;
                    break;
                case 1:
                    var_speed++;
                    if (var_speed > 10)
                        var_speed = 1;
                    break;
            }
            break;
        case LEFT:
            switch (menu_option) {
                case 0:
                    var_gain -= 0.1;
                    if (var_gain < 0.1)
                        var_gain = 3;
                    break;
                case 1:
                    var_speed--;
                    if (var_speed < 1)
                        var_speed = 10;
                    break;
            }
            break;
    }
    if (menu_option < 0) { /// if the last (down) option is set for selection and user presses joystick down, selector moves at the top
        menu_option = 2;
    }
    if (menu_option > 2) { /// if the first (up) option is set for selection and user presses joystick up, selector moves at the bottom
        menu_option = 0;
    }

    if (menu_option == 0) { ///selection in menu depends on the value of int option
        lcd.drawCircle(75,10,3,1);
    } else if (menu_option == 1) {
        lcd.drawCircle(75,27,3,1);
    } else if (menu_option == 2) {
        lcd.drawCircle(55,43,3,1);
        if (g_joyButton_flag == 1) {
            g_joyButton_flag = 0;
            confirmationJoyButton();   
        }
    }
    g_joyButton_flag = 0;
}

void confirmationJoyButton () {
    
    bool confirm = 1;
    while (confirm) {
        lcd.clear();
        lcd.printString("Continue ?",0,0);
        lcd.printString("Yes",0,2);
        lcd.printString("No",0,3);
        updateJoystick();
        switch (joystick.direction) { ///check the direction of joystick
            case UP:
                save_option--;
                break;
            case DOWN:
                save_option++;
                break;
        }

        if (save_option < 0) { /// if the last (down) option is set for selection and user presses joystick down, selector moves at the top
            save_option = 1;
        }
        if (save_option > 1) { /// if the first (up) option is set for selection and user presses joystick up, selector moves at the bottom
            save_option = 0;
        }

        if (save_option == 0) {
            lcd.drawCircle(30,19,3,1);
        }
        else if (save_option == 1) { 
            lcd.drawCircle(30,28,3,1);
        }
        if (g_swButton_flag == 1) {
            g_swButton_flag = 0;
            if (save_option == 0) {
                confirm = 0; // exit continue
                exit_menu = 1; // exit menu   
            }
            else if (save_option == 1) {
                confirm = 0; // just exit continue   
            }
        }
        lcd.refresh();
        sleep();
    }
}

void menu_isr() {
    g_menu_flag = 1;   
}

void timeout_isr() {}

void joyButton_isr() {
    g_joyButton_flag = 1;   
}


void swButton_isr() {
    g_swButton_flag = 1;   
}

void calibrateJoystick()
{
    // must not move during calibration
    joystick.x0 = xPot;  // initial positions in the range 0.0 to 1.0 (0.5 if centred exactly)
    joystick.y0 = yPot;
}

void updateJoystick()
{
    // read current joystick values relative to calibrated values (in range -0.5 to 0.5, 0.0 is centred)
    joystick.x = xPot - joystick.x0;
    joystick.y = yPot - joystick.y0;
    // read button state
    joystick.button = joyButton;

    // calculate direction depending on x,y values
    // tolerance allows a little lee-way in case joystick not exactly in the stated direction
    if ( fabs(joystick.y) < DIRECTION_TOLERANCE && fabs(joystick.x) < DIRECTION_TOLERANCE) {
        joystick.direction = CENTRE;
    } else if ( joystick.y > DIRECTION_TOLERANCE && fabs(joystick.x) < DIRECTION_TOLERANCE) {
        joystick.direction = UP;
    } else if ( joystick.y < DIRECTION_TOLERANCE && fabs(joystick.x) < DIRECTION_TOLERANCE) {
        joystick.direction = DOWN;
    } else if ( joystick.x > DIRECTION_TOLERANCE && fabs(joystick.y) < DIRECTION_TOLERANCE) {
        joystick.direction = LEFT;
    } else if ( joystick.x < DIRECTION_TOLERANCE && fabs(joystick.y) < DIRECTION_TOLERANCE) {
        joystick.direction = RIGHT;
    }/*
     else if (joystick.y > DIRECTION_TOLERANCE  && joystick.x  < DIRECTION_TOLERANCE) {
        joystick.direction = UP_LEFT;
    } else if (joystick.y > DIRECTION_TOLERANCE && joystick.x > DIRECTION_TOLERANCE) {
        joystick.direction = UP_RIGHT;
    } else if (joystick.y < DIRECTION_TOLERANCE && joystick.x < DIRECTION_TOLERANCE) {
        joystick.direction = DOWN_LEFT;
    }   else if (joystick.y < DIRECTION_TOLERANCE && joystick.x > DIRECTION_TOLERANCE) {
        joystick.direction = DOWN_RIGHT;
    }
    */
}
