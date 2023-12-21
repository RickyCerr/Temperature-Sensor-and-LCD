#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "/home/ricky/Desktop/ECE473/lab6/uart_lib_x.h"
#include "/home/ricky/Desktop/ECE473/lab6/twi_master.h"
#include "/home/ricky/Desktop/ECE473/lab6/buffers.h"
#include "/home/ricky/Desktop/ECE473/lab6/glcd_lib_x.h"

twi_master_t sens1;
twi_master_t sens2;


//32Mhz internal RC oscillator initalization
void init_32Mhzclock(void){
//setup clock for 32Mhz RC internal oscillator
  OSC.CTRL = OSC_RC32MEN_bm; // enable 32MHz clock
  while (!(OSC.STATUS & OSC_RC32MRDY_bm)); // wait for clock to be ready
  CCP = CCP_IOREG_gc; // enable protected register change
  CLK.CTRL = CLK_SCLKSEL_RC32M_gc; // switch to 32MHz clock
}

//Initalize TCC0 to normal mode
//TCCO is used to as an interrupt source for updating the frequency of
//the tone being created. Thus, it does not need to use an output pin.
//Running in normal mode does not do any port override if no WGM mode
//is set and CC channel is not enabled. See pg. 173 of AU manual.

void init_TCC0_normal(){
//setup timer for normal mode, interrupt on overflow
  TCC0_CTRLA     |= TC_CLKSEL_DIV1024_gc;
  TCC0_CTRLB     |= TC_WGMODE_NORMAL_gc;
  TCC0_PER       = 31249; // sets the ISR to repeat every second
  TCC0_INTCTRLA  = TC0_OVFINTLVL0_bm;     //interrupt level bit zero set
}


///////////////////////////////////////////////////////////////////////////////
void init_SPIC(void){
//SPI for port C
//SPI enabled, master mode, data mode0, MSB 1st, clk/4 
//  PORTC.DIRSET  |= (1<<PIN7_bp) | (1<< PIN5_bp) | (1<<PIN4_bp) | (1<<PIN0_bp); //turn on SPI outputs
  PORTC.DIRSET  |= (1<<PIN7_bp) | (1<< PIN5_bp) | (1<<PIN4_bp); //turn on SPI outputs TODO: BUG ABOVE?
  SPIC.CTRL     |= (1<<SPI_CLK2X_bp) | (1<< SPI_ENABLE_bp) | (1<<SPI_MASTER_bp); 
//  SPIC.CTRL    = (1<< SPI_ENABLE_bp) | (1<<SPI_MASTER_bp) | (0x01); //clock div 8, 2mhz 
  SPIC.INTCTRL  = 0x00; //no interrupts                
  }
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
//setup TC0 for single slope PWM 
void init_TCD1_PWM(void){
  TCD1_CTRLA |= TC_CLKSEL_DIV8_gc; //sysclock div 8  
  TCD1_CTRLB |= TC_WGMODE_SINGLESLOPE_gc | TC0_CCAEN_bm; //single slope, pin on means PC0 because of CCAENable
  TCD1_CCA = 0xFF;    //0xD0 compare level set (bigger values=dimmer)
  TCD1_PER = 0x00f0;  //period set
  PORTC.DIRSET |= PIN0_bm;  //PC.0 is the PWM output. (OC0A)
}
///////////////////////////////////////////////////////////////////////////////


float round_nearest_qtr(float number) { //rounds the numbers to the nears quarter (.00 .25 .50 .75)
    int num_whole_scaled = (int)(number * 100); // converts the number to a whole number
    int remainder = num_whole_scaled % 25; //calculates the remainder from dividing by 25

    if (remainder <= 12) {  //if the remainder is closer (25/2 = 12) so anything less than 12 should be mean its close to one or the other
        num_whole_scaled -= remainder; // subtract the number by the remainder 
    } else {
        num_whole_scaled += (25 - remainder); // other wise go the other direction
    }

    return (float)num_whole_scaled / 100.0f; // return it to a fload with the decimals
}

//ISR for TCC0 
//Overflow will occur with TCD0_CCA match. Increases/decreases frequency
//as a fraction of the frequency, not as a fixed value. 
//

ISR(TCC0_OVF_vect){
   
}


void insert_dot(char *str) { //adds a '.' before the second to last char in a string
    int len = strlen(str);

    // Ensure the string has at least two characters
    if (len >= 2) {
        // Shift characters to the right to make space for the '.'
        memmove(&str[len - 1], &str[len - 2], 3); // Move last two characters and null terminator
        str[len - 2] = '.'; // Insert the '.' before the last two characters
    }
}


int main(){
  init_32Mhzclock(); //set clock to 32Mhz
  init_TCD1_PWM(); //single slope PWM for dimming

  init_TCC0_normal();
  //sei();

  //PC2 is for dimming the LCD
  //PA0 is lcd_A0 for addressing LCD
  //PA1 is lcd_rst_n for LCD 
  //PC4 is CS1_n for LCD (set in spi initaliaztion)
  //PC0 is PWM 
  //PC7 is SCK
  //PC5 is MOSI

  //SDA -> PE0
  //SCL -> PE1

  PORTA.DIRSET  = (1<<PIN0_bp) | (1<<PIN1_bp); //lcd_A0 and lcd_rst_n respectively 

  init_SPIC(); //setup SPI
  init_glcd(); //initialize the LCD

  PMIC.CTRL |= PMIC_LOLVLEN_bm; // twi uses interrupts
  sei();

  twi_module_init(&TWIE, 35, TWI_MASTER_INTLVL_LO_gc); //initialize twi
  twi_struct_init(&sens1, &TWIE, 0b10010010);
  twi_struct_init(&sens2, &TWIE, 0b10011000); //last three are the sensor IDs

  char temp1_str[10];
  char temp2_str[10];

  char display1[] = "Temperature 1:"; //default message for temp sensor 1
  char display2[] = "Temperature 2:"; //default messahe for temp sensor 2

  uint16_t tempsnsr_1_c = 0;
  uint16_t tempsnsr_2_c = 0;

  float tempsnsr_1_f = 0;
  float tempsnsr_2_f = 0;

  float temp1_rounded = 0;
  float temp2_rounded = 0;

  int fake_temp1 = 0;
  int fake_temp2 = 0;

  while(1){ 

    twi_master_wr_rd(&sens1,0,2); //write 0 bits, read 2 bits, read snesor data into the sens1 struct
    while(sens1.status != TWIM_STATUS_READY); //wait until something can be read
    tempsnsr_1_c = (sens1.rd_array[0] << 3) | sens1.rd_array[1]>>5; //read the data from the temp sensor and store into the float (in celsius)

    twi_master_wr_rd(&sens2,0,2); //write 0 bits, read 2 bits
    while(sens2.status != TWIM_STATUS_READY); //do the same as above but with sens2
    tempsnsr_2_c = (sens2.rd_array[0] << 3) | sens2.rd_array[1]>>5;
    
    tempsnsr_1_c = tempsnsr_1_c * 0.125; //LM75A - do conversion by converting each int step as 0.125
    tempsnsr_2_c = tempsnsr_2_c * 0.125; //LM75B

    tempsnsr_1_f = ( tempsnsr_1_c * (1.8) ) + 32; // (9/5 = 1.8)
    tempsnsr_2_f = ( tempsnsr_2_c * (1.8) ) + 32; // convert to ferreinheight

    temp1_rounded = round_nearest_qtr(tempsnsr_1_f); // round the decimal points of both values to quarters
    temp2_rounded = round_nearest_qtr(tempsnsr_2_f);

    fake_temp1 = temp1_rounded * 100; // turn the float into an integer so we can convert it into a string (remove the decimal point)
    fake_temp2 = temp2_rounded * 100;
    
    sprintf(temp1_str, "%d", fake_temp1); // convert the int into a string (so it can be loaded into the glcd)
    sprintf(temp2_str, "%d", fake_temp2);

    insert_dot(temp1_str); // insert a decimal point back where its supposed to go (but as a string)
    insert_dot(temp2_str);

    string2frame_buffer_paged(temp1_str,1,0); //write to page 1  //WORKS
    string2frame_buffer_paged(temp2_str,3,0); //write to page 3  //WORKS

    string2frame_buffer_paged(display1,0,0); 
    string2frame_buffer_paged(display2,2,0); 


    write_frame_buffer();
    
    };         //freeze here to see display
}//main
