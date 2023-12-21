#include <avr/io.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>

#include "buffers.h"

#define swap(a, b) {uint8_t t = a; a = b; b = t;} //used only for line drawing function
#define COLUMNS 128
#define ROWS     32
#define PAGES     4

// This file contains all the functions required to operate the 
// FocusLCD G132CLGSBSW6WNCCXAL graphic LCD display. It is based
// on the Sitronix ST7565R chip. This should also work with the
// Newhaven displays.

//these functions are for the xmega473 boards

///////////////////////////////////////////////////////////////////////////////
// Write a command to the LCD display
///////////////////////////////////////////////////////////////////////////////
void write_command(unsigned char cmd){
  PORTA.OUT &= ~(1<<PIN0_bp); //deassert LCD A0 pin low (indicates a command is coming)   -> rev 1.00 boards
  PORTC.OUT &= ~(1<<PIN4_bp); //assert LCD chip select low
  SPIC.DATA  = cmd;
  while(!(SPIC.STATUS & (1<<SPI_IF_bp))) ; //wait for SPI completion
  PORTC.OUT |=  (1<<PIN0_bp); //deassert LCD chip select back to high
}

///////////////////////////////////////////////////////////////////////////////
// Write data to the LCD display
///////////////////////////////////////////////////////////////////////////////
void write_data(unsigned char data){
  PORTA.OUT |=  (1<<PIN0_bp); //assert LCD A0 pin high (indicates a command is coming)   -> rev 1.00 boards
  PORTC.OUT &= ~(1<<PIN4_bp); //assert LCD chip select low
  SPIC.DATA = data;
  while(!(SPIC.STATUS & (1<<SPI_IF_bp))) ; //wait for SPI completion
  PORTC.OUT |= ~(1<<PIN0_bp); //assert LCD chip select low
}

///////////////////////////////////////////////////////////////////////////////
// Initalize the LCDFocus G132CLGSBSW6WNCCXAL graphic LCD display. 
// Initalization sequence came from FocusLCD engineering group
// V0=RR*[(99+EV)/162]*2.1 = (4.0*(99+18)/162*2.1)=6.06V
///////////////////////////////////////////////////////////////////////////////
void init_glcd(void) {
  //setup data direction registers
//  DDRF |= 0x01;  //PORTF bit zero:  output             (net: lcd_rst_n)
//  DDRC |= 0x80;  //PORTC bit seven: of PORTC is output (net: glcd_pwm)
//  DDRF |= 0x02;  //PORTF bit one: of PORTF is output (net: lcd_A0)
  PORTA.OUTSET =  (1<<PIN1_bp);  _delay_ms(1);   //deassert LCD reset pin 
  PORTA.OUTCLR =  (1<<PIN1_bp);  _delay_ms(2);   //assert LCD reset pin 
  PORTA.OUTSET =  (1<<PIN1_bp);  _delay_ms(12);  //deassert LCD reset pin 

//  PORTC |= 0x80;  //turn on LCD backlight full brillance.


//All the first revision 1.0 boards (the limited build of 85) use the LCDFocus LCDs.
//The big build of 350 all use the Newhaven display.
//#define FOCUS_LCD    //for FocusLCD initialization.  Comment out for Newhaven display
#ifdef  FOCUS_LCD 
//lcd initalization sequence for FocusLCD dislay
  write_command(0xE2); _delay_us(10); //software reset, its not really redundant
  write_command(0xA2); _delay_us(10); //set LCD bias to 1/9 
  write_command(0xA0); _delay_us(10); //Set ADC output direction to normal 
  write_command(0xC8); _delay_us(10); //set common output scan direction to reversed (vertically flipped)
                                      //SHL SELECT COM1-COM64(#0C0H #0C8) common
  write_command(0x22); _delay_us(10); //REGULATOR RESISTOR SELECT (R2 R1 R0) RR 0X20-0X27(3.0-6.5)
                                      //set internal R1/R2 resistor bias
  write_command(0x81); _delay_us(10); //ELECTRONIC VOLUME SET (LCD)
                                      //enter volume mode
  write_command(0x09); _delay_us(10); //SET EV(0X00-0X3F)
                                      //set volume mode to 9, seems best
  write_command(0x2F); _delay_us(10); //SET POWER CONTROL (VB VR VF)=111 
                                      //set set internal R1/R2 resistor to 111
#else				    
//Newhaven display lcd initalization sequence
write_command(0xA0); _delay_us(10); //identical to FocusLCD
write_command(0xAE); _delay_us(10);
write_command(0xC8); _delay_us(10); //identical to FocusLCD
write_command(0xA2); _delay_us(10);
write_command(0x2F); _delay_us(10);
write_command(0x21); _delay_us(10);
write_command(0x81); _delay_us(10);  //set contrast....
write_command(0x20); _delay_us(10);  //pretty sensitive, 0x12 is barely visible, 28 is too much
write_command(0xAF); _delay_us(10);

#endif
}

////////////////////////////////////////////////////////////////////////////////
// function: fill_display()
// Fill the lcd screen CGRAM directly with the same argument data
// Does not access lcd_frame_buffer, but instead fills the LCD memory directly
// This may be of limited utility and may be removed in the future
////////////////////////////////////////////////////////////////////////////////
void fill_display (uint8_t display_data) {
  uint8_t page, column;
  write_command(0xAF);           //turn display on
  for(page=0;page<=3;page++){
    write_command(0x40);         //set start line address to 0
    write_command(0xB0 | page);  //set current page to 0-3
    write_command(0x10);	 //set most  significant 4 bits of column address
    write_command(0x00);         //set least significant 4 bits of column address
    for(column=0;column<=127;column++){write_data(display_data);} //fill CGRAM 
  } //for each page 
}

////////////////////////////////////////////////////////////////////////////////
// fill_frame_buffer()
// Fill the entire lcd frame buffer with the the given argument data.
// For this data to be displayed, afterwards, the frame buffer must be written 
// to the LCD
////////////////////////////////////////////////////////////////////////////////
void fill_frame_buffer(uint8_t buffer_data){
	//TODO: use the memset function instead in avrlibc string functions
  uint8_t page, column;
  for(page=0;page<=3;page++){ //for each page
    for(column=0;column<=127;column++){
      lcd_frame_buffer[column]=buffer_data; //fill byte stripes, top is d0, 
    } //for each column
  } //for each page 
}

////////////////////////////////////////////////////////////////////////////////
// function: string2frame_buffer()
// Fill the lcd frame buffer with pixel data from the given string array
////////////////////////////////////////////////////////////////////////////////
// Wraps around, filling the display page by page until its full. The display
// holds 16 characters per page (4 pages) using a 5x7 font.
// The offset of 0x20 is to account for non-prining ASCII characters.
// The 5x7 font is not a very pertty with large spacing between the characters.
//
void string2frame_buffer(char *lcd_string_buffer){
  for(uint8_t i=0; i<=(strlen(lcd_string_buffer)-1); i++){ //for all char in string buffer
	  //TODO: the -1 is an error??, strlen does not count \n character in avr-libc
    for(uint8_t j=0; j<=7; j++){  //for all vertical stripes per character
      lcd_frame_buffer[(i*8) + j] = font5x7[((lcd_string_buffer[i]-0x20) * 8) + j];
    }
  }
}  

////////////////////////////////////////////////////////////////////////////////
//                            char2lcd_frame_buffer
// Send single character to graphic LCD 
// "page" selects page (0-3) to be written. Each page holds 16 characters
// "char_offset" indicates where (0-15) to begin writing the characters
// usage: char2lcd_frame_buffer_paged('H',2,4); //send ASCII H to 3rd line, 5th character position 
void char2lcd_frame_buffer_paged(char character, uint8_t page, uint8_t char_offset){
    for(uint8_t j=0; j<=7; j++){  //for all vertical stripes in the character
      lcd_frame_buffer[j + (page*128) + (char_offset*8)] = font5x7[(((uint8_t) character - 0x20) * 8) + j];
    }
}

////////////////////////////////////////////////////////////////////////////////
// Send string to the graphics LCD 
// "page" selects page (0-3) to be written.  Each page holds 16 characters
// "char_offset" indicates where (0-15) to begin writing the characters
// Compiler puts '\0' at end of buffer, so only write out (strlen-1) characters.
// Decoder ring to this code:
//   What byte to write in the frame buffer.......
//     *move forward by 8 bytes for each character :                  (i*8)  +                
//     *each char is composed of 8 stripes each a byte in size:           j  +
//     *each page consists of 128 vertical stripes of one byte: (page * 128) +
//     *each character is 8 bytes:                           (char_offset*8) 
//
//   What byte to read in the font array.......
//     *for each char in the sting, substract 0x20 since the font array does 
//      not contain the first 0x20 non-printing characters:  lcd_string_buffer[i]-0x20) * 
//     *multiply by 8 to get the right vertical stripe:                              8  +
//     *do for all 8 stripes in the character:                                        j
//
void string2frame_buffer_paged(char *lcd_string_buffer, uint8_t page, uint8_t char_offset){
  for(uint8_t i=0; i<=(strlen(lcd_string_buffer)-1); i++){ //for all char in string buffer except '\0'
	  //TODO: another strlen-1 bug??  SEE: strnlen function
    for(uint8_t j=0; j<=7; j++){  //for all vertical stripes per character
      lcd_frame_buffer[(i*8) + j + (page * 128) + (char_offset*8) ] = font5x7[((lcd_string_buffer[i]-0x20) * 8) + j];
    }
  }
}  

////////////////////////////////////////////////////////////////////////////////
// Write the entire contents of the lcd_frame_buffer to the ST7565R memory.
// This will take some time so more efficient ways should maybe be devised.
// TODO: Do by pages or regions to save time, also implement interrupt driven
// 512bytes * 250nS * 8bits = 1.024mS not counting a few cycles of overhead.
////////////////////////////////////////////////////////////////////////////////
void write_frame_buffer(void){
  uint8_t page, column;
  write_command(0xAF);           //turn display on  TODO: do elsewhere in init?
  for(page=0;page<=3;page++){
    write_command(0x40);         //set start line address to 0
    write_command(0xB0 | page);  //set current page to 0-3
    write_command(0x10);	 //set most  significant 4 bits of column address to zero
    write_command(0x00);         //set least significant 4 bits of column address to zero
    for(column=0;column<=127;column++){
      write_data(lcd_frame_buffer[column + (page*128)]);} //ST7565R memory from frame buffer
  } //for each page 
}

///////////////////////////////////////////////////////////////////////////////
// Set a single pixel given the X and Y position on the display. This does not
// map directly to the frame buffer array though so some calculations are
// done to determine the index to that array. The argument "color" determines
// if the pixel is on or off. Color: '1'=pixel on, '0'=pixel off.
///////////////////////////////////////////////////////////////////////////////
void setpixel(uint8_t *buffer, uint8_t x, uint8_t y, uint8_t color) {
  if ((x >= COLUMNS) || (y >= ROWS)){return;} //check for invalid X,Y
  // x is the column, y the row that increases going downward
  // (y/8) gives you the page the pixel is in
  // d0 is the top pixel, d7 is the bottom pixel
  if (color)
    buffer[x+((y/8)*128)] |=  (_BV(y%8)); //set pixel
  else
    buffer[x+((y/8)*128)] &= ~(_BV(y%8)); //clear pixel
}


///////////////////////////////////////////////////////////////////////////////
// Bresenham's line algorithm from Wikpedia
// https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
///////////////////////////////////////////////////////////////////////////////
#define swap(a, b) {uint8_t t = a; a = b; b = t;} //used only for this function
void drawline(uint8_t *buff, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t color){
  uint8_t dx, dy, steep;
  int8_t err, ystep;

  steep = abs(y1-y0) > abs(x1-x0);
  if (steep){   swap(x0, y0); swap(x1, y1);}
  if (x0 > x1){ swap(x0, x1); swap(y0, y1);}
  dx = x1 - x0;
  dy = abs(y1-y0);
  err = dx/2;
  if (y0 < y1){ ystep =  1;} 
  else {        ystep = -1;}
  for(; x0<x1; x0++){
    if (steep){setpixel(buff, y0, x0, color); } 
    else      {setpixel(buff, x0, y0, color); }
    err -= dy;
    if (err < 0){ y0 += ystep; err += dx;}
 }
}

///////////////////////////////////////////////////////////////////////////////
// Circle drawing function from Limor Fried at Adafruit...thanks LadyAda!
///////////////////////////////////////////////////////////////////////////////
void drawcircle(uint8_t *buff, uint8_t x0, uint8_t y0, uint8_t r, uint8_t color){
  int8_t f = 1 - r;
  int8_t ddF_x = 1;
  int8_t ddF_y = -2 * r;
  int8_t x = 0;
  int8_t y = r;

  setpixel(buff, x0, y0+r, color);
  setpixel(buff, x0, y0-r, color);
  setpixel(buff, x0+r, y0, color);
  setpixel(buff, x0-r, y0, color);

  while (x<y) {
    if (f >= 0){ y--; ddF_y += 2; f += ddF_y;}
    x++;
    ddF_x += 2;
    f += ddF_x;
    setpixel(buff, x0 + x, y0 + y, color);
    setpixel(buff, x0 - x, y0 + y, color);
    setpixel(buff, x0 + x, y0 - y, color);
    setpixel(buff, x0 - x, y0 - y, color);
    setpixel(buff, x0 + y, y0 + x, color);
    setpixel(buff, x0 - y, y0 + x, color);
    setpixel(buff, x0 + y, y0 - x, color);
    setpixel(buff, x0 - y, y0 - x, color);
  }
}

///////////////////////////////////////////////////////////////////////////////
//Rectangle drawing function from Limor Fried @ Adafruit...thanks LadyAda!
///////////////////////////////////////////////////////////////////////////////
void drawrect(uint8_t *buff, uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color){
  for (uint8_t i=x; i<x+w; i++) {
    setpixel(buff, i, y, color);
    setpixel(buff, i, y+h-1, color);
  }
  for (uint8_t i=y; i<y+h; i++) {
    setpixel(buff, x, i, color);
    setpixel(buff, x+w-1, i, color);
  }
}
