#include <avr/io.h>
#include <stdlib.h>

// This file holds the definitions for: 
// lcd_frame_buffer[]  AND  ascii[]
// Declarations for these will be in buffers.h 

// Initial contents make a frame with corners
// This is easily overwritten. Done here only to test the x,y coordinate system
// The setup is for 128 column wide display, with 4 pages top to bottom, each containing
// 128 bytes vertically oriented. Bit zero is the one the first line of each page. 

uint8_t lcd_frame_buffer[512] = {
//page 0, 128 bytes
0xFF, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0xFF,
//page 1, 128 bytes
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//page 2, 128 bytes
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//page 3, 128 bytes
0xFF, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xFF,
};

//96, 5x7 format ascii characters but in RAM not ROM 
//TODO: move to program memory
//These values are offset by 0x20 for non-printing codes 0x00-0x1F.
//Total size is (96*8)i = 768 bytes
unsigned char font5x7[]={
        0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,	//" "=00
	0x00,0x00,0x00,0x4F,0x00,0x00,0x00,0x00,	//"!"=01
	0x00,0x00,0x07,0x00,0x07,0x00,0x00,0x00,	//"""=02  the two tick quote
	0x00,0x14,0x7F,0x14,0x7F,0x14,0x00,0x00,	//"#"=03
	0x00,0x24,0x2A,0x7F,0x2A,0x12,0x00,0x00,	//"$"=04
	0x00,0x23,0x13,0x08,0x64,0x62,0x00,0x00,	//"%"=05
	0x00,0x36,0x49,0x55,0x22,0x50,0x00,0x00,	//"&"=06
	0x00,0x00,0x05,0x03,0x00,0x00,0x00,0x00,	//"'"=07
	0x00,0x00,0x1C,0x22,0x41,0x00,0x00,0x00,	//"("=08
	0x00,0x00,0x41,0x22,0x1C,0x00,0x00,0x00,	//")"=09
	0x00,0x14,0x08,0x3E,0x08,0x14,0x00,0x00,	//"*"=0A
	0x00,0x08,0x08,0x3E,0x08,0x08,0x00,0x00,	//"+"=0B
	0x00,0x00,0x50,0x30,0x00,0x00,0x00,0x00,	//"//"=0C
	0x00,0x08,0x08,0x08,0x08,0x08,0x00,0x00,	//"-"=0D
	0x00,0x00,0x60,0x60,0x00,0x00,0x00,0x00,	//"."=0E
	0x00,0x20,0x10,0x08,0x04,0x02,0x00,0x00,	//"/"=0F
	0x00,0x3E,0x51,0x49,0x45,0x3E,0x00,0x00,	//"0"=10
	0x00,0x00,0x42,0x7F,0x40,0x00,0x00,0x00,	//"1"=11
	0x00,0x42,0x61,0x51,0x49,0x46,0x00,0x00,	//"2"=12
	0x00,0x21,0x41,0x45,0x4B,0x31,0x00,0x00,	//"3"=13
	0x00,0x18,0x14,0x12,0x7F,0x10,0x00,0x00,	//"4"=14
	0x00,0x27,0x45,0x45,0x45,0x39,0x00,0x00,	//"5"=15
	0x00,0x3C,0x4A,0x49,0x49,0x30,0x00,0x00,	//"6"=16
	0x00,0x01,0x01,0x79,0x05,0x03,0x00,0x00,	//"7"=17
	0x00,0x36,0x49,0x49,0x49,0x36,0x00,0x00,	//"8"=18
	0x00,0x06,0x49,0x49,0x29,0x1E,0x00,0x00,	//"9"=19
	0x00,0x00,0x36,0x36,0x00,0x00,0x00,0x00,	//":"=1A
	0x00,0x00,0x56,0x36,0x00,0x00,0x00,0x00,	//"//"=1B
	0x00,0x08,0x14,0x22,0x41,0x00,0x00,0x00,	//"<"=1C
	0x00,0x14,0x14,0x14,0x14,0x14,0x00,0x00,	//"="=1D
	0x00,0x00,0x14,0x22,0x14,0x08,0x00,0x00,	//">"=1E
	0x00,0x02,0x01,0x51,0x09,0x06,0x00,0x00,	//"?"=1F
	0x00,0x32,0x49,0x79,0x41,0x3E,0x00,0x00,	//"@"=20
	0x00,0x7E,0x11,0x11,0x11,0x7E,0x00,0x00,	//"A"=21
	0x00,0x41,0x7F,0x49,0x49,0x36,0x00,0x00,	//"B"=22
	0x00,0x3E,0x41,0x41,0x41,0x22,0x00,0x00,	//"C"=23
	0x00,0x41,0x7F,0x41,0x41,0x3E,0x00,0x00,	//"D"=24
	0x00,0x7F,0x49,0x49,0x49,0x49,0x00,0x00,	//"E"=25
	0x00,0x7F,0x09,0x09,0x09,0x01,0x00,0x00,	//"F"=26
	0x00,0x3E,0x41,0x41,0x49,0x7A,0x00,0x00,	//"G"=27
	0x00,0x7F,0x08,0x08,0x08,0x7F,0x00,0x00,	//"H"=28
	0x00,0x00,0x41,0x7F,0x41,0x00,0x00,0x00,	//"I"=29
	0x00,0x20,0x40,0x41,0x3F,0x01,0x00,0x00,	//"J"=2A
	0x00,0x7F,0x08,0x14,0x22,0x41,0x00,0x00,	//"K"=2B
	0x00,0x7F,0x40,0x40,0x40,0x40,0x00,0x00,	//"L"=2C
	0x00,0x7F,0x02,0x0C,0x02,0x7F,0x00,0x00,	//"M"=2D
	0x00,0x7F,0x06,0x08,0x30,0x7F,0x00,0x00,	//"N"=2E
	0x00,0x3E,0x41,0x41,0x41,0x3E,0x00,0x00,	//"O"=2F
	0x00,0x7F,0x09,0x09,0x09,0x06,0x00,0x00,	//"P"=30
	0x00,0x3E,0x41,0x51,0x21,0x5E,0x00,0x00,	//"Q"=31
	0x00,0x7F,0x09,0x19,0x29,0x46,0x00,0x00,	//"R"=32
	0x00,0x26,0x49,0x49,0x49,0x32,0x00,0x00,	//"S"=33
	0x00,0x01,0x01,0x7F,0x01,0x01,0x00,0x00,	//"T"=34
	0x00,0x3F,0x40,0x40,0x40,0x3F,0x00,0x00,	//"U"=35
	0x00,0x1F,0x20,0x40,0x20,0x1F,0x00,0x00,	//"V"=36
	0x00,0x7F,0x20,0x18,0x20,0x7F,0x00,0x00,	//"W"=37
	0x00,0x63,0x14,0x08,0x14,0x63,0x00,0x00,	//"x"=38
	0x00,0x07,0x08,0x70,0x08,0x07,0x00,0x00,	//"Y"=39
	0x00,0x61,0x51,0x49,0x45,0x43,0x00,0x00,	//"Z"=3A
	0x00,0x00,0x7F,0x41,0x41,0x00,0x00,0x00,	//"["=3B
	0x00,0x02,0x04,0x08,0x10,0x20,0x00,0x00,	//"\"=3C
	0x00,0x00,0x41,0x41,0x7F,0x00,0x00,0x00,	//"]"=3D
	0x00,0x04,0x02,0x01,0x02,0x04,0x00,0x00,	//"^"=3E
	0x00,0x40,0x40,0x40,0x40,0x40,0x00,0x00,	//"_"=3F
	0x00,0x01,0x02,0x04,0x00,0x00,0x00,0x00,	//"|"=40
	0x00,0x20,0x54,0x54,0x54,0x78,0x00,0x00,	//"a"=41
	0x00,0x7F,0x48,0x44,0x44,0x38,0x00,0x00,	//"b"=42
	0x00,0x38,0x44,0x44,0x44,0x28,0x00,0x00,	//"c"=43
	0x00,0x38,0x44,0x44,0x48,0x7F,0x00,0x00,	//"d"=44
	0x00,0x38,0x54,0x54,0x54,0x18,0x00,0x00,	//"e"=45
	0x00,0x00,0x08,0x7E,0x09,0x02,0x00,0x00,	//"f"=46
	0x00,0x0C,0x52,0x52,0x4C,0x3E,0x00,0x00,	//"g"=47
	0x00,0x7F,0x08,0x04,0x04,0x78,0x00,0x00,	//"" =48
	0x00,0x00,0x44,0x7D,0x40,0x00,0x00,0x00,	//"i"=49
	0x00,0x20,0x40,0x44,0x3D,0x00,0x00,0x00,	//"j"=4A
	0x00,0x00,0x7F,0x10,0x28,0x44,0x00,0x00,	//"k"=4B
	0x00,0x00,0x41,0x7F,0x40,0x00,0x00,0x00,	//"l"=4C
	0x00,0x7C,0x04,0x78,0x04,0x78,0x00,0x00,	//"m"=4D
	0x00,0x7C,0x08,0x04,0x04,0x78,0x00,0x00,	//"n"=4E
	0x00,0x38,0x44,0x44,0x44,0x38,0x00,0x00,	//"o"=4F
	0x00,0x7E,0x0C,0x12,0x12,0x0C,0x00,0x00,	//"p"=50
	0x00,0x0C,0x12,0x12,0x0C,0x7E,0x00,0x00,	//"q"=51
	0x00,0x7C,0x08,0x04,0x04,0x08,0x00,0x00,	//"r"=52
	0x00,0x58,0x54,0x54,0x54,0x64,0x00,0x00,	//"s"=53
	0x00,0x04,0x3F,0x44,0x40,0x20,0x00,0x00,	//"t"=54
	0x00,0x3C,0x40,0x40,0x3C,0x40,0x00,0x00,	//"u"=55
	0x00,0x1C,0x20,0x40,0x20,0x1C,0x00,0x00,	//"v"=56
	0x00,0x3C,0x40,0x30,0x40,0x3C,0x00,0x00,	//"w"=57
	0x00,0x44,0x28,0x10,0x28,0x44,0x00,0x00,	//"x"=58
	0x00,0x1C,0xA0,0xA0,0x90,0x7C,0x00,0x00,	//"y"=59
	0x00,0x44,0x64,0x54,0x4C,0x44,0x00,0x00,	//"z"=5A
	0x00,0x00,0x08,0x36,0x41,0x00,0x00,0x00,	//"{"=5B
	0x00,0x00,0x00,0x77,0x00,0x00,0x00,0x00,	//"|"=5C
	0x00,0x00,0x00,0x41,0x36,0x08,0x00,0x00,	//"}"=5D
	0x00,0x02,0x01,0x02,0x04,0x02,0x00,0x00,	//"~"=5E
	0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00,	//" "=5F
};
