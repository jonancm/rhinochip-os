#include "lcd.h"
#include "delay.h"

#include <p30fxxxx.h>

#define LCDLINE1CMD     0x80    //Command to set LCD display to Line 1
#define LCDLINE2CMD     0xC0    //Command to set LCD display to Line 2

//SPI_Init Function:
//SPI1 module set up to communicate with the LCD controller on-board.
//Note that when SPI1 is enabled on this device, the UART1 pins will not be
//available due to peripheral multiplexing. So this project utilizes
//alternate UART1 pins, U1ATX and U1ARX
void lcd_setup(void)
{
	//Delay5ms(100);          //Provide 500ms delay for the LCD to start-up.
	Delay5ms(500);
	
	SPI1STAT = 0x0000;
	SPI1CON = 0x0020 ;      //Set the SPI1 module to 8-bit Master mode
	IFS0bits.SPI1IF = 0;    //Clear the SPI1 Interrupt Flag
	IEC0bits.SPI1IE = 0;    //SPI1 ISR processing is not enabled.
                            //SPI1 will be used in polling-mode
	SPI1STATbits.SPIEN = 1; //Turn on the SPI1 module
}

void lcd_write(const char * const buf)
{
	int i, j, temp;
	
	temp = SPI1BUF;
	SPI1STATbits.SPIROV = 0;
	IFS0bits.SPI1IF = 0;
	Delay5us(50);
	SPI1BUF = LCDLINE1CMD; // Set cursor on line 1 of the LCD
	
	for (i = 0; i < 16 && buf[i] != '\0'; ++i)
	{
		while (!IFS0bits.SPI1IF);
		temp = SPI1BUF;
		IFS0bits.SPI1IF = 0;
		SPI1STATbits.SPIROV = 0;
		Delay5us(50);
		SPI1BUF = buf[i];
	}
	
	if (buf[i] != '\0')
	{
		while (!IFS0bits.SPI1IF);
		temp = SPI1BUF;
		IFS0bits.SPI1IF = 0;
		SPI1STATbits.SPIROV = 0;
		Delay5us(50);
		SPI1BUF = LCDLINE2CMD;
		
		for (j = 0; j < 16 && buf[i] != '\0'; ++i, ++j)
		{
			while (!IFS0bits.SPI1IF);
			temp = SPI1BUF;
			IFS0bits.SPI1IF = 0;
			SPI1STATbits.SPIROV = 0;
			Delay5us(50);
			SPI1BUF = buf[i];
		}
	}
	
	// If the LCD is not full, fill the rest of the positions with white spaces
	// in order to make the cursor disappear. Otherwise, the cursor keeps blinking
	for (; i < 32; ++i)
	{
		while (!IFS0bits.SPI1IF);
		temp = SPI1BUF;
		IFS0bits.SPI1IF = 0;
		SPI1STATbits.SPIROV = 0;
		Delay5us(50);
		SPI1BUF = ' ';
	}
}

void lcd_clear(void)
{
	// Write 16 white spaces 2 times
	// (i.e. for each line of the LCD)
	lcd_write("                "
	          "                ");
}
