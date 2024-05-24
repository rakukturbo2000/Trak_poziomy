
/** Put this in the src folder **/

#include "stm32f1xx.h"

#include "lcd.h"
#include "bit_banding.h"
#include "delay.h"
#define SLAVE_ADDRESS_LCD 0x4E // change this according to ur setup

#define LCD_E_PIN	1<<3
#define LCD_E_PORT	GPIOA

#define LCD_RS_PIN	1<<2
#define LCD_RS_PORT	GPIOA

#define LCD_D4_PIN	1<<4
#define LCD_D4_PORT	GPIOA

#define LCD_D5_PIN	1<<5
#define LCD_D5_PORT	GPIOA

#define LCD_D6_PIN	1<<6
#define LCD_D6_PORT	GPIOA

#define LCD_D7_PIN	1<<7
#define LCD_D7_PORT	GPIOA



//Duży wyświetlacz-samorobka
//#define LCD_E_PIN	1<<6
//#define LCD_E_PORT	GPIOA
//
//#define LCD_RS_PIN	1<<7
//#define LCD_RS_PORT	GPIOA
//
//#define LCD_D4_PIN	1<<5
//#define LCD_D4_PORT	GPIOA
//
//#define LCD_D5_PIN	1<<4
//#define LCD_D5_PORT	GPIOA
//
//#define LCD_D6_PIN	1<<3
//#define LCD_D6_PORT	GPIOA
//
//#define LCD_D7_PIN	1<<2
//#define LCD_D7_PORT	GPIOA
#define LCD_E_HIGH 		LCD_E_PORT->BSRR=LCD_E_PIN
#define LCD_E_LOW 		LCD_E_PORT->BRR=LCD_E_PIN
#define LCD_RS_LOW 		LCD_RS_PORT->BRR=LCD_RS_PIN
#define LCD_RS_HIGH 	LCD_RS_PORT->BSRR=LCD_RS_PIN



void lcd_sendHalf(uint8_t data)
{
	LCD_E_HIGH;
	BB(LCD_D4_PORT->ODR, LCD_D4_PIN)= 0x01==(data & 0x01);
	BB(LCD_D5_PORT->ODR, LCD_D5_PIN)= 0x02==(data & 0x02);
	BB(LCD_D6_PORT->ODR, LCD_D6_PIN)= 0x04==(data & 0x04);
	BB(LCD_D7_PORT->ODR, LCD_D7_PIN)= 0x08==(data & 0x08);
	LCD_E_LOW;
}

void lcd_write_byte(uint8_t data)
{
	lcd_sendHalf(data >> 4);
	lcd_sendHalf(data);

	Delay(1);
}
void lcd_send_cmd (char cmd)
{
	LCD_RS_LOW;
	lcd_write_byte(cmd);
}

void lcd_send_data (char data)
{
	LCD_RS_HIGH;
		lcd_write_byte(data);
}

void lcd_clear (void)
{
	lcd_send_cmd (0x80);
//	for (int i=0; i<70; i++)
//	{
//		lcd_send_data ('');
//	}
}

void lcd_put_cur(int row, int col)
{
    switch (row)
    {
        case 0:
            col |= 0x80;
            break;
        case 1:
            col |= 0xC0;
            break;
    }

    lcd_send_cmd (col);
}


void lcd_init (void)
{
	// 4 bit initialisation
	lcd_send_cmd (0x30);
	Delay(5);  // wait for >4.1ms
	lcd_send_cmd (0x30);
	Delay(1);  // wait for >100us
	lcd_send_cmd (0x30);
	Delay(10);
	lcd_send_cmd (0x20);  // 4bit mode
	Delay(10);

  // dislay initialisation
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	Delay(1);
	lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	Delay(1);
	lcd_send_cmd (0x01);  // clear display
	Delay(1);
	Delay(1);
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	Delay(1);
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}
