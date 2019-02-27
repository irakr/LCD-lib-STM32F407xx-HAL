#ifndef LCD_H
#define LCD_H

#include <stdint.h>
#include "stm32f4xx.h"

/* Uncomment to enable 8-bit mode. */
//#define EIGHT_BIT_MODE

/* Command bit masks  */
#define LCD_CLEARDISPLAY    0x01
#define LCD_RETURNHOME      0x02
#define LCD_ENTRYMODESET    0x04
#define LCD_DISPLAYCONTROL  0x08
#define LCD_CURSORSHIFT     0x10
#define LCD_FUNCTIONSET     0x20
#define LCD_SETCGRAMADDR    0x40
#define LCD_SETDDRAMADDR    0x80

/* FunctionSet bits   */
#define LCD_8BITMODE    0x10
#define LCD_4BITMODE    0x00
#define LCD_2LINE       0x08
#define LCD_1LINE       0x00
#define LCD_5x10DOTS    0x04
#define LCD_5x8DOTS     0x00

/* Display control bits */
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

/* Entry mode bits  */
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

/* Just keeping a combo of the port and pin together in one piece. */
typedef struct _PortPin_Map {
  GPIO_TypeDef *GPIOx;
  uint32_t GPIO_Pin_x;
} PortPin_Map;

/* Structure for a standard (character)LCD display.   */
typedef struct _LCD_TypeDef {
  
  PortPin_Map *RS; // Selects command(0) or data(1) register.
  PortPin_Map *RW; // Selects read(1) or write(0) operation.
  PortPin_Map *E;  // Enable LCD operation. (HIGH to LOW / Falling edge trigger)

  // Data pins.
#ifdef EIGHT_BIT_MODE
  PortPin_Map *D0, *D1, *D2, *D3;
#endif
  
  PortPin_Map *D4, *D5, *D6, *D7;
  
  uint8_t currLine, currCol;
  
} LCD_TypeDef;

/* Initialize LCD parameters and returns LCD_TypeDef.   */
LCD_TypeDef* LCD_init(PortPin_Map *RS, PortPin_Map *RW, PortPin_Map *E,
#ifdef EIGHT_BIT_MODE
              PortPin_Map *D0, PortPin_Map *D1, PortPin_Map *D2, PortPin_Map *D3,
#endif
              PortPin_Map *D4, PortPin_Map *D5, PortPin_Map *D6, PortPin_Map *D7);

/*  Set initial default parameters for the LCD. */
void LCD_begin(LCD_TypeDef* lcd);

/*  Clear the contents of the screen. */
void LCD_clearScreen(LCD_TypeDef* lcd);

/*  Bring the cursor to the top left position.  */
void LCD_home(LCD_TypeDef* lcd);

/*  Move the cursor to a specific position.
 *  If @col exceeds 0x4f, i.e., the last column of the 2nd line, then
 *  it is reset to 0x00, i.e., the 1st column of the 1st line.
 */
HAL_StatusTypeDef LCD_setCursor(LCD_TypeDef* lcd, uint8_t line, uint8_t col);

/*  Put a character on the cursor.  */
void LCD_putchar(LCD_TypeDef* lcd, uint8_t c);

/*  Put a string of chars starting from the current cursor position.  */
void LCD_putstr(LCD_TypeDef* lcd, uint8_t* s);

/* Sends a command byte to the LCD  */
void sendCommand(uint8_t cmd);

/* Sends a data byte to the LCD  */
void sendData(uint8_t data);

#endif