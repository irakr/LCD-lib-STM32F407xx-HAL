#include <stdlib.h>
//#include "stm32f4.h"
#include "stm32f4xx_hal.h"
#include "lcd.h"

/* A local copy of a pointer to an LCD_TypeDef instance passed from user app.
 * This copy is used by internal calls to low-level functions. The advantage
 * is that we dont have to pass around the pointer to the LCD_TypeDef within
 * the library call space. In other words, functions that are only accessible
 * and meaningful to the library itself(not the library user) uses this variable.
 */
static LCD_TypeDef* __lcd = NULL;

/*
 * ------- Low-level functions -----------
 */

/* Performs the enable pulse signal generation on ENABLE pin of LCD.  */
static void enable(void) {
  HAL_GPIO_WritePin(__lcd->E->GPIOx, __lcd->E->GPIO_Pin_x, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(__lcd->E->GPIOx, __lcd->E->GPIO_Pin_x, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(__lcd->E->GPIOx, __lcd->E->GPIO_Pin_x, GPIO_PIN_SET);
}

/* Returns GPIO_PIN_RESET if expr yields value>0.
 * XXX... This macro was designed so that the 3rd arg of
 * HAL_GPIO_WritePin() is passed an appropriate value type.
 */
#define SET_IF(expr)  ((expr) ? GPIO_PIN_SET : GPIO_PIN_RESET)

/* Write the lower nibble of @val to D4,D5,D6,D7  */
static void write4Bits(uint8_t val) {
  HAL_GPIO_WritePin(__lcd->D7->GPIOx, __lcd->D7->GPIO_Pin_x, SET_IF(val & 0x08)); // 4th bit of lower nibble
  HAL_GPIO_WritePin(__lcd->D6->GPIOx, __lcd->D6->GPIO_Pin_x, SET_IF(val & 0x04));
  HAL_GPIO_WritePin(__lcd->D5->GPIOx, __lcd->D5->GPIO_Pin_x, SET_IF(val & 0x02));
  HAL_GPIO_WritePin(__lcd->D4->GPIOx, __lcd->D4->GPIO_Pin_x, SET_IF(val & 0x01)); // 0th bit(LSb)
  enable();
}

// XXX... Need to be rectified.
#ifdef EIGHT_BIT_MODE
static void write8Bits(uint8_t val) {
  HAL_GPIO_WritePin(__lcd->D0->GPIOx, __lcd->D0->GPIO_Pin_x, SET_IF(val & 0x80)); // 8th bit(starting from 0-LSb)
  HAL_GPIO_WritePin(__lcd->D1->GPIOx, __lcd->D1->GPIO_Pin_x, SET_IF(val & 0x40));
  HAL_GPIO_WritePin(__lcd->D2->GPIOx, __lcd->D2->GPIO_Pin_x, SET_IF(val & 0x20));
  HAL_GPIO_WritePin(__lcd->D3->GPIOx, __lcd->D3->GPIO_Pin_x, SET_IF(val & 0x10));
  HAL_GPIO_WritePin(__lcd->D4->GPIOx, __lcd->D4->GPIO_Pin_x, SET_IF(val & 0x08)); // 4th bit
  HAL_GPIO_WritePin(__lcd->D5->GPIOx, __lcd->D5->GPIO_Pin_x, SET_IF(val & 0x04));
  HAL_GPIO_WritePin(__lcd->D6->GPIOx, __lcd->D6->GPIO_Pin_x, SET_IF(val & 0x02));
  HAL_GPIO_WritePin(__lcd->D7->GPIOx, __lcd->D7->GPIO_Pin_x, SET_IF(val & 0x01)); // LSb
  enable();
}
#endif

/* Sends a command byte to the LCD  */
void sendCommand(uint8_t cmd) {
  HAL_GPIO_WritePin(__lcd->RS->GPIOx, __lcd->RS->GPIO_Pin_x, GPIO_PIN_RESET); // Command mode
  HAL_GPIO_WritePin(__lcd->RW->GPIOx, __lcd->RW->GPIO_Pin_x, GPIO_PIN_RESET); // Write mode
  
  write4Bits(cmd >> 4);
  write4Bits(cmd);
  
}

/* Sends a data byte to the LCD  */
void sendData(uint8_t data) {
  HAL_GPIO_WritePin(__lcd->RS->GPIOx, __lcd->RS->GPIO_Pin_x, GPIO_PIN_SET); // Data mode
  HAL_GPIO_WritePin(__lcd->RW->GPIOx, __lcd->RW->GPIO_Pin_x, GPIO_PIN_RESET); // Write mode
  
  write4Bits(data >> 4);
  write4Bits(data);
}


/*
 * ------- User APIs -----------
 */

LCD_TypeDef* LCD_init(PortPin_Map *RS, PortPin_Map *RW, PortPin_Map *E,
#ifdef EIGHT_BIT_MODE
              PortPin_Map *D0, PortPin_Map *D1, PortPin_Map *D2, PortPin_Map *D3,
#endif
              PortPin_Map *D4, PortPin_Map *D5, PortPin_Map *D6, PortPin_Map *D7)
{
  LCD_TypeDef *ret;
  
  assert_param(RS != NULL);
  assert_param(RW != NULL);
  assert_param(E != NULL);

#ifdef EIGHT_BIT_MODE
  assert_param(D0 != NULL);
  assert_param(D1 != NULL);
  assert_param(D2 != NULL);
  assert_param(D3 != NULL);
#endif
  
  assert_param(D4 != NULL);
  assert_param(D5 != NULL);
  assert_param(D6 != NULL);
  assert_param(D7 != NULL);
  
  ret = calloc(1, sizeof(LCD_TypeDef));
  if(!ret)
    return NULL;
  
  ret->RS = RS;
  ret->RW = RW;
  ret->E = E;
  
#ifdef EIGHT_BIT_MODE
  ret->D0 = D0;
  ret->D1 = D1;
  ret->D2 = D2;
  ret->D3 = D3;
#endif
  
  ret->D4 = D4;
  ret->D5 = D5;
  ret->D6 = D6;
  ret->D7 = D7;
  
  ret->currLine = ret->currCol = 0;
  
  return ret;
}

void LCD_begin(LCD_TypeDef* lcd) {
  assert_param(lcd != NULL);
  __lcd = lcd; // NOTE: This line must exist for all user APIs.
  
  // Signals for initial setup commands...
  // Keep Enable pin LOW.
  HAL_GPIO_WritePin(lcd->E->GPIOx, lcd->E->GPIO_Pin_x, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(lcd->RS->GPIOx, lcd->RS->GPIO_Pin_x, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(lcd->RW->GPIOx, lcd->RW->GPIO_Pin_x, GPIO_PIN_RESET);
  HAL_Delay(10);
  
  // Function set config.
  sendCommand(LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS);
  HAL_Delay(5);
  
  // Dislpay on/off, cursor blink config.
  sendCommand(LCD_DISPLAYCONTROL | LCD_DISPLAYON | LCD_BLINKON | LCD_CURSORON);
  HAL_Delay(5);
  
  sendCommand(LCD_ENTRYMODESET | LCD_ENTRYRIGHT | LCD_ENTRYSHIFTDECREMENT);
  HAL_Delay(5);
  
  LCD_clearScreen(lcd);
  
  LCD_home(lcd);
  
  HAL_Delay(500);
}

void LCD_clearScreen(LCD_TypeDef* lcd) {
  assert_param(lcd != NULL);
  __lcd = lcd; // NOTE: This line must exist for all user APIs.
  sendCommand(LCD_CLEARDISPLAY);
  HAL_Delay(1);
}

void LCD_home(LCD_TypeDef* lcd) {
  assert_param(lcd != NULL);
  __lcd = lcd; // NOTE: This line must exist for all user APIs.
  sendCommand(LCD_RETURNHOME);
  HAL_Delay(1);
  lcd->currCol = lcd->currLine = 0;
}

HAL_StatusTypeDef LCD_setCursor(LCD_TypeDef* lcd, uint8_t line, uint8_t col) {
  assert_param(lcd != NULL);
  __lcd = lcd; // NOTE: This line must exist for all user APIs.
  
  // Making sure lines and columns are taken care of.
  uint8_t temp_col;
  col = col % 16;
  temp_col = col;
  if(line == 1)
    temp_col += 0x40;
  
  sendCommand(LCD_SETDDRAMADDR | temp_col);
  HAL_Delay(1);
  lcd->currLine = line;
  lcd->currCol = col;
  
  return HAL_OK;
}

void LCD_putchar(LCD_TypeDef* lcd, uint8_t c) {
  assert_param(lcd != NULL);
  __lcd = lcd; // NOTE: This line must exist for all user APIs.
  sendData(c);
  HAL_Delay(1);
  
  // Move cursor forward 1-step.
  if((lcd->currLine == 0) && (lcd->currCol == 15))
    LCD_setCursor(lcd, 1, 0);
  else if((lcd->currLine == 1) && (lcd->currCol == 15))
    LCD_setCursor(lcd, 0, 0);
  else
    LCD_setCursor(lcd, lcd->currLine, lcd->currCol + 1);
}

void LCD_putstr(LCD_TypeDef* lcd, uint8_t* s) {
  assert_param(lcd != NULL);
  assert_param(s != NULL);
  __lcd = lcd; // NOTE: This line must exist for all user APIs.
  
  for(; *s != 0; ++s) {
    LCD_putchar(lcd, *s);
  }
}
