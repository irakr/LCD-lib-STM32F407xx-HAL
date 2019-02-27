# LCD-lib-STM32F407xx-HAL
A mini library for driving a character LCD 16x2 using the STM32F407xx series MCUs or boards.<br><br>
<b>NOTE: This library is only compatible with STM32 HAL library. So please create your project using STM32CubeMX code generator or else things are gonna be tough.<b>
  
## Usage
* Put lcd.h into your Inc/ folder and lcd.c in Src/. If using Keil then better put lcd.c in MDK-ARM/ and also don't forget to add file to project.<br>
* Use the LCD_init() function to get an LCD_TypeDef pointer. This pointer will be used as a handle and further must be passed to all other user accessible functions. For example: <br>LCD_putchar(lcd, 'e'); // 'lcd' is of type LCD_TypeDef*
* LCD_init() parameters:
> LCD_TypeDef* LCD_init(PortPin_Map *RS, PortPin_Map *RW, PortPin_Map *E,<br>
                        PortPin_Map *D4, PortPin_Map *D5, PortPin_Map *D6, PortPin_Map *D7);

> PortPin_Map is a structure with two member; GPIO_TypeDef GPIOx, uint8_t GPIO_PIN_x.<br>
  Example: For specifying PortPin_Map for RS pin of the LCD, the appropriate GPIOx and GPIO_PIN_x must be set and then passed to the function.
* See Examples/Src/main.c to understand the proper usage of the library.