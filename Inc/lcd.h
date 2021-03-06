/*
 * lcd.h
 *
 *  Created on: 10/06/2018
 *      Author: Olivier Van den Eede
 */

#ifndef LCD_H_
#define LCD_H_

#include "stm32f0xx_hal.h"
#include "string.h"
#include "main.h"


/************************************** Command register **************************************/
#define CLEAR_DISPLAY 0x01

#define RETURN_HOME 0x02

#define ENTRY_MODE_SET 0x04
#define OPT_S	0x01					// Shift entire display to right
#define OPT_INC 0x02					// Cursor increment

#define DISPLAY_ON_OFF_CONTROL 0x08
#define OPT_D	0x04					// Turn on display
#define OPT_C	0x02					// Turn on cursor
#define OPT_B 	0x01					// Turn on cursor blink

#define CURSOR_DISPLAY_SHIFT 0x10		// Move and shift cursor
#define OPT_SC 0x08
#define OPT_RL 0x04

#define FUNCTION_SET 0x20
#define OPT_DL 0x10						// Set interface data length
#define OPT_N 0x08						// Set number of display lines
#define OPT_F 0x04						// Set alternate font

#define SET_DDRAM_ADDR 0x80				// Set DDRAM address


/************************************** Helper macros **************************************/
#define DELAY(X) HAL_Delay(X)


/************************************** LCD defines **************************************/
#define LCD_NIB 4
#define LCD_BYTE 8
#define LCD_DATA_REG    GPIO_PIN_SET
#define LCD_COMMAND_REG GPIO_PIN_RESET


/************************************** LCD typedefs **************************************/
#define LCD_PortType GPIO_TypeDef*
#define LCD_PinType uint16_t

typedef enum {
	LCD_4_BIT_MODE,
	LCD_8_BIT_MODE
} LCD_ModeTypeDef;


typedef struct {
	LCD_PortType * data_port;
	LCD_PinType * data_pin;

	LCD_PortType rs_port;
	LCD_PinType rs_pin;

	LCD_PortType en_port;
	LCD_PinType en_pin;

	LCD_ModeTypeDef mode;

} LCD_HandleTypeDef;

/************************************** Public functions **************************************/
void LCD_init(LCD_HandleTypeDef * lcd);
void LCD_blinkCursor(LCD_HandleTypeDef* lcd, uint8_t value);
void LCD_clear(LCD_HandleTypeDef* lcd);
void LCD_float(LCD_HandleTypeDef * lcd, float number, uint8_t precision);
void LCD_int(LCD_HandleTypeDef * lcd, int number);
void LCD_string(LCD_HandleTypeDef * lcd, char * string);
void LCD_cursor(LCD_HandleTypeDef * lcd, uint8_t row, uint8_t col);
void LCD_write_command(LCD_HandleTypeDef * lcd, uint8_t command);
LCD_HandleTypeDef LCD_create(
		LCD_PortType port[], LCD_PinType pin[],
		LCD_PortType rs_port, LCD_PinType rs_pin,
		LCD_PortType en_port, LCD_PinType en_pin, LCD_ModeTypeDef mode);



#endif /* LCD_H_ */
