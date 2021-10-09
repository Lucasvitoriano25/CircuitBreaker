/*
 * lcd.c
 *
 *  Created on: 10/06/2018
 *      Author: Olivier Van den Eede
 */

#include "lcd.h"


/************************************** Static declarations **************************************/

static void LCD_write_data(LCD_HandleTypeDef * lcd, uint8_t data);
static void LCD_write_command(LCD_HandleTypeDef * lcd, uint8_t command);
static void LCD_write(LCD_HandleTypeDef * lcd, uint8_t data, uint8_t len);


/************************************** Function definitions **************************************/

/**
 * Create new Lcd_HandleTypeDef and initialize the Lcd
 */
LCD_HandleTypeDef LCD_create(
		LCD_PortType port[], LCD_PinType pin[],
		LCD_PortType rs_port, LCD_PinType rs_pin,
		LCD_PortType en_port, LCD_PinType en_pin, LCD_ModeTypeDef mode)
{
	LCD_HandleTypeDef lcd;

	lcd.mode = mode;

	lcd.en_pin = en_pin;
	lcd.en_port = en_port;

	lcd.rs_pin = rs_pin;
	lcd.rs_port = rs_port;

	lcd.data_pin = pin;
	lcd.data_port = port;

	LCD_init(&lcd);

	return lcd;
}

/**
 * Initialize 16x2-lcd without cursor
 */
void LCD_init(LCD_HandleTypeDef * lcd)
{
	if(lcd->mode == LCD_4_BIT_MODE)
	{
			LCD_write_command(lcd, 0x33);
			LCD_write_command(lcd, 0x32);
			LCD_write_command(lcd, FUNCTION_SET | OPT_N);		// 4-bit mode
	}
	else
		LCD_write_command(lcd, FUNCTION_SET | OPT_DL | OPT_N);


	LCD_write_command(lcd, CLEAR_DISPLAY);					// Clear screen
	LCD_write_command(lcd, DISPLAY_ON_OFF_CONTROL | OPT_D);	                // Lcd-on, cursor-off, no-blink
	LCD_write_command(lcd, ENTRY_MODE_SET | OPT_INC);			// Increment cursor
}

/**
 * Clear display screen
 */
void LCD_blinkCursor(LCD_HandleTypeDef* lcd, uint8_t value) {
  if (value) {
    LCD_write_command(lcd, DISPLAY_ON_OFF_CONTROL | OPT_D | OPT_C | OPT_B);	// Lcd-on, cursor-on, blink-on
  } else {
    LCD_write_command(lcd, DISPLAY_ON_OFF_CONTROL | OPT_D);		        // Lcd-on, cursor-off, no-blink
  }
}

/**
 * Clear display screen
 */
void LCD_clear(LCD_HandleTypeDef* lcd) {
  LCD_write_command(lcd, CLEAR_DISPLAY);
}

/**
 * Write a float number on the current position
 */
void LCD_float(LCD_HandleTypeDef * lcd, float number, uint8_t precision) {
  int int_part;
  float remainder;
  
  int_part = (int) number;
  
  if (int_part < 100) {
    LCD_string(lcd, " ");
  }
  if (int_part < 10) {
    LCD_string(lcd, " ");
  }
  LCD_int(lcd, int_part);
  
  LCD_string(lcd, ".");
  
  remainder = number - int_part;
  for (uint8_t i = 0; i < precision; i++) {
    remainder *= 10;
    LCD_int(lcd, (int) remainder);
    remainder -= (int) remainder;
  }
}

/**
 * Write a number on the current position
 */
void LCD_int(LCD_HandleTypeDef * lcd, int number)
{
	char buffer[11];
	sprintf(buffer, "%d", number);

	LCD_string(lcd, buffer);
}

/**
 * Write a string on the current position
 */
void LCD_string(LCD_HandleTypeDef * lcd, char * string)
{
	for(uint8_t i = 0; i < strlen(string); i++)
	{
		LCD_write_data(lcd, string[i]);
	}
}

/**
 * Set the cursor position
 */
void LCD_cursor(LCD_HandleTypeDef * lcd, uint8_t row, uint8_t col)
{
  uint8_t row_offsets[4] = {0x00, 0x40, 0x00 + 16, 0x40 + 16};
  LCD_write_command(lcd, SET_DDRAM_ADDR | (row_offsets[row] + col));
}


/************************************** Static function definition **************************************/

/**
 * Write a byte to the command register
 */
void LCD_write_command(LCD_HandleTypeDef * lcd, uint8_t command)
{
	HAL_GPIO_WritePin(lcd->rs_port, lcd->rs_pin, LCD_COMMAND_REG);		// Write to command register

	if(lcd->mode == LCD_4_BIT_MODE)
	{
		LCD_write(lcd, (command >> 4), LCD_NIB);
		LCD_write(lcd, command & 0x0F, LCD_NIB);
	}
	else
	{
		LCD_write(lcd, command, LCD_BYTE);
	}

}

/**
 * Write a byte to the data register
 */
void LCD_write_data(LCD_HandleTypeDef * lcd, uint8_t data)
{
	HAL_GPIO_WritePin(lcd->rs_port, lcd->rs_pin, LCD_DATA_REG);			// Write to data register

	if(lcd->mode == LCD_4_BIT_MODE)
	{
		LCD_write(lcd, data >> 4, LCD_NIB);
		LCD_write(lcd, data & 0x0F, LCD_NIB);
	}
	else
	{
		LCD_write(lcd, data, LCD_BYTE);
	}

}

/**
 * Set len bits on the bus and toggle the enable line
 */
void LCD_write(LCD_HandleTypeDef * lcd, uint8_t data, uint8_t len)
{
	for(uint8_t i = 0; i < len; i++)
	{
		HAL_GPIO_WritePin(lcd->data_port[i], lcd->data_pin[i], (data >> i) & 0x01);
	}

	HAL_GPIO_WritePin(lcd->en_port, lcd->en_pin, GPIO_PIN_SET);
	DELAY(2);
	HAL_GPIO_WritePin(lcd->en_port, lcd->en_pin, GPIO_PIN_RESET); 		// Data receive on falling edge
}
