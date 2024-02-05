

#pragma once
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>





/// Font data stored PER GLYPH
typedef struct {
	uint16_t bitmapOffset; ///< Pointer into GFXfont->bitmap
	uint8_t width; ///< Bitmap dimensions in pixels
	uint8_t height; ///< Bitmap dimensions in pixels
	uint8_t xAdvance; ///< Distance to advance cursor (x axis)
	int8_t xOffset; ///< X dist from cursor pos to UL corner
	int8_t yOffset; ///< Y dist from cursor pos to UL corner
} GFXglyph;

/// Data stored for FONT AS A WHOLE
typedef struct {
	uint8_t *bitmap; ///< Glyph bitmaps, concatenated
	GFXglyph *glyph; ///< Glyph array
	uint16_t first; ///< ASCII extents (first char)
	uint16_t last; ///< ASCII extents (last char)
	uint8_t yAdvance; ///< Newline distance (y axis)
} GFXfont;



void LCD_clear();
void LCD_Pixel(uint16_t x1, uint16_t y1, uint16_t colour);
void LCD_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t c);
void LCD_Circle(int16_t Xc, int16_t Yc, uint16_t R, uint16_t c); 
void LCD_FilledCircle(int16_t x0, int16_t y0, int16_t r, uint8_t c);
void LCD_update();
void LCD_Text(uint16_t x0, uint16_t y0, char * text, uint16_t textColour, uint16_t backColour);
void LCD_clearBuf(); 
void LCD_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t c); 
void LCD_drawChar(int16_t x, int16_t y, char c, uint16_t color);
bool LCD_Point(uint16_t x1, uint16_t y1); 

size_t LCD_write(uint8_t c);
void LCD_setCursor(int16_t x, int16_t y);
void LCD_setCursorY(int16_t y);
uint16_t LCD_getCursorX();
uint16_t LCD_getCursorY();
void LCD_setFont(const GFXfont *f);
	



