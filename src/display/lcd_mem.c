



#include "lcd_mem.h"
#include "main.h"
#include "utils.h"



uint8_t DispGate[2];

extern SPI_HandleTypeDef hspi2;

GFXfont *gfxFont;


#define DISP_CS(x) HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, x)


//#define scr_height 336//240
//#define scr_width 536//400
//#define scr_buf scr_height * scr_width / 8


uint8_t __attribute__((section(".ospi_data"))) DispBuf[scr_buf];
bool NeedUpdate[scr_width];

int16_t cursor_x = 0; //< x location to start print()ing text
int16_t cursor_y = 0; //< y location to start print()ing text
uint8_t textsize_x = 1; //< Desired magnification in X-axis of text to print()
uint8_t textsize_y = 1; //< Desired magnification in Y-axis of text to print()
uint8_t rotation; ///< Display rotation (0 thru 3)
bool wrap =  true; ///< If set, 'wrap' text at right edge of display

uint8_t reverse(uint8_t b) {
	b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
	b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
	b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
	return b;
}

void LCD_clear()
{
	
	//clear	
	DispGate[0] = 0x20;
	DispGate[1] = 0x00;
	DISP_CS(1);
	HAL_SPI_Transmit(&hspi2, DispGate, 2, 10);
	DISP_CS(0);

	memset(DispBuf, 0xFF, scr_buf);
	for (uint16_t i = 0; i < scr_width; i++)
	{
		NeedUpdate[i] = true;
	}
}

void LCD_clearBuf()
{

	memset(DispBuf, 0xFF, scr_buf);
	for (uint16_t i = 0; i < scr_width; i++)
	{
		NeedUpdate[i] = true;
	}
}

void LCD_update()
{
	static uint16_t BufP;
	static uint16_t s;

	for (s = 1; s < scr_width + 1; s++)
	{

		if (NeedUpdate[s - 1])
		{
			BufP = (s - 1) * 42;
			DispGate[0] = 0xC0 | (reverse(s & 0x03) >> 6);
			DispGate[1] = reverse(s >> 2);
		
			DISP_CS(1);
			HAL_SPI_Transmit(&hspi2, DispGate, 2, 10);
			HAL_SPI_Transmit(&hspi2, DispBuf + BufP, 44, 100);
			DISP_CS(0);
			NeedUpdate[s - 1] = false;
		}
	}	
	
}



bool LCD_Point(uint16_t x1, uint16_t y1)
{
	if (x1 > scr_width - 1) x1 = scr_width - 1;
	if (y1 > scr_height - 1) y1 = scr_height - 1;
	
	y1 = scr_height - y1 - 1;
	
	uint32_t z1 = (uint32_t)x1 * 42 + (y1 >> 3);
	
	if (BitIsSet(DispBuf[z1], (7 - (y1 % 8)))) return true;
	else return false;
}


void LCD_Pixel(uint16_t x1, uint16_t y1, uint16_t colour)
{
	if (x1 > scr_width - 1) x1 = scr_width - 1;
	if (y1 > scr_height- 1) y1 = scr_height - 1;
	
	NeedUpdate[x1] = true;
	
	y1 = scr_height - y1 - 1;
	
	uint32_t z1 = (uint32_t)x1 * 42 + (y1 >> 3);
	
	// Basic colours
	if (colour == 1) //black
	{
		// physical black 00
		ClearBit(DispBuf[z1], (7 - (y1 % 8)));
	}
	else //white
	{
		// physical white 1
		SetBit(DispBuf[z1], (7 - (y1 % 8)));
	}
}

/***** Line *****/
void LCD_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t c)
{
	int16_t dx, dy, sx, sy, err, e2, i, tmp;

	dx = (x0 < x1) ? (x1 - x0) : (x0 - x1);
	dy = (y0 < y1) ? (y1 - y0) : (y0 - y1);
	sx = (x0 < x1) ? 1 : -1;
	sy = (y0 < y1) ? 1 : -1;
	err = ((dx > dy) ? dx : -dy) / 2;

	if (dx == 0) {
		if (y1 < y0) {
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}

		if (x1 < x0) {
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}

		/* Vertical line */
		for (i = y0; i <= y1; i++) {
			LCD_Pixel(x0, i, c);
		}

		/* Return from function */
		return;
	}

	if (dy == 0) {
		if (y1 < y0) {
			tmp = y1;
			y1 = y0;
			y0 = tmp;
		}

		if (x1 < x0) {
			tmp = x1;
			x1 = x0;
			x0 = tmp;
		}

		/* Horizontal line */
		for (i = x0; i <= x1; i++) {
			LCD_Pixel(i, y0, c);
		}

		/* Return from function */
		return;
	}

	while (1) {
		LCD_Pixel(x0, y0, c);
		if (x0 == x1 && y0 == y1) {
			break;
		}
		e2 = err;
		if (e2 > -dx) {
			err -= dy;
			x0 += sx;
		}
		if (e2 < dy) {
			err += dx;
			y0 += sy;
		}
	}	
}

// Draw circle
// input:
//   Xc, Yc - coordinates of the center of the circle
//   R - circle radius
void LCD_Circle(int16_t Xc, int16_t Yc, uint16_t R, uint16_t c)
{
	int16_t err = 1 - R;
	int16_t dx  = 1;
	int16_t dy  = -2 * R;
	int16_t x   = 0;
	int16_t y   = R;

	register int16_t sh = scr_height;
	register int16_t sw = scr_width;
	register int16_t tt;

	// Vertical and horizontal points
	if (Xc + R < sw) LCD_Pixel(Xc + R, Yc, c);
	if (Xc - R > -1) LCD_Pixel(Xc - R, Yc, c);
	if (Yc + R < sh) LCD_Pixel(Xc, Yc + R, c);
	if (Yc - R > -1) LCD_Pixel(Xc, Yc - R, c);

	while (x < y) {
		if (err >= 0) {
			dy  += 2;
			err += dy;
			y--;
		}
		dx  += 2;
		err += dx + 1;
		x++;

		// Draw pixels of eight octants
		tt = Xc + x;
		if (tt < sw) {
			if (Yc + y < sh) LCD_Pixel(tt, Yc + y, c);
			if (Yc - y > -1) LCD_Pixel(tt, Yc - y, c);
		}
		tt = Xc - x;
		if (tt > -1) {
			if (Yc + y < sh) LCD_Pixel(tt, Yc + y, c);
			if (Yc - y > -1) LCD_Pixel(tt, Yc - y, c);
		}
		tt = Xc + y;
		if (tt < sw) {
			if (Yc + x < sh) LCD_Pixel(tt, Yc + x, c);
			if (Yc - x > -1) LCD_Pixel(tt, Yc - x, c);
		}
		tt = Xc - y;
		if (tt > -1) {
			if (Yc + x < sh) LCD_Pixel(tt, Yc + x, c);
			if (Yc - x > -1) LCD_Pixel(tt, Yc - x, c);
		}
	}
}

void LCD_FilledCircle(int16_t x0, int16_t y0, int16_t r, uint8_t c) 
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	LCD_Pixel(x0, y0 + r, c);
	LCD_Pixel(x0, y0 - r, c);
	LCD_Pixel(x0 + r, y0, c);
	LCD_Pixel(x0 - r, y0, c);
	LCD_line(x0 - r, y0, x0 + r, y0, c);

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		LCD_line(x0 - x, y0 + y, x0 + x, y0 + y, c);
		LCD_line(x0 + x, y0 - y, x0 - x, y0 - y, c);

		LCD_line(x0 + y, y0 + x, x0 - y, y0 + x, c);
		LCD_line(x0 + y, y0 - x, x0 - y, y0 - x, c);
	}
}



void LCD_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t c) 
{
	uint8_t i;

	/* Check input parameters */
	if (
		x >= scr_width||
		y >= scr_height) {
		/* Return error */
		return;
	}

	/* Check width and height */
	if ((x + w) >= scr_width) {
		w = scr_width - x;
	}
	if ((y + h) >= scr_height) {
		h = scr_height - y;
	}

	/* Draw lines */
	for (i = 0; i <= h; i++) {
		/* Draw lines */
		LCD_line(x, y + i, x + w, y + i, c);
	}
}

/***** Text ******/



void LCD_drawChar(int16_t x, int16_t y, char c, uint16_t color) 
{		
	


	c -= (uint8_t)(gfxFont->first);
	GFXglyph *glyph = gfxFont->glyph + c;
	
	uint8_t *bitmap = gfxFont->bitmap;

	uint16_t bo = (glyph->bitmapOffset);
	
	uint8_t w = (glyph->width), h = (glyph->height);
	int8_t xo = (glyph->xOffset), yo = (glyph->yOffset);
		
	uint8_t xx, yy, bits = 0, bit = 0;
	
	for (yy = 0; yy < h; yy++) {
		for (xx = 0; xx < w; xx++) {
			if (!(bit++ & 7)) {
				bits = (bitmap[bo++]);
			}
			if (bits & 0x80)
			{
				LCD_Pixel(x + xo + xx, y + yo + yy, color);
			}

			bits <<= 1;
		}
	}

}



size_t LCD_write(uint8_t c) {
	// Custom font

	//gfxFont = (GFXfont *)&FreeSansBold24pt7b;
	//gfxFont = (GFXfont *)&Tiny3x3a2pt7b;
	
	//gfxFont = (GFXfont *)&FreeSansBoldOblique24pt7b;

	if (c == '\n') 
	{
		cursor_x = 0;
		cursor_y += (int16_t)textsize_y * (uint8_t)(gfxFont->yAdvance);
	}
	else if (c != '\r') 
	{
		uint8_t first = (gfxFont->first);
		if ((c >= first) && (c <= (uint8_t)(gfxFont->last))) 
		{
			GFXglyph *glyph = gfxFont->glyph + c - first;
			uint8_t w = (glyph->width),
			        h = (glyph->height);
			if ((w > 0) && (h > 0)) 
			{
				// Is there an associated bitmap?
				int16_t xo = (int8_t)(glyph->xOffset); // sic
				if (wrap && ((cursor_x + textsize_x * (xo + w)) > (scr_width - 1)))  
				{
					cursor_x = 0;
					cursor_y += (int16_t)textsize_y *
					            (uint8_t)(gfxFont->yAdvance);
				}
				LCD_drawChar(cursor_x, cursor_y, c, 1);
			}
			cursor_x += (uint8_t)(glyph->xAdvance) * (int16_t)textsize_x;
		}
	}

	return 1;
}

void LCD_setCursor(int16_t x, int16_t y) 
{
	cursor_x = x;
	cursor_y = y;
}

void LCD_setCursorY(int16_t y) 
{
	cursor_y = y;	
}

uint16_t LCD_getCursorX()
{
	return cursor_x;
}

uint16_t LCD_getCursorY()
{
	return cursor_y;
}

void LCD_setFont(const GFXfont *f) {
	if (f) {
		// Font struct pointer passed in?
		if (!gfxFont) {
			// And no current font struct?
		  // Switching from classic to new font behavior.
		  // Move cursor pos down 6 pixels so it's on baseline.
			cursor_y += 6;
		}
	}
	else if (gfxFont) {
		// NULL passed.  Current font struct defined?
	  // Switching from new to classic font behavior.
	  // Move cursor pos up 6 pixels so it's at top-left of char.
		cursor_y -= 6;
	}
	gfxFont = (GFXfont *)f;
}