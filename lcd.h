#ifndef LCD_H
#define LCD_H

#include "types.h"

void lcd_setup(void);
void lcd_write(const char * const buf);
void lcd_clear(void);

#endif
