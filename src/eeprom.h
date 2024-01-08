#ifndef _EEPROM_H_
#define _EEPROM_H_

#include <twr.h>

uint32_t crc32(const void *buf, size_t size);
bool eeprom_read(void);
bool eeprom_write(void);

#endif