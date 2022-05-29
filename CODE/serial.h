#ifndef CODE_SERIAL_H_
#define CODE_SERIAL_H_

void serial_io_init (void);
void serial_io (UARTN_enum uartn);
uint8 serialInt (UARTN_enum uartn, uint16 *data);
uint8 serialFlo (UARTN_enum uartn, float32 *data);

#endif
