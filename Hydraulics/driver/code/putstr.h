// putstr.h

#ifndef __PUTSTR_H__
#define __PUTSTR_H__

#ifdef __cplusplus
extern "C" {
#endif

void putstr(const char *str);
void putchr(char c);
void putU8(uint8_t number);
void putB8(uint8_t number);
void putint(int32_t number);
void puthex(uint8_t byte);
void putS16(int16_t number);

#ifdef __cplusplus
}
#endif

#endif // __PUTSTR_H__

