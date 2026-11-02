#ifndef LOGGING_H_
#define LOGGING_H_

#include <stdint.h>

#define SAMPLE_RATE_HZ   100
#define LOG_SIZE         4000

void     Log_init(void);          // nastaví timer, vynuluje pole
uint8_t  Log_sampleReady(void);   // 1, pokud je čas na další vzorek (a příznak smaže)
void     Log_add(int16_t value);  // uloží vzorek, po zaplnění nedělá nic
uint8_t  Log_isFull(void);
uint16_t Log_count(void);
int16_t  Log_get(uint16_t index);
void     Log_reset(void);
void Log_send(void);

#endif /* LOGGING_H_ */