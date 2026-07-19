#ifndef BEEP_H
#define BEEP_H

#include <stdint.h>

#define BUZZER_PIN  A14

void Buzzer_Init(void);
void Buzzer_Beep(void);
void Buzzer_BeepMs(uint32_t ms);
void Buzzer_Poll(void);

#endif
