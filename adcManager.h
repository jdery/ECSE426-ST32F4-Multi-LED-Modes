#ifndef	 ADCMANAGER_H
#define ADCMANAGER_H

#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

void configureADC(void);
void acquireADCValue(void);

#endif
