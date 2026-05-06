/*
 * adc.h
 *
 *  Created on: Mar 25, 2026
 *      Author: andy9899
 */

#ifndef ADC_H_
#define ADC_H_

#include <inc/tm4c123gh6pm.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "timer.h"

void adc_init(void);
uint32_t adc_read(void);



#endif /* ADC_H_ */
