/*
 * connections.h
 *
 *  Created on: Nov 14, 2021
 *      Author: ariedinger
 */

#ifndef CONNECTIONS_H_
#define CONNECTIONS_H_

#include "functions.h"

/*Pines del ADC para medir tension - PC0:*/
#define _adcVoltPort	GPIOC
#define _adcVoltPin		GPIO_Pin_0

/*Pines del ADC para medir corriente - PC3:*/
#define _adcCurrPort	GPIOC
#define _adcCurrPin		GPIO_Pin_3

#endif /* CONNECTIONS_H_ */
