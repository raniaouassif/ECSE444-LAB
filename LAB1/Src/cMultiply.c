/*
 * cMultiply.c
 *
 *  Created on: Sep. 17, 2022
 *      Author: raniaouassif
 */

#include "main.h"
#include "lab1math.h"

void cMultiply(float *array1, float *array2, uint32_t size, float *y) {
	for (uint32_t i = 0 ; i < size; i++) {
		y[i] = array1[i] * array2[i];
	}
}
