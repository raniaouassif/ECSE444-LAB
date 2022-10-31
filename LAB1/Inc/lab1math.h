/*
 * lab1math.h
 *
 *  Created on: Sep. 13, 2022
 *      Author: raniaouassif
 */


#include "main.h"

#ifndef INC_LAB1MATH_H_
#define INC_LAB1MATH_H_

//extern : though defined it, it is implemented elsewhere

void cMax(float *array, uint32_t size, float *max, uint32_t *maxIndex);

extern void asmMax(float *array, uint32_t size, float *max, uint32_t *maxIndex);

void cMultiply(float *array1, float *array2, uint32_t size, float *y);

extern void asmMultiply(float *array1, float *array2, uint32_t size, float *y);

void cstd(float *array, uint32_t size, float *std);

extern void asmstd(float *array, uint32_t size, float *std);
#endif /* INC_LAB1MATH_H_ */
