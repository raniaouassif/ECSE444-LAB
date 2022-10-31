/*
 * cstd.c
 *
 *  Created on: Sep. 19, 2022
 *      Author: raniaouassif
 */

#include "main.h"
#include <math.h>
#include "lab1math.h"

void cstd(float *array, uint32_t size, float *std) {
	float avg = 0;
	float sum = 0 ;

	for(uint32_t i = 0; i < size; i++) {
		avg += array[i];
	}
	avg/= size;

	for(uint32_t i = 0 ; i < size; i++) {
		sum += pow((array[i] - avg),2);
	}

	sum/= (size-1);
	(*std) = sqrt(sum);
}


