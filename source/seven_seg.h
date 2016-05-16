/*
 * seven_seg.h
 *
 * Seven segement display driver for LED's connected to
 * port C as shown below
 *
 *     a
 *    ----
 *  f|    |b
 *   |  g |
 *	  ----
 *	e|    |c
 *	 |    |
 *	  ----
 *	   d
 *
 *	 g = PTC9	e = PTC1
 *	 f = PTC8	d = PTC2
 *	 a = PTC10	c = PTC12
 *	 b = PTC11
 *  Created on: May 14, 2016
 *      Author: tisham
 */

#ifndef SOURCE_SEVEN_SEG_H_
#define SOURCE_SEVEN_SEG_H_

void init_7seg();

void display_num(uint8_t n);
void clear_display();


#endif /* SOURCE_SEVEN_SEG_H_ */
