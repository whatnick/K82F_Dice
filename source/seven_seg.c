/*
 * seven_seg.c
 *
 *  Created on: May 14, 2016
 *      Author: tisham
 */

#include "board.h"
#include "seven_seg.h"

void init_7seg()
{
	LED_SEG1_INIT(1);
	LED_SEG2_INIT(1);
	LED_SEG3_INIT(1);
	LED_SEG4_INIT(1);
	LED_SEG5_INIT(1);
	LED_SEG6_INIT(1);
	LED_SEG7_INIT(1);
}

void display_num(char n)
{
	if(n > 9) return;
	if(n == 1)
	{
		LED_SEG1_OFF();
		LED_SEG2_ON();
		LED_SEG3_ON();
		LED_SEG4_ON();
		LED_SEG5_ON();
		LED_SEG6_OFF();
		LED_SEG7_ON();
	}
	if(n == 2)
	{
		LED_SEG1_OFF();
		LED_SEG2_OFF();
		LED_SEG3_OFF();
		LED_SEG4_ON();
		LED_SEG5_OFF();
		LED_SEG6_ON();
		LED_SEG7_OFF();
	}
	if(n == 3)
	{
		LED_SEG1_OFF();
		LED_SEG2_OFF();
		LED_SEG3_ON();
		LED_SEG4_ON();
		LED_SEG5_OFF();
		LED_SEG6_OFF();
		LED_SEG7_OFF();
	}
	if(n == 4)
	{
		LED_SEG1_OFF();
		LED_SEG2_ON();
		LED_SEG3_ON();
		LED_SEG4_OFF();
		LED_SEG5_ON();
		LED_SEG6_OFF();
		LED_SEG7_OFF();
	}
	if(n == 5)
	{
		LED_SEG1_ON();
		LED_SEG2_OFF();
		LED_SEG3_ON();
		LED_SEG4_OFF();
		LED_SEG5_OFF();
		LED_SEG6_OFF();
		LED_SEG7_OFF();
	}
	if(n == 6)
	{
		LED_SEG1_ON();
		LED_SEG2_OFF();
		LED_SEG3_OFF();
		LED_SEG4_OFF();
		LED_SEG5_OFF();
		LED_SEG6_OFF();
		LED_SEG7_OFF();
	}
	if(n == 7)
	{
		LED_SEG1_OFF();
		LED_SEG2_OFF();
		LED_SEG3_ON();
		LED_SEG4_ON();
		LED_SEG5_OFF();
		LED_SEG6_OFF();
		LED_SEG7_ON();
	}
	if(n == 8)
	{
		LED_SEG1_OFF();
		LED_SEG2_OFF();
		LED_SEG3_OFF();
		LED_SEG4_OFF();
		LED_SEG5_OFF();
		LED_SEG6_OFF();
		LED_SEG7_OFF();
	}
	if(n == 9)
	{
		LED_SEG1_OFF();
		LED_SEG2_OFF();
		LED_SEG3_ON();
		LED_SEG4_OFF();
		LED_SEG5_OFF();
		LED_SEG6_OFF();
		LED_SEG7_OFF();
	}
	if(n == 0)
	{
		LED_SEG1_OFF();
		LED_SEG2_OFF();
		LED_SEG3_OFF();
		LED_SEG4_OFF();
		LED_SEG5_OFF();
		LED_SEG6_OFF();
		LED_SEG7_ON();
	}
}


