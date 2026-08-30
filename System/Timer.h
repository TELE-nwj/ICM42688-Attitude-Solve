#ifndef __TIMER_H
#define __TIMER_H

void Timer_Init(void);

/*200Hz定时中断初始化（TIM3，5ms），用于跑Mahony解算*/
void Timer_200Hz_Init(void);

#endif
