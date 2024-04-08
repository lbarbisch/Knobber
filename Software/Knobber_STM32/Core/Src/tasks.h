/*
 * tasks.h
 *
 *  Created on: Apr 8, 2024
 *      Author: ftobler
 */

#ifndef SRC_TASKS_H_
#define SRC_TASKS_H_


enum {
	TASK_STACK_SIZE = 768,

	TASK_IDLE = 0,

	TASK_APPLICATION = 1,
};

void task_setup();

#endif /* SRC_TASKS_H_ */
