/*
 * tasks.c
 *
 *  Created on: Apr 8, 2024
 *      Author: ftobler
 */


#include "tasks.h"
#include "application.h"
#include "flortos.h"



static void task_idle();
static void task_application();

static uint8_t stack_idle[TASK_STACK_SIZE];
static uint8_t stack_application[TASK_STACK_SIZE];



void task_setup() {
	scheduler_init();
	scheduler_addTask(TASK_IDLE, task_idle, stack_idle, TASK_STACK_SIZE); // lowest priority task (no sleep)
	scheduler_addTask(TASK_APPLICATION, task_application, stack_application, TASK_STACK_SIZE); // last task has highest priority
	scheduler_join();
}


static void task_idle() {
	init();
	while (1) {
		mainloop();
	}
}

static void task_application() {
	while (1) {
		scheduler_task_sleep(10000);
	}
}
