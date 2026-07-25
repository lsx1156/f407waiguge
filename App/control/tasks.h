#ifndef __TASKS_H__
#define __TASKS_H__

void tasks_init(void);
void network_task_run(void);
void control_task_run(void);
void safety_task_run(void);
void eeprom_task_run(void);

#endif
