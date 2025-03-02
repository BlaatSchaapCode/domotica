/*
 * display.h
 *
 *  Created on: 14 jun. 2024
 *      Author: andre
 */

#ifndef DISPLAY_H_
#define DISPLAY_H_

void display_set_key(char key);
void display_init(void);
void display_process(void);


void display_clear();
void display_print_upper(char*);
void display_print_middle(char*);
void display_print_lower(char*);
void display_apply();

#endif /* DISPLAY_H_ */
