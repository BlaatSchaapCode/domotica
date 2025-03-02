#ifndef IR_H
#define IR_H
#include <stdbool.h>

void ir_handler(bool pinstate);
void ir_init(void);
void ir_process(void);
#endif
