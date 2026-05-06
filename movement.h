


#ifndef HEADER_FILE
#define HEADER_FILE

#include "open_interface.h"
#include "Timer.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
//Function headers and macro definitions
int move_forward(oi_t *sensor_data, double distance_mm);
void move_backward(oi_t *sensor_data, double distance_mm);
void turn_left(oi_t *sensor_data, double angle_degrees);
void turn_right(oi_t *sensor_data, double angle_degrees);

#endif



