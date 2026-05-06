#include "movement.h"

//void move_forward(oi_t *sensor_data, double distance_mm) {
//    double sum = 0; // distance member in oi_t struct is type double
//    oi_setWheels(100,100); //move forward at set speed
//    while (sum < ((distance_mm)*.97)+0) {
//        oi_update(sensor_data);
//        sum += sensor_data -> distance;
//       }
//
//    oi_setWheels(0, 0); // Stop the robot
//}
//
//void move_backward(oi_t *sensor_data, double distance_mm) {
//     double sum = 0; // distance member in oi_t struct is type double
//     oi_setWheels(-100,-100); //move forward at set speed
//     while (sum > ((-distance_mm)*1)+ 13.75) {
//         oi_update(sensor_data);
//         sum += sensor_data -> distance;
//    }
//        oi_setWheels(0, 0); // Stop the robot
//}
//
//void turn_right(oi_t *sensor_data, double angle_degrees) {
//     double sum = 0; // distance member in oi_t struct is type double
//     oi_setWheels(-50,50); //move forward at set speed
//     while (sum > ((-angle_degrees)+2.5)) {
//         oi_update(sensor_data);
//         sum += sensor_data -> angle;
//    }
//        oi_setWheels(0, 0); // Stop the robot
//}
//void turn_left(oi_t *sensor_data, double angle_degrees) {
//    double sum = 0; // distance member in oi_t struct is type double
//    oi_setWheels(50,-50); //move forward at set speed
//    while (sum < ((angle_degrees)-3.4)) {
//        oi_update(sensor_data);
//        sum += sensor_data -> angle;
//       }
//
//    oi_setWheels(0, 0); // Stop the robot
//}

#include "movement.h"
#include "uart.h"

void send_event(char event[])
{
    int i = 0;
    uart_sendChar('\r');
    uart_sendChar('\n');

    for (i = 0; event[i] != '\0'; i++)
    {
        uart_sendChar(event[i]);
    }

    uart_sendChar('\r');
    uart_sendChar('\n');
}

int move_forward(oi_t *sensor_data, double distance_mm) {
    double sum = 0; // distance member in oi_t struct is type double
    char msg[50];
    oi_setWheels(200,200); //move forward at set speed

    while (sum < ((distance_mm)*.97)+0) {

        oi_update(sensor_data);
        int rightSignal = sensor_data->cliffFrontRightSignal;
        int leftSignal = sensor_data->cliffFrontLeftSignal;

        if(sensor_data->bumpRight) {
            oi_setWheels(0,0);
            send_event("EVENT:BUMP RIGHT");
            return 1;  // stop moving and signal collision
        }

        if(sensor_data->bumpLeft) {
            oi_setWheels(0,0);
            send_event("EVENT:BUMP LEFT");
            return 1;
        }

        if(rightSignal > 2600 && leftSignal > 2000){ // White Tape
            oi_setWheels(0,0);
            send_event("EVENT:WHITE TAPE");
            return 2;
        }
        if(rightSignal < 500 && leftSignal < 500){ // Drop
            oi_setWheels(0,0);
            send_event("EVENT:DROP");
            return 3;
        }

        sum += sensor_data -> distance;
       }
    
    oi_setWheels(0, 0); // Stop the robot

    sprintf(msg, "MOVE:FORWARD_%d", (int)sum);
    send_event(msg);
    return 0;

}

void move_backward(oi_t *sensor_data, double distance_mm) {
     double sum = 0; // distance member in oi_t struct is type double
     char msg[50];
     oi_setWheels(-100,-100); //move forward at set speed
     while (sum > ((-distance_mm)*1)+ 13.75) {
         oi_update(sensor_data);
        int rightSignal = sensor_data->cliffFrontRightSignal;
        int leftSignal = sensor_data->cliffFrontLeftSignal;

        if(sensor_data->bumpRight) {
            oi_setWheels(0,0);
            send_event("EVENT:BUMP RIGHT");
            return 1;  // stop moving and signal collision
        }

        if(sensor_data->bumpLeft) {
            oi_setWheels(0,0);
            send_event("EVENT:BUMP LEFT");
            return 1;
        }

        if(rightSignal > 2600 && leftSignal > 2000){ // White Tape
            oi_setWheels(0,0);
            send_event("EVENT:WHITE TAPE");
            return 2;
        }
        if(rightSignal < 500 && leftSignal < 500){ // Drop
            oi_setWheels(0,0);
            send_event("EVENT:DROP");
            return 3;
        }
         sum += sensor_data -> distance;
    }
        sprintf(msg, "MOVE:BACKWARD_%d", (int)sum);
        send_event(msg);
        oi_setWheels(0, 0); // Stop the robot
        return 0;
}
//Cybot 8 right 89
void turn_right(oi_t *sensor_data, double angle_degrees) {
     double sum = 0; // distance member in oi_t struct is type double
     char msg[50];
     oi_setWheels(-50,50); //move forward at set speed
     while (sum > ((-angle_degrees)+2.5)) {
         oi_update(sensor_data);
        int rightSignal = sensor_data->cliffFrontRightSignal;
        int leftSignal = sensor_data->cliffFrontLeftSignal;

        if(sensor_data->bumpRight) {
            oi_setWheels(0,0);
            send_event("EVENT:BUMP RIGHT");
            return 1;  // stop moving and signal collision
        }

        if(sensor_data->bumpLeft) {
            oi_setWheels(0,0);
            send_event("EVENT:BUMP LEFT");
            return 1;
        }

        if(rightSignal > 2600 && leftSignal > 2000){ // White Tape
            oi_setWheels(0,0);
            send_event("EVENT:WHITE TAPE");
            return 2;
        }
        if(rightSignal < 500 && leftSignal < 500){ // Drop
            oi_setWheels(0,0);
            send_event("EVENT:DROP");
            return 3;
        }
         sum += sensor_data -> angle;
    }
        sprintf(msg, "TURN:RIGHT_%d", (int)sum);
        send_event(msg);
        oi_setWheels(0, 0); // Stop the robot
        return 0;
}
//Cybot 8 left 95
void turn_left(oi_t *sensor_data, double angle_degrees) {
    double sum = 0; // distance member in oi_t struct is type double
    char msg[50];
    angle_degrees = angle_degrees - 4;
    oi_setWheels(50,-50); //move forward at set speed
    while (sum < ((angle_degrees)-3.4)) {
        oi_update(sensor_data);
        int rightSignal = sensor_data->cliffFrontRightSignal;
        int leftSignal = sensor_data->cliffFrontLeftSignal;

        if(sensor_data->bumpRight) {
            oi_setWheels(0,0);
            send_event("EVENT:BUMP RIGHT");
            return 1;  // stop moving and signal collision
        }

        if(sensor_data->bumpLeft) {
            oi_setWheels(0,0);
            send_event("EVENT:BUMP LEFT");
            return 1;
        }

        if(rightSignal > 2600 && leftSignal > 2000){ // White Tape
            oi_setWheels(0,0);
            send_event("EVENT:WHITE TAPE");
            return 2;
        }
        if(rightSignal < 500 && leftSignal < 500){ // Drop
            oi_setWheels(0,0);
            send_event("EVENT:DROP");
            return 3;
        }
        sum += sensor_data -> angle;
       }
    sprintf(msg, "TURN:LEFT_%d", (int)sum);
    send_event(msg);

    oi_setWheels(0, 0); // Stop the robot
    return 0;
}
