#include "movement.h"
#include "open_interface.h"
#include "lcd.h"
#include "Timer.h"
#include "uart.h"
#include "servo.h"
#include "ping.h"
#include "adc.h"
#include <stdint.h>
#include "bno.h"

void manual_mode(oi_t *sensor_data);
void scan_only_print(void);


/*
 * Readability cleanup only:
 * - keep behavior the same
 * - add named constants
 * - break repeated scan logic into helpers
 * - keep existing comments / structure where practical
 */

#define SCAN_POINTS              91
#define MAX_OBJECTS              30
#define MAX_FENCE_POSTS          10
#define UART_MSG_LEN             100

#define SERVO_STEP_DEG           2
#define SERVO_SETTLE_MS          50
#define MAX_SENSOR_DIST          110

#define OBJECT_EDGE_THRESHOLD    10
#define OBJECT_MIN_ANGULAR_WIDTH 4
#define OBJECT_MAX_ANGULAR_WIDTH 30

#define POST_MIN_WIDTH           1
#define POST_MAX_WIDTH           5
#define TREE_MIN_WIDTH           6
#define TREE_MAX_WIDTH           16

#define SCAN_ONLY_DIST           80
#define INITIAL_SEARCH_MAX_DIST  30
#define TAPE_SEARCH_MAX_DIST     30
#define TREE_SEARCH_MAX_DIST     50
#define FENCE_TRACK_MAX_DIST     50
#define FENCE_FORWARD_MIN_ANGLE  45
#define FENCE_FORWARD_MAX_ANGLE  135
#define FENCE_RIGHT_MAX_ANGLE    30
#define FENCE_LEFT_MIN_ANGLE     150


#define MOVE_SEARCH_MM           100
#define MOVE_TAPE_BACKUP_MM      80
#define MOVE_BUMP_BACKUP_MM      100
#define MOVE_BUMP_BACKUP_BIG_MM  200
#define MOVE_TREE_ADVANCE_MM     300
#define MOVE_HOLE_MM             400

#define RIGHT_90                 90
#define LEFT_90                  90
#define SMALL_AVOID_TURN         20
#define PING_OFFSET              4

#define TREE_LIMIT               3

typedef struct {
    int distArray[SCAN_POINTS];
    int distIRArray[SCAN_POINTS];
} ScanData;

typedef struct {
    int startDegree;
    int active_object;
    int objectCount;
} DetectionState;

typedef struct {
    int fence_found;
    int tape_found;
    int fence_aligned;
    int first_run;
    int rescan;
    int scan_stopped;
    int trees_cut;
    int fence_travel_right;
    int ignore_left_posts;
    int ignore_right_posts;
} BotState;

typedef struct {
    int angles[MAX_FENCE_POSTS];
    int dists[MAX_FENCE_POSTS];
    int count;
} FencePosts;

typedef struct {
    int found;
    int angle;
    int dist;
    int width;
    int best_score;
} TreeTarget;

void send(char str[])
{
    int i = 0;
    for (i = 0; str[i] != '\0'; i++)
    {
        uart_sendChar(str[i]);
    }
}

int isRoom(int angleObj[], int distObj[], int target, int size)
{
    if (size <= 1) return 1; // only one object

    int left = target - 1;
    int right = target + 1;

    // Check left
    if (left >= 0) {
        double angleDiff = abs(angleObj[target] - angleObj[left]) * M_PI / 180.0;

        double distBetween = sqrt(pow(distObj[target], 2) + pow(distObj[left], 2) -
                                  2 * distObj[target] * distObj[left] * cos(angleDiff));

        if (distBetween < 20) return 0;
    }

    // Check right
    if (right < size) {
        double angleDiff = abs(angleObj[target] - angleObj[right]) * M_PI / 180.0;

        double distBetween = sqrt(pow(distObj[target], 2) +
                                  pow(distObj[right], 2) -
                                  2 * distObj[target] * distObj[right] * cos(angleDiff));

        if (distBetween < 20) return 0;
    }

    return 1;
}


static void print_scan_header(void)
{
    send("Angle\tObj\tObj Angle\tWidth\tDist\r\n");
}

static void print_angle_only(char msg[], int angle, int dist)
{
    sprintf(msg, "%d\t%d\r\n", angle, dist);
    send(msg);
}

static void print_object_row(char msg[], int angle, int objectCount, int midPoint,
                             int width, int middleDistance)
{
    sprintf(msg, "%d\t%d\t%d\t%d\t%d\r\n",
            angle, objectCount, midPoint, width, middleDistance);
    send(msg);
}

static int clamp_sensor_distance(int value)
{
    if (value > MAX_SENSOR_DIST) {
        return MAX_SENSOR_DIST;
    }
    return value;
}

static void read_scan_point(int index, ScanData *scan)
{
    int pingDistance = 0;
    int rawIR = 0;
    int IR = 0;

    servo_move(index * SERVO_STEP_DEG);
    timer_waitMillis(SERVO_SETTLE_MS);
    ping_trigger();

    {
        float *pingData = ping_getDistance();
        pingDistance = (int) pingData[2];
    }

    rawIR = adc_read();

    if (rawIR < 1) rawIR = 1;
    IR = pow(19354.0 / rawIR, 1.0 / .869); // Cybot 08
//    IR = pow(rawIR, -1.327) * 278394.0; //Cybot 21
//      IR = pow(rawIR, -1.406) * 522524; //Cybot 05
//    IR = pow(rawIR, -1.385) * 439632 + 1; // cyBot 20

    scan->distArray[index] = clamp_sensor_distance(pingDistance);
    scan->distIRArray[index] = clamp_sensor_distance(IR);
}

static int run_scan_with_manual_interrupt(oi_t *sensor_data, char *c, ScanData *scan)
{
    int i = 0;

    for (i = 0; i < SCAN_POINTS; i++)
    {
        *c = uart_receive_nonblocking();

        if (*c == 'p') {
            manual_mode(sensor_data);
            *c = '\0';
            return 1;
        }

        read_scan_point(i, scan);
    }

    return 0;
}
static int run_coarse_scan_with_manual_interrupt(oi_t *sensor_data, char *c, ScanData *scan)
{
    int i = 0;

    for (i = 0; i < SCAN_POINTS; i++)
    {
        *c = uart_receive_nonblocking();

        if (*c == 'p') {
            manual_mode(sensor_data);
            *c = '\0';
            return 1;
        }

        if (i % 2 == 0) {
            read_scan_point(i, scan);  // real scan (servo moves)
        } else {
            // fill gaps so detection logic still works
            scan->distArray[i] = scan->distArray[i - 1];
            scan->distIRArray[i] = scan->distIRArray[i - 1];
        }
    }

    return 0;
}

static void store_fence_post(FencePosts *posts, int midPoint, int middleDistance)
{
    if (posts->count < MAX_FENCE_POSTS)
    {
        posts->angles[posts->count] = midPoint;
        posts->dists[posts->count] = middleDistance;
        posts->count++;
    }
}

static int is_post_width(int width)
{
    return (width > POST_MIN_WIDTH && width <= POST_MAX_WIDTH);
}

static int is_close_fence_post(int width, int middleDistance)
{
    return (is_post_width(width) && middleDistance < FENCE_TRACK_MAX_DIST);
}

static int is_forward_post_angle(int midPoint)
{
    return (midPoint >= FENCE_FORWARD_MIN_ANGLE &&
            midPoint <= FENCE_FORWARD_MAX_ANGLE);
}

static int is_right_side_post_angle(int midPoint)
{
    return (midPoint <= FENCE_RIGHT_MAX_ANGLE);
}

static int is_left_side_post_angle(int midPoint)
{
    return (midPoint >= FENCE_LEFT_MIN_ANGLE);
}

static int should_ignore_alignment_post(BotState *state, int midPoint)
{
    if (state->ignore_left_posts && is_left_side_post_angle(midPoint))
    {
        return 1;
    }

    if (state->ignore_right_posts && is_right_side_post_angle(midPoint))
    {
        return 1;
    }

    return 0;
}

static int process_detected_object(DetectionState *detect, ScanData *scan, int i,
                                   int *midPoint, int *middleDistance,
                                   int *angularWidth, int *width, float *radians)
{
    if (i <= 0 || i >= (SCAN_POINTS - 1)) {
        return 0;
    }

    if ((scan->distIRArray[i] - scan->distIRArray[i - 1]) > OBJECT_EDGE_THRESHOLD &&
        detect->active_object == 1)
    {
        *angularWidth = (i * SERVO_STEP_DEG - detect->startDegree) - PING_OFFSET;

        if(*angularWidth < 0){
            *angularWidth = 0;
        }


        if (*angularWidth >= OBJECT_MIN_ANGULAR_WIDTH &&
            *angularWidth <= OBJECT_MAX_ANGULAR_WIDTH)
        {
            *midPoint = (detect->startDegree + i * SERVO_STEP_DEG) / 2;
            *middleDistance = (scan->distArray[*midPoint / SERVO_STEP_DEG]) - PING_OFFSET;
            *radians = (*angularWidth * M_PI / 180);
            *width = 2 * (*middleDistance) * sinf(*radians / 2);
            detect->objectCount++;
            detect->active_object = 0;
            return 1;
        }

        detect->active_object = 0;
    }

    return 0;
}

static void start_object_if_needed(DetectionState *detect, ScanData *scan, int i, int maxDist)
{
    if (i > 1 &&
        (scan->distIRArray[i - 1] - scan->distIRArray[i]) > OBJECT_EDGE_THRESHOLD &&
        detect->active_object == 0 &&
        scan->distArray[i] < maxDist)
    {
        detect->startDegree = i * SERVO_STEP_DEG;
        detect->active_object = 1;
    }
}
static int handle_hole_on_tape_phase(oi_t *sensor_data, BotState *state,
                                     ScanData *scan, DetectionState *detect,
                                     FencePosts *posts, char msg[], int hole_length)
{
    int j = 0;
    int touch = 0;
    int i = 0;
    int thin_count = 0;
    int printedObject = 0;
    int midPoint = 0;
    int width = 0;
    int middleDistance = 0;
    int angularWidth = 0;
    float radians = 0.0f;

    while (j < hole_length)
    {
        thin_count = 0;
        posts->count = 0;
        detect->active_object = 0;

        for (i = 0; i < SCAN_POINTS; i++)
        {
            read_scan_point(i, scan);
        }

        for (i = 0; i < SCAN_POINTS; i++)
        {
            printedObject = 0;

            start_object_if_needed(detect, scan, i, INITIAL_SEARCH_MAX_DIST);

            if (process_detected_object(detect, scan, i, &midPoint, &middleDistance,
                                        &angularWidth, &width, &radians))
            {
                print_object_row(msg, i * SERVO_STEP_DEG, detect->objectCount,
                                 midPoint, width, middleDistance);
                printedObject = 1;

                if (width > POST_MIN_WIDTH && width < POST_MAX_WIDTH)
                {
                    thin_count++;
                    store_fence_post(posts, midPoint, middleDistance);
                }
            }

            if (!printedObject)
            {
                print_angle_only(msg, i * SERVO_STEP_DEG, scan->distIRArray[i]);
            }
        }

        if (thin_count > 0)
        {
            state->fence_found = 1;
            state->rescan = 1;
            lcd_printf("Found Fence");
            return 1;
        }

        touch = move_forward(sensor_data, MOVE_HOLE_MM, 0);

        if (touch == 2) // tape again
        {
            move_backward(sensor_data, MOVE_TAPE_BACKUP_MM, 0);
            turn_right(sensor_data, RIGHT_90, 1);
        }
        else if (touch == 1) // bump
        {
            move_backward(sensor_data, MOVE_BUMP_BACKUP_MM, 1);
            turn_right(sensor_data, RIGHT_90, 1);
        }
        else if (touch == 3) // another drop
        {
            move_backward(sensor_data, 100, 1);
            turn_right(sensor_data, RIGHT_90, 1);
        }

        j++;
    }

    return 0;
}
// returns 1 if fence was found during hole avoidance
// returns 0 if no fence found
int avoid_drop_return_path(oi_t *sensor_data, BotState *state,
                           ScanData *scan, DetectionState *detect,
                           FencePosts *posts, char msg[])
{
    int found = 0;

    move_backward(sensor_data, 100, 1);

    // go right around the hole
    turn_right(sensor_data, RIGHT_90, 1);

    found = handle_hole_on_tape_phase(sensor_data, state, scan, detect, posts, msg , 2);

    turn_left(sensor_data, LEFT_90, 1);

    found = handle_hole_on_tape_phase(sensor_data, state, scan, detect, posts, msg, 3);
    if (found) return 1;

    turn_left(sensor_data, LEFT_90, 1);

    found = handle_hole_on_tape_phase(sensor_data, state, scan, detect, posts, msg, 3);
    if (found) return 1;

    state->rescan = 1;
    return 0;
}

void scan_only_print(void)
{
    char msg[UART_MSG_LEN];
    ScanData scan;
    DetectionState detect = {0, 0, 0};
    int i = 0;
    int printedObject = 0;
    int midPoint = 0;
    int width = 0;
    int middleDistance = 0;
    int angularWidth = 0;
    float radians = 0.0f;

    print_scan_header();

    for (i = 0; i < SCAN_POINTS; i++)
    {
        printedObject = 0;
        read_scan_point(i, &scan);

        start_object_if_needed(&detect, &scan, i, SCAN_ONLY_DIST);

        if (process_detected_object(&detect, &scan, i, &midPoint, &middleDistance,
                                    &angularWidth, &width, &radians))
        {
            print_object_row(msg, i * SERVO_STEP_DEG, detect.objectCount,
                             midPoint, width, middleDistance);
            printedObject = 1;
        }

        if (!printedObject)
        {
            print_angle_only(msg, i * SERVO_STEP_DEG, scan.distIRArray[i]);
        }
    }
}
void cut_off(void){

             oi_setLeds(0, 0, 255, 0);

           }
void cut_on(void){

        oi_play_song(1);

        oi_setLeds(0, 0, 255, 255);

       }

void manual_mode(oi_t *sensor_data)
{
    char key = '\0';
    int sensorDetected = 0;

    lcd_printf("MANUAL");

    while (1) {
        key = uart_receive_nonblocking();

        if (key == 'w') {
            move_forward(sensor_data, 30, 0);
        }
        else if (key == 's') {
            move_backward(sensor_data, 30, sensorDetected);
        }
        else if (key == 'a') {
            turn_left(sensor_data, 10, sensorDetected);
        }
        else if (key == 'd') {
            turn_right(sensor_data, 10, sensorDetected);
        }
        else if (key == 'r') {
            scan_only_print();
            lcd_printf("SCAN ONLY");
        }
        else if (key == 'c'){
          cut_on();
        }
        else if(key == 'C'){

             cut_off();
        }
        else if (key == 'g') {
            break; //back to scan
        }
    }
}

void tap_tree_and_return(oi_t *sensor_data, int tree_angle, int tree_dist)
{
    int turn_amount = 0;
    int approach_dist = 0;
    int touched_tree = 0;

    if (tree_dist > 15)
    {
        approach_dist = (tree_dist - 5) * 10;
    }

    if (tree_angle < 90)
    {
        turn_amount = 90 - tree_angle;
        turn_right(sensor_data, turn_amount, 1);
        touched_tree = move_forward(sensor_data, approach_dist, 0); //Watch for hole here
        if (touched_tree != 3)
        {
            move_backward(sensor_data, approach_dist, 0);
        }

        turn_left(sensor_data, turn_amount, 1);
    }
    else if (tree_angle > 90)
    {
        turn_amount = tree_angle - 90;
        turn_left(sensor_data, turn_amount, 1);
        touched_tree = move_forward(sensor_data, approach_dist, 0);
        if (touched_tree != 3)
        {
            move_backward(sensor_data, approach_dist, 0);
        }
        turn_right(sensor_data, turn_amount, 0);
    }
    else
    {
        touched_tree = move_forward(sensor_data, approach_dist, 0);
        if (touched_tree != 3)
        {
            move_backward(sensor_data, 120, 0);
        }
    }

    if (touched_tree == 3)
    {
//        avoid_drop_return_path(sensor_data);
    }
}

static void restore_scan_orientation_if_needed(oi_t *sensor_data, BotState *state)
{
    if (state->fence_aligned)
    {
        if (state->fence_travel_right)
        {
            turn_left(sensor_data, LEFT_90, 0);
        }
        else
        {
            turn_right(sensor_data, RIGHT_90, 0);
        }
    }
}

static void follow_fence_direction(oi_t *sensor_data, BotState *state)
{
    if (state->fence_travel_right)
    {
        turn_right(sensor_data, RIGHT_90, 0);
    }
    else
    {
        turn_left(sensor_data, LEFT_90, 0);
    }
}

static void small_follow_adjust(oi_t *sensor_data, BotState *state)
{
    if (state->fence_travel_right)
    {
        turn_right(sensor_data, SMALL_AVOID_TURN, 0);
    }
    else
    {
        turn_left(sensor_data, SMALL_AVOID_TURN, 0);
    }
}

static int handle_first_search_phase(oi_t *sensor_data, BotState *state,
                                     ScanData *scan, DetectionState *detect,
                                     FencePosts *posts, char msg[])
{
    int i = 0;
    int thin_count = 0;
    int printedObject = 0;
    int midPoint = 0;
    int width = 0;
    int middleDistance = 0;
    int angularWidth = 0;
    float radians = 0.0f;
    int touch = 0;
    int sensorDetected = 0;

    posts->count = 0;
    detect->active_object = 0;

    for (i = 0; i < SCAN_POINTS; i+= 2)
    {
        if(i<2) continue;
        printedObject = 0;

        start_object_if_needed(detect, scan, i, INITIAL_SEARCH_MAX_DIST);

        if (process_detected_object(detect, scan, i, &midPoint, &middleDistance,
                                    &angularWidth, &width, &radians))
        {
            print_object_row(msg, i * SERVO_STEP_DEG, detect->objectCount,
                             midPoint, width, middleDistance);
            printedObject = 1;

            // thin post detection
            if (width > POST_MIN_WIDTH && width < POST_MAX_WIDTH)
            {
                thin_count++;
                store_fence_post(posts, midPoint, middleDistance);
            }
        }

        if (!printedObject)
        {
            print_angle_only(msg, i * SERVO_STEP_DEG, scan->distIRArray[i]);
        }
    }

    if (thin_count >= 1)
    {
        state->fence_found = 1;
        state->first_run = 0;
        state->rescan = 1;
        move_backward(sensor_data, 50, 0);
        return 1;
    }



    touch = move_forward(sensor_data, MOVE_SEARCH_MM, 0);

    if (touch == 2) // White tape
    {
        state->tape_found = 1;
        state->first_run = 0;
        sensorDetected = 1;
        move_backward(sensor_data, MOVE_TAPE_BACKUP_MM , sensorDetected);
        turn_right(sensor_data, RIGHT_90 , sensorDetected);
        state->rescan = 1;
        state->fence_found = 0;

        return 1;
    }

    if (touch == 1) // Bump
    {
        move_backward(sensor_data, MOVE_BUMP_BACKUP_BIG_MM, sensorDetected);
    }

    if (touch == 3) // Drop
    {
        turn_right(sensor_data, 180, sensorDetected);
    }

    state->rescan = 1;
    return 1;
}

static int handle_tape_to_fence_phase(oi_t *sensor_data, BotState *state,
                                      ScanData *scan, DetectionState *detect,
                                      FencePosts *posts, char msg[])
{
    int i = 0;
    int thin_count = 0;
    int printedObject = 0;
    int midPoint = 0;
    int width = 0;
    int middleDistance = 0;
    int angularWidth = 0;
    float radians = 0.0f;
    int touch = 0;

    posts->count = 0;
    detect->active_object = 0;

    for (i = 0; i < SCAN_POINTS; i++)
    {
        printedObject = 0;

        start_object_if_needed(detect, scan, i, TAPE_SEARCH_MAX_DIST);

        if (process_detected_object(detect, scan, i, &midPoint, &middleDistance,
                                    &angularWidth, &width, &radians))
        {
            print_object_row(msg, i * SERVO_STEP_DEG, detect->objectCount,
                             midPoint, width, middleDistance);
            printedObject = 1;

            // thin post detection
            if (width > POST_MIN_WIDTH && width < POST_MAX_WIDTH)
            {
                thin_count++;
                store_fence_post(posts, midPoint, middleDistance);
            }
        }

        if (!printedObject)
        {
            print_angle_only(msg, i * SERVO_STEP_DEG, scan->distIRArray[i]);
        }
    }

    // fence detection
    if (thin_count > 0)
    {
        state->fence_found = 1;
        state->rescan = 1;
        lcd_printf("Found Fence");
        return 1;
    }

    touch = move_forward(sensor_data, MOVE_SEARCH_MM, 0);

    if (touch == 2) // still on tape
    {
        move_backward(sensor_data, MOVE_TAPE_BACKUP_MM, 1);
        turn_right(sensor_data, RIGHT_90, 1);
        state->fence_found = 0;
    }
    else if (touch == 1) // bump
    {
        move_backward(sensor_data, MOVE_BUMP_BACKUP_MM, 0);
        turn_right(sensor_data, RIGHT_90, 0);
    }
    else if (touch == 3) // drop
    {
        if (avoid_drop_return_path(sensor_data, state, scan, detect, posts, msg))
        {
            return 1; // fence found, main will go to alignment phase next
        }
    }

    state->rescan = 1;
    return 1;
}

static void collect_alignment_posts_from_scan(BotState *state, ScanData *scan,
                                              DetectionState *detect,
                                              FencePosts *posts, char msg[])
{
    int i = 0;
    int printedObject = 0;
    int midPoint = 0;
    int width = 0;
    int middleDistance = 0;
    int angularWidth = 0;
    float radians = 0.0f;

    posts->count = 0;
    detect->active_object = 0;

    for (i = 0; i < SCAN_POINTS; i++)
    {
        printedObject = 0;

        start_object_if_needed(detect, scan, i, TREE_SEARCH_MAX_DIST);

        if (process_detected_object(detect, scan, i, &midPoint, &middleDistance,
                                    &angularWidth, &width, &radians))
        {
            print_object_row(msg, i * SERVO_STEP_DEG, detect->objectCount,
                             midPoint, width, middleDistance);
            printedObject = 1;

            // thin post detection
            if (is_close_fence_post(width, middleDistance) &&
                !should_ignore_alignment_post(state, midPoint))
            {
                store_fence_post(posts, midPoint, middleDistance);
            }
        }

        if (!printedObject)
        {
            print_angle_only(msg, i * SERVO_STEP_DEG, scan->distIRArray[i]);
        }
    }
}

static int handle_fence_alignment_phase(oi_t *sensor_data, BotState *state,
                                        ScanData *scan, DetectionState *detect,
                                        FencePosts *posts, char msg[])
{
    int avgAngle = 0;
    int j = 0;

    collect_alignment_posts_from_scan(state, scan, detect, posts, msg);

    if (posts->count == 0)
    {
        state->rescan = 1;
        return 1;
    }

    for (j = 0; j < posts->count; j++)
    {
        avgAngle += posts->angles[j];
    }

    avgAngle = avgAngle / posts->count;

    // First face the local fence segment
    if (avgAngle < 85)
    {
        avgAngle = 90 - avgAngle - 5;
        if(avgAngle <= 0){
            avgAngle = 0;
        }
        else{
        turn_right(sensor_data, avgAngle, 0);
        }
    }
    else if (avgAngle > 95)
    {
        avgAngle = avgAngle - 90 - 5;
        if(avgAngle <= 0){
            avgAngle = 0;
        }
        else{
        turn_left(sensor_data, avgAngle - 90, 0);
        }
    }

    // Then choose a direction to travel along the fence
    // Right turn if fence is mostly on the robot's left after centering,
    // left turn otherwise.
    if (avgAngle >= 90)
    {
//        turn_left(sensor_data, LEFT_90, 0);
//        state->fence_travel_right = 0;
        turn_right(sensor_data, RIGHT_90, 0);
        state->fence_travel_right = 1;
    }
    else
    {
        turn_right(sensor_data, RIGHT_90, 0);
        state->fence_travel_right = 1;
    }

    state->ignore_left_posts = 0;
    state->ignore_right_posts = 0;
    state->fence_aligned = 1;
    state->rescan = 1;
    return 1;
}

static void choose_best_tree(TreeTarget *target, int midPoint, int middleDistance, int width)
{
    int score = (middleDistance * 10) + abs(midPoint - 90);

    if (!target->found || score < target->best_score)
    {
        target->found = 1;
        target->angle = midPoint;
        target->dist = middleDistance;
        target->width = width;
        target->best_score = score;
    }
}

static int handle_corner_turn_if_needed(oi_t *sensor_data, BotState *state,
                                        int right_side_posts, int left_side_posts)
{
    if (right_side_posts > 0 && right_side_posts >= left_side_posts)
    {
        // Fence turned to the right. Turn toward the new fence segment.
        turn_right(sensor_data, RIGHT_90, 0);

        // After turning right, the old fence segment may still appear on the left side.
        // Ignore those old left-side posts during the next alignment scan.
        state->ignore_left_posts = 1;
        state->fence_aligned = 0;
        state->rescan = 1;
        return 1;
    }
    else if (left_side_posts > 0)
    {
        // Fence turned to the left. Turn toward the new fence segment.
        turn_left(sensor_data, LEFT_90, 0);

        // After turning left, the old fence segment may still appear on the right side.
        // Ignore those old right-side posts during the next alignment scan.
        state->ignore_left_posts = 0;
        state->ignore_right_posts = 1;
        state->fence_aligned = 0;
        state->rescan = 1;
        return 1;
    }

    return 0;
}

static int handle_tree_search_phase(oi_t *sensor_data, BotState *state,
                                    ScanData *scan, DetectionState *detect,
                                    char msg[])
{
    int i = 0;
    int printedObject = 0;
    int midPoint = 0;
    int width = 0;
    int middleDistance = 0;
    int angularWidth = 0;
    float radians = 0.0f;
    int touch = 0;
    int forward_posts = 0;
    int right_side_posts = 0;
    int left_side_posts = 0;
    TreeTarget tree = {0, 0, 0, 0, 100000};

    if (state->trees_cut >= TREE_LIMIT)
    {
        lcd_printf("5 Trees Done");
        state->rescan = 0;
        return 1;
    }

    detect->active_object = 0;

    for (i = 0; i < SCAN_POINTS; i++)
    {
        printedObject = 0;

        start_object_if_needed(detect, scan, i, TREE_SEARCH_MAX_DIST);

        if (process_detected_object(detect, scan, i, &midPoint, &middleDistance,
                                    &angularWidth, &width, &radians))
        {
            print_object_row(msg, i * SERVO_STEP_DEG, detect->objectCount,
                             midPoint, width, middleDistance);
            printedObject = 1;

            if (width > TREE_MIN_WIDTH && width < TREE_MAX_WIDTH)
            {
                choose_best_tree(&tree, midPoint, middleDistance, width);
            }
            if (is_post_width(width))
            {
                // fence post, ignore and keep scanning
                // Also watch for close side posts so the bot can turn with the fence at a corner.
                if (middleDistance < FENCE_TRACK_MAX_DIST)
                {
                    if (is_forward_post_angle(midPoint))
                    {
                        forward_posts++;
                    }
                    else if (is_right_side_post_angle(midPoint))
                    {
                        right_side_posts++;
                    }
                    else if (is_left_side_post_angle(midPoint))
                    {
                        left_side_posts++;
                    }
                }
            }
        }

        if (!printedObject)
        {
            print_angle_only(msg, i * SERVO_STEP_DEG, scan->distIRArray[i]);
        }
    }

    if (tree.found)
    {
        lcd_printf("Tree %d W:%d", state->trees_cut + 1, tree.width);
        tap_tree_and_return(sensor_data, tree.angle, tree.dist);
        state->trees_cut++;

        if (handle_corner_turn_if_needed(sensor_data, state, right_side_posts, left_side_posts))
        {
            return 1;
        }

        follow_fence_direction(sensor_data, state);
        move_forward(sensor_data, MOVE_TREE_ADVANCE_MM, 0);
        state->rescan = 1;
        return 1;
    }

    if (handle_corner_turn_if_needed(sensor_data, state, right_side_posts, left_side_posts))
    {
        return 1;
    }

    lcd_printf("No tree yet %d", state->trees_cut);
    follow_fence_direction(sensor_data, state);
    touch = move_forward(sensor_data, MOVE_SEARCH_MM, 0);

    if (touch == 1)
    {
        move_backward(sensor_data, MOVE_BUMP_BACKUP_MM, 0);
        small_follow_adjust(sensor_data, state);
    }
    else if (touch == 2)
    {
        move_backward(sensor_data, MOVE_TAPE_BACKUP_MM, 1);
//        small_follow_adjust(sensor_data, state);
        turn_right(sensor_data, RIGHT_90, 0);
        state->fence_found = 0;
        state->first_run = 1;
        state->tape_found = 0;
        state->fence_aligned = 0;
    }
    else if (touch == 3)
    {
//        avoid_drop_return_path(sensor_data);
    }

    state->rescan = 1;
    return 1;
}

int main(void)
{
    oi_t *sensor_data = oi_alloc();
    ScanData scan;
    DetectionState detect = {0, 0, 0};
    BotState state = {0, 0, 0, 1, 0, 0, 0, 1, 0, 0};
    FencePosts posts = {{0}, {0}, 0};

    char msg[UART_MSG_LEN];
    char c = '\0';

    /* Legacy variables kept because they may still be useful later. */
    int smallestWidth = 0;
    int degreeOfSmall = 0;
    int smallestObjDist = 0;
    int touched = 0;
    int endAngle[MAX_OBJECTS];
    int distObj[MAX_OBJECTS];
    int smallestObjIndex = 0;
//    int set_angle = 180;

    oi_init(sensor_data);
    oi_update(sensor_data);
    uart_init();
    timer_init();
    lcd_init();
    bno_init();
    lcd_clear();

    //New cybot scan stuff
    servo_init();
    ping_init();
    adc_init();


    //Testing purposes
//     move_forward(sensor_data, 1000);
//     oi_free(sensor_data);

    lcd_clear();

    while (((bno_getCalStatus() >> 4) & 0x03) != 3)
    {
        bno_printCalStatus();
        timer_waitMillis(300);
    }
    bno_calibrate_gyro_bias();

//    servo_move(set_angle);

    while (1)
    {
        char key = uart_receive_nonblocking(); // returns '\0' if no key
        if (key != '\0') {
            c = key;
        }

        lcd_printf("Key entered: %c", c);

        //implement exit blocking later
        if (c == 'm' || state.rescan)
        {
            detect.objectCount = 0;
            smallestWidth = 0;
            degreeOfSmall = 0;
            smallestObjDist = 0;
            smallestObjIndex = 0;
            detect.active_object = 0;
            touched = 0;
            state.rescan = 0;

            restore_scan_orientation_if_needed(sensor_data, &state);
            print_scan_header();

            if (state.first_run && !state.fence_found && !state.tape_found)
            {
                if (run_coarse_scan_with_manual_interrupt(sensor_data, &c, &scan))
                {
                    state.rescan = 1;
                    state.scan_stopped = 1;
                }
            }
            else
            {
                if (run_scan_with_manual_interrupt(sensor_data, &c, &scan))
                {
                    state.rescan = 1;
                    state.scan_stopped = 1;
                }
            }

            if (state.scan_stopped) {
                state.scan_stopped = 0;
                continue;
            }

            detect.active_object = 0; // maybe remove this

            if (state.first_run && !state.fence_found && !state.tape_found) //Find the white tape or fence post
            {
                handle_first_search_phase(sensor_data, &state, &scan, &detect, &posts, msg);
                continue;
            }

            if (state.tape_found && !state.fence_found) // on the white tape now find fence
            {
                handle_tape_to_fence_phase(sensor_data, &state, &scan, &detect, &posts, msg);
                continue;
            }

            if (state.fence_found && !state.fence_aligned) //Align to the fence
            {
                handle_fence_alignment_phase(sensor_data, &state, &scan, &detect, &posts, msg);
                continue;
            }

            if (state.fence_found && state.fence_aligned) // start tree search
            {
                handle_tree_search_phase(sensor_data, &state, &scan, &detect, msg);
                continue;
            }


        } // if m pressed

        /**
         * degreeOfSmall turn to
         * move foward the distance
         *
         */
    } //while(1) loop
}
