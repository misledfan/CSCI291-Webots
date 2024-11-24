#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/distance_sensor.h>
#include <webots/gps.h>
#include <webots/light_sensor.h>
#include <webots/led.h>

/Configuring the robot/
#define NUM_DIST_SENSORS 8
#define NUM_LIGHT_SENSORS 8
#define NUM_RGB_LEDS 8  
#define MAX_SPEED 6.28
#define BASE_SPEED 3.0
#define WALL_THRESHOLD 150
#define FRONT_WALL_THRESHOLD 150
#define TURN_SPEED 2.0
#define GPS_ACCURACY 0.15
#define LIGHT_SENSITIVITY 0.15
#define MIN_EXPLORE_DURATION 5000  // Minimum exploration duration in simulation steps

// Robot state definitions
typedef enum {
    INIT_GPS,
    FOLLOW_WALL,
    ROTATE,
    LOCATE_LIGHT,
    HALT
} RobotStatus;

/*Global Variables*/
WbDeviceTag distance_sensors[NUM_DIST_SENSORS];
WbDeviceTag light_sensors[NUM_LIGHT_SENSORS];
WbDeviceTag left_motor, right_motor;
WbDeviceTag gps;
WbDeviceTag rgb_leds[NUM_RGB_LEDS];  

/*Variables to hold the current state, GPS position, and light tracking*/
RobotStatus current_status = INIT_GPS;
double max_light_intensity = -INFINITY;
double starting_position[3] = {0};
double light_location[3] = {0};
int gps_attempts = 0;
int exploration_duration = 0;  /Tracks the exploration time/

/* Initialize robot sensors and devices */
void initialize_robot() {
    wb_robot_init();
    int time_step = wb_robot_get_basic_time_step();

    // Initialize distance sensors
    for (int i = 0; i < NUM_DIST_SENSORS; i++) {
        char sensor_name[4];
        sprintf(sensor_name, "ps%d", i);
        distance_sensors[i] = wb_robot_get_device(sensor_name);
        wb_distance_sensor_enable(distance_sensors[i], time_step);
    }

    // Initialize light sensors
    for (int i = 0; i < NUM_LIGHT_SENSORS; i++) {
        char sensor_name[4];
        sprintf(sensor_name, "ls%d", i);
        light_sensors[i] = wb_robot_get_device(sensor_name);
        wb_light_sensor_enable(light_sensors[i], time_step);
    }

    // Initialize motors
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);

    // Initialize GPS
    gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, time_step);

    // Initialize all LEDs
    for (int i = 0; i < NUM_RGB_LEDS; i++) {
        char led_name[5];
        sprintf(led_name, "led%d", i);
        rgb_leds[i] = wb_robot_get_device(led_name);
    }
}

/* Turn off all LEDs */
void deactivate_leds() {
    for (int i = 0; i < NUM_RGB_LEDS; i++) {
        wb_led_set(rgb_leds[i], 0);
    }
}

/* Turn on all LEDs */
void activate_leds() {
    for (int i = 0; i < NUM_RGB_LEDS; i++) {
        wb_led_set(rgb_leds[i], 1);
    }
}

/* Check if the GPS reading is valid */
int is_valid_gps_reading() {
    const double *gps_values = wb_gps_get_values(gps);
    return (gps_values[0] == gps_values[0]) && // Check for NaN
           (gps_values[1] == gps_values[1]) && 
           (gps_values[0] != 0.0) && 
           (gps_values[1] != 0.0);
}

/* Update light sensor readings and track the brightest spot */
void update_light_info() {
    double total_light = 0;
    for (int i = 0; i < NUM_LIGHT_SENSORS; i++) {
        total_light += wb_light_sensor_get_value(light_sensors[i]);
    }
    total_light /= NUM_LIGHT_SENSORS;

    const double *gps_values = wb_gps_get_values(gps);

    if (total_light > max_light_intensity) {
        max_light_intensity = total_light;
        memcpy(light_location, gps_values, 3 * sizeof(double));
        printf("New brightest location: %.2f with coordinates (%.2f, %.2f)\n", 
               max_light_intensity, light_location[0], light_location[1]);
    }
}

/* Check if the robot is near its starting position */
int is_near_start_position() {
    const double *current_pos = wb_gps_get_values(gps);
    double delta_x = current_pos[0] - starting_position[0];
    double delta_y = current_pos[1] - starting_position[1];
    return sqrt(delta_x*delta_x + delta_y*delta_y) < GPS_ACCURACY;
}

/* Wall following behavior */
void follow_wall_behavior(double *sensor_readings, double *left_speed, double *right_speed) {
    double left_front = sensor_readings[7];
    double front = sensor_readings[0];
    double right_front = sensor_readings[1];
    double right_side = sensor_readings[2];

    // Obstacle detection logic
    if (front > FRONT_WALL_THRESHOLD || left_front > FRONT_WALL_THRESHOLD || right_front > FRONT_WALL_THRESHOLD) {
        *left_speed = -TURN_SPEED;
        *right_speed = TURN_SPEED;
        return;
    }

    // Wall following logic based on distance sensor readings
    double error = WALL_THRESHOLD - right_side;
    
    if (error > 50) {
        *left_speed = BASE_SPEED;
        *right_speed = BASE_SPEED * 0.5;
    } else if (error < -50) {
        *left_speed = BASE_SPEED * 0.5;
        *right_speed = BASE_SPEED;
    } else {
        *left_speed = BASE_SPEED;
        *right_speed = BASE_SPEED;
    }
}

/* Main program loop */
int main() {
    initialize_robot();
    int time_step = wb_robot_get_basic_time_step();

    // Turn off all LEDs at the start
    deactivate_leds();

    // Main control loop
    while (wb_robot_step(time_step) != -1) {
        double left_speed = 0, right_speed = 0;
        double sensor_readings[NUM_DIST_SENSORS];
        
        // Read distance sensor values
        for (int i = 0; i < NUM_DIST_SENSORS; i++) {
            sensor_readings[i] = wb_distance_sensor_get_value(distance_sensors[i]);
        }

        // State machine
        switch (current_status) {
            case INIT_GPS:
                if (is_valid_gps_reading()) {
                    const double *initial_pos = wb_gps_get_values(gps);
                    memcpy(starting_position, initial_pos, 3 * sizeof(double));
                    printf("Initial position set to: (%.2f, %.2f)\n", starting_position[0], starting_position[1]);
                    current_status = FOLLOW_WALL;
                } else {
                    gps_attempts++;
                    printf("Waiting for valid GPS reading... (Attempt %d)\n", gps_attempts);
                    left_speed = BASE_SPEED;
                    right_speed = BASE_SPEED;
                }
                break;

            case FOLLOW_WALL:
                update_light_info();
                follow_wall_behavior(sensor_readings, &left_speed, &right_speed);
                exploration_duration++;
                if (is_near_start_position() && exploration_duration >= MIN_EXPLORE_DURATION) {
                    printf("Exploration completed. Now locating brightest spot...\n");
                    current_status = LOCATE_LIGHT;
                    exploration_duration = 0;
                }
                break;

            case LOCATE_LIGHT:
                update_light_info();
                follow_wall_behavior(sensor_readings, &left_speed, &right_speed);
                const double *current_pos = wb_gps_get_values(gps);
                double delta_x = current_pos[0] - light_location[0];
                double delta_y = current_pos[1] - light_location[1];
                if (sqrt(delta_x*delta_x + delta_y*delta_y) < LIGHT_SENSITIVITY) {
                    printf("Reached the brightest spot at (%.2f, %.2f)!\n", 
                           light_location[0], light_location[1]);
                    activate_leds();  // Turn on all LEDs
                    printf("Leds have turned on \n");
                    current_status = HALT;  // Transition to HALT state after reaching the brightest spot
                }
                break;

            case HALT:
                left_speed = 0;
                right_speed = 0;
                if (exploration_duration == 0) {
                    printf("Robot has successfully reached the brightest spot and is now halted.\n");
                    exploration_duration++;  // Set to non-zero to prevent re-entering HALT state
                }
                break;
        }

        wb_motor_set_velocity(left_motor, left_speed);
        wb_motor_set_velocity(right_motor, right_speed);
    }

    wb_robot_cleanup();
    return 0;
}
