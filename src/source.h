// source.h header file

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

#include "app.h"

#include "commander.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"

#include "log.h"
#include "param.h"

#include "usec_time.h"    // For timer use

#include "crtp_commander_high_level.h"

#define MAX(a,b) ((a>b)?a:b)
#define MIN(a,b) ((a<b)?a:b)

typedef enum {
    IDLE_STATE,
    LOWUNLOCK_STATE,
    UNLOCKED_STATE,
    TAKING_OFF_STATE,
    HOVERING_STATE,
    FLYING_STATE,
    BRAKING_STATE,
    INTERRUPT_STATE,
    LANDING_STATE,
    GROUNDED_STATE,
    STOPPING_STATE
} State;

typedef enum {
    NO_INTERRUPT,
    FORWARD_INTERRUPT,
    BACKWARD_INTERRUPT,
    LEFT_INTERRUPT,
    RIGHT_INTERRUPT,
    // GEOFENCE_INTERUPT,
    UP_INTERRUPT
} Interrupt; 

bool isAbove(uint16_t up_sensor_value);

bool isRemoved(uint16_t up_sensor_value);

uint64_t getCurrentTimeSeconds(void);

uint64_t getCurrentTimeMilliseconds(void);

Interrupt performSituationAnalysis(uint16_t bubble_radius, uint16_t up_sensor_value, uint16_t forward_sensor_value, uint16_t right_sensor_value, uint16_t backward_sensor_value, uint16_t left_sensor_value);

void performSituationResponse(Interrupt current_interrupt);

bool cancelBrake(float *x_pointer, float *y_pointer, float current_velocity, float *velocity_factor);