#include "source.h"

// Returns truth value of whether something is above the top sensor.
bool isAbove(uint16_t up_sensor_value)
{
    uint16_t unlockThLow = 100;
    bool val = false; 
    if(up_sensor_value < unlockThLow && up_sensor_value > 0.001f)
    {
        val = true;
    }
    return val;
}

// Returns truth value of wheather the thing above the sensor is now gone.
bool isRemoved(uint16_t up_sensor_value)
{
    uint16_t unlockThHigh = 300;
    bool val = false;
    if(up_sensor_value > unlockThHigh)
    {
        val = true;
    }
    return val;
}

// Returns current time in seconds as uint64_t variable.
uint64_t getCurrentTimeSeconds(void)
{
    uint64_t cTime = (usecTimestamp()/1000000); // Current time variable; 1 second = 1,000,000 microseconds.
    return cTime;
}

// Returns current time in milliseconds as uint64_t variable.
uint64_t getCurrentTimeMilliseconds(void)
{
    uint64_t cTime = usecTimestamp(); // Current time variable in milliseconds.
    return cTime;
}

// Using the Multiranger deck, and a sensing radius, this function returns which sensor is
// reading a value is reading an object below the set radius, if applicable.
Interrupt performSituationAnalysis(uint16_t bubble_radius, uint16_t up_sensor_value, uint16_t forward_sensor_value, uint16_t right_sensor_value, uint16_t backward_sensor_value, uint16_t left_sensor_value)
{
    Interrupt val = NO_INTERRUPT;
    while(1)
    {
        if(up_sensor_value < bubble_radius){val = UP_INTERRUPT;}
        if(forward_sensor_value < bubble_radius){val = FORWARD_INTERRUPT;}
        if(right_sensor_value < bubble_radius){val = RIGHT_INTERRUPT;}
        if(backward_sensor_value < bubble_radius){val = BACKWARD_INTERRUPT;}
        if(left_sensor_value < bubble_radius){val = LEFT_INTERRUPT;}
        break;
    }
    return val;
}

// Given an interrupt, executes appropriate flight commands in response.
void performSituationResponse(Interrupt current_interrupt)
{
    switch (current_interrupt)
    {
    case FORWARD_INTERRUPT:
        DEBUG_PRINT("FORWARD_INTERRUPT\n");
        break;
    case RIGHT_INTERRUPT:
        DEBUG_PRINT("RIGHT_INTERRUPT\n");
        break;
    case BACKWARD_INTERRUPT:
        DEBUG_PRINT("BACKWARD_INTERRUPT\n");
        break;
    case LEFT_INTERRUPT:
        DEBUG_PRINT("LEFT_INTERRUPT\n");
        break;
    case NO_INTERRUPT:
        break;
    case UP_INTERRUPT:
        break;
    }
}

// Cancels current trajectory and eases CrazyFlie to a complete stop.
// Returns true on completion.
bool cancelBrake(float *x_pointer, float *y_pointer, float current_velocity, float *velocity_factor)
{
    bool val = false;
    bool *func_complete = malloc(sizeof(bool));
    float time = 1.0f/8.0f;
    float magnitude = sqrt(((*x_pointer)*(*x_pointer) )+((*y_pointer)*(*y_pointer)));
    if(*velocity_factor == 1.0f)
    {
        crtpCommanderHighLevelDisable(); // Disable currently trajctory
        DEBUG_PRINT("CANCELBRAKE() Trajectory Disabled.\n");
    }
    if(*func_complete == false)
    {
        *velocity_factor -= 0.25f;            // Reduce by 25%
    }
    if(*velocity_factor < 0.1f)
    {
        if(crtpCommanderHighLevelIsTrajectoryFinished())
        {
            val = true;
            free(func_complete);
            DEBUG_PRINT("CANCELBRAKE() Brake achieved.\n");
        }
    }
    else if (crtpCommanderHighLevelIsTrajectoryFinished() || *velocity_factor == 0.75f || *func_complete == false)
    {
        time = magnitude / (current_velocity * (*velocity_factor));
        DEBUG_PRINT("CANCELBRAKE() Current velocity factor: %f.\n", (double)(*velocity_factor));
        crtpCommanderHighLevelGoTo2(*x_pointer, *y_pointer, 0.0f, 0.0f, time, true, true);
    }
    return val; 
}

bool cancelBrake(float *x_pointer, float *y_pointer, float current_velocity, float *velocity_factor)
{
    bool val = false;
    if(*velocity_factor == 1.0f)
    {
        crtpCommanderHighLevelDisable(); // Disable currently trajctory
        DEBUG_PRINT("CANCELBRAKE() Trajectory Disabled.\n");
        *velocity_factor -= 0.25f;            // Reduce by 25%
    }
    return val; 
}