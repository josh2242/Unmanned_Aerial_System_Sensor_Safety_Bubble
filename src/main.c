/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * main.c - App layer application of the onboard UAS-SSB demo. The crazyflie 
 * has to have the multiranger and the flowdeck version 2.
 */

#include "source.h"

#define DEBUG_MODULE "UAS_SSB"

static State state = IDLE_STATE;
static Interrupt interrupt = NO_INTERRUPT;

void appMain()
{
  vTaskDelay(M2T(1000));

  DEBUG_PRINT("\n\nUAS-SSB main.c operational! Version alpha v4\n\n\n");
  DEBUG_PRINT("Waiting for activation...\n");

  crtpCommanderHighLevelInit();     // Initialize crtp
  
  logVarId_t idUp = logGetVarId("range", "up");         // Initialize sensor id's.
  logVarId_t idLeft = logGetVarId("range", "left");
  logVarId_t idRight = logGetVarId("range", "right");
  logVarId_t idFront = logGetVarId("range", "front");
  logVarId_t idBack = logGetVarId("range", "back");

  static bool newState = true;
  static bool landFlag = false;
  
  const float TAKEOFF_HEIGHT_M = 1.1f;

  static uint16_t Bubble_Radius = 50; // 50 mm = 5 cm

  uint64_t timeValue = 0;

  float *x_val = malloc(sizeof(float));
  float *y_val = malloc(sizeof(float));
  float *velocity_factor = malloc(sizeof(float));

  *x_val = 2.0f;
  *y_val = 0.0f;
  const float velocity = 1.0;


  while(1) // Main loop
  {
    vTaskDelay(M2T(10));

    uint16_t up = logGetUint(idUp);                     // Up TOF sensor value
    uint16_t left = logGetUint(idLeft);                 // Left TOF sensor value
    uint16_t right = logGetUint(idRight);               // Right TOF sensor value
    uint16_t front = logGetUint(idFront);               // Front TOF sensor value
    uint16_t back = logGetUint(idBack);                 // Back TOF sensor value

    switch (state)
    {
    case IDLE_STATE:
      *velocity_factor = 1.0f;
      if(isAbove(up))
      {
        state = LOWUNLOCK_STATE;
        DEBUG_PRINT("Waiting for hand to be removed!\n");
      }
      break;
    case LOWUNLOCK_STATE:
      if(isRemoved(up))
      {
        state = UNLOCKED_STATE;
      }
      break;
    case UNLOCKED_STATE:
      DEBUG_PRINT("Unlocked!\n");
      state = TAKING_OFF_STATE;
      break;
    case TAKING_OFF_STATE:
      if(newState)
      {
        DEBUG_PRINT("Takeoff initated at: %llu seconds.\n", getCurrentTimeSeconds());
        crtpCommanderHighLevelTakeoff(TAKEOFF_HEIGHT_M, 1.0f); // Height and Time in seconds
        newState = false;
      }
      if(crtpCommanderHighLevelIsTrajectoryFinished()){
        DEBUG_PRINT("Reached steady height at time %llu seconds.\n", getCurrentTimeSeconds());
        timeValue = getCurrentTimeMilliseconds() + 1000000; // Set delay of 1 second.
        state = HOVERING_STATE;
        newState = true;
      }
      break;
    case HOVERING_STATE:
      if(landFlag){
        state = LANDING_STATE;
        newState = true;
        break;
      }
      if(getCurrentTimeMilliseconds() > timeValue) // Delay for stabilization.
      {
        DEBUG_PRINT("Initial flight path started at %llu seconds.\n", getCurrentTimeSeconds());
        crtpCommanderHighLevelGoTo2(*x_val, *y_val, 0.0f, 0.0f, 2.0f, true, true); // Initial Go Forward command.
        state = FLYING_STATE;
        newState = true;
      }
      break;
    case FLYING_STATE:
      //interrupt = NO_INTERRUPT;                        // Reset Interrupt variable
      if(crtpCommanderHighLevelIsTrajectoryFinished()) // If flight path is complete, then begin executing landing sequence.
      {
        landFlag = true;
        state = HOVERING_STATE;
        newState = true;
        break;
      }
      interrupt = performSituationAnalysis(Bubble_Radius,up,front,right,back,left);
      if(interrupt > 0)                               // If an interrupt occured...
      {
        state = BRAKING_STATE;
        newState = true;
        break;
      }                          
      break;
    case BRAKING_STATE:
      if(cancelBrake(x_val, y_val,velocity,velocity_factor))  // Once braking is complete, handle interrupt.
      {
        state = INTERRUPT_STATE;
        newState = true;
        break;
      }
      break;
    case INTERRUPT_STATE:
      if(interrupt == UP_INTERRUPT) // Emergency land interrupt triggered; prepare to land.
      {
        landFlag = true;
        state = HOVERING_STATE;
        newState = true;
        break;
      }
      performSituationResponse(interrupt);
      if(interrupt > 0)                               // If an interrupt occured...
      {
        landFlag = true;
        state = HOVERING_STATE;
        newState = true;
        break;
      }   
      break;
    case LANDING_STATE:
      if(newState)
      {
        DEBUG_PRINT("Landing initated at: %llu seconds.\n", getCurrentTimeSeconds());
        crtpCommanderHighLevelLand(0.0f, 1.5f);
        newState = false;
      }
      if(crtpCommanderHighLevelIsTrajectoryFinished())
      {
        state = GROUNDED_STATE;
        newState = true;
      }
      break;
    case GROUNDED_STATE:
      if(newState)
      {
        DEBUG_PRINT("Grounded at %llu seconds!\n", getCurrentTimeSeconds());
        newState = false;
      }
      state = IDLE_STATE;
      landFlag = false;
      newState = true;
      break;
    case STOPPING_STATE:
      break;
    }
  }
  free(x_val);
  free(y_val);
  free(velocity_factor);
}

/* Required code for Emergency Stop

uint16_t up = logGetUint(idUp);                     // Up sensor value
      uint16_t up_o = radius - MIN(up, radius);
      float height = height_sp - up_o/1000.0f;
      if (height < 0.1f) {                  // Old trigger to stop and land
        state = stopping;
        DEBUG_PRINT("Stopping\n");
      }
      break;

*/