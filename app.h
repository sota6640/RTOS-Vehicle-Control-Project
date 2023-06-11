/***************************************************************************//**
 * @file
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#ifndef APP_H
#define APP_H


#include <stdlib.h>
#include "em_cmu.h"
#include "gpio.h"
#include "capsense.h"
/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void app_init(void);


void SPEED_SETPOINT_TaskCreate(void);
void VEHICLE_DIRECTION_TaskCreate(void);
void VEHICLE_MONITOR_TaskCreate(void);
void LED_OUTPUT_TaskCreate(void);
void LCD_DISPLAY_TaskCreate(void);
void IDLE_TaskCreate(void);

void ITC_func(void);

//Part I.1 Copied interrupt/GPIO setup as needed.
uint32_t button0_test(void);
uint32_t button1_test(void);
void capsense_main(void);

void drive_interrupts(void);
void initialize_data_str(void);
//Part I.1 Copied interrupt/GPIO setup as needed.

#endif  // APP_H
