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

/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/

#include <gpio.h>
#include <capsense.h>
#include "app.h"
#include "sl_board_control.h"
#include "em_assert.h"
#include "glib.h"
#include "dmd.h"
#include "os.h"
#include "fifo.h"

static GLIB_Context_t glibContext;


#define  SPEED_SETPOINT_STK_SIZE        512u
#define  SPEED_SETPOINT_TASK_PRIORITY    1
OS_TCB   App_Speed_SetpointTaskTCB;
CPU_STK  App_Speed_SetpointTaskStk[SPEED_SETPOINT_STK_SIZE];

#define  VEHICLE_DIRECTION_STK_SIZE     512u
#define  VEHICLE_DIRECTION_TASK_PRIORITY    1
OS_TCB   App_Vehicle_DirectionTaskTCB;
CPU_STK  App_Vehicle_DirectionTaskStk[VEHICLE_DIRECTION_STK_SIZE];

#define  VEHICLE_MONITOR_STK_SIZE       512u
#define  VEHICLE_MONITOR_TASK_PRIORITY    1
OS_TCB   App_Vehicle_MonitorTaskTCB;
CPU_STK  App_Vehicle_MonitorTaskStk[VEHICLE_MONITOR_STK_SIZE];

#define  LED_OUTPUT_STK_SIZE            512u
#define  LED_OUTPUT_TASK_PRIORITY          1
OS_TCB   App_LedOutputTaskTCB;
CPU_STK  App_LedOutputTaskStk[LED_OUTPUT_STK_SIZE];

#define  LCD_DISPLAY_STK_SIZE            512u
#define  LCD_DISPLAY_TASK_PRIORITY        1
OS_TCB   App_LcdDisplayTaskTCB;
CPU_STK  App_LcdDisplayTaskStk[LCD_DISPLAY_STK_SIZE];

//#define IDLE_TASK_TASK_STK_SIZE          512u
//#define IDLE_TASK_TASK_PRIORITY          20
//OS_TCB   App_IdleTaskTCB;
//CPU_STK  App_IdleTaskStk[IDLE_TASK_TASK_STK_SIZE];

//OS Object Instantiations. Procedure Part I , # 5
static OS_FLAG_GRP event_flag_speed_direction;
static OS_FLAG_GRP event_flag_led;
static OS_SEM semaphore;
static OS_MUTEX mutex_speed;
static OS_MUTEX mutex_direction;

static OS_TMR App_Timer;
//OS Object Instantiations


//Data Structure Implementation

// Instantiate a FIFO data structure for each of the buttons.
static struct node_t* button0;
static struct node_t* button1;
// Instantiate a FIFO data structure for each of the buttons.

//Instantiate one instance of Vehicle Speed Data
static struct speed_t speed;
//Instantiate one instance of Vehicle Speed Data

//Instantiate one instance of Vehicle Direction Data
static struct direction_t direction;
//Instantiate one instance of Vehicle Direction Data

//Data Structure Implementation

uint32_t button0_state = 1; //1 is button not pressed, 0 is pressed, through debugger
uint32_t button1_state = 1; //1 is button not pressed, 0 is pressed, through debugger
uint32_t button0_test_result = 1; //result of button0, was in main.c for Lab2
uint32_t button1_test_result = 1; //result of button1, was in main.c for Lab2

static bool area0;
static bool area1;
static bool area2;
static bool area3;




//might not even need this
enum flag_event { //Procedure Part I #4
  speed_update =     0b00000001, //1
  direction_update = 0b00000010, //2
  speed_limit =      0b00000100, //4
  direction_limit=   0b00001000, //8
  no_operation =     0b00010000, //16
  clear_direction=   0b00100000, //32
  clear_speed=       0b01000000  //64
};

//might not even need this
enum direction_event {
  no_direction=      0b00000001, //Capsense not touched enum 1
  hard_left=         0b00000010, //0th touched enum 2
  hard_right=        0b00000100, //3rd touched enum 4
  soft_left=         0b00001000, //1st touched enum 8
  soft_right=        0b00010000  //2nd touched enum 16
};


//LOCAL FUNCTIONS DECLARATIONS, NO NEED FOR THESE IN THE APP.H FILE
//LOCAL FUNCTIONS DECLARATIONS, NO NEED FOR THESE IN THE APP.H FILE
//LOCAL FUNCTIONS DECLARATIONS, NO NEED FOR THESE IN THE APP.H FILE
void SPEED_SETPOINT_TASK (void  *p_arg);
void VEHICLE_DIRECTION_TASK(void *p_arg);
void VEHICLE_MONITOR_TASK(void *p_arg);
void LED_OUTPUT_TASK(void *p_arg);
void LCD_DISPLAY_TASK(void *p_arg);
//void IDLE_TASK(void *p_arg);

void App_TimerCallback (void  *p_tmr, void  *p_arg);


void App_TimerCallback (void  *p_tmr, void  *p_arg) {
  RTOS_ERR err;
  (void)&p_tmr; //unused parameters
  (void)&p_arg; //unused parameters


  OSFlagPost (&event_flag_led,
              direction_limit, // direction
              OS_OPT_POST_FLAG_SET,
            &err); //direction violation, post to the LED output indicating a direction violation


}

void initialize_timer() {
  RTOS_ERR err;

  OSTmrStart(&App_Timer, &err); //error referring to that the AppTimer has not been created.

  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
}

//LOCAL FUNCTIONS DECLARATIONS, NO NEED FOR THESE IN THE APP.H FILE
//LOCAL FUNCTIONS DECLARATIONS, NO NEED FOR THESE IN THE APP.H FILE
//LOCAL FUNCTIONS DECLARATIONS, NO NEED FOR THESE IN THE APP.H FILE

static void LCD_init() // LCD Display initialization function
//initializes the variables to draw on the screen
{
  uint32_t status;
  /* Enable the memory lcd */
  status = sl_board_enable_display();
  EFM_ASSERT(status == SL_STATUS_OK);

  /* Initialize the DMD support for memory lcd display */
  status = DMD_init(0);
  EFM_ASSERT(status == DMD_OK);

  /* Initialize the glib context */
  status = GLIB_contextInit(&glibContext);
  EFM_ASSERT(status == GLIB_OK);

  glibContext.backgroundColor = White;
  glibContext.foregroundColor = Black;

  /* Fill lcd with background color */
  GLIB_clear(&glibContext);

  /* Use Normal font */
  GLIB_setFont(&glibContext, (GLIB_Font_t *) &GLIB_FontNormal8x8);

//GLIB_drawStringOnLine(&glibContext,
                    //    "Welcome to...\n**Lab 7**!",
                     //   0,
                    //    GLIB_ALIGN_LEFT,
                     //   5,
                    //    5,
                     //   true);


  //GLIB_drawStringOnLine(&glibContext,
        //                "Review the lab\ninstructions\n SONAL TAMRAKAR!",
          //              2,
          //              GLIB_ALIGN_LEFT,
           //             5,
           //             5,
                 //       true);
  /* Post updates to display */
  DMD_updateDisplay();
}

void app_init(void)
{
  // Initialize GPIO
  gpio_open();
  // Initialize GPIO


  //interrupt driven implementation from Lab2
  drive_interrupts();
  //interrupt driven implementation from Lab2

  // Initialize our capacitive touch sensor driver!
  CAPSENSE_Init();
  // Initialize our capacitive touch sensor driver!

  initialize_data_str();

  // Initialize our LCD system
  LCD_init();
  // Initialize our LCD system

  // TODO: Call your task creation functions.
  SPEED_SETPOINT_TaskCreate();
  VEHICLE_DIRECTION_TaskCreate();
  VEHICLE_MONITOR_TaskCreate();
  LED_OUTPUT_TaskCreate();
  LCD_DISPLAY_TaskCreate();
  //IDLE_TaskCreate();


//intertask communication function
    ITC_func();
//intertask communication function

}



void initialize_data_str(void) {
  button0 = NULL;
  button1 = NULL;

  speed.current_speed = 0;
  direction.current_direction = no_direction;

}

/***************************************************************************//**
 * @brief
 * Initiation of the data structure
 * @details
 * Micrium OS functions to create the periodic timer, semaphore,
 * event flag and message queue.
 * @note
 *
 ******************************************************************************/
void ITC_func(void) {
  //create the timer
  RTOS_ERR err;

  /* Creates a flag.*/
  OSFlagCreate(&event_flag_speed_direction,              /*   Pointer to user-allocated event flag.         */
                     "Event flag for Speed and Direction",             /*   Name used for debugging.                      */
                      0,                      /*   Initial flags, all cleared.                   */
                     &err);

  /* Creates a flag.*/
  OSFlagCreate(&event_flag_led,              /*   Pointer to user-allocated event flag.         */
                       "Event flag for LED",             /*   Name used for debugging.                      */
                        0,                      /*   Initial flags, all cleared.                   */
                       &err);

  /* Create a semaphore.*/
  OSSemCreate(&semaphore, /*    Pointer to user-allocated semaphore. */
              "Semaphore for Speed Setpoint Task", /* Name Used for debugging */
               0, /* initial count : available in this case */
               &err);

  /* Create a mutex. */
  OSMutexCreate (&mutex_speed,  /*    Pointer to user-allocated mutex. */
                   "Mutex_speed",
                   &err);

  /* Create a mutex. */
  OSMutexCreate (&mutex_direction, /*    Pointer to user-allocated mutex. */
                   "Mutex_direction",
                   &err);

  OSTmrCreate(&App_Timer,   /*   Pointer to user-allocated timer.     */
                  "App Timer for Capsense", /*  Name used for debugging.   */
                  0, /* 0 iniital delay */
                  50,   /* 100 Timer Ticks period, might be higher */
                  OS_OPT_TMR_PERIODIC, /* Timer is periodic */
                  &App_TimerCallback, /*Called when timer expires*/
                  DEF_NULL, /* no arguments to callback*/
                  &err);

  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

}

/***************************************************************************//**
 * @brief
 * Task Creation for LED_OUTPUT Task
 * @details
 *  Calls the OSTaskCreate with the pointer to the task's TCB, task name, task priority,
 *  pointer to the task's argument, pointer to base of stack, stack limit, stack size,
 *  messages in task queue, Round-Robin time quanta, external TCB data and task options.
 * @note
 *
 ******************************************************************************/
void  SPEED_SETPOINT_TaskCreate(void)
{
    RTOS_ERR     err;

    OSTaskCreate(&App_Speed_SetpointTaskTCB,                /* Pointer to the task's TCB.  */
                 "Inside Speed Setpoint Task",                    /* Name to help debugging.     */
                 &SPEED_SETPOINT_TASK,                   /* Pointer to the task's code. */
                  DEF_NULL,                          /* Pointer to task's argument. */
                  SPEED_SETPOINT_TASK_PRIORITY,             /* Task's priority.            */
                 &App_Speed_SetpointTaskStk[0],             /* Pointer to base of stack.   */
                 (SPEED_SETPOINT_STK_SIZE / 10u),  /* Stack limit, from base.     */
                 SPEED_SETPOINT_STK_SIZE,         /* Stack size, in CPU_STK.     */
                  0u,                               /* Messages in task queue.     */
                  10u,                                /* Round-Robin time quanta.    */
                  DEF_NULL,                          /* External TCB data.          */
                  OS_OPT_TASK_STK_CLR,               /* Task options.               */
                 &err);
    if (err.Code != RTOS_ERR_NONE) { //CAN LIKEWISE USE AN ASSERT STATEMENT AS WELL
        /* Handle error on task creation. */
    }
}

/***************************************************************************//**
 * @brief
 * Task Creation for LED_OUTPUT Task
 * @details
 *  Calls the OSTaskCreate with the pointer to the task's TCB, task name, task priority,
 *  pointer to the task's argument, pointer to base of stack, stack limit, stack size,
 *  messages in task queue, Round-Robin time quanta, external TCB data and task options.
 * @note
 *
 ******************************************************************************/
void  VEHICLE_DIRECTION_TaskCreate(void)
{
    RTOS_ERR     err;

    OSTaskCreate(&App_Vehicle_DirectionTaskTCB,                /* Pointer to the task's TCB.  */
                 "Inside Vehicle Direction Task",                    /* Name to help debugging.     */
                 &VEHICLE_DIRECTION_TASK,                   /* Pointer to the task's code. */
                  DEF_NULL,                          /* Pointer to task's argument. */
                  VEHICLE_DIRECTION_TASK_PRIORITY,             /* Task's priority.            */
                 &App_Vehicle_DirectionTaskStk[0],             /* Pointer to base of stack.   */
                 (VEHICLE_DIRECTION_STK_SIZE / 10u),  /* Stack limit, from base.     */
                 VEHICLE_DIRECTION_STK_SIZE,         /* Stack size, in CPU_STK.     */
                  0u,                               /* Messages in task queue.     */
                  10u,                                /* Round-Robin time quanta.    */
                  DEF_NULL,                          /* External TCB data.          */
                  OS_OPT_TASK_STK_CLR,               /* Task options.               */
                 &err);
    if (err.Code != RTOS_ERR_NONE) { //CAN LIKEWISE USE AN ASSERT STATEMENT AS WELL
        /* Handle error on task creation. */
    }
}


/***************************************************************************//**
 * @brief
 * Task Creation for LED_OUTPUT Task
 * @details
 *  Calls the OSTaskCreate with the pointer to the task's TCB, task name, task priority,
 *  pointer to the task's argument, pointer to base of stack, stack limit, stack size,
 *  messages in task queue, Round-Robin time quanta, external TCB data and task options.
 * @note
 *
 ******************************************************************************/
void  VEHICLE_MONITOR_TaskCreate(void)
{
    RTOS_ERR     err;

    OSTaskCreate(&App_Vehicle_MonitorTaskTCB,                /* Pointer to the task's TCB.  */
                 "Inside Vehicle Monitor Task",                    /* Name to help debugging.     */
                 &VEHICLE_MONITOR_TASK,                   /* Pointer to the task's code. */
                  DEF_NULL,                          /* Pointer to task's argument. */
                  VEHICLE_MONITOR_TASK_PRIORITY,             /* Task's priority.            */
                 &App_Vehicle_MonitorTaskStk[0],             /* Pointer to base of stack.   */
                 (VEHICLE_MONITOR_STK_SIZE / 10u),  /* Stack limit, from base.     */
                 VEHICLE_MONITOR_STK_SIZE,         /* Stack size, in CPU_STK.     */
                  0u,                               /* Messages in task queue.     */
                  10u,                                /* Round-Robin time quanta.    */
                  DEF_NULL,                          /* External TCB data.          */
                  OS_OPT_TASK_STK_CLR,               /* Task options.               */
                 &err);
    if (err.Code != RTOS_ERR_NONE) { //CAN LIKEWISE USE AN ASSERT STATEMENT AS WELL
        /* Handle error on task creation. */
    }
}

/***************************************************************************//**
 * @brief
 * Task Creation for LED_OUTPUT Task
 * @details
 *  Calls the OSTaskCreate with the pointer to the task's TCB, task name, task priority,
 *  pointer to the task's argument, pointer to base of stack, stack limit, stack size,
 *  messages in task queue, Round-Robin time quanta, external TCB data and task options.
 * @note
 *
 ******************************************************************************/
void  LED_OUTPUT_TaskCreate(void)
{
    RTOS_ERR     err;

    OSTaskCreate(&App_LedOutputTaskTCB,                /* Pointer to the task's TCB.  */
                 "Inside LED Output Task",                    /* Name to help debugging.     */
                 &LED_OUTPUT_TASK,                   /* Pointer to the task's code. */
                  DEF_NULL,                          /* Pointer to task's argument. */
                  LED_OUTPUT_TASK_PRIORITY,             /* Task's priority.            */
                 &App_LedOutputTaskStk[0],             /* Pointer to base of stack.   */
                 (LED_OUTPUT_STK_SIZE / 10u),  /* Stack limit, from base.     */
                 LED_OUTPUT_STK_SIZE,         /* Stack size, in CPU_STK.     */
                  0u,                               /* Messages in task queue.     */
                  10u,                                /* Round-Robin time quanta.    */
                  DEF_NULL,                          /* External TCB data.          */
                  OS_OPT_TASK_STK_CLR,               /* Task options.               */
                 &err);
    if (err.Code != RTOS_ERR_NONE) { //CAN LIKEWISE USE AN ASSERT STATEMENT AS WELL
        /* Handle error on task creation. */
    }
}


/***************************************************************************//**
 * @brief
 * Task Creation for LED_OUTPUT Task
 * @details
 *  Calls the OSTaskCreate with the pointer to the task's TCB, task name, task priority,
 *  pointer to the task's argument, pointer to base of stack, stack limit, stack size,
 *  messages in task queue, Round-Robin time quanta, external TCB data and task options.
 * @note
 *
 ******************************************************************************/
void  LCD_DISPLAY_TaskCreate(void)
{
    RTOS_ERR     err;

    OSTaskCreate(&App_LcdDisplayTaskTCB,                /* Pointer to the task's TCB.  */
                 "Inside LCD Display Task",                    /* Name to help debugging.     */
                 &LCD_DISPLAY_TASK,                   /* Pointer to the task's code. */
                  DEF_NULL,                          /* Pointer to task's argument. */
                  LCD_DISPLAY_TASK_PRIORITY,             /* Task's priority.            */
                 &App_LcdDisplayTaskStk[0],             /* Pointer to base of stack.   */
                 (LCD_DISPLAY_STK_SIZE / 10u),  /* Stack limit, from base.     */
                 LCD_DISPLAY_STK_SIZE,         /* Stack size, in CPU_STK.     */
                  0u,                               /* Messages in task queue.     */
                  10u,                                /* Round-Robin time quanta.    */
                  DEF_NULL,                          /* External TCB data.          */
                  OS_OPT_TASK_STK_CLR,               /* Task options.               */
                 &err);
    if (err.Code != RTOS_ERR_NONE) { //CAN LIKEWISE USE AN ASSERT STATEMENT AS WELL
        /* Handle error on task creation. */
    }
}




/***************************************************************************//**
 * @brief
 * Task Creation for IDLE Task
 * @details
 *  Calls the OSTaskCreate with the pointer to the task's TCB, task name, task priority,
 *  pointer to the task's argument, pointer to base of stack, stack limit, stack size,
 *  messages in task queue, Round-Robin time quanta, external TCB data and task options.
 * @note
 *
 ******************************************************************************/


void SPEED_SETPOINT_TASK (void  *p_arg) {
  (void)&p_arg;
  RTOS_ERR err;

  while (1) {
      OSSemPend(&semaphore,        /* Pointer to user-allocated semaphore.    */
                        0,                 /* Wait for a maximum of 1000 OS Ticks.    */
                        OS_OPT_PEND_BLOCKING, /* Task will block.                        */
                        DEF_NULL,             /* Timestamp is not used.                  */
                        &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

      //TODO: READ EACH FIFO
      struct node_t* got_button0 = peek(button0); //accessing the fifo
      pop(&button0);
      struct node_t* got_button1 = peek(button1); //accessing the fifo
      pop(&button1);

      if (got_button0->btn == 0) {
          OSMutexPend (&mutex_speed,
                        0,
                        OS_OPT_PEND_BLOCKING,
                        NULL,
                         &err);
        speed.current_speed = speed.current_speed + 5;

          OSMutexPost (&mutex_speed,
                        OS_OPT_POST_NONE,
                        &err);
      }

      if (got_button1->btn == 0) {
          OSMutexPend (&mutex_speed,
                       0,
                       OS_OPT_PEND_BLOCKING,
                       NULL,
                       &err);
          if (speed.current_speed == 0) {speed.current_speed = 0;}
          else {
              speed.current_speed = speed.current_speed - 5;
          }

          OSMutexPost (&mutex_speed,
                       OS_OPT_POST_NONE,
                       &err);
            }

      //TODO: POST THE EVENT TO THE VEHICLE MONITOR TASK
      OSFlagPost (&event_flag_speed_direction,
                        speed_update, //enum speed_update = 1
                  OS_OPT_POST_FLAG_SET,
                            &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  }


}

void VEHICLE_DIRECTION_TASK(void *p_arg) {
  (void)&p_arg;
  RTOS_ERR err;
  initialize_timer(); // START THE TIMER

  while(1) {

  OSTimeDly(10, OS_OPT_TIME_DLY, &err);



  //TODO: READ CAPSENSE HERE
  capsense_main();



  //TODO: Post the event to the Vehicle Monitor Task
  OSFlagPost (&event_flag_speed_direction,
                      direction_update, //enum direction_update = 2
              OS_OPT_POST_FLAG_SET,
                      &err);

           EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  }
}

void VEHICLE_MONITOR_TASK(void *p_arg) {
  (void)&p_arg;
  RTOS_ERR err;
  int get_speed = 0;
  uint32_t get_direction = no_direction;

  while(1)
    {
  OSFlagPend (&event_flag_speed_direction, // task waiting
                      speed_update | direction_update,
                      0,
                      OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME, //button 0 ot button 1
                      NULL,
                      &err);
       EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

       //TODO: UPON CHANGE TO THE SPEED SETPOINT DATA
       OSMutexPend (&mutex_speed,
                              0,
                              OS_OPT_PEND_BLOCKING,
                              NULL,
                              &err);
       get_speed = speed.current_speed;

       OSMutexPost (&mutex_speed,
                    OS_OPT_POST_NONE,
                    &err);
       //TODO: UPON CHANGE TO THE SPEED SETPOINT DATA


       //TODO: UPON CHANGE TO THE VEHICLE DIRECTION DATA
       OSMutexPend (&mutex_direction,
                                     0,
                                     OS_OPT_PEND_BLOCKING,
                                     NULL,
                                     &err);
       get_direction = direction.current_direction;

       OSMutexPost (&mutex_direction,
                                     OS_OPT_POST_NONE,
                                     &err);
       //TODO: UPON CHANGE TO THE VEHICLE DIRECTION DATA


       if ((get_speed > 45 && get_direction != no_direction) || get_speed > 75 ) {
           //Inside Speed Violation ii
           OSFlagPost (&event_flag_led,
                       speed_limit,
                       OS_OPT_POST_FLAG_SET,
                       &err);
         }
       else {

           OSFlagPost (&event_flag_led,
                         no_operation,
                         OS_OPT_POST_FLAG_SET,
                         &err);
           EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
       }


       }

       //TODO: VIOLATIONS VIOLATIONS VIOLATIONS VIOLATIONS VIOLATIONS VIOLATIONS VIOLATIONS


}

void LED_OUTPUT_TASK(void *p_arg) {
  (void)&p_arg;
  RTOS_ERR err;
  OS_FLAGS all_flags;
  while(1) {
      all_flags = OSFlagPend (&event_flag_led,
                             speed_limit|no_operation| clear_speed | clear_direction | direction_limit,
                            0,
                            OS_OPT_PEND_FLAG_SET_ANY + OS_OPT_PEND_FLAG_CONSUME,
                            NULL,
                            &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
      if (all_flags & direction_limit) {
                GPIO_PinOutSet(LED1_port, LED1_pin);
          }


      else if (all_flags & speed_limit) {
          GPIO_PinOutSet(LED0_port, LED0_pin);
      }

      else if (all_flags & (direction_limit | speed_limit)) {
          GPIO_PinOutSet(LED0_port, LED0_pin);
          GPIO_PinOutSet(LED1_port, LED1_pin);
      }

      else if (all_flags & (no_operation | clear_speed)) {
          GPIO_PinOutClear(LED0_port, LED0_pin);
          GPIO_PinOutClear(LED1_port, LED1_pin);
      }

      else {
          GPIO_PinOutClear(LED0_port, LED0_pin);
          GPIO_PinOutClear(LED1_port, LED1_pin);
      }
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

  }


}

void LCD_DISPLAY_TASK(void *p_arg) {
  (void)&p_arg;
  RTOS_ERR err;
  int speed_read1 = 0;
  int direction_read1 = no_direction;

  char str_speed[100];
  char str_direction[100];

  while(1) {


      OSTimeDly(10, OS_OPT_TIME_DLY, &err);
      EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));

  OSMutexPend (&mutex_speed,
                0,
                OS_OPT_PEND_BLOCKING,
                NULL,
                &err);

                speed_read1 = speed.current_speed;

  OSMutexPost (&mutex_speed,
               OS_OPT_POST_NONE,
               &err);


  OSMutexPend (&mutex_direction,
                0,
                OS_OPT_PEND_BLOCKING,
                NULL,
                &err);

                  direction_read1 = direction.current_direction;

  OSMutexPost (&mutex_direction,
              OS_OPT_POST_NONE,
              &err);

  /* Fill lcd with background color */
  GLIB_clear(&glibContext);

  sprintf(str_speed, "CSpeed is: %d", speed_read1);

  if (direction_read1 == hard_left) { sprintf(str_direction, "Hard Left"); }
  if (direction_read1 == hard_right) { sprintf(str_direction, "Hard Right"); }
  if (direction_read1 == soft_left) {sprintf(str_direction, "Soft Left"); }
  if (direction_read1 == soft_right) {sprintf(str_direction, "Soft Right");}
  if (direction_read1 == no_direction) { sprintf(str_direction, "Noo Direction");}


  GLIB_drawStringOnLine(&glibContext,
                                str_speed,
                                 0,
                                 GLIB_ALIGN_LEFT,
                                 5,
                                 5,
                                 true);

  GLIB_drawStringOnLine(&glibContext,
                                str_direction,
                                 1,
                                 GLIB_ALIGN_LEFT,
                                 5,
                                 5,
                                 true);

  /* Post updates to display */
    DMD_updateDisplay();
    EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
  }
}



/***************************************************************************//**
LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW
LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW
LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW
LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW
LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW
LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW
LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW
LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW
LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW
LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW
LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW
LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW
LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW
LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW
LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW
LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW
LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW LAB 2 FUNCTIONS BELOW
 ******************************************************************************/



/***************************************************************************//**
 * @brief
 * tests button 0
 * @details
 * returns if button 0 is pressed or not using if/else loop
 * @note
 * function utilized in main.c to save the button 0 state into a local variable
 ******************************************************************************/
uint32_t button0_test(void) {


  if (GPIO_PinInGet(BUTTON0_port, BUTTON0_pin) == 0) { //button pressed
      button0_state = 0;
   }

  else { button0_state = 1; } //button not pressed

  return button0_state;

}
/***************************************************************************//**
 * @brief
 * tests button 1
 * @details
 * returns if button 1 is pressed or not using if/else loop
 * @note
 * function utilized in main.c to save the button 1 state into a local variable
 ******************************************************************************/
uint32_t button1_test(void) {


  if (GPIO_PinInGet(BUTTON1_port, BUTTON1_pin) == 0) { //button pressed
      button1_state = 0;
  }

  else { button1_state = 1; } //button not pressed

  return button1_state;

}


//buttons are good

/**********************************************************************************************************************************************//**
 * @brief
 * tests capslider sensor
 * @details
 * returns which side of the cap sensor is being touched. Section (0) and (1) is the "left-half" while Section (2) and (3) is the "right-half"
 * @note
 * function utilized in main.c to save which side of the cap slider is being touched, left-half, right-half, or none
 *****************************************************************************************************************************************************/
void capsense_main() {

  RTOS_ERR err;
  OSMutexPend (&mutex_direction,
                      0,
                      OS_OPT_PEND_NON_BLOCKING,
                      NULL,
                      &err);

  CAPSENSE_Sense();



  area0 = CAPSENSE_getPressed(0);
  area1 = CAPSENSE_getPressed(1);
  area2 = CAPSENSE_getPressed(2);
  area3 = CAPSENSE_getPressed(3);
  int temp_direction = direction.current_direction; // store the current into a temp variable and
  //save it to the end.

  if (area0 == true) {


      direction.current_direction = hard_left; //enum 2



  }

  else if (area1 == true) {


      direction.current_direction = soft_left; //enum 8


  }

  else if (area2 == true) {


      direction.current_direction = soft_right; //enum 16


  }

  else if (area3 == true) {


       direction.current_direction = hard_right; //enum 4


  }

  else {


             direction.current_direction = no_direction; //enum 1


  }

  OSMutexPost (&mutex_direction,
                           OS_OPT_POST_NONE,
                           &err);

  if ((temp_direction == no_direction) && (direction.current_direction == no_direction)) {

      OSTmrStart(&App_Timer, &err); //reset the timer?
  }

  else if (temp_direction != direction.current_direction) { //direction has changed
      //post an event to the LED output task (turn off the LEDs)
      OSFlagPost (&event_flag_led,
                    clear_speed,
                    OS_OPT_POST_FLAG_SET,
                    &err);

      OSTmrStart(&App_Timer, &err);
  }

  else {

      //callback

  }

}


/***************************************************************************//**
LAB 2 INTERRUPT DRIVEN IMPLEMENTATIONS BELOW
LAB 2 INTERRUPT DRIVEN IMPLEMENTATIONS BELOW
LAB 2 INTERRUPT DRIVEN IMPLEMENTATIONS BELOW
LAB 2 INTERRUPT DRIVEN IMPLEMENTATIONS BELOW
LAB 2 INTERRUPT DRIVEN IMPLEMENTATIONS BELOW
LAB 2 INTERRUPT DRIVEN IMPLEMENTATIONS BELOW
LAB 2 INTERRUPT DRIVEN IMPLEMENTATIONS BELOW
LAB 2 INTERRUPT DRIVEN IMPLEMENTATIONS BELOW
LAB 2 INTERRUPT DRIVEN IMPLEMENTATIONS BELOW
LAB 2 INTERRUPT DRIVEN IMPLEMENTATIONS BELOW
LAB 2 INTERRUPT DRIVEN IMPLEMENTATIONS BELOW
LAB 2 INTERRUPT DRIVEN IMPLEMENTATIONS BELOW
LAB 2 INTERRUPT DRIVEN IMPLEMENTATIONS BELOW
LAB 2 INTERRUPT DRIVEN IMPLEMENTATIONS BELOW
 ******************************************************************************/



/***************************************************************************//**
 * @brief
 *   Interrupt handler to service pressing of buttons
 ******************************************************************************/
void GPIO_EVEN_IRQHandler(void) //BUTTON 0 INTERRUPT
{
  uint32_t interrupt_flag;
  RTOS_ERR err;
  interrupt_flag = GPIO -> IF & GPIO -> IEN;
  GPIO-> IFC = interrupt_flag;
  button0_test_result = button0_test();
  //TODO : implement an entry into the FIFO
  push(&button0, button0_test_result);
  //Post to the semaphore each time an entry is added to the FIFO
  OSSemPost (&semaphore,
           OS_OPT_POST_ALL,
               &err);
}


/***************************************************************************//**
 * @brief
 *   Interrupt handler to service pressing of buttons
 ******************************************************************************/
void GPIO_ODD_IRQHandler(void) //BUTTON 1 INTERRUPT
{
  uint32_t interrupt_flag2;
  RTOS_ERR err;
  interrupt_flag2 = GPIO -> IF & GPIO -> IEN;
  GPIO -> IFC = interrupt_flag2;
  button1_test_result = button1_test();
  //TODO : implement an entry into the FIFO
  push(&button1, button1_test_result);
  //Post to the semaphore each time an entry is added to the FIFO
  OSSemPost (&semaphore,
           OS_OPT_POST_ALL,
               &err);
}


/***************************************************************************//**
 * @brief
 *  Add the code from Lab2 that was placed under the compile-time switch for the interrupt-driven implementation.
 *  Compile-time switch is not needed for this lab. (Lab 6)
 ******************************************************************************/
void drive_interrupts(void) {
  GPIO_ExtIntConfig(BUTTON0_port, BUTTON0_pin, BUTTON0_pin, true, true, true);
  GPIO_ExtIntConfig(BUTTON1_port, BUTTON1_pin, BUTTON1_pin, true, true, true);
  GPIO_IntClear(0xFFFFFFFF);
  NVIC_ClearPendingIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  NVIC_ClearPendingIRQ(GPIO_ODD_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
}

/***************************************************************************//**
END OF LAB 2 INTERRUPT DRIVEN IMPLEMENTATIONS
END OF LAB 2 INTERRUPT DRIVEN IMPLEMENTATIONS
END OF LAB 2 INTERRUPT DRIVEN IMPLEMENTATIONS
END OF LAB 2 INTERRUPT DRIVEN IMPLEMENTATIONS
END OF LAB 2 INTERRUPT DRIVEN IMPLEMENTATIONS
END OF LAB 2 INTERRUPT DRIVEN IMPLEMENTATIONS
END OF LAB 2 INTERRUPT DRIVEN IMPLEMENTATIONS
END OF LAB 2 INTERRUPT DRIVEN IMPLEMENTATIONS
END OF LAB 2 INTERRUPT DRIVEN IMPLEMENTATIONS
END OF LAB 2 INTERRUPT DRIVEN IMPLEMENTATIONS
 ******************************************************************************/












