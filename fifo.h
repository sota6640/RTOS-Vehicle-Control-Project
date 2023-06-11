/*
 * fifo.h
 * Data Structure Implementation
 *  Created on: Nov 4, 2022
 *      Author: Sonal Tamrakar
 */

#ifndef FIFO_H_
#define FIFO_H_


#include <stdio.h>
#include <stdlib.h>


struct fifo_t {
    // Task information
    int fifo_size;
    struct node_t *head;
};

//Create the FIFO data structure and accessor function to store the state transitions of each button.
struct node_t{
  int btn; // 0 or 1
  struct node_t *next;
};
//Create the FIFO data structure and accessor function to store the state transitions of each button.


//Create the data structure to store the Vehicle Speed Data.
struct speed_t{
  int current_speed;
};
//Create the data structure to store the Vehicle Speed Data.

//Create the data structure to store the Vehicle Direction Data.
struct direction_t{
  int current_direction;
};
//Create the data structure to store the Vehicle Direction Data.


struct node_t* create_new_node(int button);
struct node_t* peek(struct node_t* list_head);
void pop(struct node_t** head);
void push(struct node_t**head, int button);




#endif /* FIFO_H_ */
