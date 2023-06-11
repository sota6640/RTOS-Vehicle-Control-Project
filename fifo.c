/*
 * fifo.c
 * Data Structure Implementation
 *  Created on: Nov 4, 2022
 *      Author: Sonal Tamrakar
 */
#include "fifo.h"

struct node_t* create_new_node(int btn){
    struct node_t* temp = (struct node_t*)malloc( sizeof(struct node_t));
    temp->btn = btn;
    temp->next = NULL;
    return temp;
}


struct node_t* peek(struct node_t* list_head){
  if(list_head == NULL){
    return NULL;
  }
  else{
    return list_head;
  }
  return NULL;
}


void pop(struct node_t**head){
    if(*head == NULL){
        return;
    }
    else{
        struct node_t* curr = (*head)->next;
        free(*head);
        *head = curr;
    }


}
void push(struct node_t **head, int button){
    if(*head == NULL){
      struct node_t * new_node = create_new_node(button);
      *head = new_node;
      return;
    }
    struct node_t* curr = *head;
    while (curr->next != NULL){
        curr = curr->next;
    }

    struct node_t * new_node = create_new_node(button);
    curr->next = new_node;
}

