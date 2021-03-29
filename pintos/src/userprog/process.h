#ifndef USERPROG_PROCESS_H
#define USERPROG_PROCESS_H

#include "threads/thread.h"
#include "threads/synch.h"

#define TID_ERROR         ((tid_t) -1)
#define TID_INITIALIZING  ((tid_t) -2)

tid_t process_execute (const char *file_name);
int process_wait (tid_t);
void process_exit (void);
void process_activate (void);



typedef struct process_control {
    int id;
    const char * cmd;
    struct semaphore sema_loading;
    struct semaphore sema_waiting;
    struct list_elem elem;
    struct thread * parent;
    bool waiting;
    bool exited;
};

//IDK where to put this... heeelllllppppppp
struct file_desc {
    int id;
    struct list_elem elem;//May not beed needed but could be useful
    struct file * file;
};

#endif /* userprog/process.h */
