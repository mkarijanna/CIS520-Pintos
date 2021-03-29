#ifndef USERPROG_SYSCALL_H
#define USERPROG_SYSCALL_H
#include <list.h>


void syscall_init (void);
void syscall_exit (int);
typedef struct child_process {
    int child_id;
    struct semaphore * load;
    struct list_elem elem;
    bool complete;
} process;
#endif /* userprog/syscall.h */


