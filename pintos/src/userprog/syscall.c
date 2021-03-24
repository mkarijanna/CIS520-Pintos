#include "userprog/syscall.h"
#include <stdio.h>
#include <syscall-nr.h>
#include "threads/interrupt.h"
#include "threads/thread.h"

#include "devices/shutdown.h"
#include "filesys/file.h"
#include "filesys/filesys.h"
#include "threads/init.h"
#include "userprog/process.h"


static void syscall_handler (struct intr_frame *);

void
syscall_init (void) 
{
  intr_register_int (0x30, 3, INTR_ON, syscall_handler, "syscall");
}

static void
syscall_handler (struct intr_frame *f UNUSED) 
{
  printf ("system call!\n");
  thread_exit ();
}

/* --------- Karijanna's code starts here ---------- */

/* Lock is in charge of ensuring that only one process can access the file system at one time. */
struct lock lock_file;

/* Terminates pintos -- rarely used */
void halt(void) 
{
  /* From shutdown.h*/
  shutdown_power_off(); 
}

/* Terminates the current user program, returning status to the kernel. 
   If the process's parent waits for it, this is the status that will be returned. 
   Conventionally, a status of 0 indicates success and nonzero values indicate errors. */
void exit(int status) 
{
	/* Print process name and exit status */
	printf("%s: exit(%d)\n", thread_current()->name, status);
    /* Set the exit status of the current thread */
    thread_current()->exit_status = status;
	thread_exit();
}

/* Runs the executable whose name is given in cmd_line, passing any given arguments, 
   and returns the new process's program id (pid). */
pid_t exec (const char *cmd_line) 
{
  struct thread* parent = thread_current();
  /* Program cannot run */
  if(cmd_line == NULL) {
    return -1;
  }
  lock_acquire(&lock_file);
  /* Create a new process */
  pid_t child_tid = process_execute(cmd_line);
  struct thread* child = process_get_child(parent, child_tid);
  if(!child->loaded) {
    child_tid = -1;
  }
  lock_release(&lock_file);
  return child_tid;
}

/* Waits for a child process pid and retrieves the child's exit status. */
int wait (pid_t pid)
{
  /* If the thread created is a valid thread, then we must disable interupts, 
     and add it to this threads list of child threads. */
  return process_wait(pid);
}

/* Creates a new file called file initially initial_size bytes in size. 
   Returns true if successful, false otherwise. */
bool create (const char *file, unsigned initial_size)
{
  lock_acquire(&lock_file);
  bool file_status = filesys_create(file, initial_size);
  lock_release(&lock_file);
  return file_status;
}

/* Deletes the file called file. Returns true if successful, false otherwise. */
bool remove (const char *file) {
  /* Use a lock to avoid race conditions */
  lock_acquire(&lock_file);
  bool was_removed = filesys_remove(file);
  lock_release(&lock_file);
  return was_removed;
}
