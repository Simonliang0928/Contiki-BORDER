/**************************************************************************//*****
 * @file     stdio.c
 * @brief    Implementation of newlib syscall
 ********************************************************************************/

#include <stdio.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>

#undef errno
extern int errno;
extern int  _end;
/*
caddr_t _sbrk ( int incr )
{
  static unsigned char *heap = NULL;
  unsigned char *prev_heap;

  if (heap == NULL) {
    heap = (unsigned char *)&_end;
  }
  prev_heap = heap;

  heap += incr;

  return (caddr_t) prev_heap;
}*/

caddr_t _sbrk (int size)
{
   extern char __cs3_heap_start;
   extern char __cs3_heap_end ;
   static char *current_heap_end = &__cs3_heap_start;
   char *previous_heap_end;

   previous_heap_end = current_heap_end;

   if (current_heap_end + size > &__cs3_heap_end )
   {
      errno = ENOMEM;   // don't forget to include <errno.h>
      return (caddr_t) -1;
   }

   current_heap_end += size;

   return (caddr_t) previous_heap_end;
}

int link(char *old, char *new) {
return -1;
}

int _close(int file)
{
  return -1;
}

int _fstat(int file, struct stat *st)
{
  st->st_mode = S_IFCHR;
  return 0;
}

int _isatty(int file)
{
  return 1;
}

int _lseek(int file, int ptr, int dir)
{
  return 0;
}

int _read(int file, char *ptr, int len)
{
  return 0;
}

int _write(int file, char *ptr, int len)
{
  return len;
}

void abort(void)
{
  /* Abort called */
  while(1);
}
          
/* --------------------------------- End Of File ------------------------------ */
