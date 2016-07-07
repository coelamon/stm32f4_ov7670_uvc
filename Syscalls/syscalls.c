/*
 * syscalls.c
 *
 */

/*
 sbrk - увеличить размер области данных, использутся для malloc
 */

#include <sys/types.h>

caddr_t _sbrk(int incr)
{
    extern char _ebss;
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == 0)
    {
        heap_end = &_ebss;
    }
    prev_heap_end = heap_end;

    heap_end += incr;
    return (caddr_t) prev_heap_end;

}
