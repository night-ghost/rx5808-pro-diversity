#ifndef COMPAT_H
#define COMPAT_H

//typedef char prog_char;

typedef char prog_char;
#undef PSTR 
#define PSTR(s) (__extension__({static const prog_char __c[] PROGMEM = (s); &__c[0];}))

#define NOINLINE __attribute__ ((noinline))
#define INLINE __attribute__ ((always_inline))
#define WEAK __attribute__((weak))


#define BYTE_OF(v,n) (((byte *)&(v))[n])

__extension__ typedef int __guard __attribute__((mode (__DI__)));

//extern "C" int __cxa_guard_acquire(__guard *);
//extern "C" void __cxa_guard_release (__guard *);
//extern "C" void __cxa_guard_abort (__guard *);
//extern "C" void __cxa_pure_virtual(void);


/*int WEAK __cxa_guard_acquire(__guard *g) {return !*(char *)(g);};
void WEAK  __cxa_guard_release (__guard *g) {*(char *)g = 1;};
void WEAK __cxa_guard_abort (__guard *) {};
void WEAK __cxa_pure_virtual(void) {};
*/

#endif
