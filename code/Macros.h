#ifndef _Macros_
#define _Macros_

#define SET_BIT(var,bit) var|=(1<<bit)
#define CLR_BIT(var,bit) var&=~(1<<bit)
#define TOG_BIT(var,bit) var^=(1<<bit)
#define GET_BIT(var,bit) (var&(1<<bit))>>bit

#define IS_BIT_SET(var,bit) (var&(1<<bit))>>bit
#define IS_BIT_CLR(var,bit) !((var&(1<<bit))>>bit)

#define ROR (var,bit) var=(var<<(REGIESR_SIZE-bit)|(var>>bit))
#define ROL (var,bit) var=(var>>(REGIESR_SIZE-bit)|(var<<bit))


#endif