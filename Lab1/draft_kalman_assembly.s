//draft and notes for part1 subroutine

//is this just supposed to be the "update function"
//like we pass the reference of a struct and this is just the computation
//initilization and structure definition go in the c file?
//^^seems to be the case

//consider divisions by zero

//for part 2 CMSIS functions exist for correlation and standard deviation?

/*
the kalman struct's variables follow the order: 
float q
float r
float x
float p
float k
 */

//need the other definition before global
.syntax unified //or .text?
.global kalman

kalman:
//struct address is R0
//measurement iss R1
//push the registers to the stack
//load the struct contents into RX's, +4 to memory address
//how can float conents be loaded to be moved into float registers later?????
//can a general purpose register hold a float?????
//VMOV SX, RX move all struct floats into floating point units
//do operations
//need to store result to memory to return it via R0
//return to address on link register and pop back register contents
.end