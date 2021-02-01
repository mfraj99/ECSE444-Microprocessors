/*
 * kalman.h
 *      Author: Michael Frajman and Shi Tong Li
 */

#ifndef INC_KALMAN_H_
#define INC_KALMAN_H_

/*
 * Function for updating kalman filtre
 * address - The address of a struct containing all necessary kalman filtre parameters
 * measuremnt - An additional float being inputted by the user
 */
extern float kalman_update(int address, float measurement);

#endif /* INC_KALMAN_H_ */
