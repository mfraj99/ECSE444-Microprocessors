/*
 * kalman_filtre.h
 *
 *  Created on: Apr. 12, 2021
 *      Author: Michael Frajman and Shi Tong Li
 */

#ifndef INC_KALMAN_FILTRE_H_
#define INC_KALMAN_FILTRE_H_

/**
 *  Kalman filtre struct with five floating point parameters.
 *
 */
struct kalman_filtre{
	float q; //process noise covariance
	float r; //measurement noise covariance
	float x; //estimated value
	float p; //estimation error covariance
	float k; //adaptive kalman filtre gain
};


struct kalman_filtre create_filtre (float q, float r, float x, float p, float k);
float c_kalman_update(struct kalman_filtre* kalman_filtre, float measurement);
int c_kalman_filtre(float input_array[], float output_array[], struct kalman_filtre* kstate, int length);


#endif /* INC_KALMAN_FILTRE_H_ */
