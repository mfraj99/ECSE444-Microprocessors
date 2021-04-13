/*
 * kalman_filtre.c
 *
 *  Created on: Apr 11, 2021
 *      Author: Michael Frajman and Shi Tong Li
 */

struct kalman_filtre{
	float q; //process noise covariance
	float r; //measurement noise covariance
	float x; //estimated value
	float p; //estimation error covariance
	float k; //adaptive kalman filtre gain
};

// outputs a kalman filtre struct from five kalman parameters
struct kalman_filtre create_filtre (float q, float r, float x, float p, float k){
	struct kalman_filtre my_filtre;
	my_filtre.q = q;
	my_filtre.r = r;
	my_filtre.x = x;
	my_filtre.p = p;
	my_filtre.k = k;

	return my_filtre;
}

// function to update kalman filtre in C
float c_kalman_update(struct kalman_filtre* kalman_filtre, float measurement){
    kalman_filtre->p += kalman_filtre->q;
    kalman_filtre->k = kalman_filtre->p / (kalman_filtre->p + kalman_filtre->r);
    kalman_filtre->x = kalman_filtre->x + kalman_filtre->k * (measurement - kalman_filtre->x);
    kalman_filtre->p = (1 - kalman_filtre->k) * kalman_filtre->p;

    return kalman_filtre->x;
}

// function to populate the output array with kalman_filtre.x after each update using c function
int c_kalman_filtre(float input_array[], float output_array[], struct kalman_filtre* kstate, int length){
	for(int i=0; i<length; i++){
			output_array[i] = c_kalman_update(kstate, input_array[i]);
		}
	return 0;
}


