#include "stdio.h"
#include "stdlib.h"

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

int main(void){

	int len = 200;

	float q = 0;
	float r = 0.1;
	float x = 0;
	float p = 0.1;
	float k = 0.1;

	float input[] = {28,26,28,21,-22,28,19,6,3,22,11,-10,27,22,-19,-7,-1,-8,-12,-7,-24,30,-10,20,-12,10,24,-11,23,8,-18,-14,-2,-21,20,-24,23,-19,-15,12,-30,18,9,-17,-27,-10,21,-7,-30,15,5,-24,1,5,-22,4,27,19,10,-27,-3,21,-2,5,-27,13,-10,7,27,21,16,9,14,4,30,2,18,17,25,-30,1,4,-7,-3,23,25,19,30,-2,29,22,6,26,11,-20,-21,-7,-7,9,3,14,-29,8,-21,-25,5,17,-18,-23,15,28,8,-3,5,10,-21,8,12,-7,6,-25,8,27,-16,-3,-27,14,11,9,-28,1,13,10,1,16,7,28,-1,12,27,-29,-4,-25,29,-6,26,30,-29,-19,28,-1,12,16,-12,-8,18,12,19,11,-19,10,27,12,-18,-3,7,7,-3,-24,-9,27,4,-17,-22,10,30,3,30,-27,-14,22,-7,19,3,21,-24,-4,-14,-28,10,-3,-25,18,-23,4,9,15,-3,0,-24};
	float output[len];

	struct kalman_filtre test = create_filtre(q, r, x, p, k);

	c_kalman_filtre(input, output, &test, 200);

	for (int i = 1; i <= len; i++){
		printf("%f\n", output[i]);
	}
	
	return 0;
}