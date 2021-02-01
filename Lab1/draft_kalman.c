struct kalman_filtre{
    float q; // process noise variance
    float r; // measurement noise variance
    float x; // value
    float p; // estimation error covariance
    float k; // kalman gain
};

void init_kalman(struct kalman_filtre kalman_filtre, float q, float r, float initial_value){
    kalman_filtre.q = q;
    kalman_filtre.r = r;
    kalman_filtre.x = initial_value;
    kalman_filtre.p = 0.0;
    kalman_filtre.k = 0.0;
}

float update(struct kalman_filtre kalman_filtre, float measurement){
    kalman_filtre.p += kalman_filtre.q;
    kalman_filtre.k = kalman_filtre.p / (kalman_filtre.p+kalman_filtre.r);
    kalman_filtre.x = kalman_filtre.x + kalman_filtre.k * (measurement - kalman_filtre.x);
    kalman_filtre.p = (1 - kalman_filtre.k) * kalman_filtre.p;

    return kalman_filtre.x;
}