/*
 * sample_populator.c
 *
 *  Created on: Apr. 6, 2021
 *      Author: Michael Frajman and Shi Tong Li
 */

// populate the sine wave sampling array for a C6 tone
void C6_sample_populator() {
	for (int i = 0; i < 43; i++) {
		float modulus = (float) i / 43;
		float radians = 6.283185 * modulus;
		radians = (arm_sin_f32(radians) + 1);
		C6_samples_v0[i] = (uint8_t) radians * 1;
		C6_samples_v1[i] = (uint8_t) radians * 22;
		C6_samples_v2[i] = (uint8_t) radians * 43;
		C6_samples_v3[i] = (uint8_t) radians * 64;
		C6_samples_v4[i] = (uint8_t) radians * 85;
	}
}

// populate the sine wave sampling array for a D6 tone
void D6_sample_populator() {
	for (int i = 0; i < 36; i++) {
		float modulus = (float) i / 36;
		float radians = 6.283185 * modulus;
		radians = (arm_sin_f32(radians) + 1);
		D6_samples_v0[i] = (uint8_t) radians * 1;
		D6_samples_v1[i] = (uint8_t) radians * 22;
		D6_samples_v2[i] = (uint8_t) radians * 43;
		D6_samples_v3[i] = (uint8_t) radians * 64;
		D6_samples_v4[i] = (uint8_t) radians * 85;
	}
}

// populate the sine wave sampling array for an E6 tone
void E6_sample_populator() {
	for (int i = 0; i < 33; i++) {
		float modulus = (float) i / 33;
		float radians = 6.283185 * modulus;
		radians = (arm_sin_f32(radians) + 1);
		E6_samples_v0[i] = (uint8_t) radians * 1;
		E6_samples_v1[i] = (uint8_t) radians * 22;
		E6_samples_v2[i] = (uint8_t) radians * 43;
		E6_samples_v3[i] = (uint8_t) radians * 64;
		E6_samples_v4[i] = (uint8_t) radians * 85;
	}
}

// populate the sine wave sampling array for an F6 tone
void F6_sample_populator() {
	for (int i = 0; i < 30; i++) {
		float modulus = (float) i / 30;
		float radians = 6.283185 * modulus;
		radians = (arm_sin_f32(radians) + 1);
		F6_samples_v0[i] = (uint8_t) radians * 1;
		F6_samples_v1[i] = (uint8_t) radians * 22;
		F6_samples_v2[i] = (uint8_t) radians * 43;
		F6_samples_v3[i] = (uint8_t) radians * 64;
		F6_samples_v4[i] = (uint8_t) radians * 85;
	}
}

// populate the sine wave sampling array for a G6 tone
void G6_sample_populator() {
	for (int i = 0; i < 27; i++) {
		float modulus = (float) i / 27;
		float radians = 6.283185 * modulus;
		radians = (arm_sin_f32(radians) + 1);
		G6_samples_v0[i] = (uint8_t) radians * 1;
		G6_samples_v1[i] = (uint8_t) radians * 22;
		G6_samples_v2[i] = (uint8_t) radians * 43;
		G6_samples_v3[i] = (uint8_t) radians * 64;
		G6_samples_v4[i] = (uint8_t) radians * 85;
	}

}

// populate the sine wave sampling array for an A6 tone
void A6_sample_populator() {
	for (int i = 0; i < 24; i++) {
		float modulus = (float) i / 24;
		float radians = 6.283185 * modulus;
		radians = (arm_sin_f32(radians) + 1);
		A6_samples_v0[i] = (uint8_t) radians * 1;
		A6_samples_v1[i] = (uint8_t) radians * 22;
		A6_samples_v2[i] = (uint8_t) radians * 43;
		A6_samples_v3[i] = (uint8_t) radians * 64;
		A6_samples_v4[i] = (uint8_t) radians * 85;
	}
}

// populate the sine wave sampling array for an B6 tone
void B6_sample_populator() {
	for (int i = 0; i < 21; i++) {
		float modulus = (float) i / 21;
		float radians = 6.283185 * modulus;
		radians = (arm_sin_f32(radians) + 1);
		B6_samples_v0[i] = (uint8_t) radians * 1;
		B6_samples_v1[i] = (uint8_t) radians * 22;
		B6_samples_v2[i] = (uint8_t) radians * 43;
		B6_samples_v3[i] = (uint8_t) radians * 64;
		B6_samples_v4[i] = (uint8_t) radians * 85;
	}
}

// populate the sine wave sampling array for an C7 tone
void C7_sample_populator() {
	for (int i = 0; i < 20; i++) {
		float modulus = (float) i / 20;
		float radians = 6.283185 * modulus;
		radians = (arm_sin_f32(radians) + 1);
		C7_samples_v0[i] = (uint8_t) radians * 1;
		C7_samples_v1[i] = (uint8_t) radians * 22;
		C7_samples_v2[i] = (uint8_t) radians * 43;
		C7_samples_v3[i] = (uint8_t) radians * 64;
		C7_samples_v4[i] = (uint8_t) radians * 85;
	}
}


