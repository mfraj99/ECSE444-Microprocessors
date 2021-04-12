/*
 * sample_populator.h
 *
 *  Created on: Apr. 6, 2021
 *      Author: Michael Frajman and Shi Tong Li
 */

#ifndef INC_SAMPLE_POPULATOR_H_
#define INC_SAMPLE_POPULATOR_H_

//sample arrays for pitches
uint8_t C6_samples_v0[43];
uint8_t C6_samples_v1[43];
uint8_t C6_samples_v2[43];
uint8_t C6_samples_v3[43];
uint8_t C6_samples_v4[43];

uint8_t D6_samples_v0[36];
uint8_t D6_samples_v1[36];
uint8_t D6_samples_v2[36];
uint8_t D6_samples_v3[36];
uint8_t D6_samples_v4[36];

uint8_t E6_samples_v0[33];
uint8_t E6_samples_v1[33];
uint8_t E6_samples_v2[33];
uint8_t E6_samples_v3[33];
uint8_t E6_samples_v4[33];

uint8_t F6_samples_v0[30];
uint8_t F6_samples_v1[30];
uint8_t F6_samples_v2[30];
uint8_t F6_samples_v3[30];
uint8_t F6_samples_v4[30];

uint8_t G6_samples_v0[27];
uint8_t G6_samples_v1[27];
uint8_t G6_samples_v2[27];
uint8_t G6_samples_v3[27];
uint8_t G6_samples_v4[27];

uint8_t A6_samples_v0[24];
uint8_t A6_samples_v1[24];
uint8_t A6_samples_v2[24];
uint8_t A6_samples_v3[24];
uint8_t A6_samples_v4[24];

uint8_t B6_samples_v0[21];
uint8_t B6_samples_v1[21];
uint8_t B6_samples_v2[21];
uint8_t B6_samples_v3[21];
uint8_t B6_samples_v4[21];

uint8_t C7_samples_v0[20];
uint8_t C7_samples_v1[20];
uint8_t C7_samples_v2[20];
uint8_t C7_samples_v3[20];
uint8_t C7_samples_v4[20];

//populates arrays with samples for the specified musical note
void C6_sample_populator();
void D6_sample_populator();
void E6_sample_populator();
void F6_sample_populator();
void G6_sample_populator();
void H6_sample_populator();
void C7_sample_populator();

#endif /* INC_SAMPLE_POPULATOR_H_ */
