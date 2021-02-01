//Written by Michael Frajman and Shi Tong Li
//kalman filtre update subroutine

/*
the kalman struct's variables follow the order:
float q
float r
float x
float p
float k
 */

.text
.syntax unified
.global kalman_update

kalman_update:
	STMDB SP!, {R2,R3,R4,R5,R6,R7,R8,R9,R10,R11,R12,LR}	//push all registers to the stack on entering subroutine
	MOV R2, R0											//make a copy of address of struct's starting point in R2
	VLDR S1, [R2]										//S1 contains kalman_filtre.q
	ADD R2, R2, #4										//increment struct pointer
	VLDR S2, [R2]										//S2 contains kalman_filtre.r
	ADD R2, R2, #4										//incrment struct pointer
	VLDR S3, [R2]										//S3 contains kalman_filtre.x
	ADD R2, R2, #4										//increment struct pointer
	VLDR S4, [R2]										//S4 contains kalman_filtre.p
	ADD R2, R2, #4										//increment struct pointer
	VLDR S5, [R2]										//S5 contains kalman_filtre.k
	VADD.f32 S4, S4, S1									//p=p+q
	VADD.f32 S6, S4, S2									//S6=p+r
	VDIV.f32 S5, S4, S6									//k=p/S6=p/(p+r)
	VSUB.f32 S6, S0, S3									//S6=measurement-x
	VMUL.f32 S6, S6, S5									//S6=k*(S6)=k*(measurment-x)
	VADD.f32 S3, S3, S6									//x=x+S6=x+k*(measurement-x)
	VMOV.f32 S7, #1.0									//move float 1.0 into S7
	VSUB.f32 S6, S7, S5									//S6=1-k
	VMUL.f32 S4, S6, S4									//p=S6*p=(1-k)*p
	//VSTR S1, [R0]										//store updated q back to first position in struct
	//VSTR S2, [R0, #4]									//store updated r back to second position in struct
	VSTR S3, [R0, #8]									//store updated x back to third position in struct
	VSTR S4, [R0, #16]									//store updated p back to fourth position in struct
	VSTR S5, [R0, #20]									//store updated k back to fifth position in struct
	VMOV.f32 S0, S3										//return x
	//ADD R0, R0, #8										//return address of x
	LDMIA SP!, {R2,R3,R4,R5,R6,R7,R8,R9,R10,R11,R12,LR} //pop the original values back from the stack
	BX LR	 											//branch back to the point in the loop in link register

.end
