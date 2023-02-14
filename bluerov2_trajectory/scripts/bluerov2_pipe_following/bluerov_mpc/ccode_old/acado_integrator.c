/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"


real_t rk_dim16_swap;

/** Column vector of size: 16 */
real_t rk_dim16_bPerm[ 16 ];

/** Column vector of size: 32 */
real_t auxVar[ 32 ];

real_t rk_ttt;

/** Row vector of size: 22 */
real_t rk_xxx[ 22 ];

/** Matrix of size: 8 x 2 (row major format) */
real_t rk_kkk[ 16 ];

/** Matrix of size: 16 x 16 (row major format) */
real_t rk_A[ 256 ];

/** Column vector of size: 16 */
real_t rk_b[ 16 ];

/** Row vector of size: 16 */
int rk_dim16_perm[ 16 ];

/** Column vector of size: 8 */
real_t rk_rhsTemp[ 8 ];

/** Matrix of size: 2 x 96 (row major format) */
real_t rk_diffsTemp2[ 192 ];

/** Matrix of size: 8 x 2 (row major format) */
real_t rk_diffK[ 16 ];

/** Matrix of size: 8 x 12 (row major format) */
real_t rk_diffsNew2[ 96 ];

#pragma omp threadprivate( auxVar, rk_ttt, rk_xxx, rk_kkk, rk_diffK, rk_rhsTemp, rk_dim16_perm, rk_A, rk_b, rk_diffsNew2, rk_diffsTemp2, rk_dim16_swap, rk_dim16_bPerm )

void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 8;
/* Vector of auxiliary variables; number of elements: 8. */
real_t* a = auxVar;

/* Compute intermediate quantities: */
a[0] = (cos(xd[6]));
a[1] = (sin(xd[6]));
a[2] = (sin(xd[6]));
a[3] = (cos(xd[6]));
a[4] = (sqrt(((xd[3]*xd[3])+(real_t)(1.0000000000000000e-03))));
a[5] = (sqrt(((xd[4]*xd[4])+(real_t)(1.0000000000000000e-03))));
a[6] = (sqrt(((xd[5]*xd[5])+(real_t)(1.0000000000000000e-03))));
a[7] = (sqrt(((xd[7]*xd[7])+(real_t)(1.0000000000000000e-03))));

/* Compute outputs: */
out[0] = ((a[0]*xd[3])-(a[1]*xd[4]));
out[1] = ((a[2]*xd[3])+(a[3]*xd[4]));
out[2] = xd[5];
out[3] = (((u[0]+((((real_t)(1.1500000000000000e+01)*xd[4])+((real_t)(-6.0571999999999999e+00)*xd[4]))*xd[7]))+(((real_t)(-9.4600000000000004e-02)+((real_t)(-6.0418000000000003e+00)*a[4]))*xd[3]))/(real_t)(1.3440400000000000e+01));
out[4] = (((u[1]-((((real_t)(1.1500000000000000e+01)*xd[3])+((real_t)(-1.9403999999999999e+00)*xd[3]))*xd[7]))+(((real_t)(-5.8745000000000003e+00)+((real_t)(-3.0731000000000002e+01)*a[5]))*xd[4]))/(real_t)(1.7557200000000002e+01));
out[5] = ((u[2]+(((real_t)(-3.7020000000000000e+00)+((real_t)(-2.6356999999999999e+01)*a[6]))*xd[5]))/(real_t)(1.5448200000000000e+01));
out[6] = xd[7];
out[7] = ((((u[3]-((((real_t)(1.1500000000000000e+01)*xd[4])-((real_t)(-6.0571999999999999e+00)*xd[4]))*xd[3]))-((((real_t)(-1.9403999999999999e+00)*xd[3])-((real_t)(1.1500000000000000e+01)*xd[3]))*xd[4]))+(((real_t)(-2.3000000000000000e-02)+((real_t)(-4.5040000000000002e-01)*a[7]))*xd[7]))/(real_t)(3.7120000000000000e-02));
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;
/* Vector of auxiliary variables; number of elements: 32. */
real_t* a = auxVar;

/* Compute intermediate quantities: */
a[0] = (cos(xd[6]));
a[1] = (sin(xd[6]));
a[2] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[6])));
a[3] = (cos(xd[6]));
a[4] = (sin(xd[6]));
a[5] = (cos(xd[6]));
a[6] = (cos(xd[6]));
a[7] = ((real_t)(-1.0000000000000000e+00)*(sin(xd[6])));
a[8] = (1.0/sqrt(((xd[3]*xd[3])+(real_t)(1.0000000000000000e-03))));
a[9] = (a[8]*(real_t)(5.0000000000000000e-01));
a[10] = ((xd[3]+xd[3])*a[9]);
a[11] = (sqrt(((xd[3]*xd[3])+(real_t)(1.0000000000000000e-03))));
a[12] = ((real_t)(1.0000000000000000e+00)/(real_t)(1.3440400000000000e+01));
a[13] = ((real_t)(1.0000000000000000e+00)/(real_t)(1.3440400000000000e+01));
a[14] = ((real_t)(1.0000000000000000e+00)/(real_t)(1.7557200000000002e+01));
a[15] = (1.0/sqrt(((xd[4]*xd[4])+(real_t)(1.0000000000000000e-03))));
a[16] = (a[15]*(real_t)(5.0000000000000000e-01));
a[17] = ((xd[4]+xd[4])*a[16]);
a[18] = (sqrt(((xd[4]*xd[4])+(real_t)(1.0000000000000000e-03))));
a[19] = ((real_t)(1.0000000000000000e+00)/(real_t)(1.7557200000000002e+01));
a[20] = (1.0/sqrt(((xd[5]*xd[5])+(real_t)(1.0000000000000000e-03))));
a[21] = (a[20]*(real_t)(5.0000000000000000e-01));
a[22] = ((xd[5]+xd[5])*a[21]);
a[23] = (sqrt(((xd[5]*xd[5])+(real_t)(1.0000000000000000e-03))));
a[24] = ((real_t)(1.0000000000000000e+00)/(real_t)(1.5448200000000000e+01));
a[25] = ((real_t)(1.0000000000000000e+00)/(real_t)(1.5448200000000000e+01));
a[26] = ((real_t)(1.0000000000000000e+00)/(real_t)(3.7120000000000000e-02));
a[27] = (1.0/sqrt(((xd[7]*xd[7])+(real_t)(1.0000000000000000e-03))));
a[28] = (a[27]*(real_t)(5.0000000000000000e-01));
a[29] = ((xd[7]+xd[7])*a[28]);
a[30] = (sqrt(((xd[7]*xd[7])+(real_t)(1.0000000000000000e-03))));
a[31] = ((real_t)(1.0000000000000000e+00)/(real_t)(3.7120000000000000e-02));

/* Compute outputs: */
out[0] = (real_t)(0.0000000000000000e+00);
out[1] = (real_t)(0.0000000000000000e+00);
out[2] = (real_t)(0.0000000000000000e+00);
out[3] = a[0];
out[4] = ((real_t)(0.0000000000000000e+00)-a[1]);
out[5] = (real_t)(0.0000000000000000e+00);
out[6] = ((a[2]*xd[3])-(a[3]*xd[4]));
out[7] = (real_t)(0.0000000000000000e+00);
out[8] = (real_t)(0.0000000000000000e+00);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = a[4];
out[16] = a[5];
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = ((a[6]*xd[3])+(a[7]*xd[4]));
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(1.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (((((real_t)(-6.0418000000000003e+00)*a[10])*xd[3])+((real_t)(-9.4600000000000004e-02)+((real_t)(-6.0418000000000003e+00)*a[11])))*a[12]);
out[40] = ((((real_t)(5.4428000000000001e+00))*xd[7])*a[12]);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = ((((real_t)(1.1500000000000000e+01)*xd[4])+((real_t)(-6.0571999999999999e+00)*xd[4]))*a[12]);
out[44] = a[13];
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (((real_t)(0.0000000000000000e+00)-(((real_t)(9.5595999999999997e+00))*xd[7]))*a[14]);
out[52] = (((((real_t)(-3.0731000000000002e+01)*a[17])*xd[4])+((real_t)(-5.8745000000000003e+00)+((real_t)(-3.0731000000000002e+01)*a[18])))*a[14]);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = (((real_t)(0.0000000000000000e+00)-(((real_t)(1.1500000000000000e+01)*xd[3])+((real_t)(-1.9403999999999999e+00)*xd[3])))*a[14]);
out[56] = (real_t)(0.0000000000000000e+00);
out[57] = a[19];
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = (real_t)(0.0000000000000000e+00);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = (real_t)(0.0000000000000000e+00);
out[64] = (real_t)(0.0000000000000000e+00);
out[65] = (((((real_t)(-2.6356999999999999e+01)*a[22])*xd[5])+((real_t)(-3.7020000000000000e+00)+((real_t)(-2.6356999999999999e+01)*a[23])))*a[24]);
out[66] = (real_t)(0.0000000000000000e+00);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = (real_t)(0.0000000000000000e+00);
out[69] = (real_t)(0.0000000000000000e+00);
out[70] = a[25];
out[71] = (real_t)(0.0000000000000000e+00);
out[72] = (real_t)(0.0000000000000000e+00);
out[73] = (real_t)(0.0000000000000000e+00);
out[74] = (real_t)(0.0000000000000000e+00);
out[75] = (real_t)(0.0000000000000000e+00);
out[76] = (real_t)(0.0000000000000000e+00);
out[77] = (real_t)(0.0000000000000000e+00);
out[78] = (real_t)(0.0000000000000000e+00);
out[79] = (real_t)(1.0000000000000000e+00);
out[80] = (real_t)(0.0000000000000000e+00);
out[81] = (real_t)(0.0000000000000000e+00);
out[82] = (real_t)(0.0000000000000000e+00);
out[83] = (real_t)(0.0000000000000000e+00);
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = (real_t)(0.0000000000000000e+00);
out[86] = (real_t)(0.0000000000000000e+00);
out[87] = ((((real_t)(0.0000000000000000e+00)-(((real_t)(1.1500000000000000e+01)*xd[4])-((real_t)(-6.0571999999999999e+00)*xd[4])))-(((real_t)(-1.9403999999999999e+00)-(real_t)(1.1500000000000000e+01))*xd[4]))*a[26]);
out[88] = ((((real_t)(0.0000000000000000e+00)-(((real_t)(1.1500000000000000e+01)-(real_t)(-6.0571999999999999e+00))*xd[3]))-(((real_t)(-1.9403999999999999e+00)*xd[3])-((real_t)(1.1500000000000000e+01)*xd[3])))*a[26]);
out[89] = (real_t)(0.0000000000000000e+00);
out[90] = (real_t)(0.0000000000000000e+00);
out[91] = (((((real_t)(-4.5040000000000002e-01)*a[29])*xd[7])+((real_t)(-2.3000000000000000e-02)+((real_t)(-4.5040000000000002e-01)*a[30])))*a[26]);
out[92] = (real_t)(0.0000000000000000e+00);
out[93] = (real_t)(0.0000000000000000e+00);
out[94] = (real_t)(0.0000000000000000e+00);
out[95] = a[31];
}



void acado_solve_dim16_triangular( real_t* const A, real_t* const b )
{

b[15] = b[15]/A[255];
b[14] -= + A[239]*b[15];
b[14] = b[14]/A[238];
b[13] -= + A[223]*b[15];
b[13] -= + A[222]*b[14];
b[13] = b[13]/A[221];
b[12] -= + A[207]*b[15];
b[12] -= + A[206]*b[14];
b[12] -= + A[205]*b[13];
b[12] = b[12]/A[204];
b[11] -= + A[191]*b[15];
b[11] -= + A[190]*b[14];
b[11] -= + A[189]*b[13];
b[11] -= + A[188]*b[12];
b[11] = b[11]/A[187];
b[10] -= + A[175]*b[15];
b[10] -= + A[174]*b[14];
b[10] -= + A[173]*b[13];
b[10] -= + A[172]*b[12];
b[10] -= + A[171]*b[11];
b[10] = b[10]/A[170];
b[9] -= + A[159]*b[15];
b[9] -= + A[158]*b[14];
b[9] -= + A[157]*b[13];
b[9] -= + A[156]*b[12];
b[9] -= + A[155]*b[11];
b[9] -= + A[154]*b[10];
b[9] = b[9]/A[153];
b[8] -= + A[143]*b[15];
b[8] -= + A[142]*b[14];
b[8] -= + A[141]*b[13];
b[8] -= + A[140]*b[12];
b[8] -= + A[139]*b[11];
b[8] -= + A[138]*b[10];
b[8] -= + A[137]*b[9];
b[8] = b[8]/A[136];
b[7] -= + A[127]*b[15];
b[7] -= + A[126]*b[14];
b[7] -= + A[125]*b[13];
b[7] -= + A[124]*b[12];
b[7] -= + A[123]*b[11];
b[7] -= + A[122]*b[10];
b[7] -= + A[121]*b[9];
b[7] -= + A[120]*b[8];
b[7] = b[7]/A[119];
b[6] -= + A[111]*b[15];
b[6] -= + A[110]*b[14];
b[6] -= + A[109]*b[13];
b[6] -= + A[108]*b[12];
b[6] -= + A[107]*b[11];
b[6] -= + A[106]*b[10];
b[6] -= + A[105]*b[9];
b[6] -= + A[104]*b[8];
b[6] -= + A[103]*b[7];
b[6] = b[6]/A[102];
b[5] -= + A[95]*b[15];
b[5] -= + A[94]*b[14];
b[5] -= + A[93]*b[13];
b[5] -= + A[92]*b[12];
b[5] -= + A[91]*b[11];
b[5] -= + A[90]*b[10];
b[5] -= + A[89]*b[9];
b[5] -= + A[88]*b[8];
b[5] -= + A[87]*b[7];
b[5] -= + A[86]*b[6];
b[5] = b[5]/A[85];
b[4] -= + A[79]*b[15];
b[4] -= + A[78]*b[14];
b[4] -= + A[77]*b[13];
b[4] -= + A[76]*b[12];
b[4] -= + A[75]*b[11];
b[4] -= + A[74]*b[10];
b[4] -= + A[73]*b[9];
b[4] -= + A[72]*b[8];
b[4] -= + A[71]*b[7];
b[4] -= + A[70]*b[6];
b[4] -= + A[69]*b[5];
b[4] = b[4]/A[68];
b[3] -= + A[63]*b[15];
b[3] -= + A[62]*b[14];
b[3] -= + A[61]*b[13];
b[3] -= + A[60]*b[12];
b[3] -= + A[59]*b[11];
b[3] -= + A[58]*b[10];
b[3] -= + A[57]*b[9];
b[3] -= + A[56]*b[8];
b[3] -= + A[55]*b[7];
b[3] -= + A[54]*b[6];
b[3] -= + A[53]*b[5];
b[3] -= + A[52]*b[4];
b[3] = b[3]/A[51];
b[2] -= + A[47]*b[15];
b[2] -= + A[46]*b[14];
b[2] -= + A[45]*b[13];
b[2] -= + A[44]*b[12];
b[2] -= + A[43]*b[11];
b[2] -= + A[42]*b[10];
b[2] -= + A[41]*b[9];
b[2] -= + A[40]*b[8];
b[2] -= + A[39]*b[7];
b[2] -= + A[38]*b[6];
b[2] -= + A[37]*b[5];
b[2] -= + A[36]*b[4];
b[2] -= + A[35]*b[3];
b[2] = b[2]/A[34];
b[1] -= + A[31]*b[15];
b[1] -= + A[30]*b[14];
b[1] -= + A[29]*b[13];
b[1] -= + A[28]*b[12];
b[1] -= + A[27]*b[11];
b[1] -= + A[26]*b[10];
b[1] -= + A[25]*b[9];
b[1] -= + A[24]*b[8];
b[1] -= + A[23]*b[7];
b[1] -= + A[22]*b[6];
b[1] -= + A[21]*b[5];
b[1] -= + A[20]*b[4];
b[1] -= + A[19]*b[3];
b[1] -= + A[18]*b[2];
b[1] = b[1]/A[17];
b[0] -= + A[15]*b[15];
b[0] -= + A[14]*b[14];
b[0] -= + A[13]*b[13];
b[0] -= + A[12]*b[12];
b[0] -= + A[11]*b[11];
b[0] -= + A[10]*b[10];
b[0] -= + A[9]*b[9];
b[0] -= + A[8]*b[8];
b[0] -= + A[7]*b[7];
b[0] -= + A[6]*b[6];
b[0] -= + A[5]*b[5];
b[0] -= + A[4]*b[4];
b[0] -= + A[3]*b[3];
b[0] -= + A[2]*b[2];
b[0] -= + A[1]*b[1];
b[0] = b[0]/A[0];
}

real_t acado_solve_dim16_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 16; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
for( i=0; i < (15); i++ ) {
	indexMax = i;
	valueMax = fabs(A[i*16+i]);
	for( j=(i+1); j < 16; j++ ) {
		temp = fabs(A[j*16+i]);
		if( temp > valueMax ) {
			indexMax = j;
			valueMax = temp;
		}
	}
	if( indexMax > i ) {
for (k = 0; k < 16; ++k)
{
	rk_dim16_swap = A[i*16+k];
	A[i*16+k] = A[indexMax*16+k];
	A[indexMax*16+k] = rk_dim16_swap;
}
	rk_dim16_swap = b[i];
	b[i] = b[indexMax];
	b[indexMax] = rk_dim16_swap;
	intSwap = rk_perm[i];
	rk_perm[i] = rk_perm[indexMax];
	rk_perm[indexMax] = intSwap;
	}
	det *= A[i*16+i];
	for( j=i+1; j < 16; j++ ) {
		A[j*16+i] = -A[j*16+i]/A[i*16+i];
		for( k=i+1; k < 16; k++ ) {
			A[j*16+k] += A[j*16+i] * A[i*16+k];
		}
		b[j] += A[j*16+i] * b[i];
	}
}
det *= A[255];
det = fabs(det);
acado_solve_dim16_triangular( A, b );
return det;
}

void acado_solve_dim16_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

rk_dim16_bPerm[0] = b[rk_perm[0]];
rk_dim16_bPerm[1] = b[rk_perm[1]];
rk_dim16_bPerm[2] = b[rk_perm[2]];
rk_dim16_bPerm[3] = b[rk_perm[3]];
rk_dim16_bPerm[4] = b[rk_perm[4]];
rk_dim16_bPerm[5] = b[rk_perm[5]];
rk_dim16_bPerm[6] = b[rk_perm[6]];
rk_dim16_bPerm[7] = b[rk_perm[7]];
rk_dim16_bPerm[8] = b[rk_perm[8]];
rk_dim16_bPerm[9] = b[rk_perm[9]];
rk_dim16_bPerm[10] = b[rk_perm[10]];
rk_dim16_bPerm[11] = b[rk_perm[11]];
rk_dim16_bPerm[12] = b[rk_perm[12]];
rk_dim16_bPerm[13] = b[rk_perm[13]];
rk_dim16_bPerm[14] = b[rk_perm[14]];
rk_dim16_bPerm[15] = b[rk_perm[15]];
rk_dim16_bPerm[1] += A[16]*rk_dim16_bPerm[0];

rk_dim16_bPerm[2] += A[32]*rk_dim16_bPerm[0];
rk_dim16_bPerm[2] += A[33]*rk_dim16_bPerm[1];

rk_dim16_bPerm[3] += A[48]*rk_dim16_bPerm[0];
rk_dim16_bPerm[3] += A[49]*rk_dim16_bPerm[1];
rk_dim16_bPerm[3] += A[50]*rk_dim16_bPerm[2];

rk_dim16_bPerm[4] += A[64]*rk_dim16_bPerm[0];
rk_dim16_bPerm[4] += A[65]*rk_dim16_bPerm[1];
rk_dim16_bPerm[4] += A[66]*rk_dim16_bPerm[2];
rk_dim16_bPerm[4] += A[67]*rk_dim16_bPerm[3];

rk_dim16_bPerm[5] += A[80]*rk_dim16_bPerm[0];
rk_dim16_bPerm[5] += A[81]*rk_dim16_bPerm[1];
rk_dim16_bPerm[5] += A[82]*rk_dim16_bPerm[2];
rk_dim16_bPerm[5] += A[83]*rk_dim16_bPerm[3];
rk_dim16_bPerm[5] += A[84]*rk_dim16_bPerm[4];

rk_dim16_bPerm[6] += A[96]*rk_dim16_bPerm[0];
rk_dim16_bPerm[6] += A[97]*rk_dim16_bPerm[1];
rk_dim16_bPerm[6] += A[98]*rk_dim16_bPerm[2];
rk_dim16_bPerm[6] += A[99]*rk_dim16_bPerm[3];
rk_dim16_bPerm[6] += A[100]*rk_dim16_bPerm[4];
rk_dim16_bPerm[6] += A[101]*rk_dim16_bPerm[5];

rk_dim16_bPerm[7] += A[112]*rk_dim16_bPerm[0];
rk_dim16_bPerm[7] += A[113]*rk_dim16_bPerm[1];
rk_dim16_bPerm[7] += A[114]*rk_dim16_bPerm[2];
rk_dim16_bPerm[7] += A[115]*rk_dim16_bPerm[3];
rk_dim16_bPerm[7] += A[116]*rk_dim16_bPerm[4];
rk_dim16_bPerm[7] += A[117]*rk_dim16_bPerm[5];
rk_dim16_bPerm[7] += A[118]*rk_dim16_bPerm[6];

rk_dim16_bPerm[8] += A[128]*rk_dim16_bPerm[0];
rk_dim16_bPerm[8] += A[129]*rk_dim16_bPerm[1];
rk_dim16_bPerm[8] += A[130]*rk_dim16_bPerm[2];
rk_dim16_bPerm[8] += A[131]*rk_dim16_bPerm[3];
rk_dim16_bPerm[8] += A[132]*rk_dim16_bPerm[4];
rk_dim16_bPerm[8] += A[133]*rk_dim16_bPerm[5];
rk_dim16_bPerm[8] += A[134]*rk_dim16_bPerm[6];
rk_dim16_bPerm[8] += A[135]*rk_dim16_bPerm[7];

rk_dim16_bPerm[9] += A[144]*rk_dim16_bPerm[0];
rk_dim16_bPerm[9] += A[145]*rk_dim16_bPerm[1];
rk_dim16_bPerm[9] += A[146]*rk_dim16_bPerm[2];
rk_dim16_bPerm[9] += A[147]*rk_dim16_bPerm[3];
rk_dim16_bPerm[9] += A[148]*rk_dim16_bPerm[4];
rk_dim16_bPerm[9] += A[149]*rk_dim16_bPerm[5];
rk_dim16_bPerm[9] += A[150]*rk_dim16_bPerm[6];
rk_dim16_bPerm[9] += A[151]*rk_dim16_bPerm[7];
rk_dim16_bPerm[9] += A[152]*rk_dim16_bPerm[8];

rk_dim16_bPerm[10] += A[160]*rk_dim16_bPerm[0];
rk_dim16_bPerm[10] += A[161]*rk_dim16_bPerm[1];
rk_dim16_bPerm[10] += A[162]*rk_dim16_bPerm[2];
rk_dim16_bPerm[10] += A[163]*rk_dim16_bPerm[3];
rk_dim16_bPerm[10] += A[164]*rk_dim16_bPerm[4];
rk_dim16_bPerm[10] += A[165]*rk_dim16_bPerm[5];
rk_dim16_bPerm[10] += A[166]*rk_dim16_bPerm[6];
rk_dim16_bPerm[10] += A[167]*rk_dim16_bPerm[7];
rk_dim16_bPerm[10] += A[168]*rk_dim16_bPerm[8];
rk_dim16_bPerm[10] += A[169]*rk_dim16_bPerm[9];

rk_dim16_bPerm[11] += A[176]*rk_dim16_bPerm[0];
rk_dim16_bPerm[11] += A[177]*rk_dim16_bPerm[1];
rk_dim16_bPerm[11] += A[178]*rk_dim16_bPerm[2];
rk_dim16_bPerm[11] += A[179]*rk_dim16_bPerm[3];
rk_dim16_bPerm[11] += A[180]*rk_dim16_bPerm[4];
rk_dim16_bPerm[11] += A[181]*rk_dim16_bPerm[5];
rk_dim16_bPerm[11] += A[182]*rk_dim16_bPerm[6];
rk_dim16_bPerm[11] += A[183]*rk_dim16_bPerm[7];
rk_dim16_bPerm[11] += A[184]*rk_dim16_bPerm[8];
rk_dim16_bPerm[11] += A[185]*rk_dim16_bPerm[9];
rk_dim16_bPerm[11] += A[186]*rk_dim16_bPerm[10];

rk_dim16_bPerm[12] += A[192]*rk_dim16_bPerm[0];
rk_dim16_bPerm[12] += A[193]*rk_dim16_bPerm[1];
rk_dim16_bPerm[12] += A[194]*rk_dim16_bPerm[2];
rk_dim16_bPerm[12] += A[195]*rk_dim16_bPerm[3];
rk_dim16_bPerm[12] += A[196]*rk_dim16_bPerm[4];
rk_dim16_bPerm[12] += A[197]*rk_dim16_bPerm[5];
rk_dim16_bPerm[12] += A[198]*rk_dim16_bPerm[6];
rk_dim16_bPerm[12] += A[199]*rk_dim16_bPerm[7];
rk_dim16_bPerm[12] += A[200]*rk_dim16_bPerm[8];
rk_dim16_bPerm[12] += A[201]*rk_dim16_bPerm[9];
rk_dim16_bPerm[12] += A[202]*rk_dim16_bPerm[10];
rk_dim16_bPerm[12] += A[203]*rk_dim16_bPerm[11];

rk_dim16_bPerm[13] += A[208]*rk_dim16_bPerm[0];
rk_dim16_bPerm[13] += A[209]*rk_dim16_bPerm[1];
rk_dim16_bPerm[13] += A[210]*rk_dim16_bPerm[2];
rk_dim16_bPerm[13] += A[211]*rk_dim16_bPerm[3];
rk_dim16_bPerm[13] += A[212]*rk_dim16_bPerm[4];
rk_dim16_bPerm[13] += A[213]*rk_dim16_bPerm[5];
rk_dim16_bPerm[13] += A[214]*rk_dim16_bPerm[6];
rk_dim16_bPerm[13] += A[215]*rk_dim16_bPerm[7];
rk_dim16_bPerm[13] += A[216]*rk_dim16_bPerm[8];
rk_dim16_bPerm[13] += A[217]*rk_dim16_bPerm[9];
rk_dim16_bPerm[13] += A[218]*rk_dim16_bPerm[10];
rk_dim16_bPerm[13] += A[219]*rk_dim16_bPerm[11];
rk_dim16_bPerm[13] += A[220]*rk_dim16_bPerm[12];

rk_dim16_bPerm[14] += A[224]*rk_dim16_bPerm[0];
rk_dim16_bPerm[14] += A[225]*rk_dim16_bPerm[1];
rk_dim16_bPerm[14] += A[226]*rk_dim16_bPerm[2];
rk_dim16_bPerm[14] += A[227]*rk_dim16_bPerm[3];
rk_dim16_bPerm[14] += A[228]*rk_dim16_bPerm[4];
rk_dim16_bPerm[14] += A[229]*rk_dim16_bPerm[5];
rk_dim16_bPerm[14] += A[230]*rk_dim16_bPerm[6];
rk_dim16_bPerm[14] += A[231]*rk_dim16_bPerm[7];
rk_dim16_bPerm[14] += A[232]*rk_dim16_bPerm[8];
rk_dim16_bPerm[14] += A[233]*rk_dim16_bPerm[9];
rk_dim16_bPerm[14] += A[234]*rk_dim16_bPerm[10];
rk_dim16_bPerm[14] += A[235]*rk_dim16_bPerm[11];
rk_dim16_bPerm[14] += A[236]*rk_dim16_bPerm[12];
rk_dim16_bPerm[14] += A[237]*rk_dim16_bPerm[13];

rk_dim16_bPerm[15] += A[240]*rk_dim16_bPerm[0];
rk_dim16_bPerm[15] += A[241]*rk_dim16_bPerm[1];
rk_dim16_bPerm[15] += A[242]*rk_dim16_bPerm[2];
rk_dim16_bPerm[15] += A[243]*rk_dim16_bPerm[3];
rk_dim16_bPerm[15] += A[244]*rk_dim16_bPerm[4];
rk_dim16_bPerm[15] += A[245]*rk_dim16_bPerm[5];
rk_dim16_bPerm[15] += A[246]*rk_dim16_bPerm[6];
rk_dim16_bPerm[15] += A[247]*rk_dim16_bPerm[7];
rk_dim16_bPerm[15] += A[248]*rk_dim16_bPerm[8];
rk_dim16_bPerm[15] += A[249]*rk_dim16_bPerm[9];
rk_dim16_bPerm[15] += A[250]*rk_dim16_bPerm[10];
rk_dim16_bPerm[15] += A[251]*rk_dim16_bPerm[11];
rk_dim16_bPerm[15] += A[252]*rk_dim16_bPerm[12];
rk_dim16_bPerm[15] += A[253]*rk_dim16_bPerm[13];
rk_dim16_bPerm[15] += A[254]*rk_dim16_bPerm[14];


acado_solve_dim16_triangular( A, rk_dim16_bPerm );
b[0] = rk_dim16_bPerm[0];
b[1] = rk_dim16_bPerm[1];
b[2] = rk_dim16_bPerm[2];
b[3] = rk_dim16_bPerm[3];
b[4] = rk_dim16_bPerm[4];
b[5] = rk_dim16_bPerm[5];
b[6] = rk_dim16_bPerm[6];
b[7] = rk_dim16_bPerm[7];
b[8] = rk_dim16_bPerm[8];
b[9] = rk_dim16_bPerm[9];
b[10] = rk_dim16_bPerm[10];
b[11] = rk_dim16_bPerm[11];
b[12] = rk_dim16_bPerm[12];
b[13] = rk_dim16_bPerm[13];
b[14] = rk_dim16_bPerm[14];
b[15] = rk_dim16_bPerm[15];
}



/** Matrix of size: 2 x 2 (row major format) */
static const real_t acado_Ah_mat[ 4 ] = 
{ 1.2500000000000001e-02, 2.6933756729740646e-02, 
-1.9337567297406434e-03, 1.2500000000000001e-02 };


/* Fixed step size:0.05 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int i;
int j;
int k;
int run;
int run1;
int tmp_index1;
int tmp_index2;

real_t det;

rk_ttt = 0.0000000000000000e+00;
rk_xxx[8] = rk_eta[104];
rk_xxx[9] = rk_eta[105];
rk_xxx[10] = rk_eta[106];
rk_xxx[11] = rk_eta[107];
rk_xxx[12] = rk_eta[108];
rk_xxx[13] = rk_eta[109];
rk_xxx[14] = rk_eta[110];
rk_xxx[15] = rk_eta[111];
rk_xxx[16] = rk_eta[112];
rk_xxx[17] = rk_eta[113];
rk_xxx[18] = rk_eta[114];
rk_xxx[19] = rk_eta[115];
rk_xxx[20] = rk_eta[116];
rk_xxx[21] = rk_eta[117];

for (run = 0; run < 1; ++run)
{
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 8; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1 * 2]*rk_kkk[tmp_index1 * 2];
rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( rk_xxx, &(rk_diffsTemp2[ run1 * 96 ]) );
for (j = 0; j < 8; ++j)
{
tmp_index1 = (run1 * 8) + (j);
rk_A[tmp_index1 * 16] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 96) + (j * 12)];
rk_A[tmp_index1 * 16 + 1] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 96) + (j * 12 + 1)];
rk_A[tmp_index1 * 16 + 2] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 96) + (j * 12 + 2)];
rk_A[tmp_index1 * 16 + 3] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 96) + (j * 12 + 3)];
rk_A[tmp_index1 * 16 + 4] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 96) + (j * 12 + 4)];
rk_A[tmp_index1 * 16 + 5] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 96) + (j * 12 + 5)];
rk_A[tmp_index1 * 16 + 6] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 96) + (j * 12 + 6)];
rk_A[tmp_index1 * 16 + 7] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 96) + (j * 12 + 7)];
if( 0 == run1 ) rk_A[(tmp_index1 * 16) + (j)] -= 1.0000000000000000e+00;
rk_A[tmp_index1 * 16 + 8] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 96) + (j * 12)];
rk_A[tmp_index1 * 16 + 9] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 96) + (j * 12 + 1)];
rk_A[tmp_index1 * 16 + 10] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 96) + (j * 12 + 2)];
rk_A[tmp_index1 * 16 + 11] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 96) + (j * 12 + 3)];
rk_A[tmp_index1 * 16 + 12] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 96) + (j * 12 + 4)];
rk_A[tmp_index1 * 16 + 13] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 96) + (j * 12 + 5)];
rk_A[tmp_index1 * 16 + 14] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 96) + (j * 12 + 6)];
rk_A[tmp_index1 * 16 + 15] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 96) + (j * 12 + 7)];
if( 1 == run1 ) rk_A[(tmp_index1 * 16) + (j + 8)] -= 1.0000000000000000e+00;
}
acado_rhs( rk_xxx, rk_rhsTemp );
rk_b[run1 * 8] = rk_kkk[run1] - rk_rhsTemp[0];
rk_b[run1 * 8 + 1] = rk_kkk[run1 + 2] - rk_rhsTemp[1];
rk_b[run1 * 8 + 2] = rk_kkk[run1 + 4] - rk_rhsTemp[2];
rk_b[run1 * 8 + 3] = rk_kkk[run1 + 6] - rk_rhsTemp[3];
rk_b[run1 * 8 + 4] = rk_kkk[run1 + 8] - rk_rhsTemp[4];
rk_b[run1 * 8 + 5] = rk_kkk[run1 + 10] - rk_rhsTemp[5];
rk_b[run1 * 8 + 6] = rk_kkk[run1 + 12] - rk_rhsTemp[6];
rk_b[run1 * 8 + 7] = rk_kkk[run1 + 14] - rk_rhsTemp[7];
}
det = acado_solve_dim16_system( rk_A, rk_b, rk_dim16_perm );
for (j = 0; j < 2; ++j)
{
rk_kkk[j] += rk_b[j * 8];
rk_kkk[j + 2] += rk_b[j * 8 + 1];
rk_kkk[j + 4] += rk_b[j * 8 + 2];
rk_kkk[j + 6] += rk_b[j * 8 + 3];
rk_kkk[j + 8] += rk_b[j * 8 + 4];
rk_kkk[j + 10] += rk_b[j * 8 + 5];
rk_kkk[j + 12] += rk_b[j * 8 + 6];
rk_kkk[j + 14] += rk_b[j * 8 + 7];
}
}
}
for (i = 0; i < 5; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 8; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1 * 2]*rk_kkk[tmp_index1 * 2];
rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*rk_kkk[tmp_index1 * 2 + 1];
}
acado_rhs( rk_xxx, rk_rhsTemp );
rk_b[run1 * 8] = rk_kkk[run1] - rk_rhsTemp[0];
rk_b[run1 * 8 + 1] = rk_kkk[run1 + 2] - rk_rhsTemp[1];
rk_b[run1 * 8 + 2] = rk_kkk[run1 + 4] - rk_rhsTemp[2];
rk_b[run1 * 8 + 3] = rk_kkk[run1 + 6] - rk_rhsTemp[3];
rk_b[run1 * 8 + 4] = rk_kkk[run1 + 8] - rk_rhsTemp[4];
rk_b[run1 * 8 + 5] = rk_kkk[run1 + 10] - rk_rhsTemp[5];
rk_b[run1 * 8 + 6] = rk_kkk[run1 + 12] - rk_rhsTemp[6];
rk_b[run1 * 8 + 7] = rk_kkk[run1 + 14] - rk_rhsTemp[7];
}
acado_solve_dim16_system_reuse( rk_A, rk_b, rk_dim16_perm );
for (j = 0; j < 2; ++j)
{
rk_kkk[j] += rk_b[j * 8];
rk_kkk[j + 2] += rk_b[j * 8 + 1];
rk_kkk[j + 4] += rk_b[j * 8 + 2];
rk_kkk[j + 6] += rk_b[j * 8 + 3];
rk_kkk[j + 8] += rk_b[j * 8 + 4];
rk_kkk[j + 10] += rk_b[j * 8 + 5];
rk_kkk[j + 12] += rk_b[j * 8 + 6];
rk_kkk[j + 14] += rk_b[j * 8 + 7];
}
}
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 8; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1 * 2]*rk_kkk[tmp_index1 * 2];
rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( rk_xxx, &(rk_diffsTemp2[ run1 * 96 ]) );
for (j = 0; j < 8; ++j)
{
tmp_index1 = (run1 * 8) + (j);
rk_A[tmp_index1 * 16] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 96) + (j * 12)];
rk_A[tmp_index1 * 16 + 1] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 96) + (j * 12 + 1)];
rk_A[tmp_index1 * 16 + 2] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 96) + (j * 12 + 2)];
rk_A[tmp_index1 * 16 + 3] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 96) + (j * 12 + 3)];
rk_A[tmp_index1 * 16 + 4] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 96) + (j * 12 + 4)];
rk_A[tmp_index1 * 16 + 5] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 96) + (j * 12 + 5)];
rk_A[tmp_index1 * 16 + 6] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 96) + (j * 12 + 6)];
rk_A[tmp_index1 * 16 + 7] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 96) + (j * 12 + 7)];
if( 0 == run1 ) rk_A[(tmp_index1 * 16) + (j)] -= 1.0000000000000000e+00;
rk_A[tmp_index1 * 16 + 8] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 96) + (j * 12)];
rk_A[tmp_index1 * 16 + 9] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 96) + (j * 12 + 1)];
rk_A[tmp_index1 * 16 + 10] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 96) + (j * 12 + 2)];
rk_A[tmp_index1 * 16 + 11] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 96) + (j * 12 + 3)];
rk_A[tmp_index1 * 16 + 12] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 96) + (j * 12 + 4)];
rk_A[tmp_index1 * 16 + 13] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 96) + (j * 12 + 5)];
rk_A[tmp_index1 * 16 + 14] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 96) + (j * 12 + 6)];
rk_A[tmp_index1 * 16 + 15] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 96) + (j * 12 + 7)];
if( 1 == run1 ) rk_A[(tmp_index1 * 16) + (j + 8)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 8; ++run1)
{
for (i = 0; i < 2; ++i)
{
rk_b[i * 8] = - rk_diffsTemp2[(i * 96) + (run1)];
rk_b[i * 8 + 1] = - rk_diffsTemp2[(i * 96) + (run1 + 12)];
rk_b[i * 8 + 2] = - rk_diffsTemp2[(i * 96) + (run1 + 24)];
rk_b[i * 8 + 3] = - rk_diffsTemp2[(i * 96) + (run1 + 36)];
rk_b[i * 8 + 4] = - rk_diffsTemp2[(i * 96) + (run1 + 48)];
rk_b[i * 8 + 5] = - rk_diffsTemp2[(i * 96) + (run1 + 60)];
rk_b[i * 8 + 6] = - rk_diffsTemp2[(i * 96) + (run1 + 72)];
rk_b[i * 8 + 7] = - rk_diffsTemp2[(i * 96) + (run1 + 84)];
}
if( 0 == run1 ) {
det = acado_solve_dim16_system( rk_A, rk_b, rk_dim16_perm );
}
 else {
acado_solve_dim16_system_reuse( rk_A, rk_b, rk_dim16_perm );
}
for (i = 0; i < 2; ++i)
{
rk_diffK[i] = rk_b[i * 8];
rk_diffK[i + 2] = rk_b[i * 8 + 1];
rk_diffK[i + 4] = rk_b[i * 8 + 2];
rk_diffK[i + 6] = rk_b[i * 8 + 3];
rk_diffK[i + 8] = rk_b[i * 8 + 4];
rk_diffK[i + 10] = rk_b[i * 8 + 5];
rk_diffK[i + 12] = rk_b[i * 8 + 6];
rk_diffK[i + 14] = rk_b[i * 8 + 7];
}
for (i = 0; i < 8; ++i)
{
rk_diffsNew2[(i * 12) + (run1)] = (i == run1-0);
rk_diffsNew2[(i * 12) + (run1)] += + rk_diffK[i * 2]*(real_t)2.5000000000000001e-02 + rk_diffK[i * 2 + 1]*(real_t)2.5000000000000001e-02;
}
}
for (run1 = 0; run1 < 4; ++run1)
{
for (i = 0; i < 2; ++i)
{
for (j = 0; j < 8; ++j)
{
tmp_index1 = (i * 8) + (j);
tmp_index2 = (run1) + (j * 12);
rk_b[tmp_index1] = - rk_diffsTemp2[(i * 96) + (tmp_index2 + 8)];
}
}
acado_solve_dim16_system_reuse( rk_A, rk_b, rk_dim16_perm );
for (i = 0; i < 2; ++i)
{
rk_diffK[i] = rk_b[i * 8];
rk_diffK[i + 2] = rk_b[i * 8 + 1];
rk_diffK[i + 4] = rk_b[i * 8 + 2];
rk_diffK[i + 6] = rk_b[i * 8 + 3];
rk_diffK[i + 8] = rk_b[i * 8 + 4];
rk_diffK[i + 10] = rk_b[i * 8 + 5];
rk_diffK[i + 12] = rk_b[i * 8 + 6];
rk_diffK[i + 14] = rk_b[i * 8 + 7];
}
for (i = 0; i < 8; ++i)
{
rk_diffsNew2[(i * 12) + (run1 + 8)] = + rk_diffK[i * 2]*(real_t)2.5000000000000001e-02 + rk_diffK[i * 2 + 1]*(real_t)2.5000000000000001e-02;
}
}
rk_eta[0] += + rk_kkk[0]*(real_t)2.5000000000000001e-02 + rk_kkk[1]*(real_t)2.5000000000000001e-02;
rk_eta[1] += + rk_kkk[2]*(real_t)2.5000000000000001e-02 + rk_kkk[3]*(real_t)2.5000000000000001e-02;
rk_eta[2] += + rk_kkk[4]*(real_t)2.5000000000000001e-02 + rk_kkk[5]*(real_t)2.5000000000000001e-02;
rk_eta[3] += + rk_kkk[6]*(real_t)2.5000000000000001e-02 + rk_kkk[7]*(real_t)2.5000000000000001e-02;
rk_eta[4] += + rk_kkk[8]*(real_t)2.5000000000000001e-02 + rk_kkk[9]*(real_t)2.5000000000000001e-02;
rk_eta[5] += + rk_kkk[10]*(real_t)2.5000000000000001e-02 + rk_kkk[11]*(real_t)2.5000000000000001e-02;
rk_eta[6] += + rk_kkk[12]*(real_t)2.5000000000000001e-02 + rk_kkk[13]*(real_t)2.5000000000000001e-02;
rk_eta[7] += + rk_kkk[14]*(real_t)2.5000000000000001e-02 + rk_kkk[15]*(real_t)2.5000000000000001e-02;
for (i = 0; i < 8; ++i)
{
for (j = 0; j < 8; ++j)
{
tmp_index2 = (j) + (i * 8);
rk_eta[tmp_index2 + 8] = rk_diffsNew2[(i * 12) + (j)];
}
for (j = 0; j < 4; ++j)
{
tmp_index2 = (j) + (i * 4);
rk_eta[tmp_index2 + 72] = rk_diffsNew2[(i * 12) + (j + 8)];
}
}
resetIntegrator = 0;
rk_ttt += 1.0000000000000000e+00;
}
for (i = 0; i < 8; ++i)
{
}
if( det < 1e-12 ) {
error = 2;
} else if( det < 1e-6 ) {
error = 1;
} else {
error = 0;
}
return error;
}



