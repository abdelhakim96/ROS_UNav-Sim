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




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


/** Row vector of size: 118 */
real_t state[ 118 ];

int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
#pragma omp parallel for private(lRun1, state) shared(acadoWorkspace, acadoVariables)
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
state[0] = acadoVariables.x[lRun1 * 8];
state[1] = acadoVariables.x[lRun1 * 8 + 1];
state[2] = acadoVariables.x[lRun1 * 8 + 2];
state[3] = acadoVariables.x[lRun1 * 8 + 3];
state[4] = acadoVariables.x[lRun1 * 8 + 4];
state[5] = acadoVariables.x[lRun1 * 8 + 5];
state[6] = acadoVariables.x[lRun1 * 8 + 6];
state[7] = acadoVariables.x[lRun1 * 8 + 7];

state[104] = acadoVariables.u[lRun1 * 4];
state[105] = acadoVariables.u[lRun1 * 4 + 1];
state[106] = acadoVariables.u[lRun1 * 4 + 2];
state[107] = acadoVariables.u[lRun1 * 4 + 3];
state[108] = acadoVariables.od[lRun1 * 10];
state[109] = acadoVariables.od[lRun1 * 10 + 1];
state[110] = acadoVariables.od[lRun1 * 10 + 2];
state[111] = acadoVariables.od[lRun1 * 10 + 3];
state[112] = acadoVariables.od[lRun1 * 10 + 4];
state[113] = acadoVariables.od[lRun1 * 10 + 5];
state[114] = acadoVariables.od[lRun1 * 10 + 6];
state[115] = acadoVariables.od[lRun1 * 10 + 7];
state[116] = acadoVariables.od[lRun1 * 10 + 8];
state[117] = acadoVariables.od[lRun1 * 10 + 9];

ret = acado_integrate(state, 1);

acadoWorkspace.d[lRun1 * 8] = state[0] - acadoVariables.x[lRun1 * 8 + 8];
acadoWorkspace.d[lRun1 * 8 + 1] = state[1] - acadoVariables.x[lRun1 * 8 + 9];
acadoWorkspace.d[lRun1 * 8 + 2] = state[2] - acadoVariables.x[lRun1 * 8 + 10];
acadoWorkspace.d[lRun1 * 8 + 3] = state[3] - acadoVariables.x[lRun1 * 8 + 11];
acadoWorkspace.d[lRun1 * 8 + 4] = state[4] - acadoVariables.x[lRun1 * 8 + 12];
acadoWorkspace.d[lRun1 * 8 + 5] = state[5] - acadoVariables.x[lRun1 * 8 + 13];
acadoWorkspace.d[lRun1 * 8 + 6] = state[6] - acadoVariables.x[lRun1 * 8 + 14];
acadoWorkspace.d[lRun1 * 8 + 7] = state[7] - acadoVariables.x[lRun1 * 8 + 15];

acadoWorkspace.evGx[lRun1 * 64] = state[8];
acadoWorkspace.evGx[lRun1 * 64 + 1] = state[9];
acadoWorkspace.evGx[lRun1 * 64 + 2] = state[10];
acadoWorkspace.evGx[lRun1 * 64 + 3] = state[11];
acadoWorkspace.evGx[lRun1 * 64 + 4] = state[12];
acadoWorkspace.evGx[lRun1 * 64 + 5] = state[13];
acadoWorkspace.evGx[lRun1 * 64 + 6] = state[14];
acadoWorkspace.evGx[lRun1 * 64 + 7] = state[15];
acadoWorkspace.evGx[lRun1 * 64 + 8] = state[16];
acadoWorkspace.evGx[lRun1 * 64 + 9] = state[17];
acadoWorkspace.evGx[lRun1 * 64 + 10] = state[18];
acadoWorkspace.evGx[lRun1 * 64 + 11] = state[19];
acadoWorkspace.evGx[lRun1 * 64 + 12] = state[20];
acadoWorkspace.evGx[lRun1 * 64 + 13] = state[21];
acadoWorkspace.evGx[lRun1 * 64 + 14] = state[22];
acadoWorkspace.evGx[lRun1 * 64 + 15] = state[23];
acadoWorkspace.evGx[lRun1 * 64 + 16] = state[24];
acadoWorkspace.evGx[lRun1 * 64 + 17] = state[25];
acadoWorkspace.evGx[lRun1 * 64 + 18] = state[26];
acadoWorkspace.evGx[lRun1 * 64 + 19] = state[27];
acadoWorkspace.evGx[lRun1 * 64 + 20] = state[28];
acadoWorkspace.evGx[lRun1 * 64 + 21] = state[29];
acadoWorkspace.evGx[lRun1 * 64 + 22] = state[30];
acadoWorkspace.evGx[lRun1 * 64 + 23] = state[31];
acadoWorkspace.evGx[lRun1 * 64 + 24] = state[32];
acadoWorkspace.evGx[lRun1 * 64 + 25] = state[33];
acadoWorkspace.evGx[lRun1 * 64 + 26] = state[34];
acadoWorkspace.evGx[lRun1 * 64 + 27] = state[35];
acadoWorkspace.evGx[lRun1 * 64 + 28] = state[36];
acadoWorkspace.evGx[lRun1 * 64 + 29] = state[37];
acadoWorkspace.evGx[lRun1 * 64 + 30] = state[38];
acadoWorkspace.evGx[lRun1 * 64 + 31] = state[39];
acadoWorkspace.evGx[lRun1 * 64 + 32] = state[40];
acadoWorkspace.evGx[lRun1 * 64 + 33] = state[41];
acadoWorkspace.evGx[lRun1 * 64 + 34] = state[42];
acadoWorkspace.evGx[lRun1 * 64 + 35] = state[43];
acadoWorkspace.evGx[lRun1 * 64 + 36] = state[44];
acadoWorkspace.evGx[lRun1 * 64 + 37] = state[45];
acadoWorkspace.evGx[lRun1 * 64 + 38] = state[46];
acadoWorkspace.evGx[lRun1 * 64 + 39] = state[47];
acadoWorkspace.evGx[lRun1 * 64 + 40] = state[48];
acadoWorkspace.evGx[lRun1 * 64 + 41] = state[49];
acadoWorkspace.evGx[lRun1 * 64 + 42] = state[50];
acadoWorkspace.evGx[lRun1 * 64 + 43] = state[51];
acadoWorkspace.evGx[lRun1 * 64 + 44] = state[52];
acadoWorkspace.evGx[lRun1 * 64 + 45] = state[53];
acadoWorkspace.evGx[lRun1 * 64 + 46] = state[54];
acadoWorkspace.evGx[lRun1 * 64 + 47] = state[55];
acadoWorkspace.evGx[lRun1 * 64 + 48] = state[56];
acadoWorkspace.evGx[lRun1 * 64 + 49] = state[57];
acadoWorkspace.evGx[lRun1 * 64 + 50] = state[58];
acadoWorkspace.evGx[lRun1 * 64 + 51] = state[59];
acadoWorkspace.evGx[lRun1 * 64 + 52] = state[60];
acadoWorkspace.evGx[lRun1 * 64 + 53] = state[61];
acadoWorkspace.evGx[lRun1 * 64 + 54] = state[62];
acadoWorkspace.evGx[lRun1 * 64 + 55] = state[63];
acadoWorkspace.evGx[lRun1 * 64 + 56] = state[64];
acadoWorkspace.evGx[lRun1 * 64 + 57] = state[65];
acadoWorkspace.evGx[lRun1 * 64 + 58] = state[66];
acadoWorkspace.evGx[lRun1 * 64 + 59] = state[67];
acadoWorkspace.evGx[lRun1 * 64 + 60] = state[68];
acadoWorkspace.evGx[lRun1 * 64 + 61] = state[69];
acadoWorkspace.evGx[lRun1 * 64 + 62] = state[70];
acadoWorkspace.evGx[lRun1 * 64 + 63] = state[71];

acadoWorkspace.evGu[lRun1 * 32] = state[72];
acadoWorkspace.evGu[lRun1 * 32 + 1] = state[73];
acadoWorkspace.evGu[lRun1 * 32 + 2] = state[74];
acadoWorkspace.evGu[lRun1 * 32 + 3] = state[75];
acadoWorkspace.evGu[lRun1 * 32 + 4] = state[76];
acadoWorkspace.evGu[lRun1 * 32 + 5] = state[77];
acadoWorkspace.evGu[lRun1 * 32 + 6] = state[78];
acadoWorkspace.evGu[lRun1 * 32 + 7] = state[79];
acadoWorkspace.evGu[lRun1 * 32 + 8] = state[80];
acadoWorkspace.evGu[lRun1 * 32 + 9] = state[81];
acadoWorkspace.evGu[lRun1 * 32 + 10] = state[82];
acadoWorkspace.evGu[lRun1 * 32 + 11] = state[83];
acadoWorkspace.evGu[lRun1 * 32 + 12] = state[84];
acadoWorkspace.evGu[lRun1 * 32 + 13] = state[85];
acadoWorkspace.evGu[lRun1 * 32 + 14] = state[86];
acadoWorkspace.evGu[lRun1 * 32 + 15] = state[87];
acadoWorkspace.evGu[lRun1 * 32 + 16] = state[88];
acadoWorkspace.evGu[lRun1 * 32 + 17] = state[89];
acadoWorkspace.evGu[lRun1 * 32 + 18] = state[90];
acadoWorkspace.evGu[lRun1 * 32 + 19] = state[91];
acadoWorkspace.evGu[lRun1 * 32 + 20] = state[92];
acadoWorkspace.evGu[lRun1 * 32 + 21] = state[93];
acadoWorkspace.evGu[lRun1 * 32 + 22] = state[94];
acadoWorkspace.evGu[lRun1 * 32 + 23] = state[95];
acadoWorkspace.evGu[lRun1 * 32 + 24] = state[96];
acadoWorkspace.evGu[lRun1 * 32 + 25] = state[97];
acadoWorkspace.evGu[lRun1 * 32 + 26] = state[98];
acadoWorkspace.evGu[lRun1 * 32 + 27] = state[99];
acadoWorkspace.evGu[lRun1 * 32 + 28] = state[100];
acadoWorkspace.evGu[lRun1 * 32 + 29] = state[101];
acadoWorkspace.evGu[lRun1 * 32 + 30] = state[102];
acadoWorkspace.evGu[lRun1 * 32 + 31] = state[103];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 8;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = xd[6];
out[7] = xd[7];
out[8] = u[0];
out[9] = u[1];
out[10] = u[2];
out[11] = u[3];
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = xd[6];
out[7] = xd[7];
}

void acado_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
tmpQ2[0] = +tmpObjS[0];
tmpQ2[1] = +tmpObjS[1];
tmpQ2[2] = +tmpObjS[2];
tmpQ2[3] = +tmpObjS[3];
tmpQ2[4] = +tmpObjS[4];
tmpQ2[5] = +tmpObjS[5];
tmpQ2[6] = +tmpObjS[6];
tmpQ2[7] = +tmpObjS[7];
tmpQ2[8] = +tmpObjS[8];
tmpQ2[9] = +tmpObjS[9];
tmpQ2[10] = +tmpObjS[10];
tmpQ2[11] = +tmpObjS[11];
tmpQ2[12] = +tmpObjS[12];
tmpQ2[13] = +tmpObjS[13];
tmpQ2[14] = +tmpObjS[14];
tmpQ2[15] = +tmpObjS[15];
tmpQ2[16] = +tmpObjS[16];
tmpQ2[17] = +tmpObjS[17];
tmpQ2[18] = +tmpObjS[18];
tmpQ2[19] = +tmpObjS[19];
tmpQ2[20] = +tmpObjS[20];
tmpQ2[21] = +tmpObjS[21];
tmpQ2[22] = +tmpObjS[22];
tmpQ2[23] = +tmpObjS[23];
tmpQ2[24] = +tmpObjS[24];
tmpQ2[25] = +tmpObjS[25];
tmpQ2[26] = +tmpObjS[26];
tmpQ2[27] = +tmpObjS[27];
tmpQ2[28] = +tmpObjS[28];
tmpQ2[29] = +tmpObjS[29];
tmpQ2[30] = +tmpObjS[30];
tmpQ2[31] = +tmpObjS[31];
tmpQ2[32] = +tmpObjS[32];
tmpQ2[33] = +tmpObjS[33];
tmpQ2[34] = +tmpObjS[34];
tmpQ2[35] = +tmpObjS[35];
tmpQ2[36] = +tmpObjS[36];
tmpQ2[37] = +tmpObjS[37];
tmpQ2[38] = +tmpObjS[38];
tmpQ2[39] = +tmpObjS[39];
tmpQ2[40] = +tmpObjS[40];
tmpQ2[41] = +tmpObjS[41];
tmpQ2[42] = +tmpObjS[42];
tmpQ2[43] = +tmpObjS[43];
tmpQ2[44] = +tmpObjS[44];
tmpQ2[45] = +tmpObjS[45];
tmpQ2[46] = +tmpObjS[46];
tmpQ2[47] = +tmpObjS[47];
tmpQ2[48] = +tmpObjS[48];
tmpQ2[49] = +tmpObjS[49];
tmpQ2[50] = +tmpObjS[50];
tmpQ2[51] = +tmpObjS[51];
tmpQ2[52] = +tmpObjS[52];
tmpQ2[53] = +tmpObjS[53];
tmpQ2[54] = +tmpObjS[54];
tmpQ2[55] = +tmpObjS[55];
tmpQ2[56] = +tmpObjS[56];
tmpQ2[57] = +tmpObjS[57];
tmpQ2[58] = +tmpObjS[58];
tmpQ2[59] = +tmpObjS[59];
tmpQ2[60] = +tmpObjS[60];
tmpQ2[61] = +tmpObjS[61];
tmpQ2[62] = +tmpObjS[62];
tmpQ2[63] = +tmpObjS[63];
tmpQ2[64] = +tmpObjS[64];
tmpQ2[65] = +tmpObjS[65];
tmpQ2[66] = +tmpObjS[66];
tmpQ2[67] = +tmpObjS[67];
tmpQ2[68] = +tmpObjS[68];
tmpQ2[69] = +tmpObjS[69];
tmpQ2[70] = +tmpObjS[70];
tmpQ2[71] = +tmpObjS[71];
tmpQ2[72] = +tmpObjS[72];
tmpQ2[73] = +tmpObjS[73];
tmpQ2[74] = +tmpObjS[74];
tmpQ2[75] = +tmpObjS[75];
tmpQ2[76] = +tmpObjS[76];
tmpQ2[77] = +tmpObjS[77];
tmpQ2[78] = +tmpObjS[78];
tmpQ2[79] = +tmpObjS[79];
tmpQ2[80] = +tmpObjS[80];
tmpQ2[81] = +tmpObjS[81];
tmpQ2[82] = +tmpObjS[82];
tmpQ2[83] = +tmpObjS[83];
tmpQ2[84] = +tmpObjS[84];
tmpQ2[85] = +tmpObjS[85];
tmpQ2[86] = +tmpObjS[86];
tmpQ2[87] = +tmpObjS[87];
tmpQ2[88] = +tmpObjS[88];
tmpQ2[89] = +tmpObjS[89];
tmpQ2[90] = +tmpObjS[90];
tmpQ2[91] = +tmpObjS[91];
tmpQ2[92] = +tmpObjS[92];
tmpQ2[93] = +tmpObjS[93];
tmpQ2[94] = +tmpObjS[94];
tmpQ2[95] = +tmpObjS[95];
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = + tmpQ2[2];
tmpQ1[3] = + tmpQ2[3];
tmpQ1[4] = + tmpQ2[4];
tmpQ1[5] = + tmpQ2[5];
tmpQ1[6] = + tmpQ2[6];
tmpQ1[7] = + tmpQ2[7];
tmpQ1[8] = + tmpQ2[12];
tmpQ1[9] = + tmpQ2[13];
tmpQ1[10] = + tmpQ2[14];
tmpQ1[11] = + tmpQ2[15];
tmpQ1[12] = + tmpQ2[16];
tmpQ1[13] = + tmpQ2[17];
tmpQ1[14] = + tmpQ2[18];
tmpQ1[15] = + tmpQ2[19];
tmpQ1[16] = + tmpQ2[24];
tmpQ1[17] = + tmpQ2[25];
tmpQ1[18] = + tmpQ2[26];
tmpQ1[19] = + tmpQ2[27];
tmpQ1[20] = + tmpQ2[28];
tmpQ1[21] = + tmpQ2[29];
tmpQ1[22] = + tmpQ2[30];
tmpQ1[23] = + tmpQ2[31];
tmpQ1[24] = + tmpQ2[36];
tmpQ1[25] = + tmpQ2[37];
tmpQ1[26] = + tmpQ2[38];
tmpQ1[27] = + tmpQ2[39];
tmpQ1[28] = + tmpQ2[40];
tmpQ1[29] = + tmpQ2[41];
tmpQ1[30] = + tmpQ2[42];
tmpQ1[31] = + tmpQ2[43];
tmpQ1[32] = + tmpQ2[48];
tmpQ1[33] = + tmpQ2[49];
tmpQ1[34] = + tmpQ2[50];
tmpQ1[35] = + tmpQ2[51];
tmpQ1[36] = + tmpQ2[52];
tmpQ1[37] = + tmpQ2[53];
tmpQ1[38] = + tmpQ2[54];
tmpQ1[39] = + tmpQ2[55];
tmpQ1[40] = + tmpQ2[60];
tmpQ1[41] = + tmpQ2[61];
tmpQ1[42] = + tmpQ2[62];
tmpQ1[43] = + tmpQ2[63];
tmpQ1[44] = + tmpQ2[64];
tmpQ1[45] = + tmpQ2[65];
tmpQ1[46] = + tmpQ2[66];
tmpQ1[47] = + tmpQ2[67];
tmpQ1[48] = + tmpQ2[72];
tmpQ1[49] = + tmpQ2[73];
tmpQ1[50] = + tmpQ2[74];
tmpQ1[51] = + tmpQ2[75];
tmpQ1[52] = + tmpQ2[76];
tmpQ1[53] = + tmpQ2[77];
tmpQ1[54] = + tmpQ2[78];
tmpQ1[55] = + tmpQ2[79];
tmpQ1[56] = + tmpQ2[84];
tmpQ1[57] = + tmpQ2[85];
tmpQ1[58] = + tmpQ2[86];
tmpQ1[59] = + tmpQ2[87];
tmpQ1[60] = + tmpQ2[88];
tmpQ1[61] = + tmpQ2[89];
tmpQ1[62] = + tmpQ2[90];
tmpQ1[63] = + tmpQ2[91];
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[96];
tmpR2[1] = +tmpObjS[97];
tmpR2[2] = +tmpObjS[98];
tmpR2[3] = +tmpObjS[99];
tmpR2[4] = +tmpObjS[100];
tmpR2[5] = +tmpObjS[101];
tmpR2[6] = +tmpObjS[102];
tmpR2[7] = +tmpObjS[103];
tmpR2[8] = +tmpObjS[104];
tmpR2[9] = +tmpObjS[105];
tmpR2[10] = +tmpObjS[106];
tmpR2[11] = +tmpObjS[107];
tmpR2[12] = +tmpObjS[108];
tmpR2[13] = +tmpObjS[109];
tmpR2[14] = +tmpObjS[110];
tmpR2[15] = +tmpObjS[111];
tmpR2[16] = +tmpObjS[112];
tmpR2[17] = +tmpObjS[113];
tmpR2[18] = +tmpObjS[114];
tmpR2[19] = +tmpObjS[115];
tmpR2[20] = +tmpObjS[116];
tmpR2[21] = +tmpObjS[117];
tmpR2[22] = +tmpObjS[118];
tmpR2[23] = +tmpObjS[119];
tmpR2[24] = +tmpObjS[120];
tmpR2[25] = +tmpObjS[121];
tmpR2[26] = +tmpObjS[122];
tmpR2[27] = +tmpObjS[123];
tmpR2[28] = +tmpObjS[124];
tmpR2[29] = +tmpObjS[125];
tmpR2[30] = +tmpObjS[126];
tmpR2[31] = +tmpObjS[127];
tmpR2[32] = +tmpObjS[128];
tmpR2[33] = +tmpObjS[129];
tmpR2[34] = +tmpObjS[130];
tmpR2[35] = +tmpObjS[131];
tmpR2[36] = +tmpObjS[132];
tmpR2[37] = +tmpObjS[133];
tmpR2[38] = +tmpObjS[134];
tmpR2[39] = +tmpObjS[135];
tmpR2[40] = +tmpObjS[136];
tmpR2[41] = +tmpObjS[137];
tmpR2[42] = +tmpObjS[138];
tmpR2[43] = +tmpObjS[139];
tmpR2[44] = +tmpObjS[140];
tmpR2[45] = +tmpObjS[141];
tmpR2[46] = +tmpObjS[142];
tmpR2[47] = +tmpObjS[143];
tmpR1[0] = + tmpR2[8];
tmpR1[1] = + tmpR2[9];
tmpR1[2] = + tmpR2[10];
tmpR1[3] = + tmpR2[11];
tmpR1[4] = + tmpR2[20];
tmpR1[5] = + tmpR2[21];
tmpR1[6] = + tmpR2[22];
tmpR1[7] = + tmpR2[23];
tmpR1[8] = + tmpR2[32];
tmpR1[9] = + tmpR2[33];
tmpR1[10] = + tmpR2[34];
tmpR1[11] = + tmpR2[35];
tmpR1[12] = + tmpR2[44];
tmpR1[13] = + tmpR2[45];
tmpR1[14] = + tmpR2[46];
tmpR1[15] = + tmpR2[47];
}

void acado_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
tmpQN2[0] = +tmpObjSEndTerm[0];
tmpQN2[1] = +tmpObjSEndTerm[1];
tmpQN2[2] = +tmpObjSEndTerm[2];
tmpQN2[3] = +tmpObjSEndTerm[3];
tmpQN2[4] = +tmpObjSEndTerm[4];
tmpQN2[5] = +tmpObjSEndTerm[5];
tmpQN2[6] = +tmpObjSEndTerm[6];
tmpQN2[7] = +tmpObjSEndTerm[7];
tmpQN2[8] = +tmpObjSEndTerm[8];
tmpQN2[9] = +tmpObjSEndTerm[9];
tmpQN2[10] = +tmpObjSEndTerm[10];
tmpQN2[11] = +tmpObjSEndTerm[11];
tmpQN2[12] = +tmpObjSEndTerm[12];
tmpQN2[13] = +tmpObjSEndTerm[13];
tmpQN2[14] = +tmpObjSEndTerm[14];
tmpQN2[15] = +tmpObjSEndTerm[15];
tmpQN2[16] = +tmpObjSEndTerm[16];
tmpQN2[17] = +tmpObjSEndTerm[17];
tmpQN2[18] = +tmpObjSEndTerm[18];
tmpQN2[19] = +tmpObjSEndTerm[19];
tmpQN2[20] = +tmpObjSEndTerm[20];
tmpQN2[21] = +tmpObjSEndTerm[21];
tmpQN2[22] = +tmpObjSEndTerm[22];
tmpQN2[23] = +tmpObjSEndTerm[23];
tmpQN2[24] = +tmpObjSEndTerm[24];
tmpQN2[25] = +tmpObjSEndTerm[25];
tmpQN2[26] = +tmpObjSEndTerm[26];
tmpQN2[27] = +tmpObjSEndTerm[27];
tmpQN2[28] = +tmpObjSEndTerm[28];
tmpQN2[29] = +tmpObjSEndTerm[29];
tmpQN2[30] = +tmpObjSEndTerm[30];
tmpQN2[31] = +tmpObjSEndTerm[31];
tmpQN2[32] = +tmpObjSEndTerm[32];
tmpQN2[33] = +tmpObjSEndTerm[33];
tmpQN2[34] = +tmpObjSEndTerm[34];
tmpQN2[35] = +tmpObjSEndTerm[35];
tmpQN2[36] = +tmpObjSEndTerm[36];
tmpQN2[37] = +tmpObjSEndTerm[37];
tmpQN2[38] = +tmpObjSEndTerm[38];
tmpQN2[39] = +tmpObjSEndTerm[39];
tmpQN2[40] = +tmpObjSEndTerm[40];
tmpQN2[41] = +tmpObjSEndTerm[41];
tmpQN2[42] = +tmpObjSEndTerm[42];
tmpQN2[43] = +tmpObjSEndTerm[43];
tmpQN2[44] = +tmpObjSEndTerm[44];
tmpQN2[45] = +tmpObjSEndTerm[45];
tmpQN2[46] = +tmpObjSEndTerm[46];
tmpQN2[47] = +tmpObjSEndTerm[47];
tmpQN2[48] = +tmpObjSEndTerm[48];
tmpQN2[49] = +tmpObjSEndTerm[49];
tmpQN2[50] = +tmpObjSEndTerm[50];
tmpQN2[51] = +tmpObjSEndTerm[51];
tmpQN2[52] = +tmpObjSEndTerm[52];
tmpQN2[53] = +tmpObjSEndTerm[53];
tmpQN2[54] = +tmpObjSEndTerm[54];
tmpQN2[55] = +tmpObjSEndTerm[55];
tmpQN2[56] = +tmpObjSEndTerm[56];
tmpQN2[57] = +tmpObjSEndTerm[57];
tmpQN2[58] = +tmpObjSEndTerm[58];
tmpQN2[59] = +tmpObjSEndTerm[59];
tmpQN2[60] = +tmpObjSEndTerm[60];
tmpQN2[61] = +tmpObjSEndTerm[61];
tmpQN2[62] = +tmpObjSEndTerm[62];
tmpQN2[63] = +tmpObjSEndTerm[63];
tmpQN1[0] = + tmpQN2[0];
tmpQN1[1] = + tmpQN2[1];
tmpQN1[2] = + tmpQN2[2];
tmpQN1[3] = + tmpQN2[3];
tmpQN1[4] = + tmpQN2[4];
tmpQN1[5] = + tmpQN2[5];
tmpQN1[6] = + tmpQN2[6];
tmpQN1[7] = + tmpQN2[7];
tmpQN1[8] = + tmpQN2[8];
tmpQN1[9] = + tmpQN2[9];
tmpQN1[10] = + tmpQN2[10];
tmpQN1[11] = + tmpQN2[11];
tmpQN1[12] = + tmpQN2[12];
tmpQN1[13] = + tmpQN2[13];
tmpQN1[14] = + tmpQN2[14];
tmpQN1[15] = + tmpQN2[15];
tmpQN1[16] = + tmpQN2[16];
tmpQN1[17] = + tmpQN2[17];
tmpQN1[18] = + tmpQN2[18];
tmpQN1[19] = + tmpQN2[19];
tmpQN1[20] = + tmpQN2[20];
tmpQN1[21] = + tmpQN2[21];
tmpQN1[22] = + tmpQN2[22];
tmpQN1[23] = + tmpQN2[23];
tmpQN1[24] = + tmpQN2[24];
tmpQN1[25] = + tmpQN2[25];
tmpQN1[26] = + tmpQN2[26];
tmpQN1[27] = + tmpQN2[27];
tmpQN1[28] = + tmpQN2[28];
tmpQN1[29] = + tmpQN2[29];
tmpQN1[30] = + tmpQN2[30];
tmpQN1[31] = + tmpQN2[31];
tmpQN1[32] = + tmpQN2[32];
tmpQN1[33] = + tmpQN2[33];
tmpQN1[34] = + tmpQN2[34];
tmpQN1[35] = + tmpQN2[35];
tmpQN1[36] = + tmpQN2[36];
tmpQN1[37] = + tmpQN2[37];
tmpQN1[38] = + tmpQN2[38];
tmpQN1[39] = + tmpQN2[39];
tmpQN1[40] = + tmpQN2[40];
tmpQN1[41] = + tmpQN2[41];
tmpQN1[42] = + tmpQN2[42];
tmpQN1[43] = + tmpQN2[43];
tmpQN1[44] = + tmpQN2[44];
tmpQN1[45] = + tmpQN2[45];
tmpQN1[46] = + tmpQN2[46];
tmpQN1[47] = + tmpQN2[47];
tmpQN1[48] = + tmpQN2[48];
tmpQN1[49] = + tmpQN2[49];
tmpQN1[50] = + tmpQN2[50];
tmpQN1[51] = + tmpQN2[51];
tmpQN1[52] = + tmpQN2[52];
tmpQN1[53] = + tmpQN2[53];
tmpQN1[54] = + tmpQN2[54];
tmpQN1[55] = + tmpQN2[55];
tmpQN1[56] = + tmpQN2[56];
tmpQN1[57] = + tmpQN2[57];
tmpQN1[58] = + tmpQN2[58];
tmpQN1[59] = + tmpQN2[59];
tmpQN1[60] = + tmpQN2[60];
tmpQN1[61] = + tmpQN2[61];
tmpQN1[62] = + tmpQN2[62];
tmpQN1[63] = + tmpQN2[63];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 20; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 8];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 8 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 8 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 8 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 8 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[runObj * 8 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[runObj * 8 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[runObj * 8 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.u[runObj * 4];
acadoWorkspace.objValueIn[9] = acadoVariables.u[runObj * 4 + 1];
acadoWorkspace.objValueIn[10] = acadoVariables.u[runObj * 4 + 2];
acadoWorkspace.objValueIn[11] = acadoVariables.u[runObj * 4 + 3];
acadoWorkspace.objValueIn[12] = acadoVariables.od[runObj * 10];
acadoWorkspace.objValueIn[13] = acadoVariables.od[runObj * 10 + 1];
acadoWorkspace.objValueIn[14] = acadoVariables.od[runObj * 10 + 2];
acadoWorkspace.objValueIn[15] = acadoVariables.od[runObj * 10 + 3];
acadoWorkspace.objValueIn[16] = acadoVariables.od[runObj * 10 + 4];
acadoWorkspace.objValueIn[17] = acadoVariables.od[runObj * 10 + 5];
acadoWorkspace.objValueIn[18] = acadoVariables.od[runObj * 10 + 6];
acadoWorkspace.objValueIn[19] = acadoVariables.od[runObj * 10 + 7];
acadoWorkspace.objValueIn[20] = acadoVariables.od[runObj * 10 + 8];
acadoWorkspace.objValueIn[21] = acadoVariables.od[runObj * 10 + 9];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 12] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 12 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 12 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 12 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 12 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 12 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 12 + 6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.Dy[runObj * 12 + 7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.Dy[runObj * 12 + 8] = acadoWorkspace.objValueOut[8];
acadoWorkspace.Dy[runObj * 12 + 9] = acadoWorkspace.objValueOut[9];
acadoWorkspace.Dy[runObj * 12 + 10] = acadoWorkspace.objValueOut[10];
acadoWorkspace.Dy[runObj * 12 + 11] = acadoWorkspace.objValueOut[11];

acado_setObjQ1Q2( &(acadoVariables.W[ runObj * 144 ]), &(acadoWorkspace.Q1[ runObj * 64 ]), &(acadoWorkspace.Q2[ runObj * 96 ]) );

acado_setObjR1R2( &(acadoVariables.W[ runObj * 144 ]), &(acadoWorkspace.R1[ runObj * 16 ]), &(acadoWorkspace.R2[ runObj * 48 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[160];
acadoWorkspace.objValueIn[1] = acadoVariables.x[161];
acadoWorkspace.objValueIn[2] = acadoVariables.x[162];
acadoWorkspace.objValueIn[3] = acadoVariables.x[163];
acadoWorkspace.objValueIn[4] = acadoVariables.x[164];
acadoWorkspace.objValueIn[5] = acadoVariables.x[165];
acadoWorkspace.objValueIn[6] = acadoVariables.x[166];
acadoWorkspace.objValueIn[7] = acadoVariables.x[167];
acadoWorkspace.objValueIn[8] = acadoVariables.od[200];
acadoWorkspace.objValueIn[9] = acadoVariables.od[201];
acadoWorkspace.objValueIn[10] = acadoVariables.od[202];
acadoWorkspace.objValueIn[11] = acadoVariables.od[203];
acadoWorkspace.objValueIn[12] = acadoVariables.od[204];
acadoWorkspace.objValueIn[13] = acadoVariables.od[205];
acadoWorkspace.objValueIn[14] = acadoVariables.od[206];
acadoWorkspace.objValueIn[15] = acadoVariables.od[207];
acadoWorkspace.objValueIn[16] = acadoVariables.od[208];
acadoWorkspace.objValueIn[17] = acadoVariables.od[209];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.DyN[6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.DyN[7] = acadoWorkspace.objValueOut[7];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[12] + Gx1[4]*Gu1[16] + Gx1[5]*Gu1[20] + Gx1[6]*Gu1[24] + Gx1[7]*Gu1[28];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[9] + Gx1[3]*Gu1[13] + Gx1[4]*Gu1[17] + Gx1[5]*Gu1[21] + Gx1[6]*Gu1[25] + Gx1[7]*Gu1[29];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[6] + Gx1[2]*Gu1[10] + Gx1[3]*Gu1[14] + Gx1[4]*Gu1[18] + Gx1[5]*Gu1[22] + Gx1[6]*Gu1[26] + Gx1[7]*Gu1[30];
Gu2[3] = + Gx1[0]*Gu1[3] + Gx1[1]*Gu1[7] + Gx1[2]*Gu1[11] + Gx1[3]*Gu1[15] + Gx1[4]*Gu1[19] + Gx1[5]*Gu1[23] + Gx1[6]*Gu1[27] + Gx1[7]*Gu1[31];
Gu2[4] = + Gx1[8]*Gu1[0] + Gx1[9]*Gu1[4] + Gx1[10]*Gu1[8] + Gx1[11]*Gu1[12] + Gx1[12]*Gu1[16] + Gx1[13]*Gu1[20] + Gx1[14]*Gu1[24] + Gx1[15]*Gu1[28];
Gu2[5] = + Gx1[8]*Gu1[1] + Gx1[9]*Gu1[5] + Gx1[10]*Gu1[9] + Gx1[11]*Gu1[13] + Gx1[12]*Gu1[17] + Gx1[13]*Gu1[21] + Gx1[14]*Gu1[25] + Gx1[15]*Gu1[29];
Gu2[6] = + Gx1[8]*Gu1[2] + Gx1[9]*Gu1[6] + Gx1[10]*Gu1[10] + Gx1[11]*Gu1[14] + Gx1[12]*Gu1[18] + Gx1[13]*Gu1[22] + Gx1[14]*Gu1[26] + Gx1[15]*Gu1[30];
Gu2[7] = + Gx1[8]*Gu1[3] + Gx1[9]*Gu1[7] + Gx1[10]*Gu1[11] + Gx1[11]*Gu1[15] + Gx1[12]*Gu1[19] + Gx1[13]*Gu1[23] + Gx1[14]*Gu1[27] + Gx1[15]*Gu1[31];
Gu2[8] = + Gx1[16]*Gu1[0] + Gx1[17]*Gu1[4] + Gx1[18]*Gu1[8] + Gx1[19]*Gu1[12] + Gx1[20]*Gu1[16] + Gx1[21]*Gu1[20] + Gx1[22]*Gu1[24] + Gx1[23]*Gu1[28];
Gu2[9] = + Gx1[16]*Gu1[1] + Gx1[17]*Gu1[5] + Gx1[18]*Gu1[9] + Gx1[19]*Gu1[13] + Gx1[20]*Gu1[17] + Gx1[21]*Gu1[21] + Gx1[22]*Gu1[25] + Gx1[23]*Gu1[29];
Gu2[10] = + Gx1[16]*Gu1[2] + Gx1[17]*Gu1[6] + Gx1[18]*Gu1[10] + Gx1[19]*Gu1[14] + Gx1[20]*Gu1[18] + Gx1[21]*Gu1[22] + Gx1[22]*Gu1[26] + Gx1[23]*Gu1[30];
Gu2[11] = + Gx1[16]*Gu1[3] + Gx1[17]*Gu1[7] + Gx1[18]*Gu1[11] + Gx1[19]*Gu1[15] + Gx1[20]*Gu1[19] + Gx1[21]*Gu1[23] + Gx1[22]*Gu1[27] + Gx1[23]*Gu1[31];
Gu2[12] = + Gx1[24]*Gu1[0] + Gx1[25]*Gu1[4] + Gx1[26]*Gu1[8] + Gx1[27]*Gu1[12] + Gx1[28]*Gu1[16] + Gx1[29]*Gu1[20] + Gx1[30]*Gu1[24] + Gx1[31]*Gu1[28];
Gu2[13] = + Gx1[24]*Gu1[1] + Gx1[25]*Gu1[5] + Gx1[26]*Gu1[9] + Gx1[27]*Gu1[13] + Gx1[28]*Gu1[17] + Gx1[29]*Gu1[21] + Gx1[30]*Gu1[25] + Gx1[31]*Gu1[29];
Gu2[14] = + Gx1[24]*Gu1[2] + Gx1[25]*Gu1[6] + Gx1[26]*Gu1[10] + Gx1[27]*Gu1[14] + Gx1[28]*Gu1[18] + Gx1[29]*Gu1[22] + Gx1[30]*Gu1[26] + Gx1[31]*Gu1[30];
Gu2[15] = + Gx1[24]*Gu1[3] + Gx1[25]*Gu1[7] + Gx1[26]*Gu1[11] + Gx1[27]*Gu1[15] + Gx1[28]*Gu1[19] + Gx1[29]*Gu1[23] + Gx1[30]*Gu1[27] + Gx1[31]*Gu1[31];
Gu2[16] = + Gx1[32]*Gu1[0] + Gx1[33]*Gu1[4] + Gx1[34]*Gu1[8] + Gx1[35]*Gu1[12] + Gx1[36]*Gu1[16] + Gx1[37]*Gu1[20] + Gx1[38]*Gu1[24] + Gx1[39]*Gu1[28];
Gu2[17] = + Gx1[32]*Gu1[1] + Gx1[33]*Gu1[5] + Gx1[34]*Gu1[9] + Gx1[35]*Gu1[13] + Gx1[36]*Gu1[17] + Gx1[37]*Gu1[21] + Gx1[38]*Gu1[25] + Gx1[39]*Gu1[29];
Gu2[18] = + Gx1[32]*Gu1[2] + Gx1[33]*Gu1[6] + Gx1[34]*Gu1[10] + Gx1[35]*Gu1[14] + Gx1[36]*Gu1[18] + Gx1[37]*Gu1[22] + Gx1[38]*Gu1[26] + Gx1[39]*Gu1[30];
Gu2[19] = + Gx1[32]*Gu1[3] + Gx1[33]*Gu1[7] + Gx1[34]*Gu1[11] + Gx1[35]*Gu1[15] + Gx1[36]*Gu1[19] + Gx1[37]*Gu1[23] + Gx1[38]*Gu1[27] + Gx1[39]*Gu1[31];
Gu2[20] = + Gx1[40]*Gu1[0] + Gx1[41]*Gu1[4] + Gx1[42]*Gu1[8] + Gx1[43]*Gu1[12] + Gx1[44]*Gu1[16] + Gx1[45]*Gu1[20] + Gx1[46]*Gu1[24] + Gx1[47]*Gu1[28];
Gu2[21] = + Gx1[40]*Gu1[1] + Gx1[41]*Gu1[5] + Gx1[42]*Gu1[9] + Gx1[43]*Gu1[13] + Gx1[44]*Gu1[17] + Gx1[45]*Gu1[21] + Gx1[46]*Gu1[25] + Gx1[47]*Gu1[29];
Gu2[22] = + Gx1[40]*Gu1[2] + Gx1[41]*Gu1[6] + Gx1[42]*Gu1[10] + Gx1[43]*Gu1[14] + Gx1[44]*Gu1[18] + Gx1[45]*Gu1[22] + Gx1[46]*Gu1[26] + Gx1[47]*Gu1[30];
Gu2[23] = + Gx1[40]*Gu1[3] + Gx1[41]*Gu1[7] + Gx1[42]*Gu1[11] + Gx1[43]*Gu1[15] + Gx1[44]*Gu1[19] + Gx1[45]*Gu1[23] + Gx1[46]*Gu1[27] + Gx1[47]*Gu1[31];
Gu2[24] = + Gx1[48]*Gu1[0] + Gx1[49]*Gu1[4] + Gx1[50]*Gu1[8] + Gx1[51]*Gu1[12] + Gx1[52]*Gu1[16] + Gx1[53]*Gu1[20] + Gx1[54]*Gu1[24] + Gx1[55]*Gu1[28];
Gu2[25] = + Gx1[48]*Gu1[1] + Gx1[49]*Gu1[5] + Gx1[50]*Gu1[9] + Gx1[51]*Gu1[13] + Gx1[52]*Gu1[17] + Gx1[53]*Gu1[21] + Gx1[54]*Gu1[25] + Gx1[55]*Gu1[29];
Gu2[26] = + Gx1[48]*Gu1[2] + Gx1[49]*Gu1[6] + Gx1[50]*Gu1[10] + Gx1[51]*Gu1[14] + Gx1[52]*Gu1[18] + Gx1[53]*Gu1[22] + Gx1[54]*Gu1[26] + Gx1[55]*Gu1[30];
Gu2[27] = + Gx1[48]*Gu1[3] + Gx1[49]*Gu1[7] + Gx1[50]*Gu1[11] + Gx1[51]*Gu1[15] + Gx1[52]*Gu1[19] + Gx1[53]*Gu1[23] + Gx1[54]*Gu1[27] + Gx1[55]*Gu1[31];
Gu2[28] = + Gx1[56]*Gu1[0] + Gx1[57]*Gu1[4] + Gx1[58]*Gu1[8] + Gx1[59]*Gu1[12] + Gx1[60]*Gu1[16] + Gx1[61]*Gu1[20] + Gx1[62]*Gu1[24] + Gx1[63]*Gu1[28];
Gu2[29] = + Gx1[56]*Gu1[1] + Gx1[57]*Gu1[5] + Gx1[58]*Gu1[9] + Gx1[59]*Gu1[13] + Gx1[60]*Gu1[17] + Gx1[61]*Gu1[21] + Gx1[62]*Gu1[25] + Gx1[63]*Gu1[29];
Gu2[30] = + Gx1[56]*Gu1[2] + Gx1[57]*Gu1[6] + Gx1[58]*Gu1[10] + Gx1[59]*Gu1[14] + Gx1[60]*Gu1[18] + Gx1[61]*Gu1[22] + Gx1[62]*Gu1[26] + Gx1[63]*Gu1[30];
Gu2[31] = + Gx1[56]*Gu1[3] + Gx1[57]*Gu1[7] + Gx1[58]*Gu1[11] + Gx1[59]*Gu1[15] + Gx1[60]*Gu1[19] + Gx1[61]*Gu1[23] + Gx1[62]*Gu1[27] + Gx1[63]*Gu1[31];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
Gu2[12] = Gu1[12];
Gu2[13] = Gu1[13];
Gu2[14] = Gu1[14];
Gu2[15] = Gu1[15];
Gu2[16] = Gu1[16];
Gu2[17] = Gu1[17];
Gu2[18] = Gu1[18];
Gu2[19] = Gu1[19];
Gu2[20] = Gu1[20];
Gu2[21] = Gu1[21];
Gu2[22] = Gu1[22];
Gu2[23] = Gu1[23];
Gu2[24] = Gu1[24];
Gu2[25] = Gu1[25];
Gu2[26] = Gu1[26];
Gu2[27] = Gu1[27];
Gu2[28] = Gu1[28];
Gu2[29] = Gu1[29];
Gu2[30] = Gu1[30];
Gu2[31] = Gu1[31];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 320) + (iCol * 4)] = + Gu1[0]*Gu2[0] + Gu1[4]*Gu2[4] + Gu1[8]*Gu2[8] + Gu1[12]*Gu2[12] + Gu1[16]*Gu2[16] + Gu1[20]*Gu2[20] + Gu1[24]*Gu2[24] + Gu1[28]*Gu2[28];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 1)] = + Gu1[0]*Gu2[1] + Gu1[4]*Gu2[5] + Gu1[8]*Gu2[9] + Gu1[12]*Gu2[13] + Gu1[16]*Gu2[17] + Gu1[20]*Gu2[21] + Gu1[24]*Gu2[25] + Gu1[28]*Gu2[29];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 2)] = + Gu1[0]*Gu2[2] + Gu1[4]*Gu2[6] + Gu1[8]*Gu2[10] + Gu1[12]*Gu2[14] + Gu1[16]*Gu2[18] + Gu1[20]*Gu2[22] + Gu1[24]*Gu2[26] + Gu1[28]*Gu2[30];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 3)] = + Gu1[0]*Gu2[3] + Gu1[4]*Gu2[7] + Gu1[8]*Gu2[11] + Gu1[12]*Gu2[15] + Gu1[16]*Gu2[19] + Gu1[20]*Gu2[23] + Gu1[24]*Gu2[27] + Gu1[28]*Gu2[31];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4)] = + Gu1[1]*Gu2[0] + Gu1[5]*Gu2[4] + Gu1[9]*Gu2[8] + Gu1[13]*Gu2[12] + Gu1[17]*Gu2[16] + Gu1[21]*Gu2[20] + Gu1[25]*Gu2[24] + Gu1[29]*Gu2[28];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 1)] = + Gu1[1]*Gu2[1] + Gu1[5]*Gu2[5] + Gu1[9]*Gu2[9] + Gu1[13]*Gu2[13] + Gu1[17]*Gu2[17] + Gu1[21]*Gu2[21] + Gu1[25]*Gu2[25] + Gu1[29]*Gu2[29];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 2)] = + Gu1[1]*Gu2[2] + Gu1[5]*Gu2[6] + Gu1[9]*Gu2[10] + Gu1[13]*Gu2[14] + Gu1[17]*Gu2[18] + Gu1[21]*Gu2[22] + Gu1[25]*Gu2[26] + Gu1[29]*Gu2[30];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 3)] = + Gu1[1]*Gu2[3] + Gu1[5]*Gu2[7] + Gu1[9]*Gu2[11] + Gu1[13]*Gu2[15] + Gu1[17]*Gu2[19] + Gu1[21]*Gu2[23] + Gu1[25]*Gu2[27] + Gu1[29]*Gu2[31];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4)] = + Gu1[2]*Gu2[0] + Gu1[6]*Gu2[4] + Gu1[10]*Gu2[8] + Gu1[14]*Gu2[12] + Gu1[18]*Gu2[16] + Gu1[22]*Gu2[20] + Gu1[26]*Gu2[24] + Gu1[30]*Gu2[28];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 1)] = + Gu1[2]*Gu2[1] + Gu1[6]*Gu2[5] + Gu1[10]*Gu2[9] + Gu1[14]*Gu2[13] + Gu1[18]*Gu2[17] + Gu1[22]*Gu2[21] + Gu1[26]*Gu2[25] + Gu1[30]*Gu2[29];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 2)] = + Gu1[2]*Gu2[2] + Gu1[6]*Gu2[6] + Gu1[10]*Gu2[10] + Gu1[14]*Gu2[14] + Gu1[18]*Gu2[18] + Gu1[22]*Gu2[22] + Gu1[26]*Gu2[26] + Gu1[30]*Gu2[30];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 3)] = + Gu1[2]*Gu2[3] + Gu1[6]*Gu2[7] + Gu1[10]*Gu2[11] + Gu1[14]*Gu2[15] + Gu1[18]*Gu2[19] + Gu1[22]*Gu2[23] + Gu1[26]*Gu2[27] + Gu1[30]*Gu2[31];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4)] = + Gu1[3]*Gu2[0] + Gu1[7]*Gu2[4] + Gu1[11]*Gu2[8] + Gu1[15]*Gu2[12] + Gu1[19]*Gu2[16] + Gu1[23]*Gu2[20] + Gu1[27]*Gu2[24] + Gu1[31]*Gu2[28];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 1)] = + Gu1[3]*Gu2[1] + Gu1[7]*Gu2[5] + Gu1[11]*Gu2[9] + Gu1[15]*Gu2[13] + Gu1[19]*Gu2[17] + Gu1[23]*Gu2[21] + Gu1[27]*Gu2[25] + Gu1[31]*Gu2[29];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 2)] = + Gu1[3]*Gu2[2] + Gu1[7]*Gu2[6] + Gu1[11]*Gu2[10] + Gu1[15]*Gu2[14] + Gu1[19]*Gu2[18] + Gu1[23]*Gu2[22] + Gu1[27]*Gu2[26] + Gu1[31]*Gu2[30];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 3)] = + Gu1[3]*Gu2[3] + Gu1[7]*Gu2[7] + Gu1[11]*Gu2[11] + Gu1[15]*Gu2[15] + Gu1[19]*Gu2[19] + Gu1[23]*Gu2[23] + Gu1[27]*Gu2[27] + Gu1[31]*Gu2[31];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 324] = + Gu1[0]*Gu2[0] + Gu1[4]*Gu2[4] + Gu1[8]*Gu2[8] + Gu1[12]*Gu2[12] + Gu1[16]*Gu2[16] + Gu1[20]*Gu2[20] + Gu1[24]*Gu2[24] + Gu1[28]*Gu2[28] + R11[0];
acadoWorkspace.H[iRow * 324 + 1] = + Gu1[0]*Gu2[1] + Gu1[4]*Gu2[5] + Gu1[8]*Gu2[9] + Gu1[12]*Gu2[13] + Gu1[16]*Gu2[17] + Gu1[20]*Gu2[21] + Gu1[24]*Gu2[25] + Gu1[28]*Gu2[29] + R11[1];
acadoWorkspace.H[iRow * 324 + 2] = + Gu1[0]*Gu2[2] + Gu1[4]*Gu2[6] + Gu1[8]*Gu2[10] + Gu1[12]*Gu2[14] + Gu1[16]*Gu2[18] + Gu1[20]*Gu2[22] + Gu1[24]*Gu2[26] + Gu1[28]*Gu2[30] + R11[2];
acadoWorkspace.H[iRow * 324 + 3] = + Gu1[0]*Gu2[3] + Gu1[4]*Gu2[7] + Gu1[8]*Gu2[11] + Gu1[12]*Gu2[15] + Gu1[16]*Gu2[19] + Gu1[20]*Gu2[23] + Gu1[24]*Gu2[27] + Gu1[28]*Gu2[31] + R11[3];
acadoWorkspace.H[iRow * 324 + 80] = + Gu1[1]*Gu2[0] + Gu1[5]*Gu2[4] + Gu1[9]*Gu2[8] + Gu1[13]*Gu2[12] + Gu1[17]*Gu2[16] + Gu1[21]*Gu2[20] + Gu1[25]*Gu2[24] + Gu1[29]*Gu2[28] + R11[4];
acadoWorkspace.H[iRow * 324 + 81] = + Gu1[1]*Gu2[1] + Gu1[5]*Gu2[5] + Gu1[9]*Gu2[9] + Gu1[13]*Gu2[13] + Gu1[17]*Gu2[17] + Gu1[21]*Gu2[21] + Gu1[25]*Gu2[25] + Gu1[29]*Gu2[29] + R11[5];
acadoWorkspace.H[iRow * 324 + 82] = + Gu1[1]*Gu2[2] + Gu1[5]*Gu2[6] + Gu1[9]*Gu2[10] + Gu1[13]*Gu2[14] + Gu1[17]*Gu2[18] + Gu1[21]*Gu2[22] + Gu1[25]*Gu2[26] + Gu1[29]*Gu2[30] + R11[6];
acadoWorkspace.H[iRow * 324 + 83] = + Gu1[1]*Gu2[3] + Gu1[5]*Gu2[7] + Gu1[9]*Gu2[11] + Gu1[13]*Gu2[15] + Gu1[17]*Gu2[19] + Gu1[21]*Gu2[23] + Gu1[25]*Gu2[27] + Gu1[29]*Gu2[31] + R11[7];
acadoWorkspace.H[iRow * 324 + 160] = + Gu1[2]*Gu2[0] + Gu1[6]*Gu2[4] + Gu1[10]*Gu2[8] + Gu1[14]*Gu2[12] + Gu1[18]*Gu2[16] + Gu1[22]*Gu2[20] + Gu1[26]*Gu2[24] + Gu1[30]*Gu2[28] + R11[8];
acadoWorkspace.H[iRow * 324 + 161] = + Gu1[2]*Gu2[1] + Gu1[6]*Gu2[5] + Gu1[10]*Gu2[9] + Gu1[14]*Gu2[13] + Gu1[18]*Gu2[17] + Gu1[22]*Gu2[21] + Gu1[26]*Gu2[25] + Gu1[30]*Gu2[29] + R11[9];
acadoWorkspace.H[iRow * 324 + 162] = + Gu1[2]*Gu2[2] + Gu1[6]*Gu2[6] + Gu1[10]*Gu2[10] + Gu1[14]*Gu2[14] + Gu1[18]*Gu2[18] + Gu1[22]*Gu2[22] + Gu1[26]*Gu2[26] + Gu1[30]*Gu2[30] + R11[10];
acadoWorkspace.H[iRow * 324 + 163] = + Gu1[2]*Gu2[3] + Gu1[6]*Gu2[7] + Gu1[10]*Gu2[11] + Gu1[14]*Gu2[15] + Gu1[18]*Gu2[19] + Gu1[22]*Gu2[23] + Gu1[26]*Gu2[27] + Gu1[30]*Gu2[31] + R11[11];
acadoWorkspace.H[iRow * 324 + 240] = + Gu1[3]*Gu2[0] + Gu1[7]*Gu2[4] + Gu1[11]*Gu2[8] + Gu1[15]*Gu2[12] + Gu1[19]*Gu2[16] + Gu1[23]*Gu2[20] + Gu1[27]*Gu2[24] + Gu1[31]*Gu2[28] + R11[12];
acadoWorkspace.H[iRow * 324 + 241] = + Gu1[3]*Gu2[1] + Gu1[7]*Gu2[5] + Gu1[11]*Gu2[9] + Gu1[15]*Gu2[13] + Gu1[19]*Gu2[17] + Gu1[23]*Gu2[21] + Gu1[27]*Gu2[25] + Gu1[31]*Gu2[29] + R11[13];
acadoWorkspace.H[iRow * 324 + 242] = + Gu1[3]*Gu2[2] + Gu1[7]*Gu2[6] + Gu1[11]*Gu2[10] + Gu1[15]*Gu2[14] + Gu1[19]*Gu2[18] + Gu1[23]*Gu2[22] + Gu1[27]*Gu2[26] + Gu1[31]*Gu2[30] + R11[14];
acadoWorkspace.H[iRow * 324 + 243] = + Gu1[3]*Gu2[3] + Gu1[7]*Gu2[7] + Gu1[11]*Gu2[11] + Gu1[15]*Gu2[15] + Gu1[19]*Gu2[19] + Gu1[23]*Gu2[23] + Gu1[27]*Gu2[27] + Gu1[31]*Gu2[31] + R11[15];
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[8]*Gu1[4] + Gx1[16]*Gu1[8] + Gx1[24]*Gu1[12] + Gx1[32]*Gu1[16] + Gx1[40]*Gu1[20] + Gx1[48]*Gu1[24] + Gx1[56]*Gu1[28];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[8]*Gu1[5] + Gx1[16]*Gu1[9] + Gx1[24]*Gu1[13] + Gx1[32]*Gu1[17] + Gx1[40]*Gu1[21] + Gx1[48]*Gu1[25] + Gx1[56]*Gu1[29];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[8]*Gu1[6] + Gx1[16]*Gu1[10] + Gx1[24]*Gu1[14] + Gx1[32]*Gu1[18] + Gx1[40]*Gu1[22] + Gx1[48]*Gu1[26] + Gx1[56]*Gu1[30];
Gu2[3] = + Gx1[0]*Gu1[3] + Gx1[8]*Gu1[7] + Gx1[16]*Gu1[11] + Gx1[24]*Gu1[15] + Gx1[32]*Gu1[19] + Gx1[40]*Gu1[23] + Gx1[48]*Gu1[27] + Gx1[56]*Gu1[31];
Gu2[4] = + Gx1[1]*Gu1[0] + Gx1[9]*Gu1[4] + Gx1[17]*Gu1[8] + Gx1[25]*Gu1[12] + Gx1[33]*Gu1[16] + Gx1[41]*Gu1[20] + Gx1[49]*Gu1[24] + Gx1[57]*Gu1[28];
Gu2[5] = + Gx1[1]*Gu1[1] + Gx1[9]*Gu1[5] + Gx1[17]*Gu1[9] + Gx1[25]*Gu1[13] + Gx1[33]*Gu1[17] + Gx1[41]*Gu1[21] + Gx1[49]*Gu1[25] + Gx1[57]*Gu1[29];
Gu2[6] = + Gx1[1]*Gu1[2] + Gx1[9]*Gu1[6] + Gx1[17]*Gu1[10] + Gx1[25]*Gu1[14] + Gx1[33]*Gu1[18] + Gx1[41]*Gu1[22] + Gx1[49]*Gu1[26] + Gx1[57]*Gu1[30];
Gu2[7] = + Gx1[1]*Gu1[3] + Gx1[9]*Gu1[7] + Gx1[17]*Gu1[11] + Gx1[25]*Gu1[15] + Gx1[33]*Gu1[19] + Gx1[41]*Gu1[23] + Gx1[49]*Gu1[27] + Gx1[57]*Gu1[31];
Gu2[8] = + Gx1[2]*Gu1[0] + Gx1[10]*Gu1[4] + Gx1[18]*Gu1[8] + Gx1[26]*Gu1[12] + Gx1[34]*Gu1[16] + Gx1[42]*Gu1[20] + Gx1[50]*Gu1[24] + Gx1[58]*Gu1[28];
Gu2[9] = + Gx1[2]*Gu1[1] + Gx1[10]*Gu1[5] + Gx1[18]*Gu1[9] + Gx1[26]*Gu1[13] + Gx1[34]*Gu1[17] + Gx1[42]*Gu1[21] + Gx1[50]*Gu1[25] + Gx1[58]*Gu1[29];
Gu2[10] = + Gx1[2]*Gu1[2] + Gx1[10]*Gu1[6] + Gx1[18]*Gu1[10] + Gx1[26]*Gu1[14] + Gx1[34]*Gu1[18] + Gx1[42]*Gu1[22] + Gx1[50]*Gu1[26] + Gx1[58]*Gu1[30];
Gu2[11] = + Gx1[2]*Gu1[3] + Gx1[10]*Gu1[7] + Gx1[18]*Gu1[11] + Gx1[26]*Gu1[15] + Gx1[34]*Gu1[19] + Gx1[42]*Gu1[23] + Gx1[50]*Gu1[27] + Gx1[58]*Gu1[31];
Gu2[12] = + Gx1[3]*Gu1[0] + Gx1[11]*Gu1[4] + Gx1[19]*Gu1[8] + Gx1[27]*Gu1[12] + Gx1[35]*Gu1[16] + Gx1[43]*Gu1[20] + Gx1[51]*Gu1[24] + Gx1[59]*Gu1[28];
Gu2[13] = + Gx1[3]*Gu1[1] + Gx1[11]*Gu1[5] + Gx1[19]*Gu1[9] + Gx1[27]*Gu1[13] + Gx1[35]*Gu1[17] + Gx1[43]*Gu1[21] + Gx1[51]*Gu1[25] + Gx1[59]*Gu1[29];
Gu2[14] = + Gx1[3]*Gu1[2] + Gx1[11]*Gu1[6] + Gx1[19]*Gu1[10] + Gx1[27]*Gu1[14] + Gx1[35]*Gu1[18] + Gx1[43]*Gu1[22] + Gx1[51]*Gu1[26] + Gx1[59]*Gu1[30];
Gu2[15] = + Gx1[3]*Gu1[3] + Gx1[11]*Gu1[7] + Gx1[19]*Gu1[11] + Gx1[27]*Gu1[15] + Gx1[35]*Gu1[19] + Gx1[43]*Gu1[23] + Gx1[51]*Gu1[27] + Gx1[59]*Gu1[31];
Gu2[16] = + Gx1[4]*Gu1[0] + Gx1[12]*Gu1[4] + Gx1[20]*Gu1[8] + Gx1[28]*Gu1[12] + Gx1[36]*Gu1[16] + Gx1[44]*Gu1[20] + Gx1[52]*Gu1[24] + Gx1[60]*Gu1[28];
Gu2[17] = + Gx1[4]*Gu1[1] + Gx1[12]*Gu1[5] + Gx1[20]*Gu1[9] + Gx1[28]*Gu1[13] + Gx1[36]*Gu1[17] + Gx1[44]*Gu1[21] + Gx1[52]*Gu1[25] + Gx1[60]*Gu1[29];
Gu2[18] = + Gx1[4]*Gu1[2] + Gx1[12]*Gu1[6] + Gx1[20]*Gu1[10] + Gx1[28]*Gu1[14] + Gx1[36]*Gu1[18] + Gx1[44]*Gu1[22] + Gx1[52]*Gu1[26] + Gx1[60]*Gu1[30];
Gu2[19] = + Gx1[4]*Gu1[3] + Gx1[12]*Gu1[7] + Gx1[20]*Gu1[11] + Gx1[28]*Gu1[15] + Gx1[36]*Gu1[19] + Gx1[44]*Gu1[23] + Gx1[52]*Gu1[27] + Gx1[60]*Gu1[31];
Gu2[20] = + Gx1[5]*Gu1[0] + Gx1[13]*Gu1[4] + Gx1[21]*Gu1[8] + Gx1[29]*Gu1[12] + Gx1[37]*Gu1[16] + Gx1[45]*Gu1[20] + Gx1[53]*Gu1[24] + Gx1[61]*Gu1[28];
Gu2[21] = + Gx1[5]*Gu1[1] + Gx1[13]*Gu1[5] + Gx1[21]*Gu1[9] + Gx1[29]*Gu1[13] + Gx1[37]*Gu1[17] + Gx1[45]*Gu1[21] + Gx1[53]*Gu1[25] + Gx1[61]*Gu1[29];
Gu2[22] = + Gx1[5]*Gu1[2] + Gx1[13]*Gu1[6] + Gx1[21]*Gu1[10] + Gx1[29]*Gu1[14] + Gx1[37]*Gu1[18] + Gx1[45]*Gu1[22] + Gx1[53]*Gu1[26] + Gx1[61]*Gu1[30];
Gu2[23] = + Gx1[5]*Gu1[3] + Gx1[13]*Gu1[7] + Gx1[21]*Gu1[11] + Gx1[29]*Gu1[15] + Gx1[37]*Gu1[19] + Gx1[45]*Gu1[23] + Gx1[53]*Gu1[27] + Gx1[61]*Gu1[31];
Gu2[24] = + Gx1[6]*Gu1[0] + Gx1[14]*Gu1[4] + Gx1[22]*Gu1[8] + Gx1[30]*Gu1[12] + Gx1[38]*Gu1[16] + Gx1[46]*Gu1[20] + Gx1[54]*Gu1[24] + Gx1[62]*Gu1[28];
Gu2[25] = + Gx1[6]*Gu1[1] + Gx1[14]*Gu1[5] + Gx1[22]*Gu1[9] + Gx1[30]*Gu1[13] + Gx1[38]*Gu1[17] + Gx1[46]*Gu1[21] + Gx1[54]*Gu1[25] + Gx1[62]*Gu1[29];
Gu2[26] = + Gx1[6]*Gu1[2] + Gx1[14]*Gu1[6] + Gx1[22]*Gu1[10] + Gx1[30]*Gu1[14] + Gx1[38]*Gu1[18] + Gx1[46]*Gu1[22] + Gx1[54]*Gu1[26] + Gx1[62]*Gu1[30];
Gu2[27] = + Gx1[6]*Gu1[3] + Gx1[14]*Gu1[7] + Gx1[22]*Gu1[11] + Gx1[30]*Gu1[15] + Gx1[38]*Gu1[19] + Gx1[46]*Gu1[23] + Gx1[54]*Gu1[27] + Gx1[62]*Gu1[31];
Gu2[28] = + Gx1[7]*Gu1[0] + Gx1[15]*Gu1[4] + Gx1[23]*Gu1[8] + Gx1[31]*Gu1[12] + Gx1[39]*Gu1[16] + Gx1[47]*Gu1[20] + Gx1[55]*Gu1[24] + Gx1[63]*Gu1[28];
Gu2[29] = + Gx1[7]*Gu1[1] + Gx1[15]*Gu1[5] + Gx1[23]*Gu1[9] + Gx1[31]*Gu1[13] + Gx1[39]*Gu1[17] + Gx1[47]*Gu1[21] + Gx1[55]*Gu1[25] + Gx1[63]*Gu1[29];
Gu2[30] = + Gx1[7]*Gu1[2] + Gx1[15]*Gu1[6] + Gx1[23]*Gu1[10] + Gx1[31]*Gu1[14] + Gx1[39]*Gu1[18] + Gx1[47]*Gu1[22] + Gx1[55]*Gu1[26] + Gx1[63]*Gu1[30];
Gu2[31] = + Gx1[7]*Gu1[3] + Gx1[15]*Gu1[7] + Gx1[23]*Gu1[11] + Gx1[31]*Gu1[15] + Gx1[39]*Gu1[19] + Gx1[47]*Gu1[23] + Gx1[55]*Gu1[27] + Gx1[63]*Gu1[31];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[4] + Q11[2]*Gu1[8] + Q11[3]*Gu1[12] + Q11[4]*Gu1[16] + Q11[5]*Gu1[20] + Q11[6]*Gu1[24] + Q11[7]*Gu1[28] + Gu2[0];
Gu3[1] = + Q11[0]*Gu1[1] + Q11[1]*Gu1[5] + Q11[2]*Gu1[9] + Q11[3]*Gu1[13] + Q11[4]*Gu1[17] + Q11[5]*Gu1[21] + Q11[6]*Gu1[25] + Q11[7]*Gu1[29] + Gu2[1];
Gu3[2] = + Q11[0]*Gu1[2] + Q11[1]*Gu1[6] + Q11[2]*Gu1[10] + Q11[3]*Gu1[14] + Q11[4]*Gu1[18] + Q11[5]*Gu1[22] + Q11[6]*Gu1[26] + Q11[7]*Gu1[30] + Gu2[2];
Gu3[3] = + Q11[0]*Gu1[3] + Q11[1]*Gu1[7] + Q11[2]*Gu1[11] + Q11[3]*Gu1[15] + Q11[4]*Gu1[19] + Q11[5]*Gu1[23] + Q11[6]*Gu1[27] + Q11[7]*Gu1[31] + Gu2[3];
Gu3[4] = + Q11[8]*Gu1[0] + Q11[9]*Gu1[4] + Q11[10]*Gu1[8] + Q11[11]*Gu1[12] + Q11[12]*Gu1[16] + Q11[13]*Gu1[20] + Q11[14]*Gu1[24] + Q11[15]*Gu1[28] + Gu2[4];
Gu3[5] = + Q11[8]*Gu1[1] + Q11[9]*Gu1[5] + Q11[10]*Gu1[9] + Q11[11]*Gu1[13] + Q11[12]*Gu1[17] + Q11[13]*Gu1[21] + Q11[14]*Gu1[25] + Q11[15]*Gu1[29] + Gu2[5];
Gu3[6] = + Q11[8]*Gu1[2] + Q11[9]*Gu1[6] + Q11[10]*Gu1[10] + Q11[11]*Gu1[14] + Q11[12]*Gu1[18] + Q11[13]*Gu1[22] + Q11[14]*Gu1[26] + Q11[15]*Gu1[30] + Gu2[6];
Gu3[7] = + Q11[8]*Gu1[3] + Q11[9]*Gu1[7] + Q11[10]*Gu1[11] + Q11[11]*Gu1[15] + Q11[12]*Gu1[19] + Q11[13]*Gu1[23] + Q11[14]*Gu1[27] + Q11[15]*Gu1[31] + Gu2[7];
Gu3[8] = + Q11[16]*Gu1[0] + Q11[17]*Gu1[4] + Q11[18]*Gu1[8] + Q11[19]*Gu1[12] + Q11[20]*Gu1[16] + Q11[21]*Gu1[20] + Q11[22]*Gu1[24] + Q11[23]*Gu1[28] + Gu2[8];
Gu3[9] = + Q11[16]*Gu1[1] + Q11[17]*Gu1[5] + Q11[18]*Gu1[9] + Q11[19]*Gu1[13] + Q11[20]*Gu1[17] + Q11[21]*Gu1[21] + Q11[22]*Gu1[25] + Q11[23]*Gu1[29] + Gu2[9];
Gu3[10] = + Q11[16]*Gu1[2] + Q11[17]*Gu1[6] + Q11[18]*Gu1[10] + Q11[19]*Gu1[14] + Q11[20]*Gu1[18] + Q11[21]*Gu1[22] + Q11[22]*Gu1[26] + Q11[23]*Gu1[30] + Gu2[10];
Gu3[11] = + Q11[16]*Gu1[3] + Q11[17]*Gu1[7] + Q11[18]*Gu1[11] + Q11[19]*Gu1[15] + Q11[20]*Gu1[19] + Q11[21]*Gu1[23] + Q11[22]*Gu1[27] + Q11[23]*Gu1[31] + Gu2[11];
Gu3[12] = + Q11[24]*Gu1[0] + Q11[25]*Gu1[4] + Q11[26]*Gu1[8] + Q11[27]*Gu1[12] + Q11[28]*Gu1[16] + Q11[29]*Gu1[20] + Q11[30]*Gu1[24] + Q11[31]*Gu1[28] + Gu2[12];
Gu3[13] = + Q11[24]*Gu1[1] + Q11[25]*Gu1[5] + Q11[26]*Gu1[9] + Q11[27]*Gu1[13] + Q11[28]*Gu1[17] + Q11[29]*Gu1[21] + Q11[30]*Gu1[25] + Q11[31]*Gu1[29] + Gu2[13];
Gu3[14] = + Q11[24]*Gu1[2] + Q11[25]*Gu1[6] + Q11[26]*Gu1[10] + Q11[27]*Gu1[14] + Q11[28]*Gu1[18] + Q11[29]*Gu1[22] + Q11[30]*Gu1[26] + Q11[31]*Gu1[30] + Gu2[14];
Gu3[15] = + Q11[24]*Gu1[3] + Q11[25]*Gu1[7] + Q11[26]*Gu1[11] + Q11[27]*Gu1[15] + Q11[28]*Gu1[19] + Q11[29]*Gu1[23] + Q11[30]*Gu1[27] + Q11[31]*Gu1[31] + Gu2[15];
Gu3[16] = + Q11[32]*Gu1[0] + Q11[33]*Gu1[4] + Q11[34]*Gu1[8] + Q11[35]*Gu1[12] + Q11[36]*Gu1[16] + Q11[37]*Gu1[20] + Q11[38]*Gu1[24] + Q11[39]*Gu1[28] + Gu2[16];
Gu3[17] = + Q11[32]*Gu1[1] + Q11[33]*Gu1[5] + Q11[34]*Gu1[9] + Q11[35]*Gu1[13] + Q11[36]*Gu1[17] + Q11[37]*Gu1[21] + Q11[38]*Gu1[25] + Q11[39]*Gu1[29] + Gu2[17];
Gu3[18] = + Q11[32]*Gu1[2] + Q11[33]*Gu1[6] + Q11[34]*Gu1[10] + Q11[35]*Gu1[14] + Q11[36]*Gu1[18] + Q11[37]*Gu1[22] + Q11[38]*Gu1[26] + Q11[39]*Gu1[30] + Gu2[18];
Gu3[19] = + Q11[32]*Gu1[3] + Q11[33]*Gu1[7] + Q11[34]*Gu1[11] + Q11[35]*Gu1[15] + Q11[36]*Gu1[19] + Q11[37]*Gu1[23] + Q11[38]*Gu1[27] + Q11[39]*Gu1[31] + Gu2[19];
Gu3[20] = + Q11[40]*Gu1[0] + Q11[41]*Gu1[4] + Q11[42]*Gu1[8] + Q11[43]*Gu1[12] + Q11[44]*Gu1[16] + Q11[45]*Gu1[20] + Q11[46]*Gu1[24] + Q11[47]*Gu1[28] + Gu2[20];
Gu3[21] = + Q11[40]*Gu1[1] + Q11[41]*Gu1[5] + Q11[42]*Gu1[9] + Q11[43]*Gu1[13] + Q11[44]*Gu1[17] + Q11[45]*Gu1[21] + Q11[46]*Gu1[25] + Q11[47]*Gu1[29] + Gu2[21];
Gu3[22] = + Q11[40]*Gu1[2] + Q11[41]*Gu1[6] + Q11[42]*Gu1[10] + Q11[43]*Gu1[14] + Q11[44]*Gu1[18] + Q11[45]*Gu1[22] + Q11[46]*Gu1[26] + Q11[47]*Gu1[30] + Gu2[22];
Gu3[23] = + Q11[40]*Gu1[3] + Q11[41]*Gu1[7] + Q11[42]*Gu1[11] + Q11[43]*Gu1[15] + Q11[44]*Gu1[19] + Q11[45]*Gu1[23] + Q11[46]*Gu1[27] + Q11[47]*Gu1[31] + Gu2[23];
Gu3[24] = + Q11[48]*Gu1[0] + Q11[49]*Gu1[4] + Q11[50]*Gu1[8] + Q11[51]*Gu1[12] + Q11[52]*Gu1[16] + Q11[53]*Gu1[20] + Q11[54]*Gu1[24] + Q11[55]*Gu1[28] + Gu2[24];
Gu3[25] = + Q11[48]*Gu1[1] + Q11[49]*Gu1[5] + Q11[50]*Gu1[9] + Q11[51]*Gu1[13] + Q11[52]*Gu1[17] + Q11[53]*Gu1[21] + Q11[54]*Gu1[25] + Q11[55]*Gu1[29] + Gu2[25];
Gu3[26] = + Q11[48]*Gu1[2] + Q11[49]*Gu1[6] + Q11[50]*Gu1[10] + Q11[51]*Gu1[14] + Q11[52]*Gu1[18] + Q11[53]*Gu1[22] + Q11[54]*Gu1[26] + Q11[55]*Gu1[30] + Gu2[26];
Gu3[27] = + Q11[48]*Gu1[3] + Q11[49]*Gu1[7] + Q11[50]*Gu1[11] + Q11[51]*Gu1[15] + Q11[52]*Gu1[19] + Q11[53]*Gu1[23] + Q11[54]*Gu1[27] + Q11[55]*Gu1[31] + Gu2[27];
Gu3[28] = + Q11[56]*Gu1[0] + Q11[57]*Gu1[4] + Q11[58]*Gu1[8] + Q11[59]*Gu1[12] + Q11[60]*Gu1[16] + Q11[61]*Gu1[20] + Q11[62]*Gu1[24] + Q11[63]*Gu1[28] + Gu2[28];
Gu3[29] = + Q11[56]*Gu1[1] + Q11[57]*Gu1[5] + Q11[58]*Gu1[9] + Q11[59]*Gu1[13] + Q11[60]*Gu1[17] + Q11[61]*Gu1[21] + Q11[62]*Gu1[25] + Q11[63]*Gu1[29] + Gu2[29];
Gu3[30] = + Q11[56]*Gu1[2] + Q11[57]*Gu1[6] + Q11[58]*Gu1[10] + Q11[59]*Gu1[14] + Q11[60]*Gu1[18] + Q11[61]*Gu1[22] + Q11[62]*Gu1[26] + Q11[63]*Gu1[30] + Gu2[30];
Gu3[31] = + Q11[56]*Gu1[3] + Q11[57]*Gu1[7] + Q11[58]*Gu1[11] + Q11[59]*Gu1[15] + Q11[60]*Gu1[19] + Q11[61]*Gu1[23] + Q11[62]*Gu1[27] + Q11[63]*Gu1[31] + Gu2[31];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[8]*w11[1] + Gx1[16]*w11[2] + Gx1[24]*w11[3] + Gx1[32]*w11[4] + Gx1[40]*w11[5] + Gx1[48]*w11[6] + Gx1[56]*w11[7] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[9]*w11[1] + Gx1[17]*w11[2] + Gx1[25]*w11[3] + Gx1[33]*w11[4] + Gx1[41]*w11[5] + Gx1[49]*w11[6] + Gx1[57]*w11[7] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[10]*w11[1] + Gx1[18]*w11[2] + Gx1[26]*w11[3] + Gx1[34]*w11[4] + Gx1[42]*w11[5] + Gx1[50]*w11[6] + Gx1[58]*w11[7] + w12[2];
w13[3] = + Gx1[3]*w11[0] + Gx1[11]*w11[1] + Gx1[19]*w11[2] + Gx1[27]*w11[3] + Gx1[35]*w11[4] + Gx1[43]*w11[5] + Gx1[51]*w11[6] + Gx1[59]*w11[7] + w12[3];
w13[4] = + Gx1[4]*w11[0] + Gx1[12]*w11[1] + Gx1[20]*w11[2] + Gx1[28]*w11[3] + Gx1[36]*w11[4] + Gx1[44]*w11[5] + Gx1[52]*w11[6] + Gx1[60]*w11[7] + w12[4];
w13[5] = + Gx1[5]*w11[0] + Gx1[13]*w11[1] + Gx1[21]*w11[2] + Gx1[29]*w11[3] + Gx1[37]*w11[4] + Gx1[45]*w11[5] + Gx1[53]*w11[6] + Gx1[61]*w11[7] + w12[5];
w13[6] = + Gx1[6]*w11[0] + Gx1[14]*w11[1] + Gx1[22]*w11[2] + Gx1[30]*w11[3] + Gx1[38]*w11[4] + Gx1[46]*w11[5] + Gx1[54]*w11[6] + Gx1[62]*w11[7] + w12[6];
w13[7] = + Gx1[7]*w11[0] + Gx1[15]*w11[1] + Gx1[23]*w11[2] + Gx1[31]*w11[3] + Gx1[39]*w11[4] + Gx1[47]*w11[5] + Gx1[55]*w11[6] + Gx1[63]*w11[7] + w12[7];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[4]*w11[1] + Gu1[8]*w11[2] + Gu1[12]*w11[3] + Gu1[16]*w11[4] + Gu1[20]*w11[5] + Gu1[24]*w11[6] + Gu1[28]*w11[7];
U1[1] += + Gu1[1]*w11[0] + Gu1[5]*w11[1] + Gu1[9]*w11[2] + Gu1[13]*w11[3] + Gu1[17]*w11[4] + Gu1[21]*w11[5] + Gu1[25]*w11[6] + Gu1[29]*w11[7];
U1[2] += + Gu1[2]*w11[0] + Gu1[6]*w11[1] + Gu1[10]*w11[2] + Gu1[14]*w11[3] + Gu1[18]*w11[4] + Gu1[22]*w11[5] + Gu1[26]*w11[6] + Gu1[30]*w11[7];
U1[3] += + Gu1[3]*w11[0] + Gu1[7]*w11[1] + Gu1[11]*w11[2] + Gu1[15]*w11[3] + Gu1[19]*w11[4] + Gu1[23]*w11[5] + Gu1[27]*w11[6] + Gu1[31]*w11[7];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + Q11[2]*w11[2] + Q11[3]*w11[3] + Q11[4]*w11[4] + Q11[5]*w11[5] + Q11[6]*w11[6] + Q11[7]*w11[7] + w12[0];
w13[1] = + Q11[8]*w11[0] + Q11[9]*w11[1] + Q11[10]*w11[2] + Q11[11]*w11[3] + Q11[12]*w11[4] + Q11[13]*w11[5] + Q11[14]*w11[6] + Q11[15]*w11[7] + w12[1];
w13[2] = + Q11[16]*w11[0] + Q11[17]*w11[1] + Q11[18]*w11[2] + Q11[19]*w11[3] + Q11[20]*w11[4] + Q11[21]*w11[5] + Q11[22]*w11[6] + Q11[23]*w11[7] + w12[2];
w13[3] = + Q11[24]*w11[0] + Q11[25]*w11[1] + Q11[26]*w11[2] + Q11[27]*w11[3] + Q11[28]*w11[4] + Q11[29]*w11[5] + Q11[30]*w11[6] + Q11[31]*w11[7] + w12[3];
w13[4] = + Q11[32]*w11[0] + Q11[33]*w11[1] + Q11[34]*w11[2] + Q11[35]*w11[3] + Q11[36]*w11[4] + Q11[37]*w11[5] + Q11[38]*w11[6] + Q11[39]*w11[7] + w12[4];
w13[5] = + Q11[40]*w11[0] + Q11[41]*w11[1] + Q11[42]*w11[2] + Q11[43]*w11[3] + Q11[44]*w11[4] + Q11[45]*w11[5] + Q11[46]*w11[6] + Q11[47]*w11[7] + w12[5];
w13[6] = + Q11[48]*w11[0] + Q11[49]*w11[1] + Q11[50]*w11[2] + Q11[51]*w11[3] + Q11[52]*w11[4] + Q11[53]*w11[5] + Q11[54]*w11[6] + Q11[55]*w11[7] + w12[6];
w13[7] = + Q11[56]*w11[0] + Q11[57]*w11[1] + Q11[58]*w11[2] + Q11[59]*w11[3] + Q11[60]*w11[4] + Q11[61]*w11[5] + Q11[62]*w11[6] + Q11[63]*w11[7] + w12[7];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7];
w12[1] += + Gx1[8]*w11[0] + Gx1[9]*w11[1] + Gx1[10]*w11[2] + Gx1[11]*w11[3] + Gx1[12]*w11[4] + Gx1[13]*w11[5] + Gx1[14]*w11[6] + Gx1[15]*w11[7];
w12[2] += + Gx1[16]*w11[0] + Gx1[17]*w11[1] + Gx1[18]*w11[2] + Gx1[19]*w11[3] + Gx1[20]*w11[4] + Gx1[21]*w11[5] + Gx1[22]*w11[6] + Gx1[23]*w11[7];
w12[3] += + Gx1[24]*w11[0] + Gx1[25]*w11[1] + Gx1[26]*w11[2] + Gx1[27]*w11[3] + Gx1[28]*w11[4] + Gx1[29]*w11[5] + Gx1[30]*w11[6] + Gx1[31]*w11[7];
w12[4] += + Gx1[32]*w11[0] + Gx1[33]*w11[1] + Gx1[34]*w11[2] + Gx1[35]*w11[3] + Gx1[36]*w11[4] + Gx1[37]*w11[5] + Gx1[38]*w11[6] + Gx1[39]*w11[7];
w12[5] += + Gx1[40]*w11[0] + Gx1[41]*w11[1] + Gx1[42]*w11[2] + Gx1[43]*w11[3] + Gx1[44]*w11[4] + Gx1[45]*w11[5] + Gx1[46]*w11[6] + Gx1[47]*w11[7];
w12[6] += + Gx1[48]*w11[0] + Gx1[49]*w11[1] + Gx1[50]*w11[2] + Gx1[51]*w11[3] + Gx1[52]*w11[4] + Gx1[53]*w11[5] + Gx1[54]*w11[6] + Gx1[55]*w11[7];
w12[7] += + Gx1[56]*w11[0] + Gx1[57]*w11[1] + Gx1[58]*w11[2] + Gx1[59]*w11[3] + Gx1[60]*w11[4] + Gx1[61]*w11[5] + Gx1[62]*w11[6] + Gx1[63]*w11[7];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7];
w12[1] += + Gx1[8]*w11[0] + Gx1[9]*w11[1] + Gx1[10]*w11[2] + Gx1[11]*w11[3] + Gx1[12]*w11[4] + Gx1[13]*w11[5] + Gx1[14]*w11[6] + Gx1[15]*w11[7];
w12[2] += + Gx1[16]*w11[0] + Gx1[17]*w11[1] + Gx1[18]*w11[2] + Gx1[19]*w11[3] + Gx1[20]*w11[4] + Gx1[21]*w11[5] + Gx1[22]*w11[6] + Gx1[23]*w11[7];
w12[3] += + Gx1[24]*w11[0] + Gx1[25]*w11[1] + Gx1[26]*w11[2] + Gx1[27]*w11[3] + Gx1[28]*w11[4] + Gx1[29]*w11[5] + Gx1[30]*w11[6] + Gx1[31]*w11[7];
w12[4] += + Gx1[32]*w11[0] + Gx1[33]*w11[1] + Gx1[34]*w11[2] + Gx1[35]*w11[3] + Gx1[36]*w11[4] + Gx1[37]*w11[5] + Gx1[38]*w11[6] + Gx1[39]*w11[7];
w12[5] += + Gx1[40]*w11[0] + Gx1[41]*w11[1] + Gx1[42]*w11[2] + Gx1[43]*w11[3] + Gx1[44]*w11[4] + Gx1[45]*w11[5] + Gx1[46]*w11[6] + Gx1[47]*w11[7];
w12[6] += + Gx1[48]*w11[0] + Gx1[49]*w11[1] + Gx1[50]*w11[2] + Gx1[51]*w11[3] + Gx1[52]*w11[4] + Gx1[53]*w11[5] + Gx1[54]*w11[6] + Gx1[55]*w11[7];
w12[7] += + Gx1[56]*w11[0] + Gx1[57]*w11[1] + Gx1[58]*w11[2] + Gx1[59]*w11[3] + Gx1[60]*w11[4] + Gx1[61]*w11[5] + Gx1[62]*w11[6] + Gx1[63]*w11[7];
w12[0] += + Gu1[0]*U1[0] + Gu1[1]*U1[1] + Gu1[2]*U1[2] + Gu1[3]*U1[3];
w12[1] += + Gu1[4]*U1[0] + Gu1[5]*U1[1] + Gu1[6]*U1[2] + Gu1[7]*U1[3];
w12[2] += + Gu1[8]*U1[0] + Gu1[9]*U1[1] + Gu1[10]*U1[2] + Gu1[11]*U1[3];
w12[3] += + Gu1[12]*U1[0] + Gu1[13]*U1[1] + Gu1[14]*U1[2] + Gu1[15]*U1[3];
w12[4] += + Gu1[16]*U1[0] + Gu1[17]*U1[1] + Gu1[18]*U1[2] + Gu1[19]*U1[3];
w12[5] += + Gu1[20]*U1[0] + Gu1[21]*U1[1] + Gu1[22]*U1[2] + Gu1[23]*U1[3];
w12[6] += + Gu1[24]*U1[0] + Gu1[25]*U1[1] + Gu1[26]*U1[2] + Gu1[27]*U1[3];
w12[7] += + Gu1[28]*U1[0] + Gu1[29]*U1[1] + Gu1[30]*U1[2] + Gu1[31]*U1[3];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 320) + (iCol * 4)] = acadoWorkspace.H[(iCol * 320) + (iRow * 4)];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 320 + 80) + (iRow * 4)];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 320 + 160) + (iRow * 4)];
acadoWorkspace.H[(iRow * 320) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 320 + 240) + (iRow * 4)];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4)] = acadoWorkspace.H[(iCol * 320) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 320 + 80) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 320 + 160) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 320 + 80) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 320 + 240) + (iRow * 4 + 1)];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4)] = acadoWorkspace.H[(iCol * 320) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 320 + 80) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 320 + 160) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 320 + 160) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 320 + 240) + (iRow * 4 + 2)];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4)] = acadoWorkspace.H[(iCol * 320) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 1)] = acadoWorkspace.H[(iCol * 320 + 80) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 2)] = acadoWorkspace.H[(iCol * 320 + 160) + (iRow * 4 + 3)];
acadoWorkspace.H[(iRow * 320 + 240) + (iCol * 4 + 3)] = acadoWorkspace.H[(iCol * 320 + 240) + (iRow * 4 + 3)];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7] + R2[8]*Dy1[8] + R2[9]*Dy1[9] + R2[10]*Dy1[10] + R2[11]*Dy1[11];
RDy1[1] = + R2[12]*Dy1[0] + R2[13]*Dy1[1] + R2[14]*Dy1[2] + R2[15]*Dy1[3] + R2[16]*Dy1[4] + R2[17]*Dy1[5] + R2[18]*Dy1[6] + R2[19]*Dy1[7] + R2[20]*Dy1[8] + R2[21]*Dy1[9] + R2[22]*Dy1[10] + R2[23]*Dy1[11];
RDy1[2] = + R2[24]*Dy1[0] + R2[25]*Dy1[1] + R2[26]*Dy1[2] + R2[27]*Dy1[3] + R2[28]*Dy1[4] + R2[29]*Dy1[5] + R2[30]*Dy1[6] + R2[31]*Dy1[7] + R2[32]*Dy1[8] + R2[33]*Dy1[9] + R2[34]*Dy1[10] + R2[35]*Dy1[11];
RDy1[3] = + R2[36]*Dy1[0] + R2[37]*Dy1[1] + R2[38]*Dy1[2] + R2[39]*Dy1[3] + R2[40]*Dy1[4] + R2[41]*Dy1[5] + R2[42]*Dy1[6] + R2[43]*Dy1[7] + R2[44]*Dy1[8] + R2[45]*Dy1[9] + R2[46]*Dy1[10] + R2[47]*Dy1[11];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6] + Q2[7]*Dy1[7] + Q2[8]*Dy1[8] + Q2[9]*Dy1[9] + Q2[10]*Dy1[10] + Q2[11]*Dy1[11];
QDy1[1] = + Q2[12]*Dy1[0] + Q2[13]*Dy1[1] + Q2[14]*Dy1[2] + Q2[15]*Dy1[3] + Q2[16]*Dy1[4] + Q2[17]*Dy1[5] + Q2[18]*Dy1[6] + Q2[19]*Dy1[7] + Q2[20]*Dy1[8] + Q2[21]*Dy1[9] + Q2[22]*Dy1[10] + Q2[23]*Dy1[11];
QDy1[2] = + Q2[24]*Dy1[0] + Q2[25]*Dy1[1] + Q2[26]*Dy1[2] + Q2[27]*Dy1[3] + Q2[28]*Dy1[4] + Q2[29]*Dy1[5] + Q2[30]*Dy1[6] + Q2[31]*Dy1[7] + Q2[32]*Dy1[8] + Q2[33]*Dy1[9] + Q2[34]*Dy1[10] + Q2[35]*Dy1[11];
QDy1[3] = + Q2[36]*Dy1[0] + Q2[37]*Dy1[1] + Q2[38]*Dy1[2] + Q2[39]*Dy1[3] + Q2[40]*Dy1[4] + Q2[41]*Dy1[5] + Q2[42]*Dy1[6] + Q2[43]*Dy1[7] + Q2[44]*Dy1[8] + Q2[45]*Dy1[9] + Q2[46]*Dy1[10] + Q2[47]*Dy1[11];
QDy1[4] = + Q2[48]*Dy1[0] + Q2[49]*Dy1[1] + Q2[50]*Dy1[2] + Q2[51]*Dy1[3] + Q2[52]*Dy1[4] + Q2[53]*Dy1[5] + Q2[54]*Dy1[6] + Q2[55]*Dy1[7] + Q2[56]*Dy1[8] + Q2[57]*Dy1[9] + Q2[58]*Dy1[10] + Q2[59]*Dy1[11];
QDy1[5] = + Q2[60]*Dy1[0] + Q2[61]*Dy1[1] + Q2[62]*Dy1[2] + Q2[63]*Dy1[3] + Q2[64]*Dy1[4] + Q2[65]*Dy1[5] + Q2[66]*Dy1[6] + Q2[67]*Dy1[7] + Q2[68]*Dy1[8] + Q2[69]*Dy1[9] + Q2[70]*Dy1[10] + Q2[71]*Dy1[11];
QDy1[6] = + Q2[72]*Dy1[0] + Q2[73]*Dy1[1] + Q2[74]*Dy1[2] + Q2[75]*Dy1[3] + Q2[76]*Dy1[4] + Q2[77]*Dy1[5] + Q2[78]*Dy1[6] + Q2[79]*Dy1[7] + Q2[80]*Dy1[8] + Q2[81]*Dy1[9] + Q2[82]*Dy1[10] + Q2[83]*Dy1[11];
QDy1[7] = + Q2[84]*Dy1[0] + Q2[85]*Dy1[1] + Q2[86]*Dy1[2] + Q2[87]*Dy1[3] + Q2[88]*Dy1[4] + Q2[89]*Dy1[5] + Q2[90]*Dy1[6] + Q2[91]*Dy1[7] + Q2[92]*Dy1[8] + Q2[93]*Dy1[9] + Q2[94]*Dy1[10] + Q2[95]*Dy1[11];
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
for (lRun2 = 0; lRun2 < 20; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 41)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 32 ]), &(acadoWorkspace.E[ lRun3 * 32 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 20; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (8)) * (8)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (8)) * (4)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (8)) * (4)) + (0) ]) );
}

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (20)) - (1)) * (8)) * (4)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 19; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 32 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 64 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ lRun1 * 64 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (8)) * (4)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.R1[ lRun2 * 16 ]), &(acadoWorkspace.evGu[ lRun2 * 32 ]), acadoWorkspace.W1, lRun2 );
}

acado_copyHTH( 0, 1 );
acado_copyHTH( 0, 2 );
acado_copyHTH( 1, 2 );
acado_copyHTH( 0, 3 );
acado_copyHTH( 1, 3 );
acado_copyHTH( 2, 3 );
acado_copyHTH( 0, 4 );
acado_copyHTH( 1, 4 );
acado_copyHTH( 2, 4 );
acado_copyHTH( 3, 4 );
acado_copyHTH( 0, 5 );
acado_copyHTH( 1, 5 );
acado_copyHTH( 2, 5 );
acado_copyHTH( 3, 5 );
acado_copyHTH( 4, 5 );
acado_copyHTH( 0, 6 );
acado_copyHTH( 1, 6 );
acado_copyHTH( 2, 6 );
acado_copyHTH( 3, 6 );
acado_copyHTH( 4, 6 );
acado_copyHTH( 5, 6 );
acado_copyHTH( 0, 7 );
acado_copyHTH( 1, 7 );
acado_copyHTH( 2, 7 );
acado_copyHTH( 3, 7 );
acado_copyHTH( 4, 7 );
acado_copyHTH( 5, 7 );
acado_copyHTH( 6, 7 );
acado_copyHTH( 0, 8 );
acado_copyHTH( 1, 8 );
acado_copyHTH( 2, 8 );
acado_copyHTH( 3, 8 );
acado_copyHTH( 4, 8 );
acado_copyHTH( 5, 8 );
acado_copyHTH( 6, 8 );
acado_copyHTH( 7, 8 );
acado_copyHTH( 0, 9 );
acado_copyHTH( 1, 9 );
acado_copyHTH( 2, 9 );
acado_copyHTH( 3, 9 );
acado_copyHTH( 4, 9 );
acado_copyHTH( 5, 9 );
acado_copyHTH( 6, 9 );
acado_copyHTH( 7, 9 );
acado_copyHTH( 8, 9 );
acado_copyHTH( 0, 10 );
acado_copyHTH( 1, 10 );
acado_copyHTH( 2, 10 );
acado_copyHTH( 3, 10 );
acado_copyHTH( 4, 10 );
acado_copyHTH( 5, 10 );
acado_copyHTH( 6, 10 );
acado_copyHTH( 7, 10 );
acado_copyHTH( 8, 10 );
acado_copyHTH( 9, 10 );
acado_copyHTH( 0, 11 );
acado_copyHTH( 1, 11 );
acado_copyHTH( 2, 11 );
acado_copyHTH( 3, 11 );
acado_copyHTH( 4, 11 );
acado_copyHTH( 5, 11 );
acado_copyHTH( 6, 11 );
acado_copyHTH( 7, 11 );
acado_copyHTH( 8, 11 );
acado_copyHTH( 9, 11 );
acado_copyHTH( 10, 11 );
acado_copyHTH( 0, 12 );
acado_copyHTH( 1, 12 );
acado_copyHTH( 2, 12 );
acado_copyHTH( 3, 12 );
acado_copyHTH( 4, 12 );
acado_copyHTH( 5, 12 );
acado_copyHTH( 6, 12 );
acado_copyHTH( 7, 12 );
acado_copyHTH( 8, 12 );
acado_copyHTH( 9, 12 );
acado_copyHTH( 10, 12 );
acado_copyHTH( 11, 12 );
acado_copyHTH( 0, 13 );
acado_copyHTH( 1, 13 );
acado_copyHTH( 2, 13 );
acado_copyHTH( 3, 13 );
acado_copyHTH( 4, 13 );
acado_copyHTH( 5, 13 );
acado_copyHTH( 6, 13 );
acado_copyHTH( 7, 13 );
acado_copyHTH( 8, 13 );
acado_copyHTH( 9, 13 );
acado_copyHTH( 10, 13 );
acado_copyHTH( 11, 13 );
acado_copyHTH( 12, 13 );
acado_copyHTH( 0, 14 );
acado_copyHTH( 1, 14 );
acado_copyHTH( 2, 14 );
acado_copyHTH( 3, 14 );
acado_copyHTH( 4, 14 );
acado_copyHTH( 5, 14 );
acado_copyHTH( 6, 14 );
acado_copyHTH( 7, 14 );
acado_copyHTH( 8, 14 );
acado_copyHTH( 9, 14 );
acado_copyHTH( 10, 14 );
acado_copyHTH( 11, 14 );
acado_copyHTH( 12, 14 );
acado_copyHTH( 13, 14 );
acado_copyHTH( 0, 15 );
acado_copyHTH( 1, 15 );
acado_copyHTH( 2, 15 );
acado_copyHTH( 3, 15 );
acado_copyHTH( 4, 15 );
acado_copyHTH( 5, 15 );
acado_copyHTH( 6, 15 );
acado_copyHTH( 7, 15 );
acado_copyHTH( 8, 15 );
acado_copyHTH( 9, 15 );
acado_copyHTH( 10, 15 );
acado_copyHTH( 11, 15 );
acado_copyHTH( 12, 15 );
acado_copyHTH( 13, 15 );
acado_copyHTH( 14, 15 );
acado_copyHTH( 0, 16 );
acado_copyHTH( 1, 16 );
acado_copyHTH( 2, 16 );
acado_copyHTH( 3, 16 );
acado_copyHTH( 4, 16 );
acado_copyHTH( 5, 16 );
acado_copyHTH( 6, 16 );
acado_copyHTH( 7, 16 );
acado_copyHTH( 8, 16 );
acado_copyHTH( 9, 16 );
acado_copyHTH( 10, 16 );
acado_copyHTH( 11, 16 );
acado_copyHTH( 12, 16 );
acado_copyHTH( 13, 16 );
acado_copyHTH( 14, 16 );
acado_copyHTH( 15, 16 );
acado_copyHTH( 0, 17 );
acado_copyHTH( 1, 17 );
acado_copyHTH( 2, 17 );
acado_copyHTH( 3, 17 );
acado_copyHTH( 4, 17 );
acado_copyHTH( 5, 17 );
acado_copyHTH( 6, 17 );
acado_copyHTH( 7, 17 );
acado_copyHTH( 8, 17 );
acado_copyHTH( 9, 17 );
acado_copyHTH( 10, 17 );
acado_copyHTH( 11, 17 );
acado_copyHTH( 12, 17 );
acado_copyHTH( 13, 17 );
acado_copyHTH( 14, 17 );
acado_copyHTH( 15, 17 );
acado_copyHTH( 16, 17 );
acado_copyHTH( 0, 18 );
acado_copyHTH( 1, 18 );
acado_copyHTH( 2, 18 );
acado_copyHTH( 3, 18 );
acado_copyHTH( 4, 18 );
acado_copyHTH( 5, 18 );
acado_copyHTH( 6, 18 );
acado_copyHTH( 7, 18 );
acado_copyHTH( 8, 18 );
acado_copyHTH( 9, 18 );
acado_copyHTH( 10, 18 );
acado_copyHTH( 11, 18 );
acado_copyHTH( 12, 18 );
acado_copyHTH( 13, 18 );
acado_copyHTH( 14, 18 );
acado_copyHTH( 15, 18 );
acado_copyHTH( 16, 18 );
acado_copyHTH( 17, 18 );
acado_copyHTH( 0, 19 );
acado_copyHTH( 1, 19 );
acado_copyHTH( 2, 19 );
acado_copyHTH( 3, 19 );
acado_copyHTH( 4, 19 );
acado_copyHTH( 5, 19 );
acado_copyHTH( 6, 19 );
acado_copyHTH( 7, 19 );
acado_copyHTH( 8, 19 );
acado_copyHTH( 9, 19 );
acado_copyHTH( 10, 19 );
acado_copyHTH( 11, 19 );
acado_copyHTH( 12, 19 );
acado_copyHTH( 13, 19 );
acado_copyHTH( 14, 19 );
acado_copyHTH( 15, 19 );
acado_copyHTH( 16, 19 );
acado_copyHTH( 17, 19 );
acado_copyHTH( 18, 19 );

for (lRun1 = 0; lRun1 < 160; ++lRun1)
acadoWorkspace.sbar[lRun1 + 8] = acadoWorkspace.d[lRun1];


}

void acado_condenseFdb(  )
{
int lRun1;
acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];
acadoWorkspace.Dx0[6] = acadoVariables.x0[6] - acadoVariables.x[6];
acadoWorkspace.Dx0[7] = acadoVariables.x0[7] - acadoVariables.x[7];
for (lRun1 = 0; lRun1 < 240; ++lRun1)
acadoWorkspace.Dy[lRun1] -= acadoVariables.y[lRun1];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];
acadoWorkspace.DyN[4] -= acadoVariables.yN[4];
acadoWorkspace.DyN[5] -= acadoVariables.yN[5];
acadoWorkspace.DyN[6] -= acadoVariables.yN[6];
acadoWorkspace.DyN[7] -= acadoVariables.yN[7];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 48 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.g[ 4 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 96 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.g[ 8 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 144 ]), &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 192 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.g[ 16 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 240 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.g[ 20 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 288 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 336 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.g[ 28 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 384 ]), &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.g[ 32 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 432 ]), &(acadoWorkspace.Dy[ 108 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 480 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.g[ 40 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 528 ]), &(acadoWorkspace.Dy[ 132 ]), &(acadoWorkspace.g[ 44 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 576 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 624 ]), &(acadoWorkspace.Dy[ 156 ]), &(acadoWorkspace.g[ 52 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 672 ]), &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.g[ 56 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 720 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.g[ 60 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 768 ]), &(acadoWorkspace.Dy[ 192 ]), &(acadoWorkspace.g[ 64 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 816 ]), &(acadoWorkspace.Dy[ 204 ]), &(acadoWorkspace.g[ 68 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 864 ]), &(acadoWorkspace.Dy[ 216 ]), &(acadoWorkspace.g[ 72 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 912 ]), &(acadoWorkspace.Dy[ 228 ]), &(acadoWorkspace.g[ 76 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 96 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.QDy[ 8 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 192 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.QDy[ 16 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 288 ]), &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.QDy[ 24 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 384 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.QDy[ 32 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 480 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.QDy[ 40 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 576 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.QDy[ 48 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 672 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.QDy[ 56 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 768 ]), &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.QDy[ 64 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 864 ]), &(acadoWorkspace.Dy[ 108 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 960 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.QDy[ 80 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1056 ]), &(acadoWorkspace.Dy[ 132 ]), &(acadoWorkspace.QDy[ 88 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1152 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.QDy[ 96 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1248 ]), &(acadoWorkspace.Dy[ 156 ]), &(acadoWorkspace.QDy[ 104 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1344 ]), &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.QDy[ 112 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1440 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.QDy[ 120 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1536 ]), &(acadoWorkspace.Dy[ 192 ]), &(acadoWorkspace.QDy[ 128 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1632 ]), &(acadoWorkspace.Dy[ 204 ]), &(acadoWorkspace.QDy[ 136 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1728 ]), &(acadoWorkspace.Dy[ 216 ]), &(acadoWorkspace.QDy[ 144 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1824 ]), &(acadoWorkspace.Dy[ 228 ]), &(acadoWorkspace.QDy[ 152 ]) );

acadoWorkspace.QDy[160] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[7];
acadoWorkspace.QDy[161] = + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[14]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[15]*acadoWorkspace.DyN[7];
acadoWorkspace.QDy[162] = + acadoWorkspace.QN2[16]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[17]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[18]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[19]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[20]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[21]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[22]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[23]*acadoWorkspace.DyN[7];
acadoWorkspace.QDy[163] = + acadoWorkspace.QN2[24]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[25]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[26]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[27]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[28]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[29]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[30]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[31]*acadoWorkspace.DyN[7];
acadoWorkspace.QDy[164] = + acadoWorkspace.QN2[32]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[33]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[34]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[35]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[36]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[37]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[38]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[39]*acadoWorkspace.DyN[7];
acadoWorkspace.QDy[165] = + acadoWorkspace.QN2[40]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[41]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[42]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[43]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[44]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[45]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[46]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[47]*acadoWorkspace.DyN[7];
acadoWorkspace.QDy[166] = + acadoWorkspace.QN2[48]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[49]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[50]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[51]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[52]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[53]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[54]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[55]*acadoWorkspace.DyN[7];
acadoWorkspace.QDy[167] = + acadoWorkspace.QN2[56]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[57]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[58]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[59]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[60]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[61]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[62]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[63]*acadoWorkspace.DyN[7];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 8 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.sbar[ 16 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 32 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.sbar[ 32 ]), &(acadoWorkspace.sbar[ 40 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 56 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.sbar[ 56 ]), &(acadoWorkspace.sbar[ 64 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.sbar[ 64 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 80 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 640 ]), &(acadoWorkspace.sbar[ 80 ]), &(acadoWorkspace.sbar[ 88 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 704 ]), &(acadoWorkspace.sbar[ 88 ]), &(acadoWorkspace.sbar[ 96 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 768 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.sbar[ 104 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 832 ]), &(acadoWorkspace.sbar[ 104 ]), &(acadoWorkspace.sbar[ 112 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 896 ]), &(acadoWorkspace.sbar[ 112 ]), &(acadoWorkspace.sbar[ 120 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 960 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.sbar[ 128 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1024 ]), &(acadoWorkspace.sbar[ 128 ]), &(acadoWorkspace.sbar[ 136 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1088 ]), &(acadoWorkspace.sbar[ 136 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1152 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 152 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1216 ]), &(acadoWorkspace.sbar[ 152 ]), &(acadoWorkspace.sbar[ 160 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[160] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[161] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[162] + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[163] + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[164] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[165] + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[166] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[167] + acadoWorkspace.QDy[160];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[160] + acadoWorkspace.QN1[9]*acadoWorkspace.sbar[161] + acadoWorkspace.QN1[10]*acadoWorkspace.sbar[162] + acadoWorkspace.QN1[11]*acadoWorkspace.sbar[163] + acadoWorkspace.QN1[12]*acadoWorkspace.sbar[164] + acadoWorkspace.QN1[13]*acadoWorkspace.sbar[165] + acadoWorkspace.QN1[14]*acadoWorkspace.sbar[166] + acadoWorkspace.QN1[15]*acadoWorkspace.sbar[167] + acadoWorkspace.QDy[161];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[16]*acadoWorkspace.sbar[160] + acadoWorkspace.QN1[17]*acadoWorkspace.sbar[161] + acadoWorkspace.QN1[18]*acadoWorkspace.sbar[162] + acadoWorkspace.QN1[19]*acadoWorkspace.sbar[163] + acadoWorkspace.QN1[20]*acadoWorkspace.sbar[164] + acadoWorkspace.QN1[21]*acadoWorkspace.sbar[165] + acadoWorkspace.QN1[22]*acadoWorkspace.sbar[166] + acadoWorkspace.QN1[23]*acadoWorkspace.sbar[167] + acadoWorkspace.QDy[162];
acadoWorkspace.w1[3] = + acadoWorkspace.QN1[24]*acadoWorkspace.sbar[160] + acadoWorkspace.QN1[25]*acadoWorkspace.sbar[161] + acadoWorkspace.QN1[26]*acadoWorkspace.sbar[162] + acadoWorkspace.QN1[27]*acadoWorkspace.sbar[163] + acadoWorkspace.QN1[28]*acadoWorkspace.sbar[164] + acadoWorkspace.QN1[29]*acadoWorkspace.sbar[165] + acadoWorkspace.QN1[30]*acadoWorkspace.sbar[166] + acadoWorkspace.QN1[31]*acadoWorkspace.sbar[167] + acadoWorkspace.QDy[163];
acadoWorkspace.w1[4] = + acadoWorkspace.QN1[32]*acadoWorkspace.sbar[160] + acadoWorkspace.QN1[33]*acadoWorkspace.sbar[161] + acadoWorkspace.QN1[34]*acadoWorkspace.sbar[162] + acadoWorkspace.QN1[35]*acadoWorkspace.sbar[163] + acadoWorkspace.QN1[36]*acadoWorkspace.sbar[164] + acadoWorkspace.QN1[37]*acadoWorkspace.sbar[165] + acadoWorkspace.QN1[38]*acadoWorkspace.sbar[166] + acadoWorkspace.QN1[39]*acadoWorkspace.sbar[167] + acadoWorkspace.QDy[164];
acadoWorkspace.w1[5] = + acadoWorkspace.QN1[40]*acadoWorkspace.sbar[160] + acadoWorkspace.QN1[41]*acadoWorkspace.sbar[161] + acadoWorkspace.QN1[42]*acadoWorkspace.sbar[162] + acadoWorkspace.QN1[43]*acadoWorkspace.sbar[163] + acadoWorkspace.QN1[44]*acadoWorkspace.sbar[164] + acadoWorkspace.QN1[45]*acadoWorkspace.sbar[165] + acadoWorkspace.QN1[46]*acadoWorkspace.sbar[166] + acadoWorkspace.QN1[47]*acadoWorkspace.sbar[167] + acadoWorkspace.QDy[165];
acadoWorkspace.w1[6] = + acadoWorkspace.QN1[48]*acadoWorkspace.sbar[160] + acadoWorkspace.QN1[49]*acadoWorkspace.sbar[161] + acadoWorkspace.QN1[50]*acadoWorkspace.sbar[162] + acadoWorkspace.QN1[51]*acadoWorkspace.sbar[163] + acadoWorkspace.QN1[52]*acadoWorkspace.sbar[164] + acadoWorkspace.QN1[53]*acadoWorkspace.sbar[165] + acadoWorkspace.QN1[54]*acadoWorkspace.sbar[166] + acadoWorkspace.QN1[55]*acadoWorkspace.sbar[167] + acadoWorkspace.QDy[166];
acadoWorkspace.w1[7] = + acadoWorkspace.QN1[56]*acadoWorkspace.sbar[160] + acadoWorkspace.QN1[57]*acadoWorkspace.sbar[161] + acadoWorkspace.QN1[58]*acadoWorkspace.sbar[162] + acadoWorkspace.QN1[59]*acadoWorkspace.sbar[163] + acadoWorkspace.QN1[60]*acadoWorkspace.sbar[164] + acadoWorkspace.QN1[61]*acadoWorkspace.sbar[165] + acadoWorkspace.QN1[62]*acadoWorkspace.sbar[166] + acadoWorkspace.QN1[63]*acadoWorkspace.sbar[167] + acadoWorkspace.QDy[167];
acado_macBTw1( &(acadoWorkspace.evGu[ 608 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 76 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1216 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 152 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1216 ]), &(acadoWorkspace.sbar[ 152 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 576 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 72 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1152 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 144 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1152 ]), &(acadoWorkspace.sbar[ 144 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 544 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 68 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1088 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 136 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1088 ]), &(acadoWorkspace.sbar[ 136 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 512 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 64 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1024 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 128 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1024 ]), &(acadoWorkspace.sbar[ 128 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 480 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 60 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 960 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 120 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 960 ]), &(acadoWorkspace.sbar[ 120 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 448 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 56 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 896 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 112 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 896 ]), &(acadoWorkspace.sbar[ 112 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 416 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 52 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 832 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 104 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 832 ]), &(acadoWorkspace.sbar[ 104 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 384 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 48 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 768 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 96 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 768 ]), &(acadoWorkspace.sbar[ 96 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 352 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 44 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 704 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 88 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 704 ]), &(acadoWorkspace.sbar[ 88 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 320 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 40 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 640 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 80 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 640 ]), &(acadoWorkspace.sbar[ 80 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 288 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 36 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 576 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 72 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 576 ]), &(acadoWorkspace.sbar[ 72 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 256 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 32 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 512 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 64 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 512 ]), &(acadoWorkspace.sbar[ 64 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 224 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 28 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 448 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 56 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 448 ]), &(acadoWorkspace.sbar[ 56 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 192 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 24 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 384 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 48 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 384 ]), &(acadoWorkspace.sbar[ 48 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 160 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 20 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 320 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 40 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 320 ]), &(acadoWorkspace.sbar[ 40 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 128 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 16 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 256 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 32 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 256 ]), &(acadoWorkspace.sbar[ 32 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 96 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 192 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 24 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 192 ]), &(acadoWorkspace.sbar[ 24 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 64 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 8 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 128 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 16 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 128 ]), &(acadoWorkspace.sbar[ 16 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 32 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 4 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 64 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 8 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 64 ]), &(acadoWorkspace.sbar[ 8 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );

acadoWorkspace.lb[0] = acadoVariables.lbValues[0] - acadoVariables.u[0];
acadoWorkspace.lb[1] = acadoVariables.lbValues[1] - acadoVariables.u[1];
acadoWorkspace.lb[2] = acadoVariables.lbValues[2] - acadoVariables.u[2];
acadoWorkspace.lb[3] = acadoVariables.lbValues[3] - acadoVariables.u[3];
acadoWorkspace.lb[4] = acadoVariables.lbValues[4] - acadoVariables.u[4];
acadoWorkspace.lb[5] = acadoVariables.lbValues[5] - acadoVariables.u[5];
acadoWorkspace.lb[6] = acadoVariables.lbValues[6] - acadoVariables.u[6];
acadoWorkspace.lb[7] = acadoVariables.lbValues[7] - acadoVariables.u[7];
acadoWorkspace.lb[8] = acadoVariables.lbValues[8] - acadoVariables.u[8];
acadoWorkspace.lb[9] = acadoVariables.lbValues[9] - acadoVariables.u[9];
acadoWorkspace.lb[10] = acadoVariables.lbValues[10] - acadoVariables.u[10];
acadoWorkspace.lb[11] = acadoVariables.lbValues[11] - acadoVariables.u[11];
acadoWorkspace.lb[12] = acadoVariables.lbValues[12] - acadoVariables.u[12];
acadoWorkspace.lb[13] = acadoVariables.lbValues[13] - acadoVariables.u[13];
acadoWorkspace.lb[14] = acadoVariables.lbValues[14] - acadoVariables.u[14];
acadoWorkspace.lb[15] = acadoVariables.lbValues[15] - acadoVariables.u[15];
acadoWorkspace.lb[16] = acadoVariables.lbValues[16] - acadoVariables.u[16];
acadoWorkspace.lb[17] = acadoVariables.lbValues[17] - acadoVariables.u[17];
acadoWorkspace.lb[18] = acadoVariables.lbValues[18] - acadoVariables.u[18];
acadoWorkspace.lb[19] = acadoVariables.lbValues[19] - acadoVariables.u[19];
acadoWorkspace.lb[20] = acadoVariables.lbValues[20] - acadoVariables.u[20];
acadoWorkspace.lb[21] = acadoVariables.lbValues[21] - acadoVariables.u[21];
acadoWorkspace.lb[22] = acadoVariables.lbValues[22] - acadoVariables.u[22];
acadoWorkspace.lb[23] = acadoVariables.lbValues[23] - acadoVariables.u[23];
acadoWorkspace.lb[24] = acadoVariables.lbValues[24] - acadoVariables.u[24];
acadoWorkspace.lb[25] = acadoVariables.lbValues[25] - acadoVariables.u[25];
acadoWorkspace.lb[26] = acadoVariables.lbValues[26] - acadoVariables.u[26];
acadoWorkspace.lb[27] = acadoVariables.lbValues[27] - acadoVariables.u[27];
acadoWorkspace.lb[28] = acadoVariables.lbValues[28] - acadoVariables.u[28];
acadoWorkspace.lb[29] = acadoVariables.lbValues[29] - acadoVariables.u[29];
acadoWorkspace.lb[30] = acadoVariables.lbValues[30] - acadoVariables.u[30];
acadoWorkspace.lb[31] = acadoVariables.lbValues[31] - acadoVariables.u[31];
acadoWorkspace.lb[32] = acadoVariables.lbValues[32] - acadoVariables.u[32];
acadoWorkspace.lb[33] = acadoVariables.lbValues[33] - acadoVariables.u[33];
acadoWorkspace.lb[34] = acadoVariables.lbValues[34] - acadoVariables.u[34];
acadoWorkspace.lb[35] = acadoVariables.lbValues[35] - acadoVariables.u[35];
acadoWorkspace.lb[36] = acadoVariables.lbValues[36] - acadoVariables.u[36];
acadoWorkspace.lb[37] = acadoVariables.lbValues[37] - acadoVariables.u[37];
acadoWorkspace.lb[38] = acadoVariables.lbValues[38] - acadoVariables.u[38];
acadoWorkspace.lb[39] = acadoVariables.lbValues[39] - acadoVariables.u[39];
acadoWorkspace.lb[40] = acadoVariables.lbValues[40] - acadoVariables.u[40];
acadoWorkspace.lb[41] = acadoVariables.lbValues[41] - acadoVariables.u[41];
acadoWorkspace.lb[42] = acadoVariables.lbValues[42] - acadoVariables.u[42];
acadoWorkspace.lb[43] = acadoVariables.lbValues[43] - acadoVariables.u[43];
acadoWorkspace.lb[44] = acadoVariables.lbValues[44] - acadoVariables.u[44];
acadoWorkspace.lb[45] = acadoVariables.lbValues[45] - acadoVariables.u[45];
acadoWorkspace.lb[46] = acadoVariables.lbValues[46] - acadoVariables.u[46];
acadoWorkspace.lb[47] = acadoVariables.lbValues[47] - acadoVariables.u[47];
acadoWorkspace.lb[48] = acadoVariables.lbValues[48] - acadoVariables.u[48];
acadoWorkspace.lb[49] = acadoVariables.lbValues[49] - acadoVariables.u[49];
acadoWorkspace.lb[50] = acadoVariables.lbValues[50] - acadoVariables.u[50];
acadoWorkspace.lb[51] = acadoVariables.lbValues[51] - acadoVariables.u[51];
acadoWorkspace.lb[52] = acadoVariables.lbValues[52] - acadoVariables.u[52];
acadoWorkspace.lb[53] = acadoVariables.lbValues[53] - acadoVariables.u[53];
acadoWorkspace.lb[54] = acadoVariables.lbValues[54] - acadoVariables.u[54];
acadoWorkspace.lb[55] = acadoVariables.lbValues[55] - acadoVariables.u[55];
acadoWorkspace.lb[56] = acadoVariables.lbValues[56] - acadoVariables.u[56];
acadoWorkspace.lb[57] = acadoVariables.lbValues[57] - acadoVariables.u[57];
acadoWorkspace.lb[58] = acadoVariables.lbValues[58] - acadoVariables.u[58];
acadoWorkspace.lb[59] = acadoVariables.lbValues[59] - acadoVariables.u[59];
acadoWorkspace.lb[60] = acadoVariables.lbValues[60] - acadoVariables.u[60];
acadoWorkspace.lb[61] = acadoVariables.lbValues[61] - acadoVariables.u[61];
acadoWorkspace.lb[62] = acadoVariables.lbValues[62] - acadoVariables.u[62];
acadoWorkspace.lb[63] = acadoVariables.lbValues[63] - acadoVariables.u[63];
acadoWorkspace.lb[64] = acadoVariables.lbValues[64] - acadoVariables.u[64];
acadoWorkspace.lb[65] = acadoVariables.lbValues[65] - acadoVariables.u[65];
acadoWorkspace.lb[66] = acadoVariables.lbValues[66] - acadoVariables.u[66];
acadoWorkspace.lb[67] = acadoVariables.lbValues[67] - acadoVariables.u[67];
acadoWorkspace.lb[68] = acadoVariables.lbValues[68] - acadoVariables.u[68];
acadoWorkspace.lb[69] = acadoVariables.lbValues[69] - acadoVariables.u[69];
acadoWorkspace.lb[70] = acadoVariables.lbValues[70] - acadoVariables.u[70];
acadoWorkspace.lb[71] = acadoVariables.lbValues[71] - acadoVariables.u[71];
acadoWorkspace.lb[72] = acadoVariables.lbValues[72] - acadoVariables.u[72];
acadoWorkspace.lb[73] = acadoVariables.lbValues[73] - acadoVariables.u[73];
acadoWorkspace.lb[74] = acadoVariables.lbValues[74] - acadoVariables.u[74];
acadoWorkspace.lb[75] = acadoVariables.lbValues[75] - acadoVariables.u[75];
acadoWorkspace.lb[76] = acadoVariables.lbValues[76] - acadoVariables.u[76];
acadoWorkspace.lb[77] = acadoVariables.lbValues[77] - acadoVariables.u[77];
acadoWorkspace.lb[78] = acadoVariables.lbValues[78] - acadoVariables.u[78];
acadoWorkspace.lb[79] = acadoVariables.lbValues[79] - acadoVariables.u[79];
acadoWorkspace.ub[0] = acadoVariables.ubValues[0] - acadoVariables.u[0];
acadoWorkspace.ub[1] = acadoVariables.ubValues[1] - acadoVariables.u[1];
acadoWorkspace.ub[2] = acadoVariables.ubValues[2] - acadoVariables.u[2];
acadoWorkspace.ub[3] = acadoVariables.ubValues[3] - acadoVariables.u[3];
acadoWorkspace.ub[4] = acadoVariables.ubValues[4] - acadoVariables.u[4];
acadoWorkspace.ub[5] = acadoVariables.ubValues[5] - acadoVariables.u[5];
acadoWorkspace.ub[6] = acadoVariables.ubValues[6] - acadoVariables.u[6];
acadoWorkspace.ub[7] = acadoVariables.ubValues[7] - acadoVariables.u[7];
acadoWorkspace.ub[8] = acadoVariables.ubValues[8] - acadoVariables.u[8];
acadoWorkspace.ub[9] = acadoVariables.ubValues[9] - acadoVariables.u[9];
acadoWorkspace.ub[10] = acadoVariables.ubValues[10] - acadoVariables.u[10];
acadoWorkspace.ub[11] = acadoVariables.ubValues[11] - acadoVariables.u[11];
acadoWorkspace.ub[12] = acadoVariables.ubValues[12] - acadoVariables.u[12];
acadoWorkspace.ub[13] = acadoVariables.ubValues[13] - acadoVariables.u[13];
acadoWorkspace.ub[14] = acadoVariables.ubValues[14] - acadoVariables.u[14];
acadoWorkspace.ub[15] = acadoVariables.ubValues[15] - acadoVariables.u[15];
acadoWorkspace.ub[16] = acadoVariables.ubValues[16] - acadoVariables.u[16];
acadoWorkspace.ub[17] = acadoVariables.ubValues[17] - acadoVariables.u[17];
acadoWorkspace.ub[18] = acadoVariables.ubValues[18] - acadoVariables.u[18];
acadoWorkspace.ub[19] = acadoVariables.ubValues[19] - acadoVariables.u[19];
acadoWorkspace.ub[20] = acadoVariables.ubValues[20] - acadoVariables.u[20];
acadoWorkspace.ub[21] = acadoVariables.ubValues[21] - acadoVariables.u[21];
acadoWorkspace.ub[22] = acadoVariables.ubValues[22] - acadoVariables.u[22];
acadoWorkspace.ub[23] = acadoVariables.ubValues[23] - acadoVariables.u[23];
acadoWorkspace.ub[24] = acadoVariables.ubValues[24] - acadoVariables.u[24];
acadoWorkspace.ub[25] = acadoVariables.ubValues[25] - acadoVariables.u[25];
acadoWorkspace.ub[26] = acadoVariables.ubValues[26] - acadoVariables.u[26];
acadoWorkspace.ub[27] = acadoVariables.ubValues[27] - acadoVariables.u[27];
acadoWorkspace.ub[28] = acadoVariables.ubValues[28] - acadoVariables.u[28];
acadoWorkspace.ub[29] = acadoVariables.ubValues[29] - acadoVariables.u[29];
acadoWorkspace.ub[30] = acadoVariables.ubValues[30] - acadoVariables.u[30];
acadoWorkspace.ub[31] = acadoVariables.ubValues[31] - acadoVariables.u[31];
acadoWorkspace.ub[32] = acadoVariables.ubValues[32] - acadoVariables.u[32];
acadoWorkspace.ub[33] = acadoVariables.ubValues[33] - acadoVariables.u[33];
acadoWorkspace.ub[34] = acadoVariables.ubValues[34] - acadoVariables.u[34];
acadoWorkspace.ub[35] = acadoVariables.ubValues[35] - acadoVariables.u[35];
acadoWorkspace.ub[36] = acadoVariables.ubValues[36] - acadoVariables.u[36];
acadoWorkspace.ub[37] = acadoVariables.ubValues[37] - acadoVariables.u[37];
acadoWorkspace.ub[38] = acadoVariables.ubValues[38] - acadoVariables.u[38];
acadoWorkspace.ub[39] = acadoVariables.ubValues[39] - acadoVariables.u[39];
acadoWorkspace.ub[40] = acadoVariables.ubValues[40] - acadoVariables.u[40];
acadoWorkspace.ub[41] = acadoVariables.ubValues[41] - acadoVariables.u[41];
acadoWorkspace.ub[42] = acadoVariables.ubValues[42] - acadoVariables.u[42];
acadoWorkspace.ub[43] = acadoVariables.ubValues[43] - acadoVariables.u[43];
acadoWorkspace.ub[44] = acadoVariables.ubValues[44] - acadoVariables.u[44];
acadoWorkspace.ub[45] = acadoVariables.ubValues[45] - acadoVariables.u[45];
acadoWorkspace.ub[46] = acadoVariables.ubValues[46] - acadoVariables.u[46];
acadoWorkspace.ub[47] = acadoVariables.ubValues[47] - acadoVariables.u[47];
acadoWorkspace.ub[48] = acadoVariables.ubValues[48] - acadoVariables.u[48];
acadoWorkspace.ub[49] = acadoVariables.ubValues[49] - acadoVariables.u[49];
acadoWorkspace.ub[50] = acadoVariables.ubValues[50] - acadoVariables.u[50];
acadoWorkspace.ub[51] = acadoVariables.ubValues[51] - acadoVariables.u[51];
acadoWorkspace.ub[52] = acadoVariables.ubValues[52] - acadoVariables.u[52];
acadoWorkspace.ub[53] = acadoVariables.ubValues[53] - acadoVariables.u[53];
acadoWorkspace.ub[54] = acadoVariables.ubValues[54] - acadoVariables.u[54];
acadoWorkspace.ub[55] = acadoVariables.ubValues[55] - acadoVariables.u[55];
acadoWorkspace.ub[56] = acadoVariables.ubValues[56] - acadoVariables.u[56];
acadoWorkspace.ub[57] = acadoVariables.ubValues[57] - acadoVariables.u[57];
acadoWorkspace.ub[58] = acadoVariables.ubValues[58] - acadoVariables.u[58];
acadoWorkspace.ub[59] = acadoVariables.ubValues[59] - acadoVariables.u[59];
acadoWorkspace.ub[60] = acadoVariables.ubValues[60] - acadoVariables.u[60];
acadoWorkspace.ub[61] = acadoVariables.ubValues[61] - acadoVariables.u[61];
acadoWorkspace.ub[62] = acadoVariables.ubValues[62] - acadoVariables.u[62];
acadoWorkspace.ub[63] = acadoVariables.ubValues[63] - acadoVariables.u[63];
acadoWorkspace.ub[64] = acadoVariables.ubValues[64] - acadoVariables.u[64];
acadoWorkspace.ub[65] = acadoVariables.ubValues[65] - acadoVariables.u[65];
acadoWorkspace.ub[66] = acadoVariables.ubValues[66] - acadoVariables.u[66];
acadoWorkspace.ub[67] = acadoVariables.ubValues[67] - acadoVariables.u[67];
acadoWorkspace.ub[68] = acadoVariables.ubValues[68] - acadoVariables.u[68];
acadoWorkspace.ub[69] = acadoVariables.ubValues[69] - acadoVariables.u[69];
acadoWorkspace.ub[70] = acadoVariables.ubValues[70] - acadoVariables.u[70];
acadoWorkspace.ub[71] = acadoVariables.ubValues[71] - acadoVariables.u[71];
acadoWorkspace.ub[72] = acadoVariables.ubValues[72] - acadoVariables.u[72];
acadoWorkspace.ub[73] = acadoVariables.ubValues[73] - acadoVariables.u[73];
acadoWorkspace.ub[74] = acadoVariables.ubValues[74] - acadoVariables.u[74];
acadoWorkspace.ub[75] = acadoVariables.ubValues[75] - acadoVariables.u[75];
acadoWorkspace.ub[76] = acadoVariables.ubValues[76] - acadoVariables.u[76];
acadoWorkspace.ub[77] = acadoVariables.ubValues[77] - acadoVariables.u[77];
acadoWorkspace.ub[78] = acadoVariables.ubValues[78] - acadoVariables.u[78];
acadoWorkspace.ub[79] = acadoVariables.ubValues[79] - acadoVariables.u[79];

}

void acado_expand(  )
{
int lRun1;
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoVariables.u[40] += acadoWorkspace.x[40];
acadoVariables.u[41] += acadoWorkspace.x[41];
acadoVariables.u[42] += acadoWorkspace.x[42];
acadoVariables.u[43] += acadoWorkspace.x[43];
acadoVariables.u[44] += acadoWorkspace.x[44];
acadoVariables.u[45] += acadoWorkspace.x[45];
acadoVariables.u[46] += acadoWorkspace.x[46];
acadoVariables.u[47] += acadoWorkspace.x[47];
acadoVariables.u[48] += acadoWorkspace.x[48];
acadoVariables.u[49] += acadoWorkspace.x[49];
acadoVariables.u[50] += acadoWorkspace.x[50];
acadoVariables.u[51] += acadoWorkspace.x[51];
acadoVariables.u[52] += acadoWorkspace.x[52];
acadoVariables.u[53] += acadoWorkspace.x[53];
acadoVariables.u[54] += acadoWorkspace.x[54];
acadoVariables.u[55] += acadoWorkspace.x[55];
acadoVariables.u[56] += acadoWorkspace.x[56];
acadoVariables.u[57] += acadoWorkspace.x[57];
acadoVariables.u[58] += acadoWorkspace.x[58];
acadoVariables.u[59] += acadoWorkspace.x[59];
acadoVariables.u[60] += acadoWorkspace.x[60];
acadoVariables.u[61] += acadoWorkspace.x[61];
acadoVariables.u[62] += acadoWorkspace.x[62];
acadoVariables.u[63] += acadoWorkspace.x[63];
acadoVariables.u[64] += acadoWorkspace.x[64];
acadoVariables.u[65] += acadoWorkspace.x[65];
acadoVariables.u[66] += acadoWorkspace.x[66];
acadoVariables.u[67] += acadoWorkspace.x[67];
acadoVariables.u[68] += acadoWorkspace.x[68];
acadoVariables.u[69] += acadoWorkspace.x[69];
acadoVariables.u[70] += acadoWorkspace.x[70];
acadoVariables.u[71] += acadoWorkspace.x[71];
acadoVariables.u[72] += acadoWorkspace.x[72];
acadoVariables.u[73] += acadoWorkspace.x[73];
acadoVariables.u[74] += acadoWorkspace.x[74];
acadoVariables.u[75] += acadoWorkspace.x[75];
acadoVariables.u[76] += acadoWorkspace.x[76];
acadoVariables.u[77] += acadoWorkspace.x[77];
acadoVariables.u[78] += acadoWorkspace.x[78];
acadoVariables.u[79] += acadoWorkspace.x[79];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
for (lRun1 = 0; lRun1 < 160; ++lRun1)
acadoWorkspace.sbar[lRun1 + 8] = acadoWorkspace.d[lRun1];

acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 8 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 64 ]), &(acadoWorkspace.evGu[ 32 ]), &(acadoWorkspace.x[ 4 ]), &(acadoWorkspace.sbar[ 8 ]), &(acadoWorkspace.sbar[ 16 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 128 ]), &(acadoWorkspace.evGu[ 64 ]), &(acadoWorkspace.x[ 8 ]), &(acadoWorkspace.sbar[ 16 ]), &(acadoWorkspace.sbar[ 24 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 192 ]), &(acadoWorkspace.evGu[ 96 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 24 ]), &(acadoWorkspace.sbar[ 32 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 256 ]), &(acadoWorkspace.evGu[ 128 ]), &(acadoWorkspace.x[ 16 ]), &(acadoWorkspace.sbar[ 32 ]), &(acadoWorkspace.sbar[ 40 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 320 ]), &(acadoWorkspace.evGu[ 160 ]), &(acadoWorkspace.x[ 20 ]), &(acadoWorkspace.sbar[ 40 ]), &(acadoWorkspace.sbar[ 48 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 384 ]), &(acadoWorkspace.evGu[ 192 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.sbar[ 48 ]), &(acadoWorkspace.sbar[ 56 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 448 ]), &(acadoWorkspace.evGu[ 224 ]), &(acadoWorkspace.x[ 28 ]), &(acadoWorkspace.sbar[ 56 ]), &(acadoWorkspace.sbar[ 64 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 512 ]), &(acadoWorkspace.evGu[ 256 ]), &(acadoWorkspace.x[ 32 ]), &(acadoWorkspace.sbar[ 64 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 576 ]), &(acadoWorkspace.evGu[ 288 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 80 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 640 ]), &(acadoWorkspace.evGu[ 320 ]), &(acadoWorkspace.x[ 40 ]), &(acadoWorkspace.sbar[ 80 ]), &(acadoWorkspace.sbar[ 88 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 704 ]), &(acadoWorkspace.evGu[ 352 ]), &(acadoWorkspace.x[ 44 ]), &(acadoWorkspace.sbar[ 88 ]), &(acadoWorkspace.sbar[ 96 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 768 ]), &(acadoWorkspace.evGu[ 384 ]), &(acadoWorkspace.x[ 48 ]), &(acadoWorkspace.sbar[ 96 ]), &(acadoWorkspace.sbar[ 104 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 832 ]), &(acadoWorkspace.evGu[ 416 ]), &(acadoWorkspace.x[ 52 ]), &(acadoWorkspace.sbar[ 104 ]), &(acadoWorkspace.sbar[ 112 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 896 ]), &(acadoWorkspace.evGu[ 448 ]), &(acadoWorkspace.x[ 56 ]), &(acadoWorkspace.sbar[ 112 ]), &(acadoWorkspace.sbar[ 120 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 960 ]), &(acadoWorkspace.evGu[ 480 ]), &(acadoWorkspace.x[ 60 ]), &(acadoWorkspace.sbar[ 120 ]), &(acadoWorkspace.sbar[ 128 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1024 ]), &(acadoWorkspace.evGu[ 512 ]), &(acadoWorkspace.x[ 64 ]), &(acadoWorkspace.sbar[ 128 ]), &(acadoWorkspace.sbar[ 136 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1088 ]), &(acadoWorkspace.evGu[ 544 ]), &(acadoWorkspace.x[ 68 ]), &(acadoWorkspace.sbar[ 136 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1152 ]), &(acadoWorkspace.evGu[ 576 ]), &(acadoWorkspace.x[ 72 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 152 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1216 ]), &(acadoWorkspace.evGu[ 608 ]), &(acadoWorkspace.x[ 76 ]), &(acadoWorkspace.sbar[ 152 ]), &(acadoWorkspace.sbar[ 160 ]) );
for (lRun1 = 0; lRun1 < 168; ++lRun1)
acadoVariables.x[lRun1] += acadoWorkspace.sbar[lRun1];

}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
acadoVariables.lbValues[0] = -5.6407500000000005e+02;
acadoVariables.lbValues[1] = -5.6407500000000005e+02;
acadoVariables.lbValues[2] = -5.6407500000000005e+02;
acadoVariables.lbValues[3] = -5.6407500000000005e+02;
acadoVariables.lbValues[4] = -5.6407500000000005e+02;
acadoVariables.lbValues[5] = -5.6407500000000005e+02;
acadoVariables.lbValues[6] = -5.6407500000000005e+02;
acadoVariables.lbValues[7] = -5.6407500000000005e+02;
acadoVariables.lbValues[8] = -5.6407500000000005e+02;
acadoVariables.lbValues[9] = -5.6407500000000005e+02;
acadoVariables.lbValues[10] = -5.6407500000000005e+02;
acadoVariables.lbValues[11] = -5.6407500000000005e+02;
acadoVariables.lbValues[12] = -5.6407500000000005e+02;
acadoVariables.lbValues[13] = -5.6407500000000005e+02;
acadoVariables.lbValues[14] = -5.6407500000000005e+02;
acadoVariables.lbValues[15] = -5.6407500000000005e+02;
acadoVariables.lbValues[16] = -5.6407500000000005e+02;
acadoVariables.lbValues[17] = -5.6407500000000005e+02;
acadoVariables.lbValues[18] = -5.6407500000000005e+02;
acadoVariables.lbValues[19] = -5.6407500000000005e+02;
acadoVariables.lbValues[20] = -5.6407500000000005e+02;
acadoVariables.lbValues[21] = -5.6407500000000005e+02;
acadoVariables.lbValues[22] = -5.6407500000000005e+02;
acadoVariables.lbValues[23] = -5.6407500000000005e+02;
acadoVariables.lbValues[24] = -5.6407500000000005e+02;
acadoVariables.lbValues[25] = -5.6407500000000005e+02;
acadoVariables.lbValues[26] = -5.6407500000000005e+02;
acadoVariables.lbValues[27] = -5.6407500000000005e+02;
acadoVariables.lbValues[28] = -5.6407500000000005e+02;
acadoVariables.lbValues[29] = -5.6407500000000005e+02;
acadoVariables.lbValues[30] = -5.6407500000000005e+02;
acadoVariables.lbValues[31] = -5.6407500000000005e+02;
acadoVariables.lbValues[32] = -5.6407500000000005e+02;
acadoVariables.lbValues[33] = -5.6407500000000005e+02;
acadoVariables.lbValues[34] = -5.6407500000000005e+02;
acadoVariables.lbValues[35] = -5.6407500000000005e+02;
acadoVariables.lbValues[36] = -5.6407500000000005e+02;
acadoVariables.lbValues[37] = -5.6407500000000005e+02;
acadoVariables.lbValues[38] = -5.6407500000000005e+02;
acadoVariables.lbValues[39] = -5.6407500000000005e+02;
acadoVariables.lbValues[40] = -5.6407500000000005e+02;
acadoVariables.lbValues[41] = -5.6407500000000005e+02;
acadoVariables.lbValues[42] = -5.6407500000000005e+02;
acadoVariables.lbValues[43] = -5.6407500000000005e+02;
acadoVariables.lbValues[44] = -5.6407500000000005e+02;
acadoVariables.lbValues[45] = -5.6407500000000005e+02;
acadoVariables.lbValues[46] = -5.6407500000000005e+02;
acadoVariables.lbValues[47] = -5.6407500000000005e+02;
acadoVariables.lbValues[48] = -5.6407500000000005e+02;
acadoVariables.lbValues[49] = -5.6407500000000005e+02;
acadoVariables.lbValues[50] = -5.6407500000000005e+02;
acadoVariables.lbValues[51] = -5.6407500000000005e+02;
acadoVariables.lbValues[52] = -5.6407500000000005e+02;
acadoVariables.lbValues[53] = -5.6407500000000005e+02;
acadoVariables.lbValues[54] = -5.6407500000000005e+02;
acadoVariables.lbValues[55] = -5.6407500000000005e+02;
acadoVariables.lbValues[56] = -5.6407500000000005e+02;
acadoVariables.lbValues[57] = -5.6407500000000005e+02;
acadoVariables.lbValues[58] = -5.6407500000000005e+02;
acadoVariables.lbValues[59] = -5.6407500000000005e+02;
acadoVariables.lbValues[60] = -5.6407500000000005e+02;
acadoVariables.lbValues[61] = -5.6407500000000005e+02;
acadoVariables.lbValues[62] = -5.6407500000000005e+02;
acadoVariables.lbValues[63] = -5.6407500000000005e+02;
acadoVariables.lbValues[64] = -5.6407500000000005e+02;
acadoVariables.lbValues[65] = -5.6407500000000005e+02;
acadoVariables.lbValues[66] = -5.6407500000000005e+02;
acadoVariables.lbValues[67] = -5.6407500000000005e+02;
acadoVariables.lbValues[68] = -5.6407500000000005e+02;
acadoVariables.lbValues[69] = -5.6407500000000005e+02;
acadoVariables.lbValues[70] = -5.6407500000000005e+02;
acadoVariables.lbValues[71] = -5.6407500000000005e+02;
acadoVariables.lbValues[72] = -5.6407500000000005e+02;
acadoVariables.lbValues[73] = -5.6407500000000005e+02;
acadoVariables.lbValues[74] = -5.6407500000000005e+02;
acadoVariables.lbValues[75] = -5.6407500000000005e+02;
acadoVariables.lbValues[76] = -5.6407500000000005e+02;
acadoVariables.lbValues[77] = -5.6407500000000005e+02;
acadoVariables.lbValues[78] = -5.6407500000000005e+02;
acadoVariables.lbValues[79] = -5.6407500000000005e+02;
acadoVariables.ubValues[0] = 5.6407500000000005e+02;
acadoVariables.ubValues[1] = 5.6407500000000005e+02;
acadoVariables.ubValues[2] = 5.6407500000000005e+02;
acadoVariables.ubValues[3] = 5.6407500000000005e+02;
acadoVariables.ubValues[4] = 5.6407500000000005e+02;
acadoVariables.ubValues[5] = 5.6407500000000005e+02;
acadoVariables.ubValues[6] = 5.6407500000000005e+02;
acadoVariables.ubValues[7] = 5.6407500000000005e+02;
acadoVariables.ubValues[8] = 5.6407500000000005e+02;
acadoVariables.ubValues[9] = 5.6407500000000005e+02;
acadoVariables.ubValues[10] = 5.6407500000000005e+02;
acadoVariables.ubValues[11] = 5.6407500000000005e+02;
acadoVariables.ubValues[12] = 5.6407500000000005e+02;
acadoVariables.ubValues[13] = 5.6407500000000005e+02;
acadoVariables.ubValues[14] = 5.6407500000000005e+02;
acadoVariables.ubValues[15] = 5.6407500000000005e+02;
acadoVariables.ubValues[16] = 5.6407500000000005e+02;
acadoVariables.ubValues[17] = 5.6407500000000005e+02;
acadoVariables.ubValues[18] = 5.6407500000000005e+02;
acadoVariables.ubValues[19] = 5.6407500000000005e+02;
acadoVariables.ubValues[20] = 5.6407500000000005e+02;
acadoVariables.ubValues[21] = 5.6407500000000005e+02;
acadoVariables.ubValues[22] = 5.6407500000000005e+02;
acadoVariables.ubValues[23] = 5.6407500000000005e+02;
acadoVariables.ubValues[24] = 5.6407500000000005e+02;
acadoVariables.ubValues[25] = 5.6407500000000005e+02;
acadoVariables.ubValues[26] = 5.6407500000000005e+02;
acadoVariables.ubValues[27] = 5.6407500000000005e+02;
acadoVariables.ubValues[28] = 5.6407500000000005e+02;
acadoVariables.ubValues[29] = 5.6407500000000005e+02;
acadoVariables.ubValues[30] = 5.6407500000000005e+02;
acadoVariables.ubValues[31] = 5.6407500000000005e+02;
acadoVariables.ubValues[32] = 5.6407500000000005e+02;
acadoVariables.ubValues[33] = 5.6407500000000005e+02;
acadoVariables.ubValues[34] = 5.6407500000000005e+02;
acadoVariables.ubValues[35] = 5.6407500000000005e+02;
acadoVariables.ubValues[36] = 5.6407500000000005e+02;
acadoVariables.ubValues[37] = 5.6407500000000005e+02;
acadoVariables.ubValues[38] = 5.6407500000000005e+02;
acadoVariables.ubValues[39] = 5.6407500000000005e+02;
acadoVariables.ubValues[40] = 5.6407500000000005e+02;
acadoVariables.ubValues[41] = 5.6407500000000005e+02;
acadoVariables.ubValues[42] = 5.6407500000000005e+02;
acadoVariables.ubValues[43] = 5.6407500000000005e+02;
acadoVariables.ubValues[44] = 5.6407500000000005e+02;
acadoVariables.ubValues[45] = 5.6407500000000005e+02;
acadoVariables.ubValues[46] = 5.6407500000000005e+02;
acadoVariables.ubValues[47] = 5.6407500000000005e+02;
acadoVariables.ubValues[48] = 5.6407500000000005e+02;
acadoVariables.ubValues[49] = 5.6407500000000005e+02;
acadoVariables.ubValues[50] = 5.6407500000000005e+02;
acadoVariables.ubValues[51] = 5.6407500000000005e+02;
acadoVariables.ubValues[52] = 5.6407500000000005e+02;
acadoVariables.ubValues[53] = 5.6407500000000005e+02;
acadoVariables.ubValues[54] = 5.6407500000000005e+02;
acadoVariables.ubValues[55] = 5.6407500000000005e+02;
acadoVariables.ubValues[56] = 5.6407500000000005e+02;
acadoVariables.ubValues[57] = 5.6407500000000005e+02;
acadoVariables.ubValues[58] = 5.6407500000000005e+02;
acadoVariables.ubValues[59] = 5.6407500000000005e+02;
acadoVariables.ubValues[60] = 5.6407500000000005e+02;
acadoVariables.ubValues[61] = 5.6407500000000005e+02;
acadoVariables.ubValues[62] = 5.6407500000000005e+02;
acadoVariables.ubValues[63] = 5.6407500000000005e+02;
acadoVariables.ubValues[64] = 5.6407500000000005e+02;
acadoVariables.ubValues[65] = 5.6407500000000005e+02;
acadoVariables.ubValues[66] = 5.6407500000000005e+02;
acadoVariables.ubValues[67] = 5.6407500000000005e+02;
acadoVariables.ubValues[68] = 5.6407500000000005e+02;
acadoVariables.ubValues[69] = 5.6407500000000005e+02;
acadoVariables.ubValues[70] = 5.6407500000000005e+02;
acadoVariables.ubValues[71] = 5.6407500000000005e+02;
acadoVariables.ubValues[72] = 5.6407500000000005e+02;
acadoVariables.ubValues[73] = 5.6407500000000005e+02;
acadoVariables.ubValues[74] = 5.6407500000000005e+02;
acadoVariables.ubValues[75] = 5.6407500000000005e+02;
acadoVariables.ubValues[76] = 5.6407500000000005e+02;
acadoVariables.ubValues[77] = 5.6407500000000005e+02;
acadoVariables.ubValues[78] = 5.6407500000000005e+02;
acadoVariables.ubValues[79] = 5.6407500000000005e+02;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 20; ++index)
{
state[0] = acadoVariables.x[index * 8];
state[1] = acadoVariables.x[index * 8 + 1];
state[2] = acadoVariables.x[index * 8 + 2];
state[3] = acadoVariables.x[index * 8 + 3];
state[4] = acadoVariables.x[index * 8 + 4];
state[5] = acadoVariables.x[index * 8 + 5];
state[6] = acadoVariables.x[index * 8 + 6];
state[7] = acadoVariables.x[index * 8 + 7];
state[104] = acadoVariables.u[index * 4];
state[105] = acadoVariables.u[index * 4 + 1];
state[106] = acadoVariables.u[index * 4 + 2];
state[107] = acadoVariables.u[index * 4 + 3];
state[108] = acadoVariables.od[index * 10];
state[109] = acadoVariables.od[index * 10 + 1];
state[110] = acadoVariables.od[index * 10 + 2];
state[111] = acadoVariables.od[index * 10 + 3];
state[112] = acadoVariables.od[index * 10 + 4];
state[113] = acadoVariables.od[index * 10 + 5];
state[114] = acadoVariables.od[index * 10 + 6];
state[115] = acadoVariables.od[index * 10 + 7];
state[116] = acadoVariables.od[index * 10 + 8];
state[117] = acadoVariables.od[index * 10 + 9];

acado_integrate(state, index == 0);

acadoVariables.x[index * 8 + 8] = state[0];
acadoVariables.x[index * 8 + 9] = state[1];
acadoVariables.x[index * 8 + 10] = state[2];
acadoVariables.x[index * 8 + 11] = state[3];
acadoVariables.x[index * 8 + 12] = state[4];
acadoVariables.x[index * 8 + 13] = state[5];
acadoVariables.x[index * 8 + 14] = state[6];
acadoVariables.x[index * 8 + 15] = state[7];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 20; ++index)
{
acadoVariables.x[index * 8] = acadoVariables.x[index * 8 + 8];
acadoVariables.x[index * 8 + 1] = acadoVariables.x[index * 8 + 9];
acadoVariables.x[index * 8 + 2] = acadoVariables.x[index * 8 + 10];
acadoVariables.x[index * 8 + 3] = acadoVariables.x[index * 8 + 11];
acadoVariables.x[index * 8 + 4] = acadoVariables.x[index * 8 + 12];
acadoVariables.x[index * 8 + 5] = acadoVariables.x[index * 8 + 13];
acadoVariables.x[index * 8 + 6] = acadoVariables.x[index * 8 + 14];
acadoVariables.x[index * 8 + 7] = acadoVariables.x[index * 8 + 15];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[160] = xEnd[0];
acadoVariables.x[161] = xEnd[1];
acadoVariables.x[162] = xEnd[2];
acadoVariables.x[163] = xEnd[3];
acadoVariables.x[164] = xEnd[4];
acadoVariables.x[165] = xEnd[5];
acadoVariables.x[166] = xEnd[6];
acadoVariables.x[167] = xEnd[7];
}
else if (strategy == 2) 
{
state[0] = acadoVariables.x[160];
state[1] = acadoVariables.x[161];
state[2] = acadoVariables.x[162];
state[3] = acadoVariables.x[163];
state[4] = acadoVariables.x[164];
state[5] = acadoVariables.x[165];
state[6] = acadoVariables.x[166];
state[7] = acadoVariables.x[167];
if (uEnd != 0)
{
state[104] = uEnd[0];
state[105] = uEnd[1];
state[106] = uEnd[2];
state[107] = uEnd[3];
}
else
{
state[104] = acadoVariables.u[76];
state[105] = acadoVariables.u[77];
state[106] = acadoVariables.u[78];
state[107] = acadoVariables.u[79];
}
state[108] = acadoVariables.od[200];
state[109] = acadoVariables.od[201];
state[110] = acadoVariables.od[202];
state[111] = acadoVariables.od[203];
state[112] = acadoVariables.od[204];
state[113] = acadoVariables.od[205];
state[114] = acadoVariables.od[206];
state[115] = acadoVariables.od[207];
state[116] = acadoVariables.od[208];
state[117] = acadoVariables.od[209];

acado_integrate(state, 1);

acadoVariables.x[160] = state[0];
acadoVariables.x[161] = state[1];
acadoVariables.x[162] = state[2];
acadoVariables.x[163] = state[3];
acadoVariables.x[164] = state[4];
acadoVariables.x[165] = state[5];
acadoVariables.x[166] = state[6];
acadoVariables.x[167] = state[7];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 19; ++index)
{
acadoVariables.u[index * 4] = acadoVariables.u[index * 4 + 4];
acadoVariables.u[index * 4 + 1] = acadoVariables.u[index * 4 + 5];
acadoVariables.u[index * 4 + 2] = acadoVariables.u[index * 4 + 6];
acadoVariables.u[index * 4 + 3] = acadoVariables.u[index * 4 + 7];
}

if (uEnd != 0)
{
acadoVariables.u[76] = uEnd[0];
acadoVariables.u[77] = uEnd[1];
acadoVariables.u[78] = uEnd[2];
acadoVariables.u[79] = uEnd[3];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59] + acadoWorkspace.g[60]*acadoWorkspace.x[60] + acadoWorkspace.g[61]*acadoWorkspace.x[61] + acadoWorkspace.g[62]*acadoWorkspace.x[62] + acadoWorkspace.g[63]*acadoWorkspace.x[63] + acadoWorkspace.g[64]*acadoWorkspace.x[64] + acadoWorkspace.g[65]*acadoWorkspace.x[65] + acadoWorkspace.g[66]*acadoWorkspace.x[66] + acadoWorkspace.g[67]*acadoWorkspace.x[67] + acadoWorkspace.g[68]*acadoWorkspace.x[68] + acadoWorkspace.g[69]*acadoWorkspace.x[69] + acadoWorkspace.g[70]*acadoWorkspace.x[70] + acadoWorkspace.g[71]*acadoWorkspace.x[71] + acadoWorkspace.g[72]*acadoWorkspace.x[72] + acadoWorkspace.g[73]*acadoWorkspace.x[73] + acadoWorkspace.g[74]*acadoWorkspace.x[74] + acadoWorkspace.g[75]*acadoWorkspace.x[75] + acadoWorkspace.g[76]*acadoWorkspace.x[76] + acadoWorkspace.g[77]*acadoWorkspace.x[77] + acadoWorkspace.g[78]*acadoWorkspace.x[78] + acadoWorkspace.g[79]*acadoWorkspace.x[79];
kkt = fabs( kkt );
for (index = 0; index < 80; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 12 */
real_t tmpDy[ 12 ];

/** Row vector of size: 8 */
real_t tmpDyN[ 8 ];

for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 8];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 8 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 8 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 8 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 8 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[lRun1 * 8 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[lRun1 * 8 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[lRun1 * 8 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.u[lRun1 * 4];
acadoWorkspace.objValueIn[9] = acadoVariables.u[lRun1 * 4 + 1];
acadoWorkspace.objValueIn[10] = acadoVariables.u[lRun1 * 4 + 2];
acadoWorkspace.objValueIn[11] = acadoVariables.u[lRun1 * 4 + 3];
acadoWorkspace.objValueIn[12] = acadoVariables.od[lRun1 * 10];
acadoWorkspace.objValueIn[13] = acadoVariables.od[lRun1 * 10 + 1];
acadoWorkspace.objValueIn[14] = acadoVariables.od[lRun1 * 10 + 2];
acadoWorkspace.objValueIn[15] = acadoVariables.od[lRun1 * 10 + 3];
acadoWorkspace.objValueIn[16] = acadoVariables.od[lRun1 * 10 + 4];
acadoWorkspace.objValueIn[17] = acadoVariables.od[lRun1 * 10 + 5];
acadoWorkspace.objValueIn[18] = acadoVariables.od[lRun1 * 10 + 6];
acadoWorkspace.objValueIn[19] = acadoVariables.od[lRun1 * 10 + 7];
acadoWorkspace.objValueIn[20] = acadoVariables.od[lRun1 * 10 + 8];
acadoWorkspace.objValueIn[21] = acadoVariables.od[lRun1 * 10 + 9];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 12] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 12];
acadoWorkspace.Dy[lRun1 * 12 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 12 + 1];
acadoWorkspace.Dy[lRun1 * 12 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 12 + 2];
acadoWorkspace.Dy[lRun1 * 12 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 12 + 3];
acadoWorkspace.Dy[lRun1 * 12 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 12 + 4];
acadoWorkspace.Dy[lRun1 * 12 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 12 + 5];
acadoWorkspace.Dy[lRun1 * 12 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 12 + 6];
acadoWorkspace.Dy[lRun1 * 12 + 7] = acadoWorkspace.objValueOut[7] - acadoVariables.y[lRun1 * 12 + 7];
acadoWorkspace.Dy[lRun1 * 12 + 8] = acadoWorkspace.objValueOut[8] - acadoVariables.y[lRun1 * 12 + 8];
acadoWorkspace.Dy[lRun1 * 12 + 9] = acadoWorkspace.objValueOut[9] - acadoVariables.y[lRun1 * 12 + 9];
acadoWorkspace.Dy[lRun1 * 12 + 10] = acadoWorkspace.objValueOut[10] - acadoVariables.y[lRun1 * 12 + 10];
acadoWorkspace.Dy[lRun1 * 12 + 11] = acadoWorkspace.objValueOut[11] - acadoVariables.y[lRun1 * 12 + 11];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[160];
acadoWorkspace.objValueIn[1] = acadoVariables.x[161];
acadoWorkspace.objValueIn[2] = acadoVariables.x[162];
acadoWorkspace.objValueIn[3] = acadoVariables.x[163];
acadoWorkspace.objValueIn[4] = acadoVariables.x[164];
acadoWorkspace.objValueIn[5] = acadoVariables.x[165];
acadoWorkspace.objValueIn[6] = acadoVariables.x[166];
acadoWorkspace.objValueIn[7] = acadoVariables.x[167];
acadoWorkspace.objValueIn[8] = acadoVariables.od[200];
acadoWorkspace.objValueIn[9] = acadoVariables.od[201];
acadoWorkspace.objValueIn[10] = acadoVariables.od[202];
acadoWorkspace.objValueIn[11] = acadoVariables.od[203];
acadoWorkspace.objValueIn[12] = acadoVariables.od[204];
acadoWorkspace.objValueIn[13] = acadoVariables.od[205];
acadoWorkspace.objValueIn[14] = acadoVariables.od[206];
acadoWorkspace.objValueIn[15] = acadoVariables.od[207];
acadoWorkspace.objValueIn[16] = acadoVariables.od[208];
acadoWorkspace.objValueIn[17] = acadoVariables.od[209];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4] - acadoVariables.yN[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5] - acadoVariables.yN[5];
acadoWorkspace.DyN[6] = acadoWorkspace.objValueOut[6] - acadoVariables.yN[6];
acadoWorkspace.DyN[7] = acadoWorkspace.objValueOut[7] - acadoVariables.yN[7];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 12]*acadoVariables.W[lRun1 * 144] + acadoWorkspace.Dy[lRun1 * 12 + 1]*acadoVariables.W[lRun1 * 144 + 12] + acadoWorkspace.Dy[lRun1 * 12 + 2]*acadoVariables.W[lRun1 * 144 + 24] + acadoWorkspace.Dy[lRun1 * 12 + 3]*acadoVariables.W[lRun1 * 144 + 36] + acadoWorkspace.Dy[lRun1 * 12 + 4]*acadoVariables.W[lRun1 * 144 + 48] + acadoWorkspace.Dy[lRun1 * 12 + 5]*acadoVariables.W[lRun1 * 144 + 60] + acadoWorkspace.Dy[lRun1 * 12 + 6]*acadoVariables.W[lRun1 * 144 + 72] + acadoWorkspace.Dy[lRun1 * 12 + 7]*acadoVariables.W[lRun1 * 144 + 84] + acadoWorkspace.Dy[lRun1 * 12 + 8]*acadoVariables.W[lRun1 * 144 + 96] + acadoWorkspace.Dy[lRun1 * 12 + 9]*acadoVariables.W[lRun1 * 144 + 108] + acadoWorkspace.Dy[lRun1 * 12 + 10]*acadoVariables.W[lRun1 * 144 + 120] + acadoWorkspace.Dy[lRun1 * 12 + 11]*acadoVariables.W[lRun1 * 144 + 132];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 12]*acadoVariables.W[lRun1 * 144 + 1] + acadoWorkspace.Dy[lRun1 * 12 + 1]*acadoVariables.W[lRun1 * 144 + 13] + acadoWorkspace.Dy[lRun1 * 12 + 2]*acadoVariables.W[lRun1 * 144 + 25] + acadoWorkspace.Dy[lRun1 * 12 + 3]*acadoVariables.W[lRun1 * 144 + 37] + acadoWorkspace.Dy[lRun1 * 12 + 4]*acadoVariables.W[lRun1 * 144 + 49] + acadoWorkspace.Dy[lRun1 * 12 + 5]*acadoVariables.W[lRun1 * 144 + 61] + acadoWorkspace.Dy[lRun1 * 12 + 6]*acadoVariables.W[lRun1 * 144 + 73] + acadoWorkspace.Dy[lRun1 * 12 + 7]*acadoVariables.W[lRun1 * 144 + 85] + acadoWorkspace.Dy[lRun1 * 12 + 8]*acadoVariables.W[lRun1 * 144 + 97] + acadoWorkspace.Dy[lRun1 * 12 + 9]*acadoVariables.W[lRun1 * 144 + 109] + acadoWorkspace.Dy[lRun1 * 12 + 10]*acadoVariables.W[lRun1 * 144 + 121] + acadoWorkspace.Dy[lRun1 * 12 + 11]*acadoVariables.W[lRun1 * 144 + 133];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 12]*acadoVariables.W[lRun1 * 144 + 2] + acadoWorkspace.Dy[lRun1 * 12 + 1]*acadoVariables.W[lRun1 * 144 + 14] + acadoWorkspace.Dy[lRun1 * 12 + 2]*acadoVariables.W[lRun1 * 144 + 26] + acadoWorkspace.Dy[lRun1 * 12 + 3]*acadoVariables.W[lRun1 * 144 + 38] + acadoWorkspace.Dy[lRun1 * 12 + 4]*acadoVariables.W[lRun1 * 144 + 50] + acadoWorkspace.Dy[lRun1 * 12 + 5]*acadoVariables.W[lRun1 * 144 + 62] + acadoWorkspace.Dy[lRun1 * 12 + 6]*acadoVariables.W[lRun1 * 144 + 74] + acadoWorkspace.Dy[lRun1 * 12 + 7]*acadoVariables.W[lRun1 * 144 + 86] + acadoWorkspace.Dy[lRun1 * 12 + 8]*acadoVariables.W[lRun1 * 144 + 98] + acadoWorkspace.Dy[lRun1 * 12 + 9]*acadoVariables.W[lRun1 * 144 + 110] + acadoWorkspace.Dy[lRun1 * 12 + 10]*acadoVariables.W[lRun1 * 144 + 122] + acadoWorkspace.Dy[lRun1 * 12 + 11]*acadoVariables.W[lRun1 * 144 + 134];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 12]*acadoVariables.W[lRun1 * 144 + 3] + acadoWorkspace.Dy[lRun1 * 12 + 1]*acadoVariables.W[lRun1 * 144 + 15] + acadoWorkspace.Dy[lRun1 * 12 + 2]*acadoVariables.W[lRun1 * 144 + 27] + acadoWorkspace.Dy[lRun1 * 12 + 3]*acadoVariables.W[lRun1 * 144 + 39] + acadoWorkspace.Dy[lRun1 * 12 + 4]*acadoVariables.W[lRun1 * 144 + 51] + acadoWorkspace.Dy[lRun1 * 12 + 5]*acadoVariables.W[lRun1 * 144 + 63] + acadoWorkspace.Dy[lRun1 * 12 + 6]*acadoVariables.W[lRun1 * 144 + 75] + acadoWorkspace.Dy[lRun1 * 12 + 7]*acadoVariables.W[lRun1 * 144 + 87] + acadoWorkspace.Dy[lRun1 * 12 + 8]*acadoVariables.W[lRun1 * 144 + 99] + acadoWorkspace.Dy[lRun1 * 12 + 9]*acadoVariables.W[lRun1 * 144 + 111] + acadoWorkspace.Dy[lRun1 * 12 + 10]*acadoVariables.W[lRun1 * 144 + 123] + acadoWorkspace.Dy[lRun1 * 12 + 11]*acadoVariables.W[lRun1 * 144 + 135];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 12]*acadoVariables.W[lRun1 * 144 + 4] + acadoWorkspace.Dy[lRun1 * 12 + 1]*acadoVariables.W[lRun1 * 144 + 16] + acadoWorkspace.Dy[lRun1 * 12 + 2]*acadoVariables.W[lRun1 * 144 + 28] + acadoWorkspace.Dy[lRun1 * 12 + 3]*acadoVariables.W[lRun1 * 144 + 40] + acadoWorkspace.Dy[lRun1 * 12 + 4]*acadoVariables.W[lRun1 * 144 + 52] + acadoWorkspace.Dy[lRun1 * 12 + 5]*acadoVariables.W[lRun1 * 144 + 64] + acadoWorkspace.Dy[lRun1 * 12 + 6]*acadoVariables.W[lRun1 * 144 + 76] + acadoWorkspace.Dy[lRun1 * 12 + 7]*acadoVariables.W[lRun1 * 144 + 88] + acadoWorkspace.Dy[lRun1 * 12 + 8]*acadoVariables.W[lRun1 * 144 + 100] + acadoWorkspace.Dy[lRun1 * 12 + 9]*acadoVariables.W[lRun1 * 144 + 112] + acadoWorkspace.Dy[lRun1 * 12 + 10]*acadoVariables.W[lRun1 * 144 + 124] + acadoWorkspace.Dy[lRun1 * 12 + 11]*acadoVariables.W[lRun1 * 144 + 136];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 12]*acadoVariables.W[lRun1 * 144 + 5] + acadoWorkspace.Dy[lRun1 * 12 + 1]*acadoVariables.W[lRun1 * 144 + 17] + acadoWorkspace.Dy[lRun1 * 12 + 2]*acadoVariables.W[lRun1 * 144 + 29] + acadoWorkspace.Dy[lRun1 * 12 + 3]*acadoVariables.W[lRun1 * 144 + 41] + acadoWorkspace.Dy[lRun1 * 12 + 4]*acadoVariables.W[lRun1 * 144 + 53] + acadoWorkspace.Dy[lRun1 * 12 + 5]*acadoVariables.W[lRun1 * 144 + 65] + acadoWorkspace.Dy[lRun1 * 12 + 6]*acadoVariables.W[lRun1 * 144 + 77] + acadoWorkspace.Dy[lRun1 * 12 + 7]*acadoVariables.W[lRun1 * 144 + 89] + acadoWorkspace.Dy[lRun1 * 12 + 8]*acadoVariables.W[lRun1 * 144 + 101] + acadoWorkspace.Dy[lRun1 * 12 + 9]*acadoVariables.W[lRun1 * 144 + 113] + acadoWorkspace.Dy[lRun1 * 12 + 10]*acadoVariables.W[lRun1 * 144 + 125] + acadoWorkspace.Dy[lRun1 * 12 + 11]*acadoVariables.W[lRun1 * 144 + 137];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 12]*acadoVariables.W[lRun1 * 144 + 6] + acadoWorkspace.Dy[lRun1 * 12 + 1]*acadoVariables.W[lRun1 * 144 + 18] + acadoWorkspace.Dy[lRun1 * 12 + 2]*acadoVariables.W[lRun1 * 144 + 30] + acadoWorkspace.Dy[lRun1 * 12 + 3]*acadoVariables.W[lRun1 * 144 + 42] + acadoWorkspace.Dy[lRun1 * 12 + 4]*acadoVariables.W[lRun1 * 144 + 54] + acadoWorkspace.Dy[lRun1 * 12 + 5]*acadoVariables.W[lRun1 * 144 + 66] + acadoWorkspace.Dy[lRun1 * 12 + 6]*acadoVariables.W[lRun1 * 144 + 78] + acadoWorkspace.Dy[lRun1 * 12 + 7]*acadoVariables.W[lRun1 * 144 + 90] + acadoWorkspace.Dy[lRun1 * 12 + 8]*acadoVariables.W[lRun1 * 144 + 102] + acadoWorkspace.Dy[lRun1 * 12 + 9]*acadoVariables.W[lRun1 * 144 + 114] + acadoWorkspace.Dy[lRun1 * 12 + 10]*acadoVariables.W[lRun1 * 144 + 126] + acadoWorkspace.Dy[lRun1 * 12 + 11]*acadoVariables.W[lRun1 * 144 + 138];
tmpDy[7] = + acadoWorkspace.Dy[lRun1 * 12]*acadoVariables.W[lRun1 * 144 + 7] + acadoWorkspace.Dy[lRun1 * 12 + 1]*acadoVariables.W[lRun1 * 144 + 19] + acadoWorkspace.Dy[lRun1 * 12 + 2]*acadoVariables.W[lRun1 * 144 + 31] + acadoWorkspace.Dy[lRun1 * 12 + 3]*acadoVariables.W[lRun1 * 144 + 43] + acadoWorkspace.Dy[lRun1 * 12 + 4]*acadoVariables.W[lRun1 * 144 + 55] + acadoWorkspace.Dy[lRun1 * 12 + 5]*acadoVariables.W[lRun1 * 144 + 67] + acadoWorkspace.Dy[lRun1 * 12 + 6]*acadoVariables.W[lRun1 * 144 + 79] + acadoWorkspace.Dy[lRun1 * 12 + 7]*acadoVariables.W[lRun1 * 144 + 91] + acadoWorkspace.Dy[lRun1 * 12 + 8]*acadoVariables.W[lRun1 * 144 + 103] + acadoWorkspace.Dy[lRun1 * 12 + 9]*acadoVariables.W[lRun1 * 144 + 115] + acadoWorkspace.Dy[lRun1 * 12 + 10]*acadoVariables.W[lRun1 * 144 + 127] + acadoWorkspace.Dy[lRun1 * 12 + 11]*acadoVariables.W[lRun1 * 144 + 139];
tmpDy[8] = + acadoWorkspace.Dy[lRun1 * 12]*acadoVariables.W[lRun1 * 144 + 8] + acadoWorkspace.Dy[lRun1 * 12 + 1]*acadoVariables.W[lRun1 * 144 + 20] + acadoWorkspace.Dy[lRun1 * 12 + 2]*acadoVariables.W[lRun1 * 144 + 32] + acadoWorkspace.Dy[lRun1 * 12 + 3]*acadoVariables.W[lRun1 * 144 + 44] + acadoWorkspace.Dy[lRun1 * 12 + 4]*acadoVariables.W[lRun1 * 144 + 56] + acadoWorkspace.Dy[lRun1 * 12 + 5]*acadoVariables.W[lRun1 * 144 + 68] + acadoWorkspace.Dy[lRun1 * 12 + 6]*acadoVariables.W[lRun1 * 144 + 80] + acadoWorkspace.Dy[lRun1 * 12 + 7]*acadoVariables.W[lRun1 * 144 + 92] + acadoWorkspace.Dy[lRun1 * 12 + 8]*acadoVariables.W[lRun1 * 144 + 104] + acadoWorkspace.Dy[lRun1 * 12 + 9]*acadoVariables.W[lRun1 * 144 + 116] + acadoWorkspace.Dy[lRun1 * 12 + 10]*acadoVariables.W[lRun1 * 144 + 128] + acadoWorkspace.Dy[lRun1 * 12 + 11]*acadoVariables.W[lRun1 * 144 + 140];
tmpDy[9] = + acadoWorkspace.Dy[lRun1 * 12]*acadoVariables.W[lRun1 * 144 + 9] + acadoWorkspace.Dy[lRun1 * 12 + 1]*acadoVariables.W[lRun1 * 144 + 21] + acadoWorkspace.Dy[lRun1 * 12 + 2]*acadoVariables.W[lRun1 * 144 + 33] + acadoWorkspace.Dy[lRun1 * 12 + 3]*acadoVariables.W[lRun1 * 144 + 45] + acadoWorkspace.Dy[lRun1 * 12 + 4]*acadoVariables.W[lRun1 * 144 + 57] + acadoWorkspace.Dy[lRun1 * 12 + 5]*acadoVariables.W[lRun1 * 144 + 69] + acadoWorkspace.Dy[lRun1 * 12 + 6]*acadoVariables.W[lRun1 * 144 + 81] + acadoWorkspace.Dy[lRun1 * 12 + 7]*acadoVariables.W[lRun1 * 144 + 93] + acadoWorkspace.Dy[lRun1 * 12 + 8]*acadoVariables.W[lRun1 * 144 + 105] + acadoWorkspace.Dy[lRun1 * 12 + 9]*acadoVariables.W[lRun1 * 144 + 117] + acadoWorkspace.Dy[lRun1 * 12 + 10]*acadoVariables.W[lRun1 * 144 + 129] + acadoWorkspace.Dy[lRun1 * 12 + 11]*acadoVariables.W[lRun1 * 144 + 141];
tmpDy[10] = + acadoWorkspace.Dy[lRun1 * 12]*acadoVariables.W[lRun1 * 144 + 10] + acadoWorkspace.Dy[lRun1 * 12 + 1]*acadoVariables.W[lRun1 * 144 + 22] + acadoWorkspace.Dy[lRun1 * 12 + 2]*acadoVariables.W[lRun1 * 144 + 34] + acadoWorkspace.Dy[lRun1 * 12 + 3]*acadoVariables.W[lRun1 * 144 + 46] + acadoWorkspace.Dy[lRun1 * 12 + 4]*acadoVariables.W[lRun1 * 144 + 58] + acadoWorkspace.Dy[lRun1 * 12 + 5]*acadoVariables.W[lRun1 * 144 + 70] + acadoWorkspace.Dy[lRun1 * 12 + 6]*acadoVariables.W[lRun1 * 144 + 82] + acadoWorkspace.Dy[lRun1 * 12 + 7]*acadoVariables.W[lRun1 * 144 + 94] + acadoWorkspace.Dy[lRun1 * 12 + 8]*acadoVariables.W[lRun1 * 144 + 106] + acadoWorkspace.Dy[lRun1 * 12 + 9]*acadoVariables.W[lRun1 * 144 + 118] + acadoWorkspace.Dy[lRun1 * 12 + 10]*acadoVariables.W[lRun1 * 144 + 130] + acadoWorkspace.Dy[lRun1 * 12 + 11]*acadoVariables.W[lRun1 * 144 + 142];
tmpDy[11] = + acadoWorkspace.Dy[lRun1 * 12]*acadoVariables.W[lRun1 * 144 + 11] + acadoWorkspace.Dy[lRun1 * 12 + 1]*acadoVariables.W[lRun1 * 144 + 23] + acadoWorkspace.Dy[lRun1 * 12 + 2]*acadoVariables.W[lRun1 * 144 + 35] + acadoWorkspace.Dy[lRun1 * 12 + 3]*acadoVariables.W[lRun1 * 144 + 47] + acadoWorkspace.Dy[lRun1 * 12 + 4]*acadoVariables.W[lRun1 * 144 + 59] + acadoWorkspace.Dy[lRun1 * 12 + 5]*acadoVariables.W[lRun1 * 144 + 71] + acadoWorkspace.Dy[lRun1 * 12 + 6]*acadoVariables.W[lRun1 * 144 + 83] + acadoWorkspace.Dy[lRun1 * 12 + 7]*acadoVariables.W[lRun1 * 144 + 95] + acadoWorkspace.Dy[lRun1 * 12 + 8]*acadoVariables.W[lRun1 * 144 + 107] + acadoWorkspace.Dy[lRun1 * 12 + 9]*acadoVariables.W[lRun1 * 144 + 119] + acadoWorkspace.Dy[lRun1 * 12 + 10]*acadoVariables.W[lRun1 * 144 + 131] + acadoWorkspace.Dy[lRun1 * 12 + 11]*acadoVariables.W[lRun1 * 144 + 143];
objVal += + acadoWorkspace.Dy[lRun1 * 12]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 12 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 12 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 12 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 12 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 12 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 12 + 6]*tmpDy[6] + acadoWorkspace.Dy[lRun1 * 12 + 7]*tmpDy[7] + acadoWorkspace.Dy[lRun1 * 12 + 8]*tmpDy[8] + acadoWorkspace.Dy[lRun1 * 12 + 9]*tmpDy[9] + acadoWorkspace.Dy[lRun1 * 12 + 10]*tmpDy[10] + acadoWorkspace.Dy[lRun1 * 12 + 11]*tmpDy[11];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[9];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[18];
tmpDyN[3] = + acadoWorkspace.DyN[3]*acadoVariables.WN[27];
tmpDyN[4] = + acadoWorkspace.DyN[4]*acadoVariables.WN[36];
tmpDyN[5] = + acadoWorkspace.DyN[5]*acadoVariables.WN[45];
tmpDyN[6] = + acadoWorkspace.DyN[6]*acadoVariables.WN[54];
tmpDyN[7] = + acadoWorkspace.DyN[7]*acadoVariables.WN[63];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3] + acadoWorkspace.DyN[4]*tmpDyN[4] + acadoWorkspace.DyN[5]*tmpDyN[5] + acadoWorkspace.DyN[6]*tmpDyN[6] + acadoWorkspace.DyN[7]*tmpDyN[7];

objVal *= 0.5;
return objVal;
}

