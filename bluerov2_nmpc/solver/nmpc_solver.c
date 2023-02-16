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


#include "nmpc_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int nmpc_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpcWorkspace.state[0] = nmpcVariables.x[lRun1 * 8];
nmpcWorkspace.state[1] = nmpcVariables.x[lRun1 * 8 + 1];
nmpcWorkspace.state[2] = nmpcVariables.x[lRun1 * 8 + 2];
nmpcWorkspace.state[3] = nmpcVariables.x[lRun1 * 8 + 3];
nmpcWorkspace.state[4] = nmpcVariables.x[lRun1 * 8 + 4];
nmpcWorkspace.state[5] = nmpcVariables.x[lRun1 * 8 + 5];
nmpcWorkspace.state[6] = nmpcVariables.x[lRun1 * 8 + 6];
nmpcWorkspace.state[7] = nmpcVariables.x[lRun1 * 8 + 7];

nmpcWorkspace.state[104] = nmpcVariables.u[lRun1 * 4];
nmpcWorkspace.state[105] = nmpcVariables.u[lRun1 * 4 + 1];
nmpcWorkspace.state[106] = nmpcVariables.u[lRun1 * 4 + 2];
nmpcWorkspace.state[107] = nmpcVariables.u[lRun1 * 4 + 3];
nmpcWorkspace.state[108] = nmpcVariables.od[lRun1 * 3];
nmpcWorkspace.state[109] = nmpcVariables.od[lRun1 * 3 + 1];
nmpcWorkspace.state[110] = nmpcVariables.od[lRun1 * 3 + 2];

ret = nmpc_integrate(nmpcWorkspace.state, 1);

nmpcWorkspace.d[lRun1 * 8] = nmpcWorkspace.state[0] - nmpcVariables.x[lRun1 * 8 + 8];
nmpcWorkspace.d[lRun1 * 8 + 1] = nmpcWorkspace.state[1] - nmpcVariables.x[lRun1 * 8 + 9];
nmpcWorkspace.d[lRun1 * 8 + 2] = nmpcWorkspace.state[2] - nmpcVariables.x[lRun1 * 8 + 10];
nmpcWorkspace.d[lRun1 * 8 + 3] = nmpcWorkspace.state[3] - nmpcVariables.x[lRun1 * 8 + 11];
nmpcWorkspace.d[lRun1 * 8 + 4] = nmpcWorkspace.state[4] - nmpcVariables.x[lRun1 * 8 + 12];
nmpcWorkspace.d[lRun1 * 8 + 5] = nmpcWorkspace.state[5] - nmpcVariables.x[lRun1 * 8 + 13];
nmpcWorkspace.d[lRun1 * 8 + 6] = nmpcWorkspace.state[6] - nmpcVariables.x[lRun1 * 8 + 14];
nmpcWorkspace.d[lRun1 * 8 + 7] = nmpcWorkspace.state[7] - nmpcVariables.x[lRun1 * 8 + 15];

nmpcWorkspace.evGx[lRun1 * 64] = nmpcWorkspace.state[8];
nmpcWorkspace.evGx[lRun1 * 64 + 1] = nmpcWorkspace.state[9];
nmpcWorkspace.evGx[lRun1 * 64 + 2] = nmpcWorkspace.state[10];
nmpcWorkspace.evGx[lRun1 * 64 + 3] = nmpcWorkspace.state[11];
nmpcWorkspace.evGx[lRun1 * 64 + 4] = nmpcWorkspace.state[12];
nmpcWorkspace.evGx[lRun1 * 64 + 5] = nmpcWorkspace.state[13];
nmpcWorkspace.evGx[lRun1 * 64 + 6] = nmpcWorkspace.state[14];
nmpcWorkspace.evGx[lRun1 * 64 + 7] = nmpcWorkspace.state[15];
nmpcWorkspace.evGx[lRun1 * 64 + 8] = nmpcWorkspace.state[16];
nmpcWorkspace.evGx[lRun1 * 64 + 9] = nmpcWorkspace.state[17];
nmpcWorkspace.evGx[lRun1 * 64 + 10] = nmpcWorkspace.state[18];
nmpcWorkspace.evGx[lRun1 * 64 + 11] = nmpcWorkspace.state[19];
nmpcWorkspace.evGx[lRun1 * 64 + 12] = nmpcWorkspace.state[20];
nmpcWorkspace.evGx[lRun1 * 64 + 13] = nmpcWorkspace.state[21];
nmpcWorkspace.evGx[lRun1 * 64 + 14] = nmpcWorkspace.state[22];
nmpcWorkspace.evGx[lRun1 * 64 + 15] = nmpcWorkspace.state[23];
nmpcWorkspace.evGx[lRun1 * 64 + 16] = nmpcWorkspace.state[24];
nmpcWorkspace.evGx[lRun1 * 64 + 17] = nmpcWorkspace.state[25];
nmpcWorkspace.evGx[lRun1 * 64 + 18] = nmpcWorkspace.state[26];
nmpcWorkspace.evGx[lRun1 * 64 + 19] = nmpcWorkspace.state[27];
nmpcWorkspace.evGx[lRun1 * 64 + 20] = nmpcWorkspace.state[28];
nmpcWorkspace.evGx[lRun1 * 64 + 21] = nmpcWorkspace.state[29];
nmpcWorkspace.evGx[lRun1 * 64 + 22] = nmpcWorkspace.state[30];
nmpcWorkspace.evGx[lRun1 * 64 + 23] = nmpcWorkspace.state[31];
nmpcWorkspace.evGx[lRun1 * 64 + 24] = nmpcWorkspace.state[32];
nmpcWorkspace.evGx[lRun1 * 64 + 25] = nmpcWorkspace.state[33];
nmpcWorkspace.evGx[lRun1 * 64 + 26] = nmpcWorkspace.state[34];
nmpcWorkspace.evGx[lRun1 * 64 + 27] = nmpcWorkspace.state[35];
nmpcWorkspace.evGx[lRun1 * 64 + 28] = nmpcWorkspace.state[36];
nmpcWorkspace.evGx[lRun1 * 64 + 29] = nmpcWorkspace.state[37];
nmpcWorkspace.evGx[lRun1 * 64 + 30] = nmpcWorkspace.state[38];
nmpcWorkspace.evGx[lRun1 * 64 + 31] = nmpcWorkspace.state[39];
nmpcWorkspace.evGx[lRun1 * 64 + 32] = nmpcWorkspace.state[40];
nmpcWorkspace.evGx[lRun1 * 64 + 33] = nmpcWorkspace.state[41];
nmpcWorkspace.evGx[lRun1 * 64 + 34] = nmpcWorkspace.state[42];
nmpcWorkspace.evGx[lRun1 * 64 + 35] = nmpcWorkspace.state[43];
nmpcWorkspace.evGx[lRun1 * 64 + 36] = nmpcWorkspace.state[44];
nmpcWorkspace.evGx[lRun1 * 64 + 37] = nmpcWorkspace.state[45];
nmpcWorkspace.evGx[lRun1 * 64 + 38] = nmpcWorkspace.state[46];
nmpcWorkspace.evGx[lRun1 * 64 + 39] = nmpcWorkspace.state[47];
nmpcWorkspace.evGx[lRun1 * 64 + 40] = nmpcWorkspace.state[48];
nmpcWorkspace.evGx[lRun1 * 64 + 41] = nmpcWorkspace.state[49];
nmpcWorkspace.evGx[lRun1 * 64 + 42] = nmpcWorkspace.state[50];
nmpcWorkspace.evGx[lRun1 * 64 + 43] = nmpcWorkspace.state[51];
nmpcWorkspace.evGx[lRun1 * 64 + 44] = nmpcWorkspace.state[52];
nmpcWorkspace.evGx[lRun1 * 64 + 45] = nmpcWorkspace.state[53];
nmpcWorkspace.evGx[lRun1 * 64 + 46] = nmpcWorkspace.state[54];
nmpcWorkspace.evGx[lRun1 * 64 + 47] = nmpcWorkspace.state[55];
nmpcWorkspace.evGx[lRun1 * 64 + 48] = nmpcWorkspace.state[56];
nmpcWorkspace.evGx[lRun1 * 64 + 49] = nmpcWorkspace.state[57];
nmpcWorkspace.evGx[lRun1 * 64 + 50] = nmpcWorkspace.state[58];
nmpcWorkspace.evGx[lRun1 * 64 + 51] = nmpcWorkspace.state[59];
nmpcWorkspace.evGx[lRun1 * 64 + 52] = nmpcWorkspace.state[60];
nmpcWorkspace.evGx[lRun1 * 64 + 53] = nmpcWorkspace.state[61];
nmpcWorkspace.evGx[lRun1 * 64 + 54] = nmpcWorkspace.state[62];
nmpcWorkspace.evGx[lRun1 * 64 + 55] = nmpcWorkspace.state[63];
nmpcWorkspace.evGx[lRun1 * 64 + 56] = nmpcWorkspace.state[64];
nmpcWorkspace.evGx[lRun1 * 64 + 57] = nmpcWorkspace.state[65];
nmpcWorkspace.evGx[lRun1 * 64 + 58] = nmpcWorkspace.state[66];
nmpcWorkspace.evGx[lRun1 * 64 + 59] = nmpcWorkspace.state[67];
nmpcWorkspace.evGx[lRun1 * 64 + 60] = nmpcWorkspace.state[68];
nmpcWorkspace.evGx[lRun1 * 64 + 61] = nmpcWorkspace.state[69];
nmpcWorkspace.evGx[lRun1 * 64 + 62] = nmpcWorkspace.state[70];
nmpcWorkspace.evGx[lRun1 * 64 + 63] = nmpcWorkspace.state[71];

nmpcWorkspace.evGu[lRun1 * 32] = nmpcWorkspace.state[72];
nmpcWorkspace.evGu[lRun1 * 32 + 1] = nmpcWorkspace.state[73];
nmpcWorkspace.evGu[lRun1 * 32 + 2] = nmpcWorkspace.state[74];
nmpcWorkspace.evGu[lRun1 * 32 + 3] = nmpcWorkspace.state[75];
nmpcWorkspace.evGu[lRun1 * 32 + 4] = nmpcWorkspace.state[76];
nmpcWorkspace.evGu[lRun1 * 32 + 5] = nmpcWorkspace.state[77];
nmpcWorkspace.evGu[lRun1 * 32 + 6] = nmpcWorkspace.state[78];
nmpcWorkspace.evGu[lRun1 * 32 + 7] = nmpcWorkspace.state[79];
nmpcWorkspace.evGu[lRun1 * 32 + 8] = nmpcWorkspace.state[80];
nmpcWorkspace.evGu[lRun1 * 32 + 9] = nmpcWorkspace.state[81];
nmpcWorkspace.evGu[lRun1 * 32 + 10] = nmpcWorkspace.state[82];
nmpcWorkspace.evGu[lRun1 * 32 + 11] = nmpcWorkspace.state[83];
nmpcWorkspace.evGu[lRun1 * 32 + 12] = nmpcWorkspace.state[84];
nmpcWorkspace.evGu[lRun1 * 32 + 13] = nmpcWorkspace.state[85];
nmpcWorkspace.evGu[lRun1 * 32 + 14] = nmpcWorkspace.state[86];
nmpcWorkspace.evGu[lRun1 * 32 + 15] = nmpcWorkspace.state[87];
nmpcWorkspace.evGu[lRun1 * 32 + 16] = nmpcWorkspace.state[88];
nmpcWorkspace.evGu[lRun1 * 32 + 17] = nmpcWorkspace.state[89];
nmpcWorkspace.evGu[lRun1 * 32 + 18] = nmpcWorkspace.state[90];
nmpcWorkspace.evGu[lRun1 * 32 + 19] = nmpcWorkspace.state[91];
nmpcWorkspace.evGu[lRun1 * 32 + 20] = nmpcWorkspace.state[92];
nmpcWorkspace.evGu[lRun1 * 32 + 21] = nmpcWorkspace.state[93];
nmpcWorkspace.evGu[lRun1 * 32 + 22] = nmpcWorkspace.state[94];
nmpcWorkspace.evGu[lRun1 * 32 + 23] = nmpcWorkspace.state[95];
nmpcWorkspace.evGu[lRun1 * 32 + 24] = nmpcWorkspace.state[96];
nmpcWorkspace.evGu[lRun1 * 32 + 25] = nmpcWorkspace.state[97];
nmpcWorkspace.evGu[lRun1 * 32 + 26] = nmpcWorkspace.state[98];
nmpcWorkspace.evGu[lRun1 * 32 + 27] = nmpcWorkspace.state[99];
nmpcWorkspace.evGu[lRun1 * 32 + 28] = nmpcWorkspace.state[100];
nmpcWorkspace.evGu[lRun1 * 32 + 29] = nmpcWorkspace.state[101];
nmpcWorkspace.evGu[lRun1 * 32 + 30] = nmpcWorkspace.state[102];
nmpcWorkspace.evGu[lRun1 * 32 + 31] = nmpcWorkspace.state[103];
}
return ret;
}

void nmpc_evaluateLSQ(const real_t* in, real_t* out)
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

void nmpc_evaluateLSQEndTerm(const real_t* in, real_t* out)
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

void nmpc_setObjQ1Q2( real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
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

void nmpc_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
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

void nmpc_setObjQN1QN2( real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
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

void nmpc_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 30; ++runObj)
{
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[runObj * 8];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[runObj * 8 + 1];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[runObj * 8 + 2];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[runObj * 8 + 3];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[runObj * 8 + 4];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[runObj * 8 + 5];
nmpcWorkspace.objValueIn[6] = nmpcVariables.x[runObj * 8 + 6];
nmpcWorkspace.objValueIn[7] = nmpcVariables.x[runObj * 8 + 7];
nmpcWorkspace.objValueIn[8] = nmpcVariables.u[runObj * 4];
nmpcWorkspace.objValueIn[9] = nmpcVariables.u[runObj * 4 + 1];
nmpcWorkspace.objValueIn[10] = nmpcVariables.u[runObj * 4 + 2];
nmpcWorkspace.objValueIn[11] = nmpcVariables.u[runObj * 4 + 3];
nmpcWorkspace.objValueIn[12] = nmpcVariables.od[runObj * 3];
nmpcWorkspace.objValueIn[13] = nmpcVariables.od[runObj * 3 + 1];
nmpcWorkspace.objValueIn[14] = nmpcVariables.od[runObj * 3 + 2];

nmpc_evaluateLSQ( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );
nmpcWorkspace.Dy[runObj * 12] = nmpcWorkspace.objValueOut[0];
nmpcWorkspace.Dy[runObj * 12 + 1] = nmpcWorkspace.objValueOut[1];
nmpcWorkspace.Dy[runObj * 12 + 2] = nmpcWorkspace.objValueOut[2];
nmpcWorkspace.Dy[runObj * 12 + 3] = nmpcWorkspace.objValueOut[3];
nmpcWorkspace.Dy[runObj * 12 + 4] = nmpcWorkspace.objValueOut[4];
nmpcWorkspace.Dy[runObj * 12 + 5] = nmpcWorkspace.objValueOut[5];
nmpcWorkspace.Dy[runObj * 12 + 6] = nmpcWorkspace.objValueOut[6];
nmpcWorkspace.Dy[runObj * 12 + 7] = nmpcWorkspace.objValueOut[7];
nmpcWorkspace.Dy[runObj * 12 + 8] = nmpcWorkspace.objValueOut[8];
nmpcWorkspace.Dy[runObj * 12 + 9] = nmpcWorkspace.objValueOut[9];
nmpcWorkspace.Dy[runObj * 12 + 10] = nmpcWorkspace.objValueOut[10];
nmpcWorkspace.Dy[runObj * 12 + 11] = nmpcWorkspace.objValueOut[11];

nmpc_setObjQ1Q2( nmpcVariables.W, &(nmpcWorkspace.Q1[ runObj * 64 ]), &(nmpcWorkspace.Q2[ runObj * 96 ]) );

nmpc_setObjR1R2( nmpcVariables.W, &(nmpcWorkspace.R1[ runObj * 16 ]), &(nmpcWorkspace.R2[ runObj * 48 ]) );

}
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[240];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[241];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[242];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[243];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[244];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[245];
nmpcWorkspace.objValueIn[6] = nmpcVariables.x[246];
nmpcWorkspace.objValueIn[7] = nmpcVariables.x[247];
nmpcWorkspace.objValueIn[8] = nmpcVariables.od[90];
nmpcWorkspace.objValueIn[9] = nmpcVariables.od[91];
nmpcWorkspace.objValueIn[10] = nmpcVariables.od[92];
nmpc_evaluateLSQEndTerm( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );

nmpcWorkspace.DyN[0] = nmpcWorkspace.objValueOut[0];
nmpcWorkspace.DyN[1] = nmpcWorkspace.objValueOut[1];
nmpcWorkspace.DyN[2] = nmpcWorkspace.objValueOut[2];
nmpcWorkspace.DyN[3] = nmpcWorkspace.objValueOut[3];
nmpcWorkspace.DyN[4] = nmpcWorkspace.objValueOut[4];
nmpcWorkspace.DyN[5] = nmpcWorkspace.objValueOut[5];
nmpcWorkspace.DyN[6] = nmpcWorkspace.objValueOut[6];
nmpcWorkspace.DyN[7] = nmpcWorkspace.objValueOut[7];

nmpc_setObjQN1QN2( nmpcVariables.WN, nmpcWorkspace.QN1, nmpcWorkspace.QN2 );

}

void nmpc_multGxd( real_t* const dOld, real_t* const Gx1, real_t* const dNew )
{
dNew[0] += + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5] + Gx1[6]*dOld[6] + Gx1[7]*dOld[7];
dNew[1] += + Gx1[8]*dOld[0] + Gx1[9]*dOld[1] + Gx1[10]*dOld[2] + Gx1[11]*dOld[3] + Gx1[12]*dOld[4] + Gx1[13]*dOld[5] + Gx1[14]*dOld[6] + Gx1[15]*dOld[7];
dNew[2] += + Gx1[16]*dOld[0] + Gx1[17]*dOld[1] + Gx1[18]*dOld[2] + Gx1[19]*dOld[3] + Gx1[20]*dOld[4] + Gx1[21]*dOld[5] + Gx1[22]*dOld[6] + Gx1[23]*dOld[7];
dNew[3] += + Gx1[24]*dOld[0] + Gx1[25]*dOld[1] + Gx1[26]*dOld[2] + Gx1[27]*dOld[3] + Gx1[28]*dOld[4] + Gx1[29]*dOld[5] + Gx1[30]*dOld[6] + Gx1[31]*dOld[7];
dNew[4] += + Gx1[32]*dOld[0] + Gx1[33]*dOld[1] + Gx1[34]*dOld[2] + Gx1[35]*dOld[3] + Gx1[36]*dOld[4] + Gx1[37]*dOld[5] + Gx1[38]*dOld[6] + Gx1[39]*dOld[7];
dNew[5] += + Gx1[40]*dOld[0] + Gx1[41]*dOld[1] + Gx1[42]*dOld[2] + Gx1[43]*dOld[3] + Gx1[44]*dOld[4] + Gx1[45]*dOld[5] + Gx1[46]*dOld[6] + Gx1[47]*dOld[7];
dNew[6] += + Gx1[48]*dOld[0] + Gx1[49]*dOld[1] + Gx1[50]*dOld[2] + Gx1[51]*dOld[3] + Gx1[52]*dOld[4] + Gx1[53]*dOld[5] + Gx1[54]*dOld[6] + Gx1[55]*dOld[7];
dNew[7] += + Gx1[56]*dOld[0] + Gx1[57]*dOld[1] + Gx1[58]*dOld[2] + Gx1[59]*dOld[3] + Gx1[60]*dOld[4] + Gx1[61]*dOld[5] + Gx1[62]*dOld[6] + Gx1[63]*dOld[7];
}

void nmpc_moveGxT( real_t* const Gx1, real_t* const Gx2 )
{
Gx2[0] = Gx1[0];
Gx2[1] = Gx1[1];
Gx2[2] = Gx1[2];
Gx2[3] = Gx1[3];
Gx2[4] = Gx1[4];
Gx2[5] = Gx1[5];
Gx2[6] = Gx1[6];
Gx2[7] = Gx1[7];
Gx2[8] = Gx1[8];
Gx2[9] = Gx1[9];
Gx2[10] = Gx1[10];
Gx2[11] = Gx1[11];
Gx2[12] = Gx1[12];
Gx2[13] = Gx1[13];
Gx2[14] = Gx1[14];
Gx2[15] = Gx1[15];
Gx2[16] = Gx1[16];
Gx2[17] = Gx1[17];
Gx2[18] = Gx1[18];
Gx2[19] = Gx1[19];
Gx2[20] = Gx1[20];
Gx2[21] = Gx1[21];
Gx2[22] = Gx1[22];
Gx2[23] = Gx1[23];
Gx2[24] = Gx1[24];
Gx2[25] = Gx1[25];
Gx2[26] = Gx1[26];
Gx2[27] = Gx1[27];
Gx2[28] = Gx1[28];
Gx2[29] = Gx1[29];
Gx2[30] = Gx1[30];
Gx2[31] = Gx1[31];
Gx2[32] = Gx1[32];
Gx2[33] = Gx1[33];
Gx2[34] = Gx1[34];
Gx2[35] = Gx1[35];
Gx2[36] = Gx1[36];
Gx2[37] = Gx1[37];
Gx2[38] = Gx1[38];
Gx2[39] = Gx1[39];
Gx2[40] = Gx1[40];
Gx2[41] = Gx1[41];
Gx2[42] = Gx1[42];
Gx2[43] = Gx1[43];
Gx2[44] = Gx1[44];
Gx2[45] = Gx1[45];
Gx2[46] = Gx1[46];
Gx2[47] = Gx1[47];
Gx2[48] = Gx1[48];
Gx2[49] = Gx1[49];
Gx2[50] = Gx1[50];
Gx2[51] = Gx1[51];
Gx2[52] = Gx1[52];
Gx2[53] = Gx1[53];
Gx2[54] = Gx1[54];
Gx2[55] = Gx1[55];
Gx2[56] = Gx1[56];
Gx2[57] = Gx1[57];
Gx2[58] = Gx1[58];
Gx2[59] = Gx1[59];
Gx2[60] = Gx1[60];
Gx2[61] = Gx1[61];
Gx2[62] = Gx1[62];
Gx2[63] = Gx1[63];
}

void nmpc_multGxGx( real_t* const Gx1, real_t* const Gx2, real_t* const Gx3 )
{
Gx3[0] = + Gx1[0]*Gx2[0] + Gx1[1]*Gx2[8] + Gx1[2]*Gx2[16] + Gx1[3]*Gx2[24] + Gx1[4]*Gx2[32] + Gx1[5]*Gx2[40] + Gx1[6]*Gx2[48] + Gx1[7]*Gx2[56];
Gx3[1] = + Gx1[0]*Gx2[1] + Gx1[1]*Gx2[9] + Gx1[2]*Gx2[17] + Gx1[3]*Gx2[25] + Gx1[4]*Gx2[33] + Gx1[5]*Gx2[41] + Gx1[6]*Gx2[49] + Gx1[7]*Gx2[57];
Gx3[2] = + Gx1[0]*Gx2[2] + Gx1[1]*Gx2[10] + Gx1[2]*Gx2[18] + Gx1[3]*Gx2[26] + Gx1[4]*Gx2[34] + Gx1[5]*Gx2[42] + Gx1[6]*Gx2[50] + Gx1[7]*Gx2[58];
Gx3[3] = + Gx1[0]*Gx2[3] + Gx1[1]*Gx2[11] + Gx1[2]*Gx2[19] + Gx1[3]*Gx2[27] + Gx1[4]*Gx2[35] + Gx1[5]*Gx2[43] + Gx1[6]*Gx2[51] + Gx1[7]*Gx2[59];
Gx3[4] = + Gx1[0]*Gx2[4] + Gx1[1]*Gx2[12] + Gx1[2]*Gx2[20] + Gx1[3]*Gx2[28] + Gx1[4]*Gx2[36] + Gx1[5]*Gx2[44] + Gx1[6]*Gx2[52] + Gx1[7]*Gx2[60];
Gx3[5] = + Gx1[0]*Gx2[5] + Gx1[1]*Gx2[13] + Gx1[2]*Gx2[21] + Gx1[3]*Gx2[29] + Gx1[4]*Gx2[37] + Gx1[5]*Gx2[45] + Gx1[6]*Gx2[53] + Gx1[7]*Gx2[61];
Gx3[6] = + Gx1[0]*Gx2[6] + Gx1[1]*Gx2[14] + Gx1[2]*Gx2[22] + Gx1[3]*Gx2[30] + Gx1[4]*Gx2[38] + Gx1[5]*Gx2[46] + Gx1[6]*Gx2[54] + Gx1[7]*Gx2[62];
Gx3[7] = + Gx1[0]*Gx2[7] + Gx1[1]*Gx2[15] + Gx1[2]*Gx2[23] + Gx1[3]*Gx2[31] + Gx1[4]*Gx2[39] + Gx1[5]*Gx2[47] + Gx1[6]*Gx2[55] + Gx1[7]*Gx2[63];
Gx3[8] = + Gx1[8]*Gx2[0] + Gx1[9]*Gx2[8] + Gx1[10]*Gx2[16] + Gx1[11]*Gx2[24] + Gx1[12]*Gx2[32] + Gx1[13]*Gx2[40] + Gx1[14]*Gx2[48] + Gx1[15]*Gx2[56];
Gx3[9] = + Gx1[8]*Gx2[1] + Gx1[9]*Gx2[9] + Gx1[10]*Gx2[17] + Gx1[11]*Gx2[25] + Gx1[12]*Gx2[33] + Gx1[13]*Gx2[41] + Gx1[14]*Gx2[49] + Gx1[15]*Gx2[57];
Gx3[10] = + Gx1[8]*Gx2[2] + Gx1[9]*Gx2[10] + Gx1[10]*Gx2[18] + Gx1[11]*Gx2[26] + Gx1[12]*Gx2[34] + Gx1[13]*Gx2[42] + Gx1[14]*Gx2[50] + Gx1[15]*Gx2[58];
Gx3[11] = + Gx1[8]*Gx2[3] + Gx1[9]*Gx2[11] + Gx1[10]*Gx2[19] + Gx1[11]*Gx2[27] + Gx1[12]*Gx2[35] + Gx1[13]*Gx2[43] + Gx1[14]*Gx2[51] + Gx1[15]*Gx2[59];
Gx3[12] = + Gx1[8]*Gx2[4] + Gx1[9]*Gx2[12] + Gx1[10]*Gx2[20] + Gx1[11]*Gx2[28] + Gx1[12]*Gx2[36] + Gx1[13]*Gx2[44] + Gx1[14]*Gx2[52] + Gx1[15]*Gx2[60];
Gx3[13] = + Gx1[8]*Gx2[5] + Gx1[9]*Gx2[13] + Gx1[10]*Gx2[21] + Gx1[11]*Gx2[29] + Gx1[12]*Gx2[37] + Gx1[13]*Gx2[45] + Gx1[14]*Gx2[53] + Gx1[15]*Gx2[61];
Gx3[14] = + Gx1[8]*Gx2[6] + Gx1[9]*Gx2[14] + Gx1[10]*Gx2[22] + Gx1[11]*Gx2[30] + Gx1[12]*Gx2[38] + Gx1[13]*Gx2[46] + Gx1[14]*Gx2[54] + Gx1[15]*Gx2[62];
Gx3[15] = + Gx1[8]*Gx2[7] + Gx1[9]*Gx2[15] + Gx1[10]*Gx2[23] + Gx1[11]*Gx2[31] + Gx1[12]*Gx2[39] + Gx1[13]*Gx2[47] + Gx1[14]*Gx2[55] + Gx1[15]*Gx2[63];
Gx3[16] = + Gx1[16]*Gx2[0] + Gx1[17]*Gx2[8] + Gx1[18]*Gx2[16] + Gx1[19]*Gx2[24] + Gx1[20]*Gx2[32] + Gx1[21]*Gx2[40] + Gx1[22]*Gx2[48] + Gx1[23]*Gx2[56];
Gx3[17] = + Gx1[16]*Gx2[1] + Gx1[17]*Gx2[9] + Gx1[18]*Gx2[17] + Gx1[19]*Gx2[25] + Gx1[20]*Gx2[33] + Gx1[21]*Gx2[41] + Gx1[22]*Gx2[49] + Gx1[23]*Gx2[57];
Gx3[18] = + Gx1[16]*Gx2[2] + Gx1[17]*Gx2[10] + Gx1[18]*Gx2[18] + Gx1[19]*Gx2[26] + Gx1[20]*Gx2[34] + Gx1[21]*Gx2[42] + Gx1[22]*Gx2[50] + Gx1[23]*Gx2[58];
Gx3[19] = + Gx1[16]*Gx2[3] + Gx1[17]*Gx2[11] + Gx1[18]*Gx2[19] + Gx1[19]*Gx2[27] + Gx1[20]*Gx2[35] + Gx1[21]*Gx2[43] + Gx1[22]*Gx2[51] + Gx1[23]*Gx2[59];
Gx3[20] = + Gx1[16]*Gx2[4] + Gx1[17]*Gx2[12] + Gx1[18]*Gx2[20] + Gx1[19]*Gx2[28] + Gx1[20]*Gx2[36] + Gx1[21]*Gx2[44] + Gx1[22]*Gx2[52] + Gx1[23]*Gx2[60];
Gx3[21] = + Gx1[16]*Gx2[5] + Gx1[17]*Gx2[13] + Gx1[18]*Gx2[21] + Gx1[19]*Gx2[29] + Gx1[20]*Gx2[37] + Gx1[21]*Gx2[45] + Gx1[22]*Gx2[53] + Gx1[23]*Gx2[61];
Gx3[22] = + Gx1[16]*Gx2[6] + Gx1[17]*Gx2[14] + Gx1[18]*Gx2[22] + Gx1[19]*Gx2[30] + Gx1[20]*Gx2[38] + Gx1[21]*Gx2[46] + Gx1[22]*Gx2[54] + Gx1[23]*Gx2[62];
Gx3[23] = + Gx1[16]*Gx2[7] + Gx1[17]*Gx2[15] + Gx1[18]*Gx2[23] + Gx1[19]*Gx2[31] + Gx1[20]*Gx2[39] + Gx1[21]*Gx2[47] + Gx1[22]*Gx2[55] + Gx1[23]*Gx2[63];
Gx3[24] = + Gx1[24]*Gx2[0] + Gx1[25]*Gx2[8] + Gx1[26]*Gx2[16] + Gx1[27]*Gx2[24] + Gx1[28]*Gx2[32] + Gx1[29]*Gx2[40] + Gx1[30]*Gx2[48] + Gx1[31]*Gx2[56];
Gx3[25] = + Gx1[24]*Gx2[1] + Gx1[25]*Gx2[9] + Gx1[26]*Gx2[17] + Gx1[27]*Gx2[25] + Gx1[28]*Gx2[33] + Gx1[29]*Gx2[41] + Gx1[30]*Gx2[49] + Gx1[31]*Gx2[57];
Gx3[26] = + Gx1[24]*Gx2[2] + Gx1[25]*Gx2[10] + Gx1[26]*Gx2[18] + Gx1[27]*Gx2[26] + Gx1[28]*Gx2[34] + Gx1[29]*Gx2[42] + Gx1[30]*Gx2[50] + Gx1[31]*Gx2[58];
Gx3[27] = + Gx1[24]*Gx2[3] + Gx1[25]*Gx2[11] + Gx1[26]*Gx2[19] + Gx1[27]*Gx2[27] + Gx1[28]*Gx2[35] + Gx1[29]*Gx2[43] + Gx1[30]*Gx2[51] + Gx1[31]*Gx2[59];
Gx3[28] = + Gx1[24]*Gx2[4] + Gx1[25]*Gx2[12] + Gx1[26]*Gx2[20] + Gx1[27]*Gx2[28] + Gx1[28]*Gx2[36] + Gx1[29]*Gx2[44] + Gx1[30]*Gx2[52] + Gx1[31]*Gx2[60];
Gx3[29] = + Gx1[24]*Gx2[5] + Gx1[25]*Gx2[13] + Gx1[26]*Gx2[21] + Gx1[27]*Gx2[29] + Gx1[28]*Gx2[37] + Gx1[29]*Gx2[45] + Gx1[30]*Gx2[53] + Gx1[31]*Gx2[61];
Gx3[30] = + Gx1[24]*Gx2[6] + Gx1[25]*Gx2[14] + Gx1[26]*Gx2[22] + Gx1[27]*Gx2[30] + Gx1[28]*Gx2[38] + Gx1[29]*Gx2[46] + Gx1[30]*Gx2[54] + Gx1[31]*Gx2[62];
Gx3[31] = + Gx1[24]*Gx2[7] + Gx1[25]*Gx2[15] + Gx1[26]*Gx2[23] + Gx1[27]*Gx2[31] + Gx1[28]*Gx2[39] + Gx1[29]*Gx2[47] + Gx1[30]*Gx2[55] + Gx1[31]*Gx2[63];
Gx3[32] = + Gx1[32]*Gx2[0] + Gx1[33]*Gx2[8] + Gx1[34]*Gx2[16] + Gx1[35]*Gx2[24] + Gx1[36]*Gx2[32] + Gx1[37]*Gx2[40] + Gx1[38]*Gx2[48] + Gx1[39]*Gx2[56];
Gx3[33] = + Gx1[32]*Gx2[1] + Gx1[33]*Gx2[9] + Gx1[34]*Gx2[17] + Gx1[35]*Gx2[25] + Gx1[36]*Gx2[33] + Gx1[37]*Gx2[41] + Gx1[38]*Gx2[49] + Gx1[39]*Gx2[57];
Gx3[34] = + Gx1[32]*Gx2[2] + Gx1[33]*Gx2[10] + Gx1[34]*Gx2[18] + Gx1[35]*Gx2[26] + Gx1[36]*Gx2[34] + Gx1[37]*Gx2[42] + Gx1[38]*Gx2[50] + Gx1[39]*Gx2[58];
Gx3[35] = + Gx1[32]*Gx2[3] + Gx1[33]*Gx2[11] + Gx1[34]*Gx2[19] + Gx1[35]*Gx2[27] + Gx1[36]*Gx2[35] + Gx1[37]*Gx2[43] + Gx1[38]*Gx2[51] + Gx1[39]*Gx2[59];
Gx3[36] = + Gx1[32]*Gx2[4] + Gx1[33]*Gx2[12] + Gx1[34]*Gx2[20] + Gx1[35]*Gx2[28] + Gx1[36]*Gx2[36] + Gx1[37]*Gx2[44] + Gx1[38]*Gx2[52] + Gx1[39]*Gx2[60];
Gx3[37] = + Gx1[32]*Gx2[5] + Gx1[33]*Gx2[13] + Gx1[34]*Gx2[21] + Gx1[35]*Gx2[29] + Gx1[36]*Gx2[37] + Gx1[37]*Gx2[45] + Gx1[38]*Gx2[53] + Gx1[39]*Gx2[61];
Gx3[38] = + Gx1[32]*Gx2[6] + Gx1[33]*Gx2[14] + Gx1[34]*Gx2[22] + Gx1[35]*Gx2[30] + Gx1[36]*Gx2[38] + Gx1[37]*Gx2[46] + Gx1[38]*Gx2[54] + Gx1[39]*Gx2[62];
Gx3[39] = + Gx1[32]*Gx2[7] + Gx1[33]*Gx2[15] + Gx1[34]*Gx2[23] + Gx1[35]*Gx2[31] + Gx1[36]*Gx2[39] + Gx1[37]*Gx2[47] + Gx1[38]*Gx2[55] + Gx1[39]*Gx2[63];
Gx3[40] = + Gx1[40]*Gx2[0] + Gx1[41]*Gx2[8] + Gx1[42]*Gx2[16] + Gx1[43]*Gx2[24] + Gx1[44]*Gx2[32] + Gx1[45]*Gx2[40] + Gx1[46]*Gx2[48] + Gx1[47]*Gx2[56];
Gx3[41] = + Gx1[40]*Gx2[1] + Gx1[41]*Gx2[9] + Gx1[42]*Gx2[17] + Gx1[43]*Gx2[25] + Gx1[44]*Gx2[33] + Gx1[45]*Gx2[41] + Gx1[46]*Gx2[49] + Gx1[47]*Gx2[57];
Gx3[42] = + Gx1[40]*Gx2[2] + Gx1[41]*Gx2[10] + Gx1[42]*Gx2[18] + Gx1[43]*Gx2[26] + Gx1[44]*Gx2[34] + Gx1[45]*Gx2[42] + Gx1[46]*Gx2[50] + Gx1[47]*Gx2[58];
Gx3[43] = + Gx1[40]*Gx2[3] + Gx1[41]*Gx2[11] + Gx1[42]*Gx2[19] + Gx1[43]*Gx2[27] + Gx1[44]*Gx2[35] + Gx1[45]*Gx2[43] + Gx1[46]*Gx2[51] + Gx1[47]*Gx2[59];
Gx3[44] = + Gx1[40]*Gx2[4] + Gx1[41]*Gx2[12] + Gx1[42]*Gx2[20] + Gx1[43]*Gx2[28] + Gx1[44]*Gx2[36] + Gx1[45]*Gx2[44] + Gx1[46]*Gx2[52] + Gx1[47]*Gx2[60];
Gx3[45] = + Gx1[40]*Gx2[5] + Gx1[41]*Gx2[13] + Gx1[42]*Gx2[21] + Gx1[43]*Gx2[29] + Gx1[44]*Gx2[37] + Gx1[45]*Gx2[45] + Gx1[46]*Gx2[53] + Gx1[47]*Gx2[61];
Gx3[46] = + Gx1[40]*Gx2[6] + Gx1[41]*Gx2[14] + Gx1[42]*Gx2[22] + Gx1[43]*Gx2[30] + Gx1[44]*Gx2[38] + Gx1[45]*Gx2[46] + Gx1[46]*Gx2[54] + Gx1[47]*Gx2[62];
Gx3[47] = + Gx1[40]*Gx2[7] + Gx1[41]*Gx2[15] + Gx1[42]*Gx2[23] + Gx1[43]*Gx2[31] + Gx1[44]*Gx2[39] + Gx1[45]*Gx2[47] + Gx1[46]*Gx2[55] + Gx1[47]*Gx2[63];
Gx3[48] = + Gx1[48]*Gx2[0] + Gx1[49]*Gx2[8] + Gx1[50]*Gx2[16] + Gx1[51]*Gx2[24] + Gx1[52]*Gx2[32] + Gx1[53]*Gx2[40] + Gx1[54]*Gx2[48] + Gx1[55]*Gx2[56];
Gx3[49] = + Gx1[48]*Gx2[1] + Gx1[49]*Gx2[9] + Gx1[50]*Gx2[17] + Gx1[51]*Gx2[25] + Gx1[52]*Gx2[33] + Gx1[53]*Gx2[41] + Gx1[54]*Gx2[49] + Gx1[55]*Gx2[57];
Gx3[50] = + Gx1[48]*Gx2[2] + Gx1[49]*Gx2[10] + Gx1[50]*Gx2[18] + Gx1[51]*Gx2[26] + Gx1[52]*Gx2[34] + Gx1[53]*Gx2[42] + Gx1[54]*Gx2[50] + Gx1[55]*Gx2[58];
Gx3[51] = + Gx1[48]*Gx2[3] + Gx1[49]*Gx2[11] + Gx1[50]*Gx2[19] + Gx1[51]*Gx2[27] + Gx1[52]*Gx2[35] + Gx1[53]*Gx2[43] + Gx1[54]*Gx2[51] + Gx1[55]*Gx2[59];
Gx3[52] = + Gx1[48]*Gx2[4] + Gx1[49]*Gx2[12] + Gx1[50]*Gx2[20] + Gx1[51]*Gx2[28] + Gx1[52]*Gx2[36] + Gx1[53]*Gx2[44] + Gx1[54]*Gx2[52] + Gx1[55]*Gx2[60];
Gx3[53] = + Gx1[48]*Gx2[5] + Gx1[49]*Gx2[13] + Gx1[50]*Gx2[21] + Gx1[51]*Gx2[29] + Gx1[52]*Gx2[37] + Gx1[53]*Gx2[45] + Gx1[54]*Gx2[53] + Gx1[55]*Gx2[61];
Gx3[54] = + Gx1[48]*Gx2[6] + Gx1[49]*Gx2[14] + Gx1[50]*Gx2[22] + Gx1[51]*Gx2[30] + Gx1[52]*Gx2[38] + Gx1[53]*Gx2[46] + Gx1[54]*Gx2[54] + Gx1[55]*Gx2[62];
Gx3[55] = + Gx1[48]*Gx2[7] + Gx1[49]*Gx2[15] + Gx1[50]*Gx2[23] + Gx1[51]*Gx2[31] + Gx1[52]*Gx2[39] + Gx1[53]*Gx2[47] + Gx1[54]*Gx2[55] + Gx1[55]*Gx2[63];
Gx3[56] = + Gx1[56]*Gx2[0] + Gx1[57]*Gx2[8] + Gx1[58]*Gx2[16] + Gx1[59]*Gx2[24] + Gx1[60]*Gx2[32] + Gx1[61]*Gx2[40] + Gx1[62]*Gx2[48] + Gx1[63]*Gx2[56];
Gx3[57] = + Gx1[56]*Gx2[1] + Gx1[57]*Gx2[9] + Gx1[58]*Gx2[17] + Gx1[59]*Gx2[25] + Gx1[60]*Gx2[33] + Gx1[61]*Gx2[41] + Gx1[62]*Gx2[49] + Gx1[63]*Gx2[57];
Gx3[58] = + Gx1[56]*Gx2[2] + Gx1[57]*Gx2[10] + Gx1[58]*Gx2[18] + Gx1[59]*Gx2[26] + Gx1[60]*Gx2[34] + Gx1[61]*Gx2[42] + Gx1[62]*Gx2[50] + Gx1[63]*Gx2[58];
Gx3[59] = + Gx1[56]*Gx2[3] + Gx1[57]*Gx2[11] + Gx1[58]*Gx2[19] + Gx1[59]*Gx2[27] + Gx1[60]*Gx2[35] + Gx1[61]*Gx2[43] + Gx1[62]*Gx2[51] + Gx1[63]*Gx2[59];
Gx3[60] = + Gx1[56]*Gx2[4] + Gx1[57]*Gx2[12] + Gx1[58]*Gx2[20] + Gx1[59]*Gx2[28] + Gx1[60]*Gx2[36] + Gx1[61]*Gx2[44] + Gx1[62]*Gx2[52] + Gx1[63]*Gx2[60];
Gx3[61] = + Gx1[56]*Gx2[5] + Gx1[57]*Gx2[13] + Gx1[58]*Gx2[21] + Gx1[59]*Gx2[29] + Gx1[60]*Gx2[37] + Gx1[61]*Gx2[45] + Gx1[62]*Gx2[53] + Gx1[63]*Gx2[61];
Gx3[62] = + Gx1[56]*Gx2[6] + Gx1[57]*Gx2[14] + Gx1[58]*Gx2[22] + Gx1[59]*Gx2[30] + Gx1[60]*Gx2[38] + Gx1[61]*Gx2[46] + Gx1[62]*Gx2[54] + Gx1[63]*Gx2[62];
Gx3[63] = + Gx1[56]*Gx2[7] + Gx1[57]*Gx2[15] + Gx1[58]*Gx2[23] + Gx1[59]*Gx2[31] + Gx1[60]*Gx2[39] + Gx1[61]*Gx2[47] + Gx1[62]*Gx2[55] + Gx1[63]*Gx2[63];
}

void nmpc_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
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

void nmpc_moveGuE( real_t* const Gu1, real_t* const Gu2 )
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

void nmpc_setBlockH11( int iRow, int iCol, real_t* const Gu1, real_t* const Gu2 )
{
nmpcWorkspace.H[(iRow * 512 + 1024) + (iCol * 4 + 8)] += + Gu1[0]*Gu2[0] + Gu1[4]*Gu2[4] + Gu1[8]*Gu2[8] + Gu1[12]*Gu2[12] + Gu1[16]*Gu2[16] + Gu1[20]*Gu2[20] + Gu1[24]*Gu2[24] + Gu1[28]*Gu2[28];
nmpcWorkspace.H[(iRow * 512 + 1024) + (iCol * 4 + 9)] += + Gu1[0]*Gu2[1] + Gu1[4]*Gu2[5] + Gu1[8]*Gu2[9] + Gu1[12]*Gu2[13] + Gu1[16]*Gu2[17] + Gu1[20]*Gu2[21] + Gu1[24]*Gu2[25] + Gu1[28]*Gu2[29];
nmpcWorkspace.H[(iRow * 512 + 1024) + (iCol * 4 + 10)] += + Gu1[0]*Gu2[2] + Gu1[4]*Gu2[6] + Gu1[8]*Gu2[10] + Gu1[12]*Gu2[14] + Gu1[16]*Gu2[18] + Gu1[20]*Gu2[22] + Gu1[24]*Gu2[26] + Gu1[28]*Gu2[30];
nmpcWorkspace.H[(iRow * 512 + 1024) + (iCol * 4 + 11)] += + Gu1[0]*Gu2[3] + Gu1[4]*Gu2[7] + Gu1[8]*Gu2[11] + Gu1[12]*Gu2[15] + Gu1[16]*Gu2[19] + Gu1[20]*Gu2[23] + Gu1[24]*Gu2[27] + Gu1[28]*Gu2[31];
nmpcWorkspace.H[(iRow * 512 + 1152) + (iCol * 4 + 8)] += + Gu1[1]*Gu2[0] + Gu1[5]*Gu2[4] + Gu1[9]*Gu2[8] + Gu1[13]*Gu2[12] + Gu1[17]*Gu2[16] + Gu1[21]*Gu2[20] + Gu1[25]*Gu2[24] + Gu1[29]*Gu2[28];
nmpcWorkspace.H[(iRow * 512 + 1152) + (iCol * 4 + 9)] += + Gu1[1]*Gu2[1] + Gu1[5]*Gu2[5] + Gu1[9]*Gu2[9] + Gu1[13]*Gu2[13] + Gu1[17]*Gu2[17] + Gu1[21]*Gu2[21] + Gu1[25]*Gu2[25] + Gu1[29]*Gu2[29];
nmpcWorkspace.H[(iRow * 512 + 1152) + (iCol * 4 + 10)] += + Gu1[1]*Gu2[2] + Gu1[5]*Gu2[6] + Gu1[9]*Gu2[10] + Gu1[13]*Gu2[14] + Gu1[17]*Gu2[18] + Gu1[21]*Gu2[22] + Gu1[25]*Gu2[26] + Gu1[29]*Gu2[30];
nmpcWorkspace.H[(iRow * 512 + 1152) + (iCol * 4 + 11)] += + Gu1[1]*Gu2[3] + Gu1[5]*Gu2[7] + Gu1[9]*Gu2[11] + Gu1[13]*Gu2[15] + Gu1[17]*Gu2[19] + Gu1[21]*Gu2[23] + Gu1[25]*Gu2[27] + Gu1[29]*Gu2[31];
nmpcWorkspace.H[(iRow * 512 + 1280) + (iCol * 4 + 8)] += + Gu1[2]*Gu2[0] + Gu1[6]*Gu2[4] + Gu1[10]*Gu2[8] + Gu1[14]*Gu2[12] + Gu1[18]*Gu2[16] + Gu1[22]*Gu2[20] + Gu1[26]*Gu2[24] + Gu1[30]*Gu2[28];
nmpcWorkspace.H[(iRow * 512 + 1280) + (iCol * 4 + 9)] += + Gu1[2]*Gu2[1] + Gu1[6]*Gu2[5] + Gu1[10]*Gu2[9] + Gu1[14]*Gu2[13] + Gu1[18]*Gu2[17] + Gu1[22]*Gu2[21] + Gu1[26]*Gu2[25] + Gu1[30]*Gu2[29];
nmpcWorkspace.H[(iRow * 512 + 1280) + (iCol * 4 + 10)] += + Gu1[2]*Gu2[2] + Gu1[6]*Gu2[6] + Gu1[10]*Gu2[10] + Gu1[14]*Gu2[14] + Gu1[18]*Gu2[18] + Gu1[22]*Gu2[22] + Gu1[26]*Gu2[26] + Gu1[30]*Gu2[30];
nmpcWorkspace.H[(iRow * 512 + 1280) + (iCol * 4 + 11)] += + Gu1[2]*Gu2[3] + Gu1[6]*Gu2[7] + Gu1[10]*Gu2[11] + Gu1[14]*Gu2[15] + Gu1[18]*Gu2[19] + Gu1[22]*Gu2[23] + Gu1[26]*Gu2[27] + Gu1[30]*Gu2[31];
nmpcWorkspace.H[(iRow * 512 + 1408) + (iCol * 4 + 8)] += + Gu1[3]*Gu2[0] + Gu1[7]*Gu2[4] + Gu1[11]*Gu2[8] + Gu1[15]*Gu2[12] + Gu1[19]*Gu2[16] + Gu1[23]*Gu2[20] + Gu1[27]*Gu2[24] + Gu1[31]*Gu2[28];
nmpcWorkspace.H[(iRow * 512 + 1408) + (iCol * 4 + 9)] += + Gu1[3]*Gu2[1] + Gu1[7]*Gu2[5] + Gu1[11]*Gu2[9] + Gu1[15]*Gu2[13] + Gu1[19]*Gu2[17] + Gu1[23]*Gu2[21] + Gu1[27]*Gu2[25] + Gu1[31]*Gu2[29];
nmpcWorkspace.H[(iRow * 512 + 1408) + (iCol * 4 + 10)] += + Gu1[3]*Gu2[2] + Gu1[7]*Gu2[6] + Gu1[11]*Gu2[10] + Gu1[15]*Gu2[14] + Gu1[19]*Gu2[18] + Gu1[23]*Gu2[22] + Gu1[27]*Gu2[26] + Gu1[31]*Gu2[30];
nmpcWorkspace.H[(iRow * 512 + 1408) + (iCol * 4 + 11)] += + Gu1[3]*Gu2[3] + Gu1[7]*Gu2[7] + Gu1[11]*Gu2[11] + Gu1[15]*Gu2[15] + Gu1[19]*Gu2[19] + Gu1[23]*Gu2[23] + Gu1[27]*Gu2[27] + Gu1[31]*Gu2[31];
}

void nmpc_setBlockH11_R1( int iRow, int iCol, real_t* const R11 )
{
nmpcWorkspace.H[(iRow * 512 + 1024) + (iCol * 4 + 8)] = R11[0];
nmpcWorkspace.H[(iRow * 512 + 1024) + (iCol * 4 + 9)] = R11[1];
nmpcWorkspace.H[(iRow * 512 + 1024) + (iCol * 4 + 10)] = R11[2];
nmpcWorkspace.H[(iRow * 512 + 1024) + (iCol * 4 + 11)] = R11[3];
nmpcWorkspace.H[(iRow * 512 + 1152) + (iCol * 4 + 8)] = R11[4];
nmpcWorkspace.H[(iRow * 512 + 1152) + (iCol * 4 + 9)] = R11[5];
nmpcWorkspace.H[(iRow * 512 + 1152) + (iCol * 4 + 10)] = R11[6];
nmpcWorkspace.H[(iRow * 512 + 1152) + (iCol * 4 + 11)] = R11[7];
nmpcWorkspace.H[(iRow * 512 + 1280) + (iCol * 4 + 8)] = R11[8];
nmpcWorkspace.H[(iRow * 512 + 1280) + (iCol * 4 + 9)] = R11[9];
nmpcWorkspace.H[(iRow * 512 + 1280) + (iCol * 4 + 10)] = R11[10];
nmpcWorkspace.H[(iRow * 512 + 1280) + (iCol * 4 + 11)] = R11[11];
nmpcWorkspace.H[(iRow * 512 + 1408) + (iCol * 4 + 8)] = R11[12];
nmpcWorkspace.H[(iRow * 512 + 1408) + (iCol * 4 + 9)] = R11[13];
nmpcWorkspace.H[(iRow * 512 + 1408) + (iCol * 4 + 10)] = R11[14];
nmpcWorkspace.H[(iRow * 512 + 1408) + (iCol * 4 + 11)] = R11[15];
}

void nmpc_zeroBlockH11( int iRow, int iCol )
{
nmpcWorkspace.H[(iRow * 512 + 1024) + (iCol * 4 + 8)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 512 + 1024) + (iCol * 4 + 9)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 512 + 1024) + (iCol * 4 + 10)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 512 + 1024) + (iCol * 4 + 11)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 512 + 1152) + (iCol * 4 + 8)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 512 + 1152) + (iCol * 4 + 9)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 512 + 1152) + (iCol * 4 + 10)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 512 + 1152) + (iCol * 4 + 11)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 512 + 1280) + (iCol * 4 + 8)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 512 + 1280) + (iCol * 4 + 9)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 512 + 1280) + (iCol * 4 + 10)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 512 + 1280) + (iCol * 4 + 11)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 512 + 1408) + (iCol * 4 + 8)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 512 + 1408) + (iCol * 4 + 9)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 512 + 1408) + (iCol * 4 + 10)] = 0.0000000000000000e+00;
nmpcWorkspace.H[(iRow * 512 + 1408) + (iCol * 4 + 11)] = 0.0000000000000000e+00;
}

void nmpc_copyHTH( int iRow, int iCol )
{
nmpcWorkspace.H[(iRow * 512 + 1024) + (iCol * 4 + 8)] = nmpcWorkspace.H[(iCol * 512 + 1024) + (iRow * 4 + 8)];
nmpcWorkspace.H[(iRow * 512 + 1024) + (iCol * 4 + 9)] = nmpcWorkspace.H[(iCol * 512 + 1152) + (iRow * 4 + 8)];
nmpcWorkspace.H[(iRow * 512 + 1024) + (iCol * 4 + 10)] = nmpcWorkspace.H[(iCol * 512 + 1280) + (iRow * 4 + 8)];
nmpcWorkspace.H[(iRow * 512 + 1024) + (iCol * 4 + 11)] = nmpcWorkspace.H[(iCol * 512 + 1408) + (iRow * 4 + 8)];
nmpcWorkspace.H[(iRow * 512 + 1152) + (iCol * 4 + 8)] = nmpcWorkspace.H[(iCol * 512 + 1024) + (iRow * 4 + 9)];
nmpcWorkspace.H[(iRow * 512 + 1152) + (iCol * 4 + 9)] = nmpcWorkspace.H[(iCol * 512 + 1152) + (iRow * 4 + 9)];
nmpcWorkspace.H[(iRow * 512 + 1152) + (iCol * 4 + 10)] = nmpcWorkspace.H[(iCol * 512 + 1280) + (iRow * 4 + 9)];
nmpcWorkspace.H[(iRow * 512 + 1152) + (iCol * 4 + 11)] = nmpcWorkspace.H[(iCol * 512 + 1408) + (iRow * 4 + 9)];
nmpcWorkspace.H[(iRow * 512 + 1280) + (iCol * 4 + 8)] = nmpcWorkspace.H[(iCol * 512 + 1024) + (iRow * 4 + 10)];
nmpcWorkspace.H[(iRow * 512 + 1280) + (iCol * 4 + 9)] = nmpcWorkspace.H[(iCol * 512 + 1152) + (iRow * 4 + 10)];
nmpcWorkspace.H[(iRow * 512 + 1280) + (iCol * 4 + 10)] = nmpcWorkspace.H[(iCol * 512 + 1280) + (iRow * 4 + 10)];
nmpcWorkspace.H[(iRow * 512 + 1280) + (iCol * 4 + 11)] = nmpcWorkspace.H[(iCol * 512 + 1408) + (iRow * 4 + 10)];
nmpcWorkspace.H[(iRow * 512 + 1408) + (iCol * 4 + 8)] = nmpcWorkspace.H[(iCol * 512 + 1024) + (iRow * 4 + 11)];
nmpcWorkspace.H[(iRow * 512 + 1408) + (iCol * 4 + 9)] = nmpcWorkspace.H[(iCol * 512 + 1152) + (iRow * 4 + 11)];
nmpcWorkspace.H[(iRow * 512 + 1408) + (iCol * 4 + 10)] = nmpcWorkspace.H[(iCol * 512 + 1280) + (iRow * 4 + 11)];
nmpcWorkspace.H[(iRow * 512 + 1408) + (iCol * 4 + 11)] = nmpcWorkspace.H[(iCol * 512 + 1408) + (iRow * 4 + 11)];
}

void nmpc_multQ1d( real_t* const Gx1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + Gx1[0]*dOld[0] + Gx1[1]*dOld[1] + Gx1[2]*dOld[2] + Gx1[3]*dOld[3] + Gx1[4]*dOld[4] + Gx1[5]*dOld[5] + Gx1[6]*dOld[6] + Gx1[7]*dOld[7];
dNew[1] = + Gx1[8]*dOld[0] + Gx1[9]*dOld[1] + Gx1[10]*dOld[2] + Gx1[11]*dOld[3] + Gx1[12]*dOld[4] + Gx1[13]*dOld[5] + Gx1[14]*dOld[6] + Gx1[15]*dOld[7];
dNew[2] = + Gx1[16]*dOld[0] + Gx1[17]*dOld[1] + Gx1[18]*dOld[2] + Gx1[19]*dOld[3] + Gx1[20]*dOld[4] + Gx1[21]*dOld[5] + Gx1[22]*dOld[6] + Gx1[23]*dOld[7];
dNew[3] = + Gx1[24]*dOld[0] + Gx1[25]*dOld[1] + Gx1[26]*dOld[2] + Gx1[27]*dOld[3] + Gx1[28]*dOld[4] + Gx1[29]*dOld[5] + Gx1[30]*dOld[6] + Gx1[31]*dOld[7];
dNew[4] = + Gx1[32]*dOld[0] + Gx1[33]*dOld[1] + Gx1[34]*dOld[2] + Gx1[35]*dOld[3] + Gx1[36]*dOld[4] + Gx1[37]*dOld[5] + Gx1[38]*dOld[6] + Gx1[39]*dOld[7];
dNew[5] = + Gx1[40]*dOld[0] + Gx1[41]*dOld[1] + Gx1[42]*dOld[2] + Gx1[43]*dOld[3] + Gx1[44]*dOld[4] + Gx1[45]*dOld[5] + Gx1[46]*dOld[6] + Gx1[47]*dOld[7];
dNew[6] = + Gx1[48]*dOld[0] + Gx1[49]*dOld[1] + Gx1[50]*dOld[2] + Gx1[51]*dOld[3] + Gx1[52]*dOld[4] + Gx1[53]*dOld[5] + Gx1[54]*dOld[6] + Gx1[55]*dOld[7];
dNew[7] = + Gx1[56]*dOld[0] + Gx1[57]*dOld[1] + Gx1[58]*dOld[2] + Gx1[59]*dOld[3] + Gx1[60]*dOld[4] + Gx1[61]*dOld[5] + Gx1[62]*dOld[6] + Gx1[63]*dOld[7];
}

void nmpc_multQN1d( real_t* const QN1, real_t* const dOld, real_t* const dNew )
{
dNew[0] = + nmpcWorkspace.QN1[0]*dOld[0] + nmpcWorkspace.QN1[1]*dOld[1] + nmpcWorkspace.QN1[2]*dOld[2] + nmpcWorkspace.QN1[3]*dOld[3] + nmpcWorkspace.QN1[4]*dOld[4] + nmpcWorkspace.QN1[5]*dOld[5] + nmpcWorkspace.QN1[6]*dOld[6] + nmpcWorkspace.QN1[7]*dOld[7];
dNew[1] = + nmpcWorkspace.QN1[8]*dOld[0] + nmpcWorkspace.QN1[9]*dOld[1] + nmpcWorkspace.QN1[10]*dOld[2] + nmpcWorkspace.QN1[11]*dOld[3] + nmpcWorkspace.QN1[12]*dOld[4] + nmpcWorkspace.QN1[13]*dOld[5] + nmpcWorkspace.QN1[14]*dOld[6] + nmpcWorkspace.QN1[15]*dOld[7];
dNew[2] = + nmpcWorkspace.QN1[16]*dOld[0] + nmpcWorkspace.QN1[17]*dOld[1] + nmpcWorkspace.QN1[18]*dOld[2] + nmpcWorkspace.QN1[19]*dOld[3] + nmpcWorkspace.QN1[20]*dOld[4] + nmpcWorkspace.QN1[21]*dOld[5] + nmpcWorkspace.QN1[22]*dOld[6] + nmpcWorkspace.QN1[23]*dOld[7];
dNew[3] = + nmpcWorkspace.QN1[24]*dOld[0] + nmpcWorkspace.QN1[25]*dOld[1] + nmpcWorkspace.QN1[26]*dOld[2] + nmpcWorkspace.QN1[27]*dOld[3] + nmpcWorkspace.QN1[28]*dOld[4] + nmpcWorkspace.QN1[29]*dOld[5] + nmpcWorkspace.QN1[30]*dOld[6] + nmpcWorkspace.QN1[31]*dOld[7];
dNew[4] = + nmpcWorkspace.QN1[32]*dOld[0] + nmpcWorkspace.QN1[33]*dOld[1] + nmpcWorkspace.QN1[34]*dOld[2] + nmpcWorkspace.QN1[35]*dOld[3] + nmpcWorkspace.QN1[36]*dOld[4] + nmpcWorkspace.QN1[37]*dOld[5] + nmpcWorkspace.QN1[38]*dOld[6] + nmpcWorkspace.QN1[39]*dOld[7];
dNew[5] = + nmpcWorkspace.QN1[40]*dOld[0] + nmpcWorkspace.QN1[41]*dOld[1] + nmpcWorkspace.QN1[42]*dOld[2] + nmpcWorkspace.QN1[43]*dOld[3] + nmpcWorkspace.QN1[44]*dOld[4] + nmpcWorkspace.QN1[45]*dOld[5] + nmpcWorkspace.QN1[46]*dOld[6] + nmpcWorkspace.QN1[47]*dOld[7];
dNew[6] = + nmpcWorkspace.QN1[48]*dOld[0] + nmpcWorkspace.QN1[49]*dOld[1] + nmpcWorkspace.QN1[50]*dOld[2] + nmpcWorkspace.QN1[51]*dOld[3] + nmpcWorkspace.QN1[52]*dOld[4] + nmpcWorkspace.QN1[53]*dOld[5] + nmpcWorkspace.QN1[54]*dOld[6] + nmpcWorkspace.QN1[55]*dOld[7];
dNew[7] = + nmpcWorkspace.QN1[56]*dOld[0] + nmpcWorkspace.QN1[57]*dOld[1] + nmpcWorkspace.QN1[58]*dOld[2] + nmpcWorkspace.QN1[59]*dOld[3] + nmpcWorkspace.QN1[60]*dOld[4] + nmpcWorkspace.QN1[61]*dOld[5] + nmpcWorkspace.QN1[62]*dOld[6] + nmpcWorkspace.QN1[63]*dOld[7];
}

void nmpc_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7] + R2[8]*Dy1[8] + R2[9]*Dy1[9] + R2[10]*Dy1[10] + R2[11]*Dy1[11];
RDy1[1] = + R2[12]*Dy1[0] + R2[13]*Dy1[1] + R2[14]*Dy1[2] + R2[15]*Dy1[3] + R2[16]*Dy1[4] + R2[17]*Dy1[5] + R2[18]*Dy1[6] + R2[19]*Dy1[7] + R2[20]*Dy1[8] + R2[21]*Dy1[9] + R2[22]*Dy1[10] + R2[23]*Dy1[11];
RDy1[2] = + R2[24]*Dy1[0] + R2[25]*Dy1[1] + R2[26]*Dy1[2] + R2[27]*Dy1[3] + R2[28]*Dy1[4] + R2[29]*Dy1[5] + R2[30]*Dy1[6] + R2[31]*Dy1[7] + R2[32]*Dy1[8] + R2[33]*Dy1[9] + R2[34]*Dy1[10] + R2[35]*Dy1[11];
RDy1[3] = + R2[36]*Dy1[0] + R2[37]*Dy1[1] + R2[38]*Dy1[2] + R2[39]*Dy1[3] + R2[40]*Dy1[4] + R2[41]*Dy1[5] + R2[42]*Dy1[6] + R2[43]*Dy1[7] + R2[44]*Dy1[8] + R2[45]*Dy1[9] + R2[46]*Dy1[10] + R2[47]*Dy1[11];
}

void nmpc_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
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

void nmpc_multEQDy( real_t* const E1, real_t* const QDy1, real_t* const U1 )
{
U1[0] += + E1[0]*QDy1[0] + E1[4]*QDy1[1] + E1[8]*QDy1[2] + E1[12]*QDy1[3] + E1[16]*QDy1[4] + E1[20]*QDy1[5] + E1[24]*QDy1[6] + E1[28]*QDy1[7];
U1[1] += + E1[1]*QDy1[0] + E1[5]*QDy1[1] + E1[9]*QDy1[2] + E1[13]*QDy1[3] + E1[17]*QDy1[4] + E1[21]*QDy1[5] + E1[25]*QDy1[6] + E1[29]*QDy1[7];
U1[2] += + E1[2]*QDy1[0] + E1[6]*QDy1[1] + E1[10]*QDy1[2] + E1[14]*QDy1[3] + E1[18]*QDy1[4] + E1[22]*QDy1[5] + E1[26]*QDy1[6] + E1[30]*QDy1[7];
U1[3] += + E1[3]*QDy1[0] + E1[7]*QDy1[1] + E1[11]*QDy1[2] + E1[15]*QDy1[3] + E1[19]*QDy1[4] + E1[23]*QDy1[5] + E1[27]*QDy1[6] + E1[31]*QDy1[7];
}

void nmpc_multQETGx( real_t* const E1, real_t* const Gx1, real_t* const H101 )
{
H101[0] += + E1[0]*Gx1[0] + E1[4]*Gx1[8] + E1[8]*Gx1[16] + E1[12]*Gx1[24] + E1[16]*Gx1[32] + E1[20]*Gx1[40] + E1[24]*Gx1[48] + E1[28]*Gx1[56];
H101[1] += + E1[0]*Gx1[1] + E1[4]*Gx1[9] + E1[8]*Gx1[17] + E1[12]*Gx1[25] + E1[16]*Gx1[33] + E1[20]*Gx1[41] + E1[24]*Gx1[49] + E1[28]*Gx1[57];
H101[2] += + E1[0]*Gx1[2] + E1[4]*Gx1[10] + E1[8]*Gx1[18] + E1[12]*Gx1[26] + E1[16]*Gx1[34] + E1[20]*Gx1[42] + E1[24]*Gx1[50] + E1[28]*Gx1[58];
H101[3] += + E1[0]*Gx1[3] + E1[4]*Gx1[11] + E1[8]*Gx1[19] + E1[12]*Gx1[27] + E1[16]*Gx1[35] + E1[20]*Gx1[43] + E1[24]*Gx1[51] + E1[28]*Gx1[59];
H101[4] += + E1[0]*Gx1[4] + E1[4]*Gx1[12] + E1[8]*Gx1[20] + E1[12]*Gx1[28] + E1[16]*Gx1[36] + E1[20]*Gx1[44] + E1[24]*Gx1[52] + E1[28]*Gx1[60];
H101[5] += + E1[0]*Gx1[5] + E1[4]*Gx1[13] + E1[8]*Gx1[21] + E1[12]*Gx1[29] + E1[16]*Gx1[37] + E1[20]*Gx1[45] + E1[24]*Gx1[53] + E1[28]*Gx1[61];
H101[6] += + E1[0]*Gx1[6] + E1[4]*Gx1[14] + E1[8]*Gx1[22] + E1[12]*Gx1[30] + E1[16]*Gx1[38] + E1[20]*Gx1[46] + E1[24]*Gx1[54] + E1[28]*Gx1[62];
H101[7] += + E1[0]*Gx1[7] + E1[4]*Gx1[15] + E1[8]*Gx1[23] + E1[12]*Gx1[31] + E1[16]*Gx1[39] + E1[20]*Gx1[47] + E1[24]*Gx1[55] + E1[28]*Gx1[63];
H101[8] += + E1[1]*Gx1[0] + E1[5]*Gx1[8] + E1[9]*Gx1[16] + E1[13]*Gx1[24] + E1[17]*Gx1[32] + E1[21]*Gx1[40] + E1[25]*Gx1[48] + E1[29]*Gx1[56];
H101[9] += + E1[1]*Gx1[1] + E1[5]*Gx1[9] + E1[9]*Gx1[17] + E1[13]*Gx1[25] + E1[17]*Gx1[33] + E1[21]*Gx1[41] + E1[25]*Gx1[49] + E1[29]*Gx1[57];
H101[10] += + E1[1]*Gx1[2] + E1[5]*Gx1[10] + E1[9]*Gx1[18] + E1[13]*Gx1[26] + E1[17]*Gx1[34] + E1[21]*Gx1[42] + E1[25]*Gx1[50] + E1[29]*Gx1[58];
H101[11] += + E1[1]*Gx1[3] + E1[5]*Gx1[11] + E1[9]*Gx1[19] + E1[13]*Gx1[27] + E1[17]*Gx1[35] + E1[21]*Gx1[43] + E1[25]*Gx1[51] + E1[29]*Gx1[59];
H101[12] += + E1[1]*Gx1[4] + E1[5]*Gx1[12] + E1[9]*Gx1[20] + E1[13]*Gx1[28] + E1[17]*Gx1[36] + E1[21]*Gx1[44] + E1[25]*Gx1[52] + E1[29]*Gx1[60];
H101[13] += + E1[1]*Gx1[5] + E1[5]*Gx1[13] + E1[9]*Gx1[21] + E1[13]*Gx1[29] + E1[17]*Gx1[37] + E1[21]*Gx1[45] + E1[25]*Gx1[53] + E1[29]*Gx1[61];
H101[14] += + E1[1]*Gx1[6] + E1[5]*Gx1[14] + E1[9]*Gx1[22] + E1[13]*Gx1[30] + E1[17]*Gx1[38] + E1[21]*Gx1[46] + E1[25]*Gx1[54] + E1[29]*Gx1[62];
H101[15] += + E1[1]*Gx1[7] + E1[5]*Gx1[15] + E1[9]*Gx1[23] + E1[13]*Gx1[31] + E1[17]*Gx1[39] + E1[21]*Gx1[47] + E1[25]*Gx1[55] + E1[29]*Gx1[63];
H101[16] += + E1[2]*Gx1[0] + E1[6]*Gx1[8] + E1[10]*Gx1[16] + E1[14]*Gx1[24] + E1[18]*Gx1[32] + E1[22]*Gx1[40] + E1[26]*Gx1[48] + E1[30]*Gx1[56];
H101[17] += + E1[2]*Gx1[1] + E1[6]*Gx1[9] + E1[10]*Gx1[17] + E1[14]*Gx1[25] + E1[18]*Gx1[33] + E1[22]*Gx1[41] + E1[26]*Gx1[49] + E1[30]*Gx1[57];
H101[18] += + E1[2]*Gx1[2] + E1[6]*Gx1[10] + E1[10]*Gx1[18] + E1[14]*Gx1[26] + E1[18]*Gx1[34] + E1[22]*Gx1[42] + E1[26]*Gx1[50] + E1[30]*Gx1[58];
H101[19] += + E1[2]*Gx1[3] + E1[6]*Gx1[11] + E1[10]*Gx1[19] + E1[14]*Gx1[27] + E1[18]*Gx1[35] + E1[22]*Gx1[43] + E1[26]*Gx1[51] + E1[30]*Gx1[59];
H101[20] += + E1[2]*Gx1[4] + E1[6]*Gx1[12] + E1[10]*Gx1[20] + E1[14]*Gx1[28] + E1[18]*Gx1[36] + E1[22]*Gx1[44] + E1[26]*Gx1[52] + E1[30]*Gx1[60];
H101[21] += + E1[2]*Gx1[5] + E1[6]*Gx1[13] + E1[10]*Gx1[21] + E1[14]*Gx1[29] + E1[18]*Gx1[37] + E1[22]*Gx1[45] + E1[26]*Gx1[53] + E1[30]*Gx1[61];
H101[22] += + E1[2]*Gx1[6] + E1[6]*Gx1[14] + E1[10]*Gx1[22] + E1[14]*Gx1[30] + E1[18]*Gx1[38] + E1[22]*Gx1[46] + E1[26]*Gx1[54] + E1[30]*Gx1[62];
H101[23] += + E1[2]*Gx1[7] + E1[6]*Gx1[15] + E1[10]*Gx1[23] + E1[14]*Gx1[31] + E1[18]*Gx1[39] + E1[22]*Gx1[47] + E1[26]*Gx1[55] + E1[30]*Gx1[63];
H101[24] += + E1[3]*Gx1[0] + E1[7]*Gx1[8] + E1[11]*Gx1[16] + E1[15]*Gx1[24] + E1[19]*Gx1[32] + E1[23]*Gx1[40] + E1[27]*Gx1[48] + E1[31]*Gx1[56];
H101[25] += + E1[3]*Gx1[1] + E1[7]*Gx1[9] + E1[11]*Gx1[17] + E1[15]*Gx1[25] + E1[19]*Gx1[33] + E1[23]*Gx1[41] + E1[27]*Gx1[49] + E1[31]*Gx1[57];
H101[26] += + E1[3]*Gx1[2] + E1[7]*Gx1[10] + E1[11]*Gx1[18] + E1[15]*Gx1[26] + E1[19]*Gx1[34] + E1[23]*Gx1[42] + E1[27]*Gx1[50] + E1[31]*Gx1[58];
H101[27] += + E1[3]*Gx1[3] + E1[7]*Gx1[11] + E1[11]*Gx1[19] + E1[15]*Gx1[27] + E1[19]*Gx1[35] + E1[23]*Gx1[43] + E1[27]*Gx1[51] + E1[31]*Gx1[59];
H101[28] += + E1[3]*Gx1[4] + E1[7]*Gx1[12] + E1[11]*Gx1[20] + E1[15]*Gx1[28] + E1[19]*Gx1[36] + E1[23]*Gx1[44] + E1[27]*Gx1[52] + E1[31]*Gx1[60];
H101[29] += + E1[3]*Gx1[5] + E1[7]*Gx1[13] + E1[11]*Gx1[21] + E1[15]*Gx1[29] + E1[19]*Gx1[37] + E1[23]*Gx1[45] + E1[27]*Gx1[53] + E1[31]*Gx1[61];
H101[30] += + E1[3]*Gx1[6] + E1[7]*Gx1[14] + E1[11]*Gx1[22] + E1[15]*Gx1[30] + E1[19]*Gx1[38] + E1[23]*Gx1[46] + E1[27]*Gx1[54] + E1[31]*Gx1[62];
H101[31] += + E1[3]*Gx1[7] + E1[7]*Gx1[15] + E1[11]*Gx1[23] + E1[15]*Gx1[31] + E1[19]*Gx1[39] + E1[23]*Gx1[47] + E1[27]*Gx1[55] + E1[31]*Gx1[63];
}

void nmpc_zeroBlockH10( real_t* const H101 )
{
{ int lCopy; for (lCopy = 0; lCopy < 32; lCopy++) H101[ lCopy ] = 0; }
}

void nmpc_multEDu( real_t* const E1, real_t* const U1, real_t* const dNew )
{
dNew[0] += + E1[0]*U1[0] + E1[1]*U1[1] + E1[2]*U1[2] + E1[3]*U1[3];
dNew[1] += + E1[4]*U1[0] + E1[5]*U1[1] + E1[6]*U1[2] + E1[7]*U1[3];
dNew[2] += + E1[8]*U1[0] + E1[9]*U1[1] + E1[10]*U1[2] + E1[11]*U1[3];
dNew[3] += + E1[12]*U1[0] + E1[13]*U1[1] + E1[14]*U1[2] + E1[15]*U1[3];
dNew[4] += + E1[16]*U1[0] + E1[17]*U1[1] + E1[18]*U1[2] + E1[19]*U1[3];
dNew[5] += + E1[20]*U1[0] + E1[21]*U1[1] + E1[22]*U1[2] + E1[23]*U1[3];
dNew[6] += + E1[24]*U1[0] + E1[25]*U1[1] + E1[26]*U1[2] + E1[27]*U1[3];
dNew[7] += + E1[28]*U1[0] + E1[29]*U1[1] + E1[30]*U1[2] + E1[31]*U1[3];
}

void nmpc_zeroBlockH00(  )
{
nmpcWorkspace.H[0] = 0.0000000000000000e+00;
nmpcWorkspace.H[1] = 0.0000000000000000e+00;
nmpcWorkspace.H[2] = 0.0000000000000000e+00;
nmpcWorkspace.H[3] = 0.0000000000000000e+00;
nmpcWorkspace.H[4] = 0.0000000000000000e+00;
nmpcWorkspace.H[5] = 0.0000000000000000e+00;
nmpcWorkspace.H[6] = 0.0000000000000000e+00;
nmpcWorkspace.H[7] = 0.0000000000000000e+00;
nmpcWorkspace.H[128] = 0.0000000000000000e+00;
nmpcWorkspace.H[129] = 0.0000000000000000e+00;
nmpcWorkspace.H[130] = 0.0000000000000000e+00;
nmpcWorkspace.H[131] = 0.0000000000000000e+00;
nmpcWorkspace.H[132] = 0.0000000000000000e+00;
nmpcWorkspace.H[133] = 0.0000000000000000e+00;
nmpcWorkspace.H[134] = 0.0000000000000000e+00;
nmpcWorkspace.H[135] = 0.0000000000000000e+00;
nmpcWorkspace.H[256] = 0.0000000000000000e+00;
nmpcWorkspace.H[257] = 0.0000000000000000e+00;
nmpcWorkspace.H[258] = 0.0000000000000000e+00;
nmpcWorkspace.H[259] = 0.0000000000000000e+00;
nmpcWorkspace.H[260] = 0.0000000000000000e+00;
nmpcWorkspace.H[261] = 0.0000000000000000e+00;
nmpcWorkspace.H[262] = 0.0000000000000000e+00;
nmpcWorkspace.H[263] = 0.0000000000000000e+00;
nmpcWorkspace.H[384] = 0.0000000000000000e+00;
nmpcWorkspace.H[385] = 0.0000000000000000e+00;
nmpcWorkspace.H[386] = 0.0000000000000000e+00;
nmpcWorkspace.H[387] = 0.0000000000000000e+00;
nmpcWorkspace.H[388] = 0.0000000000000000e+00;
nmpcWorkspace.H[389] = 0.0000000000000000e+00;
nmpcWorkspace.H[390] = 0.0000000000000000e+00;
nmpcWorkspace.H[391] = 0.0000000000000000e+00;
nmpcWorkspace.H[512] = 0.0000000000000000e+00;
nmpcWorkspace.H[513] = 0.0000000000000000e+00;
nmpcWorkspace.H[514] = 0.0000000000000000e+00;
nmpcWorkspace.H[515] = 0.0000000000000000e+00;
nmpcWorkspace.H[516] = 0.0000000000000000e+00;
nmpcWorkspace.H[517] = 0.0000000000000000e+00;
nmpcWorkspace.H[518] = 0.0000000000000000e+00;
nmpcWorkspace.H[519] = 0.0000000000000000e+00;
nmpcWorkspace.H[640] = 0.0000000000000000e+00;
nmpcWorkspace.H[641] = 0.0000000000000000e+00;
nmpcWorkspace.H[642] = 0.0000000000000000e+00;
nmpcWorkspace.H[643] = 0.0000000000000000e+00;
nmpcWorkspace.H[644] = 0.0000000000000000e+00;
nmpcWorkspace.H[645] = 0.0000000000000000e+00;
nmpcWorkspace.H[646] = 0.0000000000000000e+00;
nmpcWorkspace.H[647] = 0.0000000000000000e+00;
nmpcWorkspace.H[768] = 0.0000000000000000e+00;
nmpcWorkspace.H[769] = 0.0000000000000000e+00;
nmpcWorkspace.H[770] = 0.0000000000000000e+00;
nmpcWorkspace.H[771] = 0.0000000000000000e+00;
nmpcWorkspace.H[772] = 0.0000000000000000e+00;
nmpcWorkspace.H[773] = 0.0000000000000000e+00;
nmpcWorkspace.H[774] = 0.0000000000000000e+00;
nmpcWorkspace.H[775] = 0.0000000000000000e+00;
nmpcWorkspace.H[896] = 0.0000000000000000e+00;
nmpcWorkspace.H[897] = 0.0000000000000000e+00;
nmpcWorkspace.H[898] = 0.0000000000000000e+00;
nmpcWorkspace.H[899] = 0.0000000000000000e+00;
nmpcWorkspace.H[900] = 0.0000000000000000e+00;
nmpcWorkspace.H[901] = 0.0000000000000000e+00;
nmpcWorkspace.H[902] = 0.0000000000000000e+00;
nmpcWorkspace.H[903] = 0.0000000000000000e+00;
}

void nmpc_multCTQC( real_t* const Gx1, real_t* const Gx2 )
{
nmpcWorkspace.H[0] += + Gx1[0]*Gx2[0] + Gx1[8]*Gx2[8] + Gx1[16]*Gx2[16] + Gx1[24]*Gx2[24] + Gx1[32]*Gx2[32] + Gx1[40]*Gx2[40] + Gx1[48]*Gx2[48] + Gx1[56]*Gx2[56];
nmpcWorkspace.H[1] += + Gx1[0]*Gx2[1] + Gx1[8]*Gx2[9] + Gx1[16]*Gx2[17] + Gx1[24]*Gx2[25] + Gx1[32]*Gx2[33] + Gx1[40]*Gx2[41] + Gx1[48]*Gx2[49] + Gx1[56]*Gx2[57];
nmpcWorkspace.H[2] += + Gx1[0]*Gx2[2] + Gx1[8]*Gx2[10] + Gx1[16]*Gx2[18] + Gx1[24]*Gx2[26] + Gx1[32]*Gx2[34] + Gx1[40]*Gx2[42] + Gx1[48]*Gx2[50] + Gx1[56]*Gx2[58];
nmpcWorkspace.H[3] += + Gx1[0]*Gx2[3] + Gx1[8]*Gx2[11] + Gx1[16]*Gx2[19] + Gx1[24]*Gx2[27] + Gx1[32]*Gx2[35] + Gx1[40]*Gx2[43] + Gx1[48]*Gx2[51] + Gx1[56]*Gx2[59];
nmpcWorkspace.H[4] += + Gx1[0]*Gx2[4] + Gx1[8]*Gx2[12] + Gx1[16]*Gx2[20] + Gx1[24]*Gx2[28] + Gx1[32]*Gx2[36] + Gx1[40]*Gx2[44] + Gx1[48]*Gx2[52] + Gx1[56]*Gx2[60];
nmpcWorkspace.H[5] += + Gx1[0]*Gx2[5] + Gx1[8]*Gx2[13] + Gx1[16]*Gx2[21] + Gx1[24]*Gx2[29] + Gx1[32]*Gx2[37] + Gx1[40]*Gx2[45] + Gx1[48]*Gx2[53] + Gx1[56]*Gx2[61];
nmpcWorkspace.H[6] += + Gx1[0]*Gx2[6] + Gx1[8]*Gx2[14] + Gx1[16]*Gx2[22] + Gx1[24]*Gx2[30] + Gx1[32]*Gx2[38] + Gx1[40]*Gx2[46] + Gx1[48]*Gx2[54] + Gx1[56]*Gx2[62];
nmpcWorkspace.H[7] += + Gx1[0]*Gx2[7] + Gx1[8]*Gx2[15] + Gx1[16]*Gx2[23] + Gx1[24]*Gx2[31] + Gx1[32]*Gx2[39] + Gx1[40]*Gx2[47] + Gx1[48]*Gx2[55] + Gx1[56]*Gx2[63];
nmpcWorkspace.H[128] += + Gx1[1]*Gx2[0] + Gx1[9]*Gx2[8] + Gx1[17]*Gx2[16] + Gx1[25]*Gx2[24] + Gx1[33]*Gx2[32] + Gx1[41]*Gx2[40] + Gx1[49]*Gx2[48] + Gx1[57]*Gx2[56];
nmpcWorkspace.H[129] += + Gx1[1]*Gx2[1] + Gx1[9]*Gx2[9] + Gx1[17]*Gx2[17] + Gx1[25]*Gx2[25] + Gx1[33]*Gx2[33] + Gx1[41]*Gx2[41] + Gx1[49]*Gx2[49] + Gx1[57]*Gx2[57];
nmpcWorkspace.H[130] += + Gx1[1]*Gx2[2] + Gx1[9]*Gx2[10] + Gx1[17]*Gx2[18] + Gx1[25]*Gx2[26] + Gx1[33]*Gx2[34] + Gx1[41]*Gx2[42] + Gx1[49]*Gx2[50] + Gx1[57]*Gx2[58];
nmpcWorkspace.H[131] += + Gx1[1]*Gx2[3] + Gx1[9]*Gx2[11] + Gx1[17]*Gx2[19] + Gx1[25]*Gx2[27] + Gx1[33]*Gx2[35] + Gx1[41]*Gx2[43] + Gx1[49]*Gx2[51] + Gx1[57]*Gx2[59];
nmpcWorkspace.H[132] += + Gx1[1]*Gx2[4] + Gx1[9]*Gx2[12] + Gx1[17]*Gx2[20] + Gx1[25]*Gx2[28] + Gx1[33]*Gx2[36] + Gx1[41]*Gx2[44] + Gx1[49]*Gx2[52] + Gx1[57]*Gx2[60];
nmpcWorkspace.H[133] += + Gx1[1]*Gx2[5] + Gx1[9]*Gx2[13] + Gx1[17]*Gx2[21] + Gx1[25]*Gx2[29] + Gx1[33]*Gx2[37] + Gx1[41]*Gx2[45] + Gx1[49]*Gx2[53] + Gx1[57]*Gx2[61];
nmpcWorkspace.H[134] += + Gx1[1]*Gx2[6] + Gx1[9]*Gx2[14] + Gx1[17]*Gx2[22] + Gx1[25]*Gx2[30] + Gx1[33]*Gx2[38] + Gx1[41]*Gx2[46] + Gx1[49]*Gx2[54] + Gx1[57]*Gx2[62];
nmpcWorkspace.H[135] += + Gx1[1]*Gx2[7] + Gx1[9]*Gx2[15] + Gx1[17]*Gx2[23] + Gx1[25]*Gx2[31] + Gx1[33]*Gx2[39] + Gx1[41]*Gx2[47] + Gx1[49]*Gx2[55] + Gx1[57]*Gx2[63];
nmpcWorkspace.H[256] += + Gx1[2]*Gx2[0] + Gx1[10]*Gx2[8] + Gx1[18]*Gx2[16] + Gx1[26]*Gx2[24] + Gx1[34]*Gx2[32] + Gx1[42]*Gx2[40] + Gx1[50]*Gx2[48] + Gx1[58]*Gx2[56];
nmpcWorkspace.H[257] += + Gx1[2]*Gx2[1] + Gx1[10]*Gx2[9] + Gx1[18]*Gx2[17] + Gx1[26]*Gx2[25] + Gx1[34]*Gx2[33] + Gx1[42]*Gx2[41] + Gx1[50]*Gx2[49] + Gx1[58]*Gx2[57];
nmpcWorkspace.H[258] += + Gx1[2]*Gx2[2] + Gx1[10]*Gx2[10] + Gx1[18]*Gx2[18] + Gx1[26]*Gx2[26] + Gx1[34]*Gx2[34] + Gx1[42]*Gx2[42] + Gx1[50]*Gx2[50] + Gx1[58]*Gx2[58];
nmpcWorkspace.H[259] += + Gx1[2]*Gx2[3] + Gx1[10]*Gx2[11] + Gx1[18]*Gx2[19] + Gx1[26]*Gx2[27] + Gx1[34]*Gx2[35] + Gx1[42]*Gx2[43] + Gx1[50]*Gx2[51] + Gx1[58]*Gx2[59];
nmpcWorkspace.H[260] += + Gx1[2]*Gx2[4] + Gx1[10]*Gx2[12] + Gx1[18]*Gx2[20] + Gx1[26]*Gx2[28] + Gx1[34]*Gx2[36] + Gx1[42]*Gx2[44] + Gx1[50]*Gx2[52] + Gx1[58]*Gx2[60];
nmpcWorkspace.H[261] += + Gx1[2]*Gx2[5] + Gx1[10]*Gx2[13] + Gx1[18]*Gx2[21] + Gx1[26]*Gx2[29] + Gx1[34]*Gx2[37] + Gx1[42]*Gx2[45] + Gx1[50]*Gx2[53] + Gx1[58]*Gx2[61];
nmpcWorkspace.H[262] += + Gx1[2]*Gx2[6] + Gx1[10]*Gx2[14] + Gx1[18]*Gx2[22] + Gx1[26]*Gx2[30] + Gx1[34]*Gx2[38] + Gx1[42]*Gx2[46] + Gx1[50]*Gx2[54] + Gx1[58]*Gx2[62];
nmpcWorkspace.H[263] += + Gx1[2]*Gx2[7] + Gx1[10]*Gx2[15] + Gx1[18]*Gx2[23] + Gx1[26]*Gx2[31] + Gx1[34]*Gx2[39] + Gx1[42]*Gx2[47] + Gx1[50]*Gx2[55] + Gx1[58]*Gx2[63];
nmpcWorkspace.H[384] += + Gx1[3]*Gx2[0] + Gx1[11]*Gx2[8] + Gx1[19]*Gx2[16] + Gx1[27]*Gx2[24] + Gx1[35]*Gx2[32] + Gx1[43]*Gx2[40] + Gx1[51]*Gx2[48] + Gx1[59]*Gx2[56];
nmpcWorkspace.H[385] += + Gx1[3]*Gx2[1] + Gx1[11]*Gx2[9] + Gx1[19]*Gx2[17] + Gx1[27]*Gx2[25] + Gx1[35]*Gx2[33] + Gx1[43]*Gx2[41] + Gx1[51]*Gx2[49] + Gx1[59]*Gx2[57];
nmpcWorkspace.H[386] += + Gx1[3]*Gx2[2] + Gx1[11]*Gx2[10] + Gx1[19]*Gx2[18] + Gx1[27]*Gx2[26] + Gx1[35]*Gx2[34] + Gx1[43]*Gx2[42] + Gx1[51]*Gx2[50] + Gx1[59]*Gx2[58];
nmpcWorkspace.H[387] += + Gx1[3]*Gx2[3] + Gx1[11]*Gx2[11] + Gx1[19]*Gx2[19] + Gx1[27]*Gx2[27] + Gx1[35]*Gx2[35] + Gx1[43]*Gx2[43] + Gx1[51]*Gx2[51] + Gx1[59]*Gx2[59];
nmpcWorkspace.H[388] += + Gx1[3]*Gx2[4] + Gx1[11]*Gx2[12] + Gx1[19]*Gx2[20] + Gx1[27]*Gx2[28] + Gx1[35]*Gx2[36] + Gx1[43]*Gx2[44] + Gx1[51]*Gx2[52] + Gx1[59]*Gx2[60];
nmpcWorkspace.H[389] += + Gx1[3]*Gx2[5] + Gx1[11]*Gx2[13] + Gx1[19]*Gx2[21] + Gx1[27]*Gx2[29] + Gx1[35]*Gx2[37] + Gx1[43]*Gx2[45] + Gx1[51]*Gx2[53] + Gx1[59]*Gx2[61];
nmpcWorkspace.H[390] += + Gx1[3]*Gx2[6] + Gx1[11]*Gx2[14] + Gx1[19]*Gx2[22] + Gx1[27]*Gx2[30] + Gx1[35]*Gx2[38] + Gx1[43]*Gx2[46] + Gx1[51]*Gx2[54] + Gx1[59]*Gx2[62];
nmpcWorkspace.H[391] += + Gx1[3]*Gx2[7] + Gx1[11]*Gx2[15] + Gx1[19]*Gx2[23] + Gx1[27]*Gx2[31] + Gx1[35]*Gx2[39] + Gx1[43]*Gx2[47] + Gx1[51]*Gx2[55] + Gx1[59]*Gx2[63];
nmpcWorkspace.H[512] += + Gx1[4]*Gx2[0] + Gx1[12]*Gx2[8] + Gx1[20]*Gx2[16] + Gx1[28]*Gx2[24] + Gx1[36]*Gx2[32] + Gx1[44]*Gx2[40] + Gx1[52]*Gx2[48] + Gx1[60]*Gx2[56];
nmpcWorkspace.H[513] += + Gx1[4]*Gx2[1] + Gx1[12]*Gx2[9] + Gx1[20]*Gx2[17] + Gx1[28]*Gx2[25] + Gx1[36]*Gx2[33] + Gx1[44]*Gx2[41] + Gx1[52]*Gx2[49] + Gx1[60]*Gx2[57];
nmpcWorkspace.H[514] += + Gx1[4]*Gx2[2] + Gx1[12]*Gx2[10] + Gx1[20]*Gx2[18] + Gx1[28]*Gx2[26] + Gx1[36]*Gx2[34] + Gx1[44]*Gx2[42] + Gx1[52]*Gx2[50] + Gx1[60]*Gx2[58];
nmpcWorkspace.H[515] += + Gx1[4]*Gx2[3] + Gx1[12]*Gx2[11] + Gx1[20]*Gx2[19] + Gx1[28]*Gx2[27] + Gx1[36]*Gx2[35] + Gx1[44]*Gx2[43] + Gx1[52]*Gx2[51] + Gx1[60]*Gx2[59];
nmpcWorkspace.H[516] += + Gx1[4]*Gx2[4] + Gx1[12]*Gx2[12] + Gx1[20]*Gx2[20] + Gx1[28]*Gx2[28] + Gx1[36]*Gx2[36] + Gx1[44]*Gx2[44] + Gx1[52]*Gx2[52] + Gx1[60]*Gx2[60];
nmpcWorkspace.H[517] += + Gx1[4]*Gx2[5] + Gx1[12]*Gx2[13] + Gx1[20]*Gx2[21] + Gx1[28]*Gx2[29] + Gx1[36]*Gx2[37] + Gx1[44]*Gx2[45] + Gx1[52]*Gx2[53] + Gx1[60]*Gx2[61];
nmpcWorkspace.H[518] += + Gx1[4]*Gx2[6] + Gx1[12]*Gx2[14] + Gx1[20]*Gx2[22] + Gx1[28]*Gx2[30] + Gx1[36]*Gx2[38] + Gx1[44]*Gx2[46] + Gx1[52]*Gx2[54] + Gx1[60]*Gx2[62];
nmpcWorkspace.H[519] += + Gx1[4]*Gx2[7] + Gx1[12]*Gx2[15] + Gx1[20]*Gx2[23] + Gx1[28]*Gx2[31] + Gx1[36]*Gx2[39] + Gx1[44]*Gx2[47] + Gx1[52]*Gx2[55] + Gx1[60]*Gx2[63];
nmpcWorkspace.H[640] += + Gx1[5]*Gx2[0] + Gx1[13]*Gx2[8] + Gx1[21]*Gx2[16] + Gx1[29]*Gx2[24] + Gx1[37]*Gx2[32] + Gx1[45]*Gx2[40] + Gx1[53]*Gx2[48] + Gx1[61]*Gx2[56];
nmpcWorkspace.H[641] += + Gx1[5]*Gx2[1] + Gx1[13]*Gx2[9] + Gx1[21]*Gx2[17] + Gx1[29]*Gx2[25] + Gx1[37]*Gx2[33] + Gx1[45]*Gx2[41] + Gx1[53]*Gx2[49] + Gx1[61]*Gx2[57];
nmpcWorkspace.H[642] += + Gx1[5]*Gx2[2] + Gx1[13]*Gx2[10] + Gx1[21]*Gx2[18] + Gx1[29]*Gx2[26] + Gx1[37]*Gx2[34] + Gx1[45]*Gx2[42] + Gx1[53]*Gx2[50] + Gx1[61]*Gx2[58];
nmpcWorkspace.H[643] += + Gx1[5]*Gx2[3] + Gx1[13]*Gx2[11] + Gx1[21]*Gx2[19] + Gx1[29]*Gx2[27] + Gx1[37]*Gx2[35] + Gx1[45]*Gx2[43] + Gx1[53]*Gx2[51] + Gx1[61]*Gx2[59];
nmpcWorkspace.H[644] += + Gx1[5]*Gx2[4] + Gx1[13]*Gx2[12] + Gx1[21]*Gx2[20] + Gx1[29]*Gx2[28] + Gx1[37]*Gx2[36] + Gx1[45]*Gx2[44] + Gx1[53]*Gx2[52] + Gx1[61]*Gx2[60];
nmpcWorkspace.H[645] += + Gx1[5]*Gx2[5] + Gx1[13]*Gx2[13] + Gx1[21]*Gx2[21] + Gx1[29]*Gx2[29] + Gx1[37]*Gx2[37] + Gx1[45]*Gx2[45] + Gx1[53]*Gx2[53] + Gx1[61]*Gx2[61];
nmpcWorkspace.H[646] += + Gx1[5]*Gx2[6] + Gx1[13]*Gx2[14] + Gx1[21]*Gx2[22] + Gx1[29]*Gx2[30] + Gx1[37]*Gx2[38] + Gx1[45]*Gx2[46] + Gx1[53]*Gx2[54] + Gx1[61]*Gx2[62];
nmpcWorkspace.H[647] += + Gx1[5]*Gx2[7] + Gx1[13]*Gx2[15] + Gx1[21]*Gx2[23] + Gx1[29]*Gx2[31] + Gx1[37]*Gx2[39] + Gx1[45]*Gx2[47] + Gx1[53]*Gx2[55] + Gx1[61]*Gx2[63];
nmpcWorkspace.H[768] += + Gx1[6]*Gx2[0] + Gx1[14]*Gx2[8] + Gx1[22]*Gx2[16] + Gx1[30]*Gx2[24] + Gx1[38]*Gx2[32] + Gx1[46]*Gx2[40] + Gx1[54]*Gx2[48] + Gx1[62]*Gx2[56];
nmpcWorkspace.H[769] += + Gx1[6]*Gx2[1] + Gx1[14]*Gx2[9] + Gx1[22]*Gx2[17] + Gx1[30]*Gx2[25] + Gx1[38]*Gx2[33] + Gx1[46]*Gx2[41] + Gx1[54]*Gx2[49] + Gx1[62]*Gx2[57];
nmpcWorkspace.H[770] += + Gx1[6]*Gx2[2] + Gx1[14]*Gx2[10] + Gx1[22]*Gx2[18] + Gx1[30]*Gx2[26] + Gx1[38]*Gx2[34] + Gx1[46]*Gx2[42] + Gx1[54]*Gx2[50] + Gx1[62]*Gx2[58];
nmpcWorkspace.H[771] += + Gx1[6]*Gx2[3] + Gx1[14]*Gx2[11] + Gx1[22]*Gx2[19] + Gx1[30]*Gx2[27] + Gx1[38]*Gx2[35] + Gx1[46]*Gx2[43] + Gx1[54]*Gx2[51] + Gx1[62]*Gx2[59];
nmpcWorkspace.H[772] += + Gx1[6]*Gx2[4] + Gx1[14]*Gx2[12] + Gx1[22]*Gx2[20] + Gx1[30]*Gx2[28] + Gx1[38]*Gx2[36] + Gx1[46]*Gx2[44] + Gx1[54]*Gx2[52] + Gx1[62]*Gx2[60];
nmpcWorkspace.H[773] += + Gx1[6]*Gx2[5] + Gx1[14]*Gx2[13] + Gx1[22]*Gx2[21] + Gx1[30]*Gx2[29] + Gx1[38]*Gx2[37] + Gx1[46]*Gx2[45] + Gx1[54]*Gx2[53] + Gx1[62]*Gx2[61];
nmpcWorkspace.H[774] += + Gx1[6]*Gx2[6] + Gx1[14]*Gx2[14] + Gx1[22]*Gx2[22] + Gx1[30]*Gx2[30] + Gx1[38]*Gx2[38] + Gx1[46]*Gx2[46] + Gx1[54]*Gx2[54] + Gx1[62]*Gx2[62];
nmpcWorkspace.H[775] += + Gx1[6]*Gx2[7] + Gx1[14]*Gx2[15] + Gx1[22]*Gx2[23] + Gx1[30]*Gx2[31] + Gx1[38]*Gx2[39] + Gx1[46]*Gx2[47] + Gx1[54]*Gx2[55] + Gx1[62]*Gx2[63];
nmpcWorkspace.H[896] += + Gx1[7]*Gx2[0] + Gx1[15]*Gx2[8] + Gx1[23]*Gx2[16] + Gx1[31]*Gx2[24] + Gx1[39]*Gx2[32] + Gx1[47]*Gx2[40] + Gx1[55]*Gx2[48] + Gx1[63]*Gx2[56];
nmpcWorkspace.H[897] += + Gx1[7]*Gx2[1] + Gx1[15]*Gx2[9] + Gx1[23]*Gx2[17] + Gx1[31]*Gx2[25] + Gx1[39]*Gx2[33] + Gx1[47]*Gx2[41] + Gx1[55]*Gx2[49] + Gx1[63]*Gx2[57];
nmpcWorkspace.H[898] += + Gx1[7]*Gx2[2] + Gx1[15]*Gx2[10] + Gx1[23]*Gx2[18] + Gx1[31]*Gx2[26] + Gx1[39]*Gx2[34] + Gx1[47]*Gx2[42] + Gx1[55]*Gx2[50] + Gx1[63]*Gx2[58];
nmpcWorkspace.H[899] += + Gx1[7]*Gx2[3] + Gx1[15]*Gx2[11] + Gx1[23]*Gx2[19] + Gx1[31]*Gx2[27] + Gx1[39]*Gx2[35] + Gx1[47]*Gx2[43] + Gx1[55]*Gx2[51] + Gx1[63]*Gx2[59];
nmpcWorkspace.H[900] += + Gx1[7]*Gx2[4] + Gx1[15]*Gx2[12] + Gx1[23]*Gx2[20] + Gx1[31]*Gx2[28] + Gx1[39]*Gx2[36] + Gx1[47]*Gx2[44] + Gx1[55]*Gx2[52] + Gx1[63]*Gx2[60];
nmpcWorkspace.H[901] += + Gx1[7]*Gx2[5] + Gx1[15]*Gx2[13] + Gx1[23]*Gx2[21] + Gx1[31]*Gx2[29] + Gx1[39]*Gx2[37] + Gx1[47]*Gx2[45] + Gx1[55]*Gx2[53] + Gx1[63]*Gx2[61];
nmpcWorkspace.H[902] += + Gx1[7]*Gx2[6] + Gx1[15]*Gx2[14] + Gx1[23]*Gx2[22] + Gx1[31]*Gx2[30] + Gx1[39]*Gx2[38] + Gx1[47]*Gx2[46] + Gx1[55]*Gx2[54] + Gx1[63]*Gx2[62];
nmpcWorkspace.H[903] += + Gx1[7]*Gx2[7] + Gx1[15]*Gx2[15] + Gx1[23]*Gx2[23] + Gx1[31]*Gx2[31] + Gx1[39]*Gx2[39] + Gx1[47]*Gx2[47] + Gx1[55]*Gx2[55] + Gx1[63]*Gx2[63];
}

void nmpc_macCTSlx( real_t* const C0, real_t* const g0 )
{
g0[0] += 0.0;
;
g0[1] += 0.0;
;
g0[2] += 0.0;
;
g0[3] += 0.0;
;
g0[4] += 0.0;
;
g0[5] += 0.0;
;
g0[6] += 0.0;
;
g0[7] += 0.0;
;
}

void nmpc_macETSlu( real_t* const E0, real_t* const g1 )
{
g1[0] += 0.0;
;
g1[1] += 0.0;
;
g1[2] += 0.0;
;
g1[3] += 0.0;
;
}

void nmpc_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
nmpc_moveGuE( nmpcWorkspace.evGu, nmpcWorkspace.E );
for (lRun1 = 1; lRun1 < 30; ++lRun1)
{
nmpc_moveGxT( &(nmpcWorkspace.evGx[ lRun1 * 64 ]), nmpcWorkspace.T );
nmpc_multGxd( &(nmpcWorkspace.d[ lRun1 * 8-8 ]), &(nmpcWorkspace.evGx[ lRun1 * 64 ]), &(nmpcWorkspace.d[ lRun1 * 8 ]) );
nmpc_multGxGx( nmpcWorkspace.T, &(nmpcWorkspace.evGx[ lRun1 * 64-64 ]), &(nmpcWorkspace.evGx[ lRun1 * 64 ]) );
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
lRun4 = (((lRun1) * (lRun1-1)) / (2)) + (lRun2);
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multGxGu( nmpcWorkspace.T, &(nmpcWorkspace.E[ lRun4 * 32 ]), &(nmpcWorkspace.E[ lRun3 * 32 ]) );
}
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_moveGuE( &(nmpcWorkspace.evGu[ lRun1 * 32 ]), &(nmpcWorkspace.E[ lRun3 * 32 ]) );
}

nmpc_multGxGx( &(nmpcWorkspace.Q1[ 64 ]), nmpcWorkspace.evGx, nmpcWorkspace.QGx );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 128 ]), &(nmpcWorkspace.evGx[ 64 ]), &(nmpcWorkspace.QGx[ 64 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 192 ]), &(nmpcWorkspace.evGx[ 128 ]), &(nmpcWorkspace.QGx[ 128 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 256 ]), &(nmpcWorkspace.evGx[ 192 ]), &(nmpcWorkspace.QGx[ 192 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 320 ]), &(nmpcWorkspace.evGx[ 256 ]), &(nmpcWorkspace.QGx[ 256 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 384 ]), &(nmpcWorkspace.evGx[ 320 ]), &(nmpcWorkspace.QGx[ 320 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 448 ]), &(nmpcWorkspace.evGx[ 384 ]), &(nmpcWorkspace.QGx[ 384 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 512 ]), &(nmpcWorkspace.evGx[ 448 ]), &(nmpcWorkspace.QGx[ 448 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 576 ]), &(nmpcWorkspace.evGx[ 512 ]), &(nmpcWorkspace.QGx[ 512 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 640 ]), &(nmpcWorkspace.evGx[ 576 ]), &(nmpcWorkspace.QGx[ 576 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 704 ]), &(nmpcWorkspace.evGx[ 640 ]), &(nmpcWorkspace.QGx[ 640 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 768 ]), &(nmpcWorkspace.evGx[ 704 ]), &(nmpcWorkspace.QGx[ 704 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 832 ]), &(nmpcWorkspace.evGx[ 768 ]), &(nmpcWorkspace.QGx[ 768 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 896 ]), &(nmpcWorkspace.evGx[ 832 ]), &(nmpcWorkspace.QGx[ 832 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 960 ]), &(nmpcWorkspace.evGx[ 896 ]), &(nmpcWorkspace.QGx[ 896 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1024 ]), &(nmpcWorkspace.evGx[ 960 ]), &(nmpcWorkspace.QGx[ 960 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1088 ]), &(nmpcWorkspace.evGx[ 1024 ]), &(nmpcWorkspace.QGx[ 1024 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1152 ]), &(nmpcWorkspace.evGx[ 1088 ]), &(nmpcWorkspace.QGx[ 1088 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1216 ]), &(nmpcWorkspace.evGx[ 1152 ]), &(nmpcWorkspace.QGx[ 1152 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1280 ]), &(nmpcWorkspace.evGx[ 1216 ]), &(nmpcWorkspace.QGx[ 1216 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1344 ]), &(nmpcWorkspace.evGx[ 1280 ]), &(nmpcWorkspace.QGx[ 1280 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1408 ]), &(nmpcWorkspace.evGx[ 1344 ]), &(nmpcWorkspace.QGx[ 1344 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1472 ]), &(nmpcWorkspace.evGx[ 1408 ]), &(nmpcWorkspace.QGx[ 1408 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1536 ]), &(nmpcWorkspace.evGx[ 1472 ]), &(nmpcWorkspace.QGx[ 1472 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1600 ]), &(nmpcWorkspace.evGx[ 1536 ]), &(nmpcWorkspace.QGx[ 1536 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1664 ]), &(nmpcWorkspace.evGx[ 1600 ]), &(nmpcWorkspace.QGx[ 1600 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1728 ]), &(nmpcWorkspace.evGx[ 1664 ]), &(nmpcWorkspace.QGx[ 1664 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1792 ]), &(nmpcWorkspace.evGx[ 1728 ]), &(nmpcWorkspace.QGx[ 1728 ]) );
nmpc_multGxGx( &(nmpcWorkspace.Q1[ 1856 ]), &(nmpcWorkspace.evGx[ 1792 ]), &(nmpcWorkspace.QGx[ 1792 ]) );
nmpc_multGxGx( nmpcWorkspace.QN1, &(nmpcWorkspace.evGx[ 1856 ]), &(nmpcWorkspace.QGx[ 1856 ]) );

for (lRun1 = 0; lRun1 < 29; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multGxGu( &(nmpcWorkspace.Q1[ lRun1 * 64 + 64 ]), &(nmpcWorkspace.E[ lRun3 * 32 ]), &(nmpcWorkspace.QE[ lRun3 * 32 ]) );
}
}

for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multGxGu( nmpcWorkspace.QN1, &(nmpcWorkspace.E[ lRun3 * 32 ]), &(nmpcWorkspace.QE[ lRun3 * 32 ]) );
}

nmpc_zeroBlockH00(  );
nmpc_multCTQC( nmpcWorkspace.evGx, nmpcWorkspace.QGx );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 64 ]), &(nmpcWorkspace.QGx[ 64 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 128 ]), &(nmpcWorkspace.QGx[ 128 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 192 ]), &(nmpcWorkspace.QGx[ 192 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 256 ]), &(nmpcWorkspace.QGx[ 256 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 320 ]), &(nmpcWorkspace.QGx[ 320 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 384 ]), &(nmpcWorkspace.QGx[ 384 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 448 ]), &(nmpcWorkspace.QGx[ 448 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 512 ]), &(nmpcWorkspace.QGx[ 512 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 576 ]), &(nmpcWorkspace.QGx[ 576 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 640 ]), &(nmpcWorkspace.QGx[ 640 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 704 ]), &(nmpcWorkspace.QGx[ 704 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 768 ]), &(nmpcWorkspace.QGx[ 768 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 832 ]), &(nmpcWorkspace.QGx[ 832 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 896 ]), &(nmpcWorkspace.QGx[ 896 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 960 ]), &(nmpcWorkspace.QGx[ 960 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1024 ]), &(nmpcWorkspace.QGx[ 1024 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1088 ]), &(nmpcWorkspace.QGx[ 1088 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1152 ]), &(nmpcWorkspace.QGx[ 1152 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1216 ]), &(nmpcWorkspace.QGx[ 1216 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1280 ]), &(nmpcWorkspace.QGx[ 1280 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1344 ]), &(nmpcWorkspace.QGx[ 1344 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1408 ]), &(nmpcWorkspace.QGx[ 1408 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1472 ]), &(nmpcWorkspace.QGx[ 1472 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1536 ]), &(nmpcWorkspace.QGx[ 1536 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1600 ]), &(nmpcWorkspace.QGx[ 1600 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1664 ]), &(nmpcWorkspace.QGx[ 1664 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1728 ]), &(nmpcWorkspace.QGx[ 1728 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1792 ]), &(nmpcWorkspace.QGx[ 1792 ]) );
nmpc_multCTQC( &(nmpcWorkspace.evGx[ 1856 ]), &(nmpcWorkspace.QGx[ 1856 ]) );

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpc_zeroBlockH10( &(nmpcWorkspace.H10[ lRun1 * 32 ]) );
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmpc_multQETGx( &(nmpcWorkspace.QE[ lRun3 * 32 ]), &(nmpcWorkspace.evGx[ lRun2 * 64 ]), &(nmpcWorkspace.H10[ lRun1 * 32 ]) );
}
}

for (lRun1 = 0;lRun1 < 8; ++lRun1)
for (lRun2 = 0;lRun2 < 120; ++lRun2)
nmpcWorkspace.H[(lRun1 * 128) + (lRun2 + 8)] = nmpcWorkspace.H10[(lRun2 * 8) + (lRun1)];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpc_setBlockH11_R1( lRun1, lRun1, &(nmpcWorkspace.R1[ lRun1 * 16 ]) );
lRun2 = lRun1;
for (lRun3 = lRun1; lRun3 < 30; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
nmpc_setBlockH11( lRun1, lRun2, &(nmpcWorkspace.E[ lRun4 * 32 ]), &(nmpcWorkspace.QE[ lRun5 * 32 ]) );
}
for (lRun2 = lRun1 + 1; lRun2 < 30; ++lRun2)
{
nmpc_zeroBlockH11( lRun1, lRun2 );
for (lRun3 = lRun2; lRun3 < 30; ++lRun3)
{
lRun4 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun1);
lRun5 = (((lRun3 + 1) * (lRun3)) / (2)) + (lRun2);
nmpc_setBlockH11( lRun1, lRun2, &(nmpcWorkspace.E[ lRun4 * 32 ]), &(nmpcWorkspace.QE[ lRun5 * 32 ]) );
}
}
}

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1; ++lRun2)
{
nmpc_copyHTH( lRun1, lRun2 );
}
}

for (lRun1 = 0;lRun1 < 120; ++lRun1)
for (lRun2 = 0;lRun2 < 8; ++lRun2)
nmpcWorkspace.H[(lRun1 * 128 + 1024) + (lRun2)] = nmpcWorkspace.H10[(lRun1 * 8) + (lRun2)];

nmpc_multQ1d( &(nmpcWorkspace.Q1[ 64 ]), nmpcWorkspace.d, nmpcWorkspace.Qd );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 128 ]), &(nmpcWorkspace.d[ 8 ]), &(nmpcWorkspace.Qd[ 8 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 192 ]), &(nmpcWorkspace.d[ 16 ]), &(nmpcWorkspace.Qd[ 16 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 256 ]), &(nmpcWorkspace.d[ 24 ]), &(nmpcWorkspace.Qd[ 24 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 320 ]), &(nmpcWorkspace.d[ 32 ]), &(nmpcWorkspace.Qd[ 32 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 384 ]), &(nmpcWorkspace.d[ 40 ]), &(nmpcWorkspace.Qd[ 40 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 448 ]), &(nmpcWorkspace.d[ 48 ]), &(nmpcWorkspace.Qd[ 48 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 512 ]), &(nmpcWorkspace.d[ 56 ]), &(nmpcWorkspace.Qd[ 56 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 576 ]), &(nmpcWorkspace.d[ 64 ]), &(nmpcWorkspace.Qd[ 64 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 640 ]), &(nmpcWorkspace.d[ 72 ]), &(nmpcWorkspace.Qd[ 72 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 704 ]), &(nmpcWorkspace.d[ 80 ]), &(nmpcWorkspace.Qd[ 80 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 768 ]), &(nmpcWorkspace.d[ 88 ]), &(nmpcWorkspace.Qd[ 88 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 832 ]), &(nmpcWorkspace.d[ 96 ]), &(nmpcWorkspace.Qd[ 96 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 896 ]), &(nmpcWorkspace.d[ 104 ]), &(nmpcWorkspace.Qd[ 104 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 960 ]), &(nmpcWorkspace.d[ 112 ]), &(nmpcWorkspace.Qd[ 112 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1024 ]), &(nmpcWorkspace.d[ 120 ]), &(nmpcWorkspace.Qd[ 120 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1088 ]), &(nmpcWorkspace.d[ 128 ]), &(nmpcWorkspace.Qd[ 128 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1152 ]), &(nmpcWorkspace.d[ 136 ]), &(nmpcWorkspace.Qd[ 136 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1216 ]), &(nmpcWorkspace.d[ 144 ]), &(nmpcWorkspace.Qd[ 144 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1280 ]), &(nmpcWorkspace.d[ 152 ]), &(nmpcWorkspace.Qd[ 152 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1344 ]), &(nmpcWorkspace.d[ 160 ]), &(nmpcWorkspace.Qd[ 160 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1408 ]), &(nmpcWorkspace.d[ 168 ]), &(nmpcWorkspace.Qd[ 168 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1472 ]), &(nmpcWorkspace.d[ 176 ]), &(nmpcWorkspace.Qd[ 176 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1536 ]), &(nmpcWorkspace.d[ 184 ]), &(nmpcWorkspace.Qd[ 184 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1600 ]), &(nmpcWorkspace.d[ 192 ]), &(nmpcWorkspace.Qd[ 192 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1664 ]), &(nmpcWorkspace.d[ 200 ]), &(nmpcWorkspace.Qd[ 200 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1728 ]), &(nmpcWorkspace.d[ 208 ]), &(nmpcWorkspace.Qd[ 208 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1792 ]), &(nmpcWorkspace.d[ 216 ]), &(nmpcWorkspace.Qd[ 216 ]) );
nmpc_multQ1d( &(nmpcWorkspace.Q1[ 1856 ]), &(nmpcWorkspace.d[ 224 ]), &(nmpcWorkspace.Qd[ 224 ]) );
nmpc_multQN1d( nmpcWorkspace.QN1, &(nmpcWorkspace.d[ 232 ]), &(nmpcWorkspace.Qd[ 232 ]) );

nmpc_macCTSlx( nmpcWorkspace.evGx, nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 64 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 128 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 192 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 256 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 320 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 384 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 448 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 512 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 576 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 640 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 704 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 768 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 832 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 896 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 960 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1024 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1088 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1152 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1216 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1280 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1344 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1408 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1472 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1536 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1600 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1664 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1728 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1792 ]), nmpcWorkspace.g );
nmpc_macCTSlx( &(nmpcWorkspace.evGx[ 1856 ]), nmpcWorkspace.g );
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmpc_macETSlu( &(nmpcWorkspace.QE[ lRun3 * 32 ]), &(nmpcWorkspace.g[ lRun1 * 4 + 8 ]) );
}
}
nmpcWorkspace.lb[8] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[0];
nmpcWorkspace.lb[9] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[1];
nmpcWorkspace.lb[10] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[2];
nmpcWorkspace.lb[11] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[3];
nmpcWorkspace.lb[12] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[4];
nmpcWorkspace.lb[13] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[5];
nmpcWorkspace.lb[14] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[6];
nmpcWorkspace.lb[15] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[7];
nmpcWorkspace.lb[16] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[8];
nmpcWorkspace.lb[17] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[9];
nmpcWorkspace.lb[18] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[10];
nmpcWorkspace.lb[19] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[11];
nmpcWorkspace.lb[20] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[12];
nmpcWorkspace.lb[21] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[13];
nmpcWorkspace.lb[22] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[14];
nmpcWorkspace.lb[23] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[15];
nmpcWorkspace.lb[24] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[16];
nmpcWorkspace.lb[25] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[17];
nmpcWorkspace.lb[26] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[18];
nmpcWorkspace.lb[27] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[19];
nmpcWorkspace.lb[28] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[20];
nmpcWorkspace.lb[29] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[21];
nmpcWorkspace.lb[30] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[22];
nmpcWorkspace.lb[31] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[23];
nmpcWorkspace.lb[32] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[24];
nmpcWorkspace.lb[33] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[25];
nmpcWorkspace.lb[34] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[26];
nmpcWorkspace.lb[35] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[27];
nmpcWorkspace.lb[36] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[28];
nmpcWorkspace.lb[37] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[29];
nmpcWorkspace.lb[38] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[30];
nmpcWorkspace.lb[39] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[31];
nmpcWorkspace.lb[40] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[32];
nmpcWorkspace.lb[41] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[33];
nmpcWorkspace.lb[42] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[34];
nmpcWorkspace.lb[43] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[35];
nmpcWorkspace.lb[44] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[36];
nmpcWorkspace.lb[45] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[37];
nmpcWorkspace.lb[46] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[38];
nmpcWorkspace.lb[47] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[39];
nmpcWorkspace.lb[48] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[40];
nmpcWorkspace.lb[49] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[41];
nmpcWorkspace.lb[50] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[42];
nmpcWorkspace.lb[51] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[43];
nmpcWorkspace.lb[52] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[44];
nmpcWorkspace.lb[53] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[45];
nmpcWorkspace.lb[54] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[46];
nmpcWorkspace.lb[55] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[47];
nmpcWorkspace.lb[56] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[48];
nmpcWorkspace.lb[57] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[49];
nmpcWorkspace.lb[58] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[50];
nmpcWorkspace.lb[59] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[51];
nmpcWorkspace.lb[60] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[52];
nmpcWorkspace.lb[61] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[53];
nmpcWorkspace.lb[62] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[54];
nmpcWorkspace.lb[63] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[55];
nmpcWorkspace.lb[64] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[56];
nmpcWorkspace.lb[65] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[57];
nmpcWorkspace.lb[66] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[58];
nmpcWorkspace.lb[67] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[59];
nmpcWorkspace.lb[68] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[60];
nmpcWorkspace.lb[69] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[61];
nmpcWorkspace.lb[70] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[62];
nmpcWorkspace.lb[71] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[63];
nmpcWorkspace.lb[72] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[64];
nmpcWorkspace.lb[73] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[65];
nmpcWorkspace.lb[74] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[66];
nmpcWorkspace.lb[75] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[67];
nmpcWorkspace.lb[76] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[68];
nmpcWorkspace.lb[77] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[69];
nmpcWorkspace.lb[78] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[70];
nmpcWorkspace.lb[79] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[71];
nmpcWorkspace.lb[80] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[72];
nmpcWorkspace.lb[81] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[73];
nmpcWorkspace.lb[82] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[74];
nmpcWorkspace.lb[83] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[75];
nmpcWorkspace.lb[84] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[76];
nmpcWorkspace.lb[85] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[77];
nmpcWorkspace.lb[86] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[78];
nmpcWorkspace.lb[87] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[79];
nmpcWorkspace.lb[88] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[80];
nmpcWorkspace.lb[89] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[81];
nmpcWorkspace.lb[90] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[82];
nmpcWorkspace.lb[91] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[83];
nmpcWorkspace.lb[92] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[84];
nmpcWorkspace.lb[93] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[85];
nmpcWorkspace.lb[94] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[86];
nmpcWorkspace.lb[95] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[87];
nmpcWorkspace.lb[96] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[88];
nmpcWorkspace.lb[97] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[89];
nmpcWorkspace.lb[98] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[90];
nmpcWorkspace.lb[99] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[91];
nmpcWorkspace.lb[100] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[92];
nmpcWorkspace.lb[101] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[93];
nmpcWorkspace.lb[102] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[94];
nmpcWorkspace.lb[103] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[95];
nmpcWorkspace.lb[104] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[96];
nmpcWorkspace.lb[105] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[97];
nmpcWorkspace.lb[106] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[98];
nmpcWorkspace.lb[107] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[99];
nmpcWorkspace.lb[108] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[100];
nmpcWorkspace.lb[109] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[101];
nmpcWorkspace.lb[110] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[102];
nmpcWorkspace.lb[111] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[103];
nmpcWorkspace.lb[112] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[104];
nmpcWorkspace.lb[113] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[105];
nmpcWorkspace.lb[114] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[106];
nmpcWorkspace.lb[115] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[107];
nmpcWorkspace.lb[116] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[108];
nmpcWorkspace.lb[117] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[109];
nmpcWorkspace.lb[118] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[110];
nmpcWorkspace.lb[119] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[111];
nmpcWorkspace.lb[120] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[112];
nmpcWorkspace.lb[121] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[113];
nmpcWorkspace.lb[122] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[114];
nmpcWorkspace.lb[123] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[115];
nmpcWorkspace.lb[124] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[116];
nmpcWorkspace.lb[125] = (real_t)-8.0000000000000000e+01 - nmpcVariables.u[117];
nmpcWorkspace.lb[126] = (real_t)-1.6000000000000000e+02 - nmpcVariables.u[118];
nmpcWorkspace.lb[127] = (real_t)-1.6000000000000000e+01 - nmpcVariables.u[119];
nmpcWorkspace.ub[8] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[0];
nmpcWorkspace.ub[9] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[1];
nmpcWorkspace.ub[10] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[2];
nmpcWorkspace.ub[11] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[3];
nmpcWorkspace.ub[12] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[4];
nmpcWorkspace.ub[13] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[5];
nmpcWorkspace.ub[14] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[6];
nmpcWorkspace.ub[15] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[7];
nmpcWorkspace.ub[16] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[8];
nmpcWorkspace.ub[17] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[9];
nmpcWorkspace.ub[18] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[10];
nmpcWorkspace.ub[19] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[11];
nmpcWorkspace.ub[20] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[12];
nmpcWorkspace.ub[21] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[13];
nmpcWorkspace.ub[22] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[14];
nmpcWorkspace.ub[23] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[15];
nmpcWorkspace.ub[24] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[16];
nmpcWorkspace.ub[25] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[17];
nmpcWorkspace.ub[26] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[18];
nmpcWorkspace.ub[27] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[19];
nmpcWorkspace.ub[28] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[20];
nmpcWorkspace.ub[29] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[21];
nmpcWorkspace.ub[30] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[22];
nmpcWorkspace.ub[31] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[23];
nmpcWorkspace.ub[32] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[24];
nmpcWorkspace.ub[33] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[25];
nmpcWorkspace.ub[34] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[26];
nmpcWorkspace.ub[35] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[27];
nmpcWorkspace.ub[36] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[28];
nmpcWorkspace.ub[37] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[29];
nmpcWorkspace.ub[38] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[30];
nmpcWorkspace.ub[39] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[31];
nmpcWorkspace.ub[40] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[32];
nmpcWorkspace.ub[41] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[33];
nmpcWorkspace.ub[42] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[34];
nmpcWorkspace.ub[43] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[35];
nmpcWorkspace.ub[44] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[36];
nmpcWorkspace.ub[45] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[37];
nmpcWorkspace.ub[46] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[38];
nmpcWorkspace.ub[47] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[39];
nmpcWorkspace.ub[48] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[40];
nmpcWorkspace.ub[49] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[41];
nmpcWorkspace.ub[50] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[42];
nmpcWorkspace.ub[51] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[43];
nmpcWorkspace.ub[52] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[44];
nmpcWorkspace.ub[53] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[45];
nmpcWorkspace.ub[54] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[46];
nmpcWorkspace.ub[55] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[47];
nmpcWorkspace.ub[56] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[48];
nmpcWorkspace.ub[57] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[49];
nmpcWorkspace.ub[58] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[50];
nmpcWorkspace.ub[59] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[51];
nmpcWorkspace.ub[60] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[52];
nmpcWorkspace.ub[61] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[53];
nmpcWorkspace.ub[62] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[54];
nmpcWorkspace.ub[63] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[55];
nmpcWorkspace.ub[64] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[56];
nmpcWorkspace.ub[65] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[57];
nmpcWorkspace.ub[66] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[58];
nmpcWorkspace.ub[67] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[59];
nmpcWorkspace.ub[68] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[60];
nmpcWorkspace.ub[69] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[61];
nmpcWorkspace.ub[70] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[62];
nmpcWorkspace.ub[71] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[63];
nmpcWorkspace.ub[72] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[64];
nmpcWorkspace.ub[73] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[65];
nmpcWorkspace.ub[74] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[66];
nmpcWorkspace.ub[75] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[67];
nmpcWorkspace.ub[76] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[68];
nmpcWorkspace.ub[77] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[69];
nmpcWorkspace.ub[78] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[70];
nmpcWorkspace.ub[79] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[71];
nmpcWorkspace.ub[80] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[72];
nmpcWorkspace.ub[81] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[73];
nmpcWorkspace.ub[82] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[74];
nmpcWorkspace.ub[83] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[75];
nmpcWorkspace.ub[84] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[76];
nmpcWorkspace.ub[85] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[77];
nmpcWorkspace.ub[86] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[78];
nmpcWorkspace.ub[87] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[79];
nmpcWorkspace.ub[88] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[80];
nmpcWorkspace.ub[89] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[81];
nmpcWorkspace.ub[90] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[82];
nmpcWorkspace.ub[91] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[83];
nmpcWorkspace.ub[92] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[84];
nmpcWorkspace.ub[93] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[85];
nmpcWorkspace.ub[94] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[86];
nmpcWorkspace.ub[95] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[87];
nmpcWorkspace.ub[96] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[88];
nmpcWorkspace.ub[97] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[89];
nmpcWorkspace.ub[98] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[90];
nmpcWorkspace.ub[99] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[91];
nmpcWorkspace.ub[100] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[92];
nmpcWorkspace.ub[101] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[93];
nmpcWorkspace.ub[102] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[94];
nmpcWorkspace.ub[103] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[95];
nmpcWorkspace.ub[104] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[96];
nmpcWorkspace.ub[105] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[97];
nmpcWorkspace.ub[106] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[98];
nmpcWorkspace.ub[107] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[99];
nmpcWorkspace.ub[108] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[100];
nmpcWorkspace.ub[109] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[101];
nmpcWorkspace.ub[110] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[102];
nmpcWorkspace.ub[111] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[103];
nmpcWorkspace.ub[112] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[104];
nmpcWorkspace.ub[113] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[105];
nmpcWorkspace.ub[114] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[106];
nmpcWorkspace.ub[115] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[107];
nmpcWorkspace.ub[116] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[108];
nmpcWorkspace.ub[117] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[109];
nmpcWorkspace.ub[118] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[110];
nmpcWorkspace.ub[119] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[111];
nmpcWorkspace.ub[120] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[112];
nmpcWorkspace.ub[121] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[113];
nmpcWorkspace.ub[122] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[114];
nmpcWorkspace.ub[123] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[115];
nmpcWorkspace.ub[124] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[116];
nmpcWorkspace.ub[125] = (real_t)8.0000000000000000e+01 - nmpcVariables.u[117];
nmpcWorkspace.ub[126] = (real_t)1.6000000000000000e+02 - nmpcVariables.u[118];
nmpcWorkspace.ub[127] = (real_t)1.6000000000000000e+01 - nmpcVariables.u[119];

}

void nmpc_condenseFdb(  )
{
int lRun1;
int lRun2;
int lRun3;
nmpcWorkspace.Dx0[0] = nmpcVariables.x0[0] - nmpcVariables.x[0];
nmpcWorkspace.Dx0[1] = nmpcVariables.x0[1] - nmpcVariables.x[1];
nmpcWorkspace.Dx0[2] = nmpcVariables.x0[2] - nmpcVariables.x[2];
nmpcWorkspace.Dx0[3] = nmpcVariables.x0[3] - nmpcVariables.x[3];
nmpcWorkspace.Dx0[4] = nmpcVariables.x0[4] - nmpcVariables.x[4];
nmpcWorkspace.Dx0[5] = nmpcVariables.x0[5] - nmpcVariables.x[5];
nmpcWorkspace.Dx0[6] = nmpcVariables.x0[6] - nmpcVariables.x[6];
nmpcWorkspace.Dx0[7] = nmpcVariables.x0[7] - nmpcVariables.x[7];

for (lRun2 = 0; lRun2 < 360; ++lRun2)
nmpcWorkspace.Dy[lRun2] -= nmpcVariables.y[lRun2];

nmpcWorkspace.DyN[0] -= nmpcVariables.yN[0];
nmpcWorkspace.DyN[1] -= nmpcVariables.yN[1];
nmpcWorkspace.DyN[2] -= nmpcVariables.yN[2];
nmpcWorkspace.DyN[3] -= nmpcVariables.yN[3];
nmpcWorkspace.DyN[4] -= nmpcVariables.yN[4];
nmpcWorkspace.DyN[5] -= nmpcVariables.yN[5];
nmpcWorkspace.DyN[6] -= nmpcVariables.yN[6];
nmpcWorkspace.DyN[7] -= nmpcVariables.yN[7];

nmpc_multRDy( nmpcWorkspace.R2, nmpcWorkspace.Dy, &(nmpcWorkspace.g[ 8 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 48 ]), &(nmpcWorkspace.Dy[ 12 ]), &(nmpcWorkspace.g[ 12 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 96 ]), &(nmpcWorkspace.Dy[ 24 ]), &(nmpcWorkspace.g[ 16 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 144 ]), &(nmpcWorkspace.Dy[ 36 ]), &(nmpcWorkspace.g[ 20 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 192 ]), &(nmpcWorkspace.Dy[ 48 ]), &(nmpcWorkspace.g[ 24 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 240 ]), &(nmpcWorkspace.Dy[ 60 ]), &(nmpcWorkspace.g[ 28 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 288 ]), &(nmpcWorkspace.Dy[ 72 ]), &(nmpcWorkspace.g[ 32 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 336 ]), &(nmpcWorkspace.Dy[ 84 ]), &(nmpcWorkspace.g[ 36 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 384 ]), &(nmpcWorkspace.Dy[ 96 ]), &(nmpcWorkspace.g[ 40 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 432 ]), &(nmpcWorkspace.Dy[ 108 ]), &(nmpcWorkspace.g[ 44 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 480 ]), &(nmpcWorkspace.Dy[ 120 ]), &(nmpcWorkspace.g[ 48 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 528 ]), &(nmpcWorkspace.Dy[ 132 ]), &(nmpcWorkspace.g[ 52 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 576 ]), &(nmpcWorkspace.Dy[ 144 ]), &(nmpcWorkspace.g[ 56 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 624 ]), &(nmpcWorkspace.Dy[ 156 ]), &(nmpcWorkspace.g[ 60 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 672 ]), &(nmpcWorkspace.Dy[ 168 ]), &(nmpcWorkspace.g[ 64 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 720 ]), &(nmpcWorkspace.Dy[ 180 ]), &(nmpcWorkspace.g[ 68 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 768 ]), &(nmpcWorkspace.Dy[ 192 ]), &(nmpcWorkspace.g[ 72 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 816 ]), &(nmpcWorkspace.Dy[ 204 ]), &(nmpcWorkspace.g[ 76 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 864 ]), &(nmpcWorkspace.Dy[ 216 ]), &(nmpcWorkspace.g[ 80 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 912 ]), &(nmpcWorkspace.Dy[ 228 ]), &(nmpcWorkspace.g[ 84 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 960 ]), &(nmpcWorkspace.Dy[ 240 ]), &(nmpcWorkspace.g[ 88 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1008 ]), &(nmpcWorkspace.Dy[ 252 ]), &(nmpcWorkspace.g[ 92 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1056 ]), &(nmpcWorkspace.Dy[ 264 ]), &(nmpcWorkspace.g[ 96 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1104 ]), &(nmpcWorkspace.Dy[ 276 ]), &(nmpcWorkspace.g[ 100 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1152 ]), &(nmpcWorkspace.Dy[ 288 ]), &(nmpcWorkspace.g[ 104 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1200 ]), &(nmpcWorkspace.Dy[ 300 ]), &(nmpcWorkspace.g[ 108 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1248 ]), &(nmpcWorkspace.Dy[ 312 ]), &(nmpcWorkspace.g[ 112 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1296 ]), &(nmpcWorkspace.Dy[ 324 ]), &(nmpcWorkspace.g[ 116 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1344 ]), &(nmpcWorkspace.Dy[ 336 ]), &(nmpcWorkspace.g[ 120 ]) );
nmpc_multRDy( &(nmpcWorkspace.R2[ 1392 ]), &(nmpcWorkspace.Dy[ 348 ]), &(nmpcWorkspace.g[ 124 ]) );

nmpc_multQDy( nmpcWorkspace.Q2, nmpcWorkspace.Dy, nmpcWorkspace.QDy );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 96 ]), &(nmpcWorkspace.Dy[ 12 ]), &(nmpcWorkspace.QDy[ 8 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 192 ]), &(nmpcWorkspace.Dy[ 24 ]), &(nmpcWorkspace.QDy[ 16 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 288 ]), &(nmpcWorkspace.Dy[ 36 ]), &(nmpcWorkspace.QDy[ 24 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 384 ]), &(nmpcWorkspace.Dy[ 48 ]), &(nmpcWorkspace.QDy[ 32 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 480 ]), &(nmpcWorkspace.Dy[ 60 ]), &(nmpcWorkspace.QDy[ 40 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 576 ]), &(nmpcWorkspace.Dy[ 72 ]), &(nmpcWorkspace.QDy[ 48 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 672 ]), &(nmpcWorkspace.Dy[ 84 ]), &(nmpcWorkspace.QDy[ 56 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 768 ]), &(nmpcWorkspace.Dy[ 96 ]), &(nmpcWorkspace.QDy[ 64 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 864 ]), &(nmpcWorkspace.Dy[ 108 ]), &(nmpcWorkspace.QDy[ 72 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 960 ]), &(nmpcWorkspace.Dy[ 120 ]), &(nmpcWorkspace.QDy[ 80 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1056 ]), &(nmpcWorkspace.Dy[ 132 ]), &(nmpcWorkspace.QDy[ 88 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1152 ]), &(nmpcWorkspace.Dy[ 144 ]), &(nmpcWorkspace.QDy[ 96 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1248 ]), &(nmpcWorkspace.Dy[ 156 ]), &(nmpcWorkspace.QDy[ 104 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1344 ]), &(nmpcWorkspace.Dy[ 168 ]), &(nmpcWorkspace.QDy[ 112 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1440 ]), &(nmpcWorkspace.Dy[ 180 ]), &(nmpcWorkspace.QDy[ 120 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1536 ]), &(nmpcWorkspace.Dy[ 192 ]), &(nmpcWorkspace.QDy[ 128 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1632 ]), &(nmpcWorkspace.Dy[ 204 ]), &(nmpcWorkspace.QDy[ 136 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1728 ]), &(nmpcWorkspace.Dy[ 216 ]), &(nmpcWorkspace.QDy[ 144 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1824 ]), &(nmpcWorkspace.Dy[ 228 ]), &(nmpcWorkspace.QDy[ 152 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 1920 ]), &(nmpcWorkspace.Dy[ 240 ]), &(nmpcWorkspace.QDy[ 160 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2016 ]), &(nmpcWorkspace.Dy[ 252 ]), &(nmpcWorkspace.QDy[ 168 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2112 ]), &(nmpcWorkspace.Dy[ 264 ]), &(nmpcWorkspace.QDy[ 176 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2208 ]), &(nmpcWorkspace.Dy[ 276 ]), &(nmpcWorkspace.QDy[ 184 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2304 ]), &(nmpcWorkspace.Dy[ 288 ]), &(nmpcWorkspace.QDy[ 192 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2400 ]), &(nmpcWorkspace.Dy[ 300 ]), &(nmpcWorkspace.QDy[ 200 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2496 ]), &(nmpcWorkspace.Dy[ 312 ]), &(nmpcWorkspace.QDy[ 208 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2592 ]), &(nmpcWorkspace.Dy[ 324 ]), &(nmpcWorkspace.QDy[ 216 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2688 ]), &(nmpcWorkspace.Dy[ 336 ]), &(nmpcWorkspace.QDy[ 224 ]) );
nmpc_multQDy( &(nmpcWorkspace.Q2[ 2784 ]), &(nmpcWorkspace.Dy[ 348 ]), &(nmpcWorkspace.QDy[ 232 ]) );

nmpcWorkspace.QDy[240] = + nmpcWorkspace.QN2[0]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[1]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[2]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[3]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[4]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[5]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[6]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[7]*nmpcWorkspace.DyN[7];
nmpcWorkspace.QDy[241] = + nmpcWorkspace.QN2[8]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[9]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[10]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[11]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[12]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[13]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[14]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[15]*nmpcWorkspace.DyN[7];
nmpcWorkspace.QDy[242] = + nmpcWorkspace.QN2[16]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[17]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[18]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[19]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[20]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[21]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[22]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[23]*nmpcWorkspace.DyN[7];
nmpcWorkspace.QDy[243] = + nmpcWorkspace.QN2[24]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[25]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[26]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[27]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[28]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[29]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[30]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[31]*nmpcWorkspace.DyN[7];
nmpcWorkspace.QDy[244] = + nmpcWorkspace.QN2[32]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[33]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[34]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[35]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[36]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[37]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[38]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[39]*nmpcWorkspace.DyN[7];
nmpcWorkspace.QDy[245] = + nmpcWorkspace.QN2[40]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[41]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[42]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[43]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[44]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[45]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[46]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[47]*nmpcWorkspace.DyN[7];
nmpcWorkspace.QDy[246] = + nmpcWorkspace.QN2[48]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[49]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[50]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[51]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[52]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[53]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[54]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[55]*nmpcWorkspace.DyN[7];
nmpcWorkspace.QDy[247] = + nmpcWorkspace.QN2[56]*nmpcWorkspace.DyN[0] + nmpcWorkspace.QN2[57]*nmpcWorkspace.DyN[1] + nmpcWorkspace.QN2[58]*nmpcWorkspace.DyN[2] + nmpcWorkspace.QN2[59]*nmpcWorkspace.DyN[3] + nmpcWorkspace.QN2[60]*nmpcWorkspace.DyN[4] + nmpcWorkspace.QN2[61]*nmpcWorkspace.DyN[5] + nmpcWorkspace.QN2[62]*nmpcWorkspace.DyN[6] + nmpcWorkspace.QN2[63]*nmpcWorkspace.DyN[7];

for (lRun2 = 0; lRun2 < 240; ++lRun2)
nmpcWorkspace.QDy[lRun2 + 8] += nmpcWorkspace.Qd[lRun2];


nmpcWorkspace.g[0] = + nmpcWorkspace.evGx[0]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[8]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[16]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[24]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[32]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[40]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[48]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[56]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[64]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[72]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[80]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[88]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[96]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[104]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[112]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[120]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[128]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[136]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[144]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[152]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[160]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[168]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[176]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[184]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[192]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[200]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[208]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[216]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[224]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[232]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[240]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[248]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[256]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[264]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[272]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[280]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[288]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[296]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[304]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[312]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[320]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[328]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[336]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[344]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[352]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[360]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[368]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[376]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[384]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[392]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[400]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[408]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[416]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[424]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[432]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[440]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[448]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[456]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[464]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[472]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[480]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[488]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[496]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[504]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[512]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[520]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[528]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[536]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[544]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[552]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[560]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[568]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[576]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[584]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[592]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[600]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[608]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[616]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[624]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[632]*nmpcWorkspace.QDy[87] + nmpcWorkspace.evGx[640]*nmpcWorkspace.QDy[88] + nmpcWorkspace.evGx[648]*nmpcWorkspace.QDy[89] + nmpcWorkspace.evGx[656]*nmpcWorkspace.QDy[90] + nmpcWorkspace.evGx[664]*nmpcWorkspace.QDy[91] + nmpcWorkspace.evGx[672]*nmpcWorkspace.QDy[92] + nmpcWorkspace.evGx[680]*nmpcWorkspace.QDy[93] + nmpcWorkspace.evGx[688]*nmpcWorkspace.QDy[94] + nmpcWorkspace.evGx[696]*nmpcWorkspace.QDy[95] + nmpcWorkspace.evGx[704]*nmpcWorkspace.QDy[96] + nmpcWorkspace.evGx[712]*nmpcWorkspace.QDy[97] + nmpcWorkspace.evGx[720]*nmpcWorkspace.QDy[98] + nmpcWorkspace.evGx[728]*nmpcWorkspace.QDy[99] + nmpcWorkspace.evGx[736]*nmpcWorkspace.QDy[100] + nmpcWorkspace.evGx[744]*nmpcWorkspace.QDy[101] + nmpcWorkspace.evGx[752]*nmpcWorkspace.QDy[102] + nmpcWorkspace.evGx[760]*nmpcWorkspace.QDy[103] + nmpcWorkspace.evGx[768]*nmpcWorkspace.QDy[104] + nmpcWorkspace.evGx[776]*nmpcWorkspace.QDy[105] + nmpcWorkspace.evGx[784]*nmpcWorkspace.QDy[106] + nmpcWorkspace.evGx[792]*nmpcWorkspace.QDy[107] + nmpcWorkspace.evGx[800]*nmpcWorkspace.QDy[108] + nmpcWorkspace.evGx[808]*nmpcWorkspace.QDy[109] + nmpcWorkspace.evGx[816]*nmpcWorkspace.QDy[110] + nmpcWorkspace.evGx[824]*nmpcWorkspace.QDy[111] + nmpcWorkspace.evGx[832]*nmpcWorkspace.QDy[112] + nmpcWorkspace.evGx[840]*nmpcWorkspace.QDy[113] + nmpcWorkspace.evGx[848]*nmpcWorkspace.QDy[114] + nmpcWorkspace.evGx[856]*nmpcWorkspace.QDy[115] + nmpcWorkspace.evGx[864]*nmpcWorkspace.QDy[116] + nmpcWorkspace.evGx[872]*nmpcWorkspace.QDy[117] + nmpcWorkspace.evGx[880]*nmpcWorkspace.QDy[118] + nmpcWorkspace.evGx[888]*nmpcWorkspace.QDy[119] + nmpcWorkspace.evGx[896]*nmpcWorkspace.QDy[120] + nmpcWorkspace.evGx[904]*nmpcWorkspace.QDy[121] + nmpcWorkspace.evGx[912]*nmpcWorkspace.QDy[122] + nmpcWorkspace.evGx[920]*nmpcWorkspace.QDy[123] + nmpcWorkspace.evGx[928]*nmpcWorkspace.QDy[124] + nmpcWorkspace.evGx[936]*nmpcWorkspace.QDy[125] + nmpcWorkspace.evGx[944]*nmpcWorkspace.QDy[126] + nmpcWorkspace.evGx[952]*nmpcWorkspace.QDy[127] + nmpcWorkspace.evGx[960]*nmpcWorkspace.QDy[128] + nmpcWorkspace.evGx[968]*nmpcWorkspace.QDy[129] + nmpcWorkspace.evGx[976]*nmpcWorkspace.QDy[130] + nmpcWorkspace.evGx[984]*nmpcWorkspace.QDy[131] + nmpcWorkspace.evGx[992]*nmpcWorkspace.QDy[132] + nmpcWorkspace.evGx[1000]*nmpcWorkspace.QDy[133] + nmpcWorkspace.evGx[1008]*nmpcWorkspace.QDy[134] + nmpcWorkspace.evGx[1016]*nmpcWorkspace.QDy[135] + nmpcWorkspace.evGx[1024]*nmpcWorkspace.QDy[136] + nmpcWorkspace.evGx[1032]*nmpcWorkspace.QDy[137] + nmpcWorkspace.evGx[1040]*nmpcWorkspace.QDy[138] + nmpcWorkspace.evGx[1048]*nmpcWorkspace.QDy[139] + nmpcWorkspace.evGx[1056]*nmpcWorkspace.QDy[140] + nmpcWorkspace.evGx[1064]*nmpcWorkspace.QDy[141] + nmpcWorkspace.evGx[1072]*nmpcWorkspace.QDy[142] + nmpcWorkspace.evGx[1080]*nmpcWorkspace.QDy[143] + nmpcWorkspace.evGx[1088]*nmpcWorkspace.QDy[144] + nmpcWorkspace.evGx[1096]*nmpcWorkspace.QDy[145] + nmpcWorkspace.evGx[1104]*nmpcWorkspace.QDy[146] + nmpcWorkspace.evGx[1112]*nmpcWorkspace.QDy[147] + nmpcWorkspace.evGx[1120]*nmpcWorkspace.QDy[148] + nmpcWorkspace.evGx[1128]*nmpcWorkspace.QDy[149] + nmpcWorkspace.evGx[1136]*nmpcWorkspace.QDy[150] + nmpcWorkspace.evGx[1144]*nmpcWorkspace.QDy[151] + nmpcWorkspace.evGx[1152]*nmpcWorkspace.QDy[152] + nmpcWorkspace.evGx[1160]*nmpcWorkspace.QDy[153] + nmpcWorkspace.evGx[1168]*nmpcWorkspace.QDy[154] + nmpcWorkspace.evGx[1176]*nmpcWorkspace.QDy[155] + nmpcWorkspace.evGx[1184]*nmpcWorkspace.QDy[156] + nmpcWorkspace.evGx[1192]*nmpcWorkspace.QDy[157] + nmpcWorkspace.evGx[1200]*nmpcWorkspace.QDy[158] + nmpcWorkspace.evGx[1208]*nmpcWorkspace.QDy[159] + nmpcWorkspace.evGx[1216]*nmpcWorkspace.QDy[160] + nmpcWorkspace.evGx[1224]*nmpcWorkspace.QDy[161] + nmpcWorkspace.evGx[1232]*nmpcWorkspace.QDy[162] + nmpcWorkspace.evGx[1240]*nmpcWorkspace.QDy[163] + nmpcWorkspace.evGx[1248]*nmpcWorkspace.QDy[164] + nmpcWorkspace.evGx[1256]*nmpcWorkspace.QDy[165] + nmpcWorkspace.evGx[1264]*nmpcWorkspace.QDy[166] + nmpcWorkspace.evGx[1272]*nmpcWorkspace.QDy[167] + nmpcWorkspace.evGx[1280]*nmpcWorkspace.QDy[168] + nmpcWorkspace.evGx[1288]*nmpcWorkspace.QDy[169] + nmpcWorkspace.evGx[1296]*nmpcWorkspace.QDy[170] + nmpcWorkspace.evGx[1304]*nmpcWorkspace.QDy[171] + nmpcWorkspace.evGx[1312]*nmpcWorkspace.QDy[172] + nmpcWorkspace.evGx[1320]*nmpcWorkspace.QDy[173] + nmpcWorkspace.evGx[1328]*nmpcWorkspace.QDy[174] + nmpcWorkspace.evGx[1336]*nmpcWorkspace.QDy[175] + nmpcWorkspace.evGx[1344]*nmpcWorkspace.QDy[176] + nmpcWorkspace.evGx[1352]*nmpcWorkspace.QDy[177] + nmpcWorkspace.evGx[1360]*nmpcWorkspace.QDy[178] + nmpcWorkspace.evGx[1368]*nmpcWorkspace.QDy[179] + nmpcWorkspace.evGx[1376]*nmpcWorkspace.QDy[180] + nmpcWorkspace.evGx[1384]*nmpcWorkspace.QDy[181] + nmpcWorkspace.evGx[1392]*nmpcWorkspace.QDy[182] + nmpcWorkspace.evGx[1400]*nmpcWorkspace.QDy[183] + nmpcWorkspace.evGx[1408]*nmpcWorkspace.QDy[184] + nmpcWorkspace.evGx[1416]*nmpcWorkspace.QDy[185] + nmpcWorkspace.evGx[1424]*nmpcWorkspace.QDy[186] + nmpcWorkspace.evGx[1432]*nmpcWorkspace.QDy[187] + nmpcWorkspace.evGx[1440]*nmpcWorkspace.QDy[188] + nmpcWorkspace.evGx[1448]*nmpcWorkspace.QDy[189] + nmpcWorkspace.evGx[1456]*nmpcWorkspace.QDy[190] + nmpcWorkspace.evGx[1464]*nmpcWorkspace.QDy[191] + nmpcWorkspace.evGx[1472]*nmpcWorkspace.QDy[192] + nmpcWorkspace.evGx[1480]*nmpcWorkspace.QDy[193] + nmpcWorkspace.evGx[1488]*nmpcWorkspace.QDy[194] + nmpcWorkspace.evGx[1496]*nmpcWorkspace.QDy[195] + nmpcWorkspace.evGx[1504]*nmpcWorkspace.QDy[196] + nmpcWorkspace.evGx[1512]*nmpcWorkspace.QDy[197] + nmpcWorkspace.evGx[1520]*nmpcWorkspace.QDy[198] + nmpcWorkspace.evGx[1528]*nmpcWorkspace.QDy[199] + nmpcWorkspace.evGx[1536]*nmpcWorkspace.QDy[200] + nmpcWorkspace.evGx[1544]*nmpcWorkspace.QDy[201] + nmpcWorkspace.evGx[1552]*nmpcWorkspace.QDy[202] + nmpcWorkspace.evGx[1560]*nmpcWorkspace.QDy[203] + nmpcWorkspace.evGx[1568]*nmpcWorkspace.QDy[204] + nmpcWorkspace.evGx[1576]*nmpcWorkspace.QDy[205] + nmpcWorkspace.evGx[1584]*nmpcWorkspace.QDy[206] + nmpcWorkspace.evGx[1592]*nmpcWorkspace.QDy[207] + nmpcWorkspace.evGx[1600]*nmpcWorkspace.QDy[208] + nmpcWorkspace.evGx[1608]*nmpcWorkspace.QDy[209] + nmpcWorkspace.evGx[1616]*nmpcWorkspace.QDy[210] + nmpcWorkspace.evGx[1624]*nmpcWorkspace.QDy[211] + nmpcWorkspace.evGx[1632]*nmpcWorkspace.QDy[212] + nmpcWorkspace.evGx[1640]*nmpcWorkspace.QDy[213] + nmpcWorkspace.evGx[1648]*nmpcWorkspace.QDy[214] + nmpcWorkspace.evGx[1656]*nmpcWorkspace.QDy[215] + nmpcWorkspace.evGx[1664]*nmpcWorkspace.QDy[216] + nmpcWorkspace.evGx[1672]*nmpcWorkspace.QDy[217] + nmpcWorkspace.evGx[1680]*nmpcWorkspace.QDy[218] + nmpcWorkspace.evGx[1688]*nmpcWorkspace.QDy[219] + nmpcWorkspace.evGx[1696]*nmpcWorkspace.QDy[220] + nmpcWorkspace.evGx[1704]*nmpcWorkspace.QDy[221] + nmpcWorkspace.evGx[1712]*nmpcWorkspace.QDy[222] + nmpcWorkspace.evGx[1720]*nmpcWorkspace.QDy[223] + nmpcWorkspace.evGx[1728]*nmpcWorkspace.QDy[224] + nmpcWorkspace.evGx[1736]*nmpcWorkspace.QDy[225] + nmpcWorkspace.evGx[1744]*nmpcWorkspace.QDy[226] + nmpcWorkspace.evGx[1752]*nmpcWorkspace.QDy[227] + nmpcWorkspace.evGx[1760]*nmpcWorkspace.QDy[228] + nmpcWorkspace.evGx[1768]*nmpcWorkspace.QDy[229] + nmpcWorkspace.evGx[1776]*nmpcWorkspace.QDy[230] + nmpcWorkspace.evGx[1784]*nmpcWorkspace.QDy[231] + nmpcWorkspace.evGx[1792]*nmpcWorkspace.QDy[232] + nmpcWorkspace.evGx[1800]*nmpcWorkspace.QDy[233] + nmpcWorkspace.evGx[1808]*nmpcWorkspace.QDy[234] + nmpcWorkspace.evGx[1816]*nmpcWorkspace.QDy[235] + nmpcWorkspace.evGx[1824]*nmpcWorkspace.QDy[236] + nmpcWorkspace.evGx[1832]*nmpcWorkspace.QDy[237] + nmpcWorkspace.evGx[1840]*nmpcWorkspace.QDy[238] + nmpcWorkspace.evGx[1848]*nmpcWorkspace.QDy[239] + nmpcWorkspace.evGx[1856]*nmpcWorkspace.QDy[240] + nmpcWorkspace.evGx[1864]*nmpcWorkspace.QDy[241] + nmpcWorkspace.evGx[1872]*nmpcWorkspace.QDy[242] + nmpcWorkspace.evGx[1880]*nmpcWorkspace.QDy[243] + nmpcWorkspace.evGx[1888]*nmpcWorkspace.QDy[244] + nmpcWorkspace.evGx[1896]*nmpcWorkspace.QDy[245] + nmpcWorkspace.evGx[1904]*nmpcWorkspace.QDy[246] + nmpcWorkspace.evGx[1912]*nmpcWorkspace.QDy[247];
nmpcWorkspace.g[1] = + nmpcWorkspace.evGx[1]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[9]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[17]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[25]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[33]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[41]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[49]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[57]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[65]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[73]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[81]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[89]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[97]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[105]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[113]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[121]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[129]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[137]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[145]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[153]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[161]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[169]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[177]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[185]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[193]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[201]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[209]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[217]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[225]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[233]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[241]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[249]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[257]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[265]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[273]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[281]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[289]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[297]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[305]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[313]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[321]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[329]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[337]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[345]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[353]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[361]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[369]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[377]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[385]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[393]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[401]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[409]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[417]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[425]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[433]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[441]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[449]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[457]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[465]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[473]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[481]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[489]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[497]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[505]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[513]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[521]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[529]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[537]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[545]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[553]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[561]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[569]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[577]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[585]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[593]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[601]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[609]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[617]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[625]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[633]*nmpcWorkspace.QDy[87] + nmpcWorkspace.evGx[641]*nmpcWorkspace.QDy[88] + nmpcWorkspace.evGx[649]*nmpcWorkspace.QDy[89] + nmpcWorkspace.evGx[657]*nmpcWorkspace.QDy[90] + nmpcWorkspace.evGx[665]*nmpcWorkspace.QDy[91] + nmpcWorkspace.evGx[673]*nmpcWorkspace.QDy[92] + nmpcWorkspace.evGx[681]*nmpcWorkspace.QDy[93] + nmpcWorkspace.evGx[689]*nmpcWorkspace.QDy[94] + nmpcWorkspace.evGx[697]*nmpcWorkspace.QDy[95] + nmpcWorkspace.evGx[705]*nmpcWorkspace.QDy[96] + nmpcWorkspace.evGx[713]*nmpcWorkspace.QDy[97] + nmpcWorkspace.evGx[721]*nmpcWorkspace.QDy[98] + nmpcWorkspace.evGx[729]*nmpcWorkspace.QDy[99] + nmpcWorkspace.evGx[737]*nmpcWorkspace.QDy[100] + nmpcWorkspace.evGx[745]*nmpcWorkspace.QDy[101] + nmpcWorkspace.evGx[753]*nmpcWorkspace.QDy[102] + nmpcWorkspace.evGx[761]*nmpcWorkspace.QDy[103] + nmpcWorkspace.evGx[769]*nmpcWorkspace.QDy[104] + nmpcWorkspace.evGx[777]*nmpcWorkspace.QDy[105] + nmpcWorkspace.evGx[785]*nmpcWorkspace.QDy[106] + nmpcWorkspace.evGx[793]*nmpcWorkspace.QDy[107] + nmpcWorkspace.evGx[801]*nmpcWorkspace.QDy[108] + nmpcWorkspace.evGx[809]*nmpcWorkspace.QDy[109] + nmpcWorkspace.evGx[817]*nmpcWorkspace.QDy[110] + nmpcWorkspace.evGx[825]*nmpcWorkspace.QDy[111] + nmpcWorkspace.evGx[833]*nmpcWorkspace.QDy[112] + nmpcWorkspace.evGx[841]*nmpcWorkspace.QDy[113] + nmpcWorkspace.evGx[849]*nmpcWorkspace.QDy[114] + nmpcWorkspace.evGx[857]*nmpcWorkspace.QDy[115] + nmpcWorkspace.evGx[865]*nmpcWorkspace.QDy[116] + nmpcWorkspace.evGx[873]*nmpcWorkspace.QDy[117] + nmpcWorkspace.evGx[881]*nmpcWorkspace.QDy[118] + nmpcWorkspace.evGx[889]*nmpcWorkspace.QDy[119] + nmpcWorkspace.evGx[897]*nmpcWorkspace.QDy[120] + nmpcWorkspace.evGx[905]*nmpcWorkspace.QDy[121] + nmpcWorkspace.evGx[913]*nmpcWorkspace.QDy[122] + nmpcWorkspace.evGx[921]*nmpcWorkspace.QDy[123] + nmpcWorkspace.evGx[929]*nmpcWorkspace.QDy[124] + nmpcWorkspace.evGx[937]*nmpcWorkspace.QDy[125] + nmpcWorkspace.evGx[945]*nmpcWorkspace.QDy[126] + nmpcWorkspace.evGx[953]*nmpcWorkspace.QDy[127] + nmpcWorkspace.evGx[961]*nmpcWorkspace.QDy[128] + nmpcWorkspace.evGx[969]*nmpcWorkspace.QDy[129] + nmpcWorkspace.evGx[977]*nmpcWorkspace.QDy[130] + nmpcWorkspace.evGx[985]*nmpcWorkspace.QDy[131] + nmpcWorkspace.evGx[993]*nmpcWorkspace.QDy[132] + nmpcWorkspace.evGx[1001]*nmpcWorkspace.QDy[133] + nmpcWorkspace.evGx[1009]*nmpcWorkspace.QDy[134] + nmpcWorkspace.evGx[1017]*nmpcWorkspace.QDy[135] + nmpcWorkspace.evGx[1025]*nmpcWorkspace.QDy[136] + nmpcWorkspace.evGx[1033]*nmpcWorkspace.QDy[137] + nmpcWorkspace.evGx[1041]*nmpcWorkspace.QDy[138] + nmpcWorkspace.evGx[1049]*nmpcWorkspace.QDy[139] + nmpcWorkspace.evGx[1057]*nmpcWorkspace.QDy[140] + nmpcWorkspace.evGx[1065]*nmpcWorkspace.QDy[141] + nmpcWorkspace.evGx[1073]*nmpcWorkspace.QDy[142] + nmpcWorkspace.evGx[1081]*nmpcWorkspace.QDy[143] + nmpcWorkspace.evGx[1089]*nmpcWorkspace.QDy[144] + nmpcWorkspace.evGx[1097]*nmpcWorkspace.QDy[145] + nmpcWorkspace.evGx[1105]*nmpcWorkspace.QDy[146] + nmpcWorkspace.evGx[1113]*nmpcWorkspace.QDy[147] + nmpcWorkspace.evGx[1121]*nmpcWorkspace.QDy[148] + nmpcWorkspace.evGx[1129]*nmpcWorkspace.QDy[149] + nmpcWorkspace.evGx[1137]*nmpcWorkspace.QDy[150] + nmpcWorkspace.evGx[1145]*nmpcWorkspace.QDy[151] + nmpcWorkspace.evGx[1153]*nmpcWorkspace.QDy[152] + nmpcWorkspace.evGx[1161]*nmpcWorkspace.QDy[153] + nmpcWorkspace.evGx[1169]*nmpcWorkspace.QDy[154] + nmpcWorkspace.evGx[1177]*nmpcWorkspace.QDy[155] + nmpcWorkspace.evGx[1185]*nmpcWorkspace.QDy[156] + nmpcWorkspace.evGx[1193]*nmpcWorkspace.QDy[157] + nmpcWorkspace.evGx[1201]*nmpcWorkspace.QDy[158] + nmpcWorkspace.evGx[1209]*nmpcWorkspace.QDy[159] + nmpcWorkspace.evGx[1217]*nmpcWorkspace.QDy[160] + nmpcWorkspace.evGx[1225]*nmpcWorkspace.QDy[161] + nmpcWorkspace.evGx[1233]*nmpcWorkspace.QDy[162] + nmpcWorkspace.evGx[1241]*nmpcWorkspace.QDy[163] + nmpcWorkspace.evGx[1249]*nmpcWorkspace.QDy[164] + nmpcWorkspace.evGx[1257]*nmpcWorkspace.QDy[165] + nmpcWorkspace.evGx[1265]*nmpcWorkspace.QDy[166] + nmpcWorkspace.evGx[1273]*nmpcWorkspace.QDy[167] + nmpcWorkspace.evGx[1281]*nmpcWorkspace.QDy[168] + nmpcWorkspace.evGx[1289]*nmpcWorkspace.QDy[169] + nmpcWorkspace.evGx[1297]*nmpcWorkspace.QDy[170] + nmpcWorkspace.evGx[1305]*nmpcWorkspace.QDy[171] + nmpcWorkspace.evGx[1313]*nmpcWorkspace.QDy[172] + nmpcWorkspace.evGx[1321]*nmpcWorkspace.QDy[173] + nmpcWorkspace.evGx[1329]*nmpcWorkspace.QDy[174] + nmpcWorkspace.evGx[1337]*nmpcWorkspace.QDy[175] + nmpcWorkspace.evGx[1345]*nmpcWorkspace.QDy[176] + nmpcWorkspace.evGx[1353]*nmpcWorkspace.QDy[177] + nmpcWorkspace.evGx[1361]*nmpcWorkspace.QDy[178] + nmpcWorkspace.evGx[1369]*nmpcWorkspace.QDy[179] + nmpcWorkspace.evGx[1377]*nmpcWorkspace.QDy[180] + nmpcWorkspace.evGx[1385]*nmpcWorkspace.QDy[181] + nmpcWorkspace.evGx[1393]*nmpcWorkspace.QDy[182] + nmpcWorkspace.evGx[1401]*nmpcWorkspace.QDy[183] + nmpcWorkspace.evGx[1409]*nmpcWorkspace.QDy[184] + nmpcWorkspace.evGx[1417]*nmpcWorkspace.QDy[185] + nmpcWorkspace.evGx[1425]*nmpcWorkspace.QDy[186] + nmpcWorkspace.evGx[1433]*nmpcWorkspace.QDy[187] + nmpcWorkspace.evGx[1441]*nmpcWorkspace.QDy[188] + nmpcWorkspace.evGx[1449]*nmpcWorkspace.QDy[189] + nmpcWorkspace.evGx[1457]*nmpcWorkspace.QDy[190] + nmpcWorkspace.evGx[1465]*nmpcWorkspace.QDy[191] + nmpcWorkspace.evGx[1473]*nmpcWorkspace.QDy[192] + nmpcWorkspace.evGx[1481]*nmpcWorkspace.QDy[193] + nmpcWorkspace.evGx[1489]*nmpcWorkspace.QDy[194] + nmpcWorkspace.evGx[1497]*nmpcWorkspace.QDy[195] + nmpcWorkspace.evGx[1505]*nmpcWorkspace.QDy[196] + nmpcWorkspace.evGx[1513]*nmpcWorkspace.QDy[197] + nmpcWorkspace.evGx[1521]*nmpcWorkspace.QDy[198] + nmpcWorkspace.evGx[1529]*nmpcWorkspace.QDy[199] + nmpcWorkspace.evGx[1537]*nmpcWorkspace.QDy[200] + nmpcWorkspace.evGx[1545]*nmpcWorkspace.QDy[201] + nmpcWorkspace.evGx[1553]*nmpcWorkspace.QDy[202] + nmpcWorkspace.evGx[1561]*nmpcWorkspace.QDy[203] + nmpcWorkspace.evGx[1569]*nmpcWorkspace.QDy[204] + nmpcWorkspace.evGx[1577]*nmpcWorkspace.QDy[205] + nmpcWorkspace.evGx[1585]*nmpcWorkspace.QDy[206] + nmpcWorkspace.evGx[1593]*nmpcWorkspace.QDy[207] + nmpcWorkspace.evGx[1601]*nmpcWorkspace.QDy[208] + nmpcWorkspace.evGx[1609]*nmpcWorkspace.QDy[209] + nmpcWorkspace.evGx[1617]*nmpcWorkspace.QDy[210] + nmpcWorkspace.evGx[1625]*nmpcWorkspace.QDy[211] + nmpcWorkspace.evGx[1633]*nmpcWorkspace.QDy[212] + nmpcWorkspace.evGx[1641]*nmpcWorkspace.QDy[213] + nmpcWorkspace.evGx[1649]*nmpcWorkspace.QDy[214] + nmpcWorkspace.evGx[1657]*nmpcWorkspace.QDy[215] + nmpcWorkspace.evGx[1665]*nmpcWorkspace.QDy[216] + nmpcWorkspace.evGx[1673]*nmpcWorkspace.QDy[217] + nmpcWorkspace.evGx[1681]*nmpcWorkspace.QDy[218] + nmpcWorkspace.evGx[1689]*nmpcWorkspace.QDy[219] + nmpcWorkspace.evGx[1697]*nmpcWorkspace.QDy[220] + nmpcWorkspace.evGx[1705]*nmpcWorkspace.QDy[221] + nmpcWorkspace.evGx[1713]*nmpcWorkspace.QDy[222] + nmpcWorkspace.evGx[1721]*nmpcWorkspace.QDy[223] + nmpcWorkspace.evGx[1729]*nmpcWorkspace.QDy[224] + nmpcWorkspace.evGx[1737]*nmpcWorkspace.QDy[225] + nmpcWorkspace.evGx[1745]*nmpcWorkspace.QDy[226] + nmpcWorkspace.evGx[1753]*nmpcWorkspace.QDy[227] + nmpcWorkspace.evGx[1761]*nmpcWorkspace.QDy[228] + nmpcWorkspace.evGx[1769]*nmpcWorkspace.QDy[229] + nmpcWorkspace.evGx[1777]*nmpcWorkspace.QDy[230] + nmpcWorkspace.evGx[1785]*nmpcWorkspace.QDy[231] + nmpcWorkspace.evGx[1793]*nmpcWorkspace.QDy[232] + nmpcWorkspace.evGx[1801]*nmpcWorkspace.QDy[233] + nmpcWorkspace.evGx[1809]*nmpcWorkspace.QDy[234] + nmpcWorkspace.evGx[1817]*nmpcWorkspace.QDy[235] + nmpcWorkspace.evGx[1825]*nmpcWorkspace.QDy[236] + nmpcWorkspace.evGx[1833]*nmpcWorkspace.QDy[237] + nmpcWorkspace.evGx[1841]*nmpcWorkspace.QDy[238] + nmpcWorkspace.evGx[1849]*nmpcWorkspace.QDy[239] + nmpcWorkspace.evGx[1857]*nmpcWorkspace.QDy[240] + nmpcWorkspace.evGx[1865]*nmpcWorkspace.QDy[241] + nmpcWorkspace.evGx[1873]*nmpcWorkspace.QDy[242] + nmpcWorkspace.evGx[1881]*nmpcWorkspace.QDy[243] + nmpcWorkspace.evGx[1889]*nmpcWorkspace.QDy[244] + nmpcWorkspace.evGx[1897]*nmpcWorkspace.QDy[245] + nmpcWorkspace.evGx[1905]*nmpcWorkspace.QDy[246] + nmpcWorkspace.evGx[1913]*nmpcWorkspace.QDy[247];
nmpcWorkspace.g[2] = + nmpcWorkspace.evGx[2]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[10]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[18]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[26]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[34]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[42]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[50]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[58]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[66]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[74]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[82]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[90]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[98]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[106]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[114]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[122]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[130]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[138]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[146]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[154]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[162]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[170]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[178]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[186]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[194]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[202]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[210]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[218]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[226]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[234]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[242]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[250]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[258]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[266]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[274]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[282]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[290]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[298]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[306]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[314]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[322]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[330]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[338]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[346]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[354]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[362]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[370]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[378]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[386]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[394]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[402]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[410]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[418]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[426]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[434]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[442]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[450]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[458]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[466]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[474]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[482]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[490]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[498]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[506]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[514]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[522]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[530]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[538]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[546]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[554]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[562]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[570]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[578]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[586]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[594]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[602]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[610]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[618]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[626]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[634]*nmpcWorkspace.QDy[87] + nmpcWorkspace.evGx[642]*nmpcWorkspace.QDy[88] + nmpcWorkspace.evGx[650]*nmpcWorkspace.QDy[89] + nmpcWorkspace.evGx[658]*nmpcWorkspace.QDy[90] + nmpcWorkspace.evGx[666]*nmpcWorkspace.QDy[91] + nmpcWorkspace.evGx[674]*nmpcWorkspace.QDy[92] + nmpcWorkspace.evGx[682]*nmpcWorkspace.QDy[93] + nmpcWorkspace.evGx[690]*nmpcWorkspace.QDy[94] + nmpcWorkspace.evGx[698]*nmpcWorkspace.QDy[95] + nmpcWorkspace.evGx[706]*nmpcWorkspace.QDy[96] + nmpcWorkspace.evGx[714]*nmpcWorkspace.QDy[97] + nmpcWorkspace.evGx[722]*nmpcWorkspace.QDy[98] + nmpcWorkspace.evGx[730]*nmpcWorkspace.QDy[99] + nmpcWorkspace.evGx[738]*nmpcWorkspace.QDy[100] + nmpcWorkspace.evGx[746]*nmpcWorkspace.QDy[101] + nmpcWorkspace.evGx[754]*nmpcWorkspace.QDy[102] + nmpcWorkspace.evGx[762]*nmpcWorkspace.QDy[103] + nmpcWorkspace.evGx[770]*nmpcWorkspace.QDy[104] + nmpcWorkspace.evGx[778]*nmpcWorkspace.QDy[105] + nmpcWorkspace.evGx[786]*nmpcWorkspace.QDy[106] + nmpcWorkspace.evGx[794]*nmpcWorkspace.QDy[107] + nmpcWorkspace.evGx[802]*nmpcWorkspace.QDy[108] + nmpcWorkspace.evGx[810]*nmpcWorkspace.QDy[109] + nmpcWorkspace.evGx[818]*nmpcWorkspace.QDy[110] + nmpcWorkspace.evGx[826]*nmpcWorkspace.QDy[111] + nmpcWorkspace.evGx[834]*nmpcWorkspace.QDy[112] + nmpcWorkspace.evGx[842]*nmpcWorkspace.QDy[113] + nmpcWorkspace.evGx[850]*nmpcWorkspace.QDy[114] + nmpcWorkspace.evGx[858]*nmpcWorkspace.QDy[115] + nmpcWorkspace.evGx[866]*nmpcWorkspace.QDy[116] + nmpcWorkspace.evGx[874]*nmpcWorkspace.QDy[117] + nmpcWorkspace.evGx[882]*nmpcWorkspace.QDy[118] + nmpcWorkspace.evGx[890]*nmpcWorkspace.QDy[119] + nmpcWorkspace.evGx[898]*nmpcWorkspace.QDy[120] + nmpcWorkspace.evGx[906]*nmpcWorkspace.QDy[121] + nmpcWorkspace.evGx[914]*nmpcWorkspace.QDy[122] + nmpcWorkspace.evGx[922]*nmpcWorkspace.QDy[123] + nmpcWorkspace.evGx[930]*nmpcWorkspace.QDy[124] + nmpcWorkspace.evGx[938]*nmpcWorkspace.QDy[125] + nmpcWorkspace.evGx[946]*nmpcWorkspace.QDy[126] + nmpcWorkspace.evGx[954]*nmpcWorkspace.QDy[127] + nmpcWorkspace.evGx[962]*nmpcWorkspace.QDy[128] + nmpcWorkspace.evGx[970]*nmpcWorkspace.QDy[129] + nmpcWorkspace.evGx[978]*nmpcWorkspace.QDy[130] + nmpcWorkspace.evGx[986]*nmpcWorkspace.QDy[131] + nmpcWorkspace.evGx[994]*nmpcWorkspace.QDy[132] + nmpcWorkspace.evGx[1002]*nmpcWorkspace.QDy[133] + nmpcWorkspace.evGx[1010]*nmpcWorkspace.QDy[134] + nmpcWorkspace.evGx[1018]*nmpcWorkspace.QDy[135] + nmpcWorkspace.evGx[1026]*nmpcWorkspace.QDy[136] + nmpcWorkspace.evGx[1034]*nmpcWorkspace.QDy[137] + nmpcWorkspace.evGx[1042]*nmpcWorkspace.QDy[138] + nmpcWorkspace.evGx[1050]*nmpcWorkspace.QDy[139] + nmpcWorkspace.evGx[1058]*nmpcWorkspace.QDy[140] + nmpcWorkspace.evGx[1066]*nmpcWorkspace.QDy[141] + nmpcWorkspace.evGx[1074]*nmpcWorkspace.QDy[142] + nmpcWorkspace.evGx[1082]*nmpcWorkspace.QDy[143] + nmpcWorkspace.evGx[1090]*nmpcWorkspace.QDy[144] + nmpcWorkspace.evGx[1098]*nmpcWorkspace.QDy[145] + nmpcWorkspace.evGx[1106]*nmpcWorkspace.QDy[146] + nmpcWorkspace.evGx[1114]*nmpcWorkspace.QDy[147] + nmpcWorkspace.evGx[1122]*nmpcWorkspace.QDy[148] + nmpcWorkspace.evGx[1130]*nmpcWorkspace.QDy[149] + nmpcWorkspace.evGx[1138]*nmpcWorkspace.QDy[150] + nmpcWorkspace.evGx[1146]*nmpcWorkspace.QDy[151] + nmpcWorkspace.evGx[1154]*nmpcWorkspace.QDy[152] + nmpcWorkspace.evGx[1162]*nmpcWorkspace.QDy[153] + nmpcWorkspace.evGx[1170]*nmpcWorkspace.QDy[154] + nmpcWorkspace.evGx[1178]*nmpcWorkspace.QDy[155] + nmpcWorkspace.evGx[1186]*nmpcWorkspace.QDy[156] + nmpcWorkspace.evGx[1194]*nmpcWorkspace.QDy[157] + nmpcWorkspace.evGx[1202]*nmpcWorkspace.QDy[158] + nmpcWorkspace.evGx[1210]*nmpcWorkspace.QDy[159] + nmpcWorkspace.evGx[1218]*nmpcWorkspace.QDy[160] + nmpcWorkspace.evGx[1226]*nmpcWorkspace.QDy[161] + nmpcWorkspace.evGx[1234]*nmpcWorkspace.QDy[162] + nmpcWorkspace.evGx[1242]*nmpcWorkspace.QDy[163] + nmpcWorkspace.evGx[1250]*nmpcWorkspace.QDy[164] + nmpcWorkspace.evGx[1258]*nmpcWorkspace.QDy[165] + nmpcWorkspace.evGx[1266]*nmpcWorkspace.QDy[166] + nmpcWorkspace.evGx[1274]*nmpcWorkspace.QDy[167] + nmpcWorkspace.evGx[1282]*nmpcWorkspace.QDy[168] + nmpcWorkspace.evGx[1290]*nmpcWorkspace.QDy[169] + nmpcWorkspace.evGx[1298]*nmpcWorkspace.QDy[170] + nmpcWorkspace.evGx[1306]*nmpcWorkspace.QDy[171] + nmpcWorkspace.evGx[1314]*nmpcWorkspace.QDy[172] + nmpcWorkspace.evGx[1322]*nmpcWorkspace.QDy[173] + nmpcWorkspace.evGx[1330]*nmpcWorkspace.QDy[174] + nmpcWorkspace.evGx[1338]*nmpcWorkspace.QDy[175] + nmpcWorkspace.evGx[1346]*nmpcWorkspace.QDy[176] + nmpcWorkspace.evGx[1354]*nmpcWorkspace.QDy[177] + nmpcWorkspace.evGx[1362]*nmpcWorkspace.QDy[178] + nmpcWorkspace.evGx[1370]*nmpcWorkspace.QDy[179] + nmpcWorkspace.evGx[1378]*nmpcWorkspace.QDy[180] + nmpcWorkspace.evGx[1386]*nmpcWorkspace.QDy[181] + nmpcWorkspace.evGx[1394]*nmpcWorkspace.QDy[182] + nmpcWorkspace.evGx[1402]*nmpcWorkspace.QDy[183] + nmpcWorkspace.evGx[1410]*nmpcWorkspace.QDy[184] + nmpcWorkspace.evGx[1418]*nmpcWorkspace.QDy[185] + nmpcWorkspace.evGx[1426]*nmpcWorkspace.QDy[186] + nmpcWorkspace.evGx[1434]*nmpcWorkspace.QDy[187] + nmpcWorkspace.evGx[1442]*nmpcWorkspace.QDy[188] + nmpcWorkspace.evGx[1450]*nmpcWorkspace.QDy[189] + nmpcWorkspace.evGx[1458]*nmpcWorkspace.QDy[190] + nmpcWorkspace.evGx[1466]*nmpcWorkspace.QDy[191] + nmpcWorkspace.evGx[1474]*nmpcWorkspace.QDy[192] + nmpcWorkspace.evGx[1482]*nmpcWorkspace.QDy[193] + nmpcWorkspace.evGx[1490]*nmpcWorkspace.QDy[194] + nmpcWorkspace.evGx[1498]*nmpcWorkspace.QDy[195] + nmpcWorkspace.evGx[1506]*nmpcWorkspace.QDy[196] + nmpcWorkspace.evGx[1514]*nmpcWorkspace.QDy[197] + nmpcWorkspace.evGx[1522]*nmpcWorkspace.QDy[198] + nmpcWorkspace.evGx[1530]*nmpcWorkspace.QDy[199] + nmpcWorkspace.evGx[1538]*nmpcWorkspace.QDy[200] + nmpcWorkspace.evGx[1546]*nmpcWorkspace.QDy[201] + nmpcWorkspace.evGx[1554]*nmpcWorkspace.QDy[202] + nmpcWorkspace.evGx[1562]*nmpcWorkspace.QDy[203] + nmpcWorkspace.evGx[1570]*nmpcWorkspace.QDy[204] + nmpcWorkspace.evGx[1578]*nmpcWorkspace.QDy[205] + nmpcWorkspace.evGx[1586]*nmpcWorkspace.QDy[206] + nmpcWorkspace.evGx[1594]*nmpcWorkspace.QDy[207] + nmpcWorkspace.evGx[1602]*nmpcWorkspace.QDy[208] + nmpcWorkspace.evGx[1610]*nmpcWorkspace.QDy[209] + nmpcWorkspace.evGx[1618]*nmpcWorkspace.QDy[210] + nmpcWorkspace.evGx[1626]*nmpcWorkspace.QDy[211] + nmpcWorkspace.evGx[1634]*nmpcWorkspace.QDy[212] + nmpcWorkspace.evGx[1642]*nmpcWorkspace.QDy[213] + nmpcWorkspace.evGx[1650]*nmpcWorkspace.QDy[214] + nmpcWorkspace.evGx[1658]*nmpcWorkspace.QDy[215] + nmpcWorkspace.evGx[1666]*nmpcWorkspace.QDy[216] + nmpcWorkspace.evGx[1674]*nmpcWorkspace.QDy[217] + nmpcWorkspace.evGx[1682]*nmpcWorkspace.QDy[218] + nmpcWorkspace.evGx[1690]*nmpcWorkspace.QDy[219] + nmpcWorkspace.evGx[1698]*nmpcWorkspace.QDy[220] + nmpcWorkspace.evGx[1706]*nmpcWorkspace.QDy[221] + nmpcWorkspace.evGx[1714]*nmpcWorkspace.QDy[222] + nmpcWorkspace.evGx[1722]*nmpcWorkspace.QDy[223] + nmpcWorkspace.evGx[1730]*nmpcWorkspace.QDy[224] + nmpcWorkspace.evGx[1738]*nmpcWorkspace.QDy[225] + nmpcWorkspace.evGx[1746]*nmpcWorkspace.QDy[226] + nmpcWorkspace.evGx[1754]*nmpcWorkspace.QDy[227] + nmpcWorkspace.evGx[1762]*nmpcWorkspace.QDy[228] + nmpcWorkspace.evGx[1770]*nmpcWorkspace.QDy[229] + nmpcWorkspace.evGx[1778]*nmpcWorkspace.QDy[230] + nmpcWorkspace.evGx[1786]*nmpcWorkspace.QDy[231] + nmpcWorkspace.evGx[1794]*nmpcWorkspace.QDy[232] + nmpcWorkspace.evGx[1802]*nmpcWorkspace.QDy[233] + nmpcWorkspace.evGx[1810]*nmpcWorkspace.QDy[234] + nmpcWorkspace.evGx[1818]*nmpcWorkspace.QDy[235] + nmpcWorkspace.evGx[1826]*nmpcWorkspace.QDy[236] + nmpcWorkspace.evGx[1834]*nmpcWorkspace.QDy[237] + nmpcWorkspace.evGx[1842]*nmpcWorkspace.QDy[238] + nmpcWorkspace.evGx[1850]*nmpcWorkspace.QDy[239] + nmpcWorkspace.evGx[1858]*nmpcWorkspace.QDy[240] + nmpcWorkspace.evGx[1866]*nmpcWorkspace.QDy[241] + nmpcWorkspace.evGx[1874]*nmpcWorkspace.QDy[242] + nmpcWorkspace.evGx[1882]*nmpcWorkspace.QDy[243] + nmpcWorkspace.evGx[1890]*nmpcWorkspace.QDy[244] + nmpcWorkspace.evGx[1898]*nmpcWorkspace.QDy[245] + nmpcWorkspace.evGx[1906]*nmpcWorkspace.QDy[246] + nmpcWorkspace.evGx[1914]*nmpcWorkspace.QDy[247];
nmpcWorkspace.g[3] = + nmpcWorkspace.evGx[3]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[11]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[19]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[27]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[35]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[43]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[51]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[59]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[67]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[75]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[83]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[91]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[99]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[107]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[115]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[123]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[131]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[139]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[147]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[155]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[163]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[171]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[179]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[187]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[195]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[203]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[211]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[219]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[227]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[235]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[243]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[251]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[259]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[267]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[275]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[283]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[291]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[299]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[307]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[315]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[323]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[331]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[339]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[347]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[355]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[363]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[371]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[379]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[387]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[395]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[403]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[411]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[419]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[427]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[435]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[443]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[451]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[459]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[467]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[475]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[483]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[491]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[499]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[507]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[515]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[523]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[531]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[539]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[547]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[555]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[563]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[571]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[579]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[587]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[595]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[603]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[611]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[619]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[627]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[635]*nmpcWorkspace.QDy[87] + nmpcWorkspace.evGx[643]*nmpcWorkspace.QDy[88] + nmpcWorkspace.evGx[651]*nmpcWorkspace.QDy[89] + nmpcWorkspace.evGx[659]*nmpcWorkspace.QDy[90] + nmpcWorkspace.evGx[667]*nmpcWorkspace.QDy[91] + nmpcWorkspace.evGx[675]*nmpcWorkspace.QDy[92] + nmpcWorkspace.evGx[683]*nmpcWorkspace.QDy[93] + nmpcWorkspace.evGx[691]*nmpcWorkspace.QDy[94] + nmpcWorkspace.evGx[699]*nmpcWorkspace.QDy[95] + nmpcWorkspace.evGx[707]*nmpcWorkspace.QDy[96] + nmpcWorkspace.evGx[715]*nmpcWorkspace.QDy[97] + nmpcWorkspace.evGx[723]*nmpcWorkspace.QDy[98] + nmpcWorkspace.evGx[731]*nmpcWorkspace.QDy[99] + nmpcWorkspace.evGx[739]*nmpcWorkspace.QDy[100] + nmpcWorkspace.evGx[747]*nmpcWorkspace.QDy[101] + nmpcWorkspace.evGx[755]*nmpcWorkspace.QDy[102] + nmpcWorkspace.evGx[763]*nmpcWorkspace.QDy[103] + nmpcWorkspace.evGx[771]*nmpcWorkspace.QDy[104] + nmpcWorkspace.evGx[779]*nmpcWorkspace.QDy[105] + nmpcWorkspace.evGx[787]*nmpcWorkspace.QDy[106] + nmpcWorkspace.evGx[795]*nmpcWorkspace.QDy[107] + nmpcWorkspace.evGx[803]*nmpcWorkspace.QDy[108] + nmpcWorkspace.evGx[811]*nmpcWorkspace.QDy[109] + nmpcWorkspace.evGx[819]*nmpcWorkspace.QDy[110] + nmpcWorkspace.evGx[827]*nmpcWorkspace.QDy[111] + nmpcWorkspace.evGx[835]*nmpcWorkspace.QDy[112] + nmpcWorkspace.evGx[843]*nmpcWorkspace.QDy[113] + nmpcWorkspace.evGx[851]*nmpcWorkspace.QDy[114] + nmpcWorkspace.evGx[859]*nmpcWorkspace.QDy[115] + nmpcWorkspace.evGx[867]*nmpcWorkspace.QDy[116] + nmpcWorkspace.evGx[875]*nmpcWorkspace.QDy[117] + nmpcWorkspace.evGx[883]*nmpcWorkspace.QDy[118] + nmpcWorkspace.evGx[891]*nmpcWorkspace.QDy[119] + nmpcWorkspace.evGx[899]*nmpcWorkspace.QDy[120] + nmpcWorkspace.evGx[907]*nmpcWorkspace.QDy[121] + nmpcWorkspace.evGx[915]*nmpcWorkspace.QDy[122] + nmpcWorkspace.evGx[923]*nmpcWorkspace.QDy[123] + nmpcWorkspace.evGx[931]*nmpcWorkspace.QDy[124] + nmpcWorkspace.evGx[939]*nmpcWorkspace.QDy[125] + nmpcWorkspace.evGx[947]*nmpcWorkspace.QDy[126] + nmpcWorkspace.evGx[955]*nmpcWorkspace.QDy[127] + nmpcWorkspace.evGx[963]*nmpcWorkspace.QDy[128] + nmpcWorkspace.evGx[971]*nmpcWorkspace.QDy[129] + nmpcWorkspace.evGx[979]*nmpcWorkspace.QDy[130] + nmpcWorkspace.evGx[987]*nmpcWorkspace.QDy[131] + nmpcWorkspace.evGx[995]*nmpcWorkspace.QDy[132] + nmpcWorkspace.evGx[1003]*nmpcWorkspace.QDy[133] + nmpcWorkspace.evGx[1011]*nmpcWorkspace.QDy[134] + nmpcWorkspace.evGx[1019]*nmpcWorkspace.QDy[135] + nmpcWorkspace.evGx[1027]*nmpcWorkspace.QDy[136] + nmpcWorkspace.evGx[1035]*nmpcWorkspace.QDy[137] + nmpcWorkspace.evGx[1043]*nmpcWorkspace.QDy[138] + nmpcWorkspace.evGx[1051]*nmpcWorkspace.QDy[139] + nmpcWorkspace.evGx[1059]*nmpcWorkspace.QDy[140] + nmpcWorkspace.evGx[1067]*nmpcWorkspace.QDy[141] + nmpcWorkspace.evGx[1075]*nmpcWorkspace.QDy[142] + nmpcWorkspace.evGx[1083]*nmpcWorkspace.QDy[143] + nmpcWorkspace.evGx[1091]*nmpcWorkspace.QDy[144] + nmpcWorkspace.evGx[1099]*nmpcWorkspace.QDy[145] + nmpcWorkspace.evGx[1107]*nmpcWorkspace.QDy[146] + nmpcWorkspace.evGx[1115]*nmpcWorkspace.QDy[147] + nmpcWorkspace.evGx[1123]*nmpcWorkspace.QDy[148] + nmpcWorkspace.evGx[1131]*nmpcWorkspace.QDy[149] + nmpcWorkspace.evGx[1139]*nmpcWorkspace.QDy[150] + nmpcWorkspace.evGx[1147]*nmpcWorkspace.QDy[151] + nmpcWorkspace.evGx[1155]*nmpcWorkspace.QDy[152] + nmpcWorkspace.evGx[1163]*nmpcWorkspace.QDy[153] + nmpcWorkspace.evGx[1171]*nmpcWorkspace.QDy[154] + nmpcWorkspace.evGx[1179]*nmpcWorkspace.QDy[155] + nmpcWorkspace.evGx[1187]*nmpcWorkspace.QDy[156] + nmpcWorkspace.evGx[1195]*nmpcWorkspace.QDy[157] + nmpcWorkspace.evGx[1203]*nmpcWorkspace.QDy[158] + nmpcWorkspace.evGx[1211]*nmpcWorkspace.QDy[159] + nmpcWorkspace.evGx[1219]*nmpcWorkspace.QDy[160] + nmpcWorkspace.evGx[1227]*nmpcWorkspace.QDy[161] + nmpcWorkspace.evGx[1235]*nmpcWorkspace.QDy[162] + nmpcWorkspace.evGx[1243]*nmpcWorkspace.QDy[163] + nmpcWorkspace.evGx[1251]*nmpcWorkspace.QDy[164] + nmpcWorkspace.evGx[1259]*nmpcWorkspace.QDy[165] + nmpcWorkspace.evGx[1267]*nmpcWorkspace.QDy[166] + nmpcWorkspace.evGx[1275]*nmpcWorkspace.QDy[167] + nmpcWorkspace.evGx[1283]*nmpcWorkspace.QDy[168] + nmpcWorkspace.evGx[1291]*nmpcWorkspace.QDy[169] + nmpcWorkspace.evGx[1299]*nmpcWorkspace.QDy[170] + nmpcWorkspace.evGx[1307]*nmpcWorkspace.QDy[171] + nmpcWorkspace.evGx[1315]*nmpcWorkspace.QDy[172] + nmpcWorkspace.evGx[1323]*nmpcWorkspace.QDy[173] + nmpcWorkspace.evGx[1331]*nmpcWorkspace.QDy[174] + nmpcWorkspace.evGx[1339]*nmpcWorkspace.QDy[175] + nmpcWorkspace.evGx[1347]*nmpcWorkspace.QDy[176] + nmpcWorkspace.evGx[1355]*nmpcWorkspace.QDy[177] + nmpcWorkspace.evGx[1363]*nmpcWorkspace.QDy[178] + nmpcWorkspace.evGx[1371]*nmpcWorkspace.QDy[179] + nmpcWorkspace.evGx[1379]*nmpcWorkspace.QDy[180] + nmpcWorkspace.evGx[1387]*nmpcWorkspace.QDy[181] + nmpcWorkspace.evGx[1395]*nmpcWorkspace.QDy[182] + nmpcWorkspace.evGx[1403]*nmpcWorkspace.QDy[183] + nmpcWorkspace.evGx[1411]*nmpcWorkspace.QDy[184] + nmpcWorkspace.evGx[1419]*nmpcWorkspace.QDy[185] + nmpcWorkspace.evGx[1427]*nmpcWorkspace.QDy[186] + nmpcWorkspace.evGx[1435]*nmpcWorkspace.QDy[187] + nmpcWorkspace.evGx[1443]*nmpcWorkspace.QDy[188] + nmpcWorkspace.evGx[1451]*nmpcWorkspace.QDy[189] + nmpcWorkspace.evGx[1459]*nmpcWorkspace.QDy[190] + nmpcWorkspace.evGx[1467]*nmpcWorkspace.QDy[191] + nmpcWorkspace.evGx[1475]*nmpcWorkspace.QDy[192] + nmpcWorkspace.evGx[1483]*nmpcWorkspace.QDy[193] + nmpcWorkspace.evGx[1491]*nmpcWorkspace.QDy[194] + nmpcWorkspace.evGx[1499]*nmpcWorkspace.QDy[195] + nmpcWorkspace.evGx[1507]*nmpcWorkspace.QDy[196] + nmpcWorkspace.evGx[1515]*nmpcWorkspace.QDy[197] + nmpcWorkspace.evGx[1523]*nmpcWorkspace.QDy[198] + nmpcWorkspace.evGx[1531]*nmpcWorkspace.QDy[199] + nmpcWorkspace.evGx[1539]*nmpcWorkspace.QDy[200] + nmpcWorkspace.evGx[1547]*nmpcWorkspace.QDy[201] + nmpcWorkspace.evGx[1555]*nmpcWorkspace.QDy[202] + nmpcWorkspace.evGx[1563]*nmpcWorkspace.QDy[203] + nmpcWorkspace.evGx[1571]*nmpcWorkspace.QDy[204] + nmpcWorkspace.evGx[1579]*nmpcWorkspace.QDy[205] + nmpcWorkspace.evGx[1587]*nmpcWorkspace.QDy[206] + nmpcWorkspace.evGx[1595]*nmpcWorkspace.QDy[207] + nmpcWorkspace.evGx[1603]*nmpcWorkspace.QDy[208] + nmpcWorkspace.evGx[1611]*nmpcWorkspace.QDy[209] + nmpcWorkspace.evGx[1619]*nmpcWorkspace.QDy[210] + nmpcWorkspace.evGx[1627]*nmpcWorkspace.QDy[211] + nmpcWorkspace.evGx[1635]*nmpcWorkspace.QDy[212] + nmpcWorkspace.evGx[1643]*nmpcWorkspace.QDy[213] + nmpcWorkspace.evGx[1651]*nmpcWorkspace.QDy[214] + nmpcWorkspace.evGx[1659]*nmpcWorkspace.QDy[215] + nmpcWorkspace.evGx[1667]*nmpcWorkspace.QDy[216] + nmpcWorkspace.evGx[1675]*nmpcWorkspace.QDy[217] + nmpcWorkspace.evGx[1683]*nmpcWorkspace.QDy[218] + nmpcWorkspace.evGx[1691]*nmpcWorkspace.QDy[219] + nmpcWorkspace.evGx[1699]*nmpcWorkspace.QDy[220] + nmpcWorkspace.evGx[1707]*nmpcWorkspace.QDy[221] + nmpcWorkspace.evGx[1715]*nmpcWorkspace.QDy[222] + nmpcWorkspace.evGx[1723]*nmpcWorkspace.QDy[223] + nmpcWorkspace.evGx[1731]*nmpcWorkspace.QDy[224] + nmpcWorkspace.evGx[1739]*nmpcWorkspace.QDy[225] + nmpcWorkspace.evGx[1747]*nmpcWorkspace.QDy[226] + nmpcWorkspace.evGx[1755]*nmpcWorkspace.QDy[227] + nmpcWorkspace.evGx[1763]*nmpcWorkspace.QDy[228] + nmpcWorkspace.evGx[1771]*nmpcWorkspace.QDy[229] + nmpcWorkspace.evGx[1779]*nmpcWorkspace.QDy[230] + nmpcWorkspace.evGx[1787]*nmpcWorkspace.QDy[231] + nmpcWorkspace.evGx[1795]*nmpcWorkspace.QDy[232] + nmpcWorkspace.evGx[1803]*nmpcWorkspace.QDy[233] + nmpcWorkspace.evGx[1811]*nmpcWorkspace.QDy[234] + nmpcWorkspace.evGx[1819]*nmpcWorkspace.QDy[235] + nmpcWorkspace.evGx[1827]*nmpcWorkspace.QDy[236] + nmpcWorkspace.evGx[1835]*nmpcWorkspace.QDy[237] + nmpcWorkspace.evGx[1843]*nmpcWorkspace.QDy[238] + nmpcWorkspace.evGx[1851]*nmpcWorkspace.QDy[239] + nmpcWorkspace.evGx[1859]*nmpcWorkspace.QDy[240] + nmpcWorkspace.evGx[1867]*nmpcWorkspace.QDy[241] + nmpcWorkspace.evGx[1875]*nmpcWorkspace.QDy[242] + nmpcWorkspace.evGx[1883]*nmpcWorkspace.QDy[243] + nmpcWorkspace.evGx[1891]*nmpcWorkspace.QDy[244] + nmpcWorkspace.evGx[1899]*nmpcWorkspace.QDy[245] + nmpcWorkspace.evGx[1907]*nmpcWorkspace.QDy[246] + nmpcWorkspace.evGx[1915]*nmpcWorkspace.QDy[247];
nmpcWorkspace.g[4] = + nmpcWorkspace.evGx[4]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[12]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[20]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[28]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[36]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[44]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[52]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[60]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[68]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[76]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[84]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[92]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[100]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[108]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[116]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[124]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[132]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[140]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[148]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[156]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[164]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[172]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[180]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[188]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[196]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[204]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[212]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[220]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[228]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[236]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[244]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[252]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[260]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[268]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[276]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[284]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[292]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[300]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[308]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[316]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[324]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[332]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[340]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[348]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[356]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[364]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[372]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[380]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[388]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[396]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[404]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[412]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[420]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[428]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[436]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[444]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[452]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[460]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[468]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[476]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[484]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[492]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[500]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[508]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[516]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[524]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[532]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[540]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[548]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[556]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[564]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[572]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[580]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[588]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[596]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[604]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[612]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[620]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[628]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[636]*nmpcWorkspace.QDy[87] + nmpcWorkspace.evGx[644]*nmpcWorkspace.QDy[88] + nmpcWorkspace.evGx[652]*nmpcWorkspace.QDy[89] + nmpcWorkspace.evGx[660]*nmpcWorkspace.QDy[90] + nmpcWorkspace.evGx[668]*nmpcWorkspace.QDy[91] + nmpcWorkspace.evGx[676]*nmpcWorkspace.QDy[92] + nmpcWorkspace.evGx[684]*nmpcWorkspace.QDy[93] + nmpcWorkspace.evGx[692]*nmpcWorkspace.QDy[94] + nmpcWorkspace.evGx[700]*nmpcWorkspace.QDy[95] + nmpcWorkspace.evGx[708]*nmpcWorkspace.QDy[96] + nmpcWorkspace.evGx[716]*nmpcWorkspace.QDy[97] + nmpcWorkspace.evGx[724]*nmpcWorkspace.QDy[98] + nmpcWorkspace.evGx[732]*nmpcWorkspace.QDy[99] + nmpcWorkspace.evGx[740]*nmpcWorkspace.QDy[100] + nmpcWorkspace.evGx[748]*nmpcWorkspace.QDy[101] + nmpcWorkspace.evGx[756]*nmpcWorkspace.QDy[102] + nmpcWorkspace.evGx[764]*nmpcWorkspace.QDy[103] + nmpcWorkspace.evGx[772]*nmpcWorkspace.QDy[104] + nmpcWorkspace.evGx[780]*nmpcWorkspace.QDy[105] + nmpcWorkspace.evGx[788]*nmpcWorkspace.QDy[106] + nmpcWorkspace.evGx[796]*nmpcWorkspace.QDy[107] + nmpcWorkspace.evGx[804]*nmpcWorkspace.QDy[108] + nmpcWorkspace.evGx[812]*nmpcWorkspace.QDy[109] + nmpcWorkspace.evGx[820]*nmpcWorkspace.QDy[110] + nmpcWorkspace.evGx[828]*nmpcWorkspace.QDy[111] + nmpcWorkspace.evGx[836]*nmpcWorkspace.QDy[112] + nmpcWorkspace.evGx[844]*nmpcWorkspace.QDy[113] + nmpcWorkspace.evGx[852]*nmpcWorkspace.QDy[114] + nmpcWorkspace.evGx[860]*nmpcWorkspace.QDy[115] + nmpcWorkspace.evGx[868]*nmpcWorkspace.QDy[116] + nmpcWorkspace.evGx[876]*nmpcWorkspace.QDy[117] + nmpcWorkspace.evGx[884]*nmpcWorkspace.QDy[118] + nmpcWorkspace.evGx[892]*nmpcWorkspace.QDy[119] + nmpcWorkspace.evGx[900]*nmpcWorkspace.QDy[120] + nmpcWorkspace.evGx[908]*nmpcWorkspace.QDy[121] + nmpcWorkspace.evGx[916]*nmpcWorkspace.QDy[122] + nmpcWorkspace.evGx[924]*nmpcWorkspace.QDy[123] + nmpcWorkspace.evGx[932]*nmpcWorkspace.QDy[124] + nmpcWorkspace.evGx[940]*nmpcWorkspace.QDy[125] + nmpcWorkspace.evGx[948]*nmpcWorkspace.QDy[126] + nmpcWorkspace.evGx[956]*nmpcWorkspace.QDy[127] + nmpcWorkspace.evGx[964]*nmpcWorkspace.QDy[128] + nmpcWorkspace.evGx[972]*nmpcWorkspace.QDy[129] + nmpcWorkspace.evGx[980]*nmpcWorkspace.QDy[130] + nmpcWorkspace.evGx[988]*nmpcWorkspace.QDy[131] + nmpcWorkspace.evGx[996]*nmpcWorkspace.QDy[132] + nmpcWorkspace.evGx[1004]*nmpcWorkspace.QDy[133] + nmpcWorkspace.evGx[1012]*nmpcWorkspace.QDy[134] + nmpcWorkspace.evGx[1020]*nmpcWorkspace.QDy[135] + nmpcWorkspace.evGx[1028]*nmpcWorkspace.QDy[136] + nmpcWorkspace.evGx[1036]*nmpcWorkspace.QDy[137] + nmpcWorkspace.evGx[1044]*nmpcWorkspace.QDy[138] + nmpcWorkspace.evGx[1052]*nmpcWorkspace.QDy[139] + nmpcWorkspace.evGx[1060]*nmpcWorkspace.QDy[140] + nmpcWorkspace.evGx[1068]*nmpcWorkspace.QDy[141] + nmpcWorkspace.evGx[1076]*nmpcWorkspace.QDy[142] + nmpcWorkspace.evGx[1084]*nmpcWorkspace.QDy[143] + nmpcWorkspace.evGx[1092]*nmpcWorkspace.QDy[144] + nmpcWorkspace.evGx[1100]*nmpcWorkspace.QDy[145] + nmpcWorkspace.evGx[1108]*nmpcWorkspace.QDy[146] + nmpcWorkspace.evGx[1116]*nmpcWorkspace.QDy[147] + nmpcWorkspace.evGx[1124]*nmpcWorkspace.QDy[148] + nmpcWorkspace.evGx[1132]*nmpcWorkspace.QDy[149] + nmpcWorkspace.evGx[1140]*nmpcWorkspace.QDy[150] + nmpcWorkspace.evGx[1148]*nmpcWorkspace.QDy[151] + nmpcWorkspace.evGx[1156]*nmpcWorkspace.QDy[152] + nmpcWorkspace.evGx[1164]*nmpcWorkspace.QDy[153] + nmpcWorkspace.evGx[1172]*nmpcWorkspace.QDy[154] + nmpcWorkspace.evGx[1180]*nmpcWorkspace.QDy[155] + nmpcWorkspace.evGx[1188]*nmpcWorkspace.QDy[156] + nmpcWorkspace.evGx[1196]*nmpcWorkspace.QDy[157] + nmpcWorkspace.evGx[1204]*nmpcWorkspace.QDy[158] + nmpcWorkspace.evGx[1212]*nmpcWorkspace.QDy[159] + nmpcWorkspace.evGx[1220]*nmpcWorkspace.QDy[160] + nmpcWorkspace.evGx[1228]*nmpcWorkspace.QDy[161] + nmpcWorkspace.evGx[1236]*nmpcWorkspace.QDy[162] + nmpcWorkspace.evGx[1244]*nmpcWorkspace.QDy[163] + nmpcWorkspace.evGx[1252]*nmpcWorkspace.QDy[164] + nmpcWorkspace.evGx[1260]*nmpcWorkspace.QDy[165] + nmpcWorkspace.evGx[1268]*nmpcWorkspace.QDy[166] + nmpcWorkspace.evGx[1276]*nmpcWorkspace.QDy[167] + nmpcWorkspace.evGx[1284]*nmpcWorkspace.QDy[168] + nmpcWorkspace.evGx[1292]*nmpcWorkspace.QDy[169] + nmpcWorkspace.evGx[1300]*nmpcWorkspace.QDy[170] + nmpcWorkspace.evGx[1308]*nmpcWorkspace.QDy[171] + nmpcWorkspace.evGx[1316]*nmpcWorkspace.QDy[172] + nmpcWorkspace.evGx[1324]*nmpcWorkspace.QDy[173] + nmpcWorkspace.evGx[1332]*nmpcWorkspace.QDy[174] + nmpcWorkspace.evGx[1340]*nmpcWorkspace.QDy[175] + nmpcWorkspace.evGx[1348]*nmpcWorkspace.QDy[176] + nmpcWorkspace.evGx[1356]*nmpcWorkspace.QDy[177] + nmpcWorkspace.evGx[1364]*nmpcWorkspace.QDy[178] + nmpcWorkspace.evGx[1372]*nmpcWorkspace.QDy[179] + nmpcWorkspace.evGx[1380]*nmpcWorkspace.QDy[180] + nmpcWorkspace.evGx[1388]*nmpcWorkspace.QDy[181] + nmpcWorkspace.evGx[1396]*nmpcWorkspace.QDy[182] + nmpcWorkspace.evGx[1404]*nmpcWorkspace.QDy[183] + nmpcWorkspace.evGx[1412]*nmpcWorkspace.QDy[184] + nmpcWorkspace.evGx[1420]*nmpcWorkspace.QDy[185] + nmpcWorkspace.evGx[1428]*nmpcWorkspace.QDy[186] + nmpcWorkspace.evGx[1436]*nmpcWorkspace.QDy[187] + nmpcWorkspace.evGx[1444]*nmpcWorkspace.QDy[188] + nmpcWorkspace.evGx[1452]*nmpcWorkspace.QDy[189] + nmpcWorkspace.evGx[1460]*nmpcWorkspace.QDy[190] + nmpcWorkspace.evGx[1468]*nmpcWorkspace.QDy[191] + nmpcWorkspace.evGx[1476]*nmpcWorkspace.QDy[192] + nmpcWorkspace.evGx[1484]*nmpcWorkspace.QDy[193] + nmpcWorkspace.evGx[1492]*nmpcWorkspace.QDy[194] + nmpcWorkspace.evGx[1500]*nmpcWorkspace.QDy[195] + nmpcWorkspace.evGx[1508]*nmpcWorkspace.QDy[196] + nmpcWorkspace.evGx[1516]*nmpcWorkspace.QDy[197] + nmpcWorkspace.evGx[1524]*nmpcWorkspace.QDy[198] + nmpcWorkspace.evGx[1532]*nmpcWorkspace.QDy[199] + nmpcWorkspace.evGx[1540]*nmpcWorkspace.QDy[200] + nmpcWorkspace.evGx[1548]*nmpcWorkspace.QDy[201] + nmpcWorkspace.evGx[1556]*nmpcWorkspace.QDy[202] + nmpcWorkspace.evGx[1564]*nmpcWorkspace.QDy[203] + nmpcWorkspace.evGx[1572]*nmpcWorkspace.QDy[204] + nmpcWorkspace.evGx[1580]*nmpcWorkspace.QDy[205] + nmpcWorkspace.evGx[1588]*nmpcWorkspace.QDy[206] + nmpcWorkspace.evGx[1596]*nmpcWorkspace.QDy[207] + nmpcWorkspace.evGx[1604]*nmpcWorkspace.QDy[208] + nmpcWorkspace.evGx[1612]*nmpcWorkspace.QDy[209] + nmpcWorkspace.evGx[1620]*nmpcWorkspace.QDy[210] + nmpcWorkspace.evGx[1628]*nmpcWorkspace.QDy[211] + nmpcWorkspace.evGx[1636]*nmpcWorkspace.QDy[212] + nmpcWorkspace.evGx[1644]*nmpcWorkspace.QDy[213] + nmpcWorkspace.evGx[1652]*nmpcWorkspace.QDy[214] + nmpcWorkspace.evGx[1660]*nmpcWorkspace.QDy[215] + nmpcWorkspace.evGx[1668]*nmpcWorkspace.QDy[216] + nmpcWorkspace.evGx[1676]*nmpcWorkspace.QDy[217] + nmpcWorkspace.evGx[1684]*nmpcWorkspace.QDy[218] + nmpcWorkspace.evGx[1692]*nmpcWorkspace.QDy[219] + nmpcWorkspace.evGx[1700]*nmpcWorkspace.QDy[220] + nmpcWorkspace.evGx[1708]*nmpcWorkspace.QDy[221] + nmpcWorkspace.evGx[1716]*nmpcWorkspace.QDy[222] + nmpcWorkspace.evGx[1724]*nmpcWorkspace.QDy[223] + nmpcWorkspace.evGx[1732]*nmpcWorkspace.QDy[224] + nmpcWorkspace.evGx[1740]*nmpcWorkspace.QDy[225] + nmpcWorkspace.evGx[1748]*nmpcWorkspace.QDy[226] + nmpcWorkspace.evGx[1756]*nmpcWorkspace.QDy[227] + nmpcWorkspace.evGx[1764]*nmpcWorkspace.QDy[228] + nmpcWorkspace.evGx[1772]*nmpcWorkspace.QDy[229] + nmpcWorkspace.evGx[1780]*nmpcWorkspace.QDy[230] + nmpcWorkspace.evGx[1788]*nmpcWorkspace.QDy[231] + nmpcWorkspace.evGx[1796]*nmpcWorkspace.QDy[232] + nmpcWorkspace.evGx[1804]*nmpcWorkspace.QDy[233] + nmpcWorkspace.evGx[1812]*nmpcWorkspace.QDy[234] + nmpcWorkspace.evGx[1820]*nmpcWorkspace.QDy[235] + nmpcWorkspace.evGx[1828]*nmpcWorkspace.QDy[236] + nmpcWorkspace.evGx[1836]*nmpcWorkspace.QDy[237] + nmpcWorkspace.evGx[1844]*nmpcWorkspace.QDy[238] + nmpcWorkspace.evGx[1852]*nmpcWorkspace.QDy[239] + nmpcWorkspace.evGx[1860]*nmpcWorkspace.QDy[240] + nmpcWorkspace.evGx[1868]*nmpcWorkspace.QDy[241] + nmpcWorkspace.evGx[1876]*nmpcWorkspace.QDy[242] + nmpcWorkspace.evGx[1884]*nmpcWorkspace.QDy[243] + nmpcWorkspace.evGx[1892]*nmpcWorkspace.QDy[244] + nmpcWorkspace.evGx[1900]*nmpcWorkspace.QDy[245] + nmpcWorkspace.evGx[1908]*nmpcWorkspace.QDy[246] + nmpcWorkspace.evGx[1916]*nmpcWorkspace.QDy[247];
nmpcWorkspace.g[5] = + nmpcWorkspace.evGx[5]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[13]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[21]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[29]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[37]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[45]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[53]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[61]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[69]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[77]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[85]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[93]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[101]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[109]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[117]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[125]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[133]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[141]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[149]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[157]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[165]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[173]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[181]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[189]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[197]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[205]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[213]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[221]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[229]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[237]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[245]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[253]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[261]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[269]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[277]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[285]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[293]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[301]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[309]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[317]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[325]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[333]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[341]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[349]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[357]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[365]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[373]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[381]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[389]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[397]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[405]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[413]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[421]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[429]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[437]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[445]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[453]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[461]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[469]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[477]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[485]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[493]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[501]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[509]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[517]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[525]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[533]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[541]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[549]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[557]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[565]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[573]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[581]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[589]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[597]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[605]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[613]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[621]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[629]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[637]*nmpcWorkspace.QDy[87] + nmpcWorkspace.evGx[645]*nmpcWorkspace.QDy[88] + nmpcWorkspace.evGx[653]*nmpcWorkspace.QDy[89] + nmpcWorkspace.evGx[661]*nmpcWorkspace.QDy[90] + nmpcWorkspace.evGx[669]*nmpcWorkspace.QDy[91] + nmpcWorkspace.evGx[677]*nmpcWorkspace.QDy[92] + nmpcWorkspace.evGx[685]*nmpcWorkspace.QDy[93] + nmpcWorkspace.evGx[693]*nmpcWorkspace.QDy[94] + nmpcWorkspace.evGx[701]*nmpcWorkspace.QDy[95] + nmpcWorkspace.evGx[709]*nmpcWorkspace.QDy[96] + nmpcWorkspace.evGx[717]*nmpcWorkspace.QDy[97] + nmpcWorkspace.evGx[725]*nmpcWorkspace.QDy[98] + nmpcWorkspace.evGx[733]*nmpcWorkspace.QDy[99] + nmpcWorkspace.evGx[741]*nmpcWorkspace.QDy[100] + nmpcWorkspace.evGx[749]*nmpcWorkspace.QDy[101] + nmpcWorkspace.evGx[757]*nmpcWorkspace.QDy[102] + nmpcWorkspace.evGx[765]*nmpcWorkspace.QDy[103] + nmpcWorkspace.evGx[773]*nmpcWorkspace.QDy[104] + nmpcWorkspace.evGx[781]*nmpcWorkspace.QDy[105] + nmpcWorkspace.evGx[789]*nmpcWorkspace.QDy[106] + nmpcWorkspace.evGx[797]*nmpcWorkspace.QDy[107] + nmpcWorkspace.evGx[805]*nmpcWorkspace.QDy[108] + nmpcWorkspace.evGx[813]*nmpcWorkspace.QDy[109] + nmpcWorkspace.evGx[821]*nmpcWorkspace.QDy[110] + nmpcWorkspace.evGx[829]*nmpcWorkspace.QDy[111] + nmpcWorkspace.evGx[837]*nmpcWorkspace.QDy[112] + nmpcWorkspace.evGx[845]*nmpcWorkspace.QDy[113] + nmpcWorkspace.evGx[853]*nmpcWorkspace.QDy[114] + nmpcWorkspace.evGx[861]*nmpcWorkspace.QDy[115] + nmpcWorkspace.evGx[869]*nmpcWorkspace.QDy[116] + nmpcWorkspace.evGx[877]*nmpcWorkspace.QDy[117] + nmpcWorkspace.evGx[885]*nmpcWorkspace.QDy[118] + nmpcWorkspace.evGx[893]*nmpcWorkspace.QDy[119] + nmpcWorkspace.evGx[901]*nmpcWorkspace.QDy[120] + nmpcWorkspace.evGx[909]*nmpcWorkspace.QDy[121] + nmpcWorkspace.evGx[917]*nmpcWorkspace.QDy[122] + nmpcWorkspace.evGx[925]*nmpcWorkspace.QDy[123] + nmpcWorkspace.evGx[933]*nmpcWorkspace.QDy[124] + nmpcWorkspace.evGx[941]*nmpcWorkspace.QDy[125] + nmpcWorkspace.evGx[949]*nmpcWorkspace.QDy[126] + nmpcWorkspace.evGx[957]*nmpcWorkspace.QDy[127] + nmpcWorkspace.evGx[965]*nmpcWorkspace.QDy[128] + nmpcWorkspace.evGx[973]*nmpcWorkspace.QDy[129] + nmpcWorkspace.evGx[981]*nmpcWorkspace.QDy[130] + nmpcWorkspace.evGx[989]*nmpcWorkspace.QDy[131] + nmpcWorkspace.evGx[997]*nmpcWorkspace.QDy[132] + nmpcWorkspace.evGx[1005]*nmpcWorkspace.QDy[133] + nmpcWorkspace.evGx[1013]*nmpcWorkspace.QDy[134] + nmpcWorkspace.evGx[1021]*nmpcWorkspace.QDy[135] + nmpcWorkspace.evGx[1029]*nmpcWorkspace.QDy[136] + nmpcWorkspace.evGx[1037]*nmpcWorkspace.QDy[137] + nmpcWorkspace.evGx[1045]*nmpcWorkspace.QDy[138] + nmpcWorkspace.evGx[1053]*nmpcWorkspace.QDy[139] + nmpcWorkspace.evGx[1061]*nmpcWorkspace.QDy[140] + nmpcWorkspace.evGx[1069]*nmpcWorkspace.QDy[141] + nmpcWorkspace.evGx[1077]*nmpcWorkspace.QDy[142] + nmpcWorkspace.evGx[1085]*nmpcWorkspace.QDy[143] + nmpcWorkspace.evGx[1093]*nmpcWorkspace.QDy[144] + nmpcWorkspace.evGx[1101]*nmpcWorkspace.QDy[145] + nmpcWorkspace.evGx[1109]*nmpcWorkspace.QDy[146] + nmpcWorkspace.evGx[1117]*nmpcWorkspace.QDy[147] + nmpcWorkspace.evGx[1125]*nmpcWorkspace.QDy[148] + nmpcWorkspace.evGx[1133]*nmpcWorkspace.QDy[149] + nmpcWorkspace.evGx[1141]*nmpcWorkspace.QDy[150] + nmpcWorkspace.evGx[1149]*nmpcWorkspace.QDy[151] + nmpcWorkspace.evGx[1157]*nmpcWorkspace.QDy[152] + nmpcWorkspace.evGx[1165]*nmpcWorkspace.QDy[153] + nmpcWorkspace.evGx[1173]*nmpcWorkspace.QDy[154] + nmpcWorkspace.evGx[1181]*nmpcWorkspace.QDy[155] + nmpcWorkspace.evGx[1189]*nmpcWorkspace.QDy[156] + nmpcWorkspace.evGx[1197]*nmpcWorkspace.QDy[157] + nmpcWorkspace.evGx[1205]*nmpcWorkspace.QDy[158] + nmpcWorkspace.evGx[1213]*nmpcWorkspace.QDy[159] + nmpcWorkspace.evGx[1221]*nmpcWorkspace.QDy[160] + nmpcWorkspace.evGx[1229]*nmpcWorkspace.QDy[161] + nmpcWorkspace.evGx[1237]*nmpcWorkspace.QDy[162] + nmpcWorkspace.evGx[1245]*nmpcWorkspace.QDy[163] + nmpcWorkspace.evGx[1253]*nmpcWorkspace.QDy[164] + nmpcWorkspace.evGx[1261]*nmpcWorkspace.QDy[165] + nmpcWorkspace.evGx[1269]*nmpcWorkspace.QDy[166] + nmpcWorkspace.evGx[1277]*nmpcWorkspace.QDy[167] + nmpcWorkspace.evGx[1285]*nmpcWorkspace.QDy[168] + nmpcWorkspace.evGx[1293]*nmpcWorkspace.QDy[169] + nmpcWorkspace.evGx[1301]*nmpcWorkspace.QDy[170] + nmpcWorkspace.evGx[1309]*nmpcWorkspace.QDy[171] + nmpcWorkspace.evGx[1317]*nmpcWorkspace.QDy[172] + nmpcWorkspace.evGx[1325]*nmpcWorkspace.QDy[173] + nmpcWorkspace.evGx[1333]*nmpcWorkspace.QDy[174] + nmpcWorkspace.evGx[1341]*nmpcWorkspace.QDy[175] + nmpcWorkspace.evGx[1349]*nmpcWorkspace.QDy[176] + nmpcWorkspace.evGx[1357]*nmpcWorkspace.QDy[177] + nmpcWorkspace.evGx[1365]*nmpcWorkspace.QDy[178] + nmpcWorkspace.evGx[1373]*nmpcWorkspace.QDy[179] + nmpcWorkspace.evGx[1381]*nmpcWorkspace.QDy[180] + nmpcWorkspace.evGx[1389]*nmpcWorkspace.QDy[181] + nmpcWorkspace.evGx[1397]*nmpcWorkspace.QDy[182] + nmpcWorkspace.evGx[1405]*nmpcWorkspace.QDy[183] + nmpcWorkspace.evGx[1413]*nmpcWorkspace.QDy[184] + nmpcWorkspace.evGx[1421]*nmpcWorkspace.QDy[185] + nmpcWorkspace.evGx[1429]*nmpcWorkspace.QDy[186] + nmpcWorkspace.evGx[1437]*nmpcWorkspace.QDy[187] + nmpcWorkspace.evGx[1445]*nmpcWorkspace.QDy[188] + nmpcWorkspace.evGx[1453]*nmpcWorkspace.QDy[189] + nmpcWorkspace.evGx[1461]*nmpcWorkspace.QDy[190] + nmpcWorkspace.evGx[1469]*nmpcWorkspace.QDy[191] + nmpcWorkspace.evGx[1477]*nmpcWorkspace.QDy[192] + nmpcWorkspace.evGx[1485]*nmpcWorkspace.QDy[193] + nmpcWorkspace.evGx[1493]*nmpcWorkspace.QDy[194] + nmpcWorkspace.evGx[1501]*nmpcWorkspace.QDy[195] + nmpcWorkspace.evGx[1509]*nmpcWorkspace.QDy[196] + nmpcWorkspace.evGx[1517]*nmpcWorkspace.QDy[197] + nmpcWorkspace.evGx[1525]*nmpcWorkspace.QDy[198] + nmpcWorkspace.evGx[1533]*nmpcWorkspace.QDy[199] + nmpcWorkspace.evGx[1541]*nmpcWorkspace.QDy[200] + nmpcWorkspace.evGx[1549]*nmpcWorkspace.QDy[201] + nmpcWorkspace.evGx[1557]*nmpcWorkspace.QDy[202] + nmpcWorkspace.evGx[1565]*nmpcWorkspace.QDy[203] + nmpcWorkspace.evGx[1573]*nmpcWorkspace.QDy[204] + nmpcWorkspace.evGx[1581]*nmpcWorkspace.QDy[205] + nmpcWorkspace.evGx[1589]*nmpcWorkspace.QDy[206] + nmpcWorkspace.evGx[1597]*nmpcWorkspace.QDy[207] + nmpcWorkspace.evGx[1605]*nmpcWorkspace.QDy[208] + nmpcWorkspace.evGx[1613]*nmpcWorkspace.QDy[209] + nmpcWorkspace.evGx[1621]*nmpcWorkspace.QDy[210] + nmpcWorkspace.evGx[1629]*nmpcWorkspace.QDy[211] + nmpcWorkspace.evGx[1637]*nmpcWorkspace.QDy[212] + nmpcWorkspace.evGx[1645]*nmpcWorkspace.QDy[213] + nmpcWorkspace.evGx[1653]*nmpcWorkspace.QDy[214] + nmpcWorkspace.evGx[1661]*nmpcWorkspace.QDy[215] + nmpcWorkspace.evGx[1669]*nmpcWorkspace.QDy[216] + nmpcWorkspace.evGx[1677]*nmpcWorkspace.QDy[217] + nmpcWorkspace.evGx[1685]*nmpcWorkspace.QDy[218] + nmpcWorkspace.evGx[1693]*nmpcWorkspace.QDy[219] + nmpcWorkspace.evGx[1701]*nmpcWorkspace.QDy[220] + nmpcWorkspace.evGx[1709]*nmpcWorkspace.QDy[221] + nmpcWorkspace.evGx[1717]*nmpcWorkspace.QDy[222] + nmpcWorkspace.evGx[1725]*nmpcWorkspace.QDy[223] + nmpcWorkspace.evGx[1733]*nmpcWorkspace.QDy[224] + nmpcWorkspace.evGx[1741]*nmpcWorkspace.QDy[225] + nmpcWorkspace.evGx[1749]*nmpcWorkspace.QDy[226] + nmpcWorkspace.evGx[1757]*nmpcWorkspace.QDy[227] + nmpcWorkspace.evGx[1765]*nmpcWorkspace.QDy[228] + nmpcWorkspace.evGx[1773]*nmpcWorkspace.QDy[229] + nmpcWorkspace.evGx[1781]*nmpcWorkspace.QDy[230] + nmpcWorkspace.evGx[1789]*nmpcWorkspace.QDy[231] + nmpcWorkspace.evGx[1797]*nmpcWorkspace.QDy[232] + nmpcWorkspace.evGx[1805]*nmpcWorkspace.QDy[233] + nmpcWorkspace.evGx[1813]*nmpcWorkspace.QDy[234] + nmpcWorkspace.evGx[1821]*nmpcWorkspace.QDy[235] + nmpcWorkspace.evGx[1829]*nmpcWorkspace.QDy[236] + nmpcWorkspace.evGx[1837]*nmpcWorkspace.QDy[237] + nmpcWorkspace.evGx[1845]*nmpcWorkspace.QDy[238] + nmpcWorkspace.evGx[1853]*nmpcWorkspace.QDy[239] + nmpcWorkspace.evGx[1861]*nmpcWorkspace.QDy[240] + nmpcWorkspace.evGx[1869]*nmpcWorkspace.QDy[241] + nmpcWorkspace.evGx[1877]*nmpcWorkspace.QDy[242] + nmpcWorkspace.evGx[1885]*nmpcWorkspace.QDy[243] + nmpcWorkspace.evGx[1893]*nmpcWorkspace.QDy[244] + nmpcWorkspace.evGx[1901]*nmpcWorkspace.QDy[245] + nmpcWorkspace.evGx[1909]*nmpcWorkspace.QDy[246] + nmpcWorkspace.evGx[1917]*nmpcWorkspace.QDy[247];
nmpcWorkspace.g[6] = + nmpcWorkspace.evGx[6]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[14]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[22]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[30]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[38]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[46]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[54]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[62]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[70]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[78]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[86]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[94]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[102]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[110]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[118]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[126]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[134]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[142]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[150]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[158]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[166]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[174]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[182]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[190]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[198]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[206]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[214]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[222]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[230]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[238]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[246]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[254]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[262]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[270]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[278]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[286]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[294]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[302]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[310]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[318]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[326]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[334]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[342]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[350]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[358]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[366]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[374]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[382]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[390]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[398]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[406]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[414]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[422]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[430]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[438]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[446]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[454]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[462]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[470]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[478]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[486]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[494]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[502]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[510]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[518]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[526]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[534]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[542]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[550]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[558]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[566]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[574]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[582]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[590]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[598]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[606]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[614]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[622]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[630]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[638]*nmpcWorkspace.QDy[87] + nmpcWorkspace.evGx[646]*nmpcWorkspace.QDy[88] + nmpcWorkspace.evGx[654]*nmpcWorkspace.QDy[89] + nmpcWorkspace.evGx[662]*nmpcWorkspace.QDy[90] + nmpcWorkspace.evGx[670]*nmpcWorkspace.QDy[91] + nmpcWorkspace.evGx[678]*nmpcWorkspace.QDy[92] + nmpcWorkspace.evGx[686]*nmpcWorkspace.QDy[93] + nmpcWorkspace.evGx[694]*nmpcWorkspace.QDy[94] + nmpcWorkspace.evGx[702]*nmpcWorkspace.QDy[95] + nmpcWorkspace.evGx[710]*nmpcWorkspace.QDy[96] + nmpcWorkspace.evGx[718]*nmpcWorkspace.QDy[97] + nmpcWorkspace.evGx[726]*nmpcWorkspace.QDy[98] + nmpcWorkspace.evGx[734]*nmpcWorkspace.QDy[99] + nmpcWorkspace.evGx[742]*nmpcWorkspace.QDy[100] + nmpcWorkspace.evGx[750]*nmpcWorkspace.QDy[101] + nmpcWorkspace.evGx[758]*nmpcWorkspace.QDy[102] + nmpcWorkspace.evGx[766]*nmpcWorkspace.QDy[103] + nmpcWorkspace.evGx[774]*nmpcWorkspace.QDy[104] + nmpcWorkspace.evGx[782]*nmpcWorkspace.QDy[105] + nmpcWorkspace.evGx[790]*nmpcWorkspace.QDy[106] + nmpcWorkspace.evGx[798]*nmpcWorkspace.QDy[107] + nmpcWorkspace.evGx[806]*nmpcWorkspace.QDy[108] + nmpcWorkspace.evGx[814]*nmpcWorkspace.QDy[109] + nmpcWorkspace.evGx[822]*nmpcWorkspace.QDy[110] + nmpcWorkspace.evGx[830]*nmpcWorkspace.QDy[111] + nmpcWorkspace.evGx[838]*nmpcWorkspace.QDy[112] + nmpcWorkspace.evGx[846]*nmpcWorkspace.QDy[113] + nmpcWorkspace.evGx[854]*nmpcWorkspace.QDy[114] + nmpcWorkspace.evGx[862]*nmpcWorkspace.QDy[115] + nmpcWorkspace.evGx[870]*nmpcWorkspace.QDy[116] + nmpcWorkspace.evGx[878]*nmpcWorkspace.QDy[117] + nmpcWorkspace.evGx[886]*nmpcWorkspace.QDy[118] + nmpcWorkspace.evGx[894]*nmpcWorkspace.QDy[119] + nmpcWorkspace.evGx[902]*nmpcWorkspace.QDy[120] + nmpcWorkspace.evGx[910]*nmpcWorkspace.QDy[121] + nmpcWorkspace.evGx[918]*nmpcWorkspace.QDy[122] + nmpcWorkspace.evGx[926]*nmpcWorkspace.QDy[123] + nmpcWorkspace.evGx[934]*nmpcWorkspace.QDy[124] + nmpcWorkspace.evGx[942]*nmpcWorkspace.QDy[125] + nmpcWorkspace.evGx[950]*nmpcWorkspace.QDy[126] + nmpcWorkspace.evGx[958]*nmpcWorkspace.QDy[127] + nmpcWorkspace.evGx[966]*nmpcWorkspace.QDy[128] + nmpcWorkspace.evGx[974]*nmpcWorkspace.QDy[129] + nmpcWorkspace.evGx[982]*nmpcWorkspace.QDy[130] + nmpcWorkspace.evGx[990]*nmpcWorkspace.QDy[131] + nmpcWorkspace.evGx[998]*nmpcWorkspace.QDy[132] + nmpcWorkspace.evGx[1006]*nmpcWorkspace.QDy[133] + nmpcWorkspace.evGx[1014]*nmpcWorkspace.QDy[134] + nmpcWorkspace.evGx[1022]*nmpcWorkspace.QDy[135] + nmpcWorkspace.evGx[1030]*nmpcWorkspace.QDy[136] + nmpcWorkspace.evGx[1038]*nmpcWorkspace.QDy[137] + nmpcWorkspace.evGx[1046]*nmpcWorkspace.QDy[138] + nmpcWorkspace.evGx[1054]*nmpcWorkspace.QDy[139] + nmpcWorkspace.evGx[1062]*nmpcWorkspace.QDy[140] + nmpcWorkspace.evGx[1070]*nmpcWorkspace.QDy[141] + nmpcWorkspace.evGx[1078]*nmpcWorkspace.QDy[142] + nmpcWorkspace.evGx[1086]*nmpcWorkspace.QDy[143] + nmpcWorkspace.evGx[1094]*nmpcWorkspace.QDy[144] + nmpcWorkspace.evGx[1102]*nmpcWorkspace.QDy[145] + nmpcWorkspace.evGx[1110]*nmpcWorkspace.QDy[146] + nmpcWorkspace.evGx[1118]*nmpcWorkspace.QDy[147] + nmpcWorkspace.evGx[1126]*nmpcWorkspace.QDy[148] + nmpcWorkspace.evGx[1134]*nmpcWorkspace.QDy[149] + nmpcWorkspace.evGx[1142]*nmpcWorkspace.QDy[150] + nmpcWorkspace.evGx[1150]*nmpcWorkspace.QDy[151] + nmpcWorkspace.evGx[1158]*nmpcWorkspace.QDy[152] + nmpcWorkspace.evGx[1166]*nmpcWorkspace.QDy[153] + nmpcWorkspace.evGx[1174]*nmpcWorkspace.QDy[154] + nmpcWorkspace.evGx[1182]*nmpcWorkspace.QDy[155] + nmpcWorkspace.evGx[1190]*nmpcWorkspace.QDy[156] + nmpcWorkspace.evGx[1198]*nmpcWorkspace.QDy[157] + nmpcWorkspace.evGx[1206]*nmpcWorkspace.QDy[158] + nmpcWorkspace.evGx[1214]*nmpcWorkspace.QDy[159] + nmpcWorkspace.evGx[1222]*nmpcWorkspace.QDy[160] + nmpcWorkspace.evGx[1230]*nmpcWorkspace.QDy[161] + nmpcWorkspace.evGx[1238]*nmpcWorkspace.QDy[162] + nmpcWorkspace.evGx[1246]*nmpcWorkspace.QDy[163] + nmpcWorkspace.evGx[1254]*nmpcWorkspace.QDy[164] + nmpcWorkspace.evGx[1262]*nmpcWorkspace.QDy[165] + nmpcWorkspace.evGx[1270]*nmpcWorkspace.QDy[166] + nmpcWorkspace.evGx[1278]*nmpcWorkspace.QDy[167] + nmpcWorkspace.evGx[1286]*nmpcWorkspace.QDy[168] + nmpcWorkspace.evGx[1294]*nmpcWorkspace.QDy[169] + nmpcWorkspace.evGx[1302]*nmpcWorkspace.QDy[170] + nmpcWorkspace.evGx[1310]*nmpcWorkspace.QDy[171] + nmpcWorkspace.evGx[1318]*nmpcWorkspace.QDy[172] + nmpcWorkspace.evGx[1326]*nmpcWorkspace.QDy[173] + nmpcWorkspace.evGx[1334]*nmpcWorkspace.QDy[174] + nmpcWorkspace.evGx[1342]*nmpcWorkspace.QDy[175] + nmpcWorkspace.evGx[1350]*nmpcWorkspace.QDy[176] + nmpcWorkspace.evGx[1358]*nmpcWorkspace.QDy[177] + nmpcWorkspace.evGx[1366]*nmpcWorkspace.QDy[178] + nmpcWorkspace.evGx[1374]*nmpcWorkspace.QDy[179] + nmpcWorkspace.evGx[1382]*nmpcWorkspace.QDy[180] + nmpcWorkspace.evGx[1390]*nmpcWorkspace.QDy[181] + nmpcWorkspace.evGx[1398]*nmpcWorkspace.QDy[182] + nmpcWorkspace.evGx[1406]*nmpcWorkspace.QDy[183] + nmpcWorkspace.evGx[1414]*nmpcWorkspace.QDy[184] + nmpcWorkspace.evGx[1422]*nmpcWorkspace.QDy[185] + nmpcWorkspace.evGx[1430]*nmpcWorkspace.QDy[186] + nmpcWorkspace.evGx[1438]*nmpcWorkspace.QDy[187] + nmpcWorkspace.evGx[1446]*nmpcWorkspace.QDy[188] + nmpcWorkspace.evGx[1454]*nmpcWorkspace.QDy[189] + nmpcWorkspace.evGx[1462]*nmpcWorkspace.QDy[190] + nmpcWorkspace.evGx[1470]*nmpcWorkspace.QDy[191] + nmpcWorkspace.evGx[1478]*nmpcWorkspace.QDy[192] + nmpcWorkspace.evGx[1486]*nmpcWorkspace.QDy[193] + nmpcWorkspace.evGx[1494]*nmpcWorkspace.QDy[194] + nmpcWorkspace.evGx[1502]*nmpcWorkspace.QDy[195] + nmpcWorkspace.evGx[1510]*nmpcWorkspace.QDy[196] + nmpcWorkspace.evGx[1518]*nmpcWorkspace.QDy[197] + nmpcWorkspace.evGx[1526]*nmpcWorkspace.QDy[198] + nmpcWorkspace.evGx[1534]*nmpcWorkspace.QDy[199] + nmpcWorkspace.evGx[1542]*nmpcWorkspace.QDy[200] + nmpcWorkspace.evGx[1550]*nmpcWorkspace.QDy[201] + nmpcWorkspace.evGx[1558]*nmpcWorkspace.QDy[202] + nmpcWorkspace.evGx[1566]*nmpcWorkspace.QDy[203] + nmpcWorkspace.evGx[1574]*nmpcWorkspace.QDy[204] + nmpcWorkspace.evGx[1582]*nmpcWorkspace.QDy[205] + nmpcWorkspace.evGx[1590]*nmpcWorkspace.QDy[206] + nmpcWorkspace.evGx[1598]*nmpcWorkspace.QDy[207] + nmpcWorkspace.evGx[1606]*nmpcWorkspace.QDy[208] + nmpcWorkspace.evGx[1614]*nmpcWorkspace.QDy[209] + nmpcWorkspace.evGx[1622]*nmpcWorkspace.QDy[210] + nmpcWorkspace.evGx[1630]*nmpcWorkspace.QDy[211] + nmpcWorkspace.evGx[1638]*nmpcWorkspace.QDy[212] + nmpcWorkspace.evGx[1646]*nmpcWorkspace.QDy[213] + nmpcWorkspace.evGx[1654]*nmpcWorkspace.QDy[214] + nmpcWorkspace.evGx[1662]*nmpcWorkspace.QDy[215] + nmpcWorkspace.evGx[1670]*nmpcWorkspace.QDy[216] + nmpcWorkspace.evGx[1678]*nmpcWorkspace.QDy[217] + nmpcWorkspace.evGx[1686]*nmpcWorkspace.QDy[218] + nmpcWorkspace.evGx[1694]*nmpcWorkspace.QDy[219] + nmpcWorkspace.evGx[1702]*nmpcWorkspace.QDy[220] + nmpcWorkspace.evGx[1710]*nmpcWorkspace.QDy[221] + nmpcWorkspace.evGx[1718]*nmpcWorkspace.QDy[222] + nmpcWorkspace.evGx[1726]*nmpcWorkspace.QDy[223] + nmpcWorkspace.evGx[1734]*nmpcWorkspace.QDy[224] + nmpcWorkspace.evGx[1742]*nmpcWorkspace.QDy[225] + nmpcWorkspace.evGx[1750]*nmpcWorkspace.QDy[226] + nmpcWorkspace.evGx[1758]*nmpcWorkspace.QDy[227] + nmpcWorkspace.evGx[1766]*nmpcWorkspace.QDy[228] + nmpcWorkspace.evGx[1774]*nmpcWorkspace.QDy[229] + nmpcWorkspace.evGx[1782]*nmpcWorkspace.QDy[230] + nmpcWorkspace.evGx[1790]*nmpcWorkspace.QDy[231] + nmpcWorkspace.evGx[1798]*nmpcWorkspace.QDy[232] + nmpcWorkspace.evGx[1806]*nmpcWorkspace.QDy[233] + nmpcWorkspace.evGx[1814]*nmpcWorkspace.QDy[234] + nmpcWorkspace.evGx[1822]*nmpcWorkspace.QDy[235] + nmpcWorkspace.evGx[1830]*nmpcWorkspace.QDy[236] + nmpcWorkspace.evGx[1838]*nmpcWorkspace.QDy[237] + nmpcWorkspace.evGx[1846]*nmpcWorkspace.QDy[238] + nmpcWorkspace.evGx[1854]*nmpcWorkspace.QDy[239] + nmpcWorkspace.evGx[1862]*nmpcWorkspace.QDy[240] + nmpcWorkspace.evGx[1870]*nmpcWorkspace.QDy[241] + nmpcWorkspace.evGx[1878]*nmpcWorkspace.QDy[242] + nmpcWorkspace.evGx[1886]*nmpcWorkspace.QDy[243] + nmpcWorkspace.evGx[1894]*nmpcWorkspace.QDy[244] + nmpcWorkspace.evGx[1902]*nmpcWorkspace.QDy[245] + nmpcWorkspace.evGx[1910]*nmpcWorkspace.QDy[246] + nmpcWorkspace.evGx[1918]*nmpcWorkspace.QDy[247];
nmpcWorkspace.g[7] = + nmpcWorkspace.evGx[7]*nmpcWorkspace.QDy[8] + nmpcWorkspace.evGx[15]*nmpcWorkspace.QDy[9] + nmpcWorkspace.evGx[23]*nmpcWorkspace.QDy[10] + nmpcWorkspace.evGx[31]*nmpcWorkspace.QDy[11] + nmpcWorkspace.evGx[39]*nmpcWorkspace.QDy[12] + nmpcWorkspace.evGx[47]*nmpcWorkspace.QDy[13] + nmpcWorkspace.evGx[55]*nmpcWorkspace.QDy[14] + nmpcWorkspace.evGx[63]*nmpcWorkspace.QDy[15] + nmpcWorkspace.evGx[71]*nmpcWorkspace.QDy[16] + nmpcWorkspace.evGx[79]*nmpcWorkspace.QDy[17] + nmpcWorkspace.evGx[87]*nmpcWorkspace.QDy[18] + nmpcWorkspace.evGx[95]*nmpcWorkspace.QDy[19] + nmpcWorkspace.evGx[103]*nmpcWorkspace.QDy[20] + nmpcWorkspace.evGx[111]*nmpcWorkspace.QDy[21] + nmpcWorkspace.evGx[119]*nmpcWorkspace.QDy[22] + nmpcWorkspace.evGx[127]*nmpcWorkspace.QDy[23] + nmpcWorkspace.evGx[135]*nmpcWorkspace.QDy[24] + nmpcWorkspace.evGx[143]*nmpcWorkspace.QDy[25] + nmpcWorkspace.evGx[151]*nmpcWorkspace.QDy[26] + nmpcWorkspace.evGx[159]*nmpcWorkspace.QDy[27] + nmpcWorkspace.evGx[167]*nmpcWorkspace.QDy[28] + nmpcWorkspace.evGx[175]*nmpcWorkspace.QDy[29] + nmpcWorkspace.evGx[183]*nmpcWorkspace.QDy[30] + nmpcWorkspace.evGx[191]*nmpcWorkspace.QDy[31] + nmpcWorkspace.evGx[199]*nmpcWorkspace.QDy[32] + nmpcWorkspace.evGx[207]*nmpcWorkspace.QDy[33] + nmpcWorkspace.evGx[215]*nmpcWorkspace.QDy[34] + nmpcWorkspace.evGx[223]*nmpcWorkspace.QDy[35] + nmpcWorkspace.evGx[231]*nmpcWorkspace.QDy[36] + nmpcWorkspace.evGx[239]*nmpcWorkspace.QDy[37] + nmpcWorkspace.evGx[247]*nmpcWorkspace.QDy[38] + nmpcWorkspace.evGx[255]*nmpcWorkspace.QDy[39] + nmpcWorkspace.evGx[263]*nmpcWorkspace.QDy[40] + nmpcWorkspace.evGx[271]*nmpcWorkspace.QDy[41] + nmpcWorkspace.evGx[279]*nmpcWorkspace.QDy[42] + nmpcWorkspace.evGx[287]*nmpcWorkspace.QDy[43] + nmpcWorkspace.evGx[295]*nmpcWorkspace.QDy[44] + nmpcWorkspace.evGx[303]*nmpcWorkspace.QDy[45] + nmpcWorkspace.evGx[311]*nmpcWorkspace.QDy[46] + nmpcWorkspace.evGx[319]*nmpcWorkspace.QDy[47] + nmpcWorkspace.evGx[327]*nmpcWorkspace.QDy[48] + nmpcWorkspace.evGx[335]*nmpcWorkspace.QDy[49] + nmpcWorkspace.evGx[343]*nmpcWorkspace.QDy[50] + nmpcWorkspace.evGx[351]*nmpcWorkspace.QDy[51] + nmpcWorkspace.evGx[359]*nmpcWorkspace.QDy[52] + nmpcWorkspace.evGx[367]*nmpcWorkspace.QDy[53] + nmpcWorkspace.evGx[375]*nmpcWorkspace.QDy[54] + nmpcWorkspace.evGx[383]*nmpcWorkspace.QDy[55] + nmpcWorkspace.evGx[391]*nmpcWorkspace.QDy[56] + nmpcWorkspace.evGx[399]*nmpcWorkspace.QDy[57] + nmpcWorkspace.evGx[407]*nmpcWorkspace.QDy[58] + nmpcWorkspace.evGx[415]*nmpcWorkspace.QDy[59] + nmpcWorkspace.evGx[423]*nmpcWorkspace.QDy[60] + nmpcWorkspace.evGx[431]*nmpcWorkspace.QDy[61] + nmpcWorkspace.evGx[439]*nmpcWorkspace.QDy[62] + nmpcWorkspace.evGx[447]*nmpcWorkspace.QDy[63] + nmpcWorkspace.evGx[455]*nmpcWorkspace.QDy[64] + nmpcWorkspace.evGx[463]*nmpcWorkspace.QDy[65] + nmpcWorkspace.evGx[471]*nmpcWorkspace.QDy[66] + nmpcWorkspace.evGx[479]*nmpcWorkspace.QDy[67] + nmpcWorkspace.evGx[487]*nmpcWorkspace.QDy[68] + nmpcWorkspace.evGx[495]*nmpcWorkspace.QDy[69] + nmpcWorkspace.evGx[503]*nmpcWorkspace.QDy[70] + nmpcWorkspace.evGx[511]*nmpcWorkspace.QDy[71] + nmpcWorkspace.evGx[519]*nmpcWorkspace.QDy[72] + nmpcWorkspace.evGx[527]*nmpcWorkspace.QDy[73] + nmpcWorkspace.evGx[535]*nmpcWorkspace.QDy[74] + nmpcWorkspace.evGx[543]*nmpcWorkspace.QDy[75] + nmpcWorkspace.evGx[551]*nmpcWorkspace.QDy[76] + nmpcWorkspace.evGx[559]*nmpcWorkspace.QDy[77] + nmpcWorkspace.evGx[567]*nmpcWorkspace.QDy[78] + nmpcWorkspace.evGx[575]*nmpcWorkspace.QDy[79] + nmpcWorkspace.evGx[583]*nmpcWorkspace.QDy[80] + nmpcWorkspace.evGx[591]*nmpcWorkspace.QDy[81] + nmpcWorkspace.evGx[599]*nmpcWorkspace.QDy[82] + nmpcWorkspace.evGx[607]*nmpcWorkspace.QDy[83] + nmpcWorkspace.evGx[615]*nmpcWorkspace.QDy[84] + nmpcWorkspace.evGx[623]*nmpcWorkspace.QDy[85] + nmpcWorkspace.evGx[631]*nmpcWorkspace.QDy[86] + nmpcWorkspace.evGx[639]*nmpcWorkspace.QDy[87] + nmpcWorkspace.evGx[647]*nmpcWorkspace.QDy[88] + nmpcWorkspace.evGx[655]*nmpcWorkspace.QDy[89] + nmpcWorkspace.evGx[663]*nmpcWorkspace.QDy[90] + nmpcWorkspace.evGx[671]*nmpcWorkspace.QDy[91] + nmpcWorkspace.evGx[679]*nmpcWorkspace.QDy[92] + nmpcWorkspace.evGx[687]*nmpcWorkspace.QDy[93] + nmpcWorkspace.evGx[695]*nmpcWorkspace.QDy[94] + nmpcWorkspace.evGx[703]*nmpcWorkspace.QDy[95] + nmpcWorkspace.evGx[711]*nmpcWorkspace.QDy[96] + nmpcWorkspace.evGx[719]*nmpcWorkspace.QDy[97] + nmpcWorkspace.evGx[727]*nmpcWorkspace.QDy[98] + nmpcWorkspace.evGx[735]*nmpcWorkspace.QDy[99] + nmpcWorkspace.evGx[743]*nmpcWorkspace.QDy[100] + nmpcWorkspace.evGx[751]*nmpcWorkspace.QDy[101] + nmpcWorkspace.evGx[759]*nmpcWorkspace.QDy[102] + nmpcWorkspace.evGx[767]*nmpcWorkspace.QDy[103] + nmpcWorkspace.evGx[775]*nmpcWorkspace.QDy[104] + nmpcWorkspace.evGx[783]*nmpcWorkspace.QDy[105] + nmpcWorkspace.evGx[791]*nmpcWorkspace.QDy[106] + nmpcWorkspace.evGx[799]*nmpcWorkspace.QDy[107] + nmpcWorkspace.evGx[807]*nmpcWorkspace.QDy[108] + nmpcWorkspace.evGx[815]*nmpcWorkspace.QDy[109] + nmpcWorkspace.evGx[823]*nmpcWorkspace.QDy[110] + nmpcWorkspace.evGx[831]*nmpcWorkspace.QDy[111] + nmpcWorkspace.evGx[839]*nmpcWorkspace.QDy[112] + nmpcWorkspace.evGx[847]*nmpcWorkspace.QDy[113] + nmpcWorkspace.evGx[855]*nmpcWorkspace.QDy[114] + nmpcWorkspace.evGx[863]*nmpcWorkspace.QDy[115] + nmpcWorkspace.evGx[871]*nmpcWorkspace.QDy[116] + nmpcWorkspace.evGx[879]*nmpcWorkspace.QDy[117] + nmpcWorkspace.evGx[887]*nmpcWorkspace.QDy[118] + nmpcWorkspace.evGx[895]*nmpcWorkspace.QDy[119] + nmpcWorkspace.evGx[903]*nmpcWorkspace.QDy[120] + nmpcWorkspace.evGx[911]*nmpcWorkspace.QDy[121] + nmpcWorkspace.evGx[919]*nmpcWorkspace.QDy[122] + nmpcWorkspace.evGx[927]*nmpcWorkspace.QDy[123] + nmpcWorkspace.evGx[935]*nmpcWorkspace.QDy[124] + nmpcWorkspace.evGx[943]*nmpcWorkspace.QDy[125] + nmpcWorkspace.evGx[951]*nmpcWorkspace.QDy[126] + nmpcWorkspace.evGx[959]*nmpcWorkspace.QDy[127] + nmpcWorkspace.evGx[967]*nmpcWorkspace.QDy[128] + nmpcWorkspace.evGx[975]*nmpcWorkspace.QDy[129] + nmpcWorkspace.evGx[983]*nmpcWorkspace.QDy[130] + nmpcWorkspace.evGx[991]*nmpcWorkspace.QDy[131] + nmpcWorkspace.evGx[999]*nmpcWorkspace.QDy[132] + nmpcWorkspace.evGx[1007]*nmpcWorkspace.QDy[133] + nmpcWorkspace.evGx[1015]*nmpcWorkspace.QDy[134] + nmpcWorkspace.evGx[1023]*nmpcWorkspace.QDy[135] + nmpcWorkspace.evGx[1031]*nmpcWorkspace.QDy[136] + nmpcWorkspace.evGx[1039]*nmpcWorkspace.QDy[137] + nmpcWorkspace.evGx[1047]*nmpcWorkspace.QDy[138] + nmpcWorkspace.evGx[1055]*nmpcWorkspace.QDy[139] + nmpcWorkspace.evGx[1063]*nmpcWorkspace.QDy[140] + nmpcWorkspace.evGx[1071]*nmpcWorkspace.QDy[141] + nmpcWorkspace.evGx[1079]*nmpcWorkspace.QDy[142] + nmpcWorkspace.evGx[1087]*nmpcWorkspace.QDy[143] + nmpcWorkspace.evGx[1095]*nmpcWorkspace.QDy[144] + nmpcWorkspace.evGx[1103]*nmpcWorkspace.QDy[145] + nmpcWorkspace.evGx[1111]*nmpcWorkspace.QDy[146] + nmpcWorkspace.evGx[1119]*nmpcWorkspace.QDy[147] + nmpcWorkspace.evGx[1127]*nmpcWorkspace.QDy[148] + nmpcWorkspace.evGx[1135]*nmpcWorkspace.QDy[149] + nmpcWorkspace.evGx[1143]*nmpcWorkspace.QDy[150] + nmpcWorkspace.evGx[1151]*nmpcWorkspace.QDy[151] + nmpcWorkspace.evGx[1159]*nmpcWorkspace.QDy[152] + nmpcWorkspace.evGx[1167]*nmpcWorkspace.QDy[153] + nmpcWorkspace.evGx[1175]*nmpcWorkspace.QDy[154] + nmpcWorkspace.evGx[1183]*nmpcWorkspace.QDy[155] + nmpcWorkspace.evGx[1191]*nmpcWorkspace.QDy[156] + nmpcWorkspace.evGx[1199]*nmpcWorkspace.QDy[157] + nmpcWorkspace.evGx[1207]*nmpcWorkspace.QDy[158] + nmpcWorkspace.evGx[1215]*nmpcWorkspace.QDy[159] + nmpcWorkspace.evGx[1223]*nmpcWorkspace.QDy[160] + nmpcWorkspace.evGx[1231]*nmpcWorkspace.QDy[161] + nmpcWorkspace.evGx[1239]*nmpcWorkspace.QDy[162] + nmpcWorkspace.evGx[1247]*nmpcWorkspace.QDy[163] + nmpcWorkspace.evGx[1255]*nmpcWorkspace.QDy[164] + nmpcWorkspace.evGx[1263]*nmpcWorkspace.QDy[165] + nmpcWorkspace.evGx[1271]*nmpcWorkspace.QDy[166] + nmpcWorkspace.evGx[1279]*nmpcWorkspace.QDy[167] + nmpcWorkspace.evGx[1287]*nmpcWorkspace.QDy[168] + nmpcWorkspace.evGx[1295]*nmpcWorkspace.QDy[169] + nmpcWorkspace.evGx[1303]*nmpcWorkspace.QDy[170] + nmpcWorkspace.evGx[1311]*nmpcWorkspace.QDy[171] + nmpcWorkspace.evGx[1319]*nmpcWorkspace.QDy[172] + nmpcWorkspace.evGx[1327]*nmpcWorkspace.QDy[173] + nmpcWorkspace.evGx[1335]*nmpcWorkspace.QDy[174] + nmpcWorkspace.evGx[1343]*nmpcWorkspace.QDy[175] + nmpcWorkspace.evGx[1351]*nmpcWorkspace.QDy[176] + nmpcWorkspace.evGx[1359]*nmpcWorkspace.QDy[177] + nmpcWorkspace.evGx[1367]*nmpcWorkspace.QDy[178] + nmpcWorkspace.evGx[1375]*nmpcWorkspace.QDy[179] + nmpcWorkspace.evGx[1383]*nmpcWorkspace.QDy[180] + nmpcWorkspace.evGx[1391]*nmpcWorkspace.QDy[181] + nmpcWorkspace.evGx[1399]*nmpcWorkspace.QDy[182] + nmpcWorkspace.evGx[1407]*nmpcWorkspace.QDy[183] + nmpcWorkspace.evGx[1415]*nmpcWorkspace.QDy[184] + nmpcWorkspace.evGx[1423]*nmpcWorkspace.QDy[185] + nmpcWorkspace.evGx[1431]*nmpcWorkspace.QDy[186] + nmpcWorkspace.evGx[1439]*nmpcWorkspace.QDy[187] + nmpcWorkspace.evGx[1447]*nmpcWorkspace.QDy[188] + nmpcWorkspace.evGx[1455]*nmpcWorkspace.QDy[189] + nmpcWorkspace.evGx[1463]*nmpcWorkspace.QDy[190] + nmpcWorkspace.evGx[1471]*nmpcWorkspace.QDy[191] + nmpcWorkspace.evGx[1479]*nmpcWorkspace.QDy[192] + nmpcWorkspace.evGx[1487]*nmpcWorkspace.QDy[193] + nmpcWorkspace.evGx[1495]*nmpcWorkspace.QDy[194] + nmpcWorkspace.evGx[1503]*nmpcWorkspace.QDy[195] + nmpcWorkspace.evGx[1511]*nmpcWorkspace.QDy[196] + nmpcWorkspace.evGx[1519]*nmpcWorkspace.QDy[197] + nmpcWorkspace.evGx[1527]*nmpcWorkspace.QDy[198] + nmpcWorkspace.evGx[1535]*nmpcWorkspace.QDy[199] + nmpcWorkspace.evGx[1543]*nmpcWorkspace.QDy[200] + nmpcWorkspace.evGx[1551]*nmpcWorkspace.QDy[201] + nmpcWorkspace.evGx[1559]*nmpcWorkspace.QDy[202] + nmpcWorkspace.evGx[1567]*nmpcWorkspace.QDy[203] + nmpcWorkspace.evGx[1575]*nmpcWorkspace.QDy[204] + nmpcWorkspace.evGx[1583]*nmpcWorkspace.QDy[205] + nmpcWorkspace.evGx[1591]*nmpcWorkspace.QDy[206] + nmpcWorkspace.evGx[1599]*nmpcWorkspace.QDy[207] + nmpcWorkspace.evGx[1607]*nmpcWorkspace.QDy[208] + nmpcWorkspace.evGx[1615]*nmpcWorkspace.QDy[209] + nmpcWorkspace.evGx[1623]*nmpcWorkspace.QDy[210] + nmpcWorkspace.evGx[1631]*nmpcWorkspace.QDy[211] + nmpcWorkspace.evGx[1639]*nmpcWorkspace.QDy[212] + nmpcWorkspace.evGx[1647]*nmpcWorkspace.QDy[213] + nmpcWorkspace.evGx[1655]*nmpcWorkspace.QDy[214] + nmpcWorkspace.evGx[1663]*nmpcWorkspace.QDy[215] + nmpcWorkspace.evGx[1671]*nmpcWorkspace.QDy[216] + nmpcWorkspace.evGx[1679]*nmpcWorkspace.QDy[217] + nmpcWorkspace.evGx[1687]*nmpcWorkspace.QDy[218] + nmpcWorkspace.evGx[1695]*nmpcWorkspace.QDy[219] + nmpcWorkspace.evGx[1703]*nmpcWorkspace.QDy[220] + nmpcWorkspace.evGx[1711]*nmpcWorkspace.QDy[221] + nmpcWorkspace.evGx[1719]*nmpcWorkspace.QDy[222] + nmpcWorkspace.evGx[1727]*nmpcWorkspace.QDy[223] + nmpcWorkspace.evGx[1735]*nmpcWorkspace.QDy[224] + nmpcWorkspace.evGx[1743]*nmpcWorkspace.QDy[225] + nmpcWorkspace.evGx[1751]*nmpcWorkspace.QDy[226] + nmpcWorkspace.evGx[1759]*nmpcWorkspace.QDy[227] + nmpcWorkspace.evGx[1767]*nmpcWorkspace.QDy[228] + nmpcWorkspace.evGx[1775]*nmpcWorkspace.QDy[229] + nmpcWorkspace.evGx[1783]*nmpcWorkspace.QDy[230] + nmpcWorkspace.evGx[1791]*nmpcWorkspace.QDy[231] + nmpcWorkspace.evGx[1799]*nmpcWorkspace.QDy[232] + nmpcWorkspace.evGx[1807]*nmpcWorkspace.QDy[233] + nmpcWorkspace.evGx[1815]*nmpcWorkspace.QDy[234] + nmpcWorkspace.evGx[1823]*nmpcWorkspace.QDy[235] + nmpcWorkspace.evGx[1831]*nmpcWorkspace.QDy[236] + nmpcWorkspace.evGx[1839]*nmpcWorkspace.QDy[237] + nmpcWorkspace.evGx[1847]*nmpcWorkspace.QDy[238] + nmpcWorkspace.evGx[1855]*nmpcWorkspace.QDy[239] + nmpcWorkspace.evGx[1863]*nmpcWorkspace.QDy[240] + nmpcWorkspace.evGx[1871]*nmpcWorkspace.QDy[241] + nmpcWorkspace.evGx[1879]*nmpcWorkspace.QDy[242] + nmpcWorkspace.evGx[1887]*nmpcWorkspace.QDy[243] + nmpcWorkspace.evGx[1895]*nmpcWorkspace.QDy[244] + nmpcWorkspace.evGx[1903]*nmpcWorkspace.QDy[245] + nmpcWorkspace.evGx[1911]*nmpcWorkspace.QDy[246] + nmpcWorkspace.evGx[1919]*nmpcWorkspace.QDy[247];


for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = lRun1; lRun2 < 30; ++lRun2)
{
lRun3 = (((lRun2 + 1) * (lRun2)) / (2)) + (lRun1);
nmpc_multEQDy( &(nmpcWorkspace.E[ lRun3 * 32 ]), &(nmpcWorkspace.QDy[ lRun2 * 8 + 8 ]), &(nmpcWorkspace.g[ lRun1 * 4 + 8 ]) );
}
}

nmpcWorkspace.lb[0] = nmpcWorkspace.Dx0[0];
nmpcWorkspace.lb[1] = nmpcWorkspace.Dx0[1];
nmpcWorkspace.lb[2] = nmpcWorkspace.Dx0[2];
nmpcWorkspace.lb[3] = nmpcWorkspace.Dx0[3];
nmpcWorkspace.lb[4] = nmpcWorkspace.Dx0[4];
nmpcWorkspace.lb[5] = nmpcWorkspace.Dx0[5];
nmpcWorkspace.lb[6] = nmpcWorkspace.Dx0[6];
nmpcWorkspace.lb[7] = nmpcWorkspace.Dx0[7];
nmpcWorkspace.ub[0] = nmpcWorkspace.Dx0[0];
nmpcWorkspace.ub[1] = nmpcWorkspace.Dx0[1];
nmpcWorkspace.ub[2] = nmpcWorkspace.Dx0[2];
nmpcWorkspace.ub[3] = nmpcWorkspace.Dx0[3];
nmpcWorkspace.ub[4] = nmpcWorkspace.Dx0[4];
nmpcWorkspace.ub[5] = nmpcWorkspace.Dx0[5];
nmpcWorkspace.ub[6] = nmpcWorkspace.Dx0[6];
nmpcWorkspace.ub[7] = nmpcWorkspace.Dx0[7];
}

void nmpc_expand(  )
{
int lRun1;
int lRun2;
int lRun3;
nmpcVariables.x[0] += nmpcWorkspace.x[0];
nmpcVariables.x[1] += nmpcWorkspace.x[1];
nmpcVariables.x[2] += nmpcWorkspace.x[2];
nmpcVariables.x[3] += nmpcWorkspace.x[3];
nmpcVariables.x[4] += nmpcWorkspace.x[4];
nmpcVariables.x[5] += nmpcWorkspace.x[5];
nmpcVariables.x[6] += nmpcWorkspace.x[6];
nmpcVariables.x[7] += nmpcWorkspace.x[7];

nmpcVariables.u[0] += nmpcWorkspace.x[8];
nmpcVariables.u[1] += nmpcWorkspace.x[9];
nmpcVariables.u[2] += nmpcWorkspace.x[10];
nmpcVariables.u[3] += nmpcWorkspace.x[11];
nmpcVariables.u[4] += nmpcWorkspace.x[12];
nmpcVariables.u[5] += nmpcWorkspace.x[13];
nmpcVariables.u[6] += nmpcWorkspace.x[14];
nmpcVariables.u[7] += nmpcWorkspace.x[15];
nmpcVariables.u[8] += nmpcWorkspace.x[16];
nmpcVariables.u[9] += nmpcWorkspace.x[17];
nmpcVariables.u[10] += nmpcWorkspace.x[18];
nmpcVariables.u[11] += nmpcWorkspace.x[19];
nmpcVariables.u[12] += nmpcWorkspace.x[20];
nmpcVariables.u[13] += nmpcWorkspace.x[21];
nmpcVariables.u[14] += nmpcWorkspace.x[22];
nmpcVariables.u[15] += nmpcWorkspace.x[23];
nmpcVariables.u[16] += nmpcWorkspace.x[24];
nmpcVariables.u[17] += nmpcWorkspace.x[25];
nmpcVariables.u[18] += nmpcWorkspace.x[26];
nmpcVariables.u[19] += nmpcWorkspace.x[27];
nmpcVariables.u[20] += nmpcWorkspace.x[28];
nmpcVariables.u[21] += nmpcWorkspace.x[29];
nmpcVariables.u[22] += nmpcWorkspace.x[30];
nmpcVariables.u[23] += nmpcWorkspace.x[31];
nmpcVariables.u[24] += nmpcWorkspace.x[32];
nmpcVariables.u[25] += nmpcWorkspace.x[33];
nmpcVariables.u[26] += nmpcWorkspace.x[34];
nmpcVariables.u[27] += nmpcWorkspace.x[35];
nmpcVariables.u[28] += nmpcWorkspace.x[36];
nmpcVariables.u[29] += nmpcWorkspace.x[37];
nmpcVariables.u[30] += nmpcWorkspace.x[38];
nmpcVariables.u[31] += nmpcWorkspace.x[39];
nmpcVariables.u[32] += nmpcWorkspace.x[40];
nmpcVariables.u[33] += nmpcWorkspace.x[41];
nmpcVariables.u[34] += nmpcWorkspace.x[42];
nmpcVariables.u[35] += nmpcWorkspace.x[43];
nmpcVariables.u[36] += nmpcWorkspace.x[44];
nmpcVariables.u[37] += nmpcWorkspace.x[45];
nmpcVariables.u[38] += nmpcWorkspace.x[46];
nmpcVariables.u[39] += nmpcWorkspace.x[47];
nmpcVariables.u[40] += nmpcWorkspace.x[48];
nmpcVariables.u[41] += nmpcWorkspace.x[49];
nmpcVariables.u[42] += nmpcWorkspace.x[50];
nmpcVariables.u[43] += nmpcWorkspace.x[51];
nmpcVariables.u[44] += nmpcWorkspace.x[52];
nmpcVariables.u[45] += nmpcWorkspace.x[53];
nmpcVariables.u[46] += nmpcWorkspace.x[54];
nmpcVariables.u[47] += nmpcWorkspace.x[55];
nmpcVariables.u[48] += nmpcWorkspace.x[56];
nmpcVariables.u[49] += nmpcWorkspace.x[57];
nmpcVariables.u[50] += nmpcWorkspace.x[58];
nmpcVariables.u[51] += nmpcWorkspace.x[59];
nmpcVariables.u[52] += nmpcWorkspace.x[60];
nmpcVariables.u[53] += nmpcWorkspace.x[61];
nmpcVariables.u[54] += nmpcWorkspace.x[62];
nmpcVariables.u[55] += nmpcWorkspace.x[63];
nmpcVariables.u[56] += nmpcWorkspace.x[64];
nmpcVariables.u[57] += nmpcWorkspace.x[65];
nmpcVariables.u[58] += nmpcWorkspace.x[66];
nmpcVariables.u[59] += nmpcWorkspace.x[67];
nmpcVariables.u[60] += nmpcWorkspace.x[68];
nmpcVariables.u[61] += nmpcWorkspace.x[69];
nmpcVariables.u[62] += nmpcWorkspace.x[70];
nmpcVariables.u[63] += nmpcWorkspace.x[71];
nmpcVariables.u[64] += nmpcWorkspace.x[72];
nmpcVariables.u[65] += nmpcWorkspace.x[73];
nmpcVariables.u[66] += nmpcWorkspace.x[74];
nmpcVariables.u[67] += nmpcWorkspace.x[75];
nmpcVariables.u[68] += nmpcWorkspace.x[76];
nmpcVariables.u[69] += nmpcWorkspace.x[77];
nmpcVariables.u[70] += nmpcWorkspace.x[78];
nmpcVariables.u[71] += nmpcWorkspace.x[79];
nmpcVariables.u[72] += nmpcWorkspace.x[80];
nmpcVariables.u[73] += nmpcWorkspace.x[81];
nmpcVariables.u[74] += nmpcWorkspace.x[82];
nmpcVariables.u[75] += nmpcWorkspace.x[83];
nmpcVariables.u[76] += nmpcWorkspace.x[84];
nmpcVariables.u[77] += nmpcWorkspace.x[85];
nmpcVariables.u[78] += nmpcWorkspace.x[86];
nmpcVariables.u[79] += nmpcWorkspace.x[87];
nmpcVariables.u[80] += nmpcWorkspace.x[88];
nmpcVariables.u[81] += nmpcWorkspace.x[89];
nmpcVariables.u[82] += nmpcWorkspace.x[90];
nmpcVariables.u[83] += nmpcWorkspace.x[91];
nmpcVariables.u[84] += nmpcWorkspace.x[92];
nmpcVariables.u[85] += nmpcWorkspace.x[93];
nmpcVariables.u[86] += nmpcWorkspace.x[94];
nmpcVariables.u[87] += nmpcWorkspace.x[95];
nmpcVariables.u[88] += nmpcWorkspace.x[96];
nmpcVariables.u[89] += nmpcWorkspace.x[97];
nmpcVariables.u[90] += nmpcWorkspace.x[98];
nmpcVariables.u[91] += nmpcWorkspace.x[99];
nmpcVariables.u[92] += nmpcWorkspace.x[100];
nmpcVariables.u[93] += nmpcWorkspace.x[101];
nmpcVariables.u[94] += nmpcWorkspace.x[102];
nmpcVariables.u[95] += nmpcWorkspace.x[103];
nmpcVariables.u[96] += nmpcWorkspace.x[104];
nmpcVariables.u[97] += nmpcWorkspace.x[105];
nmpcVariables.u[98] += nmpcWorkspace.x[106];
nmpcVariables.u[99] += nmpcWorkspace.x[107];
nmpcVariables.u[100] += nmpcWorkspace.x[108];
nmpcVariables.u[101] += nmpcWorkspace.x[109];
nmpcVariables.u[102] += nmpcWorkspace.x[110];
nmpcVariables.u[103] += nmpcWorkspace.x[111];
nmpcVariables.u[104] += nmpcWorkspace.x[112];
nmpcVariables.u[105] += nmpcWorkspace.x[113];
nmpcVariables.u[106] += nmpcWorkspace.x[114];
nmpcVariables.u[107] += nmpcWorkspace.x[115];
nmpcVariables.u[108] += nmpcWorkspace.x[116];
nmpcVariables.u[109] += nmpcWorkspace.x[117];
nmpcVariables.u[110] += nmpcWorkspace.x[118];
nmpcVariables.u[111] += nmpcWorkspace.x[119];
nmpcVariables.u[112] += nmpcWorkspace.x[120];
nmpcVariables.u[113] += nmpcWorkspace.x[121];
nmpcVariables.u[114] += nmpcWorkspace.x[122];
nmpcVariables.u[115] += nmpcWorkspace.x[123];
nmpcVariables.u[116] += nmpcWorkspace.x[124];
nmpcVariables.u[117] += nmpcWorkspace.x[125];
nmpcVariables.u[118] += nmpcWorkspace.x[126];
nmpcVariables.u[119] += nmpcWorkspace.x[127];

nmpcVariables.x[8] += + nmpcWorkspace.evGx[0]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[2]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[3]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[4]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[5]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[6]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[7]*nmpcWorkspace.x[7] + nmpcWorkspace.d[0];
nmpcVariables.x[9] += + nmpcWorkspace.evGx[8]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[9]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[10]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[11]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[12]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[13]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[14]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[15]*nmpcWorkspace.x[7] + nmpcWorkspace.d[1];
nmpcVariables.x[10] += + nmpcWorkspace.evGx[16]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[17]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[18]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[19]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[20]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[21]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[22]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[23]*nmpcWorkspace.x[7] + nmpcWorkspace.d[2];
nmpcVariables.x[11] += + nmpcWorkspace.evGx[24]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[25]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[26]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[27]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[28]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[29]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[30]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[31]*nmpcWorkspace.x[7] + nmpcWorkspace.d[3];
nmpcVariables.x[12] += + nmpcWorkspace.evGx[32]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[33]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[34]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[35]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[36]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[37]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[38]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[39]*nmpcWorkspace.x[7] + nmpcWorkspace.d[4];
nmpcVariables.x[13] += + nmpcWorkspace.evGx[40]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[41]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[42]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[43]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[44]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[45]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[46]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[47]*nmpcWorkspace.x[7] + nmpcWorkspace.d[5];
nmpcVariables.x[14] += + nmpcWorkspace.evGx[48]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[49]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[50]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[51]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[52]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[53]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[54]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[55]*nmpcWorkspace.x[7] + nmpcWorkspace.d[6];
nmpcVariables.x[15] += + nmpcWorkspace.evGx[56]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[57]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[58]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[59]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[60]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[61]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[62]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[63]*nmpcWorkspace.x[7] + nmpcWorkspace.d[7];
nmpcVariables.x[16] += + nmpcWorkspace.evGx[64]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[65]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[66]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[67]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[68]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[69]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[70]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[71]*nmpcWorkspace.x[7] + nmpcWorkspace.d[8];
nmpcVariables.x[17] += + nmpcWorkspace.evGx[72]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[73]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[74]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[75]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[76]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[77]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[78]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[79]*nmpcWorkspace.x[7] + nmpcWorkspace.d[9];
nmpcVariables.x[18] += + nmpcWorkspace.evGx[80]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[81]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[82]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[83]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[84]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[85]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[86]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[87]*nmpcWorkspace.x[7] + nmpcWorkspace.d[10];
nmpcVariables.x[19] += + nmpcWorkspace.evGx[88]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[89]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[90]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[91]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[92]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[93]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[94]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[95]*nmpcWorkspace.x[7] + nmpcWorkspace.d[11];
nmpcVariables.x[20] += + nmpcWorkspace.evGx[96]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[97]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[98]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[99]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[100]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[101]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[102]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[103]*nmpcWorkspace.x[7] + nmpcWorkspace.d[12];
nmpcVariables.x[21] += + nmpcWorkspace.evGx[104]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[105]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[106]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[107]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[108]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[109]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[110]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[111]*nmpcWorkspace.x[7] + nmpcWorkspace.d[13];
nmpcVariables.x[22] += + nmpcWorkspace.evGx[112]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[113]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[114]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[115]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[116]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[117]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[118]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[119]*nmpcWorkspace.x[7] + nmpcWorkspace.d[14];
nmpcVariables.x[23] += + nmpcWorkspace.evGx[120]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[121]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[122]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[123]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[124]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[125]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[126]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[127]*nmpcWorkspace.x[7] + nmpcWorkspace.d[15];
nmpcVariables.x[24] += + nmpcWorkspace.evGx[128]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[129]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[130]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[131]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[132]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[133]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[134]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[135]*nmpcWorkspace.x[7] + nmpcWorkspace.d[16];
nmpcVariables.x[25] += + nmpcWorkspace.evGx[136]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[137]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[138]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[139]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[140]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[141]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[142]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[143]*nmpcWorkspace.x[7] + nmpcWorkspace.d[17];
nmpcVariables.x[26] += + nmpcWorkspace.evGx[144]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[145]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[146]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[147]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[148]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[149]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[150]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[151]*nmpcWorkspace.x[7] + nmpcWorkspace.d[18];
nmpcVariables.x[27] += + nmpcWorkspace.evGx[152]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[153]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[154]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[155]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[156]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[157]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[158]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[159]*nmpcWorkspace.x[7] + nmpcWorkspace.d[19];
nmpcVariables.x[28] += + nmpcWorkspace.evGx[160]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[161]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[162]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[163]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[164]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[165]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[166]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[167]*nmpcWorkspace.x[7] + nmpcWorkspace.d[20];
nmpcVariables.x[29] += + nmpcWorkspace.evGx[168]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[169]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[170]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[171]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[172]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[173]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[174]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[175]*nmpcWorkspace.x[7] + nmpcWorkspace.d[21];
nmpcVariables.x[30] += + nmpcWorkspace.evGx[176]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[177]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[178]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[179]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[180]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[181]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[182]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[183]*nmpcWorkspace.x[7] + nmpcWorkspace.d[22];
nmpcVariables.x[31] += + nmpcWorkspace.evGx[184]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[185]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[186]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[187]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[188]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[189]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[190]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[191]*nmpcWorkspace.x[7] + nmpcWorkspace.d[23];
nmpcVariables.x[32] += + nmpcWorkspace.evGx[192]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[193]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[194]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[195]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[196]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[197]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[198]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[199]*nmpcWorkspace.x[7] + nmpcWorkspace.d[24];
nmpcVariables.x[33] += + nmpcWorkspace.evGx[200]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[201]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[202]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[203]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[204]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[205]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[206]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[207]*nmpcWorkspace.x[7] + nmpcWorkspace.d[25];
nmpcVariables.x[34] += + nmpcWorkspace.evGx[208]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[209]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[210]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[211]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[212]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[213]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[214]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[215]*nmpcWorkspace.x[7] + nmpcWorkspace.d[26];
nmpcVariables.x[35] += + nmpcWorkspace.evGx[216]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[217]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[218]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[219]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[220]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[221]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[222]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[223]*nmpcWorkspace.x[7] + nmpcWorkspace.d[27];
nmpcVariables.x[36] += + nmpcWorkspace.evGx[224]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[225]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[226]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[227]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[228]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[229]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[230]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[231]*nmpcWorkspace.x[7] + nmpcWorkspace.d[28];
nmpcVariables.x[37] += + nmpcWorkspace.evGx[232]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[233]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[234]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[235]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[236]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[237]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[238]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[239]*nmpcWorkspace.x[7] + nmpcWorkspace.d[29];
nmpcVariables.x[38] += + nmpcWorkspace.evGx[240]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[241]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[242]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[243]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[244]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[245]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[246]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[247]*nmpcWorkspace.x[7] + nmpcWorkspace.d[30];
nmpcVariables.x[39] += + nmpcWorkspace.evGx[248]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[249]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[250]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[251]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[252]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[253]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[254]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[255]*nmpcWorkspace.x[7] + nmpcWorkspace.d[31];
nmpcVariables.x[40] += + nmpcWorkspace.evGx[256]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[257]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[258]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[259]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[260]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[261]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[262]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[263]*nmpcWorkspace.x[7] + nmpcWorkspace.d[32];
nmpcVariables.x[41] += + nmpcWorkspace.evGx[264]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[265]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[266]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[267]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[268]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[269]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[270]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[271]*nmpcWorkspace.x[7] + nmpcWorkspace.d[33];
nmpcVariables.x[42] += + nmpcWorkspace.evGx[272]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[273]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[274]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[275]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[276]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[277]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[278]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[279]*nmpcWorkspace.x[7] + nmpcWorkspace.d[34];
nmpcVariables.x[43] += + nmpcWorkspace.evGx[280]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[281]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[282]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[283]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[284]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[285]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[286]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[287]*nmpcWorkspace.x[7] + nmpcWorkspace.d[35];
nmpcVariables.x[44] += + nmpcWorkspace.evGx[288]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[289]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[290]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[291]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[292]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[293]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[294]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[295]*nmpcWorkspace.x[7] + nmpcWorkspace.d[36];
nmpcVariables.x[45] += + nmpcWorkspace.evGx[296]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[297]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[298]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[299]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[300]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[301]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[302]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[303]*nmpcWorkspace.x[7] + nmpcWorkspace.d[37];
nmpcVariables.x[46] += + nmpcWorkspace.evGx[304]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[305]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[306]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[307]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[308]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[309]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[310]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[311]*nmpcWorkspace.x[7] + nmpcWorkspace.d[38];
nmpcVariables.x[47] += + nmpcWorkspace.evGx[312]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[313]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[314]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[315]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[316]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[317]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[318]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[319]*nmpcWorkspace.x[7] + nmpcWorkspace.d[39];
nmpcVariables.x[48] += + nmpcWorkspace.evGx[320]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[321]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[322]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[323]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[324]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[325]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[326]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[327]*nmpcWorkspace.x[7] + nmpcWorkspace.d[40];
nmpcVariables.x[49] += + nmpcWorkspace.evGx[328]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[329]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[330]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[331]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[332]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[333]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[334]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[335]*nmpcWorkspace.x[7] + nmpcWorkspace.d[41];
nmpcVariables.x[50] += + nmpcWorkspace.evGx[336]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[337]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[338]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[339]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[340]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[341]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[342]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[343]*nmpcWorkspace.x[7] + nmpcWorkspace.d[42];
nmpcVariables.x[51] += + nmpcWorkspace.evGx[344]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[345]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[346]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[347]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[348]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[349]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[350]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[351]*nmpcWorkspace.x[7] + nmpcWorkspace.d[43];
nmpcVariables.x[52] += + nmpcWorkspace.evGx[352]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[353]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[354]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[355]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[356]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[357]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[358]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[359]*nmpcWorkspace.x[7] + nmpcWorkspace.d[44];
nmpcVariables.x[53] += + nmpcWorkspace.evGx[360]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[361]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[362]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[363]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[364]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[365]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[366]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[367]*nmpcWorkspace.x[7] + nmpcWorkspace.d[45];
nmpcVariables.x[54] += + nmpcWorkspace.evGx[368]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[369]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[370]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[371]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[372]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[373]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[374]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[375]*nmpcWorkspace.x[7] + nmpcWorkspace.d[46];
nmpcVariables.x[55] += + nmpcWorkspace.evGx[376]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[377]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[378]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[379]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[380]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[381]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[382]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[383]*nmpcWorkspace.x[7] + nmpcWorkspace.d[47];
nmpcVariables.x[56] += + nmpcWorkspace.evGx[384]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[385]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[386]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[387]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[388]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[389]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[390]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[391]*nmpcWorkspace.x[7] + nmpcWorkspace.d[48];
nmpcVariables.x[57] += + nmpcWorkspace.evGx[392]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[393]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[394]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[395]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[396]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[397]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[398]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[399]*nmpcWorkspace.x[7] + nmpcWorkspace.d[49];
nmpcVariables.x[58] += + nmpcWorkspace.evGx[400]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[401]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[402]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[403]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[404]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[405]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[406]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[407]*nmpcWorkspace.x[7] + nmpcWorkspace.d[50];
nmpcVariables.x[59] += + nmpcWorkspace.evGx[408]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[409]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[410]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[411]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[412]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[413]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[414]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[415]*nmpcWorkspace.x[7] + nmpcWorkspace.d[51];
nmpcVariables.x[60] += + nmpcWorkspace.evGx[416]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[417]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[418]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[419]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[420]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[421]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[422]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[423]*nmpcWorkspace.x[7] + nmpcWorkspace.d[52];
nmpcVariables.x[61] += + nmpcWorkspace.evGx[424]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[425]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[426]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[427]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[428]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[429]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[430]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[431]*nmpcWorkspace.x[7] + nmpcWorkspace.d[53];
nmpcVariables.x[62] += + nmpcWorkspace.evGx[432]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[433]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[434]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[435]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[436]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[437]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[438]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[439]*nmpcWorkspace.x[7] + nmpcWorkspace.d[54];
nmpcVariables.x[63] += + nmpcWorkspace.evGx[440]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[441]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[442]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[443]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[444]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[445]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[446]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[447]*nmpcWorkspace.x[7] + nmpcWorkspace.d[55];
nmpcVariables.x[64] += + nmpcWorkspace.evGx[448]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[449]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[450]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[451]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[452]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[453]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[454]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[455]*nmpcWorkspace.x[7] + nmpcWorkspace.d[56];
nmpcVariables.x[65] += + nmpcWorkspace.evGx[456]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[457]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[458]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[459]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[460]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[461]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[462]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[463]*nmpcWorkspace.x[7] + nmpcWorkspace.d[57];
nmpcVariables.x[66] += + nmpcWorkspace.evGx[464]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[465]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[466]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[467]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[468]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[469]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[470]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[471]*nmpcWorkspace.x[7] + nmpcWorkspace.d[58];
nmpcVariables.x[67] += + nmpcWorkspace.evGx[472]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[473]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[474]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[475]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[476]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[477]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[478]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[479]*nmpcWorkspace.x[7] + nmpcWorkspace.d[59];
nmpcVariables.x[68] += + nmpcWorkspace.evGx[480]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[481]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[482]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[483]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[484]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[485]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[486]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[487]*nmpcWorkspace.x[7] + nmpcWorkspace.d[60];
nmpcVariables.x[69] += + nmpcWorkspace.evGx[488]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[489]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[490]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[491]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[492]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[493]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[494]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[495]*nmpcWorkspace.x[7] + nmpcWorkspace.d[61];
nmpcVariables.x[70] += + nmpcWorkspace.evGx[496]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[497]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[498]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[499]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[500]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[501]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[502]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[503]*nmpcWorkspace.x[7] + nmpcWorkspace.d[62];
nmpcVariables.x[71] += + nmpcWorkspace.evGx[504]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[505]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[506]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[507]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[508]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[509]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[510]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[511]*nmpcWorkspace.x[7] + nmpcWorkspace.d[63];
nmpcVariables.x[72] += + nmpcWorkspace.evGx[512]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[513]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[514]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[515]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[516]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[517]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[518]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[519]*nmpcWorkspace.x[7] + nmpcWorkspace.d[64];
nmpcVariables.x[73] += + nmpcWorkspace.evGx[520]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[521]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[522]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[523]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[524]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[525]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[526]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[527]*nmpcWorkspace.x[7] + nmpcWorkspace.d[65];
nmpcVariables.x[74] += + nmpcWorkspace.evGx[528]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[529]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[530]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[531]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[532]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[533]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[534]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[535]*nmpcWorkspace.x[7] + nmpcWorkspace.d[66];
nmpcVariables.x[75] += + nmpcWorkspace.evGx[536]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[537]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[538]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[539]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[540]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[541]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[542]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[543]*nmpcWorkspace.x[7] + nmpcWorkspace.d[67];
nmpcVariables.x[76] += + nmpcWorkspace.evGx[544]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[545]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[546]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[547]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[548]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[549]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[550]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[551]*nmpcWorkspace.x[7] + nmpcWorkspace.d[68];
nmpcVariables.x[77] += + nmpcWorkspace.evGx[552]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[553]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[554]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[555]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[556]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[557]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[558]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[559]*nmpcWorkspace.x[7] + nmpcWorkspace.d[69];
nmpcVariables.x[78] += + nmpcWorkspace.evGx[560]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[561]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[562]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[563]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[564]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[565]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[566]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[567]*nmpcWorkspace.x[7] + nmpcWorkspace.d[70];
nmpcVariables.x[79] += + nmpcWorkspace.evGx[568]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[569]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[570]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[571]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[572]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[573]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[574]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[575]*nmpcWorkspace.x[7] + nmpcWorkspace.d[71];
nmpcVariables.x[80] += + nmpcWorkspace.evGx[576]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[577]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[578]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[579]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[580]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[581]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[582]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[583]*nmpcWorkspace.x[7] + nmpcWorkspace.d[72];
nmpcVariables.x[81] += + nmpcWorkspace.evGx[584]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[585]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[586]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[587]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[588]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[589]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[590]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[591]*nmpcWorkspace.x[7] + nmpcWorkspace.d[73];
nmpcVariables.x[82] += + nmpcWorkspace.evGx[592]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[593]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[594]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[595]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[596]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[597]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[598]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[599]*nmpcWorkspace.x[7] + nmpcWorkspace.d[74];
nmpcVariables.x[83] += + nmpcWorkspace.evGx[600]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[601]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[602]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[603]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[604]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[605]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[606]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[607]*nmpcWorkspace.x[7] + nmpcWorkspace.d[75];
nmpcVariables.x[84] += + nmpcWorkspace.evGx[608]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[609]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[610]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[611]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[612]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[613]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[614]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[615]*nmpcWorkspace.x[7] + nmpcWorkspace.d[76];
nmpcVariables.x[85] += + nmpcWorkspace.evGx[616]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[617]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[618]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[619]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[620]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[621]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[622]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[623]*nmpcWorkspace.x[7] + nmpcWorkspace.d[77];
nmpcVariables.x[86] += + nmpcWorkspace.evGx[624]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[625]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[626]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[627]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[628]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[629]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[630]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[631]*nmpcWorkspace.x[7] + nmpcWorkspace.d[78];
nmpcVariables.x[87] += + nmpcWorkspace.evGx[632]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[633]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[634]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[635]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[636]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[637]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[638]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[639]*nmpcWorkspace.x[7] + nmpcWorkspace.d[79];
nmpcVariables.x[88] += + nmpcWorkspace.evGx[640]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[641]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[642]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[643]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[644]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[645]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[646]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[647]*nmpcWorkspace.x[7] + nmpcWorkspace.d[80];
nmpcVariables.x[89] += + nmpcWorkspace.evGx[648]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[649]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[650]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[651]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[652]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[653]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[654]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[655]*nmpcWorkspace.x[7] + nmpcWorkspace.d[81];
nmpcVariables.x[90] += + nmpcWorkspace.evGx[656]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[657]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[658]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[659]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[660]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[661]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[662]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[663]*nmpcWorkspace.x[7] + nmpcWorkspace.d[82];
nmpcVariables.x[91] += + nmpcWorkspace.evGx[664]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[665]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[666]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[667]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[668]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[669]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[670]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[671]*nmpcWorkspace.x[7] + nmpcWorkspace.d[83];
nmpcVariables.x[92] += + nmpcWorkspace.evGx[672]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[673]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[674]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[675]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[676]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[677]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[678]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[679]*nmpcWorkspace.x[7] + nmpcWorkspace.d[84];
nmpcVariables.x[93] += + nmpcWorkspace.evGx[680]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[681]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[682]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[683]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[684]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[685]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[686]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[687]*nmpcWorkspace.x[7] + nmpcWorkspace.d[85];
nmpcVariables.x[94] += + nmpcWorkspace.evGx[688]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[689]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[690]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[691]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[692]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[693]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[694]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[695]*nmpcWorkspace.x[7] + nmpcWorkspace.d[86];
nmpcVariables.x[95] += + nmpcWorkspace.evGx[696]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[697]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[698]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[699]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[700]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[701]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[702]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[703]*nmpcWorkspace.x[7] + nmpcWorkspace.d[87];
nmpcVariables.x[96] += + nmpcWorkspace.evGx[704]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[705]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[706]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[707]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[708]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[709]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[710]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[711]*nmpcWorkspace.x[7] + nmpcWorkspace.d[88];
nmpcVariables.x[97] += + nmpcWorkspace.evGx[712]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[713]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[714]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[715]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[716]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[717]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[718]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[719]*nmpcWorkspace.x[7] + nmpcWorkspace.d[89];
nmpcVariables.x[98] += + nmpcWorkspace.evGx[720]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[721]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[722]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[723]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[724]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[725]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[726]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[727]*nmpcWorkspace.x[7] + nmpcWorkspace.d[90];
nmpcVariables.x[99] += + nmpcWorkspace.evGx[728]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[729]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[730]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[731]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[732]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[733]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[734]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[735]*nmpcWorkspace.x[7] + nmpcWorkspace.d[91];
nmpcVariables.x[100] += + nmpcWorkspace.evGx[736]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[737]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[738]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[739]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[740]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[741]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[742]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[743]*nmpcWorkspace.x[7] + nmpcWorkspace.d[92];
nmpcVariables.x[101] += + nmpcWorkspace.evGx[744]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[745]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[746]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[747]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[748]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[749]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[750]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[751]*nmpcWorkspace.x[7] + nmpcWorkspace.d[93];
nmpcVariables.x[102] += + nmpcWorkspace.evGx[752]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[753]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[754]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[755]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[756]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[757]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[758]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[759]*nmpcWorkspace.x[7] + nmpcWorkspace.d[94];
nmpcVariables.x[103] += + nmpcWorkspace.evGx[760]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[761]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[762]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[763]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[764]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[765]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[766]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[767]*nmpcWorkspace.x[7] + nmpcWorkspace.d[95];
nmpcVariables.x[104] += + nmpcWorkspace.evGx[768]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[769]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[770]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[771]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[772]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[773]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[774]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[775]*nmpcWorkspace.x[7] + nmpcWorkspace.d[96];
nmpcVariables.x[105] += + nmpcWorkspace.evGx[776]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[777]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[778]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[779]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[780]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[781]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[782]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[783]*nmpcWorkspace.x[7] + nmpcWorkspace.d[97];
nmpcVariables.x[106] += + nmpcWorkspace.evGx[784]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[785]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[786]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[787]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[788]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[789]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[790]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[791]*nmpcWorkspace.x[7] + nmpcWorkspace.d[98];
nmpcVariables.x[107] += + nmpcWorkspace.evGx[792]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[793]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[794]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[795]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[796]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[797]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[798]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[799]*nmpcWorkspace.x[7] + nmpcWorkspace.d[99];
nmpcVariables.x[108] += + nmpcWorkspace.evGx[800]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[801]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[802]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[803]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[804]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[805]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[806]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[807]*nmpcWorkspace.x[7] + nmpcWorkspace.d[100];
nmpcVariables.x[109] += + nmpcWorkspace.evGx[808]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[809]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[810]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[811]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[812]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[813]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[814]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[815]*nmpcWorkspace.x[7] + nmpcWorkspace.d[101];
nmpcVariables.x[110] += + nmpcWorkspace.evGx[816]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[817]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[818]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[819]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[820]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[821]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[822]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[823]*nmpcWorkspace.x[7] + nmpcWorkspace.d[102];
nmpcVariables.x[111] += + nmpcWorkspace.evGx[824]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[825]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[826]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[827]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[828]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[829]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[830]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[831]*nmpcWorkspace.x[7] + nmpcWorkspace.d[103];
nmpcVariables.x[112] += + nmpcWorkspace.evGx[832]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[833]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[834]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[835]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[836]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[837]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[838]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[839]*nmpcWorkspace.x[7] + nmpcWorkspace.d[104];
nmpcVariables.x[113] += + nmpcWorkspace.evGx[840]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[841]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[842]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[843]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[844]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[845]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[846]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[847]*nmpcWorkspace.x[7] + nmpcWorkspace.d[105];
nmpcVariables.x[114] += + nmpcWorkspace.evGx[848]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[849]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[850]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[851]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[852]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[853]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[854]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[855]*nmpcWorkspace.x[7] + nmpcWorkspace.d[106];
nmpcVariables.x[115] += + nmpcWorkspace.evGx[856]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[857]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[858]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[859]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[860]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[861]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[862]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[863]*nmpcWorkspace.x[7] + nmpcWorkspace.d[107];
nmpcVariables.x[116] += + nmpcWorkspace.evGx[864]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[865]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[866]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[867]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[868]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[869]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[870]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[871]*nmpcWorkspace.x[7] + nmpcWorkspace.d[108];
nmpcVariables.x[117] += + nmpcWorkspace.evGx[872]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[873]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[874]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[875]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[876]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[877]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[878]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[879]*nmpcWorkspace.x[7] + nmpcWorkspace.d[109];
nmpcVariables.x[118] += + nmpcWorkspace.evGx[880]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[881]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[882]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[883]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[884]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[885]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[886]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[887]*nmpcWorkspace.x[7] + nmpcWorkspace.d[110];
nmpcVariables.x[119] += + nmpcWorkspace.evGx[888]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[889]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[890]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[891]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[892]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[893]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[894]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[895]*nmpcWorkspace.x[7] + nmpcWorkspace.d[111];
nmpcVariables.x[120] += + nmpcWorkspace.evGx[896]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[897]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[898]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[899]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[900]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[901]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[902]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[903]*nmpcWorkspace.x[7] + nmpcWorkspace.d[112];
nmpcVariables.x[121] += + nmpcWorkspace.evGx[904]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[905]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[906]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[907]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[908]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[909]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[910]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[911]*nmpcWorkspace.x[7] + nmpcWorkspace.d[113];
nmpcVariables.x[122] += + nmpcWorkspace.evGx[912]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[913]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[914]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[915]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[916]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[917]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[918]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[919]*nmpcWorkspace.x[7] + nmpcWorkspace.d[114];
nmpcVariables.x[123] += + nmpcWorkspace.evGx[920]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[921]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[922]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[923]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[924]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[925]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[926]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[927]*nmpcWorkspace.x[7] + nmpcWorkspace.d[115];
nmpcVariables.x[124] += + nmpcWorkspace.evGx[928]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[929]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[930]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[931]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[932]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[933]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[934]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[935]*nmpcWorkspace.x[7] + nmpcWorkspace.d[116];
nmpcVariables.x[125] += + nmpcWorkspace.evGx[936]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[937]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[938]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[939]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[940]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[941]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[942]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[943]*nmpcWorkspace.x[7] + nmpcWorkspace.d[117];
nmpcVariables.x[126] += + nmpcWorkspace.evGx[944]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[945]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[946]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[947]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[948]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[949]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[950]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[951]*nmpcWorkspace.x[7] + nmpcWorkspace.d[118];
nmpcVariables.x[127] += + nmpcWorkspace.evGx[952]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[953]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[954]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[955]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[956]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[957]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[958]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[959]*nmpcWorkspace.x[7] + nmpcWorkspace.d[119];
nmpcVariables.x[128] += + nmpcWorkspace.evGx[960]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[961]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[962]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[963]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[964]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[965]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[966]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[967]*nmpcWorkspace.x[7] + nmpcWorkspace.d[120];
nmpcVariables.x[129] += + nmpcWorkspace.evGx[968]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[969]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[970]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[971]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[972]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[973]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[974]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[975]*nmpcWorkspace.x[7] + nmpcWorkspace.d[121];
nmpcVariables.x[130] += + nmpcWorkspace.evGx[976]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[977]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[978]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[979]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[980]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[981]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[982]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[983]*nmpcWorkspace.x[7] + nmpcWorkspace.d[122];
nmpcVariables.x[131] += + nmpcWorkspace.evGx[984]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[985]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[986]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[987]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[988]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[989]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[990]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[991]*nmpcWorkspace.x[7] + nmpcWorkspace.d[123];
nmpcVariables.x[132] += + nmpcWorkspace.evGx[992]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[993]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[994]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[995]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[996]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[997]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[998]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[999]*nmpcWorkspace.x[7] + nmpcWorkspace.d[124];
nmpcVariables.x[133] += + nmpcWorkspace.evGx[1000]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1001]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1002]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1003]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1004]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1005]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1006]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1007]*nmpcWorkspace.x[7] + nmpcWorkspace.d[125];
nmpcVariables.x[134] += + nmpcWorkspace.evGx[1008]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1009]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1010]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1011]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1012]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1013]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1014]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1015]*nmpcWorkspace.x[7] + nmpcWorkspace.d[126];
nmpcVariables.x[135] += + nmpcWorkspace.evGx[1016]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1017]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1018]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1019]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1020]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1021]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1022]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1023]*nmpcWorkspace.x[7] + nmpcWorkspace.d[127];
nmpcVariables.x[136] += + nmpcWorkspace.evGx[1024]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1025]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1026]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1027]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1028]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1029]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1030]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1031]*nmpcWorkspace.x[7] + nmpcWorkspace.d[128];
nmpcVariables.x[137] += + nmpcWorkspace.evGx[1032]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1033]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1034]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1035]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1036]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1037]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1038]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1039]*nmpcWorkspace.x[7] + nmpcWorkspace.d[129];
nmpcVariables.x[138] += + nmpcWorkspace.evGx[1040]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1041]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1042]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1043]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1044]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1045]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1046]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1047]*nmpcWorkspace.x[7] + nmpcWorkspace.d[130];
nmpcVariables.x[139] += + nmpcWorkspace.evGx[1048]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1049]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1050]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1051]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1052]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1053]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1054]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1055]*nmpcWorkspace.x[7] + nmpcWorkspace.d[131];
nmpcVariables.x[140] += + nmpcWorkspace.evGx[1056]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1057]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1058]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1059]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1060]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1061]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1062]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1063]*nmpcWorkspace.x[7] + nmpcWorkspace.d[132];
nmpcVariables.x[141] += + nmpcWorkspace.evGx[1064]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1065]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1066]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1067]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1068]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1069]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1070]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1071]*nmpcWorkspace.x[7] + nmpcWorkspace.d[133];
nmpcVariables.x[142] += + nmpcWorkspace.evGx[1072]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1073]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1074]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1075]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1076]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1077]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1078]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1079]*nmpcWorkspace.x[7] + nmpcWorkspace.d[134];
nmpcVariables.x[143] += + nmpcWorkspace.evGx[1080]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1081]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1082]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1083]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1084]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1085]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1086]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1087]*nmpcWorkspace.x[7] + nmpcWorkspace.d[135];
nmpcVariables.x[144] += + nmpcWorkspace.evGx[1088]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1089]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1090]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1091]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1092]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1093]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1094]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1095]*nmpcWorkspace.x[7] + nmpcWorkspace.d[136];
nmpcVariables.x[145] += + nmpcWorkspace.evGx[1096]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1097]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1098]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1099]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1100]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1101]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1102]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1103]*nmpcWorkspace.x[7] + nmpcWorkspace.d[137];
nmpcVariables.x[146] += + nmpcWorkspace.evGx[1104]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1105]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1106]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1107]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1108]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1109]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1110]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1111]*nmpcWorkspace.x[7] + nmpcWorkspace.d[138];
nmpcVariables.x[147] += + nmpcWorkspace.evGx[1112]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1113]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1114]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1115]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1116]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1117]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1118]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1119]*nmpcWorkspace.x[7] + nmpcWorkspace.d[139];
nmpcVariables.x[148] += + nmpcWorkspace.evGx[1120]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1121]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1122]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1123]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1124]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1125]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1126]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1127]*nmpcWorkspace.x[7] + nmpcWorkspace.d[140];
nmpcVariables.x[149] += + nmpcWorkspace.evGx[1128]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1129]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1130]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1131]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1132]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1133]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1134]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1135]*nmpcWorkspace.x[7] + nmpcWorkspace.d[141];
nmpcVariables.x[150] += + nmpcWorkspace.evGx[1136]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1137]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1138]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1139]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1140]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1141]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1142]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1143]*nmpcWorkspace.x[7] + nmpcWorkspace.d[142];
nmpcVariables.x[151] += + nmpcWorkspace.evGx[1144]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1145]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1146]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1147]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1148]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1149]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1150]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1151]*nmpcWorkspace.x[7] + nmpcWorkspace.d[143];
nmpcVariables.x[152] += + nmpcWorkspace.evGx[1152]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1153]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1154]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1155]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1156]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1157]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1158]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1159]*nmpcWorkspace.x[7] + nmpcWorkspace.d[144];
nmpcVariables.x[153] += + nmpcWorkspace.evGx[1160]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1161]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1162]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1163]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1164]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1165]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1166]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1167]*nmpcWorkspace.x[7] + nmpcWorkspace.d[145];
nmpcVariables.x[154] += + nmpcWorkspace.evGx[1168]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1169]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1170]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1171]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1172]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1173]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1174]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1175]*nmpcWorkspace.x[7] + nmpcWorkspace.d[146];
nmpcVariables.x[155] += + nmpcWorkspace.evGx[1176]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1177]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1178]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1179]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1180]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1181]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1182]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1183]*nmpcWorkspace.x[7] + nmpcWorkspace.d[147];
nmpcVariables.x[156] += + nmpcWorkspace.evGx[1184]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1185]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1186]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1187]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1188]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1189]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1190]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1191]*nmpcWorkspace.x[7] + nmpcWorkspace.d[148];
nmpcVariables.x[157] += + nmpcWorkspace.evGx[1192]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1193]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1194]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1195]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1196]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1197]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1198]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1199]*nmpcWorkspace.x[7] + nmpcWorkspace.d[149];
nmpcVariables.x[158] += + nmpcWorkspace.evGx[1200]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1201]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1202]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1203]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1204]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1205]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1206]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1207]*nmpcWorkspace.x[7] + nmpcWorkspace.d[150];
nmpcVariables.x[159] += + nmpcWorkspace.evGx[1208]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1209]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1210]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1211]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1212]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1213]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1214]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1215]*nmpcWorkspace.x[7] + nmpcWorkspace.d[151];
nmpcVariables.x[160] += + nmpcWorkspace.evGx[1216]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1217]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1218]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1219]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1220]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1221]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1222]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1223]*nmpcWorkspace.x[7] + nmpcWorkspace.d[152];
nmpcVariables.x[161] += + nmpcWorkspace.evGx[1224]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1225]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1226]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1227]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1228]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1229]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1230]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1231]*nmpcWorkspace.x[7] + nmpcWorkspace.d[153];
nmpcVariables.x[162] += + nmpcWorkspace.evGx[1232]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1233]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1234]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1235]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1236]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1237]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1238]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1239]*nmpcWorkspace.x[7] + nmpcWorkspace.d[154];
nmpcVariables.x[163] += + nmpcWorkspace.evGx[1240]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1241]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1242]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1243]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1244]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1245]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1246]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1247]*nmpcWorkspace.x[7] + nmpcWorkspace.d[155];
nmpcVariables.x[164] += + nmpcWorkspace.evGx[1248]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1249]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1250]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1251]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1252]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1253]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1254]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1255]*nmpcWorkspace.x[7] + nmpcWorkspace.d[156];
nmpcVariables.x[165] += + nmpcWorkspace.evGx[1256]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1257]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1258]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1259]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1260]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1261]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1262]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1263]*nmpcWorkspace.x[7] + nmpcWorkspace.d[157];
nmpcVariables.x[166] += + nmpcWorkspace.evGx[1264]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1265]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1266]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1267]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1268]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1269]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1270]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1271]*nmpcWorkspace.x[7] + nmpcWorkspace.d[158];
nmpcVariables.x[167] += + nmpcWorkspace.evGx[1272]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1273]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1274]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1275]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1276]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1277]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1278]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1279]*nmpcWorkspace.x[7] + nmpcWorkspace.d[159];
nmpcVariables.x[168] += + nmpcWorkspace.evGx[1280]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1281]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1282]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1283]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1284]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1285]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1286]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1287]*nmpcWorkspace.x[7] + nmpcWorkspace.d[160];
nmpcVariables.x[169] += + nmpcWorkspace.evGx[1288]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1289]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1290]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1291]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1292]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1293]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1294]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1295]*nmpcWorkspace.x[7] + nmpcWorkspace.d[161];
nmpcVariables.x[170] += + nmpcWorkspace.evGx[1296]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1297]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1298]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1299]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1300]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1301]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1302]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1303]*nmpcWorkspace.x[7] + nmpcWorkspace.d[162];
nmpcVariables.x[171] += + nmpcWorkspace.evGx[1304]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1305]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1306]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1307]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1308]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1309]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1310]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1311]*nmpcWorkspace.x[7] + nmpcWorkspace.d[163];
nmpcVariables.x[172] += + nmpcWorkspace.evGx[1312]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1313]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1314]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1315]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1316]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1317]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1318]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1319]*nmpcWorkspace.x[7] + nmpcWorkspace.d[164];
nmpcVariables.x[173] += + nmpcWorkspace.evGx[1320]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1321]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1322]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1323]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1324]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1325]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1326]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1327]*nmpcWorkspace.x[7] + nmpcWorkspace.d[165];
nmpcVariables.x[174] += + nmpcWorkspace.evGx[1328]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1329]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1330]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1331]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1332]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1333]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1334]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1335]*nmpcWorkspace.x[7] + nmpcWorkspace.d[166];
nmpcVariables.x[175] += + nmpcWorkspace.evGx[1336]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1337]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1338]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1339]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1340]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1341]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1342]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1343]*nmpcWorkspace.x[7] + nmpcWorkspace.d[167];
nmpcVariables.x[176] += + nmpcWorkspace.evGx[1344]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1345]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1346]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1347]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1348]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1349]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1350]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1351]*nmpcWorkspace.x[7] + nmpcWorkspace.d[168];
nmpcVariables.x[177] += + nmpcWorkspace.evGx[1352]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1353]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1354]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1355]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1356]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1357]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1358]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1359]*nmpcWorkspace.x[7] + nmpcWorkspace.d[169];
nmpcVariables.x[178] += + nmpcWorkspace.evGx[1360]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1361]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1362]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1363]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1364]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1365]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1366]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1367]*nmpcWorkspace.x[7] + nmpcWorkspace.d[170];
nmpcVariables.x[179] += + nmpcWorkspace.evGx[1368]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1369]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1370]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1371]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1372]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1373]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1374]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1375]*nmpcWorkspace.x[7] + nmpcWorkspace.d[171];
nmpcVariables.x[180] += + nmpcWorkspace.evGx[1376]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1377]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1378]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1379]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1380]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1381]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1382]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1383]*nmpcWorkspace.x[7] + nmpcWorkspace.d[172];
nmpcVariables.x[181] += + nmpcWorkspace.evGx[1384]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1385]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1386]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1387]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1388]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1389]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1390]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1391]*nmpcWorkspace.x[7] + nmpcWorkspace.d[173];
nmpcVariables.x[182] += + nmpcWorkspace.evGx[1392]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1393]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1394]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1395]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1396]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1397]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1398]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1399]*nmpcWorkspace.x[7] + nmpcWorkspace.d[174];
nmpcVariables.x[183] += + nmpcWorkspace.evGx[1400]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1401]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1402]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1403]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1404]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1405]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1406]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1407]*nmpcWorkspace.x[7] + nmpcWorkspace.d[175];
nmpcVariables.x[184] += + nmpcWorkspace.evGx[1408]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1409]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1410]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1411]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1412]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1413]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1414]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1415]*nmpcWorkspace.x[7] + nmpcWorkspace.d[176];
nmpcVariables.x[185] += + nmpcWorkspace.evGx[1416]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1417]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1418]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1419]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1420]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1421]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1422]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1423]*nmpcWorkspace.x[7] + nmpcWorkspace.d[177];
nmpcVariables.x[186] += + nmpcWorkspace.evGx[1424]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1425]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1426]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1427]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1428]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1429]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1430]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1431]*nmpcWorkspace.x[7] + nmpcWorkspace.d[178];
nmpcVariables.x[187] += + nmpcWorkspace.evGx[1432]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1433]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1434]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1435]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1436]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1437]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1438]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1439]*nmpcWorkspace.x[7] + nmpcWorkspace.d[179];
nmpcVariables.x[188] += + nmpcWorkspace.evGx[1440]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1441]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1442]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1443]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1444]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1445]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1446]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1447]*nmpcWorkspace.x[7] + nmpcWorkspace.d[180];
nmpcVariables.x[189] += + nmpcWorkspace.evGx[1448]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1449]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1450]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1451]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1452]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1453]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1454]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1455]*nmpcWorkspace.x[7] + nmpcWorkspace.d[181];
nmpcVariables.x[190] += + nmpcWorkspace.evGx[1456]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1457]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1458]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1459]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1460]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1461]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1462]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1463]*nmpcWorkspace.x[7] + nmpcWorkspace.d[182];
nmpcVariables.x[191] += + nmpcWorkspace.evGx[1464]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1465]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1466]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1467]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1468]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1469]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1470]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1471]*nmpcWorkspace.x[7] + nmpcWorkspace.d[183];
nmpcVariables.x[192] += + nmpcWorkspace.evGx[1472]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1473]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1474]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1475]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1476]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1477]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1478]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1479]*nmpcWorkspace.x[7] + nmpcWorkspace.d[184];
nmpcVariables.x[193] += + nmpcWorkspace.evGx[1480]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1481]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1482]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1483]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1484]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1485]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1486]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1487]*nmpcWorkspace.x[7] + nmpcWorkspace.d[185];
nmpcVariables.x[194] += + nmpcWorkspace.evGx[1488]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1489]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1490]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1491]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1492]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1493]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1494]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1495]*nmpcWorkspace.x[7] + nmpcWorkspace.d[186];
nmpcVariables.x[195] += + nmpcWorkspace.evGx[1496]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1497]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1498]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1499]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1500]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1501]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1502]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1503]*nmpcWorkspace.x[7] + nmpcWorkspace.d[187];
nmpcVariables.x[196] += + nmpcWorkspace.evGx[1504]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1505]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1506]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1507]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1508]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1509]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1510]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1511]*nmpcWorkspace.x[7] + nmpcWorkspace.d[188];
nmpcVariables.x[197] += + nmpcWorkspace.evGx[1512]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1513]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1514]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1515]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1516]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1517]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1518]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1519]*nmpcWorkspace.x[7] + nmpcWorkspace.d[189];
nmpcVariables.x[198] += + nmpcWorkspace.evGx[1520]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1521]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1522]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1523]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1524]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1525]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1526]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1527]*nmpcWorkspace.x[7] + nmpcWorkspace.d[190];
nmpcVariables.x[199] += + nmpcWorkspace.evGx[1528]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1529]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1530]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1531]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1532]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1533]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1534]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1535]*nmpcWorkspace.x[7] + nmpcWorkspace.d[191];
nmpcVariables.x[200] += + nmpcWorkspace.evGx[1536]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1537]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1538]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1539]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1540]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1541]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1542]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1543]*nmpcWorkspace.x[7] + nmpcWorkspace.d[192];
nmpcVariables.x[201] += + nmpcWorkspace.evGx[1544]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1545]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1546]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1547]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1548]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1549]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1550]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1551]*nmpcWorkspace.x[7] + nmpcWorkspace.d[193];
nmpcVariables.x[202] += + nmpcWorkspace.evGx[1552]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1553]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1554]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1555]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1556]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1557]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1558]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1559]*nmpcWorkspace.x[7] + nmpcWorkspace.d[194];
nmpcVariables.x[203] += + nmpcWorkspace.evGx[1560]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1561]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1562]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1563]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1564]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1565]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1566]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1567]*nmpcWorkspace.x[7] + nmpcWorkspace.d[195];
nmpcVariables.x[204] += + nmpcWorkspace.evGx[1568]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1569]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1570]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1571]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1572]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1573]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1574]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1575]*nmpcWorkspace.x[7] + nmpcWorkspace.d[196];
nmpcVariables.x[205] += + nmpcWorkspace.evGx[1576]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1577]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1578]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1579]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1580]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1581]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1582]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1583]*nmpcWorkspace.x[7] + nmpcWorkspace.d[197];
nmpcVariables.x[206] += + nmpcWorkspace.evGx[1584]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1585]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1586]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1587]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1588]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1589]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1590]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1591]*nmpcWorkspace.x[7] + nmpcWorkspace.d[198];
nmpcVariables.x[207] += + nmpcWorkspace.evGx[1592]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1593]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1594]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1595]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1596]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1597]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1598]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1599]*nmpcWorkspace.x[7] + nmpcWorkspace.d[199];
nmpcVariables.x[208] += + nmpcWorkspace.evGx[1600]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1601]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1602]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1603]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1604]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1605]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1606]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1607]*nmpcWorkspace.x[7] + nmpcWorkspace.d[200];
nmpcVariables.x[209] += + nmpcWorkspace.evGx[1608]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1609]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1610]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1611]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1612]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1613]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1614]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1615]*nmpcWorkspace.x[7] + nmpcWorkspace.d[201];
nmpcVariables.x[210] += + nmpcWorkspace.evGx[1616]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1617]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1618]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1619]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1620]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1621]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1622]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1623]*nmpcWorkspace.x[7] + nmpcWorkspace.d[202];
nmpcVariables.x[211] += + nmpcWorkspace.evGx[1624]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1625]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1626]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1627]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1628]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1629]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1630]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1631]*nmpcWorkspace.x[7] + nmpcWorkspace.d[203];
nmpcVariables.x[212] += + nmpcWorkspace.evGx[1632]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1633]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1634]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1635]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1636]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1637]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1638]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1639]*nmpcWorkspace.x[7] + nmpcWorkspace.d[204];
nmpcVariables.x[213] += + nmpcWorkspace.evGx[1640]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1641]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1642]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1643]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1644]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1645]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1646]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1647]*nmpcWorkspace.x[7] + nmpcWorkspace.d[205];
nmpcVariables.x[214] += + nmpcWorkspace.evGx[1648]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1649]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1650]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1651]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1652]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1653]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1654]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1655]*nmpcWorkspace.x[7] + nmpcWorkspace.d[206];
nmpcVariables.x[215] += + nmpcWorkspace.evGx[1656]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1657]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1658]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1659]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1660]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1661]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1662]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1663]*nmpcWorkspace.x[7] + nmpcWorkspace.d[207];
nmpcVariables.x[216] += + nmpcWorkspace.evGx[1664]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1665]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1666]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1667]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1668]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1669]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1670]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1671]*nmpcWorkspace.x[7] + nmpcWorkspace.d[208];
nmpcVariables.x[217] += + nmpcWorkspace.evGx[1672]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1673]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1674]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1675]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1676]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1677]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1678]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1679]*nmpcWorkspace.x[7] + nmpcWorkspace.d[209];
nmpcVariables.x[218] += + nmpcWorkspace.evGx[1680]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1681]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1682]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1683]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1684]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1685]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1686]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1687]*nmpcWorkspace.x[7] + nmpcWorkspace.d[210];
nmpcVariables.x[219] += + nmpcWorkspace.evGx[1688]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1689]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1690]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1691]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1692]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1693]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1694]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1695]*nmpcWorkspace.x[7] + nmpcWorkspace.d[211];
nmpcVariables.x[220] += + nmpcWorkspace.evGx[1696]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1697]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1698]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1699]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1700]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1701]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1702]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1703]*nmpcWorkspace.x[7] + nmpcWorkspace.d[212];
nmpcVariables.x[221] += + nmpcWorkspace.evGx[1704]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1705]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1706]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1707]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1708]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1709]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1710]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1711]*nmpcWorkspace.x[7] + nmpcWorkspace.d[213];
nmpcVariables.x[222] += + nmpcWorkspace.evGx[1712]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1713]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1714]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1715]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1716]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1717]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1718]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1719]*nmpcWorkspace.x[7] + nmpcWorkspace.d[214];
nmpcVariables.x[223] += + nmpcWorkspace.evGx[1720]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1721]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1722]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1723]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1724]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1725]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1726]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1727]*nmpcWorkspace.x[7] + nmpcWorkspace.d[215];
nmpcVariables.x[224] += + nmpcWorkspace.evGx[1728]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1729]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1730]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1731]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1732]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1733]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1734]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1735]*nmpcWorkspace.x[7] + nmpcWorkspace.d[216];
nmpcVariables.x[225] += + nmpcWorkspace.evGx[1736]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1737]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1738]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1739]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1740]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1741]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1742]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1743]*nmpcWorkspace.x[7] + nmpcWorkspace.d[217];
nmpcVariables.x[226] += + nmpcWorkspace.evGx[1744]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1745]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1746]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1747]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1748]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1749]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1750]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1751]*nmpcWorkspace.x[7] + nmpcWorkspace.d[218];
nmpcVariables.x[227] += + nmpcWorkspace.evGx[1752]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1753]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1754]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1755]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1756]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1757]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1758]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1759]*nmpcWorkspace.x[7] + nmpcWorkspace.d[219];
nmpcVariables.x[228] += + nmpcWorkspace.evGx[1760]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1761]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1762]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1763]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1764]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1765]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1766]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1767]*nmpcWorkspace.x[7] + nmpcWorkspace.d[220];
nmpcVariables.x[229] += + nmpcWorkspace.evGx[1768]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1769]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1770]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1771]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1772]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1773]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1774]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1775]*nmpcWorkspace.x[7] + nmpcWorkspace.d[221];
nmpcVariables.x[230] += + nmpcWorkspace.evGx[1776]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1777]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1778]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1779]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1780]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1781]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1782]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1783]*nmpcWorkspace.x[7] + nmpcWorkspace.d[222];
nmpcVariables.x[231] += + nmpcWorkspace.evGx[1784]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1785]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1786]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1787]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1788]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1789]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1790]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1791]*nmpcWorkspace.x[7] + nmpcWorkspace.d[223];
nmpcVariables.x[232] += + nmpcWorkspace.evGx[1792]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1793]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1794]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1795]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1796]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1797]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1798]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1799]*nmpcWorkspace.x[7] + nmpcWorkspace.d[224];
nmpcVariables.x[233] += + nmpcWorkspace.evGx[1800]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1801]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1802]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1803]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1804]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1805]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1806]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1807]*nmpcWorkspace.x[7] + nmpcWorkspace.d[225];
nmpcVariables.x[234] += + nmpcWorkspace.evGx[1808]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1809]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1810]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1811]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1812]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1813]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1814]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1815]*nmpcWorkspace.x[7] + nmpcWorkspace.d[226];
nmpcVariables.x[235] += + nmpcWorkspace.evGx[1816]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1817]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1818]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1819]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1820]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1821]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1822]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1823]*nmpcWorkspace.x[7] + nmpcWorkspace.d[227];
nmpcVariables.x[236] += + nmpcWorkspace.evGx[1824]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1825]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1826]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1827]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1828]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1829]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1830]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1831]*nmpcWorkspace.x[7] + nmpcWorkspace.d[228];
nmpcVariables.x[237] += + nmpcWorkspace.evGx[1832]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1833]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1834]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1835]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1836]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1837]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1838]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1839]*nmpcWorkspace.x[7] + nmpcWorkspace.d[229];
nmpcVariables.x[238] += + nmpcWorkspace.evGx[1840]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1841]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1842]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1843]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1844]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1845]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1846]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1847]*nmpcWorkspace.x[7] + nmpcWorkspace.d[230];
nmpcVariables.x[239] += + nmpcWorkspace.evGx[1848]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1849]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1850]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1851]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1852]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1853]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1854]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1855]*nmpcWorkspace.x[7] + nmpcWorkspace.d[231];
nmpcVariables.x[240] += + nmpcWorkspace.evGx[1856]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1857]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1858]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1859]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1860]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1861]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1862]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1863]*nmpcWorkspace.x[7] + nmpcWorkspace.d[232];
nmpcVariables.x[241] += + nmpcWorkspace.evGx[1864]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1865]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1866]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1867]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1868]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1869]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1870]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1871]*nmpcWorkspace.x[7] + nmpcWorkspace.d[233];
nmpcVariables.x[242] += + nmpcWorkspace.evGx[1872]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1873]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1874]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1875]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1876]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1877]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1878]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1879]*nmpcWorkspace.x[7] + nmpcWorkspace.d[234];
nmpcVariables.x[243] += + nmpcWorkspace.evGx[1880]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1881]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1882]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1883]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1884]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1885]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1886]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1887]*nmpcWorkspace.x[7] + nmpcWorkspace.d[235];
nmpcVariables.x[244] += + nmpcWorkspace.evGx[1888]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1889]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1890]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1891]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1892]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1893]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1894]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1895]*nmpcWorkspace.x[7] + nmpcWorkspace.d[236];
nmpcVariables.x[245] += + nmpcWorkspace.evGx[1896]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1897]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1898]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1899]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1900]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1901]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1902]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1903]*nmpcWorkspace.x[7] + nmpcWorkspace.d[237];
nmpcVariables.x[246] += + nmpcWorkspace.evGx[1904]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1905]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1906]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1907]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1908]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1909]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1910]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1911]*nmpcWorkspace.x[7] + nmpcWorkspace.d[238];
nmpcVariables.x[247] += + nmpcWorkspace.evGx[1912]*nmpcWorkspace.x[0] + nmpcWorkspace.evGx[1913]*nmpcWorkspace.x[1] + nmpcWorkspace.evGx[1914]*nmpcWorkspace.x[2] + nmpcWorkspace.evGx[1915]*nmpcWorkspace.x[3] + nmpcWorkspace.evGx[1916]*nmpcWorkspace.x[4] + nmpcWorkspace.evGx[1917]*nmpcWorkspace.x[5] + nmpcWorkspace.evGx[1918]*nmpcWorkspace.x[6] + nmpcWorkspace.evGx[1919]*nmpcWorkspace.x[7] + nmpcWorkspace.d[239];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
for (lRun2 = 0; lRun2 < lRun1 + 1; ++lRun2)
{
lRun3 = (((lRun1 + 1) * (lRun1)) / (2)) + (lRun2);
nmpc_multEDu( &(nmpcWorkspace.E[ lRun3 * 32 ]), &(nmpcWorkspace.x[ lRun2 * 4 + 8 ]), &(nmpcVariables.x[ lRun1 * 8 + 8 ]) );
}
}
}

int nmpc_preparationStep(  )
{
int ret;

ret = nmpc_modelSimulation();
nmpc_evaluateObjective(  );
nmpc_condensePrep(  );
return ret;
}

int nmpc_feedbackStep(  )
{
int tmp;

nmpc_condenseFdb(  );

tmp = nmpc_solve( );

nmpc_expand(  );
return tmp;
}

int nmpc_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&nmpcWorkspace, 0, sizeof( nmpcWorkspace ));
return ret;
}

void nmpc_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 30; ++index)
{
nmpcWorkspace.state[0] = nmpcVariables.x[index * 8];
nmpcWorkspace.state[1] = nmpcVariables.x[index * 8 + 1];
nmpcWorkspace.state[2] = nmpcVariables.x[index * 8 + 2];
nmpcWorkspace.state[3] = nmpcVariables.x[index * 8 + 3];
nmpcWorkspace.state[4] = nmpcVariables.x[index * 8 + 4];
nmpcWorkspace.state[5] = nmpcVariables.x[index * 8 + 5];
nmpcWorkspace.state[6] = nmpcVariables.x[index * 8 + 6];
nmpcWorkspace.state[7] = nmpcVariables.x[index * 8 + 7];
nmpcWorkspace.state[104] = nmpcVariables.u[index * 4];
nmpcWorkspace.state[105] = nmpcVariables.u[index * 4 + 1];
nmpcWorkspace.state[106] = nmpcVariables.u[index * 4 + 2];
nmpcWorkspace.state[107] = nmpcVariables.u[index * 4 + 3];
nmpcWorkspace.state[108] = nmpcVariables.od[index * 3];
nmpcWorkspace.state[109] = nmpcVariables.od[index * 3 + 1];
nmpcWorkspace.state[110] = nmpcVariables.od[index * 3 + 2];

nmpc_integrate(nmpcWorkspace.state, index == 0);

nmpcVariables.x[index * 8 + 8] = nmpcWorkspace.state[0];
nmpcVariables.x[index * 8 + 9] = nmpcWorkspace.state[1];
nmpcVariables.x[index * 8 + 10] = nmpcWorkspace.state[2];
nmpcVariables.x[index * 8 + 11] = nmpcWorkspace.state[3];
nmpcVariables.x[index * 8 + 12] = nmpcWorkspace.state[4];
nmpcVariables.x[index * 8 + 13] = nmpcWorkspace.state[5];
nmpcVariables.x[index * 8 + 14] = nmpcWorkspace.state[6];
nmpcVariables.x[index * 8 + 15] = nmpcWorkspace.state[7];
}
}

void nmpc_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 30; ++index)
{
nmpcVariables.x[index * 8] = nmpcVariables.x[index * 8 + 8];
nmpcVariables.x[index * 8 + 1] = nmpcVariables.x[index * 8 + 9];
nmpcVariables.x[index * 8 + 2] = nmpcVariables.x[index * 8 + 10];
nmpcVariables.x[index * 8 + 3] = nmpcVariables.x[index * 8 + 11];
nmpcVariables.x[index * 8 + 4] = nmpcVariables.x[index * 8 + 12];
nmpcVariables.x[index * 8 + 5] = nmpcVariables.x[index * 8 + 13];
nmpcVariables.x[index * 8 + 6] = nmpcVariables.x[index * 8 + 14];
nmpcVariables.x[index * 8 + 7] = nmpcVariables.x[index * 8 + 15];
}

if (strategy == 1 && xEnd != 0)
{
nmpcVariables.x[240] = xEnd[0];
nmpcVariables.x[241] = xEnd[1];
nmpcVariables.x[242] = xEnd[2];
nmpcVariables.x[243] = xEnd[3];
nmpcVariables.x[244] = xEnd[4];
nmpcVariables.x[245] = xEnd[5];
nmpcVariables.x[246] = xEnd[6];
nmpcVariables.x[247] = xEnd[7];
}
else if (strategy == 2) 
{
nmpcWorkspace.state[0] = nmpcVariables.x[240];
nmpcWorkspace.state[1] = nmpcVariables.x[241];
nmpcWorkspace.state[2] = nmpcVariables.x[242];
nmpcWorkspace.state[3] = nmpcVariables.x[243];
nmpcWorkspace.state[4] = nmpcVariables.x[244];
nmpcWorkspace.state[5] = nmpcVariables.x[245];
nmpcWorkspace.state[6] = nmpcVariables.x[246];
nmpcWorkspace.state[7] = nmpcVariables.x[247];
if (uEnd != 0)
{
nmpcWorkspace.state[104] = uEnd[0];
nmpcWorkspace.state[105] = uEnd[1];
nmpcWorkspace.state[106] = uEnd[2];
nmpcWorkspace.state[107] = uEnd[3];
}
else
{
nmpcWorkspace.state[104] = nmpcVariables.u[116];
nmpcWorkspace.state[105] = nmpcVariables.u[117];
nmpcWorkspace.state[106] = nmpcVariables.u[118];
nmpcWorkspace.state[107] = nmpcVariables.u[119];
}
nmpcWorkspace.state[108] = nmpcVariables.od[90];
nmpcWorkspace.state[109] = nmpcVariables.od[91];
nmpcWorkspace.state[110] = nmpcVariables.od[92];

nmpc_integrate(nmpcWorkspace.state, 1);

nmpcVariables.x[240] = nmpcWorkspace.state[0];
nmpcVariables.x[241] = nmpcWorkspace.state[1];
nmpcVariables.x[242] = nmpcWorkspace.state[2];
nmpcVariables.x[243] = nmpcWorkspace.state[3];
nmpcVariables.x[244] = nmpcWorkspace.state[4];
nmpcVariables.x[245] = nmpcWorkspace.state[5];
nmpcVariables.x[246] = nmpcWorkspace.state[6];
nmpcVariables.x[247] = nmpcWorkspace.state[7];
}
}

void nmpc_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 29; ++index)
{
nmpcVariables.u[index * 4] = nmpcVariables.u[index * 4 + 4];
nmpcVariables.u[index * 4 + 1] = nmpcVariables.u[index * 4 + 5];
nmpcVariables.u[index * 4 + 2] = nmpcVariables.u[index * 4 + 6];
nmpcVariables.u[index * 4 + 3] = nmpcVariables.u[index * 4 + 7];
}

if (uEnd != 0)
{
nmpcVariables.u[116] = uEnd[0];
nmpcVariables.u[117] = uEnd[1];
nmpcVariables.u[118] = uEnd[2];
nmpcVariables.u[119] = uEnd[3];
}
}

real_t nmpc_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + nmpcWorkspace.g[0]*nmpcWorkspace.x[0] + nmpcWorkspace.g[1]*nmpcWorkspace.x[1] + nmpcWorkspace.g[2]*nmpcWorkspace.x[2] + nmpcWorkspace.g[3]*nmpcWorkspace.x[3] + nmpcWorkspace.g[4]*nmpcWorkspace.x[4] + nmpcWorkspace.g[5]*nmpcWorkspace.x[5] + nmpcWorkspace.g[6]*nmpcWorkspace.x[6] + nmpcWorkspace.g[7]*nmpcWorkspace.x[7] + nmpcWorkspace.g[8]*nmpcWorkspace.x[8] + nmpcWorkspace.g[9]*nmpcWorkspace.x[9] + nmpcWorkspace.g[10]*nmpcWorkspace.x[10] + nmpcWorkspace.g[11]*nmpcWorkspace.x[11] + nmpcWorkspace.g[12]*nmpcWorkspace.x[12] + nmpcWorkspace.g[13]*nmpcWorkspace.x[13] + nmpcWorkspace.g[14]*nmpcWorkspace.x[14] + nmpcWorkspace.g[15]*nmpcWorkspace.x[15] + nmpcWorkspace.g[16]*nmpcWorkspace.x[16] + nmpcWorkspace.g[17]*nmpcWorkspace.x[17] + nmpcWorkspace.g[18]*nmpcWorkspace.x[18] + nmpcWorkspace.g[19]*nmpcWorkspace.x[19] + nmpcWorkspace.g[20]*nmpcWorkspace.x[20] + nmpcWorkspace.g[21]*nmpcWorkspace.x[21] + nmpcWorkspace.g[22]*nmpcWorkspace.x[22] + nmpcWorkspace.g[23]*nmpcWorkspace.x[23] + nmpcWorkspace.g[24]*nmpcWorkspace.x[24] + nmpcWorkspace.g[25]*nmpcWorkspace.x[25] + nmpcWorkspace.g[26]*nmpcWorkspace.x[26] + nmpcWorkspace.g[27]*nmpcWorkspace.x[27] + nmpcWorkspace.g[28]*nmpcWorkspace.x[28] + nmpcWorkspace.g[29]*nmpcWorkspace.x[29] + nmpcWorkspace.g[30]*nmpcWorkspace.x[30] + nmpcWorkspace.g[31]*nmpcWorkspace.x[31] + nmpcWorkspace.g[32]*nmpcWorkspace.x[32] + nmpcWorkspace.g[33]*nmpcWorkspace.x[33] + nmpcWorkspace.g[34]*nmpcWorkspace.x[34] + nmpcWorkspace.g[35]*nmpcWorkspace.x[35] + nmpcWorkspace.g[36]*nmpcWorkspace.x[36] + nmpcWorkspace.g[37]*nmpcWorkspace.x[37] + nmpcWorkspace.g[38]*nmpcWorkspace.x[38] + nmpcWorkspace.g[39]*nmpcWorkspace.x[39] + nmpcWorkspace.g[40]*nmpcWorkspace.x[40] + nmpcWorkspace.g[41]*nmpcWorkspace.x[41] + nmpcWorkspace.g[42]*nmpcWorkspace.x[42] + nmpcWorkspace.g[43]*nmpcWorkspace.x[43] + nmpcWorkspace.g[44]*nmpcWorkspace.x[44] + nmpcWorkspace.g[45]*nmpcWorkspace.x[45] + nmpcWorkspace.g[46]*nmpcWorkspace.x[46] + nmpcWorkspace.g[47]*nmpcWorkspace.x[47] + nmpcWorkspace.g[48]*nmpcWorkspace.x[48] + nmpcWorkspace.g[49]*nmpcWorkspace.x[49] + nmpcWorkspace.g[50]*nmpcWorkspace.x[50] + nmpcWorkspace.g[51]*nmpcWorkspace.x[51] + nmpcWorkspace.g[52]*nmpcWorkspace.x[52] + nmpcWorkspace.g[53]*nmpcWorkspace.x[53] + nmpcWorkspace.g[54]*nmpcWorkspace.x[54] + nmpcWorkspace.g[55]*nmpcWorkspace.x[55] + nmpcWorkspace.g[56]*nmpcWorkspace.x[56] + nmpcWorkspace.g[57]*nmpcWorkspace.x[57] + nmpcWorkspace.g[58]*nmpcWorkspace.x[58] + nmpcWorkspace.g[59]*nmpcWorkspace.x[59] + nmpcWorkspace.g[60]*nmpcWorkspace.x[60] + nmpcWorkspace.g[61]*nmpcWorkspace.x[61] + nmpcWorkspace.g[62]*nmpcWorkspace.x[62] + nmpcWorkspace.g[63]*nmpcWorkspace.x[63] + nmpcWorkspace.g[64]*nmpcWorkspace.x[64] + nmpcWorkspace.g[65]*nmpcWorkspace.x[65] + nmpcWorkspace.g[66]*nmpcWorkspace.x[66] + nmpcWorkspace.g[67]*nmpcWorkspace.x[67] + nmpcWorkspace.g[68]*nmpcWorkspace.x[68] + nmpcWorkspace.g[69]*nmpcWorkspace.x[69] + nmpcWorkspace.g[70]*nmpcWorkspace.x[70] + nmpcWorkspace.g[71]*nmpcWorkspace.x[71] + nmpcWorkspace.g[72]*nmpcWorkspace.x[72] + nmpcWorkspace.g[73]*nmpcWorkspace.x[73] + nmpcWorkspace.g[74]*nmpcWorkspace.x[74] + nmpcWorkspace.g[75]*nmpcWorkspace.x[75] + nmpcWorkspace.g[76]*nmpcWorkspace.x[76] + nmpcWorkspace.g[77]*nmpcWorkspace.x[77] + nmpcWorkspace.g[78]*nmpcWorkspace.x[78] + nmpcWorkspace.g[79]*nmpcWorkspace.x[79] + nmpcWorkspace.g[80]*nmpcWorkspace.x[80] + nmpcWorkspace.g[81]*nmpcWorkspace.x[81] + nmpcWorkspace.g[82]*nmpcWorkspace.x[82] + nmpcWorkspace.g[83]*nmpcWorkspace.x[83] + nmpcWorkspace.g[84]*nmpcWorkspace.x[84] + nmpcWorkspace.g[85]*nmpcWorkspace.x[85] + nmpcWorkspace.g[86]*nmpcWorkspace.x[86] + nmpcWorkspace.g[87]*nmpcWorkspace.x[87] + nmpcWorkspace.g[88]*nmpcWorkspace.x[88] + nmpcWorkspace.g[89]*nmpcWorkspace.x[89] + nmpcWorkspace.g[90]*nmpcWorkspace.x[90] + nmpcWorkspace.g[91]*nmpcWorkspace.x[91] + nmpcWorkspace.g[92]*nmpcWorkspace.x[92] + nmpcWorkspace.g[93]*nmpcWorkspace.x[93] + nmpcWorkspace.g[94]*nmpcWorkspace.x[94] + nmpcWorkspace.g[95]*nmpcWorkspace.x[95] + nmpcWorkspace.g[96]*nmpcWorkspace.x[96] + nmpcWorkspace.g[97]*nmpcWorkspace.x[97] + nmpcWorkspace.g[98]*nmpcWorkspace.x[98] + nmpcWorkspace.g[99]*nmpcWorkspace.x[99] + nmpcWorkspace.g[100]*nmpcWorkspace.x[100] + nmpcWorkspace.g[101]*nmpcWorkspace.x[101] + nmpcWorkspace.g[102]*nmpcWorkspace.x[102] + nmpcWorkspace.g[103]*nmpcWorkspace.x[103] + nmpcWorkspace.g[104]*nmpcWorkspace.x[104] + nmpcWorkspace.g[105]*nmpcWorkspace.x[105] + nmpcWorkspace.g[106]*nmpcWorkspace.x[106] + nmpcWorkspace.g[107]*nmpcWorkspace.x[107] + nmpcWorkspace.g[108]*nmpcWorkspace.x[108] + nmpcWorkspace.g[109]*nmpcWorkspace.x[109] + nmpcWorkspace.g[110]*nmpcWorkspace.x[110] + nmpcWorkspace.g[111]*nmpcWorkspace.x[111] + nmpcWorkspace.g[112]*nmpcWorkspace.x[112] + nmpcWorkspace.g[113]*nmpcWorkspace.x[113] + nmpcWorkspace.g[114]*nmpcWorkspace.x[114] + nmpcWorkspace.g[115]*nmpcWorkspace.x[115] + nmpcWorkspace.g[116]*nmpcWorkspace.x[116] + nmpcWorkspace.g[117]*nmpcWorkspace.x[117] + nmpcWorkspace.g[118]*nmpcWorkspace.x[118] + nmpcWorkspace.g[119]*nmpcWorkspace.x[119] + nmpcWorkspace.g[120]*nmpcWorkspace.x[120] + nmpcWorkspace.g[121]*nmpcWorkspace.x[121] + nmpcWorkspace.g[122]*nmpcWorkspace.x[122] + nmpcWorkspace.g[123]*nmpcWorkspace.x[123] + nmpcWorkspace.g[124]*nmpcWorkspace.x[124] + nmpcWorkspace.g[125]*nmpcWorkspace.x[125] + nmpcWorkspace.g[126]*nmpcWorkspace.x[126] + nmpcWorkspace.g[127]*nmpcWorkspace.x[127];
kkt = fabs( kkt );
for (index = 0; index < 128; ++index)
{
prd = nmpcWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(nmpcWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(nmpcWorkspace.ub[index] * prd);
}
return kkt;
}

real_t nmpc_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 12 */
real_t tmpDy[ 12 ];

/** Row vector of size: 8 */
real_t tmpDyN[ 8 ];

for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[lRun1 * 8];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[lRun1 * 8 + 1];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[lRun1 * 8 + 2];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[lRun1 * 8 + 3];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[lRun1 * 8 + 4];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[lRun1 * 8 + 5];
nmpcWorkspace.objValueIn[6] = nmpcVariables.x[lRun1 * 8 + 6];
nmpcWorkspace.objValueIn[7] = nmpcVariables.x[lRun1 * 8 + 7];
nmpcWorkspace.objValueIn[8] = nmpcVariables.u[lRun1 * 4];
nmpcWorkspace.objValueIn[9] = nmpcVariables.u[lRun1 * 4 + 1];
nmpcWorkspace.objValueIn[10] = nmpcVariables.u[lRun1 * 4 + 2];
nmpcWorkspace.objValueIn[11] = nmpcVariables.u[lRun1 * 4 + 3];
nmpcWorkspace.objValueIn[12] = nmpcVariables.od[lRun1 * 3];
nmpcWorkspace.objValueIn[13] = nmpcVariables.od[lRun1 * 3 + 1];
nmpcWorkspace.objValueIn[14] = nmpcVariables.od[lRun1 * 3 + 2];

nmpc_evaluateLSQ( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );
nmpcWorkspace.Dy[lRun1 * 12] = nmpcWorkspace.objValueOut[0] - nmpcVariables.y[lRun1 * 12];
nmpcWorkspace.Dy[lRun1 * 12 + 1] = nmpcWorkspace.objValueOut[1] - nmpcVariables.y[lRun1 * 12 + 1];
nmpcWorkspace.Dy[lRun1 * 12 + 2] = nmpcWorkspace.objValueOut[2] - nmpcVariables.y[lRun1 * 12 + 2];
nmpcWorkspace.Dy[lRun1 * 12 + 3] = nmpcWorkspace.objValueOut[3] - nmpcVariables.y[lRun1 * 12 + 3];
nmpcWorkspace.Dy[lRun1 * 12 + 4] = nmpcWorkspace.objValueOut[4] - nmpcVariables.y[lRun1 * 12 + 4];
nmpcWorkspace.Dy[lRun1 * 12 + 5] = nmpcWorkspace.objValueOut[5] - nmpcVariables.y[lRun1 * 12 + 5];
nmpcWorkspace.Dy[lRun1 * 12 + 6] = nmpcWorkspace.objValueOut[6] - nmpcVariables.y[lRun1 * 12 + 6];
nmpcWorkspace.Dy[lRun1 * 12 + 7] = nmpcWorkspace.objValueOut[7] - nmpcVariables.y[lRun1 * 12 + 7];
nmpcWorkspace.Dy[lRun1 * 12 + 8] = nmpcWorkspace.objValueOut[8] - nmpcVariables.y[lRun1 * 12 + 8];
nmpcWorkspace.Dy[lRun1 * 12 + 9] = nmpcWorkspace.objValueOut[9] - nmpcVariables.y[lRun1 * 12 + 9];
nmpcWorkspace.Dy[lRun1 * 12 + 10] = nmpcWorkspace.objValueOut[10] - nmpcVariables.y[lRun1 * 12 + 10];
nmpcWorkspace.Dy[lRun1 * 12 + 11] = nmpcWorkspace.objValueOut[11] - nmpcVariables.y[lRun1 * 12 + 11];
}
nmpcWorkspace.objValueIn[0] = nmpcVariables.x[240];
nmpcWorkspace.objValueIn[1] = nmpcVariables.x[241];
nmpcWorkspace.objValueIn[2] = nmpcVariables.x[242];
nmpcWorkspace.objValueIn[3] = nmpcVariables.x[243];
nmpcWorkspace.objValueIn[4] = nmpcVariables.x[244];
nmpcWorkspace.objValueIn[5] = nmpcVariables.x[245];
nmpcWorkspace.objValueIn[6] = nmpcVariables.x[246];
nmpcWorkspace.objValueIn[7] = nmpcVariables.x[247];
nmpcWorkspace.objValueIn[8] = nmpcVariables.od[90];
nmpcWorkspace.objValueIn[9] = nmpcVariables.od[91];
nmpcWorkspace.objValueIn[10] = nmpcVariables.od[92];
nmpc_evaluateLSQEndTerm( nmpcWorkspace.objValueIn, nmpcWorkspace.objValueOut );
nmpcWorkspace.DyN[0] = nmpcWorkspace.objValueOut[0] - nmpcVariables.yN[0];
nmpcWorkspace.DyN[1] = nmpcWorkspace.objValueOut[1] - nmpcVariables.yN[1];
nmpcWorkspace.DyN[2] = nmpcWorkspace.objValueOut[2] - nmpcVariables.yN[2];
nmpcWorkspace.DyN[3] = nmpcWorkspace.objValueOut[3] - nmpcVariables.yN[3];
nmpcWorkspace.DyN[4] = nmpcWorkspace.objValueOut[4] - nmpcVariables.yN[4];
nmpcWorkspace.DyN[5] = nmpcWorkspace.objValueOut[5] - nmpcVariables.yN[5];
nmpcWorkspace.DyN[6] = nmpcWorkspace.objValueOut[6] - nmpcVariables.yN[6];
nmpcWorkspace.DyN[7] = nmpcWorkspace.objValueOut[7] - nmpcVariables.yN[7];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 30; ++lRun1)
{
tmpDy[0] = + nmpcWorkspace.Dy[lRun1 * 12]*nmpcVariables.W[0];
tmpDy[1] = + nmpcWorkspace.Dy[lRun1 * 12 + 1]*nmpcVariables.W[13];
tmpDy[2] = + nmpcWorkspace.Dy[lRun1 * 12 + 2]*nmpcVariables.W[26];
tmpDy[3] = + nmpcWorkspace.Dy[lRun1 * 12 + 3]*nmpcVariables.W[39];
tmpDy[4] = + nmpcWorkspace.Dy[lRun1 * 12 + 4]*nmpcVariables.W[52];
tmpDy[5] = + nmpcWorkspace.Dy[lRun1 * 12 + 5]*nmpcVariables.W[65];
tmpDy[6] = + nmpcWorkspace.Dy[lRun1 * 12 + 6]*nmpcVariables.W[78];
tmpDy[7] = + nmpcWorkspace.Dy[lRun1 * 12 + 7]*nmpcVariables.W[91];
tmpDy[8] = + nmpcWorkspace.Dy[lRun1 * 12 + 8]*nmpcVariables.W[104];
tmpDy[9] = + nmpcWorkspace.Dy[lRun1 * 12 + 9]*nmpcVariables.W[117];
tmpDy[10] = + nmpcWorkspace.Dy[lRun1 * 12 + 10]*nmpcVariables.W[130];
tmpDy[11] = + nmpcWorkspace.Dy[lRun1 * 12 + 11]*nmpcVariables.W[143];
objVal += + nmpcWorkspace.Dy[lRun1 * 12]*tmpDy[0] + nmpcWorkspace.Dy[lRun1 * 12 + 1]*tmpDy[1] + nmpcWorkspace.Dy[lRun1 * 12 + 2]*tmpDy[2] + nmpcWorkspace.Dy[lRun1 * 12 + 3]*tmpDy[3] + nmpcWorkspace.Dy[lRun1 * 12 + 4]*tmpDy[4] + nmpcWorkspace.Dy[lRun1 * 12 + 5]*tmpDy[5] + nmpcWorkspace.Dy[lRun1 * 12 + 6]*tmpDy[6] + nmpcWorkspace.Dy[lRun1 * 12 + 7]*tmpDy[7] + nmpcWorkspace.Dy[lRun1 * 12 + 8]*tmpDy[8] + nmpcWorkspace.Dy[lRun1 * 12 + 9]*tmpDy[9] + nmpcWorkspace.Dy[lRun1 * 12 + 10]*tmpDy[10] + nmpcWorkspace.Dy[lRun1 * 12 + 11]*tmpDy[11];
}

tmpDyN[0] = + nmpcWorkspace.DyN[0]*nmpcVariables.WN[0];
tmpDyN[1] = + nmpcWorkspace.DyN[1]*nmpcVariables.WN[9];
tmpDyN[2] = + nmpcWorkspace.DyN[2]*nmpcVariables.WN[18];
tmpDyN[3] = + nmpcWorkspace.DyN[3]*nmpcVariables.WN[27];
tmpDyN[4] = + nmpcWorkspace.DyN[4]*nmpcVariables.WN[36];
tmpDyN[5] = + nmpcWorkspace.DyN[5]*nmpcVariables.WN[45];
tmpDyN[6] = + nmpcWorkspace.DyN[6]*nmpcVariables.WN[54];
tmpDyN[7] = + nmpcWorkspace.DyN[7]*nmpcVariables.WN[63];
objVal += + nmpcWorkspace.DyN[0]*tmpDyN[0] + nmpcWorkspace.DyN[1]*tmpDyN[1] + nmpcWorkspace.DyN[2]*tmpDyN[2] + nmpcWorkspace.DyN[3]*tmpDyN[3] + nmpcWorkspace.DyN[4]*tmpDyN[4] + nmpcWorkspace.DyN[5]*tmpDyN[5] + nmpcWorkspace.DyN[6]*tmpDyN[6] + nmpcWorkspace.DyN[7]*tmpDyN[7];

objVal *= 0.5;
return objVal;
}

