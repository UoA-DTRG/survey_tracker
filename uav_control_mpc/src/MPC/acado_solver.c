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


int acado_modelSimulation(  )
{
int ret;

int lRun1;
ret = 0;
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 9];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 9 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 9 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 9 + 3];
acadoWorkspace.state[4] = acadoVariables.x[lRun1 * 9 + 4];
acadoWorkspace.state[5] = acadoVariables.x[lRun1 * 9 + 5];
acadoWorkspace.state[6] = acadoVariables.x[lRun1 * 9 + 6];
acadoWorkspace.state[7] = acadoVariables.x[lRun1 * 9 + 7];
acadoWorkspace.state[8] = acadoVariables.x[lRun1 * 9 + 8];

acadoWorkspace.state[117] = acadoVariables.u[lRun1 * 3];
acadoWorkspace.state[118] = acadoVariables.u[lRun1 * 3 + 1];
acadoWorkspace.state[119] = acadoVariables.u[lRun1 * 3 + 2];
acadoWorkspace.state[120] = acadoVariables.od[lRun1 * 3];
acadoWorkspace.state[121] = acadoVariables.od[lRun1 * 3 + 1];
acadoWorkspace.state[122] = acadoVariables.od[lRun1 * 3 + 2];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 9] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 9 + 9];
acadoWorkspace.d[lRun1 * 9 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 9 + 10];
acadoWorkspace.d[lRun1 * 9 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 9 + 11];
acadoWorkspace.d[lRun1 * 9 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 9 + 12];
acadoWorkspace.d[lRun1 * 9 + 4] = acadoWorkspace.state[4] - acadoVariables.x[lRun1 * 9 + 13];
acadoWorkspace.d[lRun1 * 9 + 5] = acadoWorkspace.state[5] - acadoVariables.x[lRun1 * 9 + 14];
acadoWorkspace.d[lRun1 * 9 + 6] = acadoWorkspace.state[6] - acadoVariables.x[lRun1 * 9 + 15];
acadoWorkspace.d[lRun1 * 9 + 7] = acadoWorkspace.state[7] - acadoVariables.x[lRun1 * 9 + 16];
acadoWorkspace.d[lRun1 * 9 + 8] = acadoWorkspace.state[8] - acadoVariables.x[lRun1 * 9 + 17];

acadoWorkspace.evGx[lRun1 * 81] = acadoWorkspace.state[9];
acadoWorkspace.evGx[lRun1 * 81 + 1] = acadoWorkspace.state[10];
acadoWorkspace.evGx[lRun1 * 81 + 2] = acadoWorkspace.state[11];
acadoWorkspace.evGx[lRun1 * 81 + 3] = acadoWorkspace.state[12];
acadoWorkspace.evGx[lRun1 * 81 + 4] = acadoWorkspace.state[13];
acadoWorkspace.evGx[lRun1 * 81 + 5] = acadoWorkspace.state[14];
acadoWorkspace.evGx[lRun1 * 81 + 6] = acadoWorkspace.state[15];
acadoWorkspace.evGx[lRun1 * 81 + 7] = acadoWorkspace.state[16];
acadoWorkspace.evGx[lRun1 * 81 + 8] = acadoWorkspace.state[17];
acadoWorkspace.evGx[lRun1 * 81 + 9] = acadoWorkspace.state[18];
acadoWorkspace.evGx[lRun1 * 81 + 10] = acadoWorkspace.state[19];
acadoWorkspace.evGx[lRun1 * 81 + 11] = acadoWorkspace.state[20];
acadoWorkspace.evGx[lRun1 * 81 + 12] = acadoWorkspace.state[21];
acadoWorkspace.evGx[lRun1 * 81 + 13] = acadoWorkspace.state[22];
acadoWorkspace.evGx[lRun1 * 81 + 14] = acadoWorkspace.state[23];
acadoWorkspace.evGx[lRun1 * 81 + 15] = acadoWorkspace.state[24];
acadoWorkspace.evGx[lRun1 * 81 + 16] = acadoWorkspace.state[25];
acadoWorkspace.evGx[lRun1 * 81 + 17] = acadoWorkspace.state[26];
acadoWorkspace.evGx[lRun1 * 81 + 18] = acadoWorkspace.state[27];
acadoWorkspace.evGx[lRun1 * 81 + 19] = acadoWorkspace.state[28];
acadoWorkspace.evGx[lRun1 * 81 + 20] = acadoWorkspace.state[29];
acadoWorkspace.evGx[lRun1 * 81 + 21] = acadoWorkspace.state[30];
acadoWorkspace.evGx[lRun1 * 81 + 22] = acadoWorkspace.state[31];
acadoWorkspace.evGx[lRun1 * 81 + 23] = acadoWorkspace.state[32];
acadoWorkspace.evGx[lRun1 * 81 + 24] = acadoWorkspace.state[33];
acadoWorkspace.evGx[lRun1 * 81 + 25] = acadoWorkspace.state[34];
acadoWorkspace.evGx[lRun1 * 81 + 26] = acadoWorkspace.state[35];
acadoWorkspace.evGx[lRun1 * 81 + 27] = acadoWorkspace.state[36];
acadoWorkspace.evGx[lRun1 * 81 + 28] = acadoWorkspace.state[37];
acadoWorkspace.evGx[lRun1 * 81 + 29] = acadoWorkspace.state[38];
acadoWorkspace.evGx[lRun1 * 81 + 30] = acadoWorkspace.state[39];
acadoWorkspace.evGx[lRun1 * 81 + 31] = acadoWorkspace.state[40];
acadoWorkspace.evGx[lRun1 * 81 + 32] = acadoWorkspace.state[41];
acadoWorkspace.evGx[lRun1 * 81 + 33] = acadoWorkspace.state[42];
acadoWorkspace.evGx[lRun1 * 81 + 34] = acadoWorkspace.state[43];
acadoWorkspace.evGx[lRun1 * 81 + 35] = acadoWorkspace.state[44];
acadoWorkspace.evGx[lRun1 * 81 + 36] = acadoWorkspace.state[45];
acadoWorkspace.evGx[lRun1 * 81 + 37] = acadoWorkspace.state[46];
acadoWorkspace.evGx[lRun1 * 81 + 38] = acadoWorkspace.state[47];
acadoWorkspace.evGx[lRun1 * 81 + 39] = acadoWorkspace.state[48];
acadoWorkspace.evGx[lRun1 * 81 + 40] = acadoWorkspace.state[49];
acadoWorkspace.evGx[lRun1 * 81 + 41] = acadoWorkspace.state[50];
acadoWorkspace.evGx[lRun1 * 81 + 42] = acadoWorkspace.state[51];
acadoWorkspace.evGx[lRun1 * 81 + 43] = acadoWorkspace.state[52];
acadoWorkspace.evGx[lRun1 * 81 + 44] = acadoWorkspace.state[53];
acadoWorkspace.evGx[lRun1 * 81 + 45] = acadoWorkspace.state[54];
acadoWorkspace.evGx[lRun1 * 81 + 46] = acadoWorkspace.state[55];
acadoWorkspace.evGx[lRun1 * 81 + 47] = acadoWorkspace.state[56];
acadoWorkspace.evGx[lRun1 * 81 + 48] = acadoWorkspace.state[57];
acadoWorkspace.evGx[lRun1 * 81 + 49] = acadoWorkspace.state[58];
acadoWorkspace.evGx[lRun1 * 81 + 50] = acadoWorkspace.state[59];
acadoWorkspace.evGx[lRun1 * 81 + 51] = acadoWorkspace.state[60];
acadoWorkspace.evGx[lRun1 * 81 + 52] = acadoWorkspace.state[61];
acadoWorkspace.evGx[lRun1 * 81 + 53] = acadoWorkspace.state[62];
acadoWorkspace.evGx[lRun1 * 81 + 54] = acadoWorkspace.state[63];
acadoWorkspace.evGx[lRun1 * 81 + 55] = acadoWorkspace.state[64];
acadoWorkspace.evGx[lRun1 * 81 + 56] = acadoWorkspace.state[65];
acadoWorkspace.evGx[lRun1 * 81 + 57] = acadoWorkspace.state[66];
acadoWorkspace.evGx[lRun1 * 81 + 58] = acadoWorkspace.state[67];
acadoWorkspace.evGx[lRun1 * 81 + 59] = acadoWorkspace.state[68];
acadoWorkspace.evGx[lRun1 * 81 + 60] = acadoWorkspace.state[69];
acadoWorkspace.evGx[lRun1 * 81 + 61] = acadoWorkspace.state[70];
acadoWorkspace.evGx[lRun1 * 81 + 62] = acadoWorkspace.state[71];
acadoWorkspace.evGx[lRun1 * 81 + 63] = acadoWorkspace.state[72];
acadoWorkspace.evGx[lRun1 * 81 + 64] = acadoWorkspace.state[73];
acadoWorkspace.evGx[lRun1 * 81 + 65] = acadoWorkspace.state[74];
acadoWorkspace.evGx[lRun1 * 81 + 66] = acadoWorkspace.state[75];
acadoWorkspace.evGx[lRun1 * 81 + 67] = acadoWorkspace.state[76];
acadoWorkspace.evGx[lRun1 * 81 + 68] = acadoWorkspace.state[77];
acadoWorkspace.evGx[lRun1 * 81 + 69] = acadoWorkspace.state[78];
acadoWorkspace.evGx[lRun1 * 81 + 70] = acadoWorkspace.state[79];
acadoWorkspace.evGx[lRun1 * 81 + 71] = acadoWorkspace.state[80];
acadoWorkspace.evGx[lRun1 * 81 + 72] = acadoWorkspace.state[81];
acadoWorkspace.evGx[lRun1 * 81 + 73] = acadoWorkspace.state[82];
acadoWorkspace.evGx[lRun1 * 81 + 74] = acadoWorkspace.state[83];
acadoWorkspace.evGx[lRun1 * 81 + 75] = acadoWorkspace.state[84];
acadoWorkspace.evGx[lRun1 * 81 + 76] = acadoWorkspace.state[85];
acadoWorkspace.evGx[lRun1 * 81 + 77] = acadoWorkspace.state[86];
acadoWorkspace.evGx[lRun1 * 81 + 78] = acadoWorkspace.state[87];
acadoWorkspace.evGx[lRun1 * 81 + 79] = acadoWorkspace.state[88];
acadoWorkspace.evGx[lRun1 * 81 + 80] = acadoWorkspace.state[89];

acadoWorkspace.evGu[lRun1 * 27] = acadoWorkspace.state[90];
acadoWorkspace.evGu[lRun1 * 27 + 1] = acadoWorkspace.state[91];
acadoWorkspace.evGu[lRun1 * 27 + 2] = acadoWorkspace.state[92];
acadoWorkspace.evGu[lRun1 * 27 + 3] = acadoWorkspace.state[93];
acadoWorkspace.evGu[lRun1 * 27 + 4] = acadoWorkspace.state[94];
acadoWorkspace.evGu[lRun1 * 27 + 5] = acadoWorkspace.state[95];
acadoWorkspace.evGu[lRun1 * 27 + 6] = acadoWorkspace.state[96];
acadoWorkspace.evGu[lRun1 * 27 + 7] = acadoWorkspace.state[97];
acadoWorkspace.evGu[lRun1 * 27 + 8] = acadoWorkspace.state[98];
acadoWorkspace.evGu[lRun1 * 27 + 9] = acadoWorkspace.state[99];
acadoWorkspace.evGu[lRun1 * 27 + 10] = acadoWorkspace.state[100];
acadoWorkspace.evGu[lRun1 * 27 + 11] = acadoWorkspace.state[101];
acadoWorkspace.evGu[lRun1 * 27 + 12] = acadoWorkspace.state[102];
acadoWorkspace.evGu[lRun1 * 27 + 13] = acadoWorkspace.state[103];
acadoWorkspace.evGu[lRun1 * 27 + 14] = acadoWorkspace.state[104];
acadoWorkspace.evGu[lRun1 * 27 + 15] = acadoWorkspace.state[105];
acadoWorkspace.evGu[lRun1 * 27 + 16] = acadoWorkspace.state[106];
acadoWorkspace.evGu[lRun1 * 27 + 17] = acadoWorkspace.state[107];
acadoWorkspace.evGu[lRun1 * 27 + 18] = acadoWorkspace.state[108];
acadoWorkspace.evGu[lRun1 * 27 + 19] = acadoWorkspace.state[109];
acadoWorkspace.evGu[lRun1 * 27 + 20] = acadoWorkspace.state[110];
acadoWorkspace.evGu[lRun1 * 27 + 21] = acadoWorkspace.state[111];
acadoWorkspace.evGu[lRun1 * 27 + 22] = acadoWorkspace.state[112];
acadoWorkspace.evGu[lRun1 * 27 + 23] = acadoWorkspace.state[113];
acadoWorkspace.evGu[lRun1 * 27 + 24] = acadoWorkspace.state[114];
acadoWorkspace.evGu[lRun1 * 27 + 25] = acadoWorkspace.state[115];
acadoWorkspace.evGu[lRun1 * 27 + 26] = acadoWorkspace.state[116];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 9;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = xd[3];
out[4] = xd[4];
out[5] = xd[5];
out[6] = xd[6];
out[7] = xd[7];
out[8] = xd[8];
out[9] = u[0];
out[10] = u[1];
out[11] = u[2];
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
out[8] = xd[8];
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
tmpQ2[96] = +tmpObjS[96];
tmpQ2[97] = +tmpObjS[97];
tmpQ2[98] = +tmpObjS[98];
tmpQ2[99] = +tmpObjS[99];
tmpQ2[100] = +tmpObjS[100];
tmpQ2[101] = +tmpObjS[101];
tmpQ2[102] = +tmpObjS[102];
tmpQ2[103] = +tmpObjS[103];
tmpQ2[104] = +tmpObjS[104];
tmpQ2[105] = +tmpObjS[105];
tmpQ2[106] = +tmpObjS[106];
tmpQ2[107] = +tmpObjS[107];
tmpQ1[0] = + tmpQ2[0];
tmpQ1[1] = + tmpQ2[1];
tmpQ1[2] = + tmpQ2[2];
tmpQ1[3] = + tmpQ2[3];
tmpQ1[4] = + tmpQ2[4];
tmpQ1[5] = + tmpQ2[5];
tmpQ1[6] = + tmpQ2[6];
tmpQ1[7] = + tmpQ2[7];
tmpQ1[8] = + tmpQ2[8];
tmpQ1[9] = + tmpQ2[12];
tmpQ1[10] = + tmpQ2[13];
tmpQ1[11] = + tmpQ2[14];
tmpQ1[12] = + tmpQ2[15];
tmpQ1[13] = + tmpQ2[16];
tmpQ1[14] = + tmpQ2[17];
tmpQ1[15] = + tmpQ2[18];
tmpQ1[16] = + tmpQ2[19];
tmpQ1[17] = + tmpQ2[20];
tmpQ1[18] = + tmpQ2[24];
tmpQ1[19] = + tmpQ2[25];
tmpQ1[20] = + tmpQ2[26];
tmpQ1[21] = + tmpQ2[27];
tmpQ1[22] = + tmpQ2[28];
tmpQ1[23] = + tmpQ2[29];
tmpQ1[24] = + tmpQ2[30];
tmpQ1[25] = + tmpQ2[31];
tmpQ1[26] = + tmpQ2[32];
tmpQ1[27] = + tmpQ2[36];
tmpQ1[28] = + tmpQ2[37];
tmpQ1[29] = + tmpQ2[38];
tmpQ1[30] = + tmpQ2[39];
tmpQ1[31] = + tmpQ2[40];
tmpQ1[32] = + tmpQ2[41];
tmpQ1[33] = + tmpQ2[42];
tmpQ1[34] = + tmpQ2[43];
tmpQ1[35] = + tmpQ2[44];
tmpQ1[36] = + tmpQ2[48];
tmpQ1[37] = + tmpQ2[49];
tmpQ1[38] = + tmpQ2[50];
tmpQ1[39] = + tmpQ2[51];
tmpQ1[40] = + tmpQ2[52];
tmpQ1[41] = + tmpQ2[53];
tmpQ1[42] = + tmpQ2[54];
tmpQ1[43] = + tmpQ2[55];
tmpQ1[44] = + tmpQ2[56];
tmpQ1[45] = + tmpQ2[60];
tmpQ1[46] = + tmpQ2[61];
tmpQ1[47] = + tmpQ2[62];
tmpQ1[48] = + tmpQ2[63];
tmpQ1[49] = + tmpQ2[64];
tmpQ1[50] = + tmpQ2[65];
tmpQ1[51] = + tmpQ2[66];
tmpQ1[52] = + tmpQ2[67];
tmpQ1[53] = + tmpQ2[68];
tmpQ1[54] = + tmpQ2[72];
tmpQ1[55] = + tmpQ2[73];
tmpQ1[56] = + tmpQ2[74];
tmpQ1[57] = + tmpQ2[75];
tmpQ1[58] = + tmpQ2[76];
tmpQ1[59] = + tmpQ2[77];
tmpQ1[60] = + tmpQ2[78];
tmpQ1[61] = + tmpQ2[79];
tmpQ1[62] = + tmpQ2[80];
tmpQ1[63] = + tmpQ2[84];
tmpQ1[64] = + tmpQ2[85];
tmpQ1[65] = + tmpQ2[86];
tmpQ1[66] = + tmpQ2[87];
tmpQ1[67] = + tmpQ2[88];
tmpQ1[68] = + tmpQ2[89];
tmpQ1[69] = + tmpQ2[90];
tmpQ1[70] = + tmpQ2[91];
tmpQ1[71] = + tmpQ2[92];
tmpQ1[72] = + tmpQ2[96];
tmpQ1[73] = + tmpQ2[97];
tmpQ1[74] = + tmpQ2[98];
tmpQ1[75] = + tmpQ2[99];
tmpQ1[76] = + tmpQ2[100];
tmpQ1[77] = + tmpQ2[101];
tmpQ1[78] = + tmpQ2[102];
tmpQ1[79] = + tmpQ2[103];
tmpQ1[80] = + tmpQ2[104];
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[108];
tmpR2[1] = +tmpObjS[109];
tmpR2[2] = +tmpObjS[110];
tmpR2[3] = +tmpObjS[111];
tmpR2[4] = +tmpObjS[112];
tmpR2[5] = +tmpObjS[113];
tmpR2[6] = +tmpObjS[114];
tmpR2[7] = +tmpObjS[115];
tmpR2[8] = +tmpObjS[116];
tmpR2[9] = +tmpObjS[117];
tmpR2[10] = +tmpObjS[118];
tmpR2[11] = +tmpObjS[119];
tmpR2[12] = +tmpObjS[120];
tmpR2[13] = +tmpObjS[121];
tmpR2[14] = +tmpObjS[122];
tmpR2[15] = +tmpObjS[123];
tmpR2[16] = +tmpObjS[124];
tmpR2[17] = +tmpObjS[125];
tmpR2[18] = +tmpObjS[126];
tmpR2[19] = +tmpObjS[127];
tmpR2[20] = +tmpObjS[128];
tmpR2[21] = +tmpObjS[129];
tmpR2[22] = +tmpObjS[130];
tmpR2[23] = +tmpObjS[131];
tmpR2[24] = +tmpObjS[132];
tmpR2[25] = +tmpObjS[133];
tmpR2[26] = +tmpObjS[134];
tmpR2[27] = +tmpObjS[135];
tmpR2[28] = +tmpObjS[136];
tmpR2[29] = +tmpObjS[137];
tmpR2[30] = +tmpObjS[138];
tmpR2[31] = +tmpObjS[139];
tmpR2[32] = +tmpObjS[140];
tmpR2[33] = +tmpObjS[141];
tmpR2[34] = +tmpObjS[142];
tmpR2[35] = +tmpObjS[143];
tmpR1[0] = + tmpR2[9];
tmpR1[1] = + tmpR2[10];
tmpR1[2] = + tmpR2[11];
tmpR1[3] = + tmpR2[21];
tmpR1[4] = + tmpR2[22];
tmpR1[5] = + tmpR2[23];
tmpR1[6] = + tmpR2[33];
tmpR1[7] = + tmpR2[34];
tmpR1[8] = + tmpR2[35];
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
tmpQN2[64] = +tmpObjSEndTerm[64];
tmpQN2[65] = +tmpObjSEndTerm[65];
tmpQN2[66] = +tmpObjSEndTerm[66];
tmpQN2[67] = +tmpObjSEndTerm[67];
tmpQN2[68] = +tmpObjSEndTerm[68];
tmpQN2[69] = +tmpObjSEndTerm[69];
tmpQN2[70] = +tmpObjSEndTerm[70];
tmpQN2[71] = +tmpObjSEndTerm[71];
tmpQN2[72] = +tmpObjSEndTerm[72];
tmpQN2[73] = +tmpObjSEndTerm[73];
tmpQN2[74] = +tmpObjSEndTerm[74];
tmpQN2[75] = +tmpObjSEndTerm[75];
tmpQN2[76] = +tmpObjSEndTerm[76];
tmpQN2[77] = +tmpObjSEndTerm[77];
tmpQN2[78] = +tmpObjSEndTerm[78];
tmpQN2[79] = +tmpObjSEndTerm[79];
tmpQN2[80] = +tmpObjSEndTerm[80];
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
tmpQN1[64] = + tmpQN2[64];
tmpQN1[65] = + tmpQN2[65];
tmpQN1[66] = + tmpQN2[66];
tmpQN1[67] = + tmpQN2[67];
tmpQN1[68] = + tmpQN2[68];
tmpQN1[69] = + tmpQN2[69];
tmpQN1[70] = + tmpQN2[70];
tmpQN1[71] = + tmpQN2[71];
tmpQN1[72] = + tmpQN2[72];
tmpQN1[73] = + tmpQN2[73];
tmpQN1[74] = + tmpQN2[74];
tmpQN1[75] = + tmpQN2[75];
tmpQN1[76] = + tmpQN2[76];
tmpQN1[77] = + tmpQN2[77];
tmpQN1[78] = + tmpQN2[78];
tmpQN1[79] = + tmpQN2[79];
tmpQN1[80] = + tmpQN2[80];
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 20; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 9];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 9 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 9 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 9 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 9 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[runObj * 9 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[runObj * 9 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[runObj * 9 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[runObj * 9 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.u[runObj * 3];
acadoWorkspace.objValueIn[10] = acadoVariables.u[runObj * 3 + 1];
acadoWorkspace.objValueIn[11] = acadoVariables.u[runObj * 3 + 2];
acadoWorkspace.objValueIn[12] = acadoVariables.od[runObj * 3];
acadoWorkspace.objValueIn[13] = acadoVariables.od[runObj * 3 + 1];
acadoWorkspace.objValueIn[14] = acadoVariables.od[runObj * 3 + 2];

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

acado_setObjQ1Q2( acadoVariables.W, &(acadoWorkspace.Q1[ runObj * 81 ]), &(acadoWorkspace.Q2[ runObj * 108 ]) );

acado_setObjR1R2( acadoVariables.W, &(acadoWorkspace.R1[ runObj * 9 ]), &(acadoWorkspace.R2[ runObj * 36 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[180];
acadoWorkspace.objValueIn[1] = acadoVariables.x[181];
acadoWorkspace.objValueIn[2] = acadoVariables.x[182];
acadoWorkspace.objValueIn[3] = acadoVariables.x[183];
acadoWorkspace.objValueIn[4] = acadoVariables.x[184];
acadoWorkspace.objValueIn[5] = acadoVariables.x[185];
acadoWorkspace.objValueIn[6] = acadoVariables.x[186];
acadoWorkspace.objValueIn[7] = acadoVariables.x[187];
acadoWorkspace.objValueIn[8] = acadoVariables.x[188];
acadoWorkspace.objValueIn[9] = acadoVariables.od[60];
acadoWorkspace.objValueIn[10] = acadoVariables.od[61];
acadoWorkspace.objValueIn[11] = acadoVariables.od[62];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.DyN[6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.DyN[7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.DyN[8] = acadoWorkspace.objValueOut[8];

acado_setObjQN1QN2( acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[3] + Gx1[2]*Gu1[6] + Gx1[3]*Gu1[9] + Gx1[4]*Gu1[12] + Gx1[5]*Gu1[15] + Gx1[6]*Gu1[18] + Gx1[7]*Gu1[21] + Gx1[8]*Gu1[24];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[4] + Gx1[2]*Gu1[7] + Gx1[3]*Gu1[10] + Gx1[4]*Gu1[13] + Gx1[5]*Gu1[16] + Gx1[6]*Gu1[19] + Gx1[7]*Gu1[22] + Gx1[8]*Gu1[25];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[5] + Gx1[2]*Gu1[8] + Gx1[3]*Gu1[11] + Gx1[4]*Gu1[14] + Gx1[5]*Gu1[17] + Gx1[6]*Gu1[20] + Gx1[7]*Gu1[23] + Gx1[8]*Gu1[26];
Gu2[3] = + Gx1[9]*Gu1[0] + Gx1[10]*Gu1[3] + Gx1[11]*Gu1[6] + Gx1[12]*Gu1[9] + Gx1[13]*Gu1[12] + Gx1[14]*Gu1[15] + Gx1[15]*Gu1[18] + Gx1[16]*Gu1[21] + Gx1[17]*Gu1[24];
Gu2[4] = + Gx1[9]*Gu1[1] + Gx1[10]*Gu1[4] + Gx1[11]*Gu1[7] + Gx1[12]*Gu1[10] + Gx1[13]*Gu1[13] + Gx1[14]*Gu1[16] + Gx1[15]*Gu1[19] + Gx1[16]*Gu1[22] + Gx1[17]*Gu1[25];
Gu2[5] = + Gx1[9]*Gu1[2] + Gx1[10]*Gu1[5] + Gx1[11]*Gu1[8] + Gx1[12]*Gu1[11] + Gx1[13]*Gu1[14] + Gx1[14]*Gu1[17] + Gx1[15]*Gu1[20] + Gx1[16]*Gu1[23] + Gx1[17]*Gu1[26];
Gu2[6] = + Gx1[18]*Gu1[0] + Gx1[19]*Gu1[3] + Gx1[20]*Gu1[6] + Gx1[21]*Gu1[9] + Gx1[22]*Gu1[12] + Gx1[23]*Gu1[15] + Gx1[24]*Gu1[18] + Gx1[25]*Gu1[21] + Gx1[26]*Gu1[24];
Gu2[7] = + Gx1[18]*Gu1[1] + Gx1[19]*Gu1[4] + Gx1[20]*Gu1[7] + Gx1[21]*Gu1[10] + Gx1[22]*Gu1[13] + Gx1[23]*Gu1[16] + Gx1[24]*Gu1[19] + Gx1[25]*Gu1[22] + Gx1[26]*Gu1[25];
Gu2[8] = + Gx1[18]*Gu1[2] + Gx1[19]*Gu1[5] + Gx1[20]*Gu1[8] + Gx1[21]*Gu1[11] + Gx1[22]*Gu1[14] + Gx1[23]*Gu1[17] + Gx1[24]*Gu1[20] + Gx1[25]*Gu1[23] + Gx1[26]*Gu1[26];
Gu2[9] = + Gx1[27]*Gu1[0] + Gx1[28]*Gu1[3] + Gx1[29]*Gu1[6] + Gx1[30]*Gu1[9] + Gx1[31]*Gu1[12] + Gx1[32]*Gu1[15] + Gx1[33]*Gu1[18] + Gx1[34]*Gu1[21] + Gx1[35]*Gu1[24];
Gu2[10] = + Gx1[27]*Gu1[1] + Gx1[28]*Gu1[4] + Gx1[29]*Gu1[7] + Gx1[30]*Gu1[10] + Gx1[31]*Gu1[13] + Gx1[32]*Gu1[16] + Gx1[33]*Gu1[19] + Gx1[34]*Gu1[22] + Gx1[35]*Gu1[25];
Gu2[11] = + Gx1[27]*Gu1[2] + Gx1[28]*Gu1[5] + Gx1[29]*Gu1[8] + Gx1[30]*Gu1[11] + Gx1[31]*Gu1[14] + Gx1[32]*Gu1[17] + Gx1[33]*Gu1[20] + Gx1[34]*Gu1[23] + Gx1[35]*Gu1[26];
Gu2[12] = + Gx1[36]*Gu1[0] + Gx1[37]*Gu1[3] + Gx1[38]*Gu1[6] + Gx1[39]*Gu1[9] + Gx1[40]*Gu1[12] + Gx1[41]*Gu1[15] + Gx1[42]*Gu1[18] + Gx1[43]*Gu1[21] + Gx1[44]*Gu1[24];
Gu2[13] = + Gx1[36]*Gu1[1] + Gx1[37]*Gu1[4] + Gx1[38]*Gu1[7] + Gx1[39]*Gu1[10] + Gx1[40]*Gu1[13] + Gx1[41]*Gu1[16] + Gx1[42]*Gu1[19] + Gx1[43]*Gu1[22] + Gx1[44]*Gu1[25];
Gu2[14] = + Gx1[36]*Gu1[2] + Gx1[37]*Gu1[5] + Gx1[38]*Gu1[8] + Gx1[39]*Gu1[11] + Gx1[40]*Gu1[14] + Gx1[41]*Gu1[17] + Gx1[42]*Gu1[20] + Gx1[43]*Gu1[23] + Gx1[44]*Gu1[26];
Gu2[15] = + Gx1[45]*Gu1[0] + Gx1[46]*Gu1[3] + Gx1[47]*Gu1[6] + Gx1[48]*Gu1[9] + Gx1[49]*Gu1[12] + Gx1[50]*Gu1[15] + Gx1[51]*Gu1[18] + Gx1[52]*Gu1[21] + Gx1[53]*Gu1[24];
Gu2[16] = + Gx1[45]*Gu1[1] + Gx1[46]*Gu1[4] + Gx1[47]*Gu1[7] + Gx1[48]*Gu1[10] + Gx1[49]*Gu1[13] + Gx1[50]*Gu1[16] + Gx1[51]*Gu1[19] + Gx1[52]*Gu1[22] + Gx1[53]*Gu1[25];
Gu2[17] = + Gx1[45]*Gu1[2] + Gx1[46]*Gu1[5] + Gx1[47]*Gu1[8] + Gx1[48]*Gu1[11] + Gx1[49]*Gu1[14] + Gx1[50]*Gu1[17] + Gx1[51]*Gu1[20] + Gx1[52]*Gu1[23] + Gx1[53]*Gu1[26];
Gu2[18] = + Gx1[54]*Gu1[0] + Gx1[55]*Gu1[3] + Gx1[56]*Gu1[6] + Gx1[57]*Gu1[9] + Gx1[58]*Gu1[12] + Gx1[59]*Gu1[15] + Gx1[60]*Gu1[18] + Gx1[61]*Gu1[21] + Gx1[62]*Gu1[24];
Gu2[19] = + Gx1[54]*Gu1[1] + Gx1[55]*Gu1[4] + Gx1[56]*Gu1[7] + Gx1[57]*Gu1[10] + Gx1[58]*Gu1[13] + Gx1[59]*Gu1[16] + Gx1[60]*Gu1[19] + Gx1[61]*Gu1[22] + Gx1[62]*Gu1[25];
Gu2[20] = + Gx1[54]*Gu1[2] + Gx1[55]*Gu1[5] + Gx1[56]*Gu1[8] + Gx1[57]*Gu1[11] + Gx1[58]*Gu1[14] + Gx1[59]*Gu1[17] + Gx1[60]*Gu1[20] + Gx1[61]*Gu1[23] + Gx1[62]*Gu1[26];
Gu2[21] = + Gx1[63]*Gu1[0] + Gx1[64]*Gu1[3] + Gx1[65]*Gu1[6] + Gx1[66]*Gu1[9] + Gx1[67]*Gu1[12] + Gx1[68]*Gu1[15] + Gx1[69]*Gu1[18] + Gx1[70]*Gu1[21] + Gx1[71]*Gu1[24];
Gu2[22] = + Gx1[63]*Gu1[1] + Gx1[64]*Gu1[4] + Gx1[65]*Gu1[7] + Gx1[66]*Gu1[10] + Gx1[67]*Gu1[13] + Gx1[68]*Gu1[16] + Gx1[69]*Gu1[19] + Gx1[70]*Gu1[22] + Gx1[71]*Gu1[25];
Gu2[23] = + Gx1[63]*Gu1[2] + Gx1[64]*Gu1[5] + Gx1[65]*Gu1[8] + Gx1[66]*Gu1[11] + Gx1[67]*Gu1[14] + Gx1[68]*Gu1[17] + Gx1[69]*Gu1[20] + Gx1[70]*Gu1[23] + Gx1[71]*Gu1[26];
Gu2[24] = + Gx1[72]*Gu1[0] + Gx1[73]*Gu1[3] + Gx1[74]*Gu1[6] + Gx1[75]*Gu1[9] + Gx1[76]*Gu1[12] + Gx1[77]*Gu1[15] + Gx1[78]*Gu1[18] + Gx1[79]*Gu1[21] + Gx1[80]*Gu1[24];
Gu2[25] = + Gx1[72]*Gu1[1] + Gx1[73]*Gu1[4] + Gx1[74]*Gu1[7] + Gx1[75]*Gu1[10] + Gx1[76]*Gu1[13] + Gx1[77]*Gu1[16] + Gx1[78]*Gu1[19] + Gx1[79]*Gu1[22] + Gx1[80]*Gu1[25];
Gu2[26] = + Gx1[72]*Gu1[2] + Gx1[73]*Gu1[5] + Gx1[74]*Gu1[8] + Gx1[75]*Gu1[11] + Gx1[76]*Gu1[14] + Gx1[77]*Gu1[17] + Gx1[78]*Gu1[20] + Gx1[79]*Gu1[23] + Gx1[80]*Gu1[26];
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
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 180) + (iCol * 3)] = + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9] + Gu1[12]*Gu2[12] + Gu1[15]*Gu2[15] + Gu1[18]*Gu2[18] + Gu1[21]*Gu2[21] + Gu1[24]*Gu2[24];
acadoWorkspace.H[(iRow * 180) + (iCol * 3 + 1)] = + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10] + Gu1[12]*Gu2[13] + Gu1[15]*Gu2[16] + Gu1[18]*Gu2[19] + Gu1[21]*Gu2[22] + Gu1[24]*Gu2[25];
acadoWorkspace.H[(iRow * 180) + (iCol * 3 + 2)] = + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11] + Gu1[12]*Gu2[14] + Gu1[15]*Gu2[17] + Gu1[18]*Gu2[20] + Gu1[21]*Gu2[23] + Gu1[24]*Gu2[26];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3)] = + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9] + Gu1[13]*Gu2[12] + Gu1[16]*Gu2[15] + Gu1[19]*Gu2[18] + Gu1[22]*Gu2[21] + Gu1[25]*Gu2[24];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3 + 1)] = + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10] + Gu1[13]*Gu2[13] + Gu1[16]*Gu2[16] + Gu1[19]*Gu2[19] + Gu1[22]*Gu2[22] + Gu1[25]*Gu2[25];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3 + 2)] = + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11] + Gu1[13]*Gu2[14] + Gu1[16]*Gu2[17] + Gu1[19]*Gu2[20] + Gu1[22]*Gu2[23] + Gu1[25]*Gu2[26];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3)] = + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9] + Gu1[14]*Gu2[12] + Gu1[17]*Gu2[15] + Gu1[20]*Gu2[18] + Gu1[23]*Gu2[21] + Gu1[26]*Gu2[24];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3 + 1)] = + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10] + Gu1[14]*Gu2[13] + Gu1[17]*Gu2[16] + Gu1[20]*Gu2[19] + Gu1[23]*Gu2[22] + Gu1[26]*Gu2[25];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3 + 2)] = + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11] + Gu1[14]*Gu2[14] + Gu1[17]*Gu2[17] + Gu1[20]*Gu2[20] + Gu1[23]*Gu2[23] + Gu1[26]*Gu2[26];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 183] = + Gu1[0]*Gu2[0] + Gu1[3]*Gu2[3] + Gu1[6]*Gu2[6] + Gu1[9]*Gu2[9] + Gu1[12]*Gu2[12] + Gu1[15]*Gu2[15] + Gu1[18]*Gu2[18] + Gu1[21]*Gu2[21] + Gu1[24]*Gu2[24] + R11[0];
acadoWorkspace.H[iRow * 183 + 1] = + Gu1[0]*Gu2[1] + Gu1[3]*Gu2[4] + Gu1[6]*Gu2[7] + Gu1[9]*Gu2[10] + Gu1[12]*Gu2[13] + Gu1[15]*Gu2[16] + Gu1[18]*Gu2[19] + Gu1[21]*Gu2[22] + Gu1[24]*Gu2[25] + R11[1];
acadoWorkspace.H[iRow * 183 + 2] = + Gu1[0]*Gu2[2] + Gu1[3]*Gu2[5] + Gu1[6]*Gu2[8] + Gu1[9]*Gu2[11] + Gu1[12]*Gu2[14] + Gu1[15]*Gu2[17] + Gu1[18]*Gu2[20] + Gu1[21]*Gu2[23] + Gu1[24]*Gu2[26] + R11[2];
acadoWorkspace.H[iRow * 183 + 60] = + Gu1[1]*Gu2[0] + Gu1[4]*Gu2[3] + Gu1[7]*Gu2[6] + Gu1[10]*Gu2[9] + Gu1[13]*Gu2[12] + Gu1[16]*Gu2[15] + Gu1[19]*Gu2[18] + Gu1[22]*Gu2[21] + Gu1[25]*Gu2[24] + R11[3];
acadoWorkspace.H[iRow * 183 + 61] = + Gu1[1]*Gu2[1] + Gu1[4]*Gu2[4] + Gu1[7]*Gu2[7] + Gu1[10]*Gu2[10] + Gu1[13]*Gu2[13] + Gu1[16]*Gu2[16] + Gu1[19]*Gu2[19] + Gu1[22]*Gu2[22] + Gu1[25]*Gu2[25] + R11[4];
acadoWorkspace.H[iRow * 183 + 62] = + Gu1[1]*Gu2[2] + Gu1[4]*Gu2[5] + Gu1[7]*Gu2[8] + Gu1[10]*Gu2[11] + Gu1[13]*Gu2[14] + Gu1[16]*Gu2[17] + Gu1[19]*Gu2[20] + Gu1[22]*Gu2[23] + Gu1[25]*Gu2[26] + R11[5];
acadoWorkspace.H[iRow * 183 + 120] = + Gu1[2]*Gu2[0] + Gu1[5]*Gu2[3] + Gu1[8]*Gu2[6] + Gu1[11]*Gu2[9] + Gu1[14]*Gu2[12] + Gu1[17]*Gu2[15] + Gu1[20]*Gu2[18] + Gu1[23]*Gu2[21] + Gu1[26]*Gu2[24] + R11[6];
acadoWorkspace.H[iRow * 183 + 121] = + Gu1[2]*Gu2[1] + Gu1[5]*Gu2[4] + Gu1[8]*Gu2[7] + Gu1[11]*Gu2[10] + Gu1[14]*Gu2[13] + Gu1[17]*Gu2[16] + Gu1[20]*Gu2[19] + Gu1[23]*Gu2[22] + Gu1[26]*Gu2[25] + R11[7];
acadoWorkspace.H[iRow * 183 + 122] = + Gu1[2]*Gu2[2] + Gu1[5]*Gu2[5] + Gu1[8]*Gu2[8] + Gu1[11]*Gu2[11] + Gu1[14]*Gu2[14] + Gu1[17]*Gu2[17] + Gu1[20]*Gu2[20] + Gu1[23]*Gu2[23] + Gu1[26]*Gu2[26] + R11[8];
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[9]*Gu1[3] + Gx1[18]*Gu1[6] + Gx1[27]*Gu1[9] + Gx1[36]*Gu1[12] + Gx1[45]*Gu1[15] + Gx1[54]*Gu1[18] + Gx1[63]*Gu1[21] + Gx1[72]*Gu1[24];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[9]*Gu1[4] + Gx1[18]*Gu1[7] + Gx1[27]*Gu1[10] + Gx1[36]*Gu1[13] + Gx1[45]*Gu1[16] + Gx1[54]*Gu1[19] + Gx1[63]*Gu1[22] + Gx1[72]*Gu1[25];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[9]*Gu1[5] + Gx1[18]*Gu1[8] + Gx1[27]*Gu1[11] + Gx1[36]*Gu1[14] + Gx1[45]*Gu1[17] + Gx1[54]*Gu1[20] + Gx1[63]*Gu1[23] + Gx1[72]*Gu1[26];
Gu2[3] = + Gx1[1]*Gu1[0] + Gx1[10]*Gu1[3] + Gx1[19]*Gu1[6] + Gx1[28]*Gu1[9] + Gx1[37]*Gu1[12] + Gx1[46]*Gu1[15] + Gx1[55]*Gu1[18] + Gx1[64]*Gu1[21] + Gx1[73]*Gu1[24];
Gu2[4] = + Gx1[1]*Gu1[1] + Gx1[10]*Gu1[4] + Gx1[19]*Gu1[7] + Gx1[28]*Gu1[10] + Gx1[37]*Gu1[13] + Gx1[46]*Gu1[16] + Gx1[55]*Gu1[19] + Gx1[64]*Gu1[22] + Gx1[73]*Gu1[25];
Gu2[5] = + Gx1[1]*Gu1[2] + Gx1[10]*Gu1[5] + Gx1[19]*Gu1[8] + Gx1[28]*Gu1[11] + Gx1[37]*Gu1[14] + Gx1[46]*Gu1[17] + Gx1[55]*Gu1[20] + Gx1[64]*Gu1[23] + Gx1[73]*Gu1[26];
Gu2[6] = + Gx1[2]*Gu1[0] + Gx1[11]*Gu1[3] + Gx1[20]*Gu1[6] + Gx1[29]*Gu1[9] + Gx1[38]*Gu1[12] + Gx1[47]*Gu1[15] + Gx1[56]*Gu1[18] + Gx1[65]*Gu1[21] + Gx1[74]*Gu1[24];
Gu2[7] = + Gx1[2]*Gu1[1] + Gx1[11]*Gu1[4] + Gx1[20]*Gu1[7] + Gx1[29]*Gu1[10] + Gx1[38]*Gu1[13] + Gx1[47]*Gu1[16] + Gx1[56]*Gu1[19] + Gx1[65]*Gu1[22] + Gx1[74]*Gu1[25];
Gu2[8] = + Gx1[2]*Gu1[2] + Gx1[11]*Gu1[5] + Gx1[20]*Gu1[8] + Gx1[29]*Gu1[11] + Gx1[38]*Gu1[14] + Gx1[47]*Gu1[17] + Gx1[56]*Gu1[20] + Gx1[65]*Gu1[23] + Gx1[74]*Gu1[26];
Gu2[9] = + Gx1[3]*Gu1[0] + Gx1[12]*Gu1[3] + Gx1[21]*Gu1[6] + Gx1[30]*Gu1[9] + Gx1[39]*Gu1[12] + Gx1[48]*Gu1[15] + Gx1[57]*Gu1[18] + Gx1[66]*Gu1[21] + Gx1[75]*Gu1[24];
Gu2[10] = + Gx1[3]*Gu1[1] + Gx1[12]*Gu1[4] + Gx1[21]*Gu1[7] + Gx1[30]*Gu1[10] + Gx1[39]*Gu1[13] + Gx1[48]*Gu1[16] + Gx1[57]*Gu1[19] + Gx1[66]*Gu1[22] + Gx1[75]*Gu1[25];
Gu2[11] = + Gx1[3]*Gu1[2] + Gx1[12]*Gu1[5] + Gx1[21]*Gu1[8] + Gx1[30]*Gu1[11] + Gx1[39]*Gu1[14] + Gx1[48]*Gu1[17] + Gx1[57]*Gu1[20] + Gx1[66]*Gu1[23] + Gx1[75]*Gu1[26];
Gu2[12] = + Gx1[4]*Gu1[0] + Gx1[13]*Gu1[3] + Gx1[22]*Gu1[6] + Gx1[31]*Gu1[9] + Gx1[40]*Gu1[12] + Gx1[49]*Gu1[15] + Gx1[58]*Gu1[18] + Gx1[67]*Gu1[21] + Gx1[76]*Gu1[24];
Gu2[13] = + Gx1[4]*Gu1[1] + Gx1[13]*Gu1[4] + Gx1[22]*Gu1[7] + Gx1[31]*Gu1[10] + Gx1[40]*Gu1[13] + Gx1[49]*Gu1[16] + Gx1[58]*Gu1[19] + Gx1[67]*Gu1[22] + Gx1[76]*Gu1[25];
Gu2[14] = + Gx1[4]*Gu1[2] + Gx1[13]*Gu1[5] + Gx1[22]*Gu1[8] + Gx1[31]*Gu1[11] + Gx1[40]*Gu1[14] + Gx1[49]*Gu1[17] + Gx1[58]*Gu1[20] + Gx1[67]*Gu1[23] + Gx1[76]*Gu1[26];
Gu2[15] = + Gx1[5]*Gu1[0] + Gx1[14]*Gu1[3] + Gx1[23]*Gu1[6] + Gx1[32]*Gu1[9] + Gx1[41]*Gu1[12] + Gx1[50]*Gu1[15] + Gx1[59]*Gu1[18] + Gx1[68]*Gu1[21] + Gx1[77]*Gu1[24];
Gu2[16] = + Gx1[5]*Gu1[1] + Gx1[14]*Gu1[4] + Gx1[23]*Gu1[7] + Gx1[32]*Gu1[10] + Gx1[41]*Gu1[13] + Gx1[50]*Gu1[16] + Gx1[59]*Gu1[19] + Gx1[68]*Gu1[22] + Gx1[77]*Gu1[25];
Gu2[17] = + Gx1[5]*Gu1[2] + Gx1[14]*Gu1[5] + Gx1[23]*Gu1[8] + Gx1[32]*Gu1[11] + Gx1[41]*Gu1[14] + Gx1[50]*Gu1[17] + Gx1[59]*Gu1[20] + Gx1[68]*Gu1[23] + Gx1[77]*Gu1[26];
Gu2[18] = + Gx1[6]*Gu1[0] + Gx1[15]*Gu1[3] + Gx1[24]*Gu1[6] + Gx1[33]*Gu1[9] + Gx1[42]*Gu1[12] + Gx1[51]*Gu1[15] + Gx1[60]*Gu1[18] + Gx1[69]*Gu1[21] + Gx1[78]*Gu1[24];
Gu2[19] = + Gx1[6]*Gu1[1] + Gx1[15]*Gu1[4] + Gx1[24]*Gu1[7] + Gx1[33]*Gu1[10] + Gx1[42]*Gu1[13] + Gx1[51]*Gu1[16] + Gx1[60]*Gu1[19] + Gx1[69]*Gu1[22] + Gx1[78]*Gu1[25];
Gu2[20] = + Gx1[6]*Gu1[2] + Gx1[15]*Gu1[5] + Gx1[24]*Gu1[8] + Gx1[33]*Gu1[11] + Gx1[42]*Gu1[14] + Gx1[51]*Gu1[17] + Gx1[60]*Gu1[20] + Gx1[69]*Gu1[23] + Gx1[78]*Gu1[26];
Gu2[21] = + Gx1[7]*Gu1[0] + Gx1[16]*Gu1[3] + Gx1[25]*Gu1[6] + Gx1[34]*Gu1[9] + Gx1[43]*Gu1[12] + Gx1[52]*Gu1[15] + Gx1[61]*Gu1[18] + Gx1[70]*Gu1[21] + Gx1[79]*Gu1[24];
Gu2[22] = + Gx1[7]*Gu1[1] + Gx1[16]*Gu1[4] + Gx1[25]*Gu1[7] + Gx1[34]*Gu1[10] + Gx1[43]*Gu1[13] + Gx1[52]*Gu1[16] + Gx1[61]*Gu1[19] + Gx1[70]*Gu1[22] + Gx1[79]*Gu1[25];
Gu2[23] = + Gx1[7]*Gu1[2] + Gx1[16]*Gu1[5] + Gx1[25]*Gu1[8] + Gx1[34]*Gu1[11] + Gx1[43]*Gu1[14] + Gx1[52]*Gu1[17] + Gx1[61]*Gu1[20] + Gx1[70]*Gu1[23] + Gx1[79]*Gu1[26];
Gu2[24] = + Gx1[8]*Gu1[0] + Gx1[17]*Gu1[3] + Gx1[26]*Gu1[6] + Gx1[35]*Gu1[9] + Gx1[44]*Gu1[12] + Gx1[53]*Gu1[15] + Gx1[62]*Gu1[18] + Gx1[71]*Gu1[21] + Gx1[80]*Gu1[24];
Gu2[25] = + Gx1[8]*Gu1[1] + Gx1[17]*Gu1[4] + Gx1[26]*Gu1[7] + Gx1[35]*Gu1[10] + Gx1[44]*Gu1[13] + Gx1[53]*Gu1[16] + Gx1[62]*Gu1[19] + Gx1[71]*Gu1[22] + Gx1[80]*Gu1[25];
Gu2[26] = + Gx1[8]*Gu1[2] + Gx1[17]*Gu1[5] + Gx1[26]*Gu1[8] + Gx1[35]*Gu1[11] + Gx1[44]*Gu1[14] + Gx1[53]*Gu1[17] + Gx1[62]*Gu1[20] + Gx1[71]*Gu1[23] + Gx1[80]*Gu1[26];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[3] + Q11[2]*Gu1[6] + Q11[3]*Gu1[9] + Q11[4]*Gu1[12] + Q11[5]*Gu1[15] + Q11[6]*Gu1[18] + Q11[7]*Gu1[21] + Q11[8]*Gu1[24] + Gu2[0];
Gu3[1] = + Q11[0]*Gu1[1] + Q11[1]*Gu1[4] + Q11[2]*Gu1[7] + Q11[3]*Gu1[10] + Q11[4]*Gu1[13] + Q11[5]*Gu1[16] + Q11[6]*Gu1[19] + Q11[7]*Gu1[22] + Q11[8]*Gu1[25] + Gu2[1];
Gu3[2] = + Q11[0]*Gu1[2] + Q11[1]*Gu1[5] + Q11[2]*Gu1[8] + Q11[3]*Gu1[11] + Q11[4]*Gu1[14] + Q11[5]*Gu1[17] + Q11[6]*Gu1[20] + Q11[7]*Gu1[23] + Q11[8]*Gu1[26] + Gu2[2];
Gu3[3] = + Q11[9]*Gu1[0] + Q11[10]*Gu1[3] + Q11[11]*Gu1[6] + Q11[12]*Gu1[9] + Q11[13]*Gu1[12] + Q11[14]*Gu1[15] + Q11[15]*Gu1[18] + Q11[16]*Gu1[21] + Q11[17]*Gu1[24] + Gu2[3];
Gu3[4] = + Q11[9]*Gu1[1] + Q11[10]*Gu1[4] + Q11[11]*Gu1[7] + Q11[12]*Gu1[10] + Q11[13]*Gu1[13] + Q11[14]*Gu1[16] + Q11[15]*Gu1[19] + Q11[16]*Gu1[22] + Q11[17]*Gu1[25] + Gu2[4];
Gu3[5] = + Q11[9]*Gu1[2] + Q11[10]*Gu1[5] + Q11[11]*Gu1[8] + Q11[12]*Gu1[11] + Q11[13]*Gu1[14] + Q11[14]*Gu1[17] + Q11[15]*Gu1[20] + Q11[16]*Gu1[23] + Q11[17]*Gu1[26] + Gu2[5];
Gu3[6] = + Q11[18]*Gu1[0] + Q11[19]*Gu1[3] + Q11[20]*Gu1[6] + Q11[21]*Gu1[9] + Q11[22]*Gu1[12] + Q11[23]*Gu1[15] + Q11[24]*Gu1[18] + Q11[25]*Gu1[21] + Q11[26]*Gu1[24] + Gu2[6];
Gu3[7] = + Q11[18]*Gu1[1] + Q11[19]*Gu1[4] + Q11[20]*Gu1[7] + Q11[21]*Gu1[10] + Q11[22]*Gu1[13] + Q11[23]*Gu1[16] + Q11[24]*Gu1[19] + Q11[25]*Gu1[22] + Q11[26]*Gu1[25] + Gu2[7];
Gu3[8] = + Q11[18]*Gu1[2] + Q11[19]*Gu1[5] + Q11[20]*Gu1[8] + Q11[21]*Gu1[11] + Q11[22]*Gu1[14] + Q11[23]*Gu1[17] + Q11[24]*Gu1[20] + Q11[25]*Gu1[23] + Q11[26]*Gu1[26] + Gu2[8];
Gu3[9] = + Q11[27]*Gu1[0] + Q11[28]*Gu1[3] + Q11[29]*Gu1[6] + Q11[30]*Gu1[9] + Q11[31]*Gu1[12] + Q11[32]*Gu1[15] + Q11[33]*Gu1[18] + Q11[34]*Gu1[21] + Q11[35]*Gu1[24] + Gu2[9];
Gu3[10] = + Q11[27]*Gu1[1] + Q11[28]*Gu1[4] + Q11[29]*Gu1[7] + Q11[30]*Gu1[10] + Q11[31]*Gu1[13] + Q11[32]*Gu1[16] + Q11[33]*Gu1[19] + Q11[34]*Gu1[22] + Q11[35]*Gu1[25] + Gu2[10];
Gu3[11] = + Q11[27]*Gu1[2] + Q11[28]*Gu1[5] + Q11[29]*Gu1[8] + Q11[30]*Gu1[11] + Q11[31]*Gu1[14] + Q11[32]*Gu1[17] + Q11[33]*Gu1[20] + Q11[34]*Gu1[23] + Q11[35]*Gu1[26] + Gu2[11];
Gu3[12] = + Q11[36]*Gu1[0] + Q11[37]*Gu1[3] + Q11[38]*Gu1[6] + Q11[39]*Gu1[9] + Q11[40]*Gu1[12] + Q11[41]*Gu1[15] + Q11[42]*Gu1[18] + Q11[43]*Gu1[21] + Q11[44]*Gu1[24] + Gu2[12];
Gu3[13] = + Q11[36]*Gu1[1] + Q11[37]*Gu1[4] + Q11[38]*Gu1[7] + Q11[39]*Gu1[10] + Q11[40]*Gu1[13] + Q11[41]*Gu1[16] + Q11[42]*Gu1[19] + Q11[43]*Gu1[22] + Q11[44]*Gu1[25] + Gu2[13];
Gu3[14] = + Q11[36]*Gu1[2] + Q11[37]*Gu1[5] + Q11[38]*Gu1[8] + Q11[39]*Gu1[11] + Q11[40]*Gu1[14] + Q11[41]*Gu1[17] + Q11[42]*Gu1[20] + Q11[43]*Gu1[23] + Q11[44]*Gu1[26] + Gu2[14];
Gu3[15] = + Q11[45]*Gu1[0] + Q11[46]*Gu1[3] + Q11[47]*Gu1[6] + Q11[48]*Gu1[9] + Q11[49]*Gu1[12] + Q11[50]*Gu1[15] + Q11[51]*Gu1[18] + Q11[52]*Gu1[21] + Q11[53]*Gu1[24] + Gu2[15];
Gu3[16] = + Q11[45]*Gu1[1] + Q11[46]*Gu1[4] + Q11[47]*Gu1[7] + Q11[48]*Gu1[10] + Q11[49]*Gu1[13] + Q11[50]*Gu1[16] + Q11[51]*Gu1[19] + Q11[52]*Gu1[22] + Q11[53]*Gu1[25] + Gu2[16];
Gu3[17] = + Q11[45]*Gu1[2] + Q11[46]*Gu1[5] + Q11[47]*Gu1[8] + Q11[48]*Gu1[11] + Q11[49]*Gu1[14] + Q11[50]*Gu1[17] + Q11[51]*Gu1[20] + Q11[52]*Gu1[23] + Q11[53]*Gu1[26] + Gu2[17];
Gu3[18] = + Q11[54]*Gu1[0] + Q11[55]*Gu1[3] + Q11[56]*Gu1[6] + Q11[57]*Gu1[9] + Q11[58]*Gu1[12] + Q11[59]*Gu1[15] + Q11[60]*Gu1[18] + Q11[61]*Gu1[21] + Q11[62]*Gu1[24] + Gu2[18];
Gu3[19] = + Q11[54]*Gu1[1] + Q11[55]*Gu1[4] + Q11[56]*Gu1[7] + Q11[57]*Gu1[10] + Q11[58]*Gu1[13] + Q11[59]*Gu1[16] + Q11[60]*Gu1[19] + Q11[61]*Gu1[22] + Q11[62]*Gu1[25] + Gu2[19];
Gu3[20] = + Q11[54]*Gu1[2] + Q11[55]*Gu1[5] + Q11[56]*Gu1[8] + Q11[57]*Gu1[11] + Q11[58]*Gu1[14] + Q11[59]*Gu1[17] + Q11[60]*Gu1[20] + Q11[61]*Gu1[23] + Q11[62]*Gu1[26] + Gu2[20];
Gu3[21] = + Q11[63]*Gu1[0] + Q11[64]*Gu1[3] + Q11[65]*Gu1[6] + Q11[66]*Gu1[9] + Q11[67]*Gu1[12] + Q11[68]*Gu1[15] + Q11[69]*Gu1[18] + Q11[70]*Gu1[21] + Q11[71]*Gu1[24] + Gu2[21];
Gu3[22] = + Q11[63]*Gu1[1] + Q11[64]*Gu1[4] + Q11[65]*Gu1[7] + Q11[66]*Gu1[10] + Q11[67]*Gu1[13] + Q11[68]*Gu1[16] + Q11[69]*Gu1[19] + Q11[70]*Gu1[22] + Q11[71]*Gu1[25] + Gu2[22];
Gu3[23] = + Q11[63]*Gu1[2] + Q11[64]*Gu1[5] + Q11[65]*Gu1[8] + Q11[66]*Gu1[11] + Q11[67]*Gu1[14] + Q11[68]*Gu1[17] + Q11[69]*Gu1[20] + Q11[70]*Gu1[23] + Q11[71]*Gu1[26] + Gu2[23];
Gu3[24] = + Q11[72]*Gu1[0] + Q11[73]*Gu1[3] + Q11[74]*Gu1[6] + Q11[75]*Gu1[9] + Q11[76]*Gu1[12] + Q11[77]*Gu1[15] + Q11[78]*Gu1[18] + Q11[79]*Gu1[21] + Q11[80]*Gu1[24] + Gu2[24];
Gu3[25] = + Q11[72]*Gu1[1] + Q11[73]*Gu1[4] + Q11[74]*Gu1[7] + Q11[75]*Gu1[10] + Q11[76]*Gu1[13] + Q11[77]*Gu1[16] + Q11[78]*Gu1[19] + Q11[79]*Gu1[22] + Q11[80]*Gu1[25] + Gu2[25];
Gu3[26] = + Q11[72]*Gu1[2] + Q11[73]*Gu1[5] + Q11[74]*Gu1[8] + Q11[75]*Gu1[11] + Q11[76]*Gu1[14] + Q11[77]*Gu1[17] + Q11[78]*Gu1[20] + Q11[79]*Gu1[23] + Q11[80]*Gu1[26] + Gu2[26];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[9]*w11[1] + Gx1[18]*w11[2] + Gx1[27]*w11[3] + Gx1[36]*w11[4] + Gx1[45]*w11[5] + Gx1[54]*w11[6] + Gx1[63]*w11[7] + Gx1[72]*w11[8] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[10]*w11[1] + Gx1[19]*w11[2] + Gx1[28]*w11[3] + Gx1[37]*w11[4] + Gx1[46]*w11[5] + Gx1[55]*w11[6] + Gx1[64]*w11[7] + Gx1[73]*w11[8] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[11]*w11[1] + Gx1[20]*w11[2] + Gx1[29]*w11[3] + Gx1[38]*w11[4] + Gx1[47]*w11[5] + Gx1[56]*w11[6] + Gx1[65]*w11[7] + Gx1[74]*w11[8] + w12[2];
w13[3] = + Gx1[3]*w11[0] + Gx1[12]*w11[1] + Gx1[21]*w11[2] + Gx1[30]*w11[3] + Gx1[39]*w11[4] + Gx1[48]*w11[5] + Gx1[57]*w11[6] + Gx1[66]*w11[7] + Gx1[75]*w11[8] + w12[3];
w13[4] = + Gx1[4]*w11[0] + Gx1[13]*w11[1] + Gx1[22]*w11[2] + Gx1[31]*w11[3] + Gx1[40]*w11[4] + Gx1[49]*w11[5] + Gx1[58]*w11[6] + Gx1[67]*w11[7] + Gx1[76]*w11[8] + w12[4];
w13[5] = + Gx1[5]*w11[0] + Gx1[14]*w11[1] + Gx1[23]*w11[2] + Gx1[32]*w11[3] + Gx1[41]*w11[4] + Gx1[50]*w11[5] + Gx1[59]*w11[6] + Gx1[68]*w11[7] + Gx1[77]*w11[8] + w12[5];
w13[6] = + Gx1[6]*w11[0] + Gx1[15]*w11[1] + Gx1[24]*w11[2] + Gx1[33]*w11[3] + Gx1[42]*w11[4] + Gx1[51]*w11[5] + Gx1[60]*w11[6] + Gx1[69]*w11[7] + Gx1[78]*w11[8] + w12[6];
w13[7] = + Gx1[7]*w11[0] + Gx1[16]*w11[1] + Gx1[25]*w11[2] + Gx1[34]*w11[3] + Gx1[43]*w11[4] + Gx1[52]*w11[5] + Gx1[61]*w11[6] + Gx1[70]*w11[7] + Gx1[79]*w11[8] + w12[7];
w13[8] = + Gx1[8]*w11[0] + Gx1[17]*w11[1] + Gx1[26]*w11[2] + Gx1[35]*w11[3] + Gx1[44]*w11[4] + Gx1[53]*w11[5] + Gx1[62]*w11[6] + Gx1[71]*w11[7] + Gx1[80]*w11[8] + w12[8];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[3]*w11[1] + Gu1[6]*w11[2] + Gu1[9]*w11[3] + Gu1[12]*w11[4] + Gu1[15]*w11[5] + Gu1[18]*w11[6] + Gu1[21]*w11[7] + Gu1[24]*w11[8];
U1[1] += + Gu1[1]*w11[0] + Gu1[4]*w11[1] + Gu1[7]*w11[2] + Gu1[10]*w11[3] + Gu1[13]*w11[4] + Gu1[16]*w11[5] + Gu1[19]*w11[6] + Gu1[22]*w11[7] + Gu1[25]*w11[8];
U1[2] += + Gu1[2]*w11[0] + Gu1[5]*w11[1] + Gu1[8]*w11[2] + Gu1[11]*w11[3] + Gu1[14]*w11[4] + Gu1[17]*w11[5] + Gu1[20]*w11[6] + Gu1[23]*w11[7] + Gu1[26]*w11[8];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + Q11[2]*w11[2] + Q11[3]*w11[3] + Q11[4]*w11[4] + Q11[5]*w11[5] + Q11[6]*w11[6] + Q11[7]*w11[7] + Q11[8]*w11[8] + w12[0];
w13[1] = + Q11[9]*w11[0] + Q11[10]*w11[1] + Q11[11]*w11[2] + Q11[12]*w11[3] + Q11[13]*w11[4] + Q11[14]*w11[5] + Q11[15]*w11[6] + Q11[16]*w11[7] + Q11[17]*w11[8] + w12[1];
w13[2] = + Q11[18]*w11[0] + Q11[19]*w11[1] + Q11[20]*w11[2] + Q11[21]*w11[3] + Q11[22]*w11[4] + Q11[23]*w11[5] + Q11[24]*w11[6] + Q11[25]*w11[7] + Q11[26]*w11[8] + w12[2];
w13[3] = + Q11[27]*w11[0] + Q11[28]*w11[1] + Q11[29]*w11[2] + Q11[30]*w11[3] + Q11[31]*w11[4] + Q11[32]*w11[5] + Q11[33]*w11[6] + Q11[34]*w11[7] + Q11[35]*w11[8] + w12[3];
w13[4] = + Q11[36]*w11[0] + Q11[37]*w11[1] + Q11[38]*w11[2] + Q11[39]*w11[3] + Q11[40]*w11[4] + Q11[41]*w11[5] + Q11[42]*w11[6] + Q11[43]*w11[7] + Q11[44]*w11[8] + w12[4];
w13[5] = + Q11[45]*w11[0] + Q11[46]*w11[1] + Q11[47]*w11[2] + Q11[48]*w11[3] + Q11[49]*w11[4] + Q11[50]*w11[5] + Q11[51]*w11[6] + Q11[52]*w11[7] + Q11[53]*w11[8] + w12[5];
w13[6] = + Q11[54]*w11[0] + Q11[55]*w11[1] + Q11[56]*w11[2] + Q11[57]*w11[3] + Q11[58]*w11[4] + Q11[59]*w11[5] + Q11[60]*w11[6] + Q11[61]*w11[7] + Q11[62]*w11[8] + w12[6];
w13[7] = + Q11[63]*w11[0] + Q11[64]*w11[1] + Q11[65]*w11[2] + Q11[66]*w11[3] + Q11[67]*w11[4] + Q11[68]*w11[5] + Q11[69]*w11[6] + Q11[70]*w11[7] + Q11[71]*w11[8] + w12[7];
w13[8] = + Q11[72]*w11[0] + Q11[73]*w11[1] + Q11[74]*w11[2] + Q11[75]*w11[3] + Q11[76]*w11[4] + Q11[77]*w11[5] + Q11[78]*w11[6] + Q11[79]*w11[7] + Q11[80]*w11[8] + w12[8];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7] + Gx1[8]*w11[8];
w12[1] += + Gx1[9]*w11[0] + Gx1[10]*w11[1] + Gx1[11]*w11[2] + Gx1[12]*w11[3] + Gx1[13]*w11[4] + Gx1[14]*w11[5] + Gx1[15]*w11[6] + Gx1[16]*w11[7] + Gx1[17]*w11[8];
w12[2] += + Gx1[18]*w11[0] + Gx1[19]*w11[1] + Gx1[20]*w11[2] + Gx1[21]*w11[3] + Gx1[22]*w11[4] + Gx1[23]*w11[5] + Gx1[24]*w11[6] + Gx1[25]*w11[7] + Gx1[26]*w11[8];
w12[3] += + Gx1[27]*w11[0] + Gx1[28]*w11[1] + Gx1[29]*w11[2] + Gx1[30]*w11[3] + Gx1[31]*w11[4] + Gx1[32]*w11[5] + Gx1[33]*w11[6] + Gx1[34]*w11[7] + Gx1[35]*w11[8];
w12[4] += + Gx1[36]*w11[0] + Gx1[37]*w11[1] + Gx1[38]*w11[2] + Gx1[39]*w11[3] + Gx1[40]*w11[4] + Gx1[41]*w11[5] + Gx1[42]*w11[6] + Gx1[43]*w11[7] + Gx1[44]*w11[8];
w12[5] += + Gx1[45]*w11[0] + Gx1[46]*w11[1] + Gx1[47]*w11[2] + Gx1[48]*w11[3] + Gx1[49]*w11[4] + Gx1[50]*w11[5] + Gx1[51]*w11[6] + Gx1[52]*w11[7] + Gx1[53]*w11[8];
w12[6] += + Gx1[54]*w11[0] + Gx1[55]*w11[1] + Gx1[56]*w11[2] + Gx1[57]*w11[3] + Gx1[58]*w11[4] + Gx1[59]*w11[5] + Gx1[60]*w11[6] + Gx1[61]*w11[7] + Gx1[62]*w11[8];
w12[7] += + Gx1[63]*w11[0] + Gx1[64]*w11[1] + Gx1[65]*w11[2] + Gx1[66]*w11[3] + Gx1[67]*w11[4] + Gx1[68]*w11[5] + Gx1[69]*w11[6] + Gx1[70]*w11[7] + Gx1[71]*w11[8];
w12[8] += + Gx1[72]*w11[0] + Gx1[73]*w11[1] + Gx1[74]*w11[2] + Gx1[75]*w11[3] + Gx1[76]*w11[4] + Gx1[77]*w11[5] + Gx1[78]*w11[6] + Gx1[79]*w11[7] + Gx1[80]*w11[8];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7] + Gx1[8]*w11[8];
w12[1] += + Gx1[9]*w11[0] + Gx1[10]*w11[1] + Gx1[11]*w11[2] + Gx1[12]*w11[3] + Gx1[13]*w11[4] + Gx1[14]*w11[5] + Gx1[15]*w11[6] + Gx1[16]*w11[7] + Gx1[17]*w11[8];
w12[2] += + Gx1[18]*w11[0] + Gx1[19]*w11[1] + Gx1[20]*w11[2] + Gx1[21]*w11[3] + Gx1[22]*w11[4] + Gx1[23]*w11[5] + Gx1[24]*w11[6] + Gx1[25]*w11[7] + Gx1[26]*w11[8];
w12[3] += + Gx1[27]*w11[0] + Gx1[28]*w11[1] + Gx1[29]*w11[2] + Gx1[30]*w11[3] + Gx1[31]*w11[4] + Gx1[32]*w11[5] + Gx1[33]*w11[6] + Gx1[34]*w11[7] + Gx1[35]*w11[8];
w12[4] += + Gx1[36]*w11[0] + Gx1[37]*w11[1] + Gx1[38]*w11[2] + Gx1[39]*w11[3] + Gx1[40]*w11[4] + Gx1[41]*w11[5] + Gx1[42]*w11[6] + Gx1[43]*w11[7] + Gx1[44]*w11[8];
w12[5] += + Gx1[45]*w11[0] + Gx1[46]*w11[1] + Gx1[47]*w11[2] + Gx1[48]*w11[3] + Gx1[49]*w11[4] + Gx1[50]*w11[5] + Gx1[51]*w11[6] + Gx1[52]*w11[7] + Gx1[53]*w11[8];
w12[6] += + Gx1[54]*w11[0] + Gx1[55]*w11[1] + Gx1[56]*w11[2] + Gx1[57]*w11[3] + Gx1[58]*w11[4] + Gx1[59]*w11[5] + Gx1[60]*w11[6] + Gx1[61]*w11[7] + Gx1[62]*w11[8];
w12[7] += + Gx1[63]*w11[0] + Gx1[64]*w11[1] + Gx1[65]*w11[2] + Gx1[66]*w11[3] + Gx1[67]*w11[4] + Gx1[68]*w11[5] + Gx1[69]*w11[6] + Gx1[70]*w11[7] + Gx1[71]*w11[8];
w12[8] += + Gx1[72]*w11[0] + Gx1[73]*w11[1] + Gx1[74]*w11[2] + Gx1[75]*w11[3] + Gx1[76]*w11[4] + Gx1[77]*w11[5] + Gx1[78]*w11[6] + Gx1[79]*w11[7] + Gx1[80]*w11[8];
w12[0] += + Gu1[0]*U1[0] + Gu1[1]*U1[1] + Gu1[2]*U1[2];
w12[1] += + Gu1[3]*U1[0] + Gu1[4]*U1[1] + Gu1[5]*U1[2];
w12[2] += + Gu1[6]*U1[0] + Gu1[7]*U1[1] + Gu1[8]*U1[2];
w12[3] += + Gu1[9]*U1[0] + Gu1[10]*U1[1] + Gu1[11]*U1[2];
w12[4] += + Gu1[12]*U1[0] + Gu1[13]*U1[1] + Gu1[14]*U1[2];
w12[5] += + Gu1[15]*U1[0] + Gu1[16]*U1[1] + Gu1[17]*U1[2];
w12[6] += + Gu1[18]*U1[0] + Gu1[19]*U1[1] + Gu1[20]*U1[2];
w12[7] += + Gu1[21]*U1[0] + Gu1[22]*U1[1] + Gu1[23]*U1[2];
w12[8] += + Gu1[24]*U1[0] + Gu1[25]*U1[1] + Gu1[26]*U1[2];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 180) + (iCol * 3)] = acadoWorkspace.H[(iCol * 180) + (iRow * 3)];
acadoWorkspace.H[(iRow * 180) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 180 + 60) + (iRow * 3)];
acadoWorkspace.H[(iRow * 180) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 180 + 120) + (iRow * 3)];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3)] = acadoWorkspace.H[(iCol * 180) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 180 + 60) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 180 + 60) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 180 + 120) + (iRow * 3 + 1)];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3)] = acadoWorkspace.H[(iCol * 180) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3 + 1)] = acadoWorkspace.H[(iCol * 180 + 60) + (iRow * 3 + 2)];
acadoWorkspace.H[(iRow * 180 + 120) + (iCol * 3 + 2)] = acadoWorkspace.H[(iCol * 180 + 120) + (iRow * 3 + 2)];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7] + R2[8]*Dy1[8] + R2[9]*Dy1[9] + R2[10]*Dy1[10] + R2[11]*Dy1[11];
RDy1[1] = + R2[12]*Dy1[0] + R2[13]*Dy1[1] + R2[14]*Dy1[2] + R2[15]*Dy1[3] + R2[16]*Dy1[4] + R2[17]*Dy1[5] + R2[18]*Dy1[6] + R2[19]*Dy1[7] + R2[20]*Dy1[8] + R2[21]*Dy1[9] + R2[22]*Dy1[10] + R2[23]*Dy1[11];
RDy1[2] = + R2[24]*Dy1[0] + R2[25]*Dy1[1] + R2[26]*Dy1[2] + R2[27]*Dy1[3] + R2[28]*Dy1[4] + R2[29]*Dy1[5] + R2[30]*Dy1[6] + R2[31]*Dy1[7] + R2[32]*Dy1[8] + R2[33]*Dy1[9] + R2[34]*Dy1[10] + R2[35]*Dy1[11];
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
QDy1[8] = + Q2[96]*Dy1[0] + Q2[97]*Dy1[1] + Q2[98]*Dy1[2] + Q2[99]*Dy1[3] + Q2[100]*Dy1[4] + Q2[101]*Dy1[5] + Q2[102]*Dy1[6] + Q2[103]*Dy1[7] + Q2[104]*Dy1[8] + Q2[105]*Dy1[9] + Q2[106]*Dy1[10] + Q2[107]*Dy1[11];
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
/** Row vector of size: 60 */
static const int xBoundIndices[ 60 ] = 
{ 15, 16, 17, 24, 25, 26, 33, 34, 35, 42, 43, 44, 51, 52, 53, 60, 61, 62, 69, 70, 71, 78, 79, 80, 87, 88, 89, 96, 97, 98, 105, 106, 107, 114, 115, 116, 123, 124, 125, 132, 133, 134, 141, 142, 143, 150, 151, 152, 159, 160, 161, 168, 169, 170, 177, 178, 179, 186, 187, 188 };
for (lRun2 = 0; lRun2 < 20; ++lRun2)
{
lRun3 = ((lRun2) * (lRun2 * -1 + 41)) / (2);
acado_moveGuE( &(acadoWorkspace.evGu[ lRun2 * 27 ]), &(acadoWorkspace.E[ lRun3 * 27 ]) );
for (lRun1 = 1; lRun1 < lRun2 * -1 + 20; ++lRun1)
{
acado_multGxGu( &(acadoWorkspace.evGx[ ((((lRun2) + (lRun1)) * (9)) * (9)) + (0) ]), &(acadoWorkspace.E[ (((((lRun3) + (lRun1)) - (1)) * (9)) * (3)) + (0) ]), &(acadoWorkspace.E[ ((((lRun3) + (lRun1)) * (9)) * (3)) + (0) ]) );
}

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ ((((((lRun3) - (lRun2)) + (20)) - (1)) * (9)) * (3)) + (0) ]), acadoWorkspace.W1 );
for (lRun1 = 19; lRun2 < lRun1; --lRun1)
{
acado_multBTW1( &(acadoWorkspace.evGu[ lRun1 * 27 ]), acadoWorkspace.W1, lRun1, lRun2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ lRun1 * 81 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ lRun1 * 81 ]), &(acadoWorkspace.E[ ((((((lRun3) + (lRun1)) - (lRun2)) - (1)) * (9)) * (3)) + (0) ]), acadoWorkspace.W2, acadoWorkspace.W1 );
}
acado_multBTW1_R1( &(acadoWorkspace.R1[ lRun2 * 9 ]), &(acadoWorkspace.evGu[ lRun2 * 27 ]), acadoWorkspace.W1, lRun2 );
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

for (lRun1 = 0; lRun1 < 180; ++lRun1)
acadoWorkspace.sbar[lRun1 + 9] = acadoWorkspace.d[lRun1];


for (lRun1 = 0; lRun1 < 60; ++lRun1)
{
lRun3 = xBoundIndices[ lRun1 ] - 9;
lRun4 = ((lRun3) / (9)) + (1);
for (lRun2 = 0; lRun2 < lRun4; ++lRun2)
{
lRun5 = ((((((lRun2) * (lRun2 * -1 + 39)) / (2)) + (lRun4)) - (1)) * (9)) + ((lRun3) % (9));
acadoWorkspace.A[(lRun1 * 60) + (lRun2 * 3)] = acadoWorkspace.E[lRun5 * 3];
acadoWorkspace.A[(lRun1 * 60) + (lRun2 * 3 + 1)] = acadoWorkspace.E[lRun5 * 3 + 1];
acadoWorkspace.A[(lRun1 * 60) + (lRun2 * 3 + 2)] = acadoWorkspace.E[lRun5 * 3 + 2];
}
}

}

void acado_condenseFdb(  )
{
int lRun1;
real_t tmp;

acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];
acadoWorkspace.Dx0[6] = acadoVariables.x0[6] - acadoVariables.x[6];
acadoWorkspace.Dx0[7] = acadoVariables.x0[7] - acadoVariables.x[7];
acadoWorkspace.Dx0[8] = acadoVariables.x0[8] - acadoVariables.x[8];
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
acadoWorkspace.DyN[8] -= acadoVariables.yN[8];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 36 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.g[ 3 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 72 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 108 ]), &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.g[ 9 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 144 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 180 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.g[ 15 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 216 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 252 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.g[ 21 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 288 ]), &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 324 ]), &(acadoWorkspace.Dy[ 108 ]), &(acadoWorkspace.g[ 27 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 360 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 396 ]), &(acadoWorkspace.Dy[ 132 ]), &(acadoWorkspace.g[ 33 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 432 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 468 ]), &(acadoWorkspace.Dy[ 156 ]), &(acadoWorkspace.g[ 39 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 504 ]), &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 540 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.g[ 45 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 576 ]), &(acadoWorkspace.Dy[ 192 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 612 ]), &(acadoWorkspace.Dy[ 204 ]), &(acadoWorkspace.g[ 51 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 648 ]), &(acadoWorkspace.Dy[ 216 ]), &(acadoWorkspace.g[ 54 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 684 ]), &(acadoWorkspace.Dy[ 228 ]), &(acadoWorkspace.g[ 57 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 108 ]), &(acadoWorkspace.Dy[ 12 ]), &(acadoWorkspace.QDy[ 9 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 216 ]), &(acadoWorkspace.Dy[ 24 ]), &(acadoWorkspace.QDy[ 18 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 324 ]), &(acadoWorkspace.Dy[ 36 ]), &(acadoWorkspace.QDy[ 27 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 432 ]), &(acadoWorkspace.Dy[ 48 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 540 ]), &(acadoWorkspace.Dy[ 60 ]), &(acadoWorkspace.QDy[ 45 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 648 ]), &(acadoWorkspace.Dy[ 72 ]), &(acadoWorkspace.QDy[ 54 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 756 ]), &(acadoWorkspace.Dy[ 84 ]), &(acadoWorkspace.QDy[ 63 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 864 ]), &(acadoWorkspace.Dy[ 96 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 972 ]), &(acadoWorkspace.Dy[ 108 ]), &(acadoWorkspace.QDy[ 81 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1080 ]), &(acadoWorkspace.Dy[ 120 ]), &(acadoWorkspace.QDy[ 90 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1188 ]), &(acadoWorkspace.Dy[ 132 ]), &(acadoWorkspace.QDy[ 99 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1296 ]), &(acadoWorkspace.Dy[ 144 ]), &(acadoWorkspace.QDy[ 108 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1404 ]), &(acadoWorkspace.Dy[ 156 ]), &(acadoWorkspace.QDy[ 117 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1512 ]), &(acadoWorkspace.Dy[ 168 ]), &(acadoWorkspace.QDy[ 126 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1620 ]), &(acadoWorkspace.Dy[ 180 ]), &(acadoWorkspace.QDy[ 135 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1728 ]), &(acadoWorkspace.Dy[ 192 ]), &(acadoWorkspace.QDy[ 144 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1836 ]), &(acadoWorkspace.Dy[ 204 ]), &(acadoWorkspace.QDy[ 153 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1944 ]), &(acadoWorkspace.Dy[ 216 ]), &(acadoWorkspace.QDy[ 162 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2052 ]), &(acadoWorkspace.Dy[ 228 ]), &(acadoWorkspace.QDy[ 171 ]) );

acadoWorkspace.QDy[180] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[8];
acadoWorkspace.QDy[181] = + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[14]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[15]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[16]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[17]*acadoWorkspace.DyN[8];
acadoWorkspace.QDy[182] = + acadoWorkspace.QN2[18]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[19]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[20]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[21]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[22]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[23]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[24]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[25]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[26]*acadoWorkspace.DyN[8];
acadoWorkspace.QDy[183] = + acadoWorkspace.QN2[27]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[28]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[29]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[30]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[31]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[32]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[33]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[34]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[35]*acadoWorkspace.DyN[8];
acadoWorkspace.QDy[184] = + acadoWorkspace.QN2[36]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[37]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[38]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[39]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[40]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[41]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[42]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[43]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[44]*acadoWorkspace.DyN[8];
acadoWorkspace.QDy[185] = + acadoWorkspace.QN2[45]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[46]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[47]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[48]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[49]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[50]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[51]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[52]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[53]*acadoWorkspace.DyN[8];
acadoWorkspace.QDy[186] = + acadoWorkspace.QN2[54]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[55]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[56]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[57]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[58]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[59]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[60]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[61]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[62]*acadoWorkspace.DyN[8];
acadoWorkspace.QDy[187] = + acadoWorkspace.QN2[63]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[64]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[65]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[66]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[67]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[68]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[69]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[70]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[71]*acadoWorkspace.DyN[8];
acadoWorkspace.QDy[188] = + acadoWorkspace.QN2[72]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[73]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[74]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[75]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[76]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[77]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[78]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[79]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[80]*acadoWorkspace.DyN[8];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.Dx0[8];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 9 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 27 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 243 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 45 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 405 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.sbar[ 54 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 486 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.sbar[ 63 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 567 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 81 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 729 ]), &(acadoWorkspace.sbar[ 81 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 810 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 99 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 891 ]), &(acadoWorkspace.sbar[ 99 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 972 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 117 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1053 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.sbar[ 126 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1134 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.sbar[ 135 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1215 ]), &(acadoWorkspace.sbar[ 135 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 153 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1377 ]), &(acadoWorkspace.sbar[ 153 ]), &(acadoWorkspace.sbar[ 162 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1458 ]), &(acadoWorkspace.sbar[ 162 ]), &(acadoWorkspace.sbar[ 171 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1539 ]), &(acadoWorkspace.sbar[ 171 ]), &(acadoWorkspace.sbar[ 180 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[188] + acadoWorkspace.QDy[180];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[9]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[10]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[11]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[12]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[13]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[14]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[15]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[16]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[17]*acadoWorkspace.sbar[188] + acadoWorkspace.QDy[181];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[18]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[19]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[20]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[21]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[22]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[23]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[24]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[25]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[26]*acadoWorkspace.sbar[188] + acadoWorkspace.QDy[182];
acadoWorkspace.w1[3] = + acadoWorkspace.QN1[27]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[28]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[29]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[30]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[31]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[32]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[33]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[34]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[35]*acadoWorkspace.sbar[188] + acadoWorkspace.QDy[183];
acadoWorkspace.w1[4] = + acadoWorkspace.QN1[36]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[37]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[38]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[39]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[40]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[41]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[42]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[43]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[44]*acadoWorkspace.sbar[188] + acadoWorkspace.QDy[184];
acadoWorkspace.w1[5] = + acadoWorkspace.QN1[45]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[46]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[47]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[48]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[49]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[50]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[51]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[52]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[53]*acadoWorkspace.sbar[188] + acadoWorkspace.QDy[185];
acadoWorkspace.w1[6] = + acadoWorkspace.QN1[54]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[55]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[56]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[57]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[58]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[59]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[60]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[61]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[62]*acadoWorkspace.sbar[188] + acadoWorkspace.QDy[186];
acadoWorkspace.w1[7] = + acadoWorkspace.QN1[63]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[64]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[65]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[66]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[67]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[68]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[69]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[70]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[71]*acadoWorkspace.sbar[188] + acadoWorkspace.QDy[187];
acadoWorkspace.w1[8] = + acadoWorkspace.QN1[72]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[73]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[74]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[75]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[76]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[77]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[78]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[79]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[80]*acadoWorkspace.sbar[188] + acadoWorkspace.QDy[188];
acado_macBTw1( &(acadoWorkspace.evGu[ 513 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 57 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1539 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 171 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1539 ]), &(acadoWorkspace.sbar[ 171 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 486 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 54 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1458 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 162 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1458 ]), &(acadoWorkspace.sbar[ 162 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 459 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 51 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1377 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 153 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1377 ]), &(acadoWorkspace.sbar[ 153 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 432 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 48 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1296 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 144 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1296 ]), &(acadoWorkspace.sbar[ 144 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 405 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 45 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1215 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 135 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1215 ]), &(acadoWorkspace.sbar[ 135 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 378 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 42 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1134 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 126 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1134 ]), &(acadoWorkspace.sbar[ 126 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 351 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 39 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1053 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 117 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1053 ]), &(acadoWorkspace.sbar[ 117 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 324 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 36 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 972 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 108 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 972 ]), &(acadoWorkspace.sbar[ 108 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 297 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 33 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 891 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 99 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 891 ]), &(acadoWorkspace.sbar[ 99 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 270 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 30 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 810 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 90 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 810 ]), &(acadoWorkspace.sbar[ 90 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 243 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 27 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 729 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 81 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 729 ]), &(acadoWorkspace.sbar[ 81 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 216 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 24 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 648 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 72 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 648 ]), &(acadoWorkspace.sbar[ 72 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 189 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 21 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 567 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 63 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 567 ]), &(acadoWorkspace.sbar[ 63 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 162 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 18 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 486 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 54 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 486 ]), &(acadoWorkspace.sbar[ 54 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 135 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 15 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 405 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 45 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 405 ]), &(acadoWorkspace.sbar[ 45 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 108 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 324 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 324 ]), &(acadoWorkspace.sbar[ 36 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 81 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 9 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 243 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 27 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 243 ]), &(acadoWorkspace.sbar[ 27 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 54 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 6 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 162 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 18 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 162 ]), &(acadoWorkspace.sbar[ 18 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 27 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 3 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 81 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 9 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 81 ]), &(acadoWorkspace.sbar[ 9 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
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

tmp = acadoWorkspace.sbar[15] + acadoVariables.x[15];
acadoWorkspace.lbA[0] = acadoVariables.lbAValues[0] - tmp;
acadoWorkspace.ubA[0] = acadoVariables.ubAValues[0] - tmp;
tmp = acadoWorkspace.sbar[16] + acadoVariables.x[16];
acadoWorkspace.lbA[1] = acadoVariables.lbAValues[1] - tmp;
acadoWorkspace.ubA[1] = acadoVariables.ubAValues[1] - tmp;
tmp = acadoWorkspace.sbar[17] + acadoVariables.x[17];
acadoWorkspace.lbA[2] = acadoVariables.lbAValues[2] - tmp;
acadoWorkspace.ubA[2] = acadoVariables.ubAValues[2] - tmp;
tmp = acadoWorkspace.sbar[24] + acadoVariables.x[24];
acadoWorkspace.lbA[3] = acadoVariables.lbAValues[3] - tmp;
acadoWorkspace.ubA[3] = acadoVariables.ubAValues[3] - tmp;
tmp = acadoWorkspace.sbar[25] + acadoVariables.x[25];
acadoWorkspace.lbA[4] = acadoVariables.lbAValues[4] - tmp;
acadoWorkspace.ubA[4] = acadoVariables.ubAValues[4] - tmp;
tmp = acadoWorkspace.sbar[26] + acadoVariables.x[26];
acadoWorkspace.lbA[5] = acadoVariables.lbAValues[5] - tmp;
acadoWorkspace.ubA[5] = acadoVariables.ubAValues[5] - tmp;
tmp = acadoWorkspace.sbar[33] + acadoVariables.x[33];
acadoWorkspace.lbA[6] = acadoVariables.lbAValues[6] - tmp;
acadoWorkspace.ubA[6] = acadoVariables.ubAValues[6] - tmp;
tmp = acadoWorkspace.sbar[34] + acadoVariables.x[34];
acadoWorkspace.lbA[7] = acadoVariables.lbAValues[7] - tmp;
acadoWorkspace.ubA[7] = acadoVariables.ubAValues[7] - tmp;
tmp = acadoWorkspace.sbar[35] + acadoVariables.x[35];
acadoWorkspace.lbA[8] = acadoVariables.lbAValues[8] - tmp;
acadoWorkspace.ubA[8] = acadoVariables.ubAValues[8] - tmp;
tmp = acadoWorkspace.sbar[42] + acadoVariables.x[42];
acadoWorkspace.lbA[9] = acadoVariables.lbAValues[9] - tmp;
acadoWorkspace.ubA[9] = acadoVariables.ubAValues[9] - tmp;
tmp = acadoWorkspace.sbar[43] + acadoVariables.x[43];
acadoWorkspace.lbA[10] = acadoVariables.lbAValues[10] - tmp;
acadoWorkspace.ubA[10] = acadoVariables.ubAValues[10] - tmp;
tmp = acadoWorkspace.sbar[44] + acadoVariables.x[44];
acadoWorkspace.lbA[11] = acadoVariables.lbAValues[11] - tmp;
acadoWorkspace.ubA[11] = acadoVariables.ubAValues[11] - tmp;
tmp = acadoWorkspace.sbar[51] + acadoVariables.x[51];
acadoWorkspace.lbA[12] = acadoVariables.lbAValues[12] - tmp;
acadoWorkspace.ubA[12] = acadoVariables.ubAValues[12] - tmp;
tmp = acadoWorkspace.sbar[52] + acadoVariables.x[52];
acadoWorkspace.lbA[13] = acadoVariables.lbAValues[13] - tmp;
acadoWorkspace.ubA[13] = acadoVariables.ubAValues[13] - tmp;
tmp = acadoWorkspace.sbar[53] + acadoVariables.x[53];
acadoWorkspace.lbA[14] = acadoVariables.lbAValues[14] - tmp;
acadoWorkspace.ubA[14] = acadoVariables.ubAValues[14] - tmp;
tmp = acadoWorkspace.sbar[60] + acadoVariables.x[60];
acadoWorkspace.lbA[15] = acadoVariables.lbAValues[15] - tmp;
acadoWorkspace.ubA[15] = acadoVariables.ubAValues[15] - tmp;
tmp = acadoWorkspace.sbar[61] + acadoVariables.x[61];
acadoWorkspace.lbA[16] = acadoVariables.lbAValues[16] - tmp;
acadoWorkspace.ubA[16] = acadoVariables.ubAValues[16] - tmp;
tmp = acadoWorkspace.sbar[62] + acadoVariables.x[62];
acadoWorkspace.lbA[17] = acadoVariables.lbAValues[17] - tmp;
acadoWorkspace.ubA[17] = acadoVariables.ubAValues[17] - tmp;
tmp = acadoWorkspace.sbar[69] + acadoVariables.x[69];
acadoWorkspace.lbA[18] = acadoVariables.lbAValues[18] - tmp;
acadoWorkspace.ubA[18] = acadoVariables.ubAValues[18] - tmp;
tmp = acadoWorkspace.sbar[70] + acadoVariables.x[70];
acadoWorkspace.lbA[19] = acadoVariables.lbAValues[19] - tmp;
acadoWorkspace.ubA[19] = acadoVariables.ubAValues[19] - tmp;
tmp = acadoWorkspace.sbar[71] + acadoVariables.x[71];
acadoWorkspace.lbA[20] = acadoVariables.lbAValues[20] - tmp;
acadoWorkspace.ubA[20] = acadoVariables.ubAValues[20] - tmp;
tmp = acadoWorkspace.sbar[78] + acadoVariables.x[78];
acadoWorkspace.lbA[21] = acadoVariables.lbAValues[21] - tmp;
acadoWorkspace.ubA[21] = acadoVariables.ubAValues[21] - tmp;
tmp = acadoWorkspace.sbar[79] + acadoVariables.x[79];
acadoWorkspace.lbA[22] = acadoVariables.lbAValues[22] - tmp;
acadoWorkspace.ubA[22] = acadoVariables.ubAValues[22] - tmp;
tmp = acadoWorkspace.sbar[80] + acadoVariables.x[80];
acadoWorkspace.lbA[23] = acadoVariables.lbAValues[23] - tmp;
acadoWorkspace.ubA[23] = acadoVariables.ubAValues[23] - tmp;
tmp = acadoWorkspace.sbar[87] + acadoVariables.x[87];
acadoWorkspace.lbA[24] = acadoVariables.lbAValues[24] - tmp;
acadoWorkspace.ubA[24] = acadoVariables.ubAValues[24] - tmp;
tmp = acadoWorkspace.sbar[88] + acadoVariables.x[88];
acadoWorkspace.lbA[25] = acadoVariables.lbAValues[25] - tmp;
acadoWorkspace.ubA[25] = acadoVariables.ubAValues[25] - tmp;
tmp = acadoWorkspace.sbar[89] + acadoVariables.x[89];
acadoWorkspace.lbA[26] = acadoVariables.lbAValues[26] - tmp;
acadoWorkspace.ubA[26] = acadoVariables.ubAValues[26] - tmp;
tmp = acadoWorkspace.sbar[96] + acadoVariables.x[96];
acadoWorkspace.lbA[27] = acadoVariables.lbAValues[27] - tmp;
acadoWorkspace.ubA[27] = acadoVariables.ubAValues[27] - tmp;
tmp = acadoWorkspace.sbar[97] + acadoVariables.x[97];
acadoWorkspace.lbA[28] = acadoVariables.lbAValues[28] - tmp;
acadoWorkspace.ubA[28] = acadoVariables.ubAValues[28] - tmp;
tmp = acadoWorkspace.sbar[98] + acadoVariables.x[98];
acadoWorkspace.lbA[29] = acadoVariables.lbAValues[29] - tmp;
acadoWorkspace.ubA[29] = acadoVariables.ubAValues[29] - tmp;
tmp = acadoWorkspace.sbar[105] + acadoVariables.x[105];
acadoWorkspace.lbA[30] = acadoVariables.lbAValues[30] - tmp;
acadoWorkspace.ubA[30] = acadoVariables.ubAValues[30] - tmp;
tmp = acadoWorkspace.sbar[106] + acadoVariables.x[106];
acadoWorkspace.lbA[31] = acadoVariables.lbAValues[31] - tmp;
acadoWorkspace.ubA[31] = acadoVariables.ubAValues[31] - tmp;
tmp = acadoWorkspace.sbar[107] + acadoVariables.x[107];
acadoWorkspace.lbA[32] = acadoVariables.lbAValues[32] - tmp;
acadoWorkspace.ubA[32] = acadoVariables.ubAValues[32] - tmp;
tmp = acadoWorkspace.sbar[114] + acadoVariables.x[114];
acadoWorkspace.lbA[33] = acadoVariables.lbAValues[33] - tmp;
acadoWorkspace.ubA[33] = acadoVariables.ubAValues[33] - tmp;
tmp = acadoWorkspace.sbar[115] + acadoVariables.x[115];
acadoWorkspace.lbA[34] = acadoVariables.lbAValues[34] - tmp;
acadoWorkspace.ubA[34] = acadoVariables.ubAValues[34] - tmp;
tmp = acadoWorkspace.sbar[116] + acadoVariables.x[116];
acadoWorkspace.lbA[35] = acadoVariables.lbAValues[35] - tmp;
acadoWorkspace.ubA[35] = acadoVariables.ubAValues[35] - tmp;
tmp = acadoWorkspace.sbar[123] + acadoVariables.x[123];
acadoWorkspace.lbA[36] = acadoVariables.lbAValues[36] - tmp;
acadoWorkspace.ubA[36] = acadoVariables.ubAValues[36] - tmp;
tmp = acadoWorkspace.sbar[124] + acadoVariables.x[124];
acadoWorkspace.lbA[37] = acadoVariables.lbAValues[37] - tmp;
acadoWorkspace.ubA[37] = acadoVariables.ubAValues[37] - tmp;
tmp = acadoWorkspace.sbar[125] + acadoVariables.x[125];
acadoWorkspace.lbA[38] = acadoVariables.lbAValues[38] - tmp;
acadoWorkspace.ubA[38] = acadoVariables.ubAValues[38] - tmp;
tmp = acadoWorkspace.sbar[132] + acadoVariables.x[132];
acadoWorkspace.lbA[39] = acadoVariables.lbAValues[39] - tmp;
acadoWorkspace.ubA[39] = acadoVariables.ubAValues[39] - tmp;
tmp = acadoWorkspace.sbar[133] + acadoVariables.x[133];
acadoWorkspace.lbA[40] = acadoVariables.lbAValues[40] - tmp;
acadoWorkspace.ubA[40] = acadoVariables.ubAValues[40] - tmp;
tmp = acadoWorkspace.sbar[134] + acadoVariables.x[134];
acadoWorkspace.lbA[41] = acadoVariables.lbAValues[41] - tmp;
acadoWorkspace.ubA[41] = acadoVariables.ubAValues[41] - tmp;
tmp = acadoWorkspace.sbar[141] + acadoVariables.x[141];
acadoWorkspace.lbA[42] = acadoVariables.lbAValues[42] - tmp;
acadoWorkspace.ubA[42] = acadoVariables.ubAValues[42] - tmp;
tmp = acadoWorkspace.sbar[142] + acadoVariables.x[142];
acadoWorkspace.lbA[43] = acadoVariables.lbAValues[43] - tmp;
acadoWorkspace.ubA[43] = acadoVariables.ubAValues[43] - tmp;
tmp = acadoWorkspace.sbar[143] + acadoVariables.x[143];
acadoWorkspace.lbA[44] = acadoVariables.lbAValues[44] - tmp;
acadoWorkspace.ubA[44] = acadoVariables.ubAValues[44] - tmp;
tmp = acadoWorkspace.sbar[150] + acadoVariables.x[150];
acadoWorkspace.lbA[45] = acadoVariables.lbAValues[45] - tmp;
acadoWorkspace.ubA[45] = acadoVariables.ubAValues[45] - tmp;
tmp = acadoWorkspace.sbar[151] + acadoVariables.x[151];
acadoWorkspace.lbA[46] = acadoVariables.lbAValues[46] - tmp;
acadoWorkspace.ubA[46] = acadoVariables.ubAValues[46] - tmp;
tmp = acadoWorkspace.sbar[152] + acadoVariables.x[152];
acadoWorkspace.lbA[47] = acadoVariables.lbAValues[47] - tmp;
acadoWorkspace.ubA[47] = acadoVariables.ubAValues[47] - tmp;
tmp = acadoWorkspace.sbar[159] + acadoVariables.x[159];
acadoWorkspace.lbA[48] = acadoVariables.lbAValues[48] - tmp;
acadoWorkspace.ubA[48] = acadoVariables.ubAValues[48] - tmp;
tmp = acadoWorkspace.sbar[160] + acadoVariables.x[160];
acadoWorkspace.lbA[49] = acadoVariables.lbAValues[49] - tmp;
acadoWorkspace.ubA[49] = acadoVariables.ubAValues[49] - tmp;
tmp = acadoWorkspace.sbar[161] + acadoVariables.x[161];
acadoWorkspace.lbA[50] = acadoVariables.lbAValues[50] - tmp;
acadoWorkspace.ubA[50] = acadoVariables.ubAValues[50] - tmp;
tmp = acadoWorkspace.sbar[168] + acadoVariables.x[168];
acadoWorkspace.lbA[51] = acadoVariables.lbAValues[51] - tmp;
acadoWorkspace.ubA[51] = acadoVariables.ubAValues[51] - tmp;
tmp = acadoWorkspace.sbar[169] + acadoVariables.x[169];
acadoWorkspace.lbA[52] = acadoVariables.lbAValues[52] - tmp;
acadoWorkspace.ubA[52] = acadoVariables.ubAValues[52] - tmp;
tmp = acadoWorkspace.sbar[170] + acadoVariables.x[170];
acadoWorkspace.lbA[53] = acadoVariables.lbAValues[53] - tmp;
acadoWorkspace.ubA[53] = acadoVariables.ubAValues[53] - tmp;
tmp = acadoWorkspace.sbar[177] + acadoVariables.x[177];
acadoWorkspace.lbA[54] = acadoVariables.lbAValues[54] - tmp;
acadoWorkspace.ubA[54] = acadoVariables.ubAValues[54] - tmp;
tmp = acadoWorkspace.sbar[178] + acadoVariables.x[178];
acadoWorkspace.lbA[55] = acadoVariables.lbAValues[55] - tmp;
acadoWorkspace.ubA[55] = acadoVariables.ubAValues[55] - tmp;
tmp = acadoWorkspace.sbar[179] + acadoVariables.x[179];
acadoWorkspace.lbA[56] = acadoVariables.lbAValues[56] - tmp;
acadoWorkspace.ubA[56] = acadoVariables.ubAValues[56] - tmp;
tmp = acadoWorkspace.sbar[186] + acadoVariables.x[186];
acadoWorkspace.lbA[57] = acadoVariables.lbAValues[57] - tmp;
acadoWorkspace.ubA[57] = acadoVariables.ubAValues[57] - tmp;
tmp = acadoWorkspace.sbar[187] + acadoVariables.x[187];
acadoWorkspace.lbA[58] = acadoVariables.lbAValues[58] - tmp;
acadoWorkspace.ubA[58] = acadoVariables.ubAValues[58] - tmp;
tmp = acadoWorkspace.sbar[188] + acadoVariables.x[188];
acadoWorkspace.lbA[59] = acadoVariables.lbAValues[59] - tmp;
acadoWorkspace.ubA[59] = acadoVariables.ubAValues[59] - tmp;

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
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.Dx0[8];
for (lRun1 = 0; lRun1 < 180; ++lRun1)
acadoWorkspace.sbar[lRun1 + 9] = acadoWorkspace.d[lRun1];

acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 9 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 81 ]), &(acadoWorkspace.evGu[ 27 ]), &(acadoWorkspace.x[ 3 ]), &(acadoWorkspace.sbar[ 9 ]), &(acadoWorkspace.sbar[ 18 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 162 ]), &(acadoWorkspace.evGu[ 54 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 27 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 243 ]), &(acadoWorkspace.evGu[ 81 ]), &(acadoWorkspace.x[ 9 ]), &(acadoWorkspace.sbar[ 27 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.evGu[ 108 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 45 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 405 ]), &(acadoWorkspace.evGu[ 135 ]), &(acadoWorkspace.x[ 15 ]), &(acadoWorkspace.sbar[ 45 ]), &(acadoWorkspace.sbar[ 54 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 486 ]), &(acadoWorkspace.evGu[ 162 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.sbar[ 63 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 567 ]), &(acadoWorkspace.evGu[ 189 ]), &(acadoWorkspace.x[ 21 ]), &(acadoWorkspace.sbar[ 63 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.evGu[ 216 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 81 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 729 ]), &(acadoWorkspace.evGu[ 243 ]), &(acadoWorkspace.x[ 27 ]), &(acadoWorkspace.sbar[ 81 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 810 ]), &(acadoWorkspace.evGu[ 270 ]), &(acadoWorkspace.x[ 30 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 99 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 891 ]), &(acadoWorkspace.evGu[ 297 ]), &(acadoWorkspace.x[ 33 ]), &(acadoWorkspace.sbar[ 99 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 972 ]), &(acadoWorkspace.evGu[ 324 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 117 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1053 ]), &(acadoWorkspace.evGu[ 351 ]), &(acadoWorkspace.x[ 39 ]), &(acadoWorkspace.sbar[ 117 ]), &(acadoWorkspace.sbar[ 126 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1134 ]), &(acadoWorkspace.evGu[ 378 ]), &(acadoWorkspace.x[ 42 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.sbar[ 135 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1215 ]), &(acadoWorkspace.evGu[ 405 ]), &(acadoWorkspace.x[ 45 ]), &(acadoWorkspace.sbar[ 135 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.evGu[ 432 ]), &(acadoWorkspace.x[ 48 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 153 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1377 ]), &(acadoWorkspace.evGu[ 459 ]), &(acadoWorkspace.x[ 51 ]), &(acadoWorkspace.sbar[ 153 ]), &(acadoWorkspace.sbar[ 162 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1458 ]), &(acadoWorkspace.evGu[ 486 ]), &(acadoWorkspace.x[ 54 ]), &(acadoWorkspace.sbar[ 162 ]), &(acadoWorkspace.sbar[ 171 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1539 ]), &(acadoWorkspace.evGu[ 513 ]), &(acadoWorkspace.x[ 57 ]), &(acadoWorkspace.sbar[ 171 ]), &(acadoWorkspace.sbar[ 180 ]) );
for (lRun1 = 0; lRun1 < 189; ++lRun1)
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
acadoVariables.lbValues[0] = -3.0000000000000000e+00;
acadoVariables.lbValues[1] = -3.0000000000000000e+00;
acadoVariables.lbValues[2] = -3.0000000000000000e+00;
acadoVariables.lbValues[3] = -3.0000000000000000e+00;
acadoVariables.lbValues[4] = -3.0000000000000000e+00;
acadoVariables.lbValues[5] = -3.0000000000000000e+00;
acadoVariables.lbValues[6] = -3.0000000000000000e+00;
acadoVariables.lbValues[7] = -3.0000000000000000e+00;
acadoVariables.lbValues[8] = -3.0000000000000000e+00;
acadoVariables.lbValues[9] = -3.0000000000000000e+00;
acadoVariables.lbValues[10] = -3.0000000000000000e+00;
acadoVariables.lbValues[11] = -3.0000000000000000e+00;
acadoVariables.lbValues[12] = -3.0000000000000000e+00;
acadoVariables.lbValues[13] = -3.0000000000000000e+00;
acadoVariables.lbValues[14] = -3.0000000000000000e+00;
acadoVariables.lbValues[15] = -3.0000000000000000e+00;
acadoVariables.lbValues[16] = -3.0000000000000000e+00;
acadoVariables.lbValues[17] = -3.0000000000000000e+00;
acadoVariables.lbValues[18] = -3.0000000000000000e+00;
acadoVariables.lbValues[19] = -3.0000000000000000e+00;
acadoVariables.lbValues[20] = -3.0000000000000000e+00;
acadoVariables.lbValues[21] = -3.0000000000000000e+00;
acadoVariables.lbValues[22] = -3.0000000000000000e+00;
acadoVariables.lbValues[23] = -3.0000000000000000e+00;
acadoVariables.lbValues[24] = -3.0000000000000000e+00;
acadoVariables.lbValues[25] = -3.0000000000000000e+00;
acadoVariables.lbValues[26] = -3.0000000000000000e+00;
acadoVariables.lbValues[27] = -3.0000000000000000e+00;
acadoVariables.lbValues[28] = -3.0000000000000000e+00;
acadoVariables.lbValues[29] = -3.0000000000000000e+00;
acadoVariables.lbValues[30] = -3.0000000000000000e+00;
acadoVariables.lbValues[31] = -3.0000000000000000e+00;
acadoVariables.lbValues[32] = -3.0000000000000000e+00;
acadoVariables.lbValues[33] = -3.0000000000000000e+00;
acadoVariables.lbValues[34] = -3.0000000000000000e+00;
acadoVariables.lbValues[35] = -3.0000000000000000e+00;
acadoVariables.lbValues[36] = -3.0000000000000000e+00;
acadoVariables.lbValues[37] = -3.0000000000000000e+00;
acadoVariables.lbValues[38] = -3.0000000000000000e+00;
acadoVariables.lbValues[39] = -3.0000000000000000e+00;
acadoVariables.lbValues[40] = -3.0000000000000000e+00;
acadoVariables.lbValues[41] = -3.0000000000000000e+00;
acadoVariables.lbValues[42] = -3.0000000000000000e+00;
acadoVariables.lbValues[43] = -3.0000000000000000e+00;
acadoVariables.lbValues[44] = -3.0000000000000000e+00;
acadoVariables.lbValues[45] = -3.0000000000000000e+00;
acadoVariables.lbValues[46] = -3.0000000000000000e+00;
acadoVariables.lbValues[47] = -3.0000000000000000e+00;
acadoVariables.lbValues[48] = -3.0000000000000000e+00;
acadoVariables.lbValues[49] = -3.0000000000000000e+00;
acadoVariables.lbValues[50] = -3.0000000000000000e+00;
acadoVariables.lbValues[51] = -3.0000000000000000e+00;
acadoVariables.lbValues[52] = -3.0000000000000000e+00;
acadoVariables.lbValues[53] = -3.0000000000000000e+00;
acadoVariables.lbValues[54] = -3.0000000000000000e+00;
acadoVariables.lbValues[55] = -3.0000000000000000e+00;
acadoVariables.lbValues[56] = -3.0000000000000000e+00;
acadoVariables.lbValues[57] = -3.0000000000000000e+00;
acadoVariables.lbValues[58] = -3.0000000000000000e+00;
acadoVariables.lbValues[59] = -3.0000000000000000e+00;
acadoVariables.ubValues[0] = 3.0000000000000000e+00;
acadoVariables.ubValues[1] = 3.0000000000000000e+00;
acadoVariables.ubValues[2] = 3.0000000000000000e+00;
acadoVariables.ubValues[3] = 3.0000000000000000e+00;
acadoVariables.ubValues[4] = 3.0000000000000000e+00;
acadoVariables.ubValues[5] = 3.0000000000000000e+00;
acadoVariables.ubValues[6] = 3.0000000000000000e+00;
acadoVariables.ubValues[7] = 3.0000000000000000e+00;
acadoVariables.ubValues[8] = 3.0000000000000000e+00;
acadoVariables.ubValues[9] = 3.0000000000000000e+00;
acadoVariables.ubValues[10] = 3.0000000000000000e+00;
acadoVariables.ubValues[11] = 3.0000000000000000e+00;
acadoVariables.ubValues[12] = 3.0000000000000000e+00;
acadoVariables.ubValues[13] = 3.0000000000000000e+00;
acadoVariables.ubValues[14] = 3.0000000000000000e+00;
acadoVariables.ubValues[15] = 3.0000000000000000e+00;
acadoVariables.ubValues[16] = 3.0000000000000000e+00;
acadoVariables.ubValues[17] = 3.0000000000000000e+00;
acadoVariables.ubValues[18] = 3.0000000000000000e+00;
acadoVariables.ubValues[19] = 3.0000000000000000e+00;
acadoVariables.ubValues[20] = 3.0000000000000000e+00;
acadoVariables.ubValues[21] = 3.0000000000000000e+00;
acadoVariables.ubValues[22] = 3.0000000000000000e+00;
acadoVariables.ubValues[23] = 3.0000000000000000e+00;
acadoVariables.ubValues[24] = 3.0000000000000000e+00;
acadoVariables.ubValues[25] = 3.0000000000000000e+00;
acadoVariables.ubValues[26] = 3.0000000000000000e+00;
acadoVariables.ubValues[27] = 3.0000000000000000e+00;
acadoVariables.ubValues[28] = 3.0000000000000000e+00;
acadoVariables.ubValues[29] = 3.0000000000000000e+00;
acadoVariables.ubValues[30] = 3.0000000000000000e+00;
acadoVariables.ubValues[31] = 3.0000000000000000e+00;
acadoVariables.ubValues[32] = 3.0000000000000000e+00;
acadoVariables.ubValues[33] = 3.0000000000000000e+00;
acadoVariables.ubValues[34] = 3.0000000000000000e+00;
acadoVariables.ubValues[35] = 3.0000000000000000e+00;
acadoVariables.ubValues[36] = 3.0000000000000000e+00;
acadoVariables.ubValues[37] = 3.0000000000000000e+00;
acadoVariables.ubValues[38] = 3.0000000000000000e+00;
acadoVariables.ubValues[39] = 3.0000000000000000e+00;
acadoVariables.ubValues[40] = 3.0000000000000000e+00;
acadoVariables.ubValues[41] = 3.0000000000000000e+00;
acadoVariables.ubValues[42] = 3.0000000000000000e+00;
acadoVariables.ubValues[43] = 3.0000000000000000e+00;
acadoVariables.ubValues[44] = 3.0000000000000000e+00;
acadoVariables.ubValues[45] = 3.0000000000000000e+00;
acadoVariables.ubValues[46] = 3.0000000000000000e+00;
acadoVariables.ubValues[47] = 3.0000000000000000e+00;
acadoVariables.ubValues[48] = 3.0000000000000000e+00;
acadoVariables.ubValues[49] = 3.0000000000000000e+00;
acadoVariables.ubValues[50] = 3.0000000000000000e+00;
acadoVariables.ubValues[51] = 3.0000000000000000e+00;
acadoVariables.ubValues[52] = 3.0000000000000000e+00;
acadoVariables.ubValues[53] = 3.0000000000000000e+00;
acadoVariables.ubValues[54] = 3.0000000000000000e+00;
acadoVariables.ubValues[55] = 3.0000000000000000e+00;
acadoVariables.ubValues[56] = 3.0000000000000000e+00;
acadoVariables.ubValues[57] = 3.0000000000000000e+00;
acadoVariables.ubValues[58] = 3.0000000000000000e+00;
acadoVariables.ubValues[59] = 3.0000000000000000e+00;
acadoVariables.lbAValues[0] = -2.0000000000000000e+00;
acadoVariables.lbAValues[1] = -2.0000000000000000e+00;
acadoVariables.lbAValues[2] = -2.0000000000000000e+00;
acadoVariables.lbAValues[3] = -2.0000000000000000e+00;
acadoVariables.lbAValues[4] = -2.0000000000000000e+00;
acadoVariables.lbAValues[5] = -2.0000000000000000e+00;
acadoVariables.lbAValues[6] = -2.0000000000000000e+00;
acadoVariables.lbAValues[7] = -2.0000000000000000e+00;
acadoVariables.lbAValues[8] = -2.0000000000000000e+00;
acadoVariables.lbAValues[9] = -2.0000000000000000e+00;
acadoVariables.lbAValues[10] = -2.0000000000000000e+00;
acadoVariables.lbAValues[11] = -2.0000000000000000e+00;
acadoVariables.lbAValues[12] = -2.0000000000000000e+00;
acadoVariables.lbAValues[13] = -2.0000000000000000e+00;
acadoVariables.lbAValues[14] = -2.0000000000000000e+00;
acadoVariables.lbAValues[15] = -2.0000000000000000e+00;
acadoVariables.lbAValues[16] = -2.0000000000000000e+00;
acadoVariables.lbAValues[17] = -2.0000000000000000e+00;
acadoVariables.lbAValues[18] = -2.0000000000000000e+00;
acadoVariables.lbAValues[19] = -2.0000000000000000e+00;
acadoVariables.lbAValues[20] = -2.0000000000000000e+00;
acadoVariables.lbAValues[21] = -2.0000000000000000e+00;
acadoVariables.lbAValues[22] = -2.0000000000000000e+00;
acadoVariables.lbAValues[23] = -2.0000000000000000e+00;
acadoVariables.lbAValues[24] = -2.0000000000000000e+00;
acadoVariables.lbAValues[25] = -2.0000000000000000e+00;
acadoVariables.lbAValues[26] = -2.0000000000000000e+00;
acadoVariables.lbAValues[27] = -2.0000000000000000e+00;
acadoVariables.lbAValues[28] = -2.0000000000000000e+00;
acadoVariables.lbAValues[29] = -2.0000000000000000e+00;
acadoVariables.lbAValues[30] = -2.0000000000000000e+00;
acadoVariables.lbAValues[31] = -2.0000000000000000e+00;
acadoVariables.lbAValues[32] = -2.0000000000000000e+00;
acadoVariables.lbAValues[33] = -2.0000000000000000e+00;
acadoVariables.lbAValues[34] = -2.0000000000000000e+00;
acadoVariables.lbAValues[35] = -2.0000000000000000e+00;
acadoVariables.lbAValues[36] = -2.0000000000000000e+00;
acadoVariables.lbAValues[37] = -2.0000000000000000e+00;
acadoVariables.lbAValues[38] = -2.0000000000000000e+00;
acadoVariables.lbAValues[39] = -2.0000000000000000e+00;
acadoVariables.lbAValues[40] = -2.0000000000000000e+00;
acadoVariables.lbAValues[41] = -2.0000000000000000e+00;
acadoVariables.lbAValues[42] = -2.0000000000000000e+00;
acadoVariables.lbAValues[43] = -2.0000000000000000e+00;
acadoVariables.lbAValues[44] = -2.0000000000000000e+00;
acadoVariables.lbAValues[45] = -2.0000000000000000e+00;
acadoVariables.lbAValues[46] = -2.0000000000000000e+00;
acadoVariables.lbAValues[47] = -2.0000000000000000e+00;
acadoVariables.lbAValues[48] = -2.0000000000000000e+00;
acadoVariables.lbAValues[49] = -2.0000000000000000e+00;
acadoVariables.lbAValues[50] = -2.0000000000000000e+00;
acadoVariables.lbAValues[51] = -2.0000000000000000e+00;
acadoVariables.lbAValues[52] = -2.0000000000000000e+00;
acadoVariables.lbAValues[53] = -2.0000000000000000e+00;
acadoVariables.lbAValues[54] = -2.0000000000000000e+00;
acadoVariables.lbAValues[55] = -2.0000000000000000e+00;
acadoVariables.lbAValues[56] = -2.0000000000000000e+00;
acadoVariables.lbAValues[57] = -2.0000000000000000e+00;
acadoVariables.lbAValues[58] = -2.0000000000000000e+00;
acadoVariables.lbAValues[59] = -2.0000000000000000e+00;
acadoVariables.ubAValues[0] = 2.0000000000000000e+00;
acadoVariables.ubAValues[1] = 2.0000000000000000e+00;
acadoVariables.ubAValues[2] = 2.0000000000000000e+00;
acadoVariables.ubAValues[3] = 2.0000000000000000e+00;
acadoVariables.ubAValues[4] = 2.0000000000000000e+00;
acadoVariables.ubAValues[5] = 2.0000000000000000e+00;
acadoVariables.ubAValues[6] = 2.0000000000000000e+00;
acadoVariables.ubAValues[7] = 2.0000000000000000e+00;
acadoVariables.ubAValues[8] = 2.0000000000000000e+00;
acadoVariables.ubAValues[9] = 2.0000000000000000e+00;
acadoVariables.ubAValues[10] = 2.0000000000000000e+00;
acadoVariables.ubAValues[11] = 2.0000000000000000e+00;
acadoVariables.ubAValues[12] = 2.0000000000000000e+00;
acadoVariables.ubAValues[13] = 2.0000000000000000e+00;
acadoVariables.ubAValues[14] = 2.0000000000000000e+00;
acadoVariables.ubAValues[15] = 2.0000000000000000e+00;
acadoVariables.ubAValues[16] = 2.0000000000000000e+00;
acadoVariables.ubAValues[17] = 2.0000000000000000e+00;
acadoVariables.ubAValues[18] = 2.0000000000000000e+00;
acadoVariables.ubAValues[19] = 2.0000000000000000e+00;
acadoVariables.ubAValues[20] = 2.0000000000000000e+00;
acadoVariables.ubAValues[21] = 2.0000000000000000e+00;
acadoVariables.ubAValues[22] = 2.0000000000000000e+00;
acadoVariables.ubAValues[23] = 2.0000000000000000e+00;
acadoVariables.ubAValues[24] = 2.0000000000000000e+00;
acadoVariables.ubAValues[25] = 2.0000000000000000e+00;
acadoVariables.ubAValues[26] = 2.0000000000000000e+00;
acadoVariables.ubAValues[27] = 2.0000000000000000e+00;
acadoVariables.ubAValues[28] = 2.0000000000000000e+00;
acadoVariables.ubAValues[29] = 2.0000000000000000e+00;
acadoVariables.ubAValues[30] = 2.0000000000000000e+00;
acadoVariables.ubAValues[31] = 2.0000000000000000e+00;
acadoVariables.ubAValues[32] = 2.0000000000000000e+00;
acadoVariables.ubAValues[33] = 2.0000000000000000e+00;
acadoVariables.ubAValues[34] = 2.0000000000000000e+00;
acadoVariables.ubAValues[35] = 2.0000000000000000e+00;
acadoVariables.ubAValues[36] = 2.0000000000000000e+00;
acadoVariables.ubAValues[37] = 2.0000000000000000e+00;
acadoVariables.ubAValues[38] = 2.0000000000000000e+00;
acadoVariables.ubAValues[39] = 2.0000000000000000e+00;
acadoVariables.ubAValues[40] = 2.0000000000000000e+00;
acadoVariables.ubAValues[41] = 2.0000000000000000e+00;
acadoVariables.ubAValues[42] = 2.0000000000000000e+00;
acadoVariables.ubAValues[43] = 2.0000000000000000e+00;
acadoVariables.ubAValues[44] = 2.0000000000000000e+00;
acadoVariables.ubAValues[45] = 2.0000000000000000e+00;
acadoVariables.ubAValues[46] = 2.0000000000000000e+00;
acadoVariables.ubAValues[47] = 2.0000000000000000e+00;
acadoVariables.ubAValues[48] = 2.0000000000000000e+00;
acadoVariables.ubAValues[49] = 2.0000000000000000e+00;
acadoVariables.ubAValues[50] = 2.0000000000000000e+00;
acadoVariables.ubAValues[51] = 2.0000000000000000e+00;
acadoVariables.ubAValues[52] = 2.0000000000000000e+00;
acadoVariables.ubAValues[53] = 2.0000000000000000e+00;
acadoVariables.ubAValues[54] = 2.0000000000000000e+00;
acadoVariables.ubAValues[55] = 2.0000000000000000e+00;
acadoVariables.ubAValues[56] = 2.0000000000000000e+00;
acadoVariables.ubAValues[57] = 2.0000000000000000e+00;
acadoVariables.ubAValues[58] = 2.0000000000000000e+00;
acadoVariables.ubAValues[59] = 2.0000000000000000e+00;
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 20; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 9];
acadoWorkspace.state[1] = acadoVariables.x[index * 9 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 9 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 9 + 3];
acadoWorkspace.state[4] = acadoVariables.x[index * 9 + 4];
acadoWorkspace.state[5] = acadoVariables.x[index * 9 + 5];
acadoWorkspace.state[6] = acadoVariables.x[index * 9 + 6];
acadoWorkspace.state[7] = acadoVariables.x[index * 9 + 7];
acadoWorkspace.state[8] = acadoVariables.x[index * 9 + 8];
acadoWorkspace.state[117] = acadoVariables.u[index * 3];
acadoWorkspace.state[118] = acadoVariables.u[index * 3 + 1];
acadoWorkspace.state[119] = acadoVariables.u[index * 3 + 2];
acadoWorkspace.state[120] = acadoVariables.od[index * 3];
acadoWorkspace.state[121] = acadoVariables.od[index * 3 + 1];
acadoWorkspace.state[122] = acadoVariables.od[index * 3 + 2];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 9 + 9] = acadoWorkspace.state[0];
acadoVariables.x[index * 9 + 10] = acadoWorkspace.state[1];
acadoVariables.x[index * 9 + 11] = acadoWorkspace.state[2];
acadoVariables.x[index * 9 + 12] = acadoWorkspace.state[3];
acadoVariables.x[index * 9 + 13] = acadoWorkspace.state[4];
acadoVariables.x[index * 9 + 14] = acadoWorkspace.state[5];
acadoVariables.x[index * 9 + 15] = acadoWorkspace.state[6];
acadoVariables.x[index * 9 + 16] = acadoWorkspace.state[7];
acadoVariables.x[index * 9 + 17] = acadoWorkspace.state[8];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 20; ++index)
{
acadoVariables.x[index * 9] = acadoVariables.x[index * 9 + 9];
acadoVariables.x[index * 9 + 1] = acadoVariables.x[index * 9 + 10];
acadoVariables.x[index * 9 + 2] = acadoVariables.x[index * 9 + 11];
acadoVariables.x[index * 9 + 3] = acadoVariables.x[index * 9 + 12];
acadoVariables.x[index * 9 + 4] = acadoVariables.x[index * 9 + 13];
acadoVariables.x[index * 9 + 5] = acadoVariables.x[index * 9 + 14];
acadoVariables.x[index * 9 + 6] = acadoVariables.x[index * 9 + 15];
acadoVariables.x[index * 9 + 7] = acadoVariables.x[index * 9 + 16];
acadoVariables.x[index * 9 + 8] = acadoVariables.x[index * 9 + 17];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[180] = xEnd[0];
acadoVariables.x[181] = xEnd[1];
acadoVariables.x[182] = xEnd[2];
acadoVariables.x[183] = xEnd[3];
acadoVariables.x[184] = xEnd[4];
acadoVariables.x[185] = xEnd[5];
acadoVariables.x[186] = xEnd[6];
acadoVariables.x[187] = xEnd[7];
acadoVariables.x[188] = xEnd[8];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[180];
acadoWorkspace.state[1] = acadoVariables.x[181];
acadoWorkspace.state[2] = acadoVariables.x[182];
acadoWorkspace.state[3] = acadoVariables.x[183];
acadoWorkspace.state[4] = acadoVariables.x[184];
acadoWorkspace.state[5] = acadoVariables.x[185];
acadoWorkspace.state[6] = acadoVariables.x[186];
acadoWorkspace.state[7] = acadoVariables.x[187];
acadoWorkspace.state[8] = acadoVariables.x[188];
if (uEnd != 0)
{
acadoWorkspace.state[117] = uEnd[0];
acadoWorkspace.state[118] = uEnd[1];
acadoWorkspace.state[119] = uEnd[2];
}
else
{
acadoWorkspace.state[117] = acadoVariables.u[57];
acadoWorkspace.state[118] = acadoVariables.u[58];
acadoWorkspace.state[119] = acadoVariables.u[59];
}
acadoWorkspace.state[120] = acadoVariables.od[60];
acadoWorkspace.state[121] = acadoVariables.od[61];
acadoWorkspace.state[122] = acadoVariables.od[62];

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[180] = acadoWorkspace.state[0];
acadoVariables.x[181] = acadoWorkspace.state[1];
acadoVariables.x[182] = acadoWorkspace.state[2];
acadoVariables.x[183] = acadoWorkspace.state[3];
acadoVariables.x[184] = acadoWorkspace.state[4];
acadoVariables.x[185] = acadoWorkspace.state[5];
acadoVariables.x[186] = acadoWorkspace.state[6];
acadoVariables.x[187] = acadoWorkspace.state[7];
acadoVariables.x[188] = acadoWorkspace.state[8];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 19; ++index)
{
acadoVariables.u[index * 3] = acadoVariables.u[index * 3 + 3];
acadoVariables.u[index * 3 + 1] = acadoVariables.u[index * 3 + 4];
acadoVariables.u[index * 3 + 2] = acadoVariables.u[index * 3 + 5];
}

if (uEnd != 0)
{
acadoVariables.u[57] = uEnd[0];
acadoVariables.u[58] = uEnd[1];
acadoVariables.u[59] = uEnd[2];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59];
kkt = fabs( kkt );
for (index = 0; index < 60; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 60; ++index)
{
prd = acadoWorkspace.y[index + 60];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 12 */
real_t tmpDy[ 12 ];

/** Row vector of size: 9 */
real_t tmpDyN[ 9 ];

for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 9];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 9 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 9 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 9 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 9 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[lRun1 * 9 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[lRun1 * 9 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[lRun1 * 9 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[lRun1 * 9 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.u[lRun1 * 3];
acadoWorkspace.objValueIn[10] = acadoVariables.u[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[11] = acadoVariables.u[lRun1 * 3 + 2];
acadoWorkspace.objValueIn[12] = acadoVariables.od[lRun1 * 3];
acadoWorkspace.objValueIn[13] = acadoVariables.od[lRun1 * 3 + 1];
acadoWorkspace.objValueIn[14] = acadoVariables.od[lRun1 * 3 + 2];

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
acadoWorkspace.objValueIn[0] = acadoVariables.x[180];
acadoWorkspace.objValueIn[1] = acadoVariables.x[181];
acadoWorkspace.objValueIn[2] = acadoVariables.x[182];
acadoWorkspace.objValueIn[3] = acadoVariables.x[183];
acadoWorkspace.objValueIn[4] = acadoVariables.x[184];
acadoWorkspace.objValueIn[5] = acadoVariables.x[185];
acadoWorkspace.objValueIn[6] = acadoVariables.x[186];
acadoWorkspace.objValueIn[7] = acadoVariables.x[187];
acadoWorkspace.objValueIn[8] = acadoVariables.x[188];
acadoWorkspace.objValueIn[9] = acadoVariables.od[60];
acadoWorkspace.objValueIn[10] = acadoVariables.od[61];
acadoWorkspace.objValueIn[11] = acadoVariables.od[62];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4] - acadoVariables.yN[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5] - acadoVariables.yN[5];
acadoWorkspace.DyN[6] = acadoWorkspace.objValueOut[6] - acadoVariables.yN[6];
acadoWorkspace.DyN[7] = acadoWorkspace.objValueOut[7] - acadoVariables.yN[7];
acadoWorkspace.DyN[8] = acadoWorkspace.objValueOut[8] - acadoVariables.yN[8];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 12]*acadoVariables.W[0];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 12 + 1]*acadoVariables.W[13];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 12 + 2]*acadoVariables.W[26];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 12 + 3]*acadoVariables.W[39];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 12 + 4]*acadoVariables.W[52];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 12 + 5]*acadoVariables.W[65];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 12 + 6]*acadoVariables.W[78];
tmpDy[7] = + acadoWorkspace.Dy[lRun1 * 12 + 7]*acadoVariables.W[91];
tmpDy[8] = + acadoWorkspace.Dy[lRun1 * 12 + 8]*acadoVariables.W[104];
tmpDy[9] = + acadoWorkspace.Dy[lRun1 * 12 + 9]*acadoVariables.W[117];
tmpDy[10] = + acadoWorkspace.Dy[lRun1 * 12 + 10]*acadoVariables.W[130];
tmpDy[11] = + acadoWorkspace.Dy[lRun1 * 12 + 11]*acadoVariables.W[143];
objVal += + acadoWorkspace.Dy[lRun1 * 12]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 12 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 12 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 12 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 12 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 12 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 12 + 6]*tmpDy[6] + acadoWorkspace.Dy[lRun1 * 12 + 7]*tmpDy[7] + acadoWorkspace.Dy[lRun1 * 12 + 8]*tmpDy[8] + acadoWorkspace.Dy[lRun1 * 12 + 9]*tmpDy[9] + acadoWorkspace.Dy[lRun1 * 12 + 10]*tmpDy[10] + acadoWorkspace.Dy[lRun1 * 12 + 11]*tmpDy[11];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[10];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[20];
tmpDyN[3] = + acadoWorkspace.DyN[3]*acadoVariables.WN[30];
tmpDyN[4] = + acadoWorkspace.DyN[4]*acadoVariables.WN[40];
tmpDyN[5] = + acadoWorkspace.DyN[5]*acadoVariables.WN[50];
tmpDyN[6] = + acadoWorkspace.DyN[6]*acadoVariables.WN[60];
tmpDyN[7] = + acadoWorkspace.DyN[7]*acadoVariables.WN[70];
tmpDyN[8] = + acadoWorkspace.DyN[8]*acadoVariables.WN[80];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3] + acadoWorkspace.DyN[4]*tmpDyN[4] + acadoWorkspace.DyN[5]*tmpDyN[5] + acadoWorkspace.DyN[6]*tmpDyN[6] + acadoWorkspace.DyN[7]*tmpDyN[7] + acadoWorkspace.DyN[8]*tmpDyN[8];

objVal *= 0.5;
return objVal;
}

