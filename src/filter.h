#pragma once


#include "stm32l5xx_hal.h"
#include "arm_math_types.h"

float const filterCoeff[] = 
{

	0.000000000000000005,
	0.000000000000000004,
	0.000000000009685996,
	0.000000000212262609,
	0.000000000765697702,
	-0.000000002303419429,
	-0.000000007009997735,
	0.000000010179368080,
	0.000000031623569900,
	-0.000000027122219418,
	-0.000000096703642286,
	0.000000049266116111,
	0.000000225465292939,
	-0.000000059843714683,
	-0.000000422016180067,
	0.000000032644289075,
	0.000000643970674292,
	0.000000047569970921,
	-0.000000783581794612,
	-0.000000148952192552,
	0.000000679028241541,
	0.000000157827940187,
	-0.000000171259130064,
	0.000000122713136299,
	-0.000000797403876562,
	-0.000000903464690441,
	0.000002069093269695,
	0.000002233794057213,
	-0.000003174991646254,
	-0.000003666771103380,
	0.000003268651461625,
	0.000003576892387610,
	-0.000001011213787577,
	0.000002568352888923,
	-0.000009525758757085,
	-0.000069151777423121,
	0.000048900306382538,
	0.000250656965925779,
	-0.000064726786746382,
	-0.000369487295692414,
	0.000017576082635125,
	0.000000171499407324,
	-0.000033638943596253,
	0.001374136681469215,
	0.000482721828794825,
	-0.003965456892366100,
	-0.001811286737516188,
	0.007183666287735983,
	0.004025106708285603,
	-0.009400614450197062,
	-0.005982429974477430,
	0.008363743998797624,
	0.004980911175609615,
	-0.002329521910186835,
	0.002788366141178850,
	-0.008555787839862420,
	-0.020702592159218606,
	0.021495753937767287,
	0.049574207750401410,
	-0.031642682269263200,
	-0.086038745336962208,
	0.034082558626377941,
	0.122619689733106607,
	-0.026322240042276731,
	-0.149892364374536069,
	0.009900273092156782,
	0.159994732620256874,
	0.009900273092156677,
	-0.149892364374534376,
	-0.026322240042276655,
	0.122619689733106468,
	0.034082558626377740,
	-0.086038745336962041,
	-0.031642682269263235,
	0.049574207750401320,
	0.021495753937767231,
	-0.020702592159218706,
	-0.008555787839862501,
	0.002788366141178866,
	-0.002329521910186762,
	0.004980911175609576,
	0.008363743998797612,
	-0.005982429974477413,
	-0.009400614450197013,
	0.004025106708285605,
	0.007183666287735996,
	-0.001811286737516173,
	-0.003965456892366096,
	0.000482721828794845,
	0.001374136681469230,
	-0.000033638943596264,
	0.000000171499407320,
	0.000017576082635144,
	-0.000369487295692411,
	-0.000064726786746374,
	0.000250656965925782,
	0.000048900306382553,
	-0.000069151777423123,
	-0.000009525758757085,
	0.000002568352888928,
	-0.000001011213787583,
	0.000003576892387608,
	0.000003268651461649,
	-0.000003666771103379,
	-0.000003174991646254,
	0.000002233794057209,
	0.000002069093269701,
	-0.000000903464690444,
	-0.000000797403876553,
	0.000000122713136295,
	-0.000000171259130062,
	0.000000157827940185,
	0.000000679028241545,
	-0.000000148952192556,
	-0.000000783581794621,
	0.000000047569970919,
	0.000000643970674300,
	0.000000032644289074,
	-0.000000422016180061,
	-0.000000059843714684,
	0.000000225465292943,
	0.000000049266116110,
	-0.000000096703642285,
	-0.000000027122219410,
	0.000000031623569904,
	0.000000010179368080,
	-0.000000007009997738,
	-0.000000002303419429,
	0.000000000765697701,
	0.000000000212262609,
	0.000000000009686014,
	0.000000000000000005,
	-0.000000000000000004

};
uint16_t FirCoe;

float const FilterOut[] =
{
	
	0.000000000000000000,
	-0.000011782700704376,
	-0.000038472858472843,
	0.000242163550440750,
	-0.000418633222834850,
	0.000107318962651679,
	0.000939065096853958,
	-0.002082196484230726,
	0.001794058409551059,
	0.001077166944515986,
	-0.005425662148755227,
	0.007359372825964543,
	-0.002697691737585132,
	-0.008388559191003323,
	0.018544966143281726,
	-0.016448543817429603,
	-0.003884331083799958,
	0.033855334391006714,
	-0.049898054321961295,
	0.025896723918085621,
	0.047701667983562859,
	-0.151616546140596553,
	0.243392371228800586,
	0.720000528505317039,
	0.243392371228800614,
	-0.151616546140596553,
	0.047701667983562845,
	0.025896723918085632,
	-0.049898054321961295,
	0.033855334391006714,
	-0.003884331083799958,
	-0.016448543817429617,
	0.018544966143281730,
	-0.008388559191003319,
	-0.002697691737585131,
	0.007359372825964545,
	-0.005425662148755231,
	0.001077166944515986,
	0.001794058409551061,
	-0.002082196484230726,
	0.000939065096853958,
	0.000107318962651679,
	-0.000418633222834850,
	0.000242163550440751,
	-0.000038472858472844,
	-0.000011782700704376,
	0.000000000000000000
		
};
uint16_t FirOutCoe;

uint16_t TimTxPack[] =
{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

struct 
{
	uint16_t AdcSize;
	float32_t FirstVal;
	float32_t SecondVal;
	uint32_t FirstPos;
	uint16_t SecondPos;
	uint8_t Delay;
	
} Glass;

struct 
{
	float32_t AbsMin1;
	float32_t AbsBreak1;
	float32_t AbsMin2;
	float32_t AbsBreak2;
	uint8_t PackLen;
	uint16_t Freq;
	char VesselName[32];
	
} Coe;




