#pragma once
#include "structDef.hpp"
#include "SP3File.hpp"
#include "Ionex1_0_File.hpp"
#include <limits>

#pragma comment(lib, "Fortran.lib")
#ifdef __cplusplus
extern "C"
{
	#endif
	// 注意：调用fortran函数，函数名要大写
	//extern void igrf13syn(int *isv,double *date,int *itype,double *alt,double *colat,double *elong,double *x,double *y,double *z,double *f);
	extern void IGRF13SYN(int *isv, double *date, int *itype, double *alt, double *colat, double *elong, double *x ,double *y, double *z, double *f);

	#ifdef __cplusplus
}
#endif

//  Copyright 2013, The National University of Defense Technology at ChangSha
namespace NUDTTK
{
	// 封装 GNSS 数据处理的常用基础修正函数
	class GNSSBasicCorrectFunc
	{
	public:
		GNSSBasicCorrectFunc(void);
	public:
		~GNSSBasicCorrectFunc(void);
	public:
		static void   correctSp3EarthRotation(double delay, SP3Datum& sp3Datum);
		static POS3D  correctLeoAntPCO_J2000(POS3D pcoAnt, POS3D posLeo, POS3D velLeo);
		static POS3D  correctLeoAntPCO_ECEF(UT1 t, POS3D pcoAnt, POS3D posLeo, POS3D velLeo);
		static double correctGPSAntPCO(int id_Block, POS3D gpsPCO, POS3D receiverPos, POS3D gpsPos, POS3D sunPos);
		static POS3D  correctLeoAntPCO_YawAttitudeModel(POS3D pcoAnt, POS3D posLeo, POS3D posSun);
		
		static double correctPhaseWindUp(int id_Block, POS3D receiverPos, POS3D unitXr, POS3D unitYr,POS3D gpsPos, POS3D sunPos, double prev = DBL_MAX);
		static double correctLeoAntPhaseWindUp(int id_Block, POS6D receiverPosVel, POS3D gpsPos, POS3D sunPos, double prev = DBL_MAX);
		static double correctStaAntPhaseWindUp(int id_Block, POS3D receiverPos, POS3D gpsPos, POS3D sunPos, double prev = DBL_MAX);
		
		static double correctPhaseWindUp_GYM95(POS3D unitXr, POS3D unitYr,POS3D vecLos, POS3D ex, POS3D ey, POS3D ez, double prev = DBL_MAX);	
		static double correctLeoAntPhaseWindUp_GYM95(POS6D receiverPosVel, POS3D vecLos, POS3D ex, POS3D ey, POS3D ez, double prev = DBL_MAX);
		static double correctStaAntPhaseWindUp_GYM95(POS3D receiverPos, POS3D vecLos, POS3D ex, POS3D ey, POS3D ez, double prev = DBL_MAX);
		static double correctLeoAntPhaseWindUp_new(POS6D receiverPosVel, POS3D gpsPos, POS3D sunPos, double prev);
		// 天线相位缠绕修正,不根据GNSS卫星类型进行区分，邵凯，2020.1.21
		static double correctLeoAntPhaseWindUp_GNSS(POS6D receiverPosVel, POS3D gpsPos, POS3D sunPos, double prev);
		static double correctLeoAntPhaseWindUp_GEO(POS6D receiverPosVel, POS3D gpsPos, POS3D sunPos, double prev);

		static bool   judgeGPSEarthShadowManeuver(POS3D sunPos, POS3D gpsPos, double &factor);
		static bool   judgeGPSEarthShadowManeuver_moon(POS3D sunPos, POS3D moonPos, POS3D gpsPos, double &factor);
		static bool   judgeGPSNoonManeuver(POS3D sunPos, POS6D gpsPosVel);

		static bool   ionIpp(POS3D leoPos, POS3D gpsPos, double H0, double &latitude_ipp, double &longitude_ipp, double &elevation_ipp);
		static double ionPrioriAlpha(double h, double h_IonLayer = 350000);
		static bool   ionexGridCorrect_IP(Ionex1_0_File &ionFile, GPST t, POS3D leoPos, POS3D gpsPos, double& value, double addLayerHeight = 80000, double frequence = GPS_FREQUENCE_L1);
		static bool   ionexGridCorrect_IP_alpha(Ionex1_0_File &ionFile, GPST t, POS3D leoPos, POS3D gpsPos, double alpha, double& value, double &d_mapFun_r_ip, double addLayerHeight = 80000, double frequence = GPS_FREQUENCE_L1);
		static bool   ionexGridCorrect_alpha(Ionex1_0_File &ionFile, GPST t, POS3D leoPos, POS3D gpsPos, double alpha, double& value, double frequence = GPS_FREQUENCE_L1);
		
		// 高阶(二、三阶)电离层修正，邵凯，2021.10.03
		static bool   ionexGridCorrect_IP_High_Order(Ionex1_0_File &ionFile, GPST t, POS3D leoPos, POS3D gpsPos, double& value, double addLayerHeight = 80000);
		static bool   ionexGridCorrect_Iono_High_Order(Ionex1_0_File &ionFile, GPST t, POS3D leoPos, POS3D gpsPos, double alpha, double& value, double addLayerHeight = 80000);
		
		static double graRelativityCorrect(POS3D satPos, POS3D staPos, double gamma = 1);
	};
}