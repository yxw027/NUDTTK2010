#pragma once
#include"constDef.hpp"
#include"structDef.hpp"
#include"jplEphFile.hpp"
#include"OceanTideFile.hpp"
#include"OceanPoleTideFile.hpp"
#include"Matrix.hpp"
#include"TimeCoordConvert.hpp"
#include"GravityModelFile.hpp"
#include"solarFluxFile.hpp"
#include"kpIndexFile.hpp"
#include"nrlmsise-00.hpp"
#include"solfsmyFile.hpp"
#include"DTCFILE.hpp"
#include"SatMacroFile.hpp"
#include"TimeAttitudeFile.hpp"
#include"CERESDataFile.hpp"
#include"graceACC1BFile.hpp"
#include"champACCFile.hpp"
#include"gracefoACT1BFile.hpp"
#include "TimeAccelerometerFile.hpp"
#pragma comment(lib, "Fortran.lib")
#ifdef __cplusplus
extern "C"
{
	#endif

	extern void HWM14(int *iyd, float *sec, float *alt, float *glat, float *glon, float *stl,float *f107a, float *f107, float *ap, float *w);
	extern void JB2008(double * AMJD, double SUN[], double SAT[], double * F10, double * F10B, double * S10, double * S10B, double * XM10, double * XM10B, double * Y10, double * Y10B, double * DSTDTC, double TEMP[], double * RHO);
	extern void DTM2020(float *day,float f[],float fbar[],float akp[], float *alti,float *hl,float *alat,float *xlon,float *tz,float *tinf,float *ro,float d[],float *wmm);

	#ifdef __cplusplus
}
#endif
//  Copyright 2012, The National University of Defense Technology at ChangSha
using namespace NUDTTK::SpaceborneGPSPod;

namespace NUDTTK
{
	namespace Geodyn
	{
		enum TYPE_SOLARPRESSURE_MODEL
		{
			TYPE_SOLARPRESSURE_1PARA      =  1, // 单参数Ball光压模型
			TYPE_SOLARPRESSURE_9PARA      =  2, // 九参数光压模型，Empirical CODE Orbit Model
			TYPE_SOLARPRESSURE_5PARA      =  3, // 五参数光压模型, 拟合精度较差暂不推荐直接使用
			TYPE_SOLARPRESSURE_9PARA_EX   =  4, // 扩展九参数光压模型，Extend Empirical CODE Orbit Model，2015-CODE新光压模型
			TYPE_SOLARPRESSURE_BGSM       =  5, // 六参数光压模型，BeiDou GEO SRP Model,3个常数项 + DS1 + YC1 + BS1,可用于北斗零偏姿态
			
			TYPE_SOLARPRESSURE_15PARA     =  6,  // 测试光压, 在9光压模型基础上扩展到2u
			TYPE_SOLARPRESSURE_21PARA     =  7,  // 测试光压, 在9光压模型基础上扩展到3u
			TYPE_SOLARPRESSURE_MACRO      =  8,  // 单参数Macro光压模型
			TYPE_SOLARPRESSURE_1PARA_AM   =  9   // 单参数ball变面积光压模型
		};

		// 2021.03.28，根据邵凯程序添加地球反照辐射模型
		enum TYPE_EARTHRADIATION_MODEL            // 地球反照辐射模型
		{
			TYPE_EARTHRADIATION_ANALYTICAL  =  1, // 分析型地球辐射模型，常值反照率 constant albedo
			TYPE_EARTHRADIATION_NUMERICAL   =  2, // 数值型地球辐射模型，常值反照率 constant albedo
			TYPE_EARTHRADIATION_LATITUDE    =  3, // 数值型地球辐射模型, 与纬度相关反照率  latitude dependent albedo
			TYPE_EARTHRADIATION_CERES       =  4  // 数值型地球辐射模型，利用CERES数据  satellite data
		}; 

		enum TYPE_SATELLITESHAPE_MODEL            // 卫星形状模型
		{
			TYPE_SATELLITESHAPE_BALL        =  1,     // 球模型, 面质比
			TYPE_SATELLITESHAPE_MACRO       =  2      // 宏模型
		};

		enum TYPE_ATMOSPHEREDRAG_MODEL           // 大气阻力密度模型
		{
			TYPE_ATMOSPHEREDRAG_J71_GEODYN  = 1,	// jacchia71-Geodyn
			TYPE_ATMOSPHEREDRAG_J71_GILL    = 2,	// jacchia71-Gill
			TYPE_ATMOSPHEREDRAG_J71_ROBERTS = 3,	// jacchia71-Roberts
			TYPE_ATMOSPHEREDRAG_NRLMSISE00  = 4,    // nrlmsise-00
			TYPE_ATMOSPHEREDRAG_JB2008      = 5,     // JB2008
			TYPE_ATMOSPHEREDRAG_DTM2020     = 6     // DTM2020
		};
		// 不同姿态模型数据
		enum TYPE_ATT_MODEL
		{
			TYPE_ATT_Body2J2000   = 1,            // 星固坐标系到惯性坐标系, GRACE,CHAMP
			TYPE_ATT_Body2ECEF    = 2             // 星固坐标系到地固坐标系，Swarm
		};
		enum TYPE_ATMOSPHEREACC_MODEL               // 大气阻力模型
		{
			TYPE_ATMOSPHEREACC_BALL       = 1,	   // 固定面积
			TYPE_ATMOSPHEREACC_BALL_AM    = 2,	   // 面值比
			TYPE_ATMOSPHEREACC_MACRO      = 3	   // 宏模型
		};

		enum TYPE_EMPIRICALFORCE_MODEL
		{
			TYPE_EMPIRICALFORCE_COSSIN   = 1,
            TYPE_EMPIRICALFORCE_SPLINE   = 2,       // 一阶线性样条, 2010/11/29
			TYPE_EMPIRICALFORCE_A0COSSIN = 3,       // 带常值项的正余弦, 2017/04/26
			TYPE_EMPIRICALFORCE_A0       = 4        // 分段常值
		};

		enum TYPE_OCEANTIDE_MODEL
		{
			TYPE_OCEANTIDE_CSR4    = 1,
			TYPE_OCEANTIDE_FES2004 = 2,
			TYPE_OCEANTIDE_FES2014 = 3
		};
		// 地球重力场类型
		enum TYPE_EARTHGRAVITY_MODEL
		{
			TYPE_EARTHGRAVITY_TIDEFREE = 1,
			TYPE_EARTHGRAVITY_ZEROTIDE = 2,
			TYPE_EARTHGRAVITY_MEANTIDE = 3
		};

		// 2021.03.28，添加加速度计数据，韦春博
		enum TYPE_SATELLITE_NAME
		{
			TYPE_SATELLITE_CHAMP = 1,
			TYPE_SATELLITE_GRACE = 2,
			TYPE_SATELLITE_GOCE  = 3,
			TYPE_SATELLITE_GRACEFO = 4
		};

		enum TYPE_CENTERBODY
		{
			TYPE_CENTERBODY_EARTH = 1,
			TYPE_CENTERBODY_SUN   = 2
		};

		struct SolarPressurePara
		{
			TDT    t0;   // 区间左端点
			TDT    t1;   // 区间右端点
			double Cr;   // 单参数模型的光压系数

			double D0;  // CODE光压模型系数--D方向常系数
			double DC1; // CODE光压模型系数--D方向余弦项cos(u)系数
			double DS1; // CODE光压模型系数--D方向正弦项sin(u)系数
			double Y0;  // CODE光压模型系数--Y方向常系数
			double YC1; // CODE光压模型系数--Y方向余弦项cos(u)系数	
			double YS1; // CODE光压模型系数--Y方向正弦项sin(u)系数	
			double B0;  // CODE光压模型系数--B方向常系数
			double BC1; // CODE光压模型系数--B方向余弦项cos(u)系数	
			double BS1; // CODE光压模型系数--B方向正弦项sin(u)系数

			// CODE's new solar radiation model (改进ECOM模型源于GLONASS卫星轨道模型的表示精度不足)
			double DC2;	// new CODE光压模型系数--D方向余弦项cos(2u)系数
			double DS2;	// new CODE光压模型系数--D方向正弦项sin(2u)系数
			double DC4;	// new CODE光压模型系数--D方向余弦项cos(4u)系数
			double DS4;	// new CODE光压模型系数--D方向正弦项sin(4u)系数

			// 针对TYPE_SOLARPRESSURE_15PARA、TYPE_SOLARPRESSURE_21PARA将光压模型进行适应性增加调整, 20161019
			double YC2;	// new CODE光压模型系数--Y方向余弦项cos(2u)系数
			double YS2;	// new CODE光压模型系数--Y方向正弦项sin(2u)系数
			double BC2;	// new CODE光压模型系数--B方向余弦项cos(2u)系数
			double BS2;	// new CODE光压模型系数--B方向正弦项sin(2u)系数
			double DC3;	// new CODE光压模型系数--D方向余弦项cos(3u)系数
			double DS3;	// new CODE光压模型系数--D方向正弦项sin(3u)系数
			double YC3;	// new CODE光压模型系数--Y方向余弦项cos(3u)系数
			double YS3;	// new CODE光压模型系数--Y方向正弦项sin(3u)系数
			double BC3;	// new CODE光压模型系数--B方向余弦项cos(3u)系数
			double BS3;	// new CODE光压模型系数--B方向正弦项sin(3u)系数

			// 为了保持与目前大多文献上的使用习惯统一, 将 DYX 坐标系替换为 DYB 坐标系, 2015/10/08 
			//double A_D0; // 九参数光压系数--D方向常系数
			//double A_DC; // 九参数光压系数--D方向余弦项系数
			//double A_DS; // 九参数光压系数--D方向正弦项系数
			//double A_Y0; // 九参数光压系数--Y方向常系数
			//double A_YC; // 九参数光压系数--Y方向余弦项系数	
			//double A_YS; // 九参数光压系数--Y方向正弦项系数	
			//double A_X0; // 九参数光压系数--X方向常系数
			//double A_XC; // 九参数光压系数--X方向余弦项系数	
			//double A_XS; // 九参数光压系数--X方向正弦项系数

			SolarPressurePara()
			{
				Cr  = 1.0;
				D0  = 0.0;
				DC1 = 0.0;
				DS1 = 0.0;
				DC2 = 0.0;
				DS2 = 0.0;
				DC3 = 0.0;
				DS3 = 0.0;
				DC4 = 0.0;
				DS4 = 0.0;
				Y0  = 0.0;
				YC1 = 0.0;
				YS1 = 0.0;
				YC2 = 0.0;
				YS2 = 0.0;
				YC3 = 0.0;
				YS3 = 0.0;
				B0  = 0.0;
				BC1 = 0.0;
				BS1 = 0.0;
				BC2 = 0.0;
				BS2 = 0.0;
				BC3 = 0.0;
				BS3 = 0.0;
			}
		};

		// 2021.03.28，根据邵凯程序添加地球反照辐射模型
		struct EarthIrradiancePara
		{
			TDT    t0;   // 区间左端点
			TDT    t1;   // 区间右端点
			double Ce;   // 单参数地球辐射模型的辐射系数

			EarthIrradiancePara()
			{
				Ce = 0.0;
			}
		};

		struct AtmosphereDragPara
		{
			TDT    t0;   // 区间左端点
			TDT    t1;   // 区间右端点
			double Cd;   // 单参数大气阻力模型的阻力系数
		};

		struct AtmosphereDensity_output
		{
			double density;
			POS3D  density_r;
			double Too;
			double h;
			double T[2];
			double Numdensity[9];
			AtmosphereDensity_output()
			{
				for(int i = 0; i < 9; i++)
					Numdensity[i] = 0.0;
				for(int i = 0; i < 2; i++)
					T[i]  = 0.0;
			}
		};

		struct AtmosphereDensity_input
		{
			// jacchia71_Geodyn、jacchia71_Gill、jacchia71_Roberts
			double   solarflux_day;
			double   solarflux_mean;
			double   kpIndex;
			double   jd1958;

			// nrlmsise-00
			ap_array ap; 

			// JB2008
			double AMJD;
			double SUN[2];
			double SAT[3];
			double F10;
			double F10B;
			double S10;
			double S10B; 
			double XM10; 
			double XM10B;
			double Y10;
			double Y10B;
			double DSTDTC;
			// DTM2020
			double kpIndex_mean;
		};

		// add
		struct AtmosphereDensity_output_add
		{
			double density;
			POS3D  density_r;
			double density_T;
			double DENS;
			double HELOG_Q2;
			double Too;
			double h;
		};

		// add,卫星表面系数

		// R方向常值经验力加速度 +,2019.12.26,邵凯
		struct RadialForcePara
		{
            TDT    t0;     
			TDT    t1;     
			double  R;   
			RadialForcePara()
			{
				R = 0.0;
			}
		};
		// T方向常值经验力加速度 +,2020.6.23,邵凯
		struct TangentialForcePara
		{
            TDT    t0;     
			TDT    t1;     
			double  T;   
			TangentialForcePara()
			{
				T = 0.0;
			}
		};
		// N方向常值经验力加速度 +,2020.6.23,邵凯
		struct NormalForcePara
		{
            TDT    t0;     
			TDT    t1;     
			double  N;   
			NormalForcePara()
			{
				N = 0.0;
			}
		};
		// 经验力
		struct EmpiricalForcePara
		{
            TDT    t0;     
			TDT    t1;     
			double cos_R;  // 经验加速度周期项, cos
			double sin_R;  // 经验加速度周期项, sin
			double cos_T; 
            double sin_T; 
			double cos_N; 
			double sin_N; 
			double a0_R;   // 经验加速度常数项
			double a1_R;   // 经验加速度线性项
			double a0_T; 
			double a1_T; 
			double a0_N; 
			double a1_N;
		};

		// 机动力
		struct ManeuverForcePara
		{
            TDT    t0;     
			TDT    t1;     
			double a0_R;   // 机动力常数项
			double a0_T; 
			double a0_N;
			int    id;	   // 机动力弧段编号
			ManeuverForcePara()
			{
				a0_R = 0.0;
				a0_T = 0.0;
				a0_N = 0.0;
				id   = 1;
			}
		};

		// 2021.03.28，韦春博，加速度计数据标校参数
		// 尺度因子
		struct Accelerometer_Scale       
		{
			TDT t0;
			TDT t1;
			POS3D scale;
		};
		// 偏差因子
		struct Accelerometer_Bias
		{
			TDT t0;
			TDT t1;
			double bias;
			// 初始化
			Accelerometer_Bias()
			{
				bias = 0.0;
			}
		};
		// 偏差参数线性漂移项
		struct Acclerometer_Drift
		{
			TDT t0;
			TDT t1;
			POS3D drift;
		};

		// 卫星动力学定轨参数
		struct SatdynBasicDatum
		{
			TDT                          T0;                       // 定轨初始时间
			POS6D                        X0;                       // 初始时间的轨道位置速度(J2000惯性系)
            POS6D                        X0_ECEF;                  // 初始轨道位置速度(地固系)
			double                       ArcLength;                // 轨道弧段长度
			double                       Am_Cr;                    // 光照面卫星面质比
			double                       Am_Cd;                    // 迎风面卫星面质比
			double                       Am_Ce;                    // 地球反照面卫星面质比
			bool                         bOn_TwoBody;              // 20180628, 谷德峰修改, 使得地球中心引力可选
			TYPE_EARTHGRAVITY_MODEL      earthGravityType;         // 地球重力场模型 +
			bool                         bOn_NonSpherical;         // 非球形摄动
			bool                         bOn_NonSpherical_Moon;    // 2018/09/28, 谷德峰添加, 月球非球形摄动
			bool                         bOn_SolidTide;            // 固体潮开关变量
			bool                         bOn_SolidPoleTide;        // 固体极潮开关变量 +
			bool                         bOn_OceanTide;            // 海潮开关变量
			bool                         bOn_OceanPoleTide;        // 海潮极潮开关变量 +
			TYPE_OCEANTIDE_MODEL         oceanTideType;            // 海潮类型
			bool                         bOn_ThirdBodyAcc;         // 第三体摄动
			bool                         bOn_SolarPressureAcc;     // 太阳光压摄动开关变量
			bool                         bOn_AtmosphereDragAcc;    // 大气阻力摄动开关变量
			bool                         bOn_RelativityAcc;        // 相对论摄动
			bool                         bOn_EmpiricalForceAcc;    // 经验力摄动
			bool                         bOn_EmpiricalForceAcc_R;  // 经验力摄动- R 方向
			bool                         bOn_EmpiricalForceAcc_T;  // 经验力摄动- T 方向
			bool                         bOn_EmpiricalForceAcc_N;  // 经验力摄动- N 方向
			bool                         bOn_ManeuverForceAcc;     // 机动力摄动
			bool                         bOn_Used_delta_u;
			TYPE_SOLARPRESSURE_MODEL     solarPressureType;        // 太阳光压模型类型
			vector<SolarPressurePara>    solarPressureParaList;    // 太阳光压参数列表  
			TYPE_ATMOSPHEREDRAG_MODEL    atmosphereDragType;       // 大气阻力密度模型类型
			vector<AtmosphereDragPara>   atmosphereDragParaList;   // 大气阻力参数列表
			double                       constSolarFlux;
			double                       constKpIndex;			
			double                       constApIndex;
			double                       constDSTDTC;
			double                       constsolfsmy;
			TYPE_EMPIRICALFORCE_MODEL    empiricalForceType;       // 经验力模型类型
			vector<EmpiricalForcePara>   empiricalForceParaList;   // 经验力参数列表
			vector<ManeuverForcePara>    maneuverForceParaList;    // 机动力参数列表	

			bool                         bOnEst_Maneuver;          // 测试代码
			int                          Sat_n;                    // 侧面数
			double                       Sat_m;                    // 卫星质量
			bool                         bOn_Atmosphere_Wind;      // 大气风开关 
			TYPE_ATMOSPHEREACC_MODEL     atmosphereAccType;        // 大气阻力模型
			TYPE_ATT_MODEL               attModelType; // 姿态数据类型
			// 行星引力开关
			bool                         bOn_ThirdBodyAcc_Earth;   // 2018/09/14, 谷德峰添加, 中心天体可选择, 便于分析日心轨道
			bool                         bOn_ThirdBodyAcc_Moon;    // 月球
			bool                         bOn_ThirdBodyAcc_Sun;     // 太阳
			bool                         bOn_ThirdBodyAcc_Venus;   // 金星
			bool                         bOn_ThirdBodyAcc_Mars;    // 火星
			bool                         bOn_ThirdBodyAcc_Jupiter; // 木星
			bool                         bOn_ThirdBodyAcc_Saturn;  // 土星
			bool                         bOn_ThirdBodyAcc_Mercury; // 水星
			bool                         bOn_ThirdBodyAcc_Uranus;  // 天王星
			bool                         bOn_ThirdBodyAcc_Neptune; // 海王星
			bool                         bOn_ThirdBodyAcc_Pluto;   // 冥王星
			
			TYPE_CENTERBODY              centerBodyType;
			double                       gm_CenterBody;
			// 加速度计数据相关，2021.03.28，韦春博
			bool                         bOn_NonconservativeForce;   // 加速度计数据使用开关
			bool                         bOn_NonconserForce_ScaleX;  // 尺度参数，一旦关闭，其值固定为推荐值
			bool                         bOn_NonconserForce_ScaleY;
			bool                         bOn_NonconserForce_ScaleZ;
			bool                         bOn_NonconserForce_c1X;     // 时间漂移项c1，一旦关闭，其值一般设置为0
			bool                         bOn_NonconserForce_c1Y;
			bool                         bOn_NonconserForce_c1Z;
			bool                         bOn_NonconserForce_c2X;     // 时间漂移项c2，一旦关闭，其值一般设置为0
			bool                         bOn_NonconserForce_c2Y;
			bool                         bOn_NonconserForce_c2Z;
			// 加速度计数据的标校参数列表，2021.03.28，韦春博
			TYPE_SATELLITE_NAME          satelliteName_NONForce;
			vector<Accelerometer_Scale>  Accelerometer_ScaleParaList; // 尺度参数列表
			vector<Accelerometer_Bias>   Accelerometer_xBiasParaList;  // x方向偏差参数列表
			vector<Accelerometer_Bias>   Accelerometer_yBiasParaList;  // y方向偏差参数列表
			vector<Accelerometer_Bias>   Accelerometer_zBiasParaList;  // z方向偏差参数列表
			vector<Acclerometer_Drift>   Accelerometer_c1DriftParaList; // 线性漂移项
			vector<Acclerometer_Drift>   Accelerometer_c2DriftParaList; // 二次漂移项
			// 2021.03.08，根据邵凯程序添加地球反照辐射模型
			TYPE_EARTHRADIATION_MODEL    earthRadiationType;       // 地球辐射模型类型
			TYPE_SATELLITESHAPE_MODEL    satelliteShapeType;       // 卫星表面模型
			vector<EarthIrradiancePara>  earthIrradianceParaList;  // 地球辐射参数列表
			bool                         bOn_EarthIrradianceAcc;   // 地球辐射摄动开关变量
			bool                         bOn_EarthIrradianceAcc_noest;   // 地球辐射摄动开关变量,不估计参数
			bool                         bOn_RadialForceAcc;       // R方向常值经验力摄动 +
			vector<RadialForcePara>      radialForceParaList;      // R方向经验力参数列表 +
			bool                         bOn_TangentialForceAcc;       // T方向常值经验力摄动 +
			vector<TangentialForcePara>  tangentialForceParaList;      // T方向经验力参数列表 +
			bool                         bOn_NormalForceAcc;       // N方向常值经验力摄动 +
			vector<NormalForcePara>      normalForceParaList;      // N方向经验力参数列表 +
			bool                         bOn_EstInitialState;      // 是否估计初始状态矢量，默认为开
			SatdynBasicDatum()
			{
				ArcLength = 0;
				Am_Cr = 1.0;
				Am_Cd = 1.0;
				Am_Ce = 1.0;
				bOn_TwoBody               = true; // 20180628, 谷德峰修改, 使得地球中心引力可选
				bOn_NonSpherical          = true;  
				earthGravityType          = TYPE_EARTHGRAVITY_ZEROTIDE;
				bOn_NonSpherical_Moon     = false;
				bOn_SolidTide             = true;       
				bOn_SolidPoleTide         = true;   
				bOn_OceanTide             = true;
				bOn_OceanPoleTide         = false;  
				oceanTideType             = TYPE_OCEANTIDE_CSR4;
				bOn_ThirdBodyAcc          = true;        
				bOn_SolarPressureAcc      = true; 
				bOn_AtmosphereDragAcc     = false;
				bOn_RelativityAcc         = true;
				bOn_EmpiricalForceAcc     = false;
				bOn_ManeuverForceAcc      = false;
				bOn_Used_delta_u          = false;
				solarPressureType         = TYPE_SOLARPRESSURE_9PARA;
				atmosphereDragType        = TYPE_ATMOSPHEREDRAG_J71_GEODYN;
				empiricalForceType        = TYPE_EMPIRICALFORCE_COSSIN;
				constSolarFlux            = 70.0;
				constKpIndex              = 3.0;
				constApIndex              = 3.0;
				constDSTDTC               = 3.0;
				constsolfsmy              = 3.0;
				maneuverForceParaList.clear();
				bOn_EmpiricalForceAcc_R   = false;
			    bOn_EmpiricalForceAcc_T   = true;
			    bOn_EmpiricalForceAcc_N   = true;

				bOnEst_Maneuver           = false; // 测试代码
				Sat_n                     = 7;
				Sat_m                     = 8500.0;
				bOn_Atmosphere_Wind       = false;   
				atmosphereAccType         = TYPE_ATMOSPHEREACC_BALL;

				// 行星引力开关
				bOn_ThirdBodyAcc_Earth    = false; // 2018/09/14, 谷德峰添加, 中心天体可选择, 便于分析日心轨道
				bOn_ThirdBodyAcc_Moon     = true;  // 月球
				bOn_ThirdBodyAcc_Sun      = true;  // 太阳
				bOn_ThirdBodyAcc_Venus    = true;  // 金星
				bOn_ThirdBodyAcc_Mars     = true;  // 火星
				bOn_ThirdBodyAcc_Jupiter  = true;  // 木星
				bOn_ThirdBodyAcc_Saturn   = true;  // 土星
				bOn_ThirdBodyAcc_Mercury  = true;  // 水星
				bOn_ThirdBodyAcc_Uranus   = true;  // 天王星
				bOn_ThirdBodyAcc_Neptune  = true;  // 海王星
				bOn_ThirdBodyAcc_Pluto    = true;  // 冥王星

				attModelType  =  TYPE_ATT_Body2J2000;
				centerBodyType = TYPE_CENTERBODY_EARTH; // 2018/09/14, 谷德峰添加, 中心天体可选择, 便于分析日心轨道
				gm_CenterBody  = GM_EARTH;
				// 2021.03.28，加速度计数据处理，韦春博
				satelliteName_NONForce    = TYPE_SATELLITE_GRACE;
				bOn_NonconservativeForce  = false;
				bOn_NonconserForce_ScaleX = false;     // 尺度参数开关，默认为关
				bOn_NonconserForce_ScaleY = false;
				bOn_NonconserForce_ScaleZ = false;
				bOn_NonconserForce_c1X = false;        // 时间漂移项开关c1，默认为关
				bOn_NonconserForce_c1Y = false;
				bOn_NonconserForce_c1Z = false;
				bOn_NonconserForce_c2X = false;        // 时间漂移项开关c2，默认为关
				bOn_NonconserForce_c2Y = false;
				bOn_NonconserForce_c2Z = false;
				// 2021.03.28，根据邵凯程序添加地球反照辐射模型
				bOn_EarthIrradianceAcc       = false;                       // 地球反照辐射模型，默认为关
				bOn_EarthIrradianceAcc_noest = false;
				earthRadiationType        = TYPE_EARTHRADIATION_ANALYTICAL; // 默认为数值型CERES地球辐射模型
				satelliteShapeType        = TYPE_SATELLITESHAPE_BALL;
				// 2021.03.28，根据邵凯程序添加R方向常值经验力
				bOn_RadialForceAcc        = false; // +
				bOn_TangentialForceAcc    = false;
				bOn_NormalForceAcc        = false;
				bOn_EstInitialState       = true;
				
			};
            // 获得局部待估参数个数
			int getSubEstParaCount();
			int getSubEstSolarPressureParaBegin();
			int getSubEstEarthIrradianceParaBegin();    // 2021.03.28,+
			int getSubEstAtmosphereDragParaBegin();
            int getSubEstEmpiricalForceParaBegin();
			int getSubEstManeuverForceParaBegin();
			int getSubEstRadialForceParaBegin();        // 2021.03.28,+
			int getSubEstTangentialForceParaBegin(); // +
			int getSubEstNormalForceParaBegin(); // +
			// 2021.03.28，加速度计标校参数
			int getSubEstScaleParaBegin();
			int getSubEstxBiasParaBegin();
			int getSubEstyBiasParaBegin();
			int getSubEstzBiasParaBegin();
			int getSubEstc1ParaBegin();
			int getSubEstc2ParaBegin();
			// 获得所有待估参数的个数
			int getSolarPressureParaCount();
			int getAllEstParaCount();
			int getIndexSolarPressureParaList(TDT t);
			int getIndexEarthIrradianceParaList(TDT t);    // 2021.03.28,+
			int getIndexAtmosphereDragParaList(TDT t);
			int getIndexRadialForceParaList(TDT t);        // 2021.03.28,+
			int getIndexTangentialForceParaList(TDT t); // + tangentialForceParaList
			int getIndexNormalForceParaList(TDT t); // + normalForceParaList
			int getIndexEmpiricalForceParaList(TDT t);
			int getIndexManeuverForceParaList(TDT t);
			// 2021.03.28，加速度计标校参数，韦春博
			int getIndexScaleParaList(TDT t);
			int getIndexxBiasParaList(TDT t);
			int getIndexyBiasParaList(TDT t);
			int getIndexzBiasParaList(TDT t);
			int getIndexc1ParaList(TDT t);
			int getIndexc2ParaList(TDT t);
			void init(double period_SolarPressure = 3600 * 24.0, 
				      double period_AtmosphereDrag = 3600 * 24.0, 
					  double period_EmpiricalForce = 3600 * 1.5,
					  double period_RadialEmpForce = 3600 * 24.0,
					  double period_TangentialEmpForce = 3600 * 24.0,
					  double period_NormalEmpForce = 3600 * 24.0,
					  double period_EarthIrradiance = 3600 * 24.0);
			bool addManeuverForcePara(GPST t0, GPST t1, double a0_R = 0.0, double a0_T = 0.0, double a0_N = 0.0, int id = 1);
			// 2021.03.28,加速度计标校参数初始化，韦春博
			void initAccelerometerPara(TDT t0, TDT t1, POS3D Scale, POS3D Bias, double period_Scale = 3600 * 24.0, 
				                                                                double period_xBias = 3600 * 1.5,
																				double period_yBias = 3600 * 3.0, 
																				double period_zBias = 3600 * 3.0,
																				double period_Drift = 3600 * 24.0);
		};

		// 摄动加速度
		struct AccPerturb
		{
			DayTime t;                    // 时间, 2010/11/03
			POS3D   accTotal;             // 综合摄动加速度
			POS3D   accNonSpherical;      // 非球形摄动, 包含固体潮汐, 海潮
			POS3D   accNonSpherical_Moon; // 2018/09/28, 谷德峰添加, 月球非球形摄动
			POS3D   accEarth;             // 2018/09/14, 谷德峰添加, 中心天体可选择, 便于分析日心轨道
			POS3D   accSun;               // 太阳引力
			POS3D   accMoon;              // 月球引力
			POS3D   accVenus;             // 金星引力
			POS3D   accMars;              // 火星引力
			POS3D   accJupiter;           // 木星引力
			POS3D   accSaturn;            // 土星引力
			POS3D   accMercury;           // 水星引力
			POS3D   accUranus;            // 天王星引力
			POS3D   accNeptune;           // 海王星引力
			POS3D   accPluto;             // 冥王星引力
			POS3D   accSolarPressure;     // 太阳光压
			POS3D   accAtmosphereDrag;    // 大气阻力
			POS3D   accRadial;            // R经验加速度摄动 +
			POS3D   accTangential;        // T经验加速度摄动 +
			POS3D   accNormal;            // T经验加速度摄动 +
			POS3D   accEmpirical;         // 经验加速度摄动
			POS3D   accRelativity;        // 相对论
			POS3D   accManeuver;          // 机动力
			double  factor_SolarPressure; // 阴影因子
			double  A_Cr;                 // 光压面积
			double  A_Cd;                 // 大气面积
			double  u;                    // 升交点角距（升交点与卫星之间的夹角）
            // 2021.03.28,加速度计数据,韦春博
			POS3D   accFileData;          // 加速度计数据
			POS3D   accEarthIrr;          // 地球辐射
			AccPerturb()
			{
				factor_SolarPressure = 0.0;
				A_Cr = 0.0;
				A_Cd = 0.0;
				u    = 0.0;
			}
		};

		class SatdynBasic
		{
		public:
			SatdynBasic(void);
		public:
			~SatdynBasic(void);
		public:
			// 柱形地影计算
			void earthColumnShadow(POS3D sunPos, POS3D leoPos, double &factor);
			// 锥形地影、月影计算
			void earthMoonConicalShadow(POS3D sunPos, POS3D moonPos, POS3D leoPos, double &factor);
			void shadowScalingFactor(POS3D sunPos, POS3D moonPos, POS3D leoPos, double &factor);
			// 固体潮重力场引力位改进
			void solidTideCorrect_EarthGravity(double jY2000_TDT, UT1 ut1, POS3D moonPos_ECEF,POS3D sunPos_ECEF, double xp, double yp, double delta_Cnm[5][5], double delta_Snm[5][5]);
			//
			void solidTideCorrect_EarthGravity_Anelastic(double jY2000_TDT, UT1 ut1, POS3D moonPos_ECEF,POS3D sunPos_ECEF, double xp, double yp, double delta_Cnm[5][5], double delta_Snm[5][5]);
			// 海潮重力场引力位改进
			void oceanTideCorrect_EarthGravity(double jY2000_TDT, UT1 ut1, double delta_Cnm[7][7], double delta_Snm[7][7]);
			void oceanTideCorrect_EarthGravity_FES2004(double jY2000_TDT, UT1 ut1, double delta_Cnm[51][51], double delta_Snm[51][51]);  //FES2004海潮模型改进，sk
			void oceanTideCorrect_EarthGravity_FES2014(double jY2000_TDT, UT1 ut1, double delta_Cnm[121][121], double delta_Snm[121][121]); 
			// 海洋极潮重力场引力位改进
			void oceanPoleTideCorrect_EarthGravity(double m1, double m2, double delta_Cnm[121][121], double delta_Snm[121][121]);  
			// 第三天体引力摄动加速度
			void accThirdCelestialBody(POS3D leoPos_j2000, POS3D bodyPos_j2000, double gmBody, POS3D &acc, Matrix &matAccPartial_r);
			// 单参数ball太阳光压摄动
			void accSolarPressure_1Parameter(double Cr, double Am, POS3D sunPos_j2000, POS3D moonPos_j2000, POS3D leoPos_j2000, POS3D &acc, Matrix &matAccPartial_r, Matrix &matAccPartial_Cr, double &factor);
			void accSolarPressure_1Parameter_Am(double Cr, double Sat_m, POS3D sunPos_j2000, POS3D moonPos_j2000, POS6D leoPosVel_j2000, Matrix matATT, int validATT, POS3D &acc,Matrix &matAccPartial_r, Matrix &matAccPartial_Cr, double &factor, double &A_cr);
			// 单参数Macro太阳光压模型
			void accSolarPressure_Macro(double Cr, double Sat_m, POS3D sunPos_j2000, POS3D moonPos_j2000, POS6D leoPosVel_j2000, Matrix matATT, int validATT,
				                       POS3D &acc,Matrix &matAccPartial_r, Matrix &matAccPartial_vel, Matrix &matAccPartial_Cr, double &factor, double &A_cr);
			// 九参数太阳光压摄动
			void accSolarPressure_9Parameter(double A_D0, double A_DC, double A_DS, double A_Y0, double A_YC, double A_YS, double A_X0, double A_XC, double A_XS, 
				                             POS3D sunPos_j2000, POS3D moonPos_j2000, POS3D leoPos_j2000, POS3D &acc, Matrix &matAccPartial_r, Matrix &matAccPartial_A);
			// 九参数太阳光压摄动(u)
			void accSolarPressure_9Parameter_u(double A_D0, double A_DC, double A_DS, double A_Y0, double A_YC, double A_YS, double A_X0, double A_XC, double A_XS, double u, Matrix matuPartial_r,
				                             POS3D sunPos_j2000, POS3D moonPos_j2000, POS3D leoPos_j2000, POS3D &acc, Matrix &matAccPartial_r, Matrix &matAccPartial_A);
			// CODE 太阳经验光压模型
			void accSolarPressure_CODE(SolarPressurePara SRPPs, double u, Matrix matuPartial_r, 
				                       POS3D sunPos_j2000, POS3D moonPos_j2000, POS6D satPosVel_j2000, POS3D &acc, Matrix &matAccPartial_r, Matrix &matAccPartial_A, TYPE_SOLARPRESSURE_MODEL solarPressureType = TYPE_SOLARPRESSURE_9PARA);
            // 2021.03.18，根据邵凯程序添加地球反照辐射模型
			void accEarthIrradiance(double Ce, double Sat_m, int doy, POS3D sunPos_ECEF, POS3D sunPos_j2000, POS6D leoPosVel_j2000, Matrix matJ2000_ECF, Matrix matATT, int validATT,
				                   POS3D &acc, Matrix &matAccPartial_r, Matrix &matAccPartial_vel, Matrix &matAccPartial_Ce, 
								   TYPE_EARTHRADIATION_MODEL earthRadiationType = TYPE_EARTHRADIATION_ANALYTICAL, TYPE_SATELLITESHAPE_MODEL satelliteShapeType = TYPE_SATELLITESHAPE_BALL);
			// T方向常值经验加速度摄动
			void accTangentialForce(POS6D leoPosVel_j2000, TangentialForcePara accPara, POS3D &acc, 
				               Matrix &matAccPartial_para, Matrix &matAccPartial_r, Matrix &matAccPartial_vel);
			// N方向常值经验加速度摄动
			void accNormalForce(POS6D leoPosVel_j2000, NormalForcePara accPara, POS3D &acc, 
				               Matrix &matAccPartial_para, Matrix &matAccPartial_r, Matrix &matAccPartial_vel);
			// 地球返照
			void accEarthIrradiance_acc(double A_m, double C_ball, int doy, 
			                            POS3D sunPos_ECEF, POS3D sunPos_j2000,
			                            POS6D leoPosVel_j2000, Matrix matJ2000_ECF,
			                            POS3D &acc, 
										TYPE_EARTHRADIATION_MODEL earthRadiationType = TYPE_EARTHRADIATION_ANALYTICAL, TYPE_SATELLITESHAPE_MODEL satelliteShapeType = TYPE_SATELLITESHAPE_BALL);
			// 大气密度
			AtmosphereDensity_output atmosphereDensity_jacchia71_Geodyn(POS3D sunPos_ECEF, POS3D leoPos_ECEF, AtmosphereDensity_input input, double& density, POS3D& density_r, double re = EARTH_R, double f = EARTH_F);
			AtmosphereDensity_output atmosphereDensity_jacchia71_Gill(POS3D sunPos_ECEF, POS3D leoPos_ECEF, AtmosphereDensity_input input, double& density, POS3D& density_r, double re = EARTH_R, double f = EARTH_F);
			AtmosphereDensity_output atmosphereDensity_nrlmsise00(POS3D sunPos_ECEF, POS3D leoPos_ECEF, AtmosphereDensity_input input, double& density, POS3D& density_r, double re = EARTH_R, double f = EARTH_F);
			AtmosphereDensity_output atmosphereDensity_jacchia71_Roberts(POS3D sunPos_ECEF, POS3D leoPos_ECEF, AtmosphereDensity_input input, double& density, POS3D& density_r, double re = EARTH_R, double f = EARTH_F);
			AtmosphereDensity_output atmosphereDensity_JB2008(POS3D sunPos_ECEF, POS3D leoPos_ECEF, AtmosphereDensity_input input, double& density, POS3D& density_r, double re = EARTH_R, double f = EARTH_F);
			AtmosphereDensity_output atmosphereDensity_dtm2020(POS3D sunPos_ECEF, POS3D leoPos_ECEF, AtmosphereDensity_input input, double& density, POS3D& density_r, double re = EARTH_R, double f = EARTH_F);
			// 大气阻力摄动加速度
			void accAtmosphereDrag(double Cd, double Am, POS3D sunPos_ECEF, POS6D leoPosVel_j2000, AtmosphereDensity_input input, Matrix matJ2000_ECF, POS3D wind,
			                       POS3D& acc, Matrix &matAccPartial_r, Matrix &matAccPartial_vel, Matrix &matAccPartial_cd, double re = EARTH_R, double f = EARTH_F, TYPE_ATMOSPHEREDRAG_MODEL atmosphereDragType = TYPE_ATMOSPHEREDRAG_J71_GEODYN);
			void accAtmosphereDrag_Am(double Cd, double Sat_m, POS3D sunPos_ECEF, POS3D sunPos_j2000, POS6D leoPosVel_j2000, AtmosphereDensity_input input, Matrix matJ2000_ECF, POS3D wind, Matrix matATT, int validATT,
			                       POS3D& acc, Matrix &matAccPartial_r, Matrix &matAccPartial_vel, Matrix &matAccPartial_cd, double re = EARTH_R, double f = EARTH_F, TYPE_ATMOSPHEREDRAG_MODEL atmosphereDragType = TYPE_ATMOSPHEREDRAG_J71_GEODYN);
			void accAtmosphereDrag_Macro(double Cd, double Sat_m, POS3D sunPos_ECEF, POS3D sunPos_j2000, POS6D leoPosVel_j2000, AtmosphereDensity_input input, Matrix matJ2000_ECF, POS3D wind, Matrix matATT, int validATT,
			                       POS3D& acc, Matrix &matAccPartial_r, Matrix &matAccPartial_vel, Matrix &matAccPartial_cd, double &am, double re = EARTH_R, double f = EARTH_F, TYPE_ATMOSPHEREDRAG_MODEL atmosphereDragType = TYPE_ATMOSPHEREDRAG_J71_GEODYN);
			// 相对论摄动
			//void accRelativity(POS6D leoPosVel_j2000, POS3D &acc, Matrix &matAccPartial_r, Matrix &matAccPartial_v, double gm_CenterBody = GM_EARTH);
			void accRelativity(POS6D leoPosVel_j2000, POS3D sunPos_j2000, POS3D sunVel_j2000, POS3D &acc, Matrix &matAccPartial_r, Matrix &matAccPartial_v, double gm_CenterBody = GM_EARTH);
			// 周期性经验加速度摄动
			void accEmpiricalForce_CosSin(POS6D leoPosVel_j2000, EmpiricalForcePara accPara, double u, Matrix matFaiPartial_r, 
				                          POS3D &acc, Matrix &matAccPartial_para, Matrix &matAccPartial_r, Matrix &matAccPartial_vel);
			// 周期性经验加速度摄动
			void accEmpiricalForce_A0CosSin(POS6D leoPosVel_j2000, EmpiricalForcePara accPara, double u, Matrix matFaiPartial_r, 
				                          POS3D &acc, Matrix &matAccPartial_para, Matrix &matAccPartial_r, Matrix &matAccPartial_vel);
    		// 线性样条经验加速度摄动
			void accEmpiricalForce_Spline(POS6D leoPosVel_j2000, EmpiricalForcePara accPara, double t, Matrix matFaiPartial_r, 
				                          POS3D &acc, Matrix &matAccPartial_para, Matrix &matAccPartial_r, Matrix &matAccPartial_vel);
			// 分段常值经验加速度摄动
			void accEmpiricalForce_A0(POS6D leoPosVel_j2000, EmpiricalForcePara accPara, POS3D &acc, Matrix &matAccPartial_para, Matrix &matAccPartial_r, Matrix &matAccPartial_vel);
            // 2021.03.28，根据邵凯程序添加R方向常值经验力
			void accRadialForce(POS6D leoPosVel_j2000, RadialForcePara accPara, POS3D &acc, 
				               Matrix &matAccPartial_para, Matrix &matAccPartial_r, Matrix &matAccPartial_vel);
			// 机动力加速度摄动
			void accManeuverForce(POS6D leoPosVel_j2000, ManeuverForcePara accPara, double t, Matrix matFaiPartial_r, 
				                  POS3D &acc, Matrix &matAccPartial_para, Matrix &matAccPartial_r, Matrix &matAccPartial_vel);
            // 2021.03.28,韦春添加,获取加速度计文件中的非保守力摄动加速度
			void accNonconservativeForce_GRACE(POS6D leoPosVel_j2000, POS3D scale, POS3D bias, POS3D c1_Drift, POS3D c2_Drift, GPST t_GPS, Matrix matATT, int validATT,
				                          POS3D &acc, Matrix &matAccPartial_para, Matrix &matAccPartial_r, Matrix &matAccPartial_vel);
			void accNonconservativeForce_CHAMP(POS6D leoPosVel_j2000, POS3D scale, POS3D bias, POS3D c1_Drift, POS3D c2_Drift, GPST t_GPS, Matrix matATT, int validATT,
				                          POS3D &acc, Matrix &matAccPartial_para, Matrix &matAccPartial_r, Matrix &matAccPartial_vel);
			// 2020.4.9，韦春博添加，用于GRACE-FO卫星加速度计数据定轨
			void accNonconservativeForce_GRACEFO(POS6D leoPosVel_j2000, POS3D scale, POS3D bias, POS3D c1_Drift, POS3D c2_Drift, GPST t_GPS, Matrix matATT, int validATT,
				                          POS3D &acc, Matrix &matAccPartial_para, Matrix &matAccPartial_r, Matrix &matAccPartial_vel);
			// 轨道摄动加速度的计算
			void getTwoBodyAcc(TDT t, POS6D leoPosVel_j2000, POS3D &accTwoBody, Matrix &matAccPartial_r, double gm_CenterBody = GM_EARTH);
			bool getPerturbAcc(SatdynBasicDatum dynamicDatum, TDT t, POS6D leoPosVel_j2000, AccPerturb &acc, Matrix &matAccPartial);
			// 动力学轨道积分方法
			bool AdamsCowell(SatdynBasicDatum dynamicDatum, TDT t_End, vector<TimePosVel> &orbitlist, vector<Matrix> &matRtPartiallist, double h, int q);
			bool AdamsCowell_RK(SatdynBasicDatum dynamicDatum, TDT t_End, vector<TimePosVel> &orbitlist, vector<Matrix> &matRtPartiallist, double h, int q);
			// AdamsCowell 的右函数
	        Matrix AdamsCowell_RFunc(TDT t, Matrix matY, Matrix matDy, SatdynBasicDatum dynamicDatum, POS3D &accPerturb, Matrix &matPerturbPartial, bool bPerturb = false);
			bool adamsCowell_Interp(vector<TDT> interpTimelist, SatdynBasicDatum dynamicDatum, vector<TimePosVel> &orbitlist, vector<Matrix> &matRtPartiallist, double h = 75.0, int q = 11);
			// 测试函数
			void atmosphereDensity_jacchia71_standard(double h, double Too, double &density_Geodyn, double &density_Gill);
			void atmosphereDensity_jacchia71_standardandHe(double h, double Too, double &density_Geodyn, double &density_Gill, double &heliumdensity_Geodyn,  double &heliumdensity_Gill);
			AtmosphereDensity_output_add atmosphereDensity_jacchia71_Geodyn_add(POS3D sunPos_ECEF, POS3D leoPos_ECEF, double solarflux_day, double solarflux_mean, double kpIndex, ap_array &ap, double jd1958, 
				                             double& density, double& density_T, double&DENS,double&HELOG_Q2, POS3D& density_r, double re = EARTH_R, double f = EARTH_F);
			// 计算大气风
			void atmosphereWind_hwm14(int iyd, double sec, double alt, double glat, double glon, double stl, double f107a, double f107, double ap1, double ap2, double &w1, double &w2);
			void atmosphereWind(UTC t_UTC, POS3D leoPos_ECEF, double kpIndex, double &w1, double &w2);
			// 8阶 RungeKutta 法计算二阶常微分方程组，处理机动时刻附近的小尺度数值积分问题
			bool RungeKutta_8(SatdynBasicDatum dynamicDatum, TDT t_Begin, TDT t_End, Matrix &matY, Matrix &matDy, Matrix &matDDy);
			void orbitExtrapolation_jerk(SatdynBasicDatum dynamicDatum, GPST t0_forecast, GPST t1_forecast,  vector<TimePosVelAccJerk> &forecastOrbList, double h, double interval = 30.0, bool bECEF = true);
		public:
			JPLEphFile        m_JPLEphFile;        // JPL DE405星历数据文件
			OceanTideFile     m_OceanTideFile;     // 海潮数据文件
			OceanPoleTideFile m_OceanPoleTideFile;   // 海洋极潮数据文件
			TimeCoordConvert  m_TimeCoordConvert;  // 时间坐标系转换
			GravityModelCoeff m_GravityModelCoeff; // 重力场系数
			solarFluxFile     m_solarFluxFile;
			kpIndexFile       m_kpIndexFile;
			kpIndexFile       m_apIndexFile;
            CERESDataFile     m_CERESData_refl;   // 可见光Vis反射率
			CERESDataFile     m_CERESData_emit;   // 红外光IR发射率
            DTCFile           m_dtcFile;
			solfsmyFile       m_solfsmyFile;
			SatMacroFile      m_satMacroFile;      // 卫星表面光学系数
			TimeAttitudeFile  m_attFile;		   // 姿态文件, 为了便于accAtmosphereDrag_Macro计算使用, 将姿态文件提前到该类中, 20170117
			Matrix            m_matAxisBody2RTN;   // 星固系到轨道系的固定准换矩阵, 用于存在固定偏差角度的三轴稳定控制, 2015/03/09

			GravityModelCoeff m_MoonGravityModelCoeff; // 月球重力场系数
			champACCFile     m_champAccFile;      // CHAMP卫星加速度计文件，用于提取非保守力摄动加速度，20190415，韦春博添加
			graceACC1BFile   m_graceAccFile;      // GRACE卫星加速度计文件，用于提取非保守力摄动加速度，20190402，韦春博添加
			gracefoACT1BFile m_gracefoActFile;    // GRACE-FO卫星加速度计文件，用于提取非保守力摄动加速度，201200409，韦春博添加
			TimeAccelerometerFile m_accFile;      // 统一加速度计数据文件读取
		};
	}
}
