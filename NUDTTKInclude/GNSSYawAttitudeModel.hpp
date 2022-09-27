#pragma once
#include "constDef.hpp"
#include "structDef.hpp"
#include "SP3File.hpp"
#include "jplEphFile.hpp"
#include "TimeCoordConvert.hpp"
#include "svnavMixedFile.hpp"
#include "GNSSBasicCorrectFunc.hpp"
#include "RuningInfoFile.hpp"

//  Copyright 2017, The National University of Defense Technology at ChangSha
namespace NUDTTK
{
	struct BLOCK_MaskString
	{
		static const char BLOCK_I[];
		static const char BLOCK_II[];
		static const char BLOCK_IIA[];
		static const char BLOCK_IIR_A[];
		static const char BLOCK_IIR_B[];
		static const char BLOCK_IIR_M[];
		static const char BLOCK_IIF[];
		static const char BLOCK_III_A[]; // GPS 三代
		static const char BEIDOU_2G[];
		static const char BEIDOU_2I[];
		static const char BEIDOU_2M[];
		static const char BEIDOU_3M[];
		static const char BEIDOU_3I[];
		static const char BEIDOU_3G[];
		static const char BEIDOU_3M_CAST[];
		static const char BEIDOU_3M_SECM_A[];
		static const char BEIDOU_3M_SECM_B[];
		static const char BEIDOU_3G_CAST[];
		static const char BEIDOU_3SM_CAST[];
		static const char BEIDOU_3SM_SECM[];
		static const char BEIDOU_3SI_CAST[];
		static const char BEIDOU_3SI_SECM[];
		static const char GLONASS[];
		static const char GLONASS_M[];
		static const char GLONASS_K1[];
		static const char GALILEO_1[];
		static const char GALILEO_2[];
		static const char GALILEO_0A[];
		static const char GALILEO_0B[];
	};

	enum TYPE_GYM_ID
	{
		TYPE_GYM_YAWNOMINAL  =  1,
		TYPE_GYM_NOONTURN    =  2,
		TYPE_GYM_SHADOWCROSS =  3,
		TYPE_GYM_SHADOWPOST  =  4,
		TYPE_GYM_ORBNORMAL   =  5,
		TYPE_GYM_UNKNOWN     =  0
	};

	// GPS卫星机动
	struct GYM_G95_NoonTurnDatum  // 包含 MidnightTurn 
	{
		GPST   ti;        // 进入 Noon Turn 时间
		double yaw_ti;    // 进入 Noon Turn 时的 yaw = nominal yaw
		double yawRate_ti;// 进入 Noon Turn 时的 yaw rate, 对应着卫星的最大硬件速率
		GPST   te;        // 离开 Noon Turn 的时间
	};

	struct GYM_G95_ShadowCrossDatum
	{
		GPST   ti;         // 进入 Shadow Cross 时间
		double yaw_ti;     // 进入 Shadow Cross 时的 yaw = nominal yaw
		double yawRate_ti; // 进入 Shadow Cross 时的 yaw rate
		GPST   t1;         // 达到硬件最大速度的时间
		GPST   te;         // 离开 Shadow Cross 的时间
		double yaw_te;     // 离开 Shadow Cross 时的 yaw
		double yawRate_te; // 离开 Shadow Cross 时的 yaw rate
	};

	struct GYM_G95_ShadowPostDatum
	{
		GPST   ti;         // 进入 Shadow Post 时间
		double yaw_ti;     // 进入 Shadow Post 时的 yaw
		double yawRate_ti; // 进入 Shadow Post 时的 yaw rate
		GPST   t1;         // 达到硬件最大速度的时间
		GPST   te;         // 离开 Shadow Post 的时间
		double yaw_te;     // 离开 Shadow Post 时的 yaw = nominal yaw
		double yawRate_te; // 离开 Shadow Post 时的 yaw rate
	};
	// GLONASS 卫星机动, 2011 The GLONASS-M satellite yaw-attitude model
	struct GYM_R11_NoonTurnDatum
	{
		GPST   ti;        // 进入 Noon Turn 时间
		double yaw_ti;    // 进入 Noon Turn 时的 yaw = nominal yaw
		double yawRate_ti;// 进入 Noon Turn 时的 yaw rate, 对应着卫星的最大硬件速率
		GPST   te;        // 离开 Noon Turn 的时间
	};

	struct GYM_R11_ShadowCrossDatum
	{
		GPST   ti;         // 进入 Shadow Cross 时间
		double yaw_ti;     // 进入 Shadow Cross 时的 yaw = nominal yaw
		double yawRate_ti; // 进入 Shadow Cross 时的 yaw rate
		GPST   t1;         // 地影期间 yaw 角达到出地影时刻对应 yaw 的时间
		GPST   te;         // 离开 Shadow Cross 的时间
		double yaw_te;     // 离开 Shadow Cross 时的 yaw
		double yawRate_te; // 离开 Shadow Cross 时的 yaw rate
	};
	// GALILEO 卫星机动, 2008 Dynamic yaw steering method for spacecraft
	struct GYM_E08_NoonTurnDatum
	{
		GPST   ti;        // 进入 Noon Turn 时间
		double yaw_ti;    // 进入 Noon Turn 时的 yaw = nominal yaw
		double yawRate_ti;// 进入 Noon Turn 时的 yaw rate, 对应着卫星的最大硬件速率
		int    sign_ti;   // 机动开始时正负号
		GPST   te;        // 离开 Noon Turn 的时间
	};

	struct GYM_E08_ShadowCrossDatum
	{
		GPST   ti;         // 进入 Shadow Cross 时间
		double yaw_ti;     // 进入 Shadow Cross 时的 yaw = nominal yaw
		double yawRate_ti; // 进入 Shadow Cross 时的 yaw rate
		int    sign_ti;    // 机动开始时正负号
		GPST   te;         // 离开 Shadow Cross 的时间
		double yaw_te;     // 离开 Shadow Cross 时的 yaw
		double yawRate_te; // 离开 Shadow Cross 时的 yaw rate
	};
	// BDS卫星机动
	struct GYM_C15_YawNominalEntryDatum
	{
		GPST   tBeta;     // beta角开始大于阈值
		double yaw_tBeta; // yaw_tBeta = 0
		int    id_time;

		GPST   t0;        // 当yaw nominal 接近 yaw_tBeta 时, 开始机动
		double yaw_t0;    // yaw_t0 = yaw_tBeta = 0
		GPST   t1;        // 完成机动, 机动过程很短, 一般不超过10秒 *
		double yaw_t1;    // yaw_t1 = nominal yaw
	};

	struct GYM_C15_OrbNormalEntryDatum
	{
		GPST   tBeta;     // beta角开始小于阈值
		double yaw_tBeta; // yaw_tBeta = nominal yaw
		int    id_time;

		GPST   t0;        // 当yaw nominal 接近 0 时, 开始机动
		double yaw_t0;    // yaw_t0 = nominal yaw, 接近 0
		GPST   t1;        // 完成机动, 机动过程很短, 一般不超过10秒 *
		double yaw_t1;    // yaw_t1 = 0
	};
	// BDS3正午午夜卫星机动, 2019 北斗导航卫星光压模型构建与精化研究-王晨
	struct GYM_C15_NoonTurnDatum
	{
		GPST   ti;        // 进入 Noon Turn 时间
		double yaw_ti;    // 进入 Noon Turn 时的 yaw = nominal yaw
		double yawRate_ti;// 进入 Noon Turn 时的 yaw rate, 对应着卫星的最大硬件速率
		double u_ti;      // 进入 Noon Turn 时的 轨道角
		double uRate_ti;  // 进入 Noon Turn 时的 轨道角速率变化
		int    sign_ti;   // 机动开始时正负号
		GPST   te;        // 离开 Noon Turn 的时间
	};
	struct GYM_MixedSat
	{
		string                               nameBlock;
		char                                 yawBiasFlag;         // Yaw Bias 标记
		double                               max_yawRate;         // 硬件 yaw 最大速度, 单位deg / s
		double                               max_yawRateRate;     // 硬件 yaw 最大加速度, 单位deg / ss
		double                               min_betaBDSYaw2Orb;
		vector<GYM_G95_NoonTurnDatum>        gpsNoonTurnList;
		vector<GYM_G95_ShadowCrossDatum>     gpsShadowCrossList;
		vector<GYM_G95_ShadowPostDatum>      gpsShadowPostList;
		vector<GYM_R11_NoonTurnDatum>        glonassNoonTurnList;
		vector<GYM_R11_ShadowCrossDatum>     glonassShadowCrossList;
		vector<GYM_E08_NoonTurnDatum>        galileoNoonTurnList;
		vector<GYM_E08_ShadowCrossDatum>     galileoShadowCrossList;
		vector<GYM_C15_YawNominalEntryDatum> bdsYawNominalEntryList;
		vector<GYM_C15_OrbNormalEntryDatum>  bdsOrbNormalEntryList;
		vector<GYM_C15_NoonTurnDatum>        bdsNoonTurnList;

		// 参数参考：2009 A simplified yaw-attitude model for eclipsing GPS satellites
		GYM_MixedSat(SvNavMixedLine mixedLine)
		{
			nameBlock   = mixedLine.szBlock;
			yawBiasFlag = mixedLine.yawBiasFlag;
			max_yawRate = mixedLine.yawRate; 
			max_yawRateRate    = 0.0018; // 硬件 yaw 加速度阈值 deg / ss
			min_betaBDSYaw2Orb = 0.0;
			// BLOCK_I BLOCK_II
			if(nameBlock.find(BLOCK_MaskString::BLOCK_I)  != -1  // BLOCK I 加速度待确定
			|| nameBlock.find(BLOCK_MaskString::BLOCK_II) != -1)
			{
				max_yawRateRate = 0.0018;
			}
			// BLOCK_IIA
			if(nameBlock.find(BLOCK_MaskString::BLOCK_IIA) != -1)
			{
				max_yawRateRate = 0.00165;
			}
			// BLOCK_IIR_A、BLOCK_IIR_B、BLOCK_IIR_M
			if(nameBlock.find(BLOCK_MaskString::BLOCK_IIR_A) != -1
			|| nameBlock.find(BLOCK_MaskString::BLOCK_IIR_B) != -1
			|| nameBlock.find(BLOCK_MaskString::BLOCK_IIR_M) != -1)
			{
				max_yawRateRate = 0.0018;
			}
			if(nameBlock.find(BLOCK_MaskString::BEIDOU_2I) != -1
			|| nameBlock.find(BLOCK_MaskString::BEIDOU_2M) != -1)
			{
				min_betaBDSYaw2Orb = 4.0; // 4度时候进行姿态切换
			}
		}
	};

	class GNSSYawAttitudeModel
	{
	public:
		GNSSYawAttitudeModel(void);
	public:
		~GNSSYawAttitudeModel(void);
	public:
		bool init(double span_t = 30.0, bool on_J2000 = true);
		void yaw2unitXYZ(TimePosVel gpsPVT, double acsYaw, POS3D &ex, POS3D &ey, POS3D &ez, bool bECEF = false);
		double getBetaSunAngle(POS3D sunPos, POS6D gnssPosVel);
		double getUOrbitAngle(POS3D sunPos, POS6D gnssPosVel);
		double getUOrbitAngle_perihelion(POS3D sunPos, POS6D gnssPosVel);
        // gps 部分
		static bool nominalYawAttitude_GPS(string nameBlock, POS3D sunPos, POS6D gpsPosVel, double &yaw, double &yawRate, double &betaSun, double b = 0.5);
		int gpsACSYawAttitude(string nameSat, GPST t, double &yaw, double &yawRate, double &betaSun, double &uOrbit, bool bUsed_NominalYawAttitude = true);
		// glonass 部分
		static bool nominalYawAttitude_GLONASS(string nameBlock, POS3D sunPos, POS6D gpsPosVel, double& yaw, double &yawRate, double &betaSun);
		int glonassACSYawAttitude(string nameSat, GPST t, double &yaw, double &yawRate, double &betaSun, double &uOrbit, bool bUsed_NominalYawAttitude = true);
		// galileo 部分
		static bool nominalYawAttitude_GALILEO(string nameBlock, POS3D sunPos, POS6D gpsPosVel, double& yaw, double &yawRate, double &betaSun);
		int galileoACSYawAttitude(string nameSat, GPST t, double &yaw, double &yawRate, double &betaSun, double &uOrbit, bool bUsed_NominalYawAttitude = true);
		// bds 部分
		static bool nominalYawAttitude_BDS(string nameBlock, POS3D sunPos, POS6D bdsPosVel, double &yaw, double &yawRate, double &betaSun);
	    static bool nominalYawAttitude_BDS_bias(string nameBlock, POS3D sunPos, POS6D bdsPosVel, double &yaw, double &yawRate, double &betaSun);
		int bdsYawAttitude(string nameSat, GPST t, double &yaw, double &yawRate, double &betaSun, double &uOrbit, bool bUsed_NominalYawAttitude = true);
		int bdsYawAttitude_continuous(string nameSat, GPST t, double &yaw, double &yawRate, double &betaSun, double &uOrbit);
	public:
		map<string, GYM_MixedSat> m_mapGYMInfo;
		SP3File           m_sp3File;           // 用于获取 gps 卫星位置和速度，默认为地固系。
		JPLEphFile        m_JPLEphFile;        // JPL DE405星历数据文件, 用于获取 sun 位置
		TimeCoordConvert  m_TimeCoordConvert;  // 用于时间坐标系转换
		svnavMixedFile    m_svnavMixedFile;	   // 用于获得卫星类型
		bool              m_bOnGYMInfo;//是否GYM模型信息记录输出
	};
}
