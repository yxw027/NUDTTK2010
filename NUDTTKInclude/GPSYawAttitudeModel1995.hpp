#pragma once
#include "constDef.hpp"
#include "structDef.hpp"
#include "SP3File.hpp"
#include "jplEphFile.hpp"
#include "TimeCoordConvert.hpp"
#include "svnavFile.hpp"
#include "GNSSBasicCorrectFunc.hpp"

//  Copyright 2014, The National University of Defense Technology at ChangSha
namespace NUDTTK
{
	struct GYM95NoonTurnDatum 
	{
		GPST   ti;        // 进入 Noon Turn 时间
		double yaw_ti;    // 进入 Noon Turn 时的 yaw = nominal yaw
		double yawRate_ti;// 进入 Noon Turn 时的 yaw rate, 对应着卫星的最大硬件速率
		GPST   te;        // 离开 Noon Turn 的时间
	};

	struct GYM95ShadowCrossDatum
	{
		GPST   ti;         // 进入 Shadow Cross 时间
		double yaw_ti;     // 进入 Shadow Cross 时的 yaw = nominal yaw
		double yawRate_ti; // 进入 Shadow Cross 时的 yaw rate
		GPST   t1;         // 达到硬件最大速度的时间
		GPST   te;         // 离开 Shadow Cross 的时间
		double yaw_te;     // 离开 Shadow Cross 时的 yaw
		double yawRate_te; // 离开 Shadow Cross 时的 yaw rate
	};

	struct GYM95ShadowPostDatum
	{
		GPST   ti;         // 进入 Shadow Post 时间
		double yaw_ti;     // 进入 Shadow Post 时的 yaw
		double yawRate_ti; // 进入 Shadow Post 时的 yaw rate
		GPST   t1;         // 达到硬件最大速度的时间
		GPST   te;         // 离开 Shadow Post 的时间
		double yaw_te;     // 离开 Shadow Post 时的 yaw = nominal yaw
		double yawRate_te; // 离开 Shadow Post 时的 yaw rate
	};

	struct GYM95Sat
	{
		int                            id_Block;
		double                         max_yawRate;         // 硬件 yaw 最大速度, 单位deg / s
		double                         max_yawRateRate;     // 硬件 yaw 最大加速度, 单位deg / ss
		vector<GYM95NoonTurnDatum>     yawNoonTurnList;
		vector<GYM95ShadowCrossDatum>  yawShadowCrossList;
		vector<GYM95ShadowPostDatum>   yawShadowPostList;

		GYM95Sat(int id_Block_0)
		{// BLK: 1=Blk I  2=Blk II    3=Blk IIA    4=Blk IIR-A   5=Blk IIR-B   6=Blk IIR-M   7=Blk IIF 8=Blk IIIA
			id_Block = id_Block_0;
			max_yawRate = 0.13;
			max_yawRateRate = 0.0018;         // RR_IIA = 0.00180
			if(id_Block <= 2)
				max_yawRateRate = 0.00165;    // RR_II  = 0.00165
			if(id_Block >= 4)
				max_yawRate = 0.20;
		}
	};

	class GPSYawAttitudeModel1995
	{
	public:
		GPSYawAttitudeModel1995(void);
	public:
		~GPSYawAttitudeModel1995(void);

	public:
		static bool nominalYawAttitude(int id_Block,POS3D sunPos, POS6D gpsPosVel, double &yaw, double &yawRate, double &beta, double b = 0.5);
		bool initGYM95Info(double span_t = 30.0,bool bOn_sp3TOJ2000 = true);
		bool acsYawAttitude(int id_Sat, GPST t, double &yaw, double &yawRate, double b = 0.5, bool bUsed_NominalYawAttitude = true);
		void yaw2unitXYZ(TimePosVel gpsPVT, double acsYaw, POS3D &ex, POS3D &ey, POS3D &ez, bool bECEF = false);
		bool get_BeiDouYawAttitudeMode();
	private:
		map<int, GYM95Sat> m_mapGYM95Info;

	public:
		SP3File           m_sp3File;           // 用于获取 gps 卫星位置和速度，默认为地固系。
		JPLEphFile        m_JPLEphFile;        // JPL DE405星历数据文件, 用于获取 sun 位置
		TimeCoordConvert  m_TimeCoordConvert;  // 用于时间坐标系转换
		svnavFile         m_svnavFile;	       // 用于获得 BLOCK ID	
		map<int, bool>    m_mapBeiDouYawMode;   // 记录北斗卫星的偏航姿态， 2016/3/1， 鞠冰
	};
}
