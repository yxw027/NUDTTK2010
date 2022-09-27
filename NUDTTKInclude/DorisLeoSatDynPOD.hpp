#pragma once
#include "structDef.hpp"
#include "SatdynBasic.hpp"
#include "dynPODStructDef.hpp"
#include "Doris2_2_EditedObsFile.hpp"
#include "DorisSTCDFile.hpp"
#include <map>

using namespace NUDTTK::Geodyn;
namespace NUDTTK
{
	namespace DORIS
	{
		struct DorisLeoSatDynPODPara
		{
	    	unsigned int                 min_arcpointcount;        // 最小连续点个数, 个数小于 min_count_arcpoint 的弧段将被删除
			double                       max_arclengh;             // 相邻连续弧段间隔阈值, 选择依据是最大连续跟踪弧段的时间, 一般不会超过半个轨道周期
			TYPE_SOLARPRESSURE_MODEL     solarPressureType;        // 太阳光压模型类型
			TYPE_ATMOSPHEREDRAG_MODEL    atmosphereDragType;       // 大气阻力模型类型
			double                       period_SolarPressure;  
			double                       period_AtmosphereDrag;
			double                       period_EmpiricalAcc;
			int                          max_OrbitIterativeNum;     // 轨道改进次数的最大值, 统一进行规定
			double                       apriorityRms_obs;          // 先验观测值精度
			bool                         bOnEst_StaTropZenithDelay; // 是否进行测站对流层天顶延迟估计
			double                       apriorityRms_TZD;          // 先验测站电离层天顶延迟一阶Gauss-Markov方程精度, 用于电离层天顶延迟约束方程加权控制
			bool                         bOnEst_ERP;                // 是否进行地球旋转参数估计
			double                       apriorityRms_xp;           // 单位: 弧度, 7.84E-7
			double                       apriorityRms_xpDot;        // 单位: 弧度/秒, 2.80E-13
			double                       apriorityRms_yp;           // 单位: 弧度, 7.84E-7
			double                       apriorityRms_ypDot;        // 单位: 弧度/秒, 2.80E-13
			double                       apriorityRms_ut1;          // 单位: 秒,  2.156E-8
			double                       apriorityRms_ut1Dot;       // 单位: 秒/秒, 3.470E-8
			
			DorisLeoSatDynPODPara()
			{	
				min_arcpointcount         = 5;
				max_arclengh              = 2000.0;
				solarPressureType         = TYPE_SOLARPRESSURE_1PARA;
				atmosphereDragType        = TYPE_ATMOSPHEREDRAG_J71_GEODYN;
				period_AtmosphereDrag     = 3600 * 24.0;
				period_SolarPressure      = 3600 * 24.0; 
				period_EmpiricalAcc       = 3600 *  6.0;
				max_OrbitIterativeNum     = 8;
				bOnEst_StaTropZenithDelay = false;
				bOnEst_ERP                = false;
				apriorityRms_TZD          = 0.1;
				apriorityRms_obs          = 0.0005 * 10; // 0.5mm/s * 10s
				// 参考 ign07wd01.eop.dsc
				// ERP | x & y pole |
				// Current IERS C04 values used as apriori |
				// estimated parameters: XPOLE, YPOLE, XPOLERATE, YPOLERATE |
				// a priori constraints: X,Y POLE 7.84D-7 !5 m |
				// X,Y POLE RATE 2.80D-13 ! 5 masec/day |
				// UT1-UTC,UT1-UTCrate |
				// UT1-UTC 2.156D-8 ! 0.01 mm |
				// UT1-UTC rate 3.470D-8 ! 3 msec/day |
				apriorityRms_xp           = 7.84E-7;     
				apriorityRms_xpDot        = 2.80E-13;
				apriorityRms_yp           = 7.84E-7;
				apriorityRms_ypDot        = 2.80E-13;
				apriorityRms_ut1          = 2.156E-8;
				apriorityRms_ut1Dot       = 3.470E-8;
			}
		};

		struct DorisObsEqEpochElement
		{
			double  obscorrected_value;
			double  duration;                // 距离差分对应的持续时间, 单位秒
			POS6D   vecLos_t0;               // 视线矢量-距离差分起始时刻
            POS6D   vecLos_t1;               // 视线矢量-距离差分终止时刻
			Matrix  interpRtPartial_t0;      // 插值偏导数序列
			Matrix  interpRtPartial_t1;
			double  weight;                  // 观测权值
			double  elevation_sat;           // 卫星观测高度角
			double  azimuth_sat;             // 卫星观测方位角
			double  elevation_sta_t0;        // 测站发射仰角-差分起始时刻
			double  elevation_sta_t1;        // 测站发射仰角-差分终止时刻
			TAI     t0_sta_transmit;         // 测站发射时刻-差分起始时刻
			TAI     t1_sta_transmit;         // 测站发射时刻-差分终止时刻
		};

		typedef map<int, DorisObsEqEpochElement> DorisPODEpochStationMap;

		struct DorisPODEpoch
		{
			int                         eyeableStaCount;
			TAI                         t;
			DorisPODEpochStationMap     mapDatum;
		};

		struct DorisObsEqArc
		{
			int                     id_Station;       // 测站号
			double                  offsetFrequence;  // 频偏估计结果
			double                  zenithDelay;      // 对流层估计结果
			vector<ObsEqArcElement> obsList;
		};

		struct DorisEopEstParameter
		{
			TAI    t0_xpyput1;
			double xp;           // 单位: 弧度
			double xpDot;        // 单位: 弧度/秒
			double yp;
			double ypDot;
			double ut1;          // 单位: 弧度
			double ut1Dot;       // 单位: 弧度/秒
			
			DorisEopEstParameter()
			{
				xp     = 0;
				xpDot  = 0;
				yp     = 0;
				ypDot  = 0;
				ut1    = 0;
				ut1Dot = 0;
			}

			// 计算地球旋转矩阵改进量
			void getEst_EOP(TAI t, Matrix &matEst_EP, Matrix &matEst_ER)
			{
				double spanSeconds = t - t0_xpyput1;
				double delta_xp = xp + xpDot * spanSeconds;
				double delta_yp = yp + ypDot * spanSeconds;
				matEst_EP.Init(3,3);
				matEst_EP.SetElement(0, 0,  1);
				matEst_EP.SetElement(0, 2,  delta_xp);
				matEst_EP.SetElement(1, 1,  1);
				matEst_EP.SetElement(1, 2, -delta_yp);
				matEst_EP.SetElement(2, 0, -delta_xp);
				matEst_EP.SetElement(2, 1,  delta_yp);
				matEst_EP.SetElement(2, 2,  1);

				double delta_ut1 = ut1 + ut1Dot * spanSeconds;
				matEst_ER.Init(3,3);
				matEst_ER.SetElement(0, 0,  1);
				matEst_ER.SetElement(0, 1,  delta_ut1);
				matEst_ER.SetElement(1, 0, -delta_ut1);
				matEst_ER.SetElement(1, 1,  1);
				matEst_ER.SetElement(2, 2,  1);
			}
		};

		class DorisLeoSatDynPOD : public SatdynBasic
		{
		public:
			DorisLeoSatDynPOD(void);
		public:
			~DorisLeoSatDynPOD(void);
		public:
			bool loadStdcFile(string strStdcFileFolder);
			bool getStationPos(string site_code, TAI t, double &x, double &y, double &z);
			bool getObsArcList(vector<Doris2_2_EditedObsEpoch> obsEpochList, vector<DorisObsEqArc> &obsArcList);
			bool adamsCowell_ac(TDT t0_Interp, TDT t1_Interp, SatdynBasicDatum dynamicDatum, vector<TimePosVel> &orbitlist_ac, vector<Matrix> &matRtPartiallist_ac, double h = 30.0, int q = 11);
			bool getOrbPartial_interp(TDT t, vector<TimePosVel>& orbitlist_ac, vector<Matrix>& matRtPartiallist_ac, TimePosVel &interpOrbit, Matrix &interpRtPartial);
			bool getTransmitPathDelay(TAI t, POS3D satPos_j2000, POS3D staPos_ECEF, DorisEopEstParameter eopEstPara, double& delay, POS6D& staPosVel_j2000, double threshold = 1.0E-07);
            bool dynamicPOD_2_2(string obsFilePath, SatdynBasicDatum &dynamicDatum, TAI t0_forecast, TAI t1_forecast, vector<TimePosVel> &forecastOrbList, double interval = 30.0, bool bForecast = true, bool bResEdit = true);
		public:
			double                 m_ppDorisStationPos[MAX_ID_DORISSTATION + 1][3]; // 记录每个DORIS测站位置(地固系)
			DorisLeoSatDynPODPara  m_podParaDefine;
			Doris2_2_EditedObsFile m_obsFile;      // 观测数据文件
			vector<DorisSTCDFile>  m_listStcdFile; // 测站位置数据文件
			POS3D                  m_pcoAnt;       // 天线偏心量
		};
	}
}
