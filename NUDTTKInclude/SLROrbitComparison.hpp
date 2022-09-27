#pragma once
#include "jplEphFile.hpp"
#include "TimeCoordConvert.hpp"
#include "StaEccFile.hpp"
#include "StaSscFile.hpp"
#include "StaSscFile_14.hpp"
#include "StaOceanLoadingDisplacementFile.hpp"
#include "TimeAttitudeFile.hpp"
#include "Sinex2_0_File.hpp"
#include "float.h"

namespace NUDTTK
{
	namespace SLR
	{
		// 不同姿态模型数据
		enum TYPE_ATT_MODEL_SLR
		{
			Body2J2000   = 1,            // 星固坐标系到惯性坐标系, GRACE,CHAMP
			Body2ECEF    = 2             // 星固坐标系到地固坐标系，Swarm
		};
		enum TYPE_StaPOS_MODEL
		{
			Ssc   = 1,             // Ssc产品
			Snx    = 2             // Snx产品
		};
		// 激光预处理结果的基本数据结构单元, 2008/04/11
		struct SLREditedObsElement
		{
			unsigned int id;                    // 测站在美国宇航局所编站址录中的编号
			GPST         Ts;                    // 地面测站激光信号发射时间
			GPST         Tr;                    // 瞬时激光反射时刻
			double       obs;                   // 单程激光测距值（原始观测数据）
			double       obscorrected_value;    // 单程激光修正值
			double       r_mean;                // 由卫星位置计算的上下行距离的平均值 
			double       v_r;                   // 视线方向多普勒速度, 2009/01/20
			POS6D        leoPV_ECEF;            // 激光反射时刻的卫星轨道位置, 用于方向分析
			POS3D        staPos_ECEF;           // 测站位置, 用于方向分析
			BLH          staBLH;
			double       wavelength;
			double       temperature;
			double       pressure;
			double       humidity;

			double       dr_correct_Trop;       // 对流层改正
			double       dr_correct_Relativity; // 相对论改正
			double       dr_correct_SatMco;     // 卫星质心改正 Mass Center Correct
			double       dr_correct_StaEcc;     // 测站偏心改正
			double       dr_correct_Tide;       // 潮汐改正

			bool         bOn_Trop;
			bool         bOn_Relativity;
			bool         bOn_SatMco;
			bool         bOn_StaEcc;
			bool         bOn_Tide;

			bool         bOn_Simu;              // 仿真标记
			int          s_i_pass;              // 在原始观测数据中弧段序号
			int          s_j;                   // 在原始观测数据中弧段s_i内的具体位置

			
			double       getStaLosElevation(); // 计算激光测站的高度角
			double       getLeoLosElevation();

			SLREditedObsElement()
			{
				v_r                   = 0;
				dr_correct_Trop       = 0;
				dr_correct_Relativity = 0;
				dr_correct_Tide       = 0;
				dr_correct_StaEcc     = 0;
				dr_correct_SatMco     = 0;
				bOn_Trop              = 0;  
				bOn_Relativity        = 0;
				bOn_SatMco            = 0;
				bOn_StaEcc            = 0;
				bOn_Tide              = 0;
				bOn_Simu              = 0;
			}
		};

		struct SLREditedObsArc
		{
			unsigned int id;	
			double mean;
			double rms;
			vector<SLREditedObsElement> editedSLRObsList;

			SLREditedObsArc()
			{
				rms = 0;
			}
		};

		class SLROrbitComparison
		{
		public:
			SLROrbitComparison(void);
		public:
			~SLROrbitComparison(void);
		public:
			bool getSubArcOrbList(GPST t0, GPST t1, vector<TimePosVel> &orbList, int nExtern = 8, double maxSpan = DBL_MAX);
			bool getSubArcAttList(GPST t0, GPST t1, vector<TimeAttLine> &attList, int nExtern = 8, double maxSpan = DBL_MAX);
			bool getStaPosvel(UTC t, int id, POS6D& posvel);
			bool getStaEcc(UTC t, int id, ENU& ecc);
			bool getStaPosvel_14(UTC t, int id, POS6D& posvel);
			//bool main_cstg(string strCstgObsFileName, vector<SLREditedObsArc>& editedObsArc, double min_elevation = 10.0, double threshold_res = 1.0, bool bResEdit = true);
			bool mainOrbComparison(string stSLRObsFileName, int nObsFileType, vector<SLREditedObsArc>& editedObsArc, double min_elevation = 10.0, double threshold_res = 1.0, bool bResEdit = true);
		public:
			static bool getOrbInterp(vector<TimePosVel> orbList, GPST t, TimePosVel &point, unsigned int nLagrange = 8);
			static bool getAttMatrixInterp(vector<TimeAttLine> attList, GPST t, Matrix &matATT, unsigned int nlagrange = 4); 
		public:
			bool                  m_bOn_YawAttitudeModel;			
			POS3D                 m_mcoLaserRetroReflector; // 激光反射器质心偏移
			JPLEphFile            m_JPLEphFile;             // JPL DE405星历数据文件
			TimeCoordConvert      m_TimeCoordConvert;       // 时间坐标系转换
			vector<StaEccRecord>  m_staEccList;             // 激光测站的偏心信息
			vector<StaSscRecord>  m_staSscList;             // 激光测站的坐标信息
			//vector<StaSscRecord_14>  m_staSscList_14;       // 激光测站的坐标信息，增加时间信息 +
			StaOceanLoadingDisplacementFile  m_staOldFile;  // 测站的海潮振幅和相位文件( http://www.oso.chalmers.se/~loading/ 网站提供 )
            vector<TimePosVel>    m_orbList;                // 低轨卫星的精密轨道数据
			TYPE_ATT_MODEL_SLR    m_att_model;              // 低轨卫星的姿态类型
			//TimeAttitudeFile      m_attFile;                // 低轨卫星的姿态文件
			vector<TimeAttLine>   m_attList;                 // 低轨卫星的姿态数据,+ 邵凯，2021.6.24
			double                m_constRangeBias;         // 固定偏差校正, 依靠外部输入, 默认为 0
			bool                  m_bChecksum;
			TYPE_StaPOS_MODEL     m_staPos_model;
			Sinex2_0_File         m_snxFile;
		};
	}
}
