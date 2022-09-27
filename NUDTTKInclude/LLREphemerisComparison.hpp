#pragma once
#include "jplEphFile.hpp"
#include "TimeCoordConvert.hpp"
#include "StaEccFile.hpp"
#include "StaSscFile.hpp"
#include "StaOceanLoadingDisplacementFile.hpp"
#include "TimeAttitudeFile.hpp"
#include "float.h"
#include  <map>
#include "LLREphemerisComparison.hpp"

namespace NUDTTK
{
	namespace SLR
	{
		// 激光测月数据预处理结果的基本数据结构单元
		struct LLREditedObsElement
		{
			unsigned int id;                    // 地面测站在美国宇航局所编站址录中的编号
			string       LunarStaName;          // 月球激光后向反射器名称
			GPST         Ts;                    // 地面测站激光信号发射时间
			GPST         Tr;                    // 瞬时激光反射时刻
			double       obs;                   // 单程激光测距值（原始观测数据）
			double       obscorrected_value;    // 单程激光修正值
			double       r_mean;                // 由月球位置计算的上下行距离的平均值 
			double       v_r;                   // 视线方向多普勒速度, 2009/01/20
			POS3D        staPos_ECEF;           // 地面测站位置, 用于方向分析
			BLH          staBLH;
			double       wavelength;
			double       temperature;
			double       pressure;
			double       humidity;
			double       dr_correct_Trop;        // 对流层改正
			double       dr_correct_Relativity;  // 相对论改正
			double       dr_correct_StaEcc;      // 测站偏心改正
			double       dr_correct_Tide;        // 潮汐改正
			bool         bOn_Trop;
			bool         bOn_Relativity;
			bool         bOn_StaEcc;
			bool         bOn_Tide;
			int          s_i_pass;               // 在原始观测数据中弧段序号
			int          s_j;                    // 在原始观测数据中弧段s_i内的具体位置
			LLREditedObsElement()
			{
				v_r                   = 0;
				dr_correct_Trop       = 0;
				dr_correct_Relativity = 0;
				dr_correct_Tide       = 0;
				dr_correct_StaEcc     = 0;
				bOn_Trop              = 0;  
				bOn_Relativity        = 0;
				bOn_StaEcc            = 0;
				bOn_Tide              = 0;
			}
		};
		struct LLREditedObsArc
		{
			unsigned int id;	
			string       LunarStaName;          // 月球激光后向反射器名称
			double mean;
			double rms;
			vector<LLREditedObsElement> editedLLRObsList;
			LLREditedObsArc()
			{
				mean = 0.0;
				rms  = 0.0;
			}
		};

		// 月球角反射器结构
		struct LunarStaDatum
		{
			POS3D                          pos_ECEF;      // 月球测站位置，月固坐标系
            ENU                              arpAnt;	  // 月球测站激光反射器先验安装位置
			vector<LLREditedObsArc>    editedObsArc;      // 观测数据弧段编辑
			bool                              bUsed;
			LunarStaDatum()
			{
				bUsed = false;
			}
		};

		typedef map<string, LunarStaDatum> LunarStaDatumMap;

		class LLREphemerisComparison
		{
			public:
				LLREphemerisComparison(void);
			public:
				~LLREphemerisComparison(void);
			public:
				bool getStaPosvel(UTC t, int id, POS6D& posvel);
				bool getStaEcc(UTC t, int id, ENU& ecc);
				bool mainLLREphemerisComparison(string stLLRObsFileName, int nObsFileType, double min_elevation = 10.0, double threshold_res = 100.0, bool bResEdit = true);
			public:
				JPLEphFile            m_JPLEphFile;               // JPL DE星历数据文件
				TimeCoordConvert      m_TimeCoordConvert;         // 时间坐标系转换
				// 月面激光后向反射器信息
				LunarStaDatumMap      m_mapStaDatum;
				// 地球激光测站信息
				vector<StaEccRecord>  m_staEccList;               // 地球激光测站的偏心信息，地球ENU
				vector<StaSscRecord>  m_staSscList;               // 地球激光测站的坐标信息，地固系
				StaOceanLoadingDisplacementFile  m_staOldFile;    // 测站的海潮振幅和相位文件( http://www.oso.chalmers.se/~loading/ 网站提供 )
		};
	}
}
