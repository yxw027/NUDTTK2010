#pragma once
#include "SP3File.hpp"
#include "CLKFile.hpp"
#include "Rinex2_1_ObsFile.hpp"
#include "Rinex2_1_MixedObsFile.hpp"
#include "Rinex3_03_EditedObsFile.hpp"
#include "Rinex3_0_ObsFile.hpp"
#include "TimeCoordConvert.hpp"
#include "jplEphFile.hpp"
#include <time.h>
#include "LeoGPSObsPreproc.hpp"
#include "GNSSBasicCorrectFunc.hpp"
#include "igs05atxFile.hpp"
#include "AntPCVFile.hpp"
#include "svnavMixedFile.hpp"
#include "GNSSYawAttitudeModel.hpp"

// Copyright 2018, SUN YAT-SEN UNIVERSITY TianQin Research Center at ZhuHai
using namespace NUDTTK;
using namespace NUDTTK::Math;

// 高轨卫星星载GNSS数据仿真

// 建议采用 RINEX 3.04 格式
namespace NUDTTK
{
	namespace TQGNSSPod
	{
		// HEO多GNSS系统结构
		struct TQGNSS_MixedSys
		{
			// 需要初始化部分
			char                                    cSys;       // 系统标识
			string                                  name_C1;    // 伪码
			string                                  name_C2;
			string                                  name_L1;    // 相位
			string                                  name_L2;
			string                                  nameFreq_L1;                  // 用于PCV修正
            string                                  nameFreq_L2;
			bool                                    bOn_P1; 
			bool                                    bOn_P2; 
			bool                                    bOn_L1; 
			bool                                    bOn_L2; 
			double                                  freq_L1;
			double                                  freq_L2;
			double									noiseSigma_P1;         // P1 观测噪声大小
			double									noiseSigma_P2;         // P2 观测噪声大小
			double									noiseSigma_L1;         // L1 观测噪声大小
			double									noiseSigma_L2;         // L2 观测噪声大小
			bool									bOn_GNSSSAT_AntPCOPCV;
			bool									bOn_GNSSSAT_Clk;
			bool									bOn_GNSSSAT_Relativity;
			// 主瓣(Main Lobe)+旁瓣(Side Lobe)
			double                                  coverAngleMainlobe;   // 主瓣信号覆盖角
			bool									bOn_SidelobeSignal;   // 是否使用旁瓣信号
			double                                  coverAngleSidelobe;   // 旁瓣信号覆盖角
			double                                  transPowerMainlobe;   // 主瓣信号发射功率
			double                                  transPowerSidelobe;   // 旁瓣信号发射功率
			//卫星与地球的切线夹角
			double                                  angel_meo;  //meo卫星与地球的切线夹角
			double                                  angel_geo;  //geo卫星与地球的切线夹角
			//功率相关参数
			double                                  minReceiveSignalPower;//SSV最小民用信号功率
			double									 satAntMaximumGain;//卫星天线最大增益
			// 不需要初始化部分
			int                                     index_C1;
			int                                     index_C2;
			int                                     index_L1;
			int                                     index_L2;
			int                                     iSys;                         // 记录数据在m_editedMixedObsFile存储的位置
			string getSysFreqName()
			{
				// 针对3_03混合格式修改, 谷德峰, 2018/06/14 
				char sysFreqName[4];
				sprintf(sysFreqName, "%1c%1c%1c", cSys, name_L1[1], name_L2[1]); // "Gij" "Cij" "Eij" "Rxx"
				sysFreqName[3] = '\0';
				return sysFreqName;
			}
			TQGNSS_MixedSys()
			{
				index_C1 = -1;
				index_C2 = -1;
				index_L1 = -1;
				index_L2 = -1;
				bOn_P1   = false;
				bOn_P2   = false;
				bOn_L1   = false;
				bOn_L2   = false;
				noiseSigma_P1          = 0.5;    
				noiseSigma_P2          = 0.5;    
				noiseSigma_L1          = 0.002;   
				noiseSigma_L2          = 0.002;  
				freq_L1                = GPS_FREQUENCE_L1;
				freq_L2                = GPS_FREQUENCE_L2;
				bOn_GNSSSAT_AntPCOPCV  = false;
				bOn_GNSSSAT_Relativity = false;
				bOn_GNSSSAT_Clk        = false;
				coverAngleMainlobe     = 42.0;
				transPowerMainlobe     = 27.0; //  ?确认
				bOn_SidelobeSignal     = false;
				coverAngleSidelobe     = 80.0;
				transPowerSidelobe     = 0.0; //  ?确认
				minReceiveSignalPower = -185.9;//BDS SSV
				satAntMaximumGain	=12.7;//dBw
				name_C1  = "";
				name_C2  = "";
				name_L1  = "";
				name_L2  = "";
				nameFreq_L1 = "";
				nameFreq_L2 = "";
				angel_meo   = 13.23;
				angel_geo   = 8.7;

			}
		};
		struct GNSSHeoRecObsSimuPara
		{ 			
			bool    bOn_Rec_AntPCO;
			bool    bOn_Rec_AntPCV;
			bool    bOn_Rec_Clock;
			double  recMaximumGain;      // 接收机最大增益
			double  recClockAllanVar_h_0;
			double 	recClockAllanVar_h_2;
			bool    bOn_Rec_ObsNoise;
			bool    bOn_Rec_Relativity;
			bool    bOn_Ionosphere; 
			bool    bOn_PhaseWindUp;       // 相位缠绕       
			double  min_elevation;         // 截至高度角阈值
			bool    bOn_SysBias;           // 2015/01/07,刘俊宏，是否增加系统偏差
			double  sysBias;               // 2015/01/07,刘俊宏，系统偏差大小
			double  receiverThreshold; //接收机阈值 dB Hz 2022.03.17杨诚鋆
			GNSSHeoRecObsSimuPara()
			{
				min_elevation          = 10.0;
				recClockAllanVar_h_0   = 0.05;
				recClockAllanVar_h_2   = 1.0E-15;
				recMaximumGain         = 0.0;  // ?确认
				bOn_Rec_AntPCO         = false;
				bOn_Rec_AntPCV         = false;
				bOn_Ionosphere         = false; 
				bOn_Rec_Clock          = false;
				bOn_Rec_ObsNoise       = false;
				bOn_Rec_Relativity     = false;
				bOn_PhaseWindUp        = false;
				bOn_SysBias            = false;
				sysBias                = 1.0;
			}
		};

		class TQGNSSSimu
		{
		public:
			TQGNSSSimu(void);
		public:
			~TQGNSSSimu(void);
		public:
			void   setSP3File(SP3File sp3File);
			void   setCLKFile(CLKFile clkFile);
			bool   loadSP3File(string  strSP3FileName);
			bool   loadCLKFile(string  strCLKFileName);
			bool   loadCLKFile_rinex304(string  strCLKFileName);
			void   setAntPCO(double x,double y,double z);
			bool   simuRecClock(double& p_t, double& q_t, double delta, double AllanVar_h_0 = 0.05, double AllanVar_h_2 = 1.0E-15);
		public: 
			// 输入strSatName-G01、C01，根据卫星标记来选择设置主瓣信号角度，替换原来judgeGPSSignalCover
			bool   judgeGNSSSignalCover(TQGNSS_MixedSys heoGNSSsys_i, string strSatName, POS3D posGNSSSat, POS3D posRec, double freq_i, double cut_elevation = 10.0);
			// 关注几个问题：使用单频 or 双频，相位能用吗？ 噪声先使用高斯白噪声，后续需尽量接近真实，比如电离层误差影响等
			bool   simuGNSSMixedObsFile(Rinex2_1_MixedObsFile &obsFile);
		private:
			SP3File                  m_sp3File;    // GPS精密星历数据文件
			CLKFile                  m_clkFile;
		public:
			GNSSHeoRecObsSimuPara     m_simuParaDefine;
			JPLEphFile				  m_JPLEphFile;       // JPL DE405 星历数据文件
			TimeCoordConvert		  m_TimeCoordConvert;  // 时间坐标系转换
			svnavMixedFile            m_svnavMixedFile;    // +
			GNSSYawAttitudeModel      m_gymMixed;	       // +
			vector<TimePosVel>        m_pvOrbList;
			igs05atxFile	          m_AtxFile;           // 天线修正文件
			AntPCVFile                m_pcvFile;           // 天线相位中心
			POS3D                     m_pcoAnt;            // 接收机天线偏移矢量	
			vector<TQGNSS_MixedSys>  m_dataMixedSysList;  // +
		};
	}
}
