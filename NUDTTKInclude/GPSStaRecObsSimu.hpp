#pragma once
#include "SP3File.hpp"
#include "CLKFile.hpp"
#include "igs05atxFile.hpp"
#include "Rinex2_1_ObsFile.hpp"
#include "TimeCoordConvert.hpp"
#include <time.h>
//  Copyright 2012, The National University of Defense Technology at ChangSha
namespace NUDTTK
{
	namespace Simu
	{
		struct GPSStaRecObsSimuPara
		{ 
			bool    bOn_ARP_GPSAnt;
			bool    bOn_PCV_GPSAnt;
			bool    bOn_ARP_RecAnt;
			bool    bOn_PCV_RecAnt;
			bool    bOn_Rel_GPSSAT;
			bool    bOn_Clk_GPSSAT;
			bool    bOn_Ionosphere; 
			bool    bOn_RecClock;
			bool    bOn_RecObsNoise;
			double  min_elevation;         // 截至高度角阈值
			double  noiseSigma_P1;         // P1 观测噪声大小
			double  noiseSigma_P2;         // P2 观测噪声大小
			double  noiseSigma_L1;         // L1 观测噪声大小
			double  noiseSigma_L2;         // L2 观测噪声大小
			double  recClockAllanVar_h_0;
			double 	recClockAllanVar_h_2;

			GPSStaRecObsSimuPara()
			{
				min_elevation          = 10.0;   
				noiseSigma_P1          = 0.5;    
				noiseSigma_P2          = 0.5;    
				noiseSigma_L1          = 0.002;   
				noiseSigma_L2          = 0.002;   
				recClockAllanVar_h_0   = 0.05;
				recClockAllanVar_h_2   = 1.0E-15;
				bOn_ARP_GPSAnt         = false;
				bOn_PCV_GPSAnt         = false;
				bOn_ARP_RecAnt         = false;
				bOn_PCV_RecAnt         = false;
				bOn_Rel_GPSSAT         = true;
				bOn_Clk_GPSSAT         = false;
				bOn_Ionosphere         = false; 
				bOn_RecClock           = false;
				bOn_RecObsNoise        = true;
			}
		};

		class GPSStaRecObsSimu
		{
		public:
			GPSStaRecObsSimu(void);
		public:
			~GPSStaRecObsSimu(void);
		public:
			void   setSP3File(SP3File sp3File);
			void   setCLKFile(CLKFile clkFile);
			bool   loadSP3File(string  strSP3FileName);
			bool   loadCLKFile(string  strCLKFileName);
			void   setReceiverPos(double x,double y,double z);
			bool   judgeGPSSignalCover(POS3D posGPSSat, POS3D posRec, double cut_elevation = 10);
			bool   simuRecClock(double& p_t, double& q_t, double delta, double AllanVar_h_0 = 0.05, double AllanVar_h_2 = 1.0E-15);
			bool   simuGPSObsFile(Rinex2_1_ObsFile &obsFile, string MakerName, GPST t0, double h = 30.0, double arcLenth = 86400.0);
			bool   simuGPSObsFile_new(Rinex2_1_ObsFile &obsFile, GPST t0, double h = 30.0, double arcLenth = 86400.0);
		private:
			SP3File      m_sp3File;                  // GPS精密星历数据文件
			CLKFile      m_clkFile;
		    POS3D        m_posReceiver;              // 接收机位置
			igs05atxFile m_igs05atxFile;
		public:
			GPSStaRecObsSimuPara m_simuParaDefine;
			TimeCoordConvert     m_TimeCoordConvert; // 时间坐标系转换
		};
	}
}
