#pragma once
#include "SP3File.hpp"
#include "CLKFile.hpp"
#include "Rinex2_1_ObsFile.hpp"
#include "Rinex2_1_MixedObsFile.hpp"
#include "TimeCoordConvert.hpp"
#include "svnavFile.hpp"
#include "svnavMixedFile.hpp"
#include "jplEphFile.hpp"
#include <time.h>
#include "LeoGPSObsPreproc.hpp"
#include "GNSSBasicCorrectFunc.hpp"
#include "igs05atxFile.hpp"
#include "AntPCVFile.hpp"
#include "Ionex1_0_File.hpp"

namespace NUDTTK
{
	namespace Simu
	{
		struct GPSLeoRecObsSimuPara
		{ 			
			bool    bOn_GNSSSAT_AntPCOPCV;
			bool    bOn_GNSSSAT_Clk;
			bool    bOn_GNSSSAT_Relativity;
			bool    bOn_Rec_AntPCO;
			bool    bOn_Rec_AntPCV;
			bool    bOn_Rec_Clock;
			double  recClockAllanVar_h_0;
			double 	recClockAllanVar_h_2;
			bool    bOn_Rec_ObsNoise;
			bool    bOn_Rec_Relativity;
			bool    bOn_Ionosphere; 
			bool    bOn_C1; 
			bool    bOn_P1; 
			bool    bOn_P2; 
			bool    bOn_L1; 
			bool    bOn_L2; 
			bool    bOn_PhaseWindUp;       // 相位缠绕       
			double  noiseSigma_C1;         // C1 观测噪声大小[GPS] 单位是什么
			double  noiseSigma_P1;         // P1 观测噪声大小[GPS]
			double  noiseSigma_P2;         // P2 观测噪声大小[GPS]
			double  noiseSigma_L1;         // L1 观测噪声大小[GPS]
			double  noiseSigma_L2;         // L2 观测噪声大小[GPS]
			double  min_elevation;         // 截至高度角阈值

            // 兼容双模 - BDS + GPS
			bool    bOn_Rec_AntPCO_bds;
			bool    bOn_Rec_AntPCV_bds;
			bool    bOn_C1_bds; 
			bool    bOn_P1_bds; 
			bool    bOn_P2_bds;
			bool    bOn_P5_bds; 
			bool    bOn_L1_bds; 
			bool    bOn_L2_bds;		
			bool    bOn_L5_bds;
			bool    bOn_SysBias;           // 2015/01/07,刘俊宏，是否增加系统偏差
			double  noiseSigma_C1_bds;     // CA 观测噪声大小[BDS]
			double  noiseSigma_P1_bds;     // P1 观测噪声大小[BDS]
			double  noiseSigma_P2_bds;     // P2 观测噪声大小[BDS]
			double  noiseSigma_L1_bds;     // L1 观测噪声大小[BDS]
			double  noiseSigma_L2_bds;     // L2 观测噪声大小[BDS]
			double  noiseSigma_P5_bds;     // P5 观测噪声大小[BDS]
			double  noiseSigma_L5_bds;     // L5 观测噪声大小[BDS]
			double  sysBias;               // 2015/01/07,刘俊宏，系统偏差大小

			GPSLeoRecObsSimuPara()
			{
				min_elevation          = 10.0;
				bOn_C1                 = false; 
				bOn_P1                 = true; 
				bOn_P2                 = true; 
				bOn_L1                 = true; 
				bOn_L2                 = true; 
				noiseSigma_C1          = 0.5;
				noiseSigma_P1          = 0.5;    
				noiseSigma_P2          = 0.5;    
				noiseSigma_L1          = 0.002;   
				noiseSigma_L2          = 0.002;   
				recClockAllanVar_h_0   = 0.05;
				recClockAllanVar_h_2   = 1.0E-15;
				bOn_GNSSSAT_AntPCOPCV  = false;
				bOn_GNSSSAT_Relativity = true;
				bOn_GNSSSAT_Clk        = true;
				bOn_Rec_AntPCO         = true;
				bOn_Rec_AntPCV         = false;
				bOn_Ionosphere         = false; 
				bOn_Rec_Clock          = true;
				bOn_Rec_ObsNoise       = true;
				bOn_Rec_Relativity     = true;
				bOn_PhaseWindUp        = false;
                // 兼容双模 - BDS + GPS
				bOn_Rec_AntPCO_bds     = true;
			    bOn_Rec_AntPCV_bds     = false;
				bOn_C1_bds             = false; 
				bOn_P1_bds             = true; 
				bOn_P2_bds             = true; 
				bOn_P5_bds             = false;
				bOn_L1_bds             = true; 
				bOn_L2_bds             = true;
				bOn_L5_bds             = false;
				bOn_SysBias            = false;
				noiseSigma_C1_bds      = 0.5;
				noiseSigma_P1_bds      = 0.5;    
				noiseSigma_P2_bds      = 0.5;    
				noiseSigma_P5_bds      = 0.5;
				noiseSigma_L1_bds      = 0.002;   
				noiseSigma_L2_bds      = 0.002;
				noiseSigma_L5_bds      = 0.002;
				sysBias                = 1.0;
			}
		};

		class GPSLeoRecObsSimu
		{
		public:
			GPSLeoRecObsSimu(void);
		public:
			~GPSLeoRecObsSimu(void);
		public:
			void   setSP3File(SP3File sp3File);
			void   setCLKFile(CLKFile clkFile);
			bool   loadSP3File(string  strSP3FileName);
			bool   loadCLKFile(string  strCLKFileName);
			void   setAntPCO(double x,double y,double z);
			static bool judgeGPSSignalCover(POS3D posGPSSat, POS3D posRec, double cut_elevation = 10);
			bool   simuRecClock(double& p_t, double& q_t, double delta, double AllanVar_h_0 = 0.05, double AllanVar_h_2 = 1.0E-15);
			bool   simuGPSObsFile(Rinex2_1_ObsFile &obsFile, char cSatSystem = 'G');
			bool   simuGNSSMixedObsFile(Rinex2_1_MixedObsFile &obsFile, string strSystem = "G+C");
		private:
			SP3File              m_sp3File;    // GPS精密星历数据文件
			CLKFile              m_clkFile;
		public:
			GPSLeoRecObsSimuPara m_simuParaDefine;
			TimeCoordConvert     m_TimeCoordConvert; // 时间坐标系转换
			svnavFile            m_svnavFile;        // GPS天线偏移			
			svnavMixedFile   m_svnavMixedFile;//多系统GNSS天线偏移
			JPLEphFile           m_JPLEphFile;       // JPL DE405 星历数据文件
			vector<TimePosVel>   m_pvOrbList;
			igs05atxFile	     m_AtxFile;          // 天线修正文件
			AntPCVFile           m_pcvFile;          // 天线相位中心
			POS3D                m_pcoAnt;           // 接收机天线偏移矢量
			Ionex1_0_File        m_ionFile;          // IGS电离层网格产品
		public:
			// 兼容双模 - BDS + GPS
            AntPCVFile           m_pcvFile_bds;      // 天线相位中心
			POS3D                m_pcoAnt_bds;       // 接收机天线偏移矢量
		};

		class GPSTQRecObsSimu
		{
		public:
			GPSTQRecObsSimu(void);
		public:
			~GPSTQRecObsSimu(void);
		public:
			void   setSP3File(SP3File sp3File);
			void   setCLKFile(CLKFile clkFile);
			bool   loadSP3File(string  strSP3FileName);
			bool   loadCLKFile(string  strCLKFileName);
			bool   loadCLKFile_rinex304(string  strCLKFileName);
			void   setAntPCO(double x,double y,double z);
			static bool judgeGPSSignalCover(POS3D posGPSSat, POS3D posRec, double cut_elevation = 10);
			static bool judgeBDSSignalCover(POS3D posBDSSat, POS3D posRec, double cut_elevation);
			bool   simuRecClock(double& p_t, double& q_t, double delta, double AllanVar_h_0 = 0.05, double AllanVar_h_2 = 1.0E-15);
			bool   simuGPSObsFile(Rinex2_1_ObsFile &obsFile, char cSatSystem = 'G');
			bool   simuGNSSMixedObsFile(Rinex2_1_MixedObsFile &obsFile, string strSystem = "G+C");
		private:
			SP3File              m_sp3File;    // GPS精密星历数据文件
			CLKFile              m_clkFile;
		public:
			GPSLeoRecObsSimuPara m_simuParaDefine;
			TimeCoordConvert     m_TimeCoordConvert; // 时间坐标系转换
			svnavFile            m_svnavFile;        // GPS天线偏移
			svnavMixedFile                 m_svnavMixedFile;	     // 用于获得卫星类型			
			JPLEphFile           m_JPLEphFile;       // JPL DE405 星历数据文件
			vector<TimePosVel>   m_pvOrbList;
			igs05atxFile	     m_AtxFile;          // 天线修正文件
			AntPCVFile           m_pcvFile;          // 天线相位中心
			POS3D                m_pcoAnt;           // 接收机天线偏移矢量
			Ionex1_0_File        m_ionFile;          // IGS电离层网格产品
		public:
			// 兼容双模 - BDS + GPS
            AntPCVFile           m_pcvFile_bds;      // 天线相位中心
			POS3D                m_pcoAnt_bds;       // 接收机天线偏移矢量
		};
	}
}
