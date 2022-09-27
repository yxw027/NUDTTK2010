#pragma once
#include "SP3File.hpp"
#include "CLKFile.hpp"
#include "svnavFile.hpp"
#include "igs05atxFile.hpp"
#include "TimeAttitudeFile.hpp"
#include "Rinex2_1_LeoEditedObsFile.hpp"
#include "dynPODStructDef.hpp"
#include "LeoGPSObsPreproc.hpp"
#include "AntPCVFile.hpp"

//  Copyright 2013, The National University of Defense Technology at ChangSha
using namespace NUDTTK::SpaceborneGPSPreproc;
using namespace NUDTTK::Geodyn;
using namespace NUDTTK::SpaceborneGPSPreproc;
namespace NUDTTK
{
	namespace SpaceborneGPSPod
	{
		struct PPPSolution
		{
			DayTime t;
			POS3D   pos;
			POS3D   vel;
			double  clock;
			int     pppMark;  // 记录该时刻的单点定位完成情况, 0: 插值获得; 1: 有效解(伪码约束解); 2: 相位约束解
			double  pdop;
			double  sigma_L; 
			
			POS6D getPosVel()
			{
				POS6D pv;
				pv.x  = pos.x;
				pv.y  = pos.y;
				pv.z  = pos.z;
				pv.vx = vel.x;
				pv.vy = vel.y;
				pv.vz = vel.z;
				return pv;
			}
		};

		struct GPSLeoSatKinematicPPPPara
		{
			double       max_pdop;               // 几何精度因子阈值(加权后), 超过该值的观测点将不参与运算
	        int          min_eyeableGPSCount;    // 最小可视卫星个数
			double       apriorityRms_PIF;       // 先验无电离层码观测精度, 用于伪码和相位加权控制
			double       apriorityRms_LIF;       // 先验无电离层相位观测精度, 用于伪码和相位加权控制
			double       max_arclengh; 
			unsigned int min_arcpointcount;      // 最小连续点个数, 个数小于 min_arcpointcount 的弧段将被删除
			double       min_elevation;
			bool         bOn_WeightElevation;    // 是否进行高度角加权
			bool         bOn_GPSRelativity;      // 是否进行 GPS 卫星相对论改正
			bool         bOn_GPSAntPCO;          // 是否进行 GPS 卫星天线偏移修正
			bool         bOn_LEOAntPCO;          // 是否进行 LEO 卫星天线偏移修正
			bool         bOn_LEOAntPCV;          // 是否进行 LEO 卫星天线相位中心修正
			bool         bOn_PhaseWindUp;

			GPSLeoSatKinematicPPPPara()
			{
				max_pdop            = 4.5;
				min_eyeableGPSCount = 5;
				apriorityRms_PIF    = 0.50;
				apriorityRms_LIF    = 0.005;
				bOn_WeightElevation = false;
				min_elevation       = 5.0;
				max_arclengh        = 2000.0;
				min_arcpointcount   = 30;
				bOn_GPSRelativity   = true;
				bOn_GPSAntPCO       = true;
				bOn_PhaseWindUp     = true;
				bOn_LEOAntPCO       = true;
				bOn_LEOAntPCV       = true;
			}
		};

		class GPSLeoSatKinematicPPP
		{
		public:
			GPSLeoSatKinematicPPP(void);
		public:
			~GPSLeoSatKinematicPPP(void);
		public:
			void setSP3File(SP3File sp3File); 
			void setCLKFile(CLKFile clkFile); 
			bool loadSP3File(string  strSp3FileName);
			bool loadCLKFile(string  strCLKFileName);
			void weighting_Elevation(double Elevation, double& weight_P_IF, double& weight_L_IF);
			bool pdopSPP(int index_P1, int index_P2, Rinex2_1_LeoEditedObsEpoch obsEpoch, int& eyeableGPSCount, double& pdop);
		    bool kinematicPPP_phase(string editedObsFilePath,  vector<PPPSolution> &pppList, bool bResEdit = true);
		private:
			bool loadEditedObsFile(string  strEditedObsFileName);
		public:
			GPSLeoSatKinematicPPPPara m_pppParaDefine;
			CLKFile                   m_clkFile;			  // 精密钟差数据文件
			SP3File                   m_sp3File;			  // 精密星历数据文件
			Rinex2_1_LeoEditedObsFile m_editedObsFile;		  // 原始观测数据
			POS3D                     m_pcoAnt;				  // 天线偏心量
			svnavFile                 m_svnavFile;			  // GPS天线偏移
			igs05atxFile			  m_AtxFile;			  // 天线修正文件(2013/04/18, 鞠冰)
			TimeAttitudeFile          m_attFile;			  // 姿态文件
			AntPCVFile                m_pcvFile;			  // 天线相位中心
			vector<O_CResEpoch>       m_ocResP_IFEpochList;	  // 无电离层伪码O-C残差
			vector<O_CResArc>         m_ocResL_IFArcList;	  // 无电离层相位O-C残差
		public:
			TimeCoordConvert          m_TimeCoordConvert;  // 时间坐标系转换
			JPLEphFile                m_JPLEphFile;        // JPL DE405星历数据文件
		};
	}
}
