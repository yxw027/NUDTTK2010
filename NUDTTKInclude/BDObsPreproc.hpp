#pragma once
#include "Rinex2_1_ObsFile.hpp"
#include "Rinex2_1_EditedObsFile.hpp"
#include "Rinex2_1_NavFile.hpp"
#include "Rinex3_0_NavFile.hpp"
#include "MathAlgorithm.hpp"
#include "SP3File.hpp"
#include "CLKFile.hpp"
#include "TimeCoordConvert.hpp"
#include "Troposphere_model.hpp"
#include <direct.h>


namespace NUDTTK
{
	namespace BDPreproc
	{
		struct BDObsPreprocDefine
		{
			double       max_ionosphere;           // 电离层延迟组合最大值，用于伪距野值剔除
	        double       min_ionosphere;           // 电离层延迟组合最小值
			double       min_elevation;            // 最低观测仰角,度
			unsigned int min_arcpointcount;        // 最小连续点个数, 个数小于 min_arcpointcount 的弧段将被删除
			double       max_arclengh;             // 相邻连续弧段间隔秒数阈值, 选择依据是最大连续跟踪弧段的时间, 一般不会超过半个轨道周期(秒)
			double       threshold_slipsize_mw;    // melbourne wuebbena 组合量大周跳的探测阈值(周)
			double       threshold_rms_mw;         // melbourne wuebbena 组合量的均方根阈值, 以宽巷周跳为单位
			double       vondrak_PIF_eps;          // vondrak 拟合参数
			double       vondrak_PIF_max;          // vondrak 拟合参数
			double       vondrak_PIF_min;          // vondrak 拟合参数
			unsigned int vondrak_PIF_width;        // vondrak 拟合参数
			double       interval;                 // 数据采样间隔
			double       threshold_gap;            // 弧段内的中断间隔阈值, 控制不要出现较大的中断(秒)
			double       threshold_gap_L1_L2;      // L1-L2判断周跳，控制不要出现较大的中断(秒)，中断太大，电离层影响会较大。
			double       threshold_slipsize_L1_L2; // L1-L2组合量探测周跳阈值(m)
			double       threshold_outliersize_L1_L2; // L1-L2组合量探测野值阈值(m)
			double       max_thrfrecodecom;        // 三频伪距组合最大值  
			double       threshold_slipsize_PLGIF; // 三频伪距相位无几何距离、无电离层组合周跳探测阈值
			double       threshold_slipsize_LGIF;  // 三频相位无几何距离、无电离层组合周跳探测阈值
			double       threshold_slipsize_LGF;   // 三频相位无几何距离组合周跳探测阈值
			double       threshold_LGIF_slipcheck; // 三频相位GIF组合周跳虚警检测阈值，历元差超过这一值，则不进行周跳虚警检测
			double       threshold_LGF_slipcheck;  // 三频相位GF组合周跳虚警检测阈值，历元差超过这一值，则不进行周跳虚警检测		

	        bool         bOn_IonosphereFree;       // 是否用消电离层组合探测周跳(m)
			double       threshold_recClk;     // 检验接收机钟差解是否有误的阈值(m)

			int          order_L1_L2;				// 拟合L1-L2的多项式阶数
			double       nwidth_L1_L2;				// L1-L2拟合的输出窗口宽度(单位：秒)
			double	     extendwidth_L1_L2;			// L1_L2拟合参数(单位：秒)

			BDObsPreprocDefine()
			{
				max_ionosphere           =  100;
		        min_ionosphere           = -20;
				min_elevation            =  10;
				min_arcpointcount        =  20;
				max_arclengh             =  3600;
				threshold_slipsize_mw    =  4.0;   //周跳阈值不能太小，如果取2会探测出很多虚警
				threshold_rms_mw         =  1.0;
				vondrak_PIF_eps          =  1.0E-18;
				vondrak_PIF_max          =  1.5;
				vondrak_PIF_min          =  0.5;
				vondrak_PIF_width        =  400;
				interval                 =  30;  // 默认数据采样间隔为30s 
				threshold_gap            =  600;
				threshold_gap_L1_L2      =  60;
				threshold_slipsize_L1_L2 =  0.15;  // 周跳探测能力为1周，
				threshold_outliersize_L1_L2 = 0.08; 
				max_thrfrecodecom        = 1.0e5;  // XIA1站的三频伪距组合值达到了1e4量级，其余站均小于200
				threshold_slipsize_PLGIF = 3.0;   
				threshold_slipsize_LGIF  = 0.03; 
				threshold_slipsize_LGF   = 0.20; 
				threshold_LGIF_slipcheck = 0.05;   //历元差绝对值小于该阈值则检验是判断出的周跳是否为虚警
				threshold_LGF_slipcheck  = 0.5;	   //历元差绝对值小于该阈值则检验是判断出的周跳是否为虚警	


				bOn_IonosphereFree       =  false;
				threshold_recClk         = 100;

				order_L1_L2				  =         4;	// 多项式阶数
				nwidth_L1_L2              =       600;  // 单位：秒
				extendwidth_L1_L2         =       120;	// 单位：秒
			}
		};		
		class BDObsPreproc
		{
		public:
			BDObsPreproc(void);
		public:
			~BDObsPreproc(void);
		public:
			void    setObsFile(Rinex2_1_ObsFile obsFile);
			bool    loadObsFile(string  strObsfileName);
			bool    loadNavFile(string  strNavfileName);
			bool    loadSp3File(string  strSp3fileName);
			bool    loadClkFile(string  strClkfileName);
			void    setStationPosition(POS3D pos);			
			BYTE    obsPreprocInfo2EditedMark1(int obsPreprocInfo);
			BYTE    obsPreprocInfo2EditedMark2(int obsPreprocInfo);

			// 双频观测数据处理，早期版本(刘俊宏)
			bool    detectCodeOutlier_ionosphere(int index_P1, int index_P2, double frequence_L1, double frequence_L2, Rinex2_1_EditedObsSat& editedObsSat,bool bOutTempFile = false);// 刘俊宏
			bool    detectPhaseSlip(int index_P1, int index_P2, int index_L1, int index_L2, double frequence_L1, double frequence_L2, Rinex2_1_EditedObsSat& editedObsSat,bool bOutTempFile = false);			
			bool    mainFuncObsPreproc(Rinex2_1_EditedObsFile &editedObsFile,bool bOutTempFile = false); 
			
			// 双频观测数据处理，GPS地面数据处理版本(鞠冰)
			bool    detectCodeOutlier_ionosphere_GPS(int index_P1, int index_P2, double frequence_P1, double frequence_P2, DayTime T0, Rinex2_1_EditedObsSat& editedObsSat, bool bOutTempFile = false);			
			bool    detectPhaseSlip_GPS(int index_P1, int index_P2, int index_L1, int index_L2, double frequence_L1, double frequence_L2, DayTime T0, Rinex2_1_EditedObsSat& editedObsSat, bool bOutTempFile = false);
			bool    mainFuncDualFreObsPreproc_GPS(Rinex2_1_EditedObsFile &editedObsFile, bool bOutTempFile = false);
			bool    RobustPolyFitL1_L2(double x[], double y[], double y_fit[], int n, int offset, int n_out, double N0, int m = 3);

            
			//  三频数据预处理(刘俊宏)
			bool    detectThrFreCodeOutlier(int index_P1, int index_P2,int index_P5, Rinex2_1_EditedObsSat& editedObsSat,bool bOutTempFile = false);			 
			bool    detectThrFrePhaseSlip(int index_L1, int index_L2,int index_L5, int index_P1, int index_P2, int index_P5, Rinex2_1_EditedObsSat& editedObsSat,bool bOutTempFile = false);	 
			bool    mainFuncThrFreObsPreproc(Rinex2_1_EditedObsFile &editedObsFile,bool bOutTempFile = false); 
			
			bool    receiverClkEst(Rinex2_1_EditedObsFile &editedObsFile,Rinex2_1_EditedObsFile &editedObsFile_clk);
			static bool    desampling_unsmoothed(POS3D posStaion,Rinex2_1_EditedObsFile &editedObsFile,Rinex2_1_EditedObsFile &desampleFile,bool btroCor = false,int nSampleSpan = 120, int Freq1 = 1, int Freq2 = 2);
			static bool    desampling_unsmoothed_GPS(POS3D posStaion, Rinex2_1_EditedObsFile &editedObsFile, Rinex2_1_EditedObsFile &desampleFile,bool btroCor = false,int nSampleSpan = 120);
			static bool    exportSP3File_GPST(string strnavFilePath,GPST t0, GPST t1,double interval = 5 * 60);
			static bool    exportCLKFile_GPST(string strnavFilePath,GPST t0, GPST t1,double interval = 5 * 60);
			//static bool    desampling_unsmoothed(Rinex2_1_EditedObsFile &editedObsFile,Rinex2_1_EditedObsFile &desampleFile,int nSampleSpan = 120);
			bool    mainFuncObsEdit(Rinex2_1_EditedObsFile &editedObsFile);    // 暂时不编
		public:
			BDObsPreprocDefine            m_PreprocessorDefine;		     
			string                        m_strPreprocFilePath;// 方便输出预处理和质量评估过程信息
			vector<TYPE_OBSPREPROC_INFO>  m_obsPreprocInfoList; // 预处理后的信息
			//Rinex2_1_ObsFile              m_obsFile;         // 观测数据
		private:
			bool getEditedObsEpochList(vector<Rinex2_1_EditedObsEpoch>& editedObsEpochlist); 
			bool getEditedObsSatList(vector<Rinex2_1_EditedObsSat>& editedObsSatlist);
			bool datalist_epoch2sat(vector<Rinex2_1_EditedObsEpoch> &editedObsEpochlist, vector<Rinex2_1_EditedObsSat>& editedObsSatlist);
		    bool datalist_sat2epoch(vector<Rinex2_1_EditedObsSat> &editedObsSatlist, vector<Rinex2_1_EditedObsEpoch>& editedObsEpochlist);			
		private:
			// 输入数据
			Rinex2_1_ObsFile              m_obsFile;         // 观测数据
			Rinex2_1_NavFile              m_navFile;         // 广播星历文件
			Rinex3_0_NavFile              m_navFile_3_0;     // 广播星历文件
			POS3D                         m_posStation;      // 测站位置
			SP3File                       m_sp3File;         // 卫星轨道，供编辑使用
			CLKFile                       m_clkFile;         // 导航卫星钟差数据
		};
	}
}