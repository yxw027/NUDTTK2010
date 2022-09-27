#pragma once
#include "SP3File.hpp"
#include "CLKFile.hpp"
#include "svnavFile.hpp"
#include "igs05atxFile.hpp"
#include "Rinex2_1_EditedObsFile.hpp"
#include "dynPODStructDef.hpp"
#include "AntPCVFile.hpp"
#include "StaOceanLoadingDisplacementFile.hpp"
#include "Troposphere_model.hpp"
#include "TROZPDFile.hpp"
#include "GPSYawAttitudeModel1995.hpp"
#include "Rinex2_1_MixedEditedObsFile.hpp"
#include "Rinex3_0_ObsFile.hpp"
#include "svnavMixedFile.hpp"
#include "GNSSYawAttitudeModel.hpp"

//  Copyright 2013, The National University of Defense Technology at ChangSha
using namespace NUDTTK::Geodyn;
namespace NUDTTK
{
	namespace GNSSPrecisePointPositioning
	{
		struct StaticPPPSolution_Epoch
		{
			DayTime t;
			double  clock;
			int     pppMark;  // 记录该时刻的单点定位完成情况, 0: 插值获得; 1: 有效解(伪码约束解); 2: 相位约束解
		    double  pdop;  
		};

		struct StaticPPPSolution
		{
			POS3D pos_Init; 
			POS3D pos_Est;                                      // 静态情形只估计一个位置参数
			vector<TropZenithDelayEstPara>  zenithDelayEstList; // 对流层天顶延迟参数，每 2-6 小时估计 1 次
			vector<TropGradEstPara>         troGradEstList;     // 对流层水平梯度参数，每24小时估计 1 次,2014/10/28			
			vector<StaticPPPSolution_Epoch> clkEstList;         // 钟差参数每个有效时刻估计一次
			int getIndexZenithDelayEstList(GPST t);
			int getIndexTroGradEstList(GPST t);                 // 2014/10/28
		};

		struct KinematicPPPSolution_Epoch
		{
			DayTime t;
			POS3D   pos;
			double  clock;
			int     pppMark;  // 记录该时刻的单点定位完成情况, 0: 插值获得; 1: 有效解(伪码约束解); 2: 相位约束解
			double  pdop;  
			double  sigma_L;
			int     eyeableGPSCount_L;
		};

		struct KinematicPPPSolution
		{
			vector<TropZenithDelayEstPara>     zenithDelayEstList; // 对流层天顶延迟参数每 2 小时估计 1 次
			vector<TropGradEstPara>            troGradEstList;     // 对流层水平梯度参数，每24小时估计 1 次,2014/10/28	
			vector<KinematicPPPSolution_Epoch> posclkEstList;      // 钟差参数每个有效时刻估计一次
			int getIndexZenithDelayEstList(GPST t);
			int getIndexTroGradEstList(GPST t);                 // 2014/10/28
		};

		struct pppClean_ObsEqArcElement
		{      
			int    nObsTime;     // 观测时间序号
			double obs;          // 原始相位无电离层组合观测量
			double res;   
			double robustweight; // 鲁棒估计调整权
			double prioriweight; // 先验权值
			double obs_MW;       // 用于计算MW模糊度
			int    prepro_flag;  // 用于周跳分析
			double obs_L1_L2;
			double t;
			double recClk;       // 接收机钟差，用于钟跳分析，2015/01/10,刘俊宏

			pppClean_ObsEqArcElement()
			{
				robustweight = 1.0;
				res = 0.0;
				prioriweight = 1.0;
				recClk = 0.0;
			}
		};

		typedef map<int, pppClean_ObsEqArcElement> pppClean_ObsEqArcElementMap;

		struct pppClean_ObsEqArc
		{
			double                      ambiguity_LIF;   // 消电离层模糊度
			double                      ambiguity_MW;    // MW模糊度
			int                         id_Sat;          // 卫星标号
			int                         slip_L1;
			int                         slip_L2;
			int                         bSlipMarked;     // 0, 新弧段; 1, 周跳（未修复）;  2, 周跳（已修复）
			pppClean_ObsEqArcElementMap obsList;

			int  updateRobustWeight(vector<int> &slipindexlist, double threshold_slip, double threshold_rms, unsigned int threshold_count_np = 20, double factor = 3, double threshold_max = DBL_MAX);
			int  updateRobustWeight_SlipCheck(vector<int> &slipindexlist, double threshold_slip, double threshold_rms, unsigned int threshold_count_np = 20, double factor = 3, double threshold_max = DBL_MAX);
			bool getLast_L1_L2_polyfit(double t_extra, double &L1_L2_extra, int m = 3, double max_span = 1200.0);
			bool getFirst_L1_L2_polyfit(double &L1_L2, int m = 3, double max_span = 1200.0);

			pppClean_ObsEqArc()
			{
				slip_L1 = 0;
				slip_L2 = 0;
				bSlipMarked = 0; 
			}
		};

		struct StaPPPPara
		{
			double       max_pdop;                  // 几何精度因子阈值(加权后), 超过该值的观测点将不参与运算
	        int          min_eyeableGPSCount;       // 最小可视卫星个数
			int          sampleInterval;            // 输入数据的采样间隔(s)
			double       apriorityRms_PIF;          // 先验无电离层码观测精度, 用于伪码和相位加权控制
			double       apriorityRms_LIF;          // 先验无电离层相位观测精度, 用于伪码和相位加权控制
			double       max_arclengh; 
			unsigned int min_arcpointcount;         // 最小连续点个数, 个数小于 min_arcpointcount 的弧段将被删除
			double       min_elevation;
			double       apriorityRms_TZD_abs;      // 先验测站电离层天顶延迟精度, 用于电离层天顶延迟绝对约束方程加权控制
			double       apriorityRms_TZD_rel;      // 用于相邻电离层天顶延迟参数的相对约束方程加权控制
			double       apriorityWet_TZD;          // 测站对流层湿分量估计先验值
			double       apriorityWet_TZD_period;   // 测站对流层湿分量估计周期
			double       apriorityRms_Grad_abs;     // 水平梯度绝对约束			
			double       apriorityRms_Grad_period;  // 水平梯度估计周期
			int          min_Wet_TZD_ncount;        // 对流层估计区间合并阈值
			int          zpdProductInterval;        // 输出的对流层产品采样间隔(s)			
			bool         bOn_WeightElevation;       // 是否进行高度角加权
			bool         bOn_Clk_GPSSAT;
			bool         bOn_GPSRelativity;         // 是否进行 GPS 卫星相对论改正,轨道偏心改正
			bool         bOn_GraRelativity;         // 是否进行引力引起的相对论改正
			bool         bOn_GPSAntPCOPCV;          // 是否进行 GPS 卫星天线PCO/PCV修正
			bool         bOn_RecAntARP;             // 是否进行接收天线参考点修正
			bool         bOn_RecAntPCOPCV;          // 是否进行接收天线相位中心修正
			bool         bOn_PhaseWindUp;
			bool         bOnEst_StaTropZenithDelay; // 是否进行测站对流层天顶延迟估计
			bool         bOnEst_StaTroGradient;     // 是否进行测站对流层水平梯度估计
			bool         bOn_SolidTides;
			bool         bOn_OceanTides;
			bool         bOn_TROProduct;            // 是否输出对流层产品
			bool         bOn_GYM95;                 // 是否采用GYM95模型计算卫星姿态
			int          troProduct_doy;            // 输出对流层产品的年积日，计划用下述变量替代
			GPST         troProductName_Time;       //子频道文件命名参考历元，鞠冰，2016/1/19
			double       robustfactor_OC_edited;
			double       threshold_OC_max;
			bool         bOn_DebugInfo_Shadow;      // 是否输出阴影信息
			
			bool         bOn_GEOSatWeight;          // 是否对北斗GEO卫星加权处理
			double       GEOSatWeight;              // 北斗GEO卫星权重
			bool         bOn_AntPCV;                // 2015/01/14,北斗PCV修正测试

			StaPPPPara()
			{
				max_pdop                  = 4.5;
				min_eyeableGPSCount       = 5;
				sampleInterval            = 300;
				apriorityRms_PIF          = 0.50;
				apriorityRms_LIF          = 0.005;
				bOn_WeightElevation       = false;
				min_elevation             = 10.0;
				max_arclengh              = 3600.0;
				min_arcpointcount         = 20;                    
				apriorityRms_TZD_abs      = 0.5;
				apriorityRms_TZD_rel      = 0.04;
				apriorityWet_TZD          = 0;        // 测站对流层湿分量估计先验值
				apriorityWet_TZD_period   = 3600 * 2; // 周期(2-6h)
				apriorityRms_Grad_abs     = 0.03;     // GAMIT，待测试，不增加相对约束(Reza Ghoddousi-Fard,2009)				
				apriorityRms_Grad_period  = 3600 * 24;// GAMIT,Bernese
				min_Wet_TZD_ncount        = 20;       // 少于此值则合并估计区间
				zpdProductInterval        = 300;
				bOn_Clk_GPSSAT            = true;
				bOn_GPSRelativity         = true;
				bOn_GraRelativity         = false;
				bOn_GPSAntPCOPCV          = true;
				bOn_PhaseWindUp           = true;
				bOn_RecAntARP             = true;
				bOn_RecAntPCOPCV          = true;
				bOnEst_StaTropZenithDelay = true;
				bOnEst_StaTroGradient     = false;
				bOn_SolidTides            = true;
				bOn_OceanTides            = true;
				bOn_TROProduct            = false;
				bOn_GYM95                 = true;
				troProduct_doy            = 0;
				robustfactor_OC_edited    = 3.0;
				threshold_OC_max          = DBL_MAX;
				bOn_GEOSatWeight          = true;
				GEOSatWeight              = 1.0/3.0;
				bOn_AntPCV                = true;
				bOn_DebugInfo_Shadow      = false;
			}
		};
		
		//为处理融合观测数据添加的数据结构   2017/3/31  昌虓
		struct MixedGNSSPPPDatum
		{
			char                                  recType_CPN;                  //接收机类型，主要针对GPS数据
			char                                  cSystem;                      //系统标识
	 		vector<Rinex2_1_MixedEditedObsEpoch>  editedObsEpochlist;           //单系统历元格式观测数据，混合格式
			vector<Rinex2_1_MixedEditedObsSat>    editedObsSatlist;             //单系统卫星格式观测数据，混合格式
			map<int, PODEpoch>                    mapDynEpochList;              //历元法方程数据
			vector<ObsEqEpoch>                    P_IFEpochList;                //消电离层伪码数据（历元记）
			vector<ObsEqEpoch>                    L_IFEpochList;                //消电离层伪码数据（历元记）
			vector<pppClean_ObsEqArc>             L_IFArcList;                  //消电离层相位数据（弧段记）, 这里和融合定轨中用的数据结构不同
			int                                   ambiguityIndexBegin;          //各系统模糊度起始编号，用于融合数据处理时记录弧段的初始索引
			map<int, double>                      mapWindupPrev;                //记录Windup修正数据
			vector<int>                           mixedEpochIdList;             //记录当前历元序列在[混合格式]中的历元序号，便于相互索引
			vector<int>                           epochIdList;                  //记录混合历元序列在[当前格式]中的历元序号，便于相互索引
			vector<O_CResEpoch>                   ocResP_IFEpochList;           //消电离层伪码O-C残差
			vector<O_CResArc>                     ocResL_IFArcList;             //消电离层相位O-C残差
			double                                FREQUENCE_L1;                 //此处及以下定义中的的Li,Pi（i=1,2）只是形式上的记法，实际可能对应于1,2,5三个频点中的某一个
			double                                FREQUENCE_L2;
			double                                WAVELENGTH_L1;
			double                                WAVELENGTH_L2;
			double                                coefficient_L1;
			double                                coefficient_L2;
			int                                   index_P1;
			int                                   index_P2;
			int                                   index_L1;
			int                                   index_L2;
			double                                pSlipRepairValue_L1[MAX_PRN];
			double                                pSlipRepairValue_L2[MAX_PRN];
			double                                pRobustWeight_code[MAX_PRN];
			BYTE                                  pbySatList[MAX_PRN];
			double                                weightSystem;                 //系统间的权系数之比，默认1.0
			double                                sysBias;                      //以GPS为参考系统的系统间偏差，默认为0

			MixedGNSSPPPDatum(double frequence1 = GPS_FREQUENCE_L1, double frequence2 = GPS_FREQUENCE_L2)
			{
				ambiguityIndexBegin = 0;
				FREQUENCE_L1        = frequence1;
				FREQUENCE_L2        = frequence2;
				WAVELENGTH_L1       = SPEED_LIGHT / FREQUENCE_L1;
				WAVELENGTH_L2       = SPEED_LIGHT / FREQUENCE_L2;
				coefficient_L1      = 1 / (1-pow(FREQUENCE_L2 / FREQUENCE_L1, 2));
				coefficient_L2      = 1 / (1-pow(FREQUENCE_L1 / FREQUENCE_L2, 2));
				memset(pSlipRepairValue_L1, 0, sizeof(double) * MAX_PRN);
				memset(pSlipRepairValue_L2, 0, sizeof(double) * MAX_PRN);
				memset(pRobustWeight_code , 0, sizeof(double) * MAX_PRN);
				memset(pbySatList         , 0, sizeof(BYTE) * MAX_PRN);
				weightSystem        = 1.0;
				sysBias             = 0.0;
			}
		};

		typedef vector<MixedGNSSPPPDatum>  MixedGNSSPPPDatumList;

		//为处理融合星历和钟差添加  昌虓
		struct  MixedClkSP3Datum
		{
			char      cSystem;
			SP3File   sp3File;
			CLKFile   clkFile;
		};

		struct  MixedAtxDatum
		{
			char          cSystem;
			igs05atxFile  atxFile;
		};

		class StaPPP
		{
		public:
			StaPPP(void);
		public:
			~StaPPP(void);
		public:
			void setSP3File(SP3File sp3File); 
			void setCLKFile(CLKFile clkFile); 
			bool loadSP3File(string  strSp3FileName);
			bool loadCLKFile(string  strCLKFileName);
			void weighting_Elevation(double Elevation, double& weight_P_IF, double& weight_L_IF);
			bool pdopSPP(int index_P1, int index_P2, POS3D recPos, Rinex2_1_EditedObsEpoch obsEpoch, int& eyeableGPSCount, double& pdop, double FREQUENCE_L1 = GPS_FREQUENCE_L1, double FREQUENCE_L2 = GPS_FREQUENCE_L2);
			bool SinglePointPositioning_PIF(int index_P1, int index_P2, POSCLK& posclk, Rinex2_1_EditedObsEpoch obsEpoch, int& eyeableGPSCount, double& pdop, double& rms_res, double FREQUENCE_L1 = GPS_FREQUENCE_L1, double FREQUENCE_L2 = GPS_FREQUENCE_L2, char cSatSystem = 'G', double threshold = 1.0E-002);
			bool pdopSPP_BDS(int index_P1, int index_P2, POS3D recPos, Rinex2_1_EditedObsEpoch obsEpoch, int& eyeableGPSCount, double& pdop, double FREQUENCE_L1 = GPS_FREQUENCE_L1, double FREQUENCE_L2 = GPS_FREQUENCE_L2);
		    bool pppclean(Rinex2_1_EditedObsFile &editedObsFile,  StaticPPPSolution& pppSolution, string outputPath = "");
			bool pppclean_GPS(Rinex2_1_EditedObsFile &editedObsFile,  StaticPPPSolution& pppSolution, string outputPath = "");
			bool pppclean_kinematic(Rinex2_1_EditedObsFile &editedObsFile, KinematicPPPSolution& pppSolution, string outputPath = "");
			bool pppclean_BDS(Rinex2_1_EditedObsFile &editedObsFile,  StaticPPPSolution& pppSolution, string outputPath = "");
			bool staticPPP_phase(string editedObsFilePath,  StaticPPPSolution& pppSolution, bool bResEdit = true);
			bool staticPPP_phase_BDS(string editedObsFilePath,  StaticPPPSolution& pppSolution, bool bResEdit = true);
			bool kinematicPPP_phase(string editedObsFilePath,  KinematicPPPSolution& pppSolution, bool bResEdit = true);
			int  getFlagIGSPCVInfo();
			//多系统融合PPP添加
			bool loadSP3CLKFileSet(char *strSys, vector<string> strSp3FileNameSet, vector<string> strClkFileNameSet, int num_Sys);
			bool pppMixedObsPreproc(string  strMixedObsFileName, Rinex2_1_MixedEditedObsFile  &mixedEditedObsFile);	
			bool pppclean_Mixed(Rinex2_1_MixedEditedObsFile  &mixedEditedObsFile, StaticPPPSolution& pppSolution, MixedGNSSPPPDatumList &multiGNSSData, bool &bOnGPS, bool &bOnBDS, string outputPath = "");
		private:
			bool loadEditedObsFile(string  strEditedObsFileName);
			int  m_flagIGSPCVInfo; // 0: 缺失; 1: 正确修正; 2: 替代, 存在风险			
		public:
			StaPPPPara                        m_pppParaDefine;
			CLKFile                           m_clkFile;		   // 精密钟差数据文件
			SP3File                           m_sp3File;		   // 精密星历数据文件
			Rinex2_1_EditedObsFile            m_editedObsFile;	   // 原始观测数据
			ENU                               m_arpAnt;			   // 天线偏心量
			svnavFile                         m_svnavFile;		   // GPS天线偏移
			igs05atxFile			          m_AtxFile;		   // 天线修正文件(2013/04/18, 鞠冰)
			StaOceanTide                      m_sotDatum;          // 海潮文件
			vector<O_CResEpoch>               m_ocResP_IFEpochList;// 无电离层伪码O-C残差
			vector<O_CResArc>                 m_ocResL_IFArcList;  // 无电离层相位O-C残差
		public:
			TimeCoordConvert                  m_TimeCoordConvert;  // 时间坐标系转换
			JPLEphFile                        m_JPLEphFile;        // JPL DE405星历数据文件
			GPSYawAttitudeModel1995           m_GYM95;             // 获取姿态控制模式下的卫星姿态

		   // 邵凯，2019/09/30，新模型文件
			svnavMixedFile                  m_svnavMixedFile;
			GNSSYawAttitudeModel            m_gymMixed;						
			map<string, AntCorrectionBlk>   m_mapGnssPCVBlk;// 获取卫星PCV信息

			//处理融合数据增加的变量  昌虓
		public:
			typedef vector<MixedClkSP3Datum>  clksp3FileList;      // 分系统存储星历和钟差数据
			clksp3FileList                    m_clksp3FileList;
			typedef vector<MixedAtxDatum>     atxFileList;         // 分系统存储天线修正数据
			atxFileList                       m_AtxFileList;

		};
	}
}
