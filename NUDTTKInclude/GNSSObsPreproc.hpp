#pragma once
#include "Rinex3_03_EditedObsFile.hpp"
#include "MathAlgorithm.hpp"
#include "Matrix.hpp"
#include "SP3File.hpp"
#include "svnavMixedFile.hpp"
#include "SinexBiasDCBFile.hpp"

using namespace NUDTTK::Math;

namespace NUDTTK
{
	namespace GPSPreproc
	{
		enum TYPE_FREQ_ID
		{
			TYPE_FREQ_TWO   = 0, // 双频模式
			TYPE_FREQ_THREE = 1
		};
		
		// 频率1和2在处理时主用, 频率3在进行三频处理时才使用
		struct PRE_Freq
		{
			double freq_L1;
			double freq_L2;
			double freq_L3;
		};

		// 系统观测类型对应的序号
		struct SysObsTypeNum                                               
		{
			char                        cSatSys;                            // 卫星系统
			vector<size_t>              obsTypeNumList;                            // 观测类型序号
		};
		struct PRE_MixedSys
		{
			// 需要初始化部分
			char                  cSys;             // 系统标识
			TYPE_FREQ_ID          type_Freq;             // 双频/三频周跳探测的开关
			string                name_C1;               // 伪码
			string                name_C2;
			string                name_C3;
			string                name_L1;              // 相位
			string                name_L2;
			string                name_L3;
		    string                name_S1;             // 信噪比 
			string                name_S2;
			string                name_S3;
			string                nameFreq_L1;         // 频率名称"G01"，主要用于PCV修正
			string                nameFreq_L2;   
			string                nameFreq_L3;
			PRE_Freq              freqSys;             // GLONASS的频率信息存储在freqSatList中, 程序里自动赋值
			string                signalPriority_L1;   // L1频率对应的优先级
			string                signalPriority_L2;
			string                signalPriority_L3;

			// 与系统有关的预定义部分
			double                min_Elevation;            // 观测仰角阈值, 仰角低于min_Elevation的观测数据不使用
			double			      max_Ionosphere;           // 伪码电离层组合阈值（最大值）
			double			      min_Ionosphere;           // 伪码电离层组合阈值（最小值）
			bool                  on_L1_L2_SlipDetect;      // 是否使用 L1-L2 历元差探测周跳，鞠冰，2016/2/24
            bool                  on_IF_SlipDetect;         // 是否使用 A_IF 组合探测周跳，鞠冰,2016/3/3
			double			      max_L1_L2_Difference;		// L1-L2历元差阈值, 小于此阈值的点按数值大小排列, 作为备选剔除点, 鞠 冰,  2016/3/17
			double			      max_L1_L2_SlipSize;	    // L1-L2探测周跳阈值(单位：米)
			double			      max_MW_SlipSize;	    	// MW组合周跳探测阈值(单位：周)
			double                max_IF_SlipSize;		    // A_IF组合周跳探测阈值(单位：米)
			double                width_L1_L2_SubSecFit;    // L1-L2序列分段拟合的输出窗口宽度(单位：秒)
			BYTE                  order_L1_L2_SubSecFit;	// L1-L2序列分段拟合的多项式阶数（多项式次数 + 1）
			double			      exten_L1_L2_SubSecFit;    // L1_L2序列分段拟合的左右端点向外拓展宽度(单位：秒)
			BYTE				  count_GrossPoint_PreDel;	// 预先剔除的超差点个数
			bool                 bOnTripleFreqsCSD;			// 三频周跳探测开关
			double       max_thrfrecodecom;        // 三频伪距组合最大值  
			double       threshold_slipsize_PLGIF; // 三频伪距相位无几何距离、无电离层组合周跳探测阈值
			double       threshold_slipsize_LGIF;  // 三频相位无几何距离、无电离层组合周跳探测阈值
			double       threshold_slipsize_LGF;   // 三频相位无几何距离组合周跳探测阈值
			double       threshold_LGIF_slipcheck; // 三频相位GIF组合周跳虚警检测阈值，历元差超过这一值，则不进行周跳虚警检测
			double       threshold_LGF_slipcheck;  // 三频相位GF组合周跳虚警检测阈值，历元差超过这一值，则不进行周跳虚警检测
			double       threshold_gap;            // 弧段内的中断间隔阈值, 控制不要出现较大的中断(秒)
			double       max_arclengh;             // 相邻连续弧段间隔秒数阈值, 选择依据是最大连续跟踪弧段的时间, 一般不会超过半个轨道周期(秒)

			// 不需要初始化部分
			int                   index_C1; // 频率1和2在处理时主用, 频率3在进行三频处理时才使用
			int                   index_C2;
			int                   index_C3;
			int                   index_L1;
			int                   index_L2;
			int                   index_L3;
			int                   index_S1;
			int                   index_S2;
			int                   index_S3;
			map<string, PRE_Freq> freqSatList; // 主要针对GLONASS，string类型为卫星名

			PRE_MixedSys()
			{
				type_Freq = TYPE_FREQ_TWO;
				index_C1 = -1;
				index_C2 = -1;
				index_C3 = -1;
				index_L1 = -1;
				index_L2 = -1;
				index_L3 = -1;
				index_S1 = -1;
				index_S2 = -1;
				index_S3 = -1;
				name_C1  = "";
				name_C2  = "";
				name_C3  = "";
				name_L1  = "";
				name_L2  = "";
				name_L3  = "";
				name_S1  = "";
				name_S2  = "";
				name_S3  = "";
				nameFreq_L1 = "";
				nameFreq_L2 = "";
				nameFreq_L3 = "";
				signalPriority_L1 = "";
				signalPriority_L2 = "";
				signalPriority_L3 = "";

				on_L1_L2_SlipDetect     = true;
				on_IF_SlipDetect        = true; // 默认使用 A_IF 组合探测 N1,N2 等大小的周跳
				bOnTripleFreqsCSD       = false; // 三频周跳探测开关，默认为 关 
				min_Elevation           = 7;	// 单位：度，2016/3/3, 鞠冰修改, (GAMIT软件中此阈值取为 10，2013/05/27)
				max_L1_L2_Difference    = 0.03; // 单位：米，默认取 3 cm
				max_L1_L2_SlipSize      = 0.05; // 单位：米，保证比两个频点的波长都小，L1波长19cm，L2波长24cm.（取15cm时,BRFT的PRN1、PRN6存在漏警，2013/6/27）
				max_MW_SlipSize	        = 5;	// 单位：周，此阈值过小在低仰角时容易产生虚警, 假设码噪声sigma=60cm, MW组合噪声为 0.7*sigma/86cm = 0.5 周
				max_IF_SlipSize         = 50;	// 单位：米，N1,N2同时发生1周的周跳A_IF组合跳跃10.7cm(窄巷组合波长c/(f1+f2))，假设P_IF组合精度为1m，探测阈值不宜过小，取10m（约100周）
				width_L1_L2_SubSecFit   = 600;  // 单位：秒
				order_L1_L2_SubSecFit   = 3;	// 多项式阶数，默认采用2次多项式拟合
				exten_L1_L2_SubSecFit   = 0;	// 单位：秒
				count_GrossPoint_PreDel = 3;	// 拟合前剔除超差点的个数，默认3个
				max_Ionosphere		    =  200; // （alpha - 1) * I，有些卫星电离层延迟较大 60m+ (2016/03/30)
				min_Ionosphere		    = -200; // （alpha - 1) * I，一般电离层延迟负值较少且数值较小, 但YEBE测站2012/1/1数据显示,电离层组合负值较大(DCB引起)
				max_thrfrecodecom        = 1.0e5;  // XIA1站的三频伪距组合值达到了1e4量级，其余站均小于200
				threshold_slipsize_PLGIF = 3.0;   
				threshold_slipsize_LGIF  = 0.03; 
				threshold_slipsize_LGF   = 0.20; 
				threshold_LGIF_slipcheck = 0.05;   //历元差绝对值小于该阈值则检验是判断出的周跳是否为虚警
				threshold_LGF_slipcheck  = 0.5;	   //历元差绝对值小于该阈值则检验是判断出的周跳是否为虚警	
				threshold_gap            =  600;
				max_arclengh             =  3600;
			}

			void InitSignalPriority();
		};

		struct PRE_DEF
		{
			double		                 obsEpochInterval;    // 数据采样间隔, 用于处理高频采样数据,鞠冰, 2015/11/27
			size_t                       min_ArcPointCount;   // 最小连续点个数, 个数小于 min_ArcPointCount 的弧段将被删除
			double                       max_ArcInterval;     // 历元间隔阈值, 超出此阈值认为是新弧段(单位：秒)
			double			             vondrak_PIF_eps;     // vondrak滤波参数:光滑因子
			double                       vondrak_PIF_max;     // vondrak滤波参数
			double			             vondrak_PIF_min;     // vondrak滤波参数
			int 			             vondrak_PIF_width;   // vondrak滤波参数
			bool                         on_OutputTempFile;

			PRE_DEF()
			{
				// 与系统无关的预定义部分
				obsEpochInterval  = 30.0;	// 默认Rinex文件采样间隔为30s
				min_ArcPointCount = 20;
				max_ArcInterval	  = 180;    // 2016/3/3，鞠冰修改为3分钟，Bernese 5.0 中 arc 与 arc 的间隔至少为 180 秒；以前采用1800s, 实测数据测试当前阈值可靠(2013-05-21)
				vondrak_PIF_eps	  = 1.0E-14;// 此参数越小拟合曲线越光滑，但取值不能过小，否则方程会病态, 鞠 冰, 2016/3/3
				vondrak_PIF_max	  = 1.5;    // 测试......通过
				vondrak_PIF_min	  = 0.5;    // 测试......通过
				vondrak_PIF_width = 100;    // 测试......通过
				on_OutputTempFile = false;
			}
		};

		class GNSSObsPreproc
		{
		public:
			GNSSObsPreproc(void);
		public:
			~GNSSObsPreproc(void);
		private:
			bool RobustPolyFit_L1_L2(PRE_MixedSys &preMixedSys, double x[], double y[], int n, int offset, int n_out, double y_fit[], double N0, int m = 3); // 主要使用此函数，2016/2/23，supice
		public:
			BYTE obsPreprocInfo2EditedMark1(int obsPreprocInfo);
			BYTE obsPreprocInfo2EditedMark2(int obsPreprocInfo);
			bool getEditedObsEpochList(vector<Rinex3_03_EditedObsEpoch> &editedObsEpochList);
			bool getEditedObsSatList(vector<Rinex3_03_EditedObsSat> &editedObsSatList);
			bool getEditedObsFile(Rinex3_03_EditedObsFile &editedObsFile);
			bool datalist_epoch2sat(vector<Rinex3_03_EditedObsEpoch> editedObsEpochList, vector<Rinex3_03_EditedObsSat> &editedObsSatList);
			bool datalist_sat2epoch(vector<Rinex3_03_EditedObsSat> editedObsSatList, vector<Rinex3_03_EditedObsEpoch> &editedObsEpochList);
			bool detectCodeOutlier_ionosphere(PRE_MixedSys &preMixedSys, DayTime t0, Rinex3_03_EditedObsSat& editedObsSat);
			bool detectPhaseSlip_MW(PRE_MixedSys &preMixedSys, DayTime t0, Rinex3_03_EditedObsSat& editedObsSat);
			bool detectPhaseSlip_suice(PRE_MixedSys &preMixedSys, DayTime t0, Rinex3_03_EditedObsSat& editedObsSat);
			// 三频数据处理
			bool detectThrFreCodeOutlier(PRE_MixedSys &preMixedSys, DayTime T0, Rinex3_03_EditedObsSat& editedObsSat,bool bOutTempFile = false);	
			bool detectThrFrePhaseSlip(PRE_MixedSys &preMixedSys, DayTime T0, Rinex3_03_EditedObsSat& editedObsSat, bool bOutTempFile = false);
			// 主函数
			bool mainPRE_MixedGNSS(Rinex3_03_EditedObsFile &editedObsFile);
		public:
			PRE_DEF              m_preDefine;
			vector<PRE_MixedSys> m_preMixedSysList;
			SP3File				 m_sp3File;			 // GNSS卫星轨道
			POS3D				 m_posStation;		 // 测站位置坐标 
			Rinex3_03_ObsFile	 m_obsFile;			 // 观测数据
			svnavMixedFile       m_svnavMixedFile;   // 用于获取GLONASS的频率信息
			SinexBiasDCBFile     m_sinexBiasDCBFile; // 多系统DCB文件
			string              m_pathPreFileFolder;
		};
	}
}
