#pragma once
#include "Rinex2_1_ObsFile.hpp"
#include "Rinex2_1_EditedObsFile.hpp"
#include "Rinex2_1_MixedEditedObsFile.hpp"
#include "Rinex2_1_NavFile.hpp"
#include "MathAlgorithm.hpp"
#include "SP3File.hpp"
#include "structDef.hpp"
#include "TimeCoordConvert.hpp"
#include "Troposphere_model.hpp"
#include <direct.h>
using namespace NUDTTK;
using namespace NUDTTK::Math;
// 模块名称：GPSObsPreproc
// 模块功能：实现GPS观测数据的预处理
// 功能说明：1、GPS数据预处理包括野值剔除和周跳探测
//           2、主要采用时间序列方法对伪码及相位数据做预处理
//           3、对编辑数据文件进行降采样处理及对流层延迟修正
// 语言：C++
// 创建者：鞠 冰
// 创建时间：2012/9/12
// 版本时间：2012/9/12
// 修改记录：1、调整相位数据预处理策略，综合利用 M-W 组合、L_IF 组合和 L1-L2 组合，2013/5/20
//			 2、添加成员变量 strObsFilePath 便于提取数据文件路径，用于存储预处理结果，2013/9/13
// 备注： 
namespace NUDTTK
{
	namespace GPSPreproc
	{
		// 定义预处理方法中涉及到的阈值及参数结构体
		struct GPSObsPreprocParam
		{
			double		    interval;                   // 数据采样间隔，用于处理高频采样数据，supice, 2015-11-27
			double			max_ThredIonosphere;		// 伪码电离层组合阈值（最大值）
			double			min_ThredIonosphere;		// 伪码电离层组合阈值（最小值）
			unsigned int	min_arcpointcount;			// 最小连续点个数, 个数小于 min_arcpointcount 的弧段将被删除
			double		    max_arclengh;				// 历元间隔阈值，超出此阈值认为是新弧段(单位：秒)
			double			min_elevation;				// 观测仰角阈值,仰角低于 min_elevation 的观测数据不使用
			double			vondrak_PIF_eps;			// Vondrak滤波参数：光滑因子
			double          vondrak_PIF_max;			// Vondrak滤波参数
			double			vondrak_PIF_min;			// Vondrak滤波参数
			int 			vondrak_PIF_width;			// Vondrak滤波参数
			double			threshold_slipsize_mw;		// M-W组合周跳探测阈值(单位：周)
			double			threshold_maxD_L1_L2;		// L1-L2 历元差阈值，小于此阈值的点按数值大小排列，作为备选剔除点，鞠 冰，2016/3/17
			double			threshold_slipsize_L1_L2;	// L1-L2探测周跳阈值(单位：米)
			double			priori_rms_mw;				// M-W组合先验RMS(单位：周)
			int             order_L1_L2;				// 拟合L1-L2的多项式阶数
			double          nwidth_L1_L2;				// L1-L2序列分段拟合的输出窗口宽度(单位：秒)
			double			extendwidth_L1_L2;			// L1_L2序列分段拟合左右断点向外拓展宽度(单位：秒)
			//以下为鞠冰添加A_IF组合进行周跳探测而增加的成员变量  2017/01/04整理
			int				GrossPointNum_PreDelete;	// 预先剔除超差点个数
			bool            bOnTripleFreqsCSD;			// 三频周跳探测开关
			double          threshold_slipsize_IF;		// A_IF组合周跳探测阈值(单位：米)
			bool            bOn_IF;						// 是否使用 A_IF 组合探测周跳，supice,2016/3/3
			bool            bOn_L1_L2_PhaseSlipDetect;	// 是否使用 L1-L2 历元差探测周跳，supice，2016/2/24
			
			GPSObsPreprocParam()
			{
				interval                  =      30.0;	// 默认 Rinex 文件采样间隔为 30 s
				max_ThredIonosphere		  =		  200;	// （alpha - 1)*I，有些卫星电离层延迟较大60m+(2016-03-30)
				min_ThredIonosphere		  =		 -200;	// （alpha - 1)*I，一般电离层延迟负值较少且数值较小,但YEBE测站2012/1/1数据显示，电离层组合负值较大(DCB引起)
				min_arcpointcount		  =		   20;  // 最短弧段要求10min，Bernese 5.0 中 arc 的最少点数为 10(5min)
				max_arclengh			  =		  180;	// 2016/3/3，鞠冰修改为 3min，Bernese 5.0 中 arc 与 arc 的间隔至少为 180 秒；以前采用1800s, 实测数据测试当前阈值可靠(2013-05-21)
				min_elevation             =		    7;	// 单位：度，2016/3/3, 鞠冰修改，(gamit软件中此阈值取为10，2013-05-27)
				vondrak_PIF_eps			  =   1.0E-14;	// 此参数越小拟合曲线越光滑，但取值不能过小，否则方程会病态，鞠 冰，2016/3/3
				vondrak_PIF_max			  =		  1.5;	// 测试......通过
				vondrak_PIF_min			  =		  0.5;	// 测试......通过
				vondrak_PIF_width         =		  100;	// 测试......通过
				threshold_slipsize_mw	  =		    5;	// 单位：周，此阈值过小在低仰角时容易产生虚警, 假设码噪声 sigma = 60cm, MW组合噪声为 0.7*sigma / 86cm = 0.5 周
				threshold_maxD_L1_L2	  =      0.03;	// 单位：米，默认取 3 cm
				threshold_slipsize_L1_L2  =		 0.05;	// 单位：米，保证比两个频点的波长都小，L1波长19cm，L2波长24cm(取 15cm brft_PRN1、PRN6漏警，2013/6/27)
				priori_rms_mw			  =       0.5;	// 单位：周，根据TurboEdit(1990)文献中的叙述选取
				order_L1_L2				  =         3;	// 多项式阶数，默认采用2次多项式拟合
				nwidth_L1_L2              =       600;  // 单位：秒
				extendwidth_L1_L2         =         0;	// 单位：秒
				
				GrossPointNum_PreDelete   =         3;	// 拟合前剔除超差点的个数，默认取3
				bOnTripleFreqsCSD         =     false;
				threshold_slipsize_IF     =        50;	// 单位：米，N1,N2同时发生1周的周跳A_IF组合跳跃10.7cm(窄巷组合波长c/(f1+f2))，假设P_IF组合精度为1m，探测阈值不宜过小，取10m（约100周）
				bOn_IF					  =      true;	// 默认使用 A_IF 组合探测 N1,N2 等大小的周跳
				bOn_L1_L2_PhaseSlipDetect =     false;  // 默认不使用 L1-L2 历元差探测小周跳：一方面电离层有时变化并不平缓(靠近两极地区)，虚警率过高；另一方面实测数据发生小周跳时L1-L2组合并不敏感
			}
		};

		// 定义GPS观测数据预处理类
		class GPSObsPreproc
		{
		public:
			GPSObsPreproc(void);
			GPSObsPreproc(Rinex2_1_ObsFile obsFile, Rinex2_1_NavFile navFile, POS3D posStation, char cSatSystem = 'G', char RecType = 'N');
			GPSObsPreproc(Rinex2_1_ObsFile obsFile, SP3File sp3File, POS3D posStation, char cSatSystem = 'G', char RecType = 'N');
		public:
			~GPSObsPreproc(void);
		public:
			void setObsFile(Rinex2_1_ObsFile obsFile);
			bool loadObsFile(string strObsFileName);
			bool loadNavFile(string strNavFileName);
			bool loadSp3File(string strSp3FileName);
			void setStationPosition(POS3D pos);
			void getPreprocFilePath(string strObsFileName);
			bool getEditedObsEpochList(vector<Rinex2_1_EditedObsEpoch> &editedObsEpochList);
			bool getEditedObsSatList(vector<Rinex2_1_EditedObsSat> &editedObsSatList);
			BYTE obsPreprocInfo2EditedMark1(int obsPreprocInfo);
			BYTE obsPreprocInfo2EditedMark2(int obsPreprocInfo);

			bool detectCodeOutlier_ionosphere(int index_P1, int index_P2, double frequence_P1, double frequence_P2, DayTime T0, Rinex2_1_EditedObsSat& editedObsSat, bool bOutTempFile = false);
			bool detectPhaseSlip_MW(int index_P1, int index_P2, int index_L1, int index_L2, double frequence_L1, double frequence_L2, DayTime T0, Rinex2_1_EditedObsSat& editedObsSat, bool bOutTempFile = false);
			bool detectPhaseSlip(int index_P1, int index_P2, int index_L1, int index_L2, double frequence_L1, double frequence_L2, DayTime T0, Rinex2_1_EditedObsSat& editedObsSat);
			bool detectPhaseSlip_suice(int index_P1, int index_P2, int index_L1, int index_L2, double frequence_L1, double frequence_L2, DayTime T0, Rinex2_1_EditedObsSat& editedObsSat, bool bOutTempFile = false);
			bool detectPhaseSlipTripleFreqs(int index_L1, int index_L2, int index_L5, double frequence_L1, double frequence_L2, double frequence_L5, DayTime T0, Rinex2_1_EditedObsSat& editedObsSat, bool bOutTempFile = false);
			bool mainFuncObsPreproc(Rinex2_1_EditedObsFile &editedObsFile, int Freq1 = 1, int Freq2 = 2, bool bOutTempFile = false);

		private:
			bool RobustPolyFitL1_L2(double x[], double y[], int n, int offset, int n_out, double y_fit[], double N0, int m = 3); // 主要使用此函数，2016/2/23，supice
		public:
			static bool downSampling(POS3D posStaion, Rinex2_1_EditedObsFile editedObsFile, Rinex2_1_EditedObsFile &downSamplingFile, bool bOnTropCorret = true, int nSampleSpan = 120);
			static bool downSampling_new(POS3D posStaion, Rinex2_1_EditedObsFile editedObsFile, Rinex2_1_EditedObsFile &downSamplingFile, bool bOnTropCorret = true, int nSampleSpan = 120);
			//降采样重载，融合系统添加  昌虓 20170921
			static bool downSampling(POS3D posStaion, Rinex2_1_MixedEditedObsFile editedObsFile, Rinex2_1_MixedEditedObsFile &downSamplingFile, bool bOnTropCorret = true, int nSampleSpan = 300);
		public:
			bool datalist_epoch2sat(vector<Rinex2_1_EditedObsEpoch> editedObsEpochList, vector<Rinex2_1_EditedObsSat> &editedObsSatList);
			bool datalist_sat2epoch(vector<Rinex2_1_EditedObsSat> editedObsSatList, vector<Rinex2_1_EditedObsEpoch> &editedObsEpochList);	
		public: 
			GPSObsPreprocParam  m_ObsPreprocParam;
			string              m_strPreprocFilePath;
		private:
			Rinex2_1_ObsFile			m_obsFile;			// 观测数据
			Rinex2_1_NavFile			m_navFile;			// 广播星历数据
			POS3D						m_posStation;		// 测站位置坐标 
			SP3File						m_sp3File;			// 卫星轨道
			char                        m_recType;
			// 兼容北斗系统，鞠 冰，2016/3/3
			char						m_cSatSystem;		
			double                      FREQUENCE_L1;
			double                      FREQUENCE_L2;
			double						FREQUENCE_L5;
		};
	}
}