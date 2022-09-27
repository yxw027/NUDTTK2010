#pragma once
#include "structDef.hpp"
#include "constDef.hpp"
#include "MathAlgorithm.hpp"
#include "Rinex2_1_EditedObsFile.hpp"
#include <direct.h>
using namespace NUDTTK;
using namespace NUDTTK::Math;
// 模块名称：GPSObsQualityEvaluate
// 模块功能：实现GPS观测数据的质量评估
// 功能说明：1、伪码数据质量评估，多径组合
//           2、相位数据质量评估，Vondrak 滤波拟合 L1-L2
//           3、可视卫星数目评估
// 语言：C++
// 创建者：鞠 冰
// 创建时间：2012/9/12
// 版本时间：2012/9/12
// 修改记录：1、2013/6/14 清理优化质量评估程序代码
// 备注： 
namespace NUDTTK
{
	namespace GPSPreproc
	{
		// 历元信息
		struct QERESIDUAL
		{
			DayTime  t;
			double   Azimuth;    
			double   Elevation; 
			double   obs;
			double   fit;
			double   res;
			int      id_elevation_Inter; // 当前仰角的数据位于仰角列表中的位置， 2013/9/26
			QERESIDUAL()
			{
				Azimuth   = 0;
				Elevation = 0;
				res       = 0;	
				id_elevation_Inter = 0;
			}
		};
		// 弧段信息
		struct QERESIDUAL_ARC
		{
			vector<QERESIDUAL> resList;
			double rms_arc;
		};
		// 卫星信息
		struct QERESIDUAL_SAT
		{
			vector<QERESIDUAL_ARC> arcList;
			BYTE                   id_sat;
			double                 rms_sat;
			int                    epochs;   // 有效历元个数，为定轨数据选择提供依据
			int                    slips;    // 周跳总数
			QERESIDUAL_SAT()
			{
				rms_sat = 0;
				epochs  = 0;
				slips   = 0;
			}
		};

		typedef map<BYTE, int>	QEAbNormalObsCountMap;	// 异常信息列表
		struct Q_ELE_STATION
		{// 观测质量与仰角的关系， 2013/9/26
			double                 e0;       // 起始仰角
			double                 e1;       // 终止仰角
			int                    epochs_P; // 统计伪距多径时使用的伪距历元个数
			int                    epochs_L; // 统计相位多径(噪声)时使用的相位历元个数
			double                 rms_P1;   
			double                 rms_P2;
			double                 rms_P5;
			double                 rms_L;
			int                    slips;    // 周跳个数
			double                 CN_L1;    // 第一个频点信噪比
			double                 CN_L2;    // 第二个频点信噪比
			double                 CN_L5;    // 第三个频点信噪比
			Q_ELE_STATION()
			{
				epochs_P = 0;
				epochs_L = 0;
				rms_P1 = 0;
				rms_P2 = 0;
				rms_P5 = 0;
				rms_L  = 0;
				slips  = 0;
				CN_L1  = 0;  
				CN_L2  = 0; 
				CN_L5  = 0; 
			}
		};
		struct CycleSlip_Info                                  
		{  // 周跳的详细信息, 2013/9/26
			DayTime                t;                             // 周跳时间
			BYTE                   preproc_info;                  // 探测出周跳的方法
			BYTE                   id_sat;                        // 周跳卫星id				
		};

		// 测站信息
		struct QERESIDUAL_STATION
		{
			vector<QERESIDUAL_SAT> satInfoList_P1;		// P1 统计信息列表(QEcode)
			vector<QERESIDUAL_SAT> satInfoList_P2;		// P2 统计信息列表(QEcode)
			vector<QERESIDUAL_SAT> satInfoList_L;		// L1-L2 统计信息列表(QEphase)
			vector<Q_ELE_STATION>  satInfolist_ele;        // 2013/9/26
			vector<CycleSlip_Info> satInforlist_CycleSlip; // 2013/9/26
			double		rms_P1;							// P1 评估精度(QEcode)
			double      rms_P2;							// P2 评估精度(QEcode)
			double      rms_L;							// L1-L2 评估精度(QEphase)
			double      ratio_P_normal;					// 伪码数据正常比率(getQEInfo)
			double      ratio_L_normal;					// 相位数据正常比率(getQEInfo)
			double      ratio_cycleslip;				// 周跳比率(getQEInfo)
			int			max_ObsSatNum;				    // 最大可视卫星数(QESatCounts)
			double		mean_ObsSatNum;				    // 平均可视卫星数(QESatCounts)
			QEAbNormalObsCountMap  AbnormalObsCount;	// 异常信息列表(getQEInfo)
			void init(double elevation_Inter = 5.0);    // 默认的仰角间隔为5度,2013/9/26
			int  getInterval(double elevation);         // 确认仰角在仰角列表中的位置,2013/9/26
			QERESIDUAL_STATION()
			{				
				max_ObsSatNum   = 0;
				mean_ObsSatNum  = 0;
				rms_P1          = 0;
				rms_P2          = 0;				
				rms_L           = 0;
				ratio_P_normal  = 0;
				ratio_L_normal  = 0;
				ratio_cycleslip = 0;
			}
		};

		// GPS 观测数据质量评估类
		class GPSObsQualityEvaluate
		{
		public:
			GPSObsQualityEvaluate();
		public:
			~GPSObsQualityEvaluate();
		private:
			string  m_strOQEResPath;
			string  m_strStaName;
		public:	
			void setOQEResPath(string strOQEResPath, string strStaName = "");
			bool mainFuncObsQE(string  strEditedObsfilePath, int Freq1, int Freq2); // 鞠 冰，增加Freq1、Freq2, 便于北斗选择频率, 2016/3/24 
			bool getQEInfo(vector<Rinex2_1_EditedObsSat>  editedObsSatlist, int index_P1, int index_P2, int index_L1, int index_L2);
		private:
			bool QEcode_multipath(vector<Rinex2_1_EditedObsSat> editedObsSatlist, int index_P1, int index_P2, int index_L1, int index_L2, double frequence_L1 = GPS_FREQUENCE_L1,double frequence_L2 = GPS_FREQUENCE_L2);
			bool QEphase_vondrak(vector<Rinex2_1_EditedObsSat>  editedObsSatlist, int index_L1, int index_L2, double frequence_L1 = GPS_FREQUENCE_L1,double frequence_L2 = GPS_FREQUENCE_L2, double vondrak_LIF_eps = 1.0E-10, double vondrak_LIF_max = 0.5, double vondrak_LIF_min = 0.05, int vondrak_LIF_width = 100);
			bool QESatCounts(Rinex2_1_EditedObsFile editedObsFile);
		public:
			QERESIDUAL_STATION	m_QEInfo;

		//鞠 冰，供研究使用, 2016/3/24
		public:
			bool getMP1MP2(GPST t0);
		public:
			string              m_OQEFilePath; // 存储目录, 供getMP1MP2使用
			POS3D               m_staPos;      // 存储测站位置信息
		};
	}
}