#pragma once
#include "structDef.hpp"
#include "constDef.hpp"
#include "MathAlgorithm.hpp"
#include "Rinex3_03_EditedObsFile.hpp"
#include "GNSSObsPreproc.hpp"
#include <direct.h>
using namespace NUDTTK;
using namespace NUDTTK::Math;

namespace NUDTTK
{
	namespace GPSPreproc
	{
		struct QE_Res
		{
			DayTime  t;
			float    Elevation; 
			float    Azimuth;    
			float    res;
			float    SNR;      // 信噪比
			int      id_Epoch; // 当前仰角的数据位于仰角列表中的位置， 2013/9/26			
			
			QE_Res()
			{
				Azimuth   = 0.0f;
				Elevation = 0.0f;
				res       = 0.0f;
				SNR       = 0.0f;
				id_Epoch  = 0;				
			}
		};

		struct QE_Arc
		{
			vector<QE_Res> resList;
			float          rms;
			int            countEpochs;   // 有效历元个数
			int            countSlips;    // 周跳总数

			QE_Arc()
			{
				rms = 0.0f;
				countEpochs = 0;
				countSlips = 0;
			}
		};

		struct QE_Sat
		{
			string                 satName;
			vector<QE_Arc>         arcList;
			float                  rms;
			int                    countEpochs;   // 有效历元个数
			int                    countSlips;    // 周跳总数

			QE_Sat()
			{
				rms = 0.0f;
				countEpochs = 0;
				countSlips = 0;
			}
		};

		struct QE_SlipInfo                                  
		{
			string                 satName;
			DayTime                t;       // 周跳时间
			int                    preInfo; // 周跳探测预处理信息		
		};

		struct QE_Elevation
		{
			float                  e0;         // 起始仰角
			float                  e1;         // 终止仰角
			int                    count_P1;   // 统计伪距多径时使用的伪距历元个数
			int                    count_P2;   // 统计伪距多径时使用的伪距历元个数
			int                    count_P3;   // 统计伪距多径时使用的伪距历元个数
			int                    count_L;    // 统计相位噪声时使用的相位历元个数
			float                  rms_P1;   
			float                  rms_P2;
			float                  rms_P3;
			float                  rms_L;
			float                  meanS1;     // 第一个频点平均信噪比
			float                  meanS2;     // 第二个频点平均信噪比
			float                  meanS3;     // 第三个频点平均信噪比
			int                    countSlips; // 周跳个数

			QE_Elevation()
			{
				count_P1   = 0;
				count_P2   = 0;
				count_P3   = 0;
				count_L    = 0;
				rms_P1     = 0.0f;
				rms_P2     = 0.0f;
				rms_P3     = 0.0f;
				rms_L      = 0.0f;
				meanS1     = 0.0f;  
				meanS2     = 0.0f; 
				meanS3     = 0.0f; 
				countSlips = 0;
			}
		};

		struct QE_MixedSys
		{
			// 需要初始化部分
			char                  cSys;        // 系统标识
			TYPE_FREQ_ID          type_Freq;   // 双频/三频周跳探测的开关
			string                name_C1;     // 伪码
			string                name_C2;
			string                name_C3;
			string                name_L1;     // 相位
			string                name_L2;
			string                name_L3;
		    string                name_S1;     // 信噪比 
			string                name_S2;
			string                name_S3;
			string                nameFreq_L1;         // 频率名称"G01"，主要用于PCV修正
			string                nameFreq_L2;   
			string                nameFreq_L3;
			PRE_Freq              freqSys;     // GLONASS的频率信息存储在freqSatList中, 程序里自动赋值
			double			      vondrak_LIF_eps;     // vondrak滤波参数:光滑因子
			double                vondrak_LIF_max;     // vondrak滤波参数
			double			      vondrak_LIF_min;     // vondrak滤波参数
			int 			      vondrak_LIF_width;   // vondrak滤波参数

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

			// 评估生成部分
			map<string, QE_Sat>       satInfoList_P1;
			map<string, QE_Sat>       satInfoList_P2;
			map<string, QE_Sat>       satInfoList_P3;
			map<string, QE_Sat>       satInfoList_L;
			map<string, QE_SlipInfo>  satInfoList_Slip;  
			map<BYTE, int>            satInfoList_Abnormal; // 异常信息列表
			vector<QE_Elevation>      satInfoList_Elevation; 
			int                   countEpochs;   // 历元个数
			int                   countSlips;
			int                   count_P1;   
			int                   count_P2;   
			int                   count_P3;   
			int                   count_L; 
			double                ratio_P_normal;			 // 伪码数据正常比率
			double                ratio_L_normal;			 // 相位数据正常比率
			double                ratio_cycleslip;		 // 周跳比率
			double                thrfre_coefficient;   // 三频组合系数 gama
			float                 meanCount_Sat; // 平均可视卫星数
			float                 rms_P1;
			float                 rms_P2;
			float                 rms_P3;
			float                 rms_L;

			void InitSatInfoList_Elevation(float Interval = 5.0f);
			int  getIndex_Elevation(float Elevation);

			QE_MixedSys()
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
				vondrak_LIF_eps	  = 1.0E-10; 
				vondrak_LIF_max	  = 0.5;    
				vondrak_LIF_min	  = 0.05;    
				vondrak_LIF_width = 100;  
				thrfre_coefficient = 1.0;
			}
		};

		class GNSSObsQualityEvaluate
		{
		public:
			GNSSObsQualityEvaluate(void);
		public:
			~GNSSObsQualityEvaluate(void);
		private:
			string  m_rootPathQE;
			string  m_staName;
			vector<Rinex3_03_EditedObsSat> m_editedObsSatList;
		public:	
			void setRootPath(string strRootPath, string staName = "");
			bool mainQE_MixedGNSS(string strEditedObsFilePath);
			bool getQEInfo(QE_MixedSys &qe_MixedSys);
		private:
			bool QE_C_Multipath(QE_MixedSys &qe_MixedSys);
			bool QE_L_poly(QE_MixedSys &qe_MixedSys, double poly_halfwidth, int poly_order);
			bool QE_L_Vondrak(QE_MixedSys &qe_MixedSys);
			bool QE_C_L_thrfre(QE_MixedSys &qe_MixedSys);
		public:	
			vector<QE_MixedSys>   m_qeMixedSysList;
			svnavMixedFile       m_svnavMixedFile;   // 用于获取GLONASS的频率信息
		};
	}
}
