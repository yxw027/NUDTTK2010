#pragma once
#include "constDef.hpp"
#include "structDef.hpp"
#include "Rinex2_1_ObsFile.hpp"
#include "TimeCoordConvert.hpp"
#include <vector>

using namespace NUDTTK;
namespace NUDTTK
{
	struct   BDRawObsFileline     //北斗实验数据行结构，每行37列,其中Q支路无数据  
	{
		double    BDT_SECOND;     //(第1列)北斗时的秒
		int       BDT_NAVWN;      //(第2列)北斗时的周数（导航时间系统）
		int       BDT_LOCALWN;    //(第3列)北斗时的周数（当地时间系统）
		int       SAT_ID;         //(第4列)北斗卫星编号，01—GEO，03—GEO，06—IGSO，30—MEO
		double    B1IW;           //(第5列)B1频点I支路宽相关测距值（m）
		double    B1IN;           //(第6列)B1频点I支路窄相关测距值（m）
		double    B1IA;           //(第7列)B1频点I支路抗多径测距值（m）
		double    B1QW;           //(第8列)B1频点Q支路宽相关测距值（m）
		double    B1QN;           //(第9列)B1频点Q支路窄相关测距值（m）
		double    B1QA;           //(第10列)B1频点Q支路抗多径测距值（m）
		double    B2IW;           //(第11列)B2频点I支路宽相关测距值（m）
		double    B2IN;           //(第12列)B2频点I支路窄相关测距值（m）
		double    B2IA;           //(第13列)B2频点I支路抗多径测距值（m）
		double    B2QW;           //(第14列)B2频点Q支路宽相关测距值（m）
		double    B2QN;           //(第15列)B2频点Q支路窄相关测距值（m）
		double    B2QA;           //(第16列)B2频点Q支路抗多径测距值（m）
		double    B3IW;           //(第17列)B3频点I支路宽相关测距值（m）
		double    B3IN;           //(第18列)B3频点I支路窄相关测距值（m）
		double    B3IA;           //(第19列)B3频点I支路抗多径测距值（m）
		double    B3QW;           //(第20列)B3频点Q支路宽相关测距值（m）
		double    B3QN;           //(第21列)B3频点Q支路窄相关测距值（m）
		double    B3QA;           //(第22列)B3频点Q支路抗多径测距值（m）
		double    B1IC;           //(第23列)B1频点I支路相位测量值
		double    B1QC;           //(第24列)B1频点Q支路相位测量值
		double    B2IC;           //(第25列)B2频点I支路相位测量值
		double    B2QC;           //(第26列)B2频点Q支路相位测量值
		double    B3IC;           //(第27列)B3频点I支路相位测量值
		double    B3QC;           //(第28列)B3频点Q支路相位测量值
		double    B1D;            //(第29列)B1频点多普勒测量值
		double    B2D;            //(第30列)B2频点多普勒测量值
		double    B3D;            //(第31列)B3频点多普勒测量值
		double    B1ICN;          //(第32列)B1频点I支路载噪比 
		double    B1QCN;          //(第33列)B1频点Q支路载噪比
		double    B2ICN;          //(第34列)B2频点I支路载噪比 
		double    B2QCN;          //(第35列)B2频点Q支路载噪比
		double    B3ICN;          //(第36列)B3频点I支路载噪比 
		double    B3QCN;          //(第37列)B3频点Q支路载噪比

		BDRawObsFileline()
		{
			memset(this, 0, sizeof(BDRawObsFileline));
		}
		BDT gettime()             
		{				
			BDT    t0(2006, 1, 1, 0, 0, 0);
			return t0 + BDT_NAVWN * 7 * 86400.0 + BDT_SECOND;
		};
		void settime(BDT t)
		{
			BDT    t0(2006, 1, 1, 0, 0, 0);
			BDT_NAVWN   = (int)((t - t0) / (7 * 86400));
			BDT_LOCALWN = BDT_NAVWN;
			BDT_SECOND  = t - t0 - BDT_NAVWN * 7 * 86400.0;
		}
		
	};
	struct  BDRawObsFiledata                      //北斗实验数据文件中没有直接包含测站信息，为了今后处理方便，增加了测站信息
	{
		char               STA_ID[4+1];           //测站名称
		BDRawObsFileline   BDrawobsfileline;      //北斗实验数据行结构
		BDRawObsFiledata()
		{
			memset(this, 0, sizeof(BDRawObsFiledata));
		}
	};	
	class  BDRawObsFile
	{
	public:
		BDRawObsFile(void);
	public:
		~BDRawObsFile(void);
	public:
		bool    isEmpty();	
		bool    isValidEpochLine(string strLine, BDRawObsFileline& line); 
		bool    open(string  strBDRawObsFileName);
		bool    write(string  strBDRawObsFileName);
		bool    bdRaw_To_Rinex2_1(Rinex2_1_ObsFile &rinexObsFile, int interval = 30);

	public:
		vector<BDRawObsFiledata>  m_data;
		char                      BDRawObsFileHeader[700];   //用于储存北斗实验数据的第一行		
	};
	
}
