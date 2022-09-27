#pragma once
#include "constDef.hpp"
#include "structDef.hpp"
#include "TimeCoordConvert.hpp"
#include "Matrix.hpp"
#include "MathAlgorithm.hpp"
#include <vector>
#include <string>
#include <map>

//  Copyright 2012, The National University of Defense Technology at ChangSha
using namespace std;
namespace NUDTTK
{
	struct AntexFile_HeaderLabel
	{
		static const char szAntexVerSyst[];
		static const char szPCVTypeRefAnt[];
		static const char szComment[];
		static const char szEndOfHead[];
		static const char szStartOfAnt[];
		static const char szTypeSerialNo[];
		static const char szMethByDate[];
		static const char szDAZI[];
		static const char szZen1Zen2DZen[];
		static const char szOfFreqs[];
		static const char szValidFrom[];
		static const char szValidUntil[];
		static const char szSinexCode[];
		static const char szStartOfFreq[];
		static const char szNEU[];
		static const char szEndOfFreq[];
		static const char szStartOfFreqRms[];
		static const char szEndOfFreqRms[];
		static const char szEndOfAnt[];
		static const char szNOAZI[];
	};

	struct AntexFile_Header
	{
		double	AntexVersion;					// 文件版本，F8.1,12X
		char	SatSysytem;						// 卫星导航系统名称，A1,39X
		char	PCVType;						// PCV类型，A1,19X
		char	szRefAntType[20 + 1];			// 参考天线类型，A20
		char	szRefAntNumber[20 + 1];			// 参考天线序列号，A20
		vector<string> pstrCommentList;         // 注释行序列，A60 * m 行
		AntexFile_Header()
		{
			memset(this,0,sizeof(AntexFile_Header));
		}
	};
	
	struct AntCorrectionBlk
	{
		bool		flagAntType;					// '0'-卫星，'1'-接收机
		char		AntType[20 + 1];				// 天线类型，A20
		char		sNN[3 + 1];						// 卫星系统 + Number，A20(如：G01表示： GPS + PRN01)
		char		sNNN[4 + 1];					// 卫星系统 + Number，A10(如：G032表示：GPS + SVN032)
		char		COSPARId[10 + 1];				// YYYY-XXXA，A10(YYYY-卫星入轨年份，XXX-运载火箭序号，A-alpha序号)
		char		Method[20 + 1];					// 校准方法，A20
		char		Agency[20 + 1];					// 机构名称，A20
		int			IdvAntNum;						// 单独校准的天线数目(有待确认2013-04-05)，I6,4X
		char		Date[10 + 1];					// 产品日期，A10
		double		DAZI;							// 方位角等分间隔，单位：度，2X，F6.1，52X
		double		ZEN1;							// 天顶角1，单位：度，F6.1
		double		ZEN2;							// 天顶角2，单位：度，F6.1
		double		DZEN;							// 天顶角等分间隔，单位：度，F6.1
		int			FreqNum;						// 频点数，I6,54X
		DayTime		ValidFrom;						// 数据可用期的起始时刻(GPST)，5I6，F13.7，17X
		DayTime		ValidUntil;						// 数据可用期的结束时刻(GPST)，5I6，F13.7，17X
		char		SinexCode[10 + 1];				// SINEX文件格式中天线校准模型名称，A10，50X
		vector<double>  zenithList;					// 天顶角列表(2013-04-17 添加)
		vector<double>  azimuthList;				// 方位角列表(2013-04-17 添加)
		vector<string>	pstrCommentList;			// 注释行序列，A60 * m 行
		vector<string>	flagFreqList;				// GPS：'G01' - L1          
		vector<POS3D>	PCOList;					// Phase Center Offset，单位：mm
		vector<POS3D>	RmsPCOList;					// Rms of Phase Center Offset，单位：mm
		vector<Matrix>	NOAZI_PCVList;				// Phase Center Variation(不依赖于方位角)，单位：mm
		vector<Matrix>	RmsNOAZI_PCVList;			// Rms of Phase Center Variation(不依赖于方位角)，单位：mm
		vector<Matrix>	AZIDEPT_PCVList;			// Phase Center Variation(依赖于方位角),单位：mm
		vector<Matrix>	RmsAZIDEPT_PCVList;			// Rms of Phase Center Variation(依赖于方位角),单位：mm
		AntCorrectionBlk()
		{
			//memset(this,0,sizeof(AntCorrectionBlk));
			ValidFrom   = DayTime(1980,1,1,0,0,0.0);	// 默认数据有效期起始
			ValidUntil  = DayTime(2500,1,1,0,0,0.0);	// 默认数据有效期结束
		}
	};

	typedef	map<string, AntCorrectionBlk>	RecAntCorrectionMap;	// 接收机天线相位中心修正数据列表，按天线类型索引
	
	typedef	vector<AntCorrectionBlk>		SatAntCorrectBlkList;	// 同一编号卫星、不同时段的天线修正数据列表
	typedef map<string, SatAntCorrectBlkList>	SatAntCorrectionMap;// 卫星天线相位中心修正数据列表，按卫星号索引

	class igs05atxFile
	{
	public:
		igs05atxFile(void);
	public:
		~igs05atxFile(void);
		bool  open(string  strAtxFileName);
		bool  write(string strAtxFileName);
		bool  write();
        bool  getAntCorrectBlk(string index_Name, GPST t, AntCorrectionBlk &datablk, bool flag_Sat = true);
		double correctSatAntPCOPCV(AntCorrectionBlk datablk, int FreqId, POS3D recPos, POS3D satPos, POS3D sunPos, bool bOn_PCV = true);
		double correctSatAntPCOPCV_YawFixed(AntCorrectionBlk datablk, int FreqId, POS3D vecLOS, POS3D ex, POS3D ey, POS3D ez, bool bOn_PCV = true);
		double correctSatAntPCOPCV_GYM95(AntCorrectionBlk datablk, int FreqId, POS3D vecLOS, POS3D ex, POS3D ey, POS3D ez, bool bOn_PCV = true);
		double correctRecAntPCOPCV(AntCorrectionBlk datablk, int FreqId, POS3D recPos, POS3D satPos, bool bOn_PCV = true);
		double correctRecAntPCOPCV(AntCorrectionBlk datablk, int FreqId, POS3D vecLOS, bool bOn_PCV = true);

		double correctSatAntPCOPCV_GYM95(AntCorrectionBlk datablk, string nameFreq, POS3D vecLOS, POS3D ex, POS3D ey, POS3D ez, bool bOn_PCV = true);
		double correctRecAntPCOPCV(AntCorrectionBlk datablk, string nameFreq, POS3D recPos, POS3D satPos, bool bOn_PCV = true);
	public:
		AntexFile_Header		m_header;
		SatAntCorrectionMap		m_satdata;
		RecAntCorrectionMap		m_recdata;
	};
}