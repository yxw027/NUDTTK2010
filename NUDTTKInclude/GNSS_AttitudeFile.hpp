#pragma once
#include "structDef.hpp"
#include <vector>
#include <windows.h>
#include <map>
#include <limits>
#include "Matrix.hpp"
//  Copyright 2021, Sysu at zhuhai
//  COD提供的姿态数据为 ECEF->SAT. BODY FRAME
//  COORD_SYSTEM        IGS14
//  FRAME_TYPE          ECEF
//  TIME_SYSTEM         GPS
using namespace std;
namespace NUDTTK
{
	// 姿态文件头部分
	struct AttHeader
	{
		char           szTimeSystem[4+1];   // 第9行，时间系统
		GPST           tmStart;             // 第10行，文件起始时间
		GPST           tmEnd;               // 第11行，文件结束时间
		double         EpochSpan;           // 第12行，EPOCH_INTERVAL
		char           szCoordinateSys[5+1];// 第13行，Coord_system
		char           szFrameType[4+1];    // 第14行，Frame_type
		char           szOrbit_Type[4+1];   // 第15行
		vector<string> pstrSatNameList;     // 卫星列表
		AttHeader()
		{
			 pstrSatNameList.clear();
		}
		~AttHeader()
		{
		}	
	};

	// 姿态数据部分
	typedef map<string, ATT_Q4> AttSatMap;      // 单个时刻，不同卫星姿态数据列表

	struct AttEpoch
	{
		GPST t;
		AttSatMap attMap;
	};

	class GNSS_AttFile
	{
	public:
		GNSS_AttFile(void);
	public:
		~GNSS_AttFile(void);
	public:
		static int getSatPRN(string strSatName);
	public:
		void   clear();
		bool   isEmpty();
		int    isValidEpochLine(string strLine, FILE *pAttFILE = NULL);
		double getEpochSpan();
		bool   open(string strAttFileName);
		bool   write(string strAttFileName);
		bool   getQ4(GPST t, string name, ATT_Q4 &Q4, int nlagrange = 9);
		bool   getAttMatrix(GPST t, string name, Matrix& matATT, int nlagrange = 9);//时间间隔长一个点，插值阶数就得高。LEO的我看4阶，GNSS的9阶
	public:
		AttHeader        m_header;
		vector<AttEpoch> m_data;

	};
}