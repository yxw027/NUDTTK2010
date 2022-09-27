#pragma once
#include "structDef.hpp"
#include <vector>

using namespace NUDTTK;
namespace NUDTTK
{
	namespace SLR
	{
		// 记录激光测站的偏心数据
		struct StaEccRecord
		{
			unsigned int id;   // 测站在美国宇航局所编站址录中的编号
			UTC          t0;   // 起始时刻
			UTC          t1;   // 终止时刻
			ENU          ecc;
		};

		struct EccLine
		{
			int      id;
			char     szPT[3];
			char     szSoln[5];
			char     szT[2];
			UTC      t0;
			UTC      t1;
			char     szUNE[4];
			ENU      ecc;
			char     szCDP[9];
		};

		struct StaEccEnsemble
		{
			int                id;
			vector<EccLine>    eccLineList;
		};

		// 激光测站的偏心数据文件, 由谷德峰创建
		class StaEccFile
		{
		public:
			StaEccFile(void);
		public:
			~StaEccFile(void);
		public:
			UTC  getTime(int year, int day, int second);
			bool readEccLine(string strLine, EccLine& line);
			bool open(string strEccFileName);
			bool getStaEccRecordList(vector<StaEccRecord> &staEccList);
		public:
			vector<StaEccEnsemble> m_data;
		};
	}
}
