#pragma once
#include "structDef.hpp"
#include <vector>

using namespace NUDTTK;
namespace NUDTTK
{
	namespace SLR
	{
		// 记录地面激光测站的基本信息, 包括测站的位置速度
		struct StaSscRecord_14
		{
			unsigned int id;        // 测站在美国宇航局所编站址录中的编号
			string       name;      // 名称
			UTC          t;         // 坐标参考时刻
			UTC          t0;        // 起始时刻
			UTC          t1;        // 终止时刻
			double       x;         // 单位（m）
			double       y;         // 单位（m）
			double       z;         // 单位（m）
			double       vx;        // 单位（m/s）
			double       vy;        // 单位（m/s）
			double       vz;        // 单位（m/s）
		};

		struct SscLine_14
		{
			char     szStationFlag[10]; // '10002S001 '
			char     szName[18];        // 'GRASSE            '
			char     szUse[4];          // 'SLR '
			int      id;                // '7835 '
			UTC      t0;
			UTC      t1;
			//int      sat_id;            // 序号
			double   x;
			double   y;
			double   z;
			double   sigma_x;
			double   sigma_y;
			double   sigma_z;
		};

		struct  StaSscLine
		{
			string       name;      // 名称
			int      id;
			UTC      t0;
			UTC      t1;
			POS6D    posvel;
			POS6D    sigma;
		};
		struct StaSscEnsemble_14
		{
			int                   id;
			vector<StaSscLine>    sscLineList;
		};

		class StaSscFile_14
		{
		public:
			StaSscFile_14(void);
		public:
			~StaSscFile_14(void);
		public:
			UTC  getTime(int year, int day, int second);
			int  isValidSscLine(string strLine, FILE * pSscFile = NULL);
			bool readSscLine(string strLine, SscLine_14& line);
			void setEpoch(double epoch);
			bool getStaPosVel(UTC t, int id, POS6D& posvel);
			bool getStaSscList(vector<StaSscRecord_14> &staSscList);
			bool open(string  strSscFileName);
		public:
			double                    m_Epoch;
			vector<StaSscEnsemble_14> m_data;

		};
	}
}
