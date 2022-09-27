#pragma once
#include "structDef.hpp"
#include <vector>

using namespace NUDTTK;
namespace NUDTTK
{
	namespace SLR
	{
		// 记录地面激光测站的基本信息, 包括测站的位置速度
		struct StaSscRecord
		{
			unsigned int id;        // 测站在美国宇航局所编站址录中的编号
			string       name;      // 名称
			UTC          t0;        // 坐标参考时刻
			double       x;         // 单位（m）
			double       y;         // 单位（m）
			double       z;         // 单位（m）
			double       vx;        // 单位（m/s）
			double       vy;        // 单位（m/s）
			double       vz;        // 单位（m/s）
		};

		struct SscLine
		{
			char     szStationFlag[10]; // '10002S001 '
			char     szName[18];        // 'GRASSE            '
			char     szUse[4];          // 'SLR '
			int      id;                // '7835 '
			double   x;
			double   y;
			double   z;
			double   sigma_x;
			double   sigma_y;
			double   sigma_z;
		};

		struct  StaSscEnsemble
		{
			int      id;
			string   name;
			POS6D    posvel;
			POS6D    sigma;
		};

		class StaSscFile
		{
		public:
			StaSscFile(void);
		public:
			~StaSscFile(void);
		public:
			int  isValidSscLine(string strLine, FILE * pSscFile = NULL);
			bool readSscLine(string strLine, SscLine& line);
			bool readSscLine_14(string strLine, SscLine& line);
			void setEpoch(double epoch);
			bool getStaPosVel(UTC t, int id, POS6D& posvel);
			bool getStaSscList(vector<StaSscRecord> &staSscList);
			bool open(string  strSscFileName);
			bool open_14(string  strSscFileName);
		public:
			double                 m_Epoch;
			vector<StaSscEnsemble> m_data;

		};
	}
}
