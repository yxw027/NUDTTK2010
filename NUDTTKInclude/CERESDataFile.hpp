#pragma once
#include "structDef.hpp"
#include "constDef.hpp"
#include <string>
#include <vector>

// open reflectivity、emissvity CERES data file. 2017.05.09, sk
// 地球辐射模型，可见光反射率、红外发射率
namespace NUDTTK
{
	namespace Geodyn
	{
		class CERESDataFile
		{
		public:
			CERESDataFile(void);
		public:
			~CERESDataFile(void);
		public:
			void  clear();
			bool  isValidEpochLine(string strLine, vector<double> &dataList);
			bool  open(string  strCERESDataName);
			bool  write(string  strCERESDataName);
		public:
			vector<double> m_listLatitude;           // 纬度
			vector<double> m_listLongitude;;         // 经度
			double**       m_data; 
		};
	}
}
