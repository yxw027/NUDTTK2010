#pragma once
#include "AntPCVFile.hpp"

// 
using namespace NUDTTK;

namespace NUDTTK
{
	namespace SpaceborneGPSPod
	{
		struct O_CResLine
		{
			double   t;
			double   Elevation;        
			double   Azimuth;
			double   res;
		};

		class AntPCVFromResStat
		{
		public:
			AntPCVFromResStat(void);
		public:
			~AntPCVFromResStat(void);
			bool isValidEpochLine(string strLine, O_CResLine& ocResLine);
			bool addResFile(string  ocResFileName);
			bool generateOcRmsByElevation(double spanElevation = 5, double maxElevation = 90);
			bool generateMeanAntPCVFile(AntPCVFile &pcvMeanFile, double spanElevation = 5, double spanAzimuth = 5, double maxElevation = 90, double maxAzimuth = 360);
			bool generateAntPCVFile(AntPCVFile &pcvMeanFile, AntPCVFile &pcvStdFile, AntPCVFile &pcvNumFile, double spanElevation = 5, double spanAzimuth = 5, double maxElevation = 90, double maxAzimuth = 360);
		public:
			vector<O_CResLine> m_data;
		};
	}
}
