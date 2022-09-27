#pragma once
#include "structDef.hpp"
#include "constDef.hpp"
#include <string>
#include <vector>

// 卫星微模型参数文件
namespace NUDTTK
{
	namespace Geodyn
	{
		struct SatMacroLine
		{
			char    satName[6];
			int       on_solar;  // 是否为太阳翻版面，1:是，0否
			int       on_nofix;  // 太阳翻版面是否需要调整，1：是，0：否
			double       shape;  // 曲面与平面比例，0：平面，1：曲面
			double        area;
			double           x;
			double           y;
			double           z;
			double   absorpVis;
			double     geomVis;
			double     diffVis;
			double     emissIR;
			double      geomIR;
			double      diffIR;

			SatMacroLine() 
			{
			 on_solar = 0;
		     on_nofix = 0;
				shape = 0.0;
				 area = 0.0;
				    x = 1.0;
				    y = 0.0;
				    z = 0.0;
			absorpVis = 0.0;
              geomVis = 0.0;
              diffVis = 0.0;
              emissIR = 0.0;
               geomIR = 0.0;
			   diffIR = 0.0;
			}
		};

		class SatMacroFile
		{
			public:
				SatMacroFile(void);
			public:
				~SatMacroFile(void);
			public:
				void  clear();
				bool  openSatMacroFile(string strSatMacroFileName, string satName);
			public:
				vector<SatMacroLine> m_data;
		};
	}
}
