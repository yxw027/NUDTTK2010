#include "AntPCVFile.hpp"
#include <sstream>
#include <math.h>
#include "MathAlgorithm.hpp"

using namespace NUDTTK::Math;
namespace NUDTTK
{
	AntPCVFile::AntPCVFile(void)
	{
		m_data = NULL;
	}

	AntPCVFile::~AntPCVFile(void)
	{
		//if(m_data != NULL)
		//{
		//	for(int i = 0; i < int(m_listElevation.size()); i++)
		//	{
		//		delete m_data[i];
		//	}
		//	delete m_data;
		//	m_data = NULL;
		//}
	}

	void AntPCVFile::clear()
	{
		if(m_data != NULL)
		{
			for(int i = 0; i < int(m_listElevation.size()); i++)
			{
				delete m_data[i];
			}
			delete m_data;
			m_data = NULL;
		}
		m_listElevation.clear();
		m_listAzimuth.clear();
	}

	bool AntPCVFile::isValidEpochLine(string strLine, vector<double> &dataList)
	{
		bool bFlag = true;
		dataList.clear();
		istringstream stringstream(strLine);
		while(!stringstream.eof())
		{
			char szValue[100];
			stringstream>>szValue;
			if(strlen(szValue) > 0)
			{
				double value = atof(szValue);
				dataList.push_back(value);
			}
		}
		if(dataList.size() == 0)
			bFlag = false;
		return bFlag;
	}

	bool AntPCVFile::open(string strPCVFileName)
	{
		FILE * pFile = fopen(strPCVFileName.c_str(), "r+t");
		if(pFile == NULL) 
			return false;
		char line[1500];
		int i = 0;
		clear();
		while(!feof(pFile))
		{
			if(fgets(line, 1500, pFile))
			{
				vector<double> dataList;
				if(isValidEpochLine(line, dataList))
				{
					i++;
					if(i == 1)
						m_listElevation = dataList;
					else
					{
						if(m_listElevation.size() == dataList.size() - 1)
							m_listAzimuth.push_back(dataList[0]);
					}
				}
			}
		}
		// 重新将文件指针指向起始位置
		fseek(pFile, 0, SEEK_SET);
		m_data = new double *[m_listAzimuth.size()];
		for(int j = 0; j < int(m_listAzimuth.size()); j++)
			m_data[j] = new double [m_listElevation.size()];
		i = 0;
		while(!feof(pFile))
		{
			if(fgets(line, 1500, pFile))
			{
				vector<double> dataList;
				if(isValidEpochLine(line, dataList))
				{
					i++;
					if(i != 1)
					{
						if(m_listElevation.size() == dataList.size() - 1)
						{
							for(int j = 0; j < int(m_listElevation.size()); j++)
								m_data[i-2][j] = dataList[j+1];
						}
					}
				}
			}
		}
		fclose(pFile);
		return true;
	}

	bool AntPCVFile::write(string strPCVFileName, bool bInt)
	{
		FILE* pFile = fopen(strPCVFileName.c_str(), "w+");
		fprintf(pFile, "%-12s", " ");
		for(size_t s_i = 0; s_i < m_listElevation.size(); s_i++)
			fprintf(pFile, "%12.1f", m_listElevation[s_i]);
		fprintf(pFile, "\n");
        for(size_t s_i = 0; s_i < m_listAzimuth.size(); s_i++)
		{
			fprintf(pFile, "%12.1f", m_listAzimuth[s_i]);
			for(size_t s_j = 0; s_j < m_listElevation.size(); s_j++)
			{
				if(!bInt)
					fprintf(pFile, "%12.5f", m_data[s_i][s_j]);
				else
					fprintf(pFile, "%12.0f", m_data[s_i][s_j]);
			}
			fprintf(pFile, "\n");
		}
		fclose(pFile);
		return true;
	}

	// 子程序名称： getPCVValue   
	// 功能：根据观测方向, 利用双线性插值方法计算天线相位中心的值
	// 变量类型：Elevation : 高度角[0,  90]
	//           Azimuth   : 方位角[0, 360]
	//           nLagrange : 插值阶数
	// 输入：Elevation, Azimuth, nLagrange
	// 输出：value
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2010/03/26
	// 版本时间：
	// 修改记录：
	// 备注： 
	double AntPCVFile::getPCVValue(double Elevation, double Azimuth, int nLagrange)
	{
		if(int(m_listAzimuth.size()) < nLagrange || int(m_listElevation.size()) < nLagrange)
			return 0.0;
		if(Elevation < m_listElevation[0] || Elevation > m_listElevation[m_listElevation.size() - 1]
		|| Azimuth < m_listAzimuth[0] || Azimuth > m_listAzimuth[m_listAzimuth.size() - 1])
			return 0.0;
		// 获得相邻时间间隔, 默认等距
		double interval_E = m_listElevation[1] - m_listElevation[0];
		double span_E = Elevation - m_listElevation[0];
		int nLeftPos_E  = int(floor(span_E / interval_E));
		int nLeftNum_E  = int(floor(nLagrange / 2.0));
		int nRightNum_E = int(ceil(nLagrange / 2.0));
		int nBegin_E,nEnd_E; // 位于区间[0, nCLKDataNumber - 1]
		if(nLeftPos_E - nLeftNum_E + 1 < 0) // nEnd - nBegin = nLagrange - 1 
		{
			nBegin_E = 0;
			nEnd_E = nLagrange - 1;
		}
		else if(nLeftPos_E + nRightNum_E >= int(m_listElevation.size()))
		{
			nBegin_E = int(m_listElevation.size()) - nLagrange;
			nEnd_E = int(m_listElevation.size()) - 1;
		}
		else
		{
			nBegin_E = nLeftPos_E - nLeftNum_E + 1;
			nEnd_E = nLeftPos_E + nRightNum_E;
		}
		double interval_A = m_listAzimuth[1] - m_listAzimuth[0];
		double span_A = Azimuth - m_listAzimuth[0];
		int nLeftPos_A = int(floor(span_A / interval_A));//一个小数向下取整
		int nLeftNum_A = int(floor(nLagrange / 2.0));
		int nRightNum_A = int(ceil (nLagrange / 2.0));//一个数向上取整
		int nBegin_A,nEnd_A; // 位于区间[0, nCLKDataNumber - 1]
		if(nLeftPos_A - nLeftNum_A + 1 < 0) // nEnd - nBegin = nLagrange - 1 
		{
			nBegin_A = 0;
			nEnd_A = nLagrange - 1;
		}
		else if(nLeftPos_A + nRightNum_A >= int(m_listAzimuth.size()))
		{
			nBegin_A = int(m_listAzimuth.size()) - nLagrange;
			nEnd_A = int(m_listAzimuth.size()) - 1;
		}
		else
		{
			nBegin_A = nLeftPos_A - nLeftNum_A + 1;
			nEnd_A = nLeftPos_A + nRightNum_A;
		}
		// 1 插值计算[azimuth, elevation_i]的值
		double *x = new double [nLagrange];
		for(int i = nBegin_A; i <= nEnd_A; i++)
			x[i - nBegin_A] = m_listAzimuth[i];
		double *xx = new double [nLagrange];
		for(int i = nBegin_E; i <= nEnd_E; i++)
			xx[i - nBegin_E] = m_listElevation[i];
		double *y  = new double [nLagrange];
		double *yy = new double [nLagrange];
		/*double span_E_Min = 100.0;
		double xx_near = 0.0;
		bool bFind_E_Zero = false;*/
		for(int i = nBegin_E; i <= nEnd_E; i++)
		{
			/*double span_A_Min = 400.0;
			double yy_near = 0.0;
			bool bFind_A_Zero = false;*/
			for(int j = nBegin_A; j <= nEnd_A; j++)
			{
				y[j - nBegin_A] = m_data[j][i];
				//// 2016/04/13
				//if(span_A_Min > fabs(Azimuth - m_listAzimuth[j]) && m_data[j][i] != 0.0)
				//{
				//	span_A_Min = fabs(Azimuth - m_listAzimuth[j]);
				//	yy_near = m_data[j][i];
				//}
				//if(m_data[j][i] == 0.0)
				//	bFind_A_Zero = true;
			}
			// 2016/04/13
			//if(!bFind_A_Zero)
				InterploationLagrange(x, y, nLagrange, Azimuth,  yy[i - nBegin_E]);
			//else
			//	yy[i - nBegin_E] = yy_near;

			/*if(span_E_Min > fabs(Elevation - m_listElevation[i]) && yy[i - nBegin_E] != 0.0)
			{
				span_E_Min = fabs(Elevation - m_listElevation[i]);
				xx_near = yy[i - nBegin_E];
			}
			if(yy[i - nBegin_E] == 0.0)
				bFind_E_Zero = true;*/
		}

		// 2  插值计算[azimuth, elevation]的值
		double value = 0;
		//if(!bFind_E_Zero)
			InterploationLagrange(xx, yy, nLagrange, Elevation, value);
		//else
		//	value = xx_near;

		delete x;
		delete y;
		delete xx;
		delete yy;
		return value;
	}
}
