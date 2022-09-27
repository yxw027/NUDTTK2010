#include "StaOceanLoadingDisplacementFile.hpp"

namespace NUDTTK
{
	StaOceanLoadingDisplacementFile::StaOceanLoadingDisplacementFile(void)
	{
	}

	StaOceanLoadingDisplacementFile::~StaOceanLoadingDisplacementFile(void)
	{
	}

	// 返回值: 0 - EOF; 1 - 有效行; 2 - 无效行
	int StaOceanLoadingDisplacementFile::isValidLine(string strLine,FILE * pStaOldFile)
	{
		if(pStaOldFile != NULL) // 判断是否为文件末尾
		{
			if(feof(pStaOldFile))
				return 0;
		}
		char szHead[3];
		sscanf(strLine.c_str(),"%2c",szHead);
		szHead[2] = '\0';
		// 文字注释行
		if(strcmp(szHead,"$$") == 0)
			return 2;
		return 1;
	}

	// 获得下一个有效的记录行(忽略“$$”行)
	bool StaOceanLoadingDisplacementFile::nextValidLine(string &strLine, FILE * pStaOldFile)
	{
		if(pStaOldFile != NULL) // 判断是否为文件末尾
		{
			if(feof(pStaOldFile))
				return false;
		}
		char szline[200]; 
		int  bFlag = 1;
		fgets(szline, 200, pStaOldFile);
		while(bFlag)
		{
			int nLineFlag = isValidLine(szline, pStaOldFile);
			if(nLineFlag == 0)
			{// 文件结束
				bFlag = false;
				return  false;
			}
			else if(nLineFlag == 1)
			{// 发现有效记录
				strLine = szline;
				return  true;
			}
			else
			{// 无效行
				fgets(szline, 200, pStaOldFile);
			}
		}
		return false;
	}

    // 解析数据行
	bool StaOceanLoadingDisplacementFile::readOceanTideWaveLine(string strLine, OceanTideWave& line)
	{
		if(strLine.length() < 78)
			return false;
		OceanTideWave OTWLine;
		// M2  S2  N2  K2  K1  O1  P1  Q1  MF  MM SSA
		sscanf(strLine.c_str(), "%*2c%6lf%*c%6lf%*c%6lf%*c%6lf%*c%6lf%*c%6lf%*c%6lf%*c%6lf%*c%6lf%*c%6lf%*c%6lf",
								&OTWLine.M2,
								&OTWLine.S2,
								&OTWLine.N2,
								&OTWLine.K2,
								&OTWLine.K1,
								&OTWLine.O1,
								&OTWLine.P1,
								&OTWLine.Q1,
								&OTWLine.MF,
								&OTWLine.MM,
								&OTWLine.SSA);
		line = OTWLine;
		return true;
	}

	bool StaOceanLoadingDisplacementFile::open(string  strStaOldFileName)
	{
		FILE * pStaOldFile = fopen(strStaOldFileName.c_str(),"r+t");
		if(pStaOldFile == NULL) 
			return false;
		string strLine; 
		int k = 0;
		m_data.clear();
		StaOceanTide sotDatum;
		while(nextValidLine(strLine, pStaOldFile))
		{
			switch(k%7)
			{
			case 0:
				if(k > 0)
				{// 添加上一个测站的潮汐数据
					m_data.push_back(sotDatum);
				}
				// 记录新测站ID
				char szName_4c[4+1];
				sscanf(strLine.c_str(),"%*2c%4c", szName_4c);
				szName_4c[4] = '\0';
                sotDatum.id = atoi(szName_4c);
				sotDatum.name_4c = szName_4c;
				//sscanf(strLine.c_str(),"%*2c%4d", &sotDatum.id); // 2013/10/22, 调整为测站名称
				break;
			case 1:
				// 振幅
				readOceanTideWaveLine(strLine, sotDatum.amplitude[0]);
				break;
			case 2:
				// 振幅
				readOceanTideWaveLine(strLine, sotDatum.amplitude[1]);
				break;
			case 3:
				// 振幅
				readOceanTideWaveLine(strLine, sotDatum.amplitude[2]);
				break;
			case 4:
				// 相位
				readOceanTideWaveLine(strLine, sotDatum.phase[0]);
				break;
			case 5:
				// 相位
				readOceanTideWaveLine(strLine, sotDatum.phase[1]);
				break;
			case 6:
				// 相位
				readOceanTideWaveLine(strLine, sotDatum.phase[2]);
				break;
			}
			k++;
		}
		if(k > 0)
		{// 添加最后一个测站的潮汐数据, 2014/10/12
			m_data.push_back(sotDatum);
		}
		fclose(pStaOldFile);
		return true;
	}

	// 获得给定序号的测站的潮汐参数
	bool StaOceanLoadingDisplacementFile::getStaOceanTide(int id, StaOceanTide& sotDatum)
	{
		bool bFind = false;
		size_t count = m_data.size();
		if(count <= 0)
			return false;
		int i;
		for(i =  int(count) - 1; i >= 0; i--)
		{// 倒序查找最近的测站信息, 2008/01/21
			if(m_data[i].id == id)
			{
				bFind = true;
				break;
			}
		}
		if(!bFind)
			return false;
		sotDatum = m_data[i];
		return true;
	}

	// 获得给定名称的测站的潮汐参数
	bool StaOceanLoadingDisplacementFile::getStaOceanTide(string name, StaOceanTide& sotDatum)
	{
		bool bFind = false;
		size_t count = m_data.size();
		if(count <= 0)
			return false;
		// 测站名称转换成大写
        char scNAME[5];
		for(int i = 0; i < 4; i++)
			scNAME[i] = toCapital(name[i]);
		scNAME[4] = '\0';
		int i;
		for(i =  int(count) - 1; i >= 0; i--)
		{// 倒序查找最近的测站信息, 2008/01/21
			if(m_data[i].name_4c.find(scNAME) != -1)
			{
				bFind = true;
				break;
			}
		}
		if(!bFind)
			return false;
		sotDatum = m_data[i];
		return true;
	}
}
