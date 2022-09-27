#include "SP3File.hpp"
#include"TimeCoordConvert.hpp"
#include <limits>
#include "MathAlgorithm.hpp"
#include "RuningInfoFile.hpp"
#include "CLKFile.hpp"

using namespace NUDTTK::Math;
namespace NUDTTK
{
	double SP3Epoch::getMJD()
	{
		return TimeCoordConvert::DayTime2MJD(t);
	}

	SP3File::SP3File(void)
	{
	}

	SP3File::~SP3File(void)
	{
	}

	int SP3File::getSatPRN(string strSatName)
	{
		char szSatPRN[3] = "  ";
		strSatName.copy(szSatPRN, 2, 1);
        szSatPRN[2] = '\0';
		return atoi(szSatPRN);
	}

	void  SP3File::clear()
	{
		m_header = SP3Header::SP3Header();
		m_data.clear();
	}
	bool SP3File::isEmpty()
	{
		if(m_data.size() > 0)
			return false;
		else
			return true;
	}
	// 子程序名称： isValidEpochLine   
	// 功能：判断当前文本行数据是否为有效时刻行 
	//         返回0 -> 文件末尾
	//         返回1 -> 有效时刻
	//         返回2 -> 无效
	// 变量类型：strLine           : 行文本 
	//           pSP3file      　　: 文件指针
	// 输入：strLine, pSP3file
	// 输出：
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2008/05/10
	// 版本时间：2008/05/10
	// 修改记录：
	// 备注： 
	int SP3File::isValidEpochLine(string strLine, FILE * pSP3file)
	{
		GPST tmEpoch;
		// 下面几种数据用int型，避免因为strLine的格式问题引起sscanf函数发生错误                       
		if(pSP3file != NULL)
		{ // 判断是否为文件末尾
			if(feof(pSP3file) || (strLine.find("EOF") < strLine.length()))
				return 0;
		}
		char szSymbols[2+1];
		sscanf(strLine.c_str(),"%2c%*1c%4d%*1c%2d%*1c%2d%*1c%2d%*1c%2d%*1c%11lf",
			szSymbols,&tmEpoch.year,&tmEpoch.month,&tmEpoch.day,&tmEpoch.hour,&tmEpoch.minute,&tmEpoch.second);
		szSymbols[2] = '\0';
		if(szSymbols[0] == '*') // 2008/05/10
		{
			int nFlag = 1;
			if(tmEpoch.month > 12 || tmEpoch.month < 0)
				nFlag = 2;
			if(tmEpoch.day > 31||tmEpoch.day < 0)
				nFlag = 2;
			if(tmEpoch.hour > 24 || tmEpoch.hour < 0)
				nFlag = 2;
			if(tmEpoch.minute > 60 || tmEpoch.minute < 0)
				nFlag = 2;
			if(tmEpoch.second > 60 || tmEpoch.second < 0)
				nFlag = 2;
			return nFlag;
		}
		else
		{
			return 2;
		}
	}

	// 子程序名称： getEpochSpan   
	// 功能：获得数据间隔 
	// 变量类型：
	// 输入：
	// 输出：
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2012/03/16
	// 版本时间：2012/03/16
	// 修改记录：
	// 备注： 
	double SP3File::getEpochSpan()
	{
		if( m_data.size() > 1 )
		{
			return m_data[1].t - m_data[0].t;
		}
		else
			return DBL_MAX;
	}

	// 子程序名称： open   
	// 功能：观测数据解析 
	// 变量类型：strSp3FileName : 观测数据文件路径
	// 输入：strSp3FileName
	// 输出：
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2008/05/10
	// 版本时间：2008/05/10
	// 修改记录：1、针对sp3-c sp3-d 格式不同，利用标识号识别头文件类型，邵凯，2019/03/14
	// 备注： 
	bool SP3File::open(string  strSp3FileName)
	{
		if(!isWildcardMatch(strSp3FileName.c_str(), "*.sp3", true) && !isWildcardMatch(strSp3FileName.c_str(), "*.eph", true))
			return false;
		FILE * pSP3file = fopen(strSp3FileName.c_str(),"r+t");
		if(pSP3file == NULL) 
			return false;
		// 读取头文件
		m_header = SP3Header::SP3Header();
		// Line 1
		char line[100];
		string strLine;
		fgets(line, 100, pSP3file);
		sscanf(line,"%2c%1c%4d%*1c%2d%*1c%2d%*1c%2d%*1c%2d%*1c%11lf%*1c%7d%*1c%5c%*1c%5c%*1c%3c%*1c%4c",
			         m_header.szSP3Version,
					 m_header.szPosVelFlag,
					 &m_header.tmStart.year,
			         &m_header.tmStart.month,
					 &m_header.tmStart.day,
					 &m_header.tmStart.hour,
					 &m_header.tmStart.minute,
					 &m_header.tmStart.second,
			         &m_header.nNumberofEpochs,
					 &m_header.szDataType,
					 &m_header.szCoordinateSys,
					 &m_header.szOrbitType,
					 &m_header.szAgency);
		m_header.szSP3Version[2] = '\0';
		m_header.szPosVelFlag[1] = '\0';
		m_header.szDataType[5]   = '\0';
		m_header.szCoordinateSys[5] = '\0';
		m_header.szOrbitType[3] = '\0';
		m_header.szAgency[4] = '\0';
		// Line 2
		fgets(line, 100, pSP3file);
		sscanf(line,"%2c%*1c%4d%*1c%15lf%*1c%14lf%*1c%5d%*1c%15lf",
			         m_header.szLine2Symbols,
					&m_header.tmGPSWeek.week,
					&m_header.tmGPSWeek.second,
			        &m_header.dEpochInterval,
					&m_header.nModJulDaySt,
					&m_header.dFractionalDay);
		m_header.szLine2Symbols[2] = '\0';
		// 从识别号为"+"开始逐行读取，直到识别为 "++"
		fgets(line, 100, pSP3file);
		strLine = line;
		sscanf(line,"%2c%*1c%3d",
			       m_header.szLine3Symbols,
					&m_header.bNumberofSats);
		m_header.szLine3Symbols[2] = '\0';
		int nLine    = m_header.bNumberofSats / 17;//行数；
		int nResidue = m_header.bNumberofSats % 17;//余数；
		//m_header.pstrSatNameList.clear(); // 2014/03/23, 兼容混合系统
		// 第三行数据
		if(m_header.bNumberofSats <= 17)
		{
			for(int i = 0; i < m_header.bNumberofSats; i++)
			{
				char strSatName[4];
				strLine.copy(strSatName, 3, 9 + i * 3);
				if(strSatName[0] == ' ')
					strSatName[0] = 'G';
				if(strSatName[1] == ' ')// 2014/03/22, 将G 1->G01
					strSatName[1] = '0';
				strSatName[3] = '\0';
                m_header.pstrSatNameList.push_back(strSatName);
			}
		}
		else
		{
			for(int i = 0; i < 17; i++)
			{
				char strSatName[4];
				strLine.copy(strSatName, 3, 9 + i * 3);
				if(strSatName[0] == ' ')
					strSatName[0] = 'G';
				if(strSatName[1] == ' ')// 2014/03/22, 将G 1->G01
					strSatName[1] = '0';
				strSatName[3] = '\0';
                m_header.pstrSatNameList.push_back(strSatName);
			}
		}
		// 读取中间行数据，直到 "++"
		bool flag = true;
		char szSymbols[2+1]; 
		int n_i = 0;
		while(flag)
		{
			fgets(line, 100, pSP3file);
			strLine = line;
		    sscanf(line,"%2c%*1c",szSymbols); // "++"
			n_i += 1;
			if(szSymbols[0] == '+' && szSymbols[1] == ' ')
			{
				if(n_i < nLine)
				{
					for(int i = 0; i < 17; i++)
					{
						char strSatName[4];
						strLine.copy(strSatName, 3, 9 + i * 3);
						if(strSatName[0] == ' ')
							strSatName[0] = 'G';
						if(strSatName[1] == ' ')// 2014/03/22, 将G 1->G01
							strSatName[1] = '0';
						strSatName[3] = '\0';
						m_header.pstrSatNameList.push_back(strSatName);
					}
				}
				// 读取最后 nResidue 个数据
				if(n_i == nLine && nResidue > 0)
				{
					for(int i = 0; i < nResidue; i++)
					{
						char strSatName[4];
						strLine.copy(strSatName, 3, 9 + i * 3);
						if(strSatName[0] == ' ')
							strSatName[0] = 'G';
						if(strSatName[1] == ' ')// 2014/03/22, 将G 1->G01
							strSatName[1] = '0';
						strSatName[3] = '\0';
						m_header.pstrSatNameList.push_back(strSatName);
					}
				}
			}
			else // "++"
			{
				// "++"行第一行
				sscanf(line, "%2s%*1c", m_header.szLine8Symbols);
				m_header.szLine8Symbols[2] = '\0';
		       m_header.pbySatAccuracyList.clear();
				if(m_header.bNumberofSats <= 17)
				{
					for(int i = 0; i < m_header.bNumberofSats; i++)
					{
						char strSatAccuracy[4];
						int bySatAccuracy;
						strLine.copy(strSatAccuracy,3,9+i*3);
						strSatAccuracy[3] = '\0';
						sscanf(strSatAccuracy,"%3d",&bySatAccuracy);
						m_header.pbySatAccuracyList.push_back(BYTE(bySatAccuracy));
					}
				}
				else
				{
					for(int i = 0; i < 17; i++)
					{
						char strSatAccuracy[4];
						int bySatAccuracy;
						strLine.copy(strSatAccuracy,3,9+i*3);
						strSatAccuracy[3] = '\0';
						sscanf(strSatAccuracy,"%3d",&bySatAccuracy);
						m_header.pbySatAccuracyList.push_back(BYTE(bySatAccuracy));
					}
				}
				flag = false; // 跳出循环
			}
		}
		// 读取中间行数据，直到 "%c"
		flag = true;
		n_i = 0;
		while(flag)
		{
			fgets(line, 100, pSP3file);
			strLine = line;
		    sscanf(line,"%2c%*1c",szSymbols); // "++"
			n_i += 1;
			if(szSymbols[0] == '+' && szSymbols[1] == '+')
			{
				if(n_i < nLine)
				{
					for(int i = 0; i < 17; i++)
					{
						char strSatAccuracy[4];
						int bySatAccuracy;
						strLine.copy(strSatAccuracy, 3, 9 + i * 3);
						strSatAccuracy[3] = '\0';
						sscanf(strSatAccuracy, "%3d", &bySatAccuracy);
						m_header.pbySatAccuracyList.push_back(BYTE(bySatAccuracy));
					}
				}
				// 读取最后 nResidue 个数据
				if(n_i == nLine && nResidue > 0)
				{
					for(int i = 0; i < nResidue; i++)
					{
						char strSatAccuracy[4];
						int bySatAccuracy;
						strLine.copy(strSatAccuracy, 3, 9 + i * 3);
						strSatAccuracy[3] = '\0';
						sscanf(strSatAccuracy, "%3d", &bySatAccuracy);
						m_header.pbySatAccuracyList.push_back(BYTE(bySatAccuracy));
					}
				}
			}
			else // "%c"
			{
				// "%c"行第一行
				sscanf(line,"%2c%*1c%2c%*4c%3c", m_header.szLine13Symbols, m_header.szFileType, m_header.szTimeSystem);
				m_header.szLine13Symbols[2] = '\0';
				m_header.szFileType[2]      = '\0';
				m_header.szTimeSystem[3]    = '\0';
				flag = false; // 跳出循环
			}
		}
		fgets(line, 100, pSP3file);
		// 根据导航系统标记清理卫星列表, 过滤非北斗 和 GPS 导航系统的卫星, 2012/12/17
		char cSatSystem = m_header.getSatSystemChar();
		m_header.bNumberofSats = int(m_header.pstrSatNameList.size());
		m_header.szFileType[0] = cSatSystem;
		// Line 15-16,"%f"
		fgets(line, 100, pSP3file);
		sscanf(line,"%2c%*1c%10lf%*1c%12lf", m_header.szLine15Symbols, &m_header.dBaseforPosVel, &m_header.dBaseforClkRate);
		m_header.szLine15Symbols[2] = '\0';
		fgets(line, 100, pSP3file);
		// Line 17-18,"%i"
		fgets(line, 100, pSP3file);
		sscanf(line,"%2c%*1c", m_header.szLine17Symbols);
		m_header.szLine17Symbols[2] = '\0';
		fgets(line, 100, pSP3file);
		
		//// Line 19-22
		//fgets(line, 100, pSP3file);
		//sscanf(line,"%2c%*1c", m_header.szLine19Symbols);
		//strLine     = line;
		//strLine.copy(m_header.szLine19Comment,57,2);
		//fgets(line, 100, pSP3file);
		//strLine     = line;
		//strLine.copy(m_header.szLine20Comment,57,2);
		//fgets(line, 100, pSP3file);
		//strLine     = line;
		//strLine.copy(m_header.szLine21Comment,57,2);
		//fgets(line, 100, pSP3file);
		//strLine     = line;
		//strLine.copy(m_header.szLine22Comment,57,2);

		//读取轨道数据----------------------------------------------------------
		bool bFlag = true;
		int k = 0;
		fgets(line, 100, pSP3file);
		m_data.clear();
		while(bFlag)
		{
			strLine = line;
			int nFlag = isValidEpochLine(strLine, pSP3file);//新时刻的判断；
			if(nFlag == 0) // 文件末尾
			{
				bFlag = false;
			}
			else if(nFlag == 1) // 找到新时刻的数据段
			{
				k++;
				SP3Epoch sp3Epoch;
				sscanf(line,"%*3c%4d%*1c%2d%*1c%2d%*1c%2d%*1c%2d%*1c%11lf",
					        &sp3Epoch.t.year,
					        &sp3Epoch.t.month,
							&sp3Epoch.t.day,
							&sp3Epoch.t.hour,
					        &sp3Epoch.t.minute,
							&sp3Epoch.t.second);
				//sp3Epoch.mjd = TimeCoordConvert::DayTime2MJD(sp3Epoch.t);
				sp3Epoch.sp3.clear();
				for(int i = 0; i < m_header.bNumberofSats; i++)
				{
					SP3Datum sp3Datum;
					// 读一行
					fgets(line, 100, pSP3file);
					char szSatName[3 + 1];// 2014/03/23, 兼容混合系统
					sscanf(line,"%*1c%3c%14lf%14lf%14lf%14lf",
						        szSatName,
								&sp3Datum.pos.x,
							    &sp3Datum.pos.y,
								&sp3Datum.pos.z,
								&sp3Datum.clk);
					if(szSatName[0] == ' ')
							szSatName[0] = 'G';
					if(szSatName[1] == ' ')// 2012/10/31, 将  1->G01
							szSatName[1] = '0';
					szSatName[3] = '\0';
					// 读两行
					if(m_header.szPosVelFlag[0] == 'V')
					{
						fgets(line, 100, pSP3file);
						sscanf(line,"%*4c%14lf%14lf%14lf%14lf",
							        &sp3Datum.vel.x,
							        &sp3Datum.vel.y,
									&sp3Datum.vel.z,
									&sp3Datum.clkrate);
					}
					//if(line[1] == m_header.getSatSystemChar() || line[1] == ' ') // 过滤非北斗 和 GPS 导航系统的卫星, 2012/12/17
						sp3Epoch.sp3.insert(SP3SatMap::value_type(szSatName, sp3Datum));
				}
				m_data.push_back(sp3Epoch);
				fgets(line, 100, pSP3file);
			}
			else
			{
				fgets(line, 100, pSP3file);
			}
		}
		fclose(pSP3file);
		if( m_data.size() > 0 )
		{
			m_header.tmStart = m_data[0].t;
			// 20150822, 进行连续化处理, 谷德峰
			double spanInterval = m_header.dEpochInterval;
			vector<SP3Epoch> data;
			data.push_back(m_data[0]);
			size_t s_i = 1;
			while(s_i < m_data.size())
			{
				GPST t_last = data[data.size() - 1].t;
				double span_t = m_data[s_i].t - t_last;
				if(span_t == spanInterval)
				{// 数据刚好匹配
					data.push_back(m_data[s_i]);
					s_i++;
					continue;
				}
				else if(span_t > spanInterval)
				{// 添加新的空纪录
					SP3Epoch sp3Epoch_t;
					sp3Epoch_t.sp3.clear();
					sp3Epoch_t.t = t_last + spanInterval;
					data.push_back(sp3Epoch_t);
					//char info[100];
					//sprintf(info, "星历数据不连续, 历元%s缺失!", sp3Epoch_t.t.toString().c_str());
					//RuningInfoFile::Add(info);
					continue;
				}
				else
				{// 跳过该条记录
					s_i++;
					continue;
				}
			}
			m_data = data; // 更新数据
		}
		return true;
	}

	bool SP3File::openV(string  strSp3FileName)
	{
		if(!isWildcardMatch(strSp3FileName.c_str(), "*.sp3", true) && !isWildcardMatch(strSp3FileName.c_str(), "*.PRE", true))
			return false;
		FILE * pSP3file = fopen(strSp3FileName.c_str(),"r+t");
		if(pSP3file == NULL) 
			return false;
		// 读取头文件
		m_header = SP3Header::SP3Header();
		// Line 1
		char line[100];
		string strLine;
		fgets(line, 100, pSP3file);
		sscanf(line,"%2c%1c%4d%*1c%2d%*1c%2d%*1c%2d%*1c%2d%*1c%11lf%*1c%7d%*1c%5c%*1c%5c%*1c%3c%*1c%4c",
			         m_header.szSP3Version,
					 m_header.szPosVelFlag,
					 &m_header.tmStart.year,
			         &m_header.tmStart.month,
					 &m_header.tmStart.day,
					 &m_header.tmStart.hour,
					 &m_header.tmStart.minute,
					 &m_header.tmStart.second,
			         &m_header.nNumberofEpochs,
					 &m_header.szDataType,
					 &m_header.szCoordinateSys,
					 &m_header.szOrbitType,
					 &m_header.szAgency);
		// Line 2
		fgets(line, 100, pSP3file);
		sscanf(line,"%2c%*1c%4d%*1c%15lf%*1c%14lf%*1c%5d%*1c%15lf",
			         m_header.szLine2Symbols,
					&m_header.tmGPSWeek.week,
					&m_header.tmGPSWeek.second,
			        &m_header.dEpochInterval,
					&m_header.nModJulDaySt,
					&m_header.dFractionalDay);
		// 从识别号为"+"开始逐行读取，直到识别为 "++"
		fgets(line, 100, pSP3file);
		strLine = line;
		sscanf(line,"%2c%*1c%3d",
			       m_header.szLine3Symbols,
					&m_header.bNumberofSats);
		int nLine    = m_header.bNumberofSats / 17;//行数；
		int nResidue = m_header.bNumberofSats % 17;//余数；
		m_header.pstrSatNameList.clear(); // 2014/03/23, 兼容混合系统
		// 第三行数据
		if(m_header.bNumberofSats <= 17)
		{
			for(int i = 0; i < m_header.bNumberofSats; i++)
			{
				char strSatName[4];
				strLine.copy(strSatName, 3, 9 + i * 3);
				if(strSatName[0] == ' ')
					strSatName[0] = 'G';
				if(strSatName[1] == ' ')// 2014/03/22, 将G 1->G01
					strSatName[1] = '0';
				strSatName[3] = '\0';
                m_header.pstrSatNameList.push_back(strSatName);
			}
		}
		else
		{
			for(int i = 0; i < 17; i++)
			{
				char strSatName[4];
				strLine.copy(strSatName, 3, 9 + i * 3);
				if(strSatName[0] == ' ')
					strSatName[0] = 'G';
				if(strSatName[1] == ' ')// 2014/03/22, 将G 1->G01
					strSatName[1] = '0';
				strSatName[3] = '\0';
                m_header.pstrSatNameList.push_back(strSatName);
			}
		}
		// 读取中间行数据，直到 "++"
		bool flag = true;
		char szSymbols[2+1]; 
		int n_i = 0;
		while(flag)
		{
			fgets(line, 100, pSP3file);
			strLine = line;
		    sscanf(line,"%2c%*1c",szSymbols); // "++"
			n_i += 1;
			if(szSymbols[0] == '+' && szSymbols[1] == ' ')
			{
				if(n_i < nLine)
				{
					for(int i = 0; i < 17; i++)
					{
						char strSatName[4];
						strLine.copy(strSatName, 3, 9 + i * 3);
						if(strSatName[0] == ' ')
							strSatName[0] = 'G';
						if(strSatName[1] == ' ')// 2014/03/22, 将G 1->G01
							strSatName[1] = '0';
						strSatName[3] = '\0';
						m_header.pstrSatNameList.push_back(strSatName);
					}
				}
				// 读取最后 nResidue 个数据
				if(n_i == nLine && nResidue > 0)
				{
					for(int i = 0; i < nResidue; i++)
					{
						char strSatName[4];
						strLine.copy(strSatName, 3, 9 + i * 3);
						if(strSatName[0] == ' ')
							strSatName[0] = 'G';
						if(strSatName[1] == ' ')// 2014/03/22, 将G 1->G01
							strSatName[1] = '0';
						strSatName[3] = '\0';
						m_header.pstrSatNameList.push_back(strSatName);
					}
				}
			}
			else // "++"
			{
				// "++"行第一行
				sscanf(line, "%2s%*1c", m_header.szLine8Symbols);
		       m_header.pbySatAccuracyList.clear();
				if(m_header.bNumberofSats <= 17)
				{
					for(int i = 0; i < m_header.bNumberofSats; i++)
					{
						char strSatAccuracy[4];
						int bySatAccuracy;
						strLine.copy(strSatAccuracy,3,9+i*3);
						strSatAccuracy[3] = '\0';
						sscanf(strSatAccuracy,"%3d",&bySatAccuracy);
						m_header.pbySatAccuracyList.push_back(BYTE(bySatAccuracy));
					}
				}
				else
				{
					for(int i = 0; i < 17; i++)
					{
						char strSatAccuracy[4];
						int bySatAccuracy;
						strLine.copy(strSatAccuracy,3,9+i*3);
						strSatAccuracy[3] = '\0';
						sscanf(strSatAccuracy,"%3d",&bySatAccuracy);
						m_header.pbySatAccuracyList.push_back(BYTE(bySatAccuracy));
					}
				}
				flag = false; // 跳出循环
			}
		}
		// 读取中间行数据，直到 "%c"
		flag = true;
		n_i = 0;
		while(flag)
		{
			fgets(line, 100, pSP3file);
			strLine = line;
		    sscanf(line,"%2c%*1c",szSymbols); // "++"
			n_i += 1;
			if(szSymbols[0] == '+' && szSymbols[1] == '+')
			{
				if(n_i < nLine)
				{
					for(int i = 0; i < 17; i++)
					{
						char strSatAccuracy[4];
						int bySatAccuracy;
						strLine.copy(strSatAccuracy, 3, 9 + i * 3);
						strSatAccuracy[3] = '\0';
						sscanf(strSatAccuracy, "%3d", &bySatAccuracy);
						m_header.pbySatAccuracyList.push_back(BYTE(bySatAccuracy));
					}
				}
				// 读取最后 nResidue 个数据
				if(n_i == nLine && nResidue > 0)
				{
					for(int i = 0; i < nResidue; i++)
					{
						char strSatAccuracy[4];
						int bySatAccuracy;
						strLine.copy(strSatAccuracy, 3, 9 + i * 3);
						strSatAccuracy[3] = '\0';
						sscanf(strSatAccuracy, "%3d", &bySatAccuracy);
						m_header.pbySatAccuracyList.push_back(BYTE(bySatAccuracy));
					}
				}
			}
			else // "%c"
			{
				// "%c"行第一行
				sscanf(line,"%2c%*1c%2c%*4c%3c", m_header.szLine13Symbols, m_header.szFileType, m_header.szTimeSystem);
				flag = false; // 跳出循环
			}
		}
		fgets(line, 100, pSP3file);
		// 根据导航系统标记清理卫星列表, 过滤非北斗 和 GPS 导航系统的卫星, 2012/12/17
		char cSatSystem = m_header.getSatSystemChar();
		m_header.bNumberofSats = int(m_header.pstrSatNameList.size());
		m_header.szFileType[0] = cSatSystem;
		// Line 15-16,"%f"
		fgets(line, 100, pSP3file);
		sscanf(line,"%2c%*1c%10lf%*1c%12lf", m_header.szLine15Symbols, &m_header.dBaseforPosVel, &m_header.dBaseforClkRate);
		fgets(line, 100, pSP3file);
		// Line 17-18,"%i"
		fgets(line, 100, pSP3file);
		sscanf(line,"%2c%*1c", m_header.szLine17Symbols);
		fgets(line, 100, pSP3file);
		
		// Line 19-22
		fgets(line, 100, pSP3file);
		sscanf(line,"%2c%*1c", m_header.szLine19Symbols);
		strLine     = line;
		strLine.copy(m_header.szLine19Comment,57,2);
		fgets(line, 100, pSP3file);
		strLine     = line;
		strLine.copy(m_header.szLine20Comment,57,2);
		fgets(line, 100, pSP3file);
		strLine     = line;
		strLine.copy(m_header.szLine21Comment,57,2);
		fgets(line, 100, pSP3file);
		strLine     = line;
		strLine.copy(m_header.szLine22Comment,57,2);

		//读取轨道数据----------------------------------------------------------
		bool bFlag = true;
		int k = 0;
		fgets(line, 100, pSP3file);
		m_data.clear();
		while(bFlag)
		{
			strLine = line;
			int nFlag = isValidEpochLine(strLine, pSP3file);//新时刻的判断
			if(nFlag == 0) // 文件末尾
			{
				bFlag = false;
			}
			else if(nFlag == 1) // 找到新时刻的数据段
			{
				k++;
				SP3Epoch sp3Epoch;
				sscanf(line,"%*3c%4d%*1c%2d%*1c%2d%*1c%2d%*1c%2d%*1c%11lf",
					        &sp3Epoch.t.year,
					        &sp3Epoch.t.month,
							&sp3Epoch.t.day,
							&sp3Epoch.t.hour,
					        &sp3Epoch.t.minute,
							&sp3Epoch.t.second);
				//sp3Epoch.mjd = TimeCoordConvert::DayTime2MJD(sp3Epoch.t);
				sp3Epoch.sp3.clear();
				for(int i = 0; i < m_header.bNumberofSats; i++)
				{
					SP3Datum sp3Datum;
					// 读一行
					fgets(line, 100, pSP3file);
					char szSatName[3 + 1];// 2014/03/23, 兼容混合系统
					sscanf(line,"%*1c%3c%14lf%14lf%14lf%14lf",
						        szSatName,
								&sp3Datum.pos.x,
							    &sp3Datum.pos.y,
								&sp3Datum.pos.z,
								&sp3Datum.clk);
					if(szSatName[0] == ' ')
							szSatName[0] = 'G';
					if(szSatName[1] == ' ')// 2012/10/31, 将  1->G01
							szSatName[1] = '0';
					szSatName[3] = '\0';
					// 读两行
					if(m_header.szPosVelFlag[0] == 'V')
					{
						fgets(line, 100, pSP3file);
						sscanf(line,"%*4c%14lf%14lf%14lf%14lf",
							        &sp3Datum.vel.x,
							        &sp3Datum.vel.y,
									&sp3Datum.vel.z,
									&sp3Datum.clkrate);
					}
					sp3Epoch.sp3.insert(SP3SatMap::value_type(szSatName, sp3Datum));
				}
				m_data.push_back(sp3Epoch);
				fgets(line, 100, pSP3file);
			}
			else
			{
				fgets(line, 100, pSP3file);
			}
		}
		fclose(pSP3file);
		//if( m_data.size() > 0 )
		//{
		//	m_header.tmStart = m_data[0].t;
		//	// 20150822, 进行连续化处理, 谷德峰
		//	double spanInterval = m_header.dEpochInterval;
		//	vector<SP3Epoch> data;
		//	data.push_back(m_data[0]);
		//	size_t s_i = 1;
		//	while(s_i < m_data.size())
		//	{
		//		GPST t_last = data[data.size() - 1].t;
		//		double span_t = m_data[s_i].t - t_last;
		//		if(span_t == spanInterval)
		//		{// 数据刚好匹配
		//			data.push_back(m_data[s_i]);
		//			s_i++;
		//			continue;
		//		}
		//		else if(span_t > spanInterval)
		//		{// 添加新的空纪录
		//			SP3Epoch sp3Epoch_t;
		//			sp3Epoch_t.sp3.clear();
		//			sp3Epoch_t.t = t_last + spanInterval;
		//			data.push_back(sp3Epoch_t);
		//			//char info[100];
		//			//sprintf(info, "星历数据不连续, 历元%s缺失!", sp3Epoch_t.t.toString().c_str());
		//			//RuningInfoFile::Add(info);
		//			continue;
		//		}
		//		else
		//		{// 跳过该条记录
		//			s_i++;
		//			continue;
		//		}
		//	}
		//	m_data = data; // 更新数据
		//}
		return true;
	}
	// 子程序名称： write   
	// 功能：将SP3数据写到文件 
	// 变量类型： strSp3FileName      : 完整的观测数据文件路径
	// 输入：strSp3FileName
	// 输出：
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2008/05/10
	// 版本时间：2008/05/10
	// 修改记录：1、针对sp3-c sp3-d 格式不同，利用标识号识别头文件类型，邵凯，2019/03/14
	// 备注： 
	bool SP3File::write(string strSp3FileName)
	{
		if(m_data.size() < 0)
			return false;
		FILE* pSP3file = fopen(strSp3FileName.c_str(), "w+");
		// Line 1
		fprintf(pSP3file,"%2s%1s%4d %2d %2d %2d %2d %11lf %7d %5s %5s %3s %4s\n",
			            m_header.szSP3Version,
						 m_header.szPosVelFlag,
						 m_header.tmStart.year,
			             m_header.tmStart.month,
						 m_header.tmStart.day,
						 m_header.tmStart.hour,
						 m_header.tmStart.minute,
						 m_header.tmStart.second,
			             m_header.nNumberofEpochs,
						 m_header.szDataType,
						 m_header.szCoordinateSys,
						 m_header.szOrbitType,
						 m_header.szAgency);
		// Line 2
		fprintf(pSP3file,"%2s %4d %15.8f %14.8f %5d %15.13f\n",
			             m_header.szLine2Symbols,
				         m_header.tmGPSWeek.week,
				         m_header.tmGPSWeek.second,
			             m_header.dEpochInterval,
				         m_header.nModJulDaySt,
				         m_header.dFractionalDay);

		int nLine    = m_header.bNumberofSats / 17;//行数；
		int nResidue = m_header.bNumberofSats % 17;//余数；

		// Line 3
		fprintf(pSP3file,"%2s%-1s%3d%-3s",
			              m_header.szLine3Symbols,
						  " ",
						  m_header.bNumberofSats,
						  " ");
		for(int i = 0; i < 17; i++)
		{
			if(i < m_header.bNumberofSats)
				fprintf(pSP3file,"%3s", m_header.pstrSatNameList[i].c_str());   // 2014/03/23, 兼容混合系统
			else
				fprintf(pSP3file," %2d",0);
		}
		fprintf(pSP3file,"\n");

		// Line "+ "
		for(int i = 1; i < nLine+1; i++)
		{
			fprintf(pSP3file,"%2s%-7s",m_header.szLine3Symbols," ");
			for(int j = 0; j < 17; j++)
			{
				int nIndex = i * 17 + j;
				if(nIndex < m_header.bNumberofSats)
					fprintf(pSP3file,"%3s", m_header.pstrSatNameList[nIndex].c_str());   // 2014/03/23, 兼容混合系统
				else
					fprintf(pSP3file, " %2d", 0);
			}
			fprintf(pSP3file, "\n");
		}
		// Line 8--12
		for(int i = 0; i < nLine+1; i++)
		{
			fprintf(pSP3file, "%2s%-7s", m_header.szLine8Symbols, " ");
			for(int j = 0; j < 17; j++)
			{
				int nIndex = i * 17 + j;
				if(nIndex < m_header.bNumberofSats)
					fprintf(pSP3file, "%3d", m_header.pbySatAccuracyList[nIndex]);
				else
					fprintf(pSP3file, "%3d", 0);
			}
			fprintf(pSP3file, "\n");
		}
		// Line 13-14
		fprintf(pSP3file,"%2s %-2s cc %3s ccc cccc cccc cccc cccc ccccc ccccc ccccc ccccc\n",
			             m_header.szLine13Symbols,
						 m_header.szFileType,
						 m_header.szTimeSystem);
		fprintf(pSP3file,"%2s cc cc ccc ccc cccc cccc cccc cccc ccccc ccccc ccccc ccccc\n",
			             m_header.szLine13Symbols);
		// Line 15-16
		fprintf(pSP3file,"%2s %10.7f %12.9f %14.11f %18.15f\n",
			             m_header.szLine15Symbols,
						 m_header.dBaseforPosVel,
						 m_header.dBaseforClkRate,
						 0.0,
						 0.0);
		fprintf(pSP3file,"%2s %10.7f %12.9f %14.11f %18.15f\n",
			             m_header.szLine15Symbols,
						 0.0,
						 0.0,
						 0.0,
						 0.0);
		// Line 17-18
		fprintf(pSP3file,"%2s %4d %4d %4d %4d %6d %6d %6d %6d %9d\n",
			             m_header.szLine17Symbols,
						 0,
						 0,
						 0,
						 0,
						 0,
						 0,
						 0,
						 0,
						 0);
		fprintf(pSP3file,"%2s %4d %4d %4d %4d %6d %6d %6d %6d %9d\n",
			             m_header.szLine17Symbols,
						 0,
						 0,
						 0,
						 0,
						 0,
						 0,
						 0,
						 0,
						 0);
		////Line 19-22
		//fprintf(pSP3file,"%2s %57s\n",
		//	             m_header.szLine19Symbols,
		//				 m_header.szLine19Comment);
		//fprintf(pSP3file,"%2s %57s\n",
		//	             m_header.szLine19Symbols,
		//				 m_header.szLine20Comment);
		//fprintf(pSP3file,"%2s %57s\n",
		//	             m_header.szLine19Symbols,
		//				 m_header.szLine21Comment);
		//fprintf(pSP3file,"%2s %57s\n",
		//	             m_header.szLine19Symbols,
		//				 m_header.szLine22Comment);

		for(size_t s_i = 0; s_i < m_data.size(); s_i++)
		{
			SP3Epoch sp3Epoch = m_data[s_i];
			// 时间量化处理, 2007/11/06 
			// 在文件保存时,为了避免出现 60 秒, 分钟不进位的情况
			DayTime t = sp3Epoch.t;
			t.second = 0;
			double sencond = Round(sp3Epoch.t.second * 1.0E+8) * 1.0E-8;
			t = t + sencond;
			fprintf(pSP3file,"*  %4d %2d %2d %2d %2d %11.8f\n",
				             t.year,
					         t.month,
							 t.day,
							 t.hour,
					         t.minute,
							 t.second);
			for(SP3SatMap::iterator it = sp3Epoch.sp3.begin(); it != sp3Epoch.sp3.end(); ++it)
			{
				SP3Datum sp3Datum = it->second;
				// 写一行
				fprintf(pSP3file,"P%3s%14.6f%14.6f%14.6f%14.6f\n",
					             it->first.c_str(),// 2014/03/23, 兼容混合系统
								 sp3Datum.pos.x,
					             sp3Datum.pos.y,
								 sp3Datum.pos.z,
								 sp3Datum.clk);
				// 写两行
				if(m_header.szPosVelFlag[0] == 'V')
				{
					fprintf(pSP3file,"V%3s%14.6f%14.6f%14.6f%14.6f\n",
						             it->first.c_str(),// 2014/03/23, 兼容混合系统
									 sp3Datum.vel.x,
						             sp3Datum.vel.y,
									 sp3Datum.vel.z,
									 sp3Datum.clkrate);
				}
			}
		}
		fprintf(pSP3file,"EOF\n");
		fclose(pSP3file);
		return true;
	}

	// 子程序名称： getEphemeris   
	// 功能：滑动lagrange插值获得任意时刻t、卫星号为nPRN的GPS卫星星历
	// 变量类型： t                     :  GPST
	//            name                  :  名称
	//            sp3Datum              :  星历数值, 坐标单位: 米 (sp3文件中的单位 km)
	//            nLagrange             :  Lagrange插值已知点个数, 默认为9, 对应 8 阶 Lagrange 插值
	// 输入：t, nPRN, nLagrange
	// 输出：sp3Datum
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/2/07
	// 版本时间：2014/3/23
	// 修改记录：1. 2012/9/23 由谷德峰修改, (m_data[i].getMJD() - m_data[0].getMJD()) * 86400.0 有问题, getMJD 存在小数位精度问题
	//           2. 2014/3/23 由谷德峰修改, 兼容混合系统
	// 备注： 
	bool SP3File::getEphemeris(GPST t, string name, SP3Datum& sp3Datum, int nLagrange)
	{
		if(getEphemeris_0(t, name, sp3Datum, nLagrange)) 
			return true;
		else
		{// 针对 igc 实时星历数据存在部分小间隔缺失现象, 进行15分钟粗采样
			int nSP3DataNumber = int(m_data.size()); // 数据点个数
			if(nSP3DataNumber < nLagrange || t - m_data[0].t < -1.0 ||  t - m_data[nSP3DataNumber - 1].t > 1.0) // 如果数据点个数小于n，返回
				return false;
			// 获得相邻时间间隔，默认等距
			double dSpanSecond = getEpochSpan();
			if(dSpanSecond == 30.0) // 处理 igc 实时星历数据, 避免短时星历不连续情况
			{
				double span_fix = 900.0; // 15分钟
				int span_num = int(span_fix / dSpanSecond);
				int count_fix = int(ceil((m_data[nSP3DataNumber - 1].t - m_data[0].t) / span_fix)); 
				if(count_fix < nLagrange)
					return false;
				double dSpanSecond_T = t - m_data[0].t;
				int nLeftPos_fix  = int(dSpanSecond_T / span_fix);
				// 理论上nLeftPos左右两边参考点的个数,nLeftNum + nRightNum = nLagrange
				int nLeftNum_fix  = int(floor(nLagrange / 2.0));
				int nRightNum_fix = int(ceil (nLagrange / 2.0));
				int nBegin_fix, nEnd_fix;               // 位于区间[0, nSP3DataNumber - 1]
				if(nLeftPos_fix - nLeftNum_fix + 1 < 0) // nEnd_fix - nBegin_fix = nLagrange - 1 
				{
					nBegin_fix = 0;
					nEnd_fix   = nLagrange - 1;
				}
				else if(nLeftPos_fix + nRightNum_fix >= count_fix)
				{
					nBegin_fix = count_fix - nLagrange;
					nEnd_fix   = count_fix - 1;
				}
				else
				{
					nBegin_fix = nLeftPos_fix - nLeftNum_fix + 1;
					nEnd_fix   = nLeftPos_fix + nRightNum_fix;
				} 
				// 取出 nBegin 到 nEnd 间nPRN号GPS卫星的星历, 构成列表 sp3DatumList
				double *xa_t = new double [nLagrange];
				double *ya_X = new double [nLagrange];
				double *ya_Y = new double [nLagrange];
				double *ya_Z = new double [nLagrange];
				// 此处循环搜索部分需要提高效率，20080222
				int validcount = 0;
				for(int i = nBegin_fix; i <= nEnd_fix; i++)
				{
					bool bFind = false;
					for(int j = i * span_num; j < (i + 1) * span_num; j++)
					{
						if(j < nSP3DataNumber) // 确保不溢出
						{	SP3SatMap::const_iterator it = m_data[j].sp3.find(name);
							if(it != m_data[j].sp3.end())
							{
								if(!(it->second.pos.x == 0.0 && it->second.pos.y == 0.0 && it->second.pos.z == 0.0)) // 20150118, 不使用部分问题星历产品, 谷德峰
								{
									bFind = true;
									ya_X[i - nBegin_fix] = it->second.pos.x;
									ya_Y[i - nBegin_fix] = it->second.pos.y;
									ya_Z[i - nBegin_fix] = it->second.pos.z;
									xa_t[i - nBegin_fix] = m_data[j].t - m_data[0].t;
									validcount++;
									break; // 每隔区间只要保证 1 个点即可
								}
							}
						}
					}
					if(!bFind)
					{
						//char info[100];
						//sprintf(info, "%s %s 星历数据不完整!", t.toString().c_str(), name.c_str());
						//RuningInfoFile::Add(info);
						break; // 只要缺失一个点及时跳出
					}
				}
				if(validcount != nLagrange) // nPRN号GPS卫星数据完整性较验
				{
					delete xa_t;
					delete ya_X;
					delete ya_Y;
					delete ya_Z;
					return false;
				}
				// 通过位置插值出位置和速度
				InterploationLagrange(xa_t, ya_X, nLagrange, dSpanSecond_T, sp3Datum.pos.x, sp3Datum.vel.x);
				InterploationLagrange(xa_t, ya_Y, nLagrange, dSpanSecond_T, sp3Datum.pos.y, sp3Datum.vel.y);
				InterploationLagrange(xa_t, ya_Z, nLagrange, dSpanSecond_T, sp3Datum.pos.z, sp3Datum.vel.z);
				// 转换到单位米
				sp3Datum.pos.x = sp3Datum.pos.x * 1000;
				sp3Datum.pos.y = sp3Datum.pos.y * 1000;
				sp3Datum.pos.z = sp3Datum.pos.z * 1000;
				sp3Datum.vel.x = sp3Datum.vel.x * 1000;
				sp3Datum.vel.y = sp3Datum.vel.y * 1000;
				sp3Datum.vel.z = sp3Datum.vel.z * 1000;
				delete xa_t;
				delete ya_X;
				delete ya_Y;
				delete ya_Z;
				return true;
			}
			else
				return false;
		}
	}

	// 子程序名称： getEphemeris_0   
	// 功能：滑动lagrange插值获得任意时刻t、卫星号为nPRN的GPS卫星星历
	// 变量类型： t                     :  GPST
	//            name                  :  名称
	//            sp3Datum              :  星历数值, 坐标单位: 米 (sp3文件中的单位 km)
	//            nLagrange             :  Lagrange插值已知点个数, 默认为9, 对应 8 阶 Lagrange 插值
	// 输入：t, nPRN, nLagrange
	// 输出：sp3Datum
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/2/07
	// 版本时间：2014/3/23
	// 修改记录：1. 2012/9/23 由谷德峰修改, (m_data[i].getMJD() - m_data[0].getMJD()) * 86400.0 有问题, getMJD 存在小数位精度问题
	//           2. 2014/3/23 由谷德峰修改, 兼容混合系统
	// 备注： 
	bool SP3File::getEphemeris_0(GPST t, string name, SP3Datum& sp3Datum, int nLagrange)
	{
		int nSP3DataNumber = int(m_data.size()); // 数据点个数
		if(nSP3DataNumber < nLagrange || t - m_data[0].t < -1.0 ||  t - m_data[nSP3DataNumber - 1].t > 1.0) // 如果数据点个数小于n，返回
			return false;
		// 获得相邻时间间隔，默认等距
		double dSpanSecond = getEpochSpan();
		double dSpanSecond_T = t - m_data[0].t;
		// 记录n个参考已知点
		// 首先寻找最靠近时间T的左端点, 从0开始计数, 默认GPS星历数据是等时间间隔的
		int nLeftPos  = int(dSpanSecond_T / dSpanSecond);
		// 理论上nLeftPos左右两边参考点的个数,nLeftNum + nRightNum = nLagrange
		int nLeftNum  = int(floor(nLagrange / 2.0));
		int nRightNum = int(ceil (nLagrange / 2.0));
		int nBegin, nEnd;               // 位于区间[0, nSP3DataNumber - 1]
		if(nLeftPos - nLeftNum + 1 < 0) // nEnd - nBegin = nLagrange - 1 
		{
			nBegin = 0;
			nEnd   = nLagrange - 1;
		}
		else if(nLeftPos + nRightNum >= nSP3DataNumber)
		{
			nBegin = nSP3DataNumber - nLagrange;
			nEnd   = nSP3DataNumber - 1;
		}
		else
		{
			nBegin = nLeftPos - nLeftNum + 1;
			nEnd   = nLeftPos + nRightNum;
		}
		// 取出 nBegin 到 nEnd 间nPRN号GPS卫星的星历, 构成列表 sp3DatumList
		double *xa_t = new double [nLagrange];
		double *ya_X = new double [nLagrange];
		double *ya_Y = new double [nLagrange];
		double *ya_Z = new double [nLagrange];
		// 此处循环搜索部分需要提高效率，20080222
		int validcount = 0;
		for(int i = nBegin; i <= nEnd; i++)
		{
			xa_t[i - nBegin] = m_data[i].t - m_data[0].t; // (m_data[i].getMJD() - m_data[0].getMJD()) * 86400.0 有问题, getMJD 存在小数位精度问题
			SP3SatMap::const_iterator it;
			if((it = m_data[i].sp3.find(name)) != m_data[i].sp3.end())
			{
				if(!(it->second.pos.x == 0.0 && it->second.pos.y == 0.0 && it->second.pos.z == 0.0)) // 20150118, 不使用部分问题星历产品, 谷德峰
				{
					ya_X[i - nBegin] = it->second.pos.x;
					ya_Y[i - nBegin] = it->second.pos.y;
					ya_Z[i - nBegin] = it->second.pos.z;
					validcount++;
				}
				else
				{
					//char info[100];
					//sprintf(info, "%s %s 星历数据不完整!", t.toString().c_str(), name.c_str());
					//RuningInfoFile::Add(info);
					break; // 只要缺失一个点及时跳出
				}
			}
		}
		if(validcount != nLagrange) // nPRN号GPS卫星数据完整性较验
		{
			delete xa_t;
			delete ya_X;
			delete ya_Y;
			delete ya_Z;
			return false;
		}
		// 通过位置插值出位置和速度
		InterploationLagrange(xa_t, ya_X, nLagrange, dSpanSecond_T, sp3Datum.pos.x, sp3Datum.vel.x);
		InterploationLagrange(xa_t, ya_Y, nLagrange, dSpanSecond_T, sp3Datum.pos.y, sp3Datum.vel.y);
		InterploationLagrange(xa_t, ya_Z, nLagrange, dSpanSecond_T, sp3Datum.pos.z, sp3Datum.vel.z);
		// 转换到单位米
		sp3Datum.pos.x = sp3Datum.pos.x * 1000;
		sp3Datum.pos.y = sp3Datum.pos.y * 1000;
		sp3Datum.pos.z = sp3Datum.pos.z * 1000;
		sp3Datum.vel.x = sp3Datum.vel.x * 1000;
		sp3Datum.vel.y = sp3Datum.vel.y * 1000;
		sp3Datum.vel.z = sp3Datum.vel.z * 1000;
		delete xa_t;
		delete ya_X;
		delete ya_Y;
		delete ya_Z;
		return true;
	}

	// 子程序名称： getEphemeris   
	// 功能：滑动lagrange插值获得任意时刻t、卫星号为nPRN的GPS卫星星历
	// 变量类型： t                     :  GPST
	//            nPRN                  :  GPS卫星号
	//            sp3Datum              :  星历数值, 坐标单位: 米 (sp3文件中的单位 km)
	//            nLagrange             :  Lagrange插值已知点个数, 默认为9, 对应 8 阶 Lagrange 插值
	//            cSatSystem            :  导航系统标记
	// 输入：t, nPRN, nLagrange
	// 输出：sp3Datum
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/2/07
	// 版本时间：2014/3/23
	// 修改记录：1. 2012/9/23 由谷德峰修改, (m_data[i].getMJD() - m_data[0].getMJD()) * 86400.0 有问题, getMJD 存在小数位精度问题
	//           2. 2014/3/23 由谷德峰修改, 兼容混合系统
	// 备注： 
	bool SP3File::getEphemeris(GPST t, int nPRN, SP3Datum& sp3Datum, int nLagrange, char cSatSystem)
	{	
		char szSatName[4];
		sprintf(szSatName, "%c%02d", cSatSystem, nPRN);
        szSatName[3] = '\0';
		return getEphemeris(t, szSatName, sp3Datum, nLagrange);
	}

	// 子程序名称： getEphemeris_PathDelay   
	// 功能：根据接收机的概略位置、信号接收时间和GPS卫星的先验轨道,
	//       计算GPS卫星nPRN的信号传播延迟时间(即GPS卫星“准确的”信号发射时间)
	// 变量类型： t                  : 信号接收时间, 不准确, 需要考虑钟差
	//            receiverPosClock   : 接收机概略位置 + 接收机本地钟面时间, 概略位置单位：米, 概略钟差单位：米
	//            name               : GNSS卫星名称
	//            delay              : 信号传播延迟时间, 单位：秒
	//            sp3Datum           : 确定了正确的信号发射时间后，顺便返回本颗GPS卫星星历
	//            threshold          : 迭代阈值，默认 1.0E-007
	// 输入：t, receiverPosClock, sp3file, nPRN, threshold
	// 输出：sp3Datum
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/2/06
	// 版本时间：
	// 修改记录：1. 2007/07/15 由谷德峰修改, abs函数改为fabs
	//           2. 2007/04/06 由谷德峰修改, 防止 delay 溢出
	// 备注： 
	//		在概略距离的计算中, 由于时空的不一致, ITRF坐标系还存在一个地球自转修正的问题，
	//		但地球自转修正引起的位置改正为100米左右, 对应的星历位置改正大约为0.001m, 
	//		而且地球自转修正为旋转变换, 对应的星历位置改正更小, 因此在延迟的计算中可以忽略。
	bool SP3File::getEphemeris_PathDelay(GPST t, POSCLK receiverPosClock, string name, double& delay, SP3Datum& sp3Datum,double threshold)
	{
		// 信号真实接收时间 = 观测时间(T) - 接收机钟差(receiverPos.dClock)
		GPST t_Receive  = t - receiverPosClock.clk / SPEED_LIGHT;
		GPST t_Transmit = t_Receive; // 初始化GPS信号发射时间
		// 获得GPS卫星nPRN的位置
		if(!getEphemeris(t_Transmit, name, sp3Datum))
			return false;
		double distance = pow(receiverPosClock.x - sp3Datum.pos.x, 2)
                        + pow(receiverPosClock.y - sp3Datum.pos.y, 2)
					    + pow(receiverPosClock.z - sp3Datum.pos.z, 2);
		distance = sqrt(distance); // 获得GPS信号发射传播距离
		double delay_k_1 = 0;
		delay = distance / SPEED_LIGHT;  // 获得GPS信号发射传播延迟
		const double delay_max  = 1.0;   // 为了防止迭代dDelay溢出，这里设置一个阈值
		const int    k_max      = 5;     // 迭代次数阈值，一般1次迭代就会收敛？ 
		int          k          = 0;
		while(fabs(delay - delay_k_1) > threshold)   // 迭代阈值控制, abs-->fabs, 2007/07/15
		{
			k++;
			if(fabs(delay) > delay_max || k > k_max) // 为防止 delay 溢出, 2007/04/06
			{
				printf("%d%d%f delay 迭代发散!\n", t.hour, t.minute, t.second);				
				return false;
			}
			// 更新 GPS 信号发射时间
			t_Transmit = t_Receive - delay;
			if(!getEphemeris(t_Transmit, name, sp3Datum))
				return false;
			// 更新概略距离
			distance =  pow(receiverPosClock.x - sp3Datum.pos.x, 2)
                      + pow(receiverPosClock.y - sp3Datum.pos.y, 2)
                      + pow(receiverPosClock.z - sp3Datum.pos.z, 2);
			distance = sqrt(distance);
			// 更新延迟数据
			delay_k_1 = delay;
			delay = distance / SPEED_LIGHT;
		}
		return true;
	}

	// 子程序名称： getEphemeris_PathDelay   
	// 功能：根据接收机的概略位置、信号接收时间和GPS卫星的先验轨道,
	//       计算GPS卫星nPRN的信号传播延迟时间(即GPS卫星“准确的”信号发射时间)
	// 变量类型： t                  : 信号接收时间, 不准确, 需要考虑钟差
	//            receiverPosClock   : 接收机概略位置 + 接收机本地钟面时间, 概略位置单位：米, 概略钟差单位：米
	//            nPRN               : GPS卫星号
	//            delay              : 信号传播延迟时间, 单位：秒
	//            sp3Datum           : 确定了正确的信号发射时间后，顺便返回本颗GPS卫星星历
	//            threshold          : 迭代阈值，默认 1.0E-007
	// 输入：t, receiverPosClock, sp3file, nPRN, threshold
	// 输出：sp3Datum
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2014/04/05
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool SP3File::getEphemeris_PathDelay(GPST t, POSCLK receiverPosClock, int nPRN, double& delay, SP3Datum& sp3Datum,double threshold)
	{
		char szSatName[4];
		sprintf(szSatName, "G%02d", nPRN);
        szSatName[3] = '\0';
		return getEphemeris_PathDelay(t, receiverPosClock, szSatName, delay, sp3Datum, threshold);
	}

	// 子程序名称： getEphemeris   
	// 功能：滑动lagrange插值获得任意时刻t、卫星号为nPRN的GPS卫星钟差
	// 变量类型： t                     :  GPST
	//            name                  :  名称
	//            clk                   :  钟差
	//            clkrate               :  钟差变化率
	//            nLagrange             :  Lagrange插值已知点个数, 默认为9, 对应 8 阶 Lagrange 插值
	// 输入：t, name, nLagrange
	// 输出：clk, clkrate
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2015/10/05
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool SP3File::getClock(GPST t, string name, double &clk, double &clkrate, int nLagrange)
	{
		int nSP3DataNumber = int(m_data.size()); // 数据点个数
		if(nSP3DataNumber < nLagrange) // 如果数据点个数小于n，返回
			return false;
		// 获得相邻时间间隔，默认等距
		double dSpanSecond = getEpochSpan();
		if(dSpanSecond == 30.0) // 处理 igc 实时星历数据, 避免短时星历不连续情况
		{
			double span_fix = 450.0; // 7.5分钟
			int span_num = int(span_fix / dSpanSecond);
			int count_fix = int(ceil((m_data[nSP3DataNumber - 1].t - m_data[0].t) / span_fix)); 
			if(count_fix < nLagrange)
				return false;
			double dSpanSecond_T = t - m_data[0].t;
			int nLeftPos_fix  = int(dSpanSecond_T / span_fix);
			// 理论上nLeftPos左右两边参考点的个数,nLeftNum + nRightNum = nLagrange
			int nLeftNum_fix  = int(floor(nLagrange / 2.0));
			int nRightNum_fix = int(ceil (nLagrange / 2.0));
			int nBegin_fix, nEnd_fix;               // 位于区间[0, nSP3DataNumber - 1]
			if(nLeftPos_fix - nLeftNum_fix + 1 < 0) // nEnd_fix - nBegin_fix = nLagrange - 1 
			{
				nBegin_fix = 0;
				nEnd_fix   = nLagrange - 1;
			}
			else if(nLeftPos_fix + nRightNum_fix >= count_fix)
			{
				nBegin_fix = count_fix - nLagrange;
				nEnd_fix   = count_fix - 1;
			}
			else
			{
				nBegin_fix = nLeftPos_fix - nLeftNum_fix + 1;
				nEnd_fix   = nLeftPos_fix + nRightNum_fix;
			} 
			// 取出 nBegin 到 nEnd 间nPRN号GPS卫星的星历, 构成列表 sp3DatumList
			double *xa_t = new double [nLagrange];
			double *ya_c = new double [nLagrange];
			// 此处循环搜索部分需要提高效率，20080222
			int validcount = 0;
			for(int i = nBegin_fix; i <= nEnd_fix; i++)
			{
				bool bFind = false;
				for(int j = i * span_num; j < (i + 1) * span_num; j++)
				{
					SP3SatMap::const_iterator it;
					if(j < nSP3DataNumber) // 确保不溢出
					{
						if((it = m_data[j].sp3.find(name)) != m_data[j].sp3.end())
						{
							if(it->second.clk != 999999.999999) // 20150118, 不使用部分问题星历产品, 谷德峰
							{
								bFind = true;
								ya_c[i - nBegin_fix] = it->second.clk;
								xa_t[i - nBegin_fix] = m_data[j].t - m_data[0].t;
								validcount++;
								break; // 每隔区间只要保证 1 个点即可
							}
						}
					}
				}
				if(!bFind)
					break; // 只要缺失一个点及时跳出
			}
			if(validcount != nLagrange) // nPRN号GPS卫星数据完整性较验
			{
				delete xa_t;
				delete ya_c;
				return false;
			}
			// 通过位置插值出位置和速度
			InterploationLagrange(xa_t, ya_c, nLagrange, dSpanSecond_T, clk, clkrate);
			clk = clk * 1.0E-6;  // 转换到秒 
			clkrate = clkrate * 1.0E-6;
			delete xa_t;
			delete ya_c;
			return true;
		}
		else
		{
			double dSpanSecond_T = t - m_data[0].t;
			// 记录n个参考已知点
			// 首先寻找最靠近时间T的左端点, 从0开始计数, 默认GPS星历数据是等时间间隔的
			int nLeftPos  = int(dSpanSecond_T / dSpanSecond);
			// 理论上nLeftPos左右两边参考点的个数,nLeftNum + nRightNum = nLagrange
			int nLeftNum  = int(floor(nLagrange / 2.0));
			int nRightNum = int(ceil (nLagrange / 2.0));
			int nBegin, nEnd;               // 位于区间[0, nSP3DataNumber - 1]
			if(nLeftPos - nLeftNum + 1 < 0) // nEnd - nBegin = nLagrange - 1 
			{
				nBegin = 0;
				nEnd   = nLagrange - 1;
			}
			else if(nLeftPos + nRightNum >= nSP3DataNumber)
			{
				nBegin = nSP3DataNumber - nLagrange;
				nEnd   = nSP3DataNumber - 1;
			}
			else
			{
				nBegin = nLeftPos - nLeftNum + 1;
				nEnd   = nLeftPos + nRightNum;
			}
			// 取出 nBegin 到 nEnd 间nPRN号GPS卫星的星历, 构成列表 sp3DatumList
			double *xa_t = new double [nLagrange];
			double *ya_c = new double [nLagrange];
			// 此处循环搜索部分需要提高效率，20080222
			int validcount = 0;
			for(int i = nBegin; i <= nEnd; i++)
			{
				xa_t[i - nBegin] = m_data[i].t - m_data[0].t; // (m_data[i].getMJD() - m_data[0].getMJD()) * 86400.0 有问题, getMJD 存在小数位精度问题
				SP3SatMap::const_iterator it;
				if((it = m_data[i].sp3.find(name)) != m_data[i].sp3.end())
				{
					if(it->second.clk != 999999.999999) // 20150118, 不使用部分问题星历产品, 谷德峰
					{
						ya_c[i - nBegin] = it->second.clk;
						validcount++;
					}
					else
					{// 解决 igu 在某个历元如 "2014  4 19 17 45  0.00000000" 处过多卫星出现钟差 "999999.999999" 而导致的钟差导出时成片缺失现象
						if(i == nBegin && nBegin - 1 >= 0)
						{// 首尾数据可向两边各延伸一个
							it = m_data[nBegin - 1].sp3.find(name);
							if(it != m_data[nBegin - 1].sp3.end())
							{
								if(it->second.clk != 999999.999999) // 20150118, 不使用部分问题星历产品, 谷德峰
								{
									xa_t[i - nBegin] = m_data[nBegin - 1].t - m_data[0].t;
									ya_c[i - nBegin] = it->second.clk;
									validcount++;
									continue;
								}
							}
						}
						if(i == nEnd && nEnd + 1 < nSP3DataNumber)
						{// 首尾数据可向两边各延伸一个
							it = m_data[nEnd + 1].sp3.find(name);
							if(it != m_data[nEnd + 1].sp3.end())
							{
								if(it->second.clk != 999999.999999) // 20150118, 不使用部分问题星历产品, 谷德峰
								{
									xa_t[i - nBegin] = m_data[nEnd + 1].t - m_data[0].t;
									ya_c[i - nBegin] = it->second.clk;
									validcount++;
									continue;
								}
							}
						}
						break; // 只要缺失一个点及时跳出
					}
				}
			}
			if(validcount != nLagrange) // nPRN号GPS卫星数据完整性较验
			{
				delete xa_t;
				delete ya_c;
				return false;
			}
			// 通过位置插值出位置和速度
			InterploationLagrange(xa_t, ya_c, nLagrange, dSpanSecond_T, clk, clkrate);
			clk = clk * 1.0E-6;  // 转换到秒 
			clkrate = clkrate * 1.0E-6;
			delete xa_t;
			delete ya_c;
			return true;
		}
	}

	// 子程序名称： exportCLKFile   
	// 作用：导出clk格式钟差文件
	// 类型：strCLKfileName  : 文件名称
	//       T_Begin         : 星历开始时间
	//       T_End           : 星历结束时间
	//       spanSeconds    : 星历相邻时间点的时间间隔，默认2分钟
	// 输入：T_Begin, T_End, dSpanSeconds
	// 输出：
	// 语言：C++
	// 创建时间：2015/10/05
	// 版本时间：
	// 修改记录：
	// 备注：
	void SP3File::exportCLKFile(string strCLKfileName, DayTime T_Begin, DayTime T_End, double spanSeconds)
	{
		CLKFile clkfile;
		BYTE pbySatList[MAX_PRN_GPS];    // 卫星列表
		for(int i = 0; i < MAX_PRN_GPS; i++)
			pbySatList[i] = 0;
		DayTime T = T_Begin;
		int k = 0;
		while( T_End - T > 0 )
		{
			CLKEpoch clkEpoch;
			clkEpoch.t = T;
			clkEpoch.ARList.clear();
			clkEpoch.ASList.clear();
			for(int i = 0; i < MAX_PRN_GPS; i++)
			{
				char  satname[4];
				sprintf(satname,"G%02d",i);
				satname[3] = '\0';
				double clk = 0;
				double clkrate = 0;
				if(getClock(T, satname, clk, clkrate, 2))
				{
					pbySatList[i] = 1;
					CLKDatum   ASDatum;
					ASDatum.count = 2;
					ASDatum.name = satname;						
					ASDatum.clkBias = clk;
					ASDatum.clkBiasSigma = 0;
					clkEpoch.ASList.insert(CLKMap::value_type(satname, ASDatum));					
				}
			}
			clkfile.m_data.push_back(clkEpoch);
			T = T + spanSeconds;
		}
		// 书写文件头
		sprintf(clkfile.m_header.szRinexVersion, "2.0");
		clkfile.m_header.cFileType = 'C';
		sprintf(clkfile.m_header.szProgramName,"NUDT 2.0");
		sprintf(clkfile.m_header.szAgencyName,"%s", m_header.szAgency);
		clkfile.m_header.LeapSecond = 0;
		clkfile.m_header.ClockDataTypeCount = 1;
		clkfile.m_header.pstrClockDataTypeList.clear();
		clkfile.m_header.pstrClockDataTypeList.push_back("AS");
		sprintf(clkfile.m_header.szACShortName,"NDT");
		clkfile.m_header.nStaCount = 0;
		sprintf(clkfile.m_header.szStaCoordFrame,"%s", m_header.szCoordinateSys);
		// 综合统计可视卫星列表
		clkfile.m_header.pszSatList.clear();
		for(int i = 0; i < MAX_PRN_GPS; i++)
		{
			if(pbySatList[i] == 1)
			{
				char szPRN[4];
				sprintf(szPRN, "G%02d", i);
				szPRN[3] = '\0';
				clkfile.m_header.pszSatList.push_back(szPRN);
			}
		}
		clkfile.m_header.bySatCount = BYTE(clkfile.m_header.pszSatList.size());
		clkfile.write(strCLKfileName);
	}
}
