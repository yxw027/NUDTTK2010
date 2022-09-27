#include "MathAlgorithm.hpp"
#include "GNSS_AttitudeFile.hpp"
#include <time.h>

using namespace NUDTTK::Math;
namespace NUDTTK
{
	GNSS_AttFile::GNSS_AttFile(void)
	{
	}

	GNSS_AttFile::~GNSS_AttFile(void)
	{
	}

	int GNSS_AttFile::getSatPRN(string strSatName)
	{
		char szSatPRN[3] = "  ";
		strSatName.copy(szSatPRN, 2, 1);
		szSatPRN[2] = '\0';
		return atoi(szSatPRN);
	}

	void GNSS_AttFile::clear()
	{
		m_header = AttHeader::AttHeader();
		m_data.clear();
	}

	bool GNSS_AttFile::isEmpty()
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
	//           pAttFILE      　　: 文件指针
	// 输入：strLine, pAttFILE
	// 输出：
	// 语言：C++
	// 创建者：韦春博
	// 创建时间：2021/10/22
	// 版本时间：2021/10/22
	// 修改记录：
	// 备注：
	int GNSS_AttFile::isValidEpochLine(string strLine, FILE *pAttFILE)
	{
		GPST tmEpoch;
		if(pAttFILE != NULL)
		{// 判断是否为文件末尾
			if(feof(pAttFILE) || (strLine.find("END") < strLine.length()))
				return 0;
		}
		char szSymbols[2+1];
		sscanf(strLine.c_str(), "%2c%*1c%4d%*1c%2d%*1c%2d%*1c%2d%*1c%2d%*1c%15lf", 
			                     szSymbols, 
			                     &tmEpoch.year, 
								 &tmEpoch.month, 
								 &tmEpoch.day, 
								 &tmEpoch.hour, 
								 &tmEpoch.minute, 
								 &tmEpoch.second);
		szSymbols[2] = '\0';
		if(szSymbols[0] == '#')
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
			return 2;
	}
	// 子程序名称： getEpochSpan   
	// 功能：获得数据间隔 
	// 变量类型：
	// 输入：
	// 输出：
	// 语言：C++
	// 创建者：韦春博
	// 创建时间：2021/10/22
	// 版本时间：2021/10/22
	// 修改记录：
	// 备注： 
	double GNSS_AttFile::getEpochSpan()
	{
		if(m_data.size() > 1)
			return m_data[1].t - m_data[0].t;
		else
			return DBL_MAX;
	}
	// 子程序名称： open   
	// 功能：观测数据解析 
	// 变量类型：strAttFileName : 观测数据文件路径
	// 输入：strAttFileName
	// 输出：
	// 语言：C++
	// 创建者：韦春博
	// 创建时间：2021/10/22
	// 版本时间：2021/10/22
	// 修改记录：
	// 备注：
	bool GNSS_AttFile::open(string strAttFileName)
	{
		if(!isWildcardMatch(strAttFileName.c_str(), "*_ATT.OBX", true)
			&& !isWildcardMatch(strAttFileName.c_str(), "*.att", true)
			&& !isWildcardMatch(strAttFileName.c_str(), "*.obx", true))
			return false;
		FILE *pAttFile = fopen(strAttFileName.c_str(), "r+t");
		if(pAttFile == NULL)
			return false;
		// 读取头文件部分
		m_header = AttHeader::AttHeader();
		// 跳过前8行
		char line[200];
		string strLine;
		for(int s_i = 0; s_i < 8; s_i++)
			fgets(line, 200, pAttFile);
		// 第9行，TIME_SYSTEM
		fgets(line, 200, pAttFile);
		sscanf(line, "%*21c%4c", &m_header.szTimeSystem);
		m_header.szTimeSystem[4] = '\0';
		// 第10行，START_TIME
		fgets(line, 200, pAttFile);
		sscanf(line, "%*21c%4d%*1c%2d%*1c%2d%*1c%2d%*1c%2d%*1c%15lf", 
			          &m_header.tmStart.year, 
					  &m_header.tmStart.month,
					  &m_header.tmStart.day,
					  &m_header.tmStart.hour, 
					  &m_header.tmStart.minute,
					  &m_header.tmStart.second);
		// 第11行，END_TIME
		fgets(line, 200, pAttFile);
		sscanf(line, "%*21c%4d%*1c%2d%*1c%2d%*1c%2d%*1c%2d%*1c%15lf", 
			          &m_header.tmEnd.year, 
					  &m_header.tmEnd.month,
					  &m_header.tmEnd.day,
					  &m_header.tmEnd.hour, 
					  &m_header.tmEnd.minute,
					  &m_header.tmEnd.second);
		// 第12行，EPOCH_INTERVAL
		fgets(line, 200, pAttFile);
		sscanf(line, "%*21c%8lf", &m_header.EpochSpan);
		// 第13行，COORD_SYSTEM
		fgets(line, 200, pAttFile);
		sscanf(line, "%*21c%5c", &m_header.szCoordinateSys);
		m_header.szCoordinateSys[5] = '\0';
		// 第14行，FRAME_TYPE
		fgets(line, 200, pAttFile);
		sscanf(line, "%*21c%4c", &m_header.szFrameType);
		m_header.szFrameType[4] = '\0';
		// 第15行 ORBIT_TYPE
		/*fgets(line,200,pAttFile);
		sscanf(line,"%*21c%4c", &m_header.szOrbit_Type);
		m_header.szOrbit_Type[4] = '\0';*/
		// 跳过3行
		for(int s_i = 0; s_i < 3; s_i++)
			fgets(line, 200, pAttFile);
		// 读取卫星列表 SATELLITE/ID_AND_DESCRIPTION
		bool Flag = true;
		while(Flag)
		{
			char szSymbols[1+1];
			char strSatName[4];
			fgets(line, 200, pAttFile);
			sscanf(line, "%1c%3c", szSymbols, strSatName);
			szSymbols[1] = '\0';
			strSatName[3] = '\0';
			if(szSymbols[0] == ' ')
				m_header.pstrSatNameList.push_back(strSatName);
			else
				Flag = false;
		}
		// 寻找数据部分并进行读取
		bool bFlag = true;
		m_data.clear();
		fgets(line, 200, pAttFile);
		while(bFlag)
		{
			strLine = line;
			int nFlag = isValidEpochLine(strLine, pAttFile);
			if(nFlag == 0)
			{
				bFlag = false;        // 文件末尾
			}
			else if(nFlag == 1)       // 找到新时刻的数据段
			{
				// nFlag == 1，有效时刻
				int NumSat;
				AttEpoch i_attEpoch;
				sscanf(line, "%*3c%4d%*1c%2d%*1c%2d%*1c%2d%*1c%2d%*1c%15lf%*1c%3d", 
					          &i_attEpoch.t.year, 
							  &i_attEpoch.t.month, 
							  &i_attEpoch.t.day, 
							  &i_attEpoch.t.hour, 
							  &i_attEpoch.t.minute, 
						      &i_attEpoch.t.second, 
							  &NumSat);
				i_attEpoch.attMap.clear();
				for(int s_i = 0; s_i < NumSat; s_i++)
				{
					ATT_Q4 Q4;
					char szSatName[3+1];
					fgets(line, 200, pAttFile);
					sscanf(line, "%*5c%3c%*16c%19lf%*1c%19lf%*1c%19lf%*1c%19lf",
						          szSatName, 
								  &Q4.q4, &Q4.q1, &Q4.q2, &Q4.q3);
					szSatName[3] = '\0';
					i_attEpoch.attMap.insert(AttSatMap::value_type(szSatName, Q4));
				}
				m_data.push_back(i_attEpoch);
				fgets(line, 200, pAttFile);
			}
			else
			{
				fgets(line, 200, pAttFile);
			}
		}
		fclose(pAttFile);
		return true;
	}

	// 子程序名称： write   
	// 功能：将ATT数据写到文件 
	// 变量类型： strAttFileName      : 完整的观测数据文件路径
	// 输入：strAttFileName
	// 输出：
	// 语言：C++
	// 创建者：韦春博
	// 创建时间：2021/10/22
	// 版本时间：2021/10/22
	// 修改记录：
	// 备注：
	bool GNSS_AttFile::write(string strAttFileName)
	{
		if(m_data.size() < 0)
			return false;
		FILE *pATTFile = fopen(strAttFileName.c_str(), "w+");
		// 当前时间
		time_t tt = time(NULL);
		tm* t = localtime(&tt);
		// Line 1
		fprintf(pATTFile, "%%=ORBEX  0.09\n");
		fprintf(pATTFile, "%%%\n");
		fprintf(pATTFile, "+FILE/DESCRIPTION\n");
		fprintf(pATTFile, " DESCRIPTION         Attitude quaternions for CODE products\n");
		fprintf(pATTFile, " CREATED_BY          CODE IGS-AC\n");
		fprintf(pATTFile, " CREATION_DATE       %4d %02d %02d %02d %02d %02d\n", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
		fprintf(pATTFile, " INPUT_DATA          F1_14103.STD\n");
		fprintf(pATTFile, " CONTACT             code@aiub.unibe.ch\n");
		fprintf(pATTFile, " TIME_SYSTEM         GPS\n");
		fprintf(pATTFile, " START_TIME          %4d %02d %02d %02d %02d %15.12f\n", m_header.tmStart.year, m_header.tmStart.month, m_header.tmStart.day, m_header.tmStart.hour, m_header.tmStart.minute, m_header.tmStart.second);
		fprintf(pATTFile, " END_TIME            %4d %02d %02d %02d %02d %15.12f\n", m_header.tmEnd.year, m_header.tmEnd.month, m_header.tmEnd.day, m_header.tmEnd.hour, m_header.tmEnd.minute, m_header.tmEnd.second);
		fprintf(pATTFile, " EPOCH_INTERVAL      %8.3f\n", m_header.EpochSpan);
		fprintf(pATTFile, " COORD_SYSTEM        IGS14\n");
		fprintf(pATTFile, " FRAME_TYPE          ECEF\n");
		fprintf(pATTFile, " LIST_OF_REC_TYPES   ATT\n");
		fprintf(pATTFile, "-FILE/DESCRIPTION\n");
		fprintf(pATTFile, "+SATELLITE/ID_AND_DESCRIPTION\n");
		for(size_t s_i = 0; s_i < m_header.pstrSatNameList.size(); s_i++)
		{
			fprintf(pATTFile, " %3s\n", m_header.pstrSatNameList[s_i].c_str());
		}
		fprintf(pATTFile, "-SATELLITE/ID_AND_DESCRIPTION\n");
		fprintf(pATTFile, "+EPHEMERIS/DATA\n");
		fprintf(pATTFile, "+SATELLITE/ID_AND_DESCRIPTION\n");
		fprintf(pATTFile, "*ATT RECORDS: ECEF --> SAT. BODY FRAME\n");
		fprintf(pATTFile, "*\n");
		fprintf(pATTFile, "*REC_ID_              N ___q0_(scalar)_____  ___q1__x__________ ___q1__y___________ ___q1__z___________\n");
		for(size_t s_i = 0; s_i < m_data.size(); s_i++)
		{
			AttEpoch i_attEpoch = m_data[s_i];
			// 时间行
			fprintf(pATTFile, "## %4d %02d %02d %02d %02d %15.12f %3d\n", 
				               i_attEpoch.t.year, 
							   i_attEpoch.t.month, 
							   i_attEpoch.t.day, 
							   i_attEpoch.t.hour, 
							   i_attEpoch.t.minute, 
							   i_attEpoch.t.second, 
							   int(i_attEpoch.attMap.size()));
			// 数据行
			for(AttSatMap::iterator it = i_attEpoch.attMap.begin(); it != i_attEpoch.attMap.end(); ++it)
			{
				ATT_Q4 Q4 = it->second;
				// 数据行
				fprintf(pATTFile, " ATT %3s              4 %19.16f %19.16f %19.16f %19.16f\n", it->first.c_str(), Q4.q4, Q4.q1, Q4.q2, Q4.q3);
			}
		}
		fprintf(pATTFile, "-EPHEMERIS/DATA\n");
		fprintf(pATTFile, "%%END_ORBEX\n");

		fclose(pATTFile);
		return true;
	}

	// 子程序名称： getQ4    
	// 功能：滑动lagrange插值获得任意时刻t、卫星号为nPRN的GPS卫星姿态
	// 变量类型： t                      :  GPST
	//            name                  :  名称
	//            AttEpoch              :  姿态数值, 四元数
	//            nLagrange             :  Lagrange插值已知点个数, 默认为9, 对应 8 阶 Lagrange 插值
	// 输入：t, nPRN, nLagrange
	// 输出：AttEpoch
	// 语言：C++
	// 创建者：韦春博
	// 创建时间：2021/10/22
	// 版本时间：2021/10/22
	// 修改记录：
	// 备注：
	bool GNSS_AttFile::getQ4(GPST t, string name, ATT_Q4 &Q4, int nlagrange)
	{
		int nATTDataNumber = int(m_data.size());   // 数据点个数
		if(nATTDataNumber < nlagrange || t - m_data[0].t < -1.0 ||  t - m_data[nATTDataNumber - 1].t > 1.0) // 如果数据点个数小于n，返回
			return false;
		// 获得相邻时刻间隔，默认等间距
		double dSpanSecond = m_header.EpochSpan/10;
		double dSpanSecond_T = t - m_data[0].t;	
		// 记录n个参考已知点
		// 首先寻找最靠近时间T的左端点, 从0开始计数, 默认GPS星历数据是等时间间隔的
		int nLeftPos  = int(dSpanSecond_T / dSpanSecond);
		// 理论上nLeftPos左右两边参考点的个数,nLeftNum + nRightNum = nLagrange
		int nLeftNum  = int(floor(nlagrange / 2.0));
		int nRightNum = int(ceil (nlagrange / 2.0));
		int nBegin, nEnd;               // 位于区间[0, nSP3DataNumber - 1]
		if(nLeftPos - nLeftNum + 1 < 0) // nEnd - nBegin = nLagrange - 1 
		{
			nBegin = 0;
			nEnd   = nlagrange - 1;
		}
		else if(nLeftPos + nRightNum >= nATTDataNumber)
		{
			nBegin = nATTDataNumber - nlagrange;
			nEnd   = nATTDataNumber - 1;
		}
		else
		{
			nBegin = nLeftPos - nLeftNum + 1;
			nEnd   = nLeftPos + nRightNum;
		}
		// 取出 nBegin 到 nEnd 间nPRN号GPS卫星的姿态, 构成列表 attEpochList
		double *xa_t  = new double [nlagrange];
		double *ya_q1 = new double [nlagrange];
		double *ya_q2 = new double [nlagrange];
		double *ya_q3 = new double [nlagrange];
		double *ya_q4 = new double [nlagrange];
		int validcount = 0;
		for(int i = nBegin; i <= nEnd; i++)
		{
			xa_t[i - nBegin] = m_data[i].t - m_data[0].t;
			AttSatMap::const_iterator it;
			if((it = m_data[i].attMap.find(name)) != m_data[i].attMap.end())
			{
				// 暂时不考虑GNSS卫星姿态数据中存在问题点
				ya_q1[i - nBegin] = it->second.q1;
				ya_q2[i - nBegin] = it->second.q2;
				ya_q3[i - nBegin] = it->second.q3;
				ya_q4[i - nBegin] = it->second.q4;
				validcount++;
			}
		}
		if(validcount != nlagrange)
		{
			delete xa_t;
			delete ya_q1;
			delete ya_q2;
			delete ya_q3;
			delete ya_q4;
			return false;
		}
		// 通过插值获得观测时刻的GNSS卫星姿态
		InterploationLagrange(xa_t, ya_q1, nlagrange, dSpanSecond_T, Q4.q1);
		InterploationLagrange(xa_t, ya_q2, nlagrange, dSpanSecond_T, Q4.q2);
		InterploationLagrange(xa_t, ya_q3, nlagrange, dSpanSecond_T, Q4.q3);
		InterploationLagrange(xa_t, ya_q4, nlagrange, dSpanSecond_T, Q4.q4);
		delete xa_t;
		delete ya_q1;
		delete ya_q2;
		delete ya_q3;
		delete ya_q4;
		return true;
	}

	// 子程序名称： getAttMatrix   
	// 功能：滑动lagrange插值获得任意时刻t、卫星号为nPRN的GPS卫星姿态旋转矩阵 
	// 备注：官方发布的GNSS卫星姿态为地固坐标系-》星体系的旋转信息
	bool GNSS_AttFile::getAttMatrix(GPST t, string name, Matrix& matATT, int nlagrange)
	{
		ATT_Q4 Q4;
		if(!getQ4(t, name, Q4))
			return false;
		/*
			  地固坐标系到星体坐标系
			|x|                   |x|
			|y|      =[姿态矩阵] *|y| 
			|z|星体               |z|地固
		*/
		matATT.Init(3, 3);
		double q1 = Q4.q1;
		double q2 = Q4.q2;
		double q3 = Q4.q3;
		double q4 = Q4.q4;//q4就是文献里的q0第一个数
		//1
		/*matATT.SetElement(0, 0,  q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4);
		matATT.SetElement(0, 1,  2 * (q1 * q2 + q3 * q4));
		matATT.SetElement(0, 2,  2 * (q1 * q3 - q2 * q4));
		matATT.SetElement(1, 0,  2 * (q1 * q2 - q3 * q4));
		matATT.SetElement(1, 1, -q1 * q1 + q2 * q2 - q3 * q3 + q4 * q4);
		matATT.SetElement(1, 2,  2 * (q2 * q3 + q1 * q4));
		matATT.SetElement(2, 0,  2 * (q1 * q3 + q2 * q4));
		matATT.SetElement(2, 1,  2 * (q2 * q3 - q1 * q4));
		matATT.SetElement(2, 2, -q1 * q1 - q2 * q2 + q3 * q3 + q4 * q4);*/
		//2
		matATT.SetElement(0, 0, (q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4));
		matATT.SetElement(0, 1,  2 * (q1 * q2 - q3 * q4));
		matATT.SetElement(0, 2,  2 * (q1 * q3 + q2 * q4));
		matATT.SetElement(1, 0,  2 * (q1 * q2 + q3 * q4));
		matATT.SetElement(1, 1, (-q1 * q1 + q2 * q2 - q3 * q3 + q4 * q4));
		matATT.SetElement(1, 2,  2 * (q2 * q3 - q1 * q4));
		matATT.SetElement(2, 0,  2 * (q1 * q3 - q2 * q4));
		matATT.SetElement(2, 1,  2 * (q2 * q3 + q1 * q4));
		matATT.SetElement(2, 2, (-q1 * q1 - q2 * q2 + q3 * q3 + q4 * q4));
		//3
		//matATT.SetElement(0, 0, -(q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4));//**********************
		//matATT.SetElement(0, 1,  -2 * (q1 * q2 - q3 * q4));//***************************
		//matATT.SetElement(0, 2,  -2 * (q1 * q3 + q2 * q4));//*********************************
		//matATT.SetElement(1, 0,  -2 * (q1 * q2 + q3 * q4));//***************************
		//matATT.SetElement(1, 1, -(-q1 * q1 + q2 * q2 - q3 * q3 + q4 * q4));//***********************
		//matATT.SetElement(1, 2,  -2 * (q2 * q3 - q1 * q4));//************************
		//matATT.SetElement(2, 0,  2 * (q1 * q3 - q2 * q4));
		//matATT.SetElement(2, 1,  2 * (q2 * q3 + q1 * q4));
		//matATT.SetElement(2, 2, +(-q1 * q1 - q2 * q2 + q3 * q3 + q4 * q4));
		//4
		/*matATT.SetElement(0, 0,  -(q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4));
		matATT.SetElement(0, 1,  -2 * (q1 * q2 + q3 * q4));
		matATT.SetElement(0, 2,  -2 * (q1 * q3 - q2 * q4));
		matATT.SetElement(1, 0,  -2 * (q1 * q2 - q3 * q4));
		matATT.SetElement(1, 1,  -(-q1 * q1 + q2 * q2 - q3 * q3 + q4 * q4));
		matATT.SetElement(1, 2,  -2 * (q2 * q3 + q1 * q4));
		matATT.SetElement(2, 0,  -2 * (q1 * q3 + q2 * q4));
		matATT.SetElement(2, 1,  -2 * (q2 * q3 - q1 * q4));
		matATT.SetElement(2, 2,  -(-q1 * q1 - q2 * q2 + q3 * q3 + q4 * q4));*/
	    //5 
		/*matATT.SetElement(0, 0,  q4 * q4 +q1 * q1 - q2 * q2 - q3 * q3);
		matATT.SetElement(0, 1,  2 * (q1 * q2 - q4 * q3));
		matATT.SetElement(0, 2,  2 * (q1 * q3 + q4 * q2));
		matATT.SetElement(1, 0,  2 * (q1 * q2 + q4 * q3));
		matATT.SetElement(1, 1,  q4* q4 - q1 * q1 + q2 * q2 - q3 * q3);
		matATT.SetElement(1, 2,  2 * (q2 * q3 - q4 * q1));
		matATT.SetElement(2, 0,  2 * (q1 * q3 - q4 * q2));
		matATT.SetElement(2, 1,  2 * (q2 * q3 + q4 * q1));
		matATT.SetElement(2, 2,  q4* q4 - q1 * q1 - q2 * q2 + q3 * q3);*/
		return true;
	}
}