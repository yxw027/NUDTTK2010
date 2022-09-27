#include "Ionex1_0_File.hpp"

namespace NUDTTK
{
	const char Ionex1_0_MaskString::ioVerType[]                           ="IONEX VERSION / TYPE";
	const char Ionex1_0_MaskString::ioComment[]                           ="COMMENT             ";
	const char Ionex1_0_MaskString::ioPgmRunDate[]                        ="PGM / RUN BY / DATE ";
	const char Ionex1_0_MaskString::ioDescription[]                       ="DESCRIPTION         ";
	const char Ionex1_0_MaskString::ioEpochFirstMap[]                     ="EPOCH OF FIRST MAP  ";
	const char Ionex1_0_MaskString::ioEpochLastMap[]                      ="EPOCH OF LAST MAP   ";
	const char Ionex1_0_MaskString::ioInterval[]                          ="INTERVAL            ";
	const char Ionex1_0_MaskString::ioOfMapsinFile[]                      ="# OF MAPS IN FILE   ";
	const char Ionex1_0_MaskString::ioMappingFun[]                        ="MAPPING FUNCTION    ";
	const char Ionex1_0_MaskString::ioElevationCutOff[]                   ="ELEVATION CUTOFF    ";
	const char Ionex1_0_MaskString::ioObservablesUsed[]                   ="OBSERVABLES USED    ";
	const char Ionex1_0_MaskString::ioOfStastions[]                       ="# OF STATIONS       ";
	const char Ionex1_0_MaskString::ioOfSatellites[]                      ="# OF SATELLITES     ";
	const char Ionex1_0_MaskString::ioBaseRadius[]                        ="BASE RADIUS         ";
	const char Ionex1_0_MaskString::ioMapDimension[]                      ="MAP DIMENSION       ";
	const char Ionex1_0_MaskString::ioHgt1Hgt2Dhgt[]                      ="HGT1 / HGT2 / DHGT  ";
	const char Ionex1_0_MaskString::ioLat1Lat2Dlat[]                      ="LAT1 / LAT2 / DLAT  ";
	const char Ionex1_0_MaskString::ioLon1Lon2Dlon[]                      ="LON1 / LON2 / DLON  ";
	const char Ionex1_0_MaskString::ioExponent[]                          ="EXPONENT            ";
	const char Ionex1_0_MaskString::ioEndofHeader[]                       ="END OF HEADER       ";

	Ionex1_0_File::Ionex1_0_File(void)
	{
	}

	Ionex1_0_File::~Ionex1_0_File(void)
	{
	}

	void Ionex1_0_File::clear()
	{
		m_header = Ionex1_0_Header::Ionex1_0_Header();
		m_data.clear();
	}

	bool Ionex1_0_File::isEmpty()
	{
		if(m_data.size() > 0)
			return false;
		else
			return true;
	}

	bool Ionex1_0_File::open(string  strIONFileName)
	{
		if(!isWildcardMatch(strIONFileName.c_str(), "*.*I", true)
			&& !isWildcardMatch(strIONFileName.c_str(), "*.ion", true)
			)     //检查文件名的匹配问题
		{
			printf(" %s 文件名不匹配!\n", strIONFileName.c_str());
			return false;
		}

		FILE * pIONfile = fopen(strIONFileName.c_str(),"r+t");
		if(pIONfile == NULL) 
			return false;

		m_header = Ionex1_0_Header::Ionex1_0_Header();

		/*开始循环读取文件头Header，直到 END OF HEADER*/
		bool bFlag = true;
		while(bFlag)
		{
			char line[100];
			fgets(line,100,pIONfile);
			string strLineMask = line;
			string strLine     = line;
			strLineMask.erase(0, 60);
			// 剔除 \n
			size_t nPos_n = strLineMask.find('\n');
			if(nPos_n < strLineMask.length())
				strLineMask.erase(nPos_n, 1);
			// 超过20位，截取20位
			while(strLineMask.length() > 20)
				strLineMask.erase(strLineMask.length() - 1, 1);
			// 补齐20位
			if(strLineMask.length() < 20)
				strLineMask.append(20-strLineMask.length(),' ');
			if(strLineMask == Ionex1_0_MaskString::ioVerType)
			{
				strLine.copy(m_header.ioVersion,20,0);
				m_header.ioVersion[20] = '\0';
				strLine.copy(m_header.ioFileType,20,20);
				m_header.ioFileType[20] = '\0';
				strLine.copy(m_header.ioSatelliteSystem,20,40);
				m_header.ioSatelliteSystem[20] = '\0';
			}
			else if(strLineMask == Ionex1_0_MaskString::ioPgmRunDate)
			{
				strLine.copy(m_header.ioProgramName,20,0);
				m_header.ioProgramName[20] = '\0';
				strLine.copy(m_header.ioProgramAgencyName,20,20);
				m_header.ioProgramAgencyName[20] = '\0';
				strLine.copy(m_header.ioFileDate,20,40);
				m_header.ioFileDate[20] = '\0';
			}
			else if(strLineMask == Ionex1_0_MaskString::ioDescription)
			{
				strLine.copy(m_header.ioDescription,60,0);
				m_header.ioDescription[60] = '\0';
			}
			else if(strLineMask == Ionex1_0_MaskString::ioComment)
			{
				strLine.copy(m_header.ioCommentList,60,0);
				m_header.ioCommentList[60] = '\0';
			}		    
			else if(strLineMask == Ionex1_0_MaskString::ioEpochFirstMap)
			{
				sscanf(strLine.c_str(),"%6d%6d%6d%6d%6d%lf",
					&m_header.epochFirst.year,
					&m_header.epochFirst.month,
					&m_header.epochFirst.day,
					&m_header.epochFirst.hour,
					&m_header.epochFirst.minute,
					&m_header.epochFirst.second);

					//&int(m_header.epochFirst.second));
			}

			else if(strLineMask == Ionex1_0_MaskString::ioEpochLastMap)
			{
				sscanf(strLine.c_str(),"%6d%6d%6d%6d%6d%lf",
					&m_header.epochLast.year,
					&m_header.epochLast.month,
					&m_header.epochLast.day,
					&m_header.epochLast.hour,
					&m_header.epochLast.minute,
					&m_header.epochLast.second);
			}

			else if(strLineMask == Ionex1_0_MaskString::ioInterval)
			{
				sscanf(strLine.c_str(),"%6d",&m_header.Interval);
			}

			else if(strLineMask == Ionex1_0_MaskString::ioOfMapsinFile)
			{
				sscanf(strLine.c_str(),"%6d",&m_header.NumberOfMaps);
			}

			else if(strLineMask == Ionex1_0_MaskString::ioMappingFun)
			{
				strLine.copy(m_header.ioMappingFunction,60,0);
				m_header.ioMappingFunction[60] = '\0';
			}

			else if(strLineMask == Ionex1_0_MaskString::ioElevationCutOff)
			{
				sscanf(strLine.c_str(),"%8lf",&m_header.ElevationCutoff);
			}

			else if(strLineMask == Ionex1_0_MaskString::ioObservablesUsed)
			{
				strLine.copy(m_header.ioObservablesUsed,60,0);
				m_header.ioObservablesUsed[60] = '\0';
			}

			else if(strLineMask == Ionex1_0_MaskString::ioOfStastions)
			{
				sscanf(strLine.c_str(),"%6d",&m_header.NumberOfStations);
			}

			else if(strLineMask == Ionex1_0_MaskString::ioOfSatellites)
			{
				sscanf(strLine.c_str(),"%6d",&m_header.NumberOfSatellites);
			}

			else if(strLineMask == Ionex1_0_MaskString::ioBaseRadius)
			{
				sscanf(strLine.c_str(),"%8lf",&m_header.BaseRadius);
			}

			else if(strLineMask == Ionex1_0_MaskString::ioHgt1Hgt2Dhgt)
			{
				sscanf(strLine.c_str(),"%*2c%6lf%6lf%6lf",&m_header.Height1,&m_header.Height2,&m_header.DeltaHeight);
			}

			else if(strLineMask == Ionex1_0_MaskString::ioLat1Lat2Dlat)
			{
				sscanf(strLine.c_str(),"%*2c%6lf%6lf%6lf",&m_header.Lat1,&m_header.Lat2,&m_header.DeltaLat);
			}

			else if(strLineMask == Ionex1_0_MaskString::ioLon1Lon2Dlon)
			{
				sscanf(strLine.c_str(),"%*2c%6lf%6lf%6lf",&m_header.Lon1,&m_header.Lon2,&m_header.DeltaLon);
			}

			else if(strLineMask == Ionex1_0_MaskString::ioExponent)
			{
				sscanf(strLine.c_str(),"%6d",&m_header.Exponent);
			}

			else if(strLineMask == Ionex1_0_MaskString::ioEndofHeader)
			{
				bFlag=false;
			}
		} 
       /*开始循环读取数据文件，直到END OF FILE*/
		bFlag = true;
		m_data.clear();
	    int k = -1;         //记录当前取行数
		int LonNum = (int)( (m_header.Lon2 - m_header.Lon1) / m_header.DeltaLon +1);     //头文件规定的Grid列数
		int LatNum = (int)( (m_header.Lat2 - m_header.Lat1) / m_header.DeltaLat +1);     //头文件规定的Grid行数
		int MapNum;     	//记录当前正在读取第MapNum个IONMap	
		Ionex1_0_Map  IonMap;
		while(bFlag)
		{	
			char line[100];
			fgets(line,100,pIONfile);
		    string strLineMask = line;
		    string strLine     = line;
			strLineMask.erase(0, 60); // 从第 0 个元素开始，删除 60 个
			// 剔除 \n
			size_t nPos_n = strLineMask.find('\n');
			if(nPos_n < strLineMask.length())
				strLineMask.erase(nPos_n, 1);
			// 超过20位，截取20位
			while(strLineMask.length() > 20)
				strLineMask.erase(strLineMask.length() - 1, 1);
			// 补齐20位
			if(strLineMask.length() < 20) // strLineMask.length 包含最后的'\0'
				strLineMask.append(20-strLineMask.length(),' ');

			if(strLineMask == "START OF TEC MAP    ")
			{
				sscanf(strLine.c_str(),"%6d",&MapNum);
				
				continue;
			}
			else if(strLineMask == "EPOCH OF CURRENT MAP")
			{
				sscanf(strLine.c_str(),"%6d%6d%6d%6d%6d%lf",&IonMap.t.year,
					                                        &IonMap.t.month,
					                                        &IonMap.t.day,
													        &IonMap.t.hour,
													        &IonMap.t.minute,
													        &IonMap.t.second);
				
				
				continue;
			}
			else if(strLineMask == "LAT/LON1/LON2/DLON/H")
			{
				IonMap.Grid.Init(LatNum, LonNum);     //初始化IonMap.Grid，定义行列并给m_pdata分配内存

				while(1)
				{					
					k++;
					double Lat;
					double Lon1;
					double Lon2;
					double DeltaLon;
					double Height;
					sscanf(strLine.c_str(),"%*2c%6lf%6lf%6lf%6lf%6lf",&Lat,&Lon1,&Lon2,&DeltaLon,&Height);
					//这里先不考虑 LAT/LON1/LON2/DLON/H与头文件不吻合的情况
					int nLine =  LonNum / 16;				
					int nResidue = LonNum % 16;

					for(int i = 1; i <= nLine; i++)
					{										
						fgets(line,100,pIONfile);
						strLine = line;
						for(int j = 1; j <= 16; j++)
						{						
							char strDelay[6];
							int delay;
							strLine.copy(strDelay,5,5 * (j-1));
							strDelay[5] = '\0';
							sscanf(strDelay,"%5d",&delay);
							IonMap.Grid.SetElement(k,16 * (i - 1) + j - 1,delay);							
						}									
					}
					
					if(nResidue > 0)
					{
						fgets(line,100,pIONfile);
						strLine = line;

						for(int j = 1; j <= nResidue; j++)
						{
							char strDelay[6];
							int delay;
							strLine.copy(strDelay,5,5 * (j-1));
							strDelay[5] = '\0';
							sscanf(strDelay,"%5d",&delay);
							IonMap.Grid.SetElement(k, 16 * nLine + j - 1, delay);
						}
					}

					fgets(line,100,pIONfile);
					strLineMask = line;
					strLine = line;
					strLineMask.erase(0, 60); // 从第 0 个元素开始，删除 60 个
					// 剔除 \n
					size_t nPos_n = strLineMask.find('\n');
					if(nPos_n < strLineMask.length())
						strLineMask.erase(nPos_n, 1);
					// 超过20位，截取20位
					while(strLineMask.length() > 20)
						strLineMask.erase(strLineMask.length() - 1, 1);
					// 补齐20位
					if(strLineMask.length() < 20) // strLineMask.length 包含最后的'\0'
						strLineMask.append(20 - strLineMask.length(),' ');

					if(strLineMask == "LAT/LON1/LON2/DLON/H")
					{
						continue;
					}
						
					else if(strLineMask == "END OF TEC MAP      ")
					{
						//printf("%d %d\n",IonMap.Grid.GetNumRows(),IonMap.Grid.GetNumColumns());
						IonMap.Lat1 = m_header.Lat1;
						IonMap.Lat2 = m_header.Lat2;
						IonMap.DeltaLat = m_header.DeltaLat;
						IonMap.Lon1 = m_header.Lon1;
						IonMap.Lon2 = m_header.Lon2;
						IonMap.DeltaLon = m_header.DeltaLon;
						IonMap.Height = m_header.Height1;
						m_data.push_back(IonMap);
						//printf("%f\n",m_data[MapNum - 1].Grid.GetElement(3,6));
						break;
					}					
				}
				k = -1;
			}
			else if(strLineMask == "START OF RMS MAP    ")
			{
				bFlag = false;
			}
			else if(strLineMask == "END OF FILE         ")
			{
				bFlag = false;  
			}
			Ionex1_0_Map  IonMap;
		}
		fclose(pIONfile);
		
		return !bFlag;
	}
	// 子程序名称： getVTEC  
	// 功能：插值得到t时，纬度Lat、经度Lon的VTEC（垂直电子总量）
	// 变量类型： t               :  UT
	//            Lat             :  穿刺点所在位置纬度
	//            Lon             :  穿刺点所在位置经度
	//            VTEC            :  垂直电离层总量
	// 输入：t，Lat，Lon，VTEC
	// 输出：VTEC
	// 语言：C++
	// 创建者：高永飞
	// 创建时间：2013/10/23
	// 版本时间：2013/10/23
	// 修改记录：
	// 备注：插值方法使用“IONEX：The IONosphere Map Exchange Format Version 1”中方法（2）
	bool Ionex1_0_File::getVTEC(DayTime t, double Lat, double Lon, double &VTEC)
	{
		double t0_second;
		double t_second;		
		int MapOrder;

		t0_second = m_data[0].t.hour * 3600 + m_data[0].t.minute * 60 + m_data[0].t.second;
		t_second = t.hour * 3600 + t.minute * 60 + t.second;

		MapOrder = (int)((t_second - t0_second) / m_header.Interval);

		double T1, T2;
		T1 = m_data[MapOrder].t.hour * 3600 + m_data[MapOrder].t.minute * 60 + m_data[MapOrder].t.second;
		T2 = m_data[MapOrder + 1].t.hour * 3600 + m_data[MapOrder + 1].t.minute * 60 + m_data[MapOrder + 1].t.second;		
		
		double p, q;               //网格点加权系数
		int    LatNum, LonNum;     //E00点行列数
		double VTEC1, VTEC2;       //相邻时间的VTEC
        
		//考虑到穿刺点经纬度可能在电离层网格经纬度范围之外，分情况讨论
		if(Lat >= m_header.Lat1)
		{
			if(Lon > 180)		
				Lon = Lon - 360;
			else if(Lon < -180)
				Lon = Lon + 360;

			LatNum = 0;
			LonNum = int((Lon - m_header.Lon1) / m_header.DeltaLon);

			p = (Lon - m_header.Lon1 - LonNum * m_header.DeltaLon) / m_header.DeltaLon;
			q = 0.0;
			VTEC1 = (1 - p) * (1 - q) * m_data[MapOrder].Grid.GetElement(0 , LonNum) +
			        p * (1 - q) * m_data[MapOrder].Grid.GetElement(0 , LonNum + 1) +
				    q * (1 - p) * m_data[MapOrder].Grid.GetElement(0 , LonNum) + 
				    p * q * m_data[MapOrder].Grid.GetElement(0 , LonNum + 1);
			VTEC2 = (1 - p) * (1 - q) * m_data[MapOrder + 1].Grid.GetElement(0 , LonNum) +
			        p * (1 - q) * m_data[MapOrder + 1].Grid.GetElement(0 , LonNum + 1) +
				    q * (1 - p) * m_data[MapOrder + 1].Grid.GetElement(0 , LonNum) + 
				    p * q * m_data[MapOrder + 1].Grid.GetElement(0 , LonNum + 1);
		}
		else if(Lat <= m_header.Lat2)
		{
			if(Lon > 180)		
				Lon = Lon - 360;
			else if(Lon < -180)
				Lon = Lon + 360;

			LatNum = m_data[MapOrder].Grid.GetNumRows() - 1;
			LonNum = int((Lon - m_header.Lon1) / m_header.DeltaLon);

			p = (Lon - m_header.Lon1 - LonNum * m_header.DeltaLon) / m_header.DeltaLon;
			q = 0.0;
			VTEC1 = (1 - p) * (1 - q) * m_data[MapOrder].Grid.GetElement(m_data[MapOrder].Grid.GetNumRows() - 1 , LonNum) +
			        p * (1 - q) * m_data[MapOrder].Grid.GetElement(m_data[MapOrder].Grid.GetNumRows() - 1, LonNum + 1) +
				    q * (1 - p) * m_data[MapOrder].Grid.GetElement(m_data[MapOrder].Grid.GetNumRows() - 1, LonNum) + 
				    p * q * m_data[MapOrder].Grid.GetElement(m_data[MapOrder].Grid.GetNumRows() - 1, LonNum + 1);
			VTEC2 = (1 - p) * (1 - q) * m_data[MapOrder + 1].Grid.GetElement(m_data[MapOrder + 1].Grid.GetNumRows() - 1, LonNum) +
			        p * (1 - q) * m_data[MapOrder + 1].Grid.GetElement(m_data[MapOrder + 1].Grid.GetNumRows() - 1, LonNum + 1) +
				    q * (1 - p) * m_data[MapOrder + 1].Grid.GetElement(m_data[MapOrder + 1].Grid.GetNumRows() - 1, LonNum) + 
				    p * q * m_data[MapOrder + 1].Grid.GetElement(m_data[MapOrder + 1].Grid.GetNumRows() - 1, LonNum + 1);
		}
		else
		{
			LatNum = int((Lat - m_header.Lat1) / m_header.DeltaLat);
			if(Lon > 180)		
				Lon = Lon - 360;
			else if(Lon < -180)
				Lon = Lon + 360;

			LonNum = int((Lon - m_header.Lon1) / m_header.DeltaLon);
			

			p = (Lon - m_header.Lon1 - LonNum * m_header.DeltaLon) / m_header.DeltaLon;
			q = (Lat - m_header.Lat1 - LatNum * m_header.DeltaLat) / m_header.DeltaLat;
			VTEC1 = (1 - p) * (1 - q) * m_data[MapOrder].Grid.GetElement(LatNum , LonNum) +
			      p * (1 - q) * m_data[MapOrder].Grid.GetElement(LatNum , LonNum + 1) +
				  q * (1 - p) * m_data[MapOrder].Grid.GetElement(LatNum + 1 , LonNum) + 
				  p * q * m_data[MapOrder].Grid.GetElement(LatNum + 1 , LonNum + 1);
			VTEC2 = (1 - p) * (1 - q) * m_data[MapOrder + 1].Grid.GetElement(LatNum , LonNum) +
			        p * (1 - q) * m_data[MapOrder + 1].Grid.GetElement(LatNum , LonNum + 1) +
				    q * (1 - p) * m_data[MapOrder + 1].Grid.GetElement(LatNum + 1 , LonNum) + 
				    p * q * m_data[MapOrder + 1].Grid.GetElement(LatNum + 1 , LonNum + 1);
		}

		VTEC = ( (T2 - t_second) / (T2 - T1) * VTEC1 + (t_second - T1) / (T2 - T1) * VTEC2 ) * pow(10.0, m_header.Exponent);
		return true;
	}

}
