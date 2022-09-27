#include "Doris2_2_EditedObsFile.hpp"
#include "constDef.hpp"
#include <math.h>

namespace NUDTTK
{
	namespace DORIS
	{
		Doris2_2_EditedObsFile::Doris2_2_EditedObsFile(void)
		{
		}

		Doris2_2_EditedObsFile::~Doris2_2_EditedObsFile(void)
		{
		}

		int Doris2_2_EditedObsFile::isValidDoris2_2_EditedObsLine(string strLine, FILE * pFile)
		{
			Doris2_2_EditedObsLine  dataLine;
			if(pFile != NULL)
			{ // 判断是否为文件末尾
				if(feof(pFile) || (strLine.find("EOF") < strLine.length()))
					return 0;
			}
			char syear[2+1];
			char sDoy[3+1];
			char sSecond_int[5+1];
			char sSecond_fra[6+1];
			char sCout_interval[10+1];
			char sRang_rate[11+1];
			char sSurface_pressure[4+1];
			char sSurface_temperature[3+1];
			char sRela_humidity[3+1];
            char sObsev_RMS[6+1];
			char sIono_correction[8+1];
			char sTrop_correction[7+1];
			sscanf(strLine.c_str(),"%7c%2d%1d%1d%4c%*1c%2c%3c%5c%6c%1d%1d%1d%10c%11c%4c%3c%3c%6c%8c%7c%1d%1d%1d%6d",
				                    dataLine.Sat_ID,
									&dataLine.Meas_type,
			                        &dataLine.Time_location,
                                    &dataLine.Time_type,
									dataLine.Station_ID,
									syear,
									sDoy,
									sSecond_int,
									sSecond_fra,
									&dataLine.Iono_apply,
									&dataLine.Trop_apply,
									&dataLine.Point_infor,
									sCout_interval,
									sRang_rate,
									sSurface_pressure,
									sSurface_temperature,
									sRela_humidity,
									sObsev_RMS,
									sIono_correction,
									sTrop_correction,
									&dataLine.Beacon_type,
									&dataLine.Meteor_source,
									&dataLine.Channel_ID,
									&dataLine.mass_correction);
            syear[2] = '\0';
	        sDoy[3] = '\0';
	        sSecond_int[5] = '\0';
	        sSecond_fra[6] = '\0';
	        sCout_interval[10] = '\0';
	        sRang_rate[11] = '\0';
	        sSurface_pressure[4] = '\0';
	        sSurface_temperature[3] = '\0';
	        sRela_humidity[3] = '\0';
            sObsev_RMS[6] = '\0';
	        sIono_correction[8] = '\0';
	        sTrop_correction[7] = '\0';
			sscanf(syear,"%d",&dataLine.Year);
			sscanf(sDoy,"%d",&dataLine.Doy);
			sscanf(sSecond_int,"%d",&dataLine.Second_int);
			sscanf(sSecond_fra,"%d",&dataLine.Second_fra);
			sscanf(sCout_interval,"%lf",&dataLine.Cout_interval);
			sscanf(sRang_rate,"%lf",&dataLine.Range_rate);
			sscanf(sSurface_pressure,"%d",&dataLine.Surface_pressure);
			sscanf(sSurface_temperature,"%d",&dataLine.Surface_temperature);
			sscanf(sRela_humidity,"%d",&dataLine.Rela_humidity);
			sscanf(sObsev_RMS,"%d",&dataLine.Obsev_RMS);
			sscanf(sIono_correction,"%d",&dataLine.Iono_correction);
			sscanf(sTrop_correction,"%d",&dataLine.Trop_correction);
			int nflag = 1;
			if (dataLine.Meas_type==34||dataLine.Meas_type==38||dataLine.Meas_type==39)
			{
				if (dataLine.Time_location>3||dataLine.Time_location<0)
					nflag=2;
				if (dataLine.Time_type>9||dataLine.Time_type<0)
					nflag=2;
				if (dataLine.Year>100||dataLine.Year<0)
					nflag=2;
			    if (dataLine.Doy>366||dataLine.Doy<0)
				    nflag=2;
			    if (dataLine.Second_int>86400||dataLine.Second_int<0)
				    nflag=2;
			    if (dataLine.Second_fra<0)
				    nflag=2;
				if (dataLine.Iono_apply>1||dataLine.Iono_apply<0)
					nflag=2;
				if (dataLine.Trop_apply>1||dataLine.Trop_apply<0)
					nflag=2;
				if (dataLine.Point_infor>4||dataLine.Point_infor<0)
					nflag=2;
				if (dataLine.Beacon_type>3||dataLine.Beacon_type<0)
					nflag=2;
				if (dataLine.Meteor_source>9||dataLine.Meteor_source<0)
					nflag=2;
				if (dataLine.Channel_ID<0)
					nflag=2;
				return nflag;
			}
			else
			{
				nflag = 2;
				return nflag;
			}
		}

		bool Doris2_2_EditedObsFile::open(string  strEditedObsFileName)
		{
			char line[100];
			m_data.clear();
			FILE * pFile = fopen(strEditedObsFileName.c_str(),"r");
			if(pFile == NULL) 
				return false;
			bool bflag=true;
			fgets(line,sizeof(line),pFile);
			while (bflag)
			{
				int nflag = isValidDoris2_2_EditedObsLine(line, pFile);
				if (nflag==0)
				{
					bflag=false;
				}
				else if (nflag==1)
				{
					Doris2_2_EditedObsLine  predata;
					char syear[2+1];
			        char sDoy[3+1];
			        char sSecond_int[5+1];
			        char sSecond_fra[6+1];
			        char sCout_interval[10+1];
			        char sRang_rate[11+1];
			        char sSurface_pressure[4+1];
			        char sSurface_temperature[3+1];
			        char sRela_humidity[3+1];
                    char sObsev_RMS[6+1];
			        char sIono_correction[8+1];
			        char sTrop_correction[7+1];
			        sscanf(line,"%7c%2d%1d%1d%4c%*1c%2c%3c%5c%6c%1d%1d%1d%10c%11c%4c%3c%3c%6c%8c%7c%1d%1d%1d%6d",
				                    predata.Sat_ID,
									&predata.Meas_type,
			                        &predata.Time_location,
                                    &predata.Time_type,
									predata.Station_ID,
									syear,
									sDoy,
									sSecond_int,
									sSecond_fra,
									&predata.Iono_apply,
									&predata.Trop_apply,
									&predata.Point_infor,
									sCout_interval,
									sRang_rate,
									sSurface_pressure,
									sSurface_temperature,
									sRela_humidity,
									sObsev_RMS,
									sIono_correction,
									sTrop_correction,
									&predata.Beacon_type,
									&predata.Meteor_source,
									&predata.Channel_ID,
									&predata.mass_correction);
					syear[2] = '\0';
			        sDoy[3] = '\0';
			        sSecond_int[5] = '\0';
			        sSecond_fra[6] = '\0';
			        sCout_interval[10] = '\0';
			        sRang_rate[11] = '\0';
			        sSurface_pressure[4] = '\0';
			        sSurface_temperature[3] = '\0';
			        sRela_humidity[3] = '\0';
                    sObsev_RMS[6] = '\0';
			        sIono_correction[8] = '\0';
			        sTrop_correction[7] = '\0';
			        sscanf(syear,"%d",&predata.Year);
			        sscanf(sDoy,"%d",&predata.Doy);
			        sscanf(sSecond_int,"%d",&predata.Second_int);
			        sscanf(sSecond_fra,"%d",&predata.Second_fra);
			        sscanf(sCout_interval,"%lf",&predata.Cout_interval);
			        sscanf(sRang_rate,"%lf",&predata.Range_rate);
			        sscanf(sSurface_pressure,"%d",&predata.Surface_pressure);
			        sscanf(sSurface_temperature,"%d",&predata.Surface_temperature);
			        sscanf(sRela_humidity,"%d",&predata.Rela_humidity);
			        sscanf(sObsev_RMS,"%d",&predata.Obsev_RMS);
			        sscanf(sIono_correction,"%d",&predata.Iono_correction);
			        sscanf(sTrop_correction,"%d",&predata.Trop_correction);
					m_data.push_back(predata);
					fgets(line,sizeof(line),pFile);
				}
				else
				{
					fgets(line,sizeof(line),pFile);
				}
			}
			fclose(pFile);
			
			if (m_data.size() > 0)
				return true;
			else
				return false;
		}

		bool Doris2_2_EditedObsFile::write(string strEditedObsFileName)
		{
			if (m_data.size() == 0)
				return false;
			FILE *wpreprocessedFile = fopen(strEditedObsFileName.c_str(), "w");
			for (size_t i = 0; i < m_data.size(); i++)
			{
				fprintf(wpreprocessedFile,"%7s%2d%1d%1d%4s %02d%3d%5d%6d%1d%1d%1d%10.0lf%11.0lf%4d%3d%3d%6d%8d%7d%1d%1d%1d%6d\n",
				                    m_data[i].Sat_ID,
									m_data[i].Meas_type,
			                        m_data[i].Time_location,
                                    m_data[i].Time_type,
									m_data[i].Station_ID,
									m_data[i].Year,
									m_data[i].Doy,
									m_data[i].Second_int,
									m_data[i].Second_fra,
									m_data[i].Iono_apply,
									m_data[i].Trop_apply,
									m_data[i].Point_infor,
									m_data[i].Cout_interval,
									m_data[i].Range_rate,
                                    m_data[i].Surface_pressure,
									m_data[i].Surface_temperature,
									m_data[i].Rela_humidity,
									m_data[i].Obsev_RMS,
									m_data[i].Iono_correction,
									m_data[i].Trop_correction,
									m_data[i].Beacon_type,
									m_data[i].Meteor_source,
									m_data[i].Channel_ID,
									m_data[i].mass_correction);
			}
			fclose(wpreprocessedFile);
			return true;
		}

		bool Doris2_2_EditedObsFile::cutdata(TAI t0, TAI t1)
		{
			int k = 0;
			while(k < int(m_data.size()))
			{
                TAI t = m_data[k].GetTime();
				if( t - t0 >= 0 && t - t1 <= 0)
				{
					k++;
					continue;
				}
				else
				{
					m_data.erase(m_data.begin() + k);
					continue;
				}
			}
			return true;
		}

		// 子程序名称： split  
		// 功能：将长周期数据拆分成以天为单位
		// 变量类型：strName   : 卫星名称 
		//           period    : 周期
		// 输入：strName
		// 输出：
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2011/07/11
		// 版本时间：
		// 修改记录：
		// 备注： 
		void  Doris2_2_EditedObsFile::split(string strName, double period)
		{
			if(m_data.size() <= 0)
				return;
			TAI t_Begin = m_data[0].GetTime();
			t_Begin.hour = 0;
			t_Begin.minute = 0;
			t_Begin.second = 0.0;
			TAI t_End = m_data[m_data.size() - 1].GetTime();
			t_End.hour = 0;
			t_End.minute = 0;
			t_End.second = 0.0;
			TAI t = t_Begin;
			while(t - t_End <= 0.0)
			{
                char szFile[MAX_PATH];
			    sprintf(szFile,"%s_%4d%02d%02d_add3h.dat", strName.c_str(), t.year, t.month, t.day);
				FILE *wpreprocessedFile = fopen(szFile, "w");
				for (size_t i = 0; i < m_data.size(); i++)
				{
					double spanSeconds = m_data[i].GetTime() - t;
					if(spanSeconds >= 0 - 3600.0 * 3 && spanSeconds < period + 3600.0 * 3)
					{// 两端各延伸 3 个小时
						if(m_data[i].Iono_apply == 21 || m_data[i].Second_fra > 999999)
							printf("%d", m_data[i].Iono_apply);
						fprintf(wpreprocessedFile,"%7s%2d%1d%1d%4s %02d%3d%5d%6d%1d%1d%1d%10.0lf%11.0lf%4d%3d%3d%6d%8d%7d%1d%1d%1d%6d\n",
											m_data[i].Sat_ID,
											m_data[i].Meas_type,
											m_data[i].Time_location,
											m_data[i].Time_type,
											m_data[i].Station_ID,
											m_data[i].Year,
											m_data[i].Doy,
											m_data[i].Second_int,
											m_data[i].Second_fra,
											m_data[i].Iono_apply,
											m_data[i].Trop_apply,
											m_data[i].Point_infor,
											m_data[i].Cout_interval,
											m_data[i].Range_rate,
											m_data[i].Surface_pressure,
											m_data[i].Surface_temperature,
											m_data[i].Rela_humidity,
											m_data[i].Obsev_RMS,
											m_data[i].Iono_correction,
											m_data[i].Trop_correction,
											m_data[i].Beacon_type,
											m_data[i].Meteor_source,
											m_data[i].Channel_ID,
											m_data[i].mass_correction);
					}
				}
				fclose(wpreprocessedFile);
				t = t + period;
			}  
		}

		// 子程序名称： getObsStationList  
		// 功能：根据行数据, 获得测站数据列表
		// 变量类型：obsStationList           : 测站数据列表 
		// 输入：
		// 输出：obsStationList
		// 语言：C++
		// 创建者：谷德峰
		// 创建时间：2011/07/11
		// 版本时间：
		// 修改记录：
		// 备注： 
        bool  Doris2_2_EditedObsFile::getObsStationList(vector<Doris2_2_EditedObsStation>& obsStationList)
		{
			if(m_data.size() <= 0)
				return false;
			Doris2_2_EditedObsStation stationList[MAX_ID_DORISSTATION + 1];
			for(int i = 0; i < MAX_ID_DORISSTATION + 1; i++)
			{
				stationList[i].id_sation = i;
				stationList[i].obs.clear();
			}
			int k = 0;
			char station_id_now[5];
			strcpy(station_id_now, m_data[0].Station_ID);
			int  index_station_now = string2DorisStationId(station_id_now);
			while(k < int(m_data.size()))
			{
				if(station_id_now[0] == m_data[k].Station_ID[0]
				&& station_id_now[1] == m_data[k].Station_ID[1]
				&& station_id_now[2] == m_data[k].Station_ID[2]
				&& station_id_now[3] == m_data[k].Station_ID[3])
				{
					if(index_station_now != DORISSTATION_UNKNOWN)
					{
						stationList[index_station_now].obs.insert(Doris2_2_EditedObsEpochMap::value_type(m_data[k].GetTime(), m_data[k]));
					}
				}
				else
				{
					// 更新当前弧段信息	
					strcpy(station_id_now, m_data[k].Station_ID);
					index_station_now = string2DorisStationId(station_id_now);

					if(index_station_now != DORISSTATION_UNKNOWN)
					{
						stationList[index_station_now].obs.insert(Doris2_2_EditedObsEpochMap::value_type(m_data[k].GetTime(), m_data[k]));
					}
				}
				k++;
			}
			obsStationList.clear();
			for(int i = 0; i < MAX_ID_DORISSTATION + 1; i++)
			{
				if(stationList[i].obs.size() > 0)
				{
					obsStationList.push_back(stationList[i]);
				}
			}
			return true;
		}

		bool Doris2_2_EditedObsFile::getObsEpochList(vector<Doris2_2_EditedObsStation> obsStationList, vector<Doris2_2_EditedObsEpoch>& obsEpochList)
		{
			obsEpochList.clear();
            while(1)
			{
				Doris2_2_EditedObsEpoch obsEpoch;
				obsEpoch.obs.clear();
				// 寻找第一个时间点
				bool bFind = false;
				TAI t_now;
				for(int i = 0; i < int(obsStationList.size()); i++)
				{
					if(obsStationList[i].obs.size() > 0)
					{
						if(!bFind)
							t_now = obsStationList[i].obs.begin()->first;
						else
							t_now = t_now - obsStationList[i].obs.begin()->first > 0 ? obsStationList[i].obs.begin()->first : t_now;
						bFind = true;
					}
				}
				if(!bFind)
					break;
				else
				{
					obsEpoch.t = t_now;
				}
				// 查找每一个与t_now相等的点
				for(int i = 0; i < int(obsStationList.size()); i++)
				{
					if(obsStationList[i].obs.size() > 0)
					{
						Doris2_2_EditedObsEpochMap::iterator it = obsStationList[i].obs.begin();
                        if(fabs(t_now - it->first) <= 1.0E-12)
						{
							// 添加记录点
                            obsEpoch.obs.insert(Doris2_2_EditedObsStationMap::value_type(obsStationList[i].id_sation, it->second));
							obsStationList[i].obs.erase(it);
						}
					}
				}
				if(obsEpoch.obs.size() > 0)
				{
					obsEpochList.push_back(obsEpoch);
				}
			}
			return true;
		}
	}
}
