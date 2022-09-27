#include "TQUXBObsFile.hpp"
namespace NUDTTK
{
	namespace TQPod
	{
		TQUXBObsFile::TQUXBObsFile(void)
		{
		}

		TQUXBObsFile::~TQUXBObsFile(void)
		{
		}

		bool TQUXBObsFile::write(string strUXBObsFileName)
		{
			FILE* pFile = fopen(strUXBObsFileName.c_str(), "w+");
			for(size_t s_i = 0; s_i < m_data.size(); s_i++)
			{
				fprintf(pFile, "%4s %4s  %4.2f  %s %8.2f%8.2f%30.10f%20.10f%20.10f%20.10f%20.10f%20.10f\n", 
							   m_data[s_i].staName.c_str(),
							   m_data[s_i].satName.c_str(),
                               m_data[s_i].time_Doppler,
					           m_data[s_i].t.toString().c_str(),
							   m_data[s_i].Elevation,
							   m_data[s_i].Azimuth,
							   m_data[s_i].R1,
							   m_data[s_i].V1,
							   m_data[s_i].R0s,
							   m_data[s_i].R0g,
							   m_data[s_i].err_Trop,
							   m_data[s_i].err_Iono);
			}
			fclose(pFile);
			return true;
		}
		//2022.04.20安子聪添加写入实测数据（类似深空网格式）
        bool TQUXBObsFile::write1(string strUXBObsFileName)
		{
			FILE* pFile = fopen(strUXBObsFileName.c_str(), "w+");
			for(size_t s_i = 0; s_i < m_data1.size(); s_i++)
			{

				 if(m_data1[s_i].typeofdata == 12 && m_data1[s_i].shifoushiyong == 1)
				 {
						 if(m_data1[s_i].staName1 == 3121)
						 {
						//m_data1[s_i].oc_doppler = m_data1[s_i].cancha;
						fprintf(pFile, "%s %s  %4.2f  %s %8.2f %d %d %20.10f  %d %d %d %d\n", 
									   m_data1[s_i].a.c_str(),
									   m_data1[s_i].d.c_str(),
									   m_data1[s_i].time_Doppler,
									   m_data1[s_i].t_TDB.toString().c_str(),
									   (m_data1[s_i].elevation1 + m_data1[s_i].elevation2) * 0.5,
									   m_data1[s_i].Fiveth,//方位角
									   m_data1[s_i].Fiveth,//测距用0代替
									   0.5 * m_data1[s_i].obs,  // 测速值
									   m_data1[s_i].Fiveth,
									   m_data1[s_i].Fiveth,
									   m_data1[s_i].Fiveth,
									   m_data1[s_i].Fiveth);
						 }
						 else if(m_data1[s_i].staName1 == 3201)
						 {
						//m_data1[s_i].oc_doppler = m_data1[s_i].cancha;
						fprintf(pFile, "%s %s  %4.2f  %s %8.2f %d %d %20.10f  %d %d %d %d\n", 
									   m_data1[s_i].b.c_str(),
									   m_data1[s_i].d.c_str(),
									   m_data1[s_i].time_Doppler,
									   m_data1[s_i].t_TDB.toString().c_str(),
									   (m_data1[s_i].elevation1 + m_data1[s_i].elevation2) * 0.5,
									   m_data1[s_i].Fiveth,//方位角
									   m_data1[s_i].Fiveth,//测距用0代替
									   0.5 * m_data1[s_i].obs,  // 测速值
									   m_data1[s_i].Fiveth,
									   m_data1[s_i].Fiveth,
									   m_data1[s_i].Fiveth,
									   m_data1[s_i].Fiveth);
						 }
						 else
						 {
						 fprintf(pFile, "%s %s  %4.2f  %s %8.2f %d %d %20.10f  %d %d %d %d\n", 
									   m_data1[s_i].c.c_str(),
									   m_data1[s_i].d.c_str(),
									   m_data1[s_i].time_Doppler,
									   m_data1[s_i].t_TDB.toString().c_str(),
									   (m_data1[s_i].elevation1 + m_data1[s_i].elevation2) * 0.5,
									   m_data1[s_i].Fiveth,//方位角
									   m_data1[s_i].Fiveth,//测距用0代替
									   0.5 * m_data1[s_i].obs,  // 测速值
									   m_data1[s_i].Fiveth,
									   m_data1[s_i].Fiveth,
									   m_data1[s_i].Fiveth,
									   m_data1[s_i].Fiveth);
						 }

				 }
				else if(m_data1[s_i].typeofdata == 11 && m_data1[s_i].shifoushiyong == 1)
				{
					   if(m_data1[s_i].staName1 == 3121)
						 {
						//m_data1[s_i].oc_doppler = m_data1[s_i].cancha;
						fprintf(pFile, "%s %s  %4.2f  %s %8.2f %d %d %20.10f  %d %d %d %d\n", 
									   m_data1[s_i].a.c_str(),
									   m_data1[s_i].d.c_str(),
									   m_data1[s_i].time_Doppler,
									   m_data1[s_i].t_TDB.toString().c_str(),
									   (m_data1[s_i].elevation1 + m_data1[s_i].elevation2) * 0.5,
									   m_data1[s_i].Fiveth,//方位角
									   0.5 * m_data1[s_i].obs,  // 测距值
									   m_data1[s_i].Fiveth,//测速用0代替
									   m_data1[s_i].Fiveth,
									   m_data1[s_i].Fiveth,
									   m_data1[s_i].Fiveth,
									   m_data1[s_i].Fiveth);
						 }
						 else if(m_data1[s_i].staName1 == 3201)
						 {
						//m_data1[s_i].oc_doppler = m_data1[s_i].cancha;
						fprintf(pFile, "%s %s  %4.2f  %s %8.2f %d %d %20.10f  %d %d %d %d\n", 
									   m_data1[s_i].b.c_str(),
									   m_data1[s_i].d.c_str(),
									   m_data1[s_i].time_Doppler,
									   m_data1[s_i].t_TDB.toString().c_str(),
									   (m_data1[s_i].elevation1 + m_data1[s_i].elevation2) * 0.5,
									   m_data1[s_i].Fiveth,//方位角
									   0.5 * m_data1[s_i].obs,  // 测距值
									   m_data1[s_i].Fiveth,//测速用0代替
									   m_data1[s_i].Fiveth,
									   m_data1[s_i].Fiveth,
									   m_data1[s_i].Fiveth,
									   m_data1[s_i].Fiveth);
						 }
					 else
						 {
						 fprintf(pFile, "%s %s  %4.2f  %s %8.2f %d %d %20.10f  %d %d %d %d\n", 
									   m_data1[s_i].c.c_str(),
									   m_data1[s_i].d.c_str(),
									   m_data1[s_i].time_Doppler,
									   m_data1[s_i].t_TDB.toString().c_str(),
									   (m_data1[s_i].elevation1 + m_data1[s_i].elevation2) * 0.5,
									   m_data1[s_i].Fiveth,//方位角
									   0.5 * m_data1[s_i].obs,  // 测距值
									   m_data1[s_i].Fiveth,//测速用0代替
									   m_data1[s_i].Fiveth,
									   m_data1[s_i].Fiveth,
									   m_data1[s_i].Fiveth,
									   m_data1[s_i].Fiveth);
						 }
				}
				else
				{
					continue;
				}
		   }
			fclose(pFile);
			return true;
		}
		// 读取文件数据 
		bool TQUXBObsFile::open(string strUXBObsFileName)
		{
			bool bflag = true;
			FILE* pFile = fopen(strUXBObsFileName.c_str(),"r+t");
			if (pFile == NULL)
				return false;
			m_data.clear();
			char line[500];
			while(!feof(pFile))
			{
				TQUXBObsLine obsLine; 
				if(fgets(line,500,pFile))
				{
					char staN[100];
					char satN[100];
					char timeD[100];
					char year[100];
					char month[100];
					char day[100];
					char hour[100];
					char minute[100];
					char second[100];
					char elevation[100];
					char azimuth[100];
					char R1[100];
					char V1[100];
					char R0s[100];
					char R0g[100];
					char err_Trop[100];
					char err_Iono[100];

					sscanf(line,"%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s",
						         staN,
								 satN,
								 timeD,
								 year,
								 month,
								 day,
								 hour,
								 minute,
								 second,
								 elevation,
								 azimuth,
								 R1,
								 V1,
								 R0s,
								 R0g,
								 err_Trop,
								 err_Iono);

					obsLine.staName      = staN;
					obsLine.satName      = satN;
					obsLine.time_Doppler = atof(timeD);
					obsLine.t.year       = atoi(year);
					obsLine.t.month      = atoi(month);
					obsLine.t.day        = atoi(day);
					obsLine.t.hour       = atoi(hour);
					obsLine.t.minute     = atoi(minute);
					obsLine.t.second     = atof(second);
					obsLine.Elevation    = atof(elevation);
					obsLine.Azimuth      = atof(azimuth);
					obsLine.R1           = atof(R1);
					obsLine.V1           = atof(V1);
					obsLine.R0s          = atof(R0s);
					obsLine.R0g          = atof(R0g);
					obsLine.err_Trop     = atof(err_Trop);
					obsLine.err_Iono     = atof(err_Iono);
 
					// 更新 t1
					obsLine.t1.year      = obsLine.t.year;
					obsLine.t1.month     = obsLine.t.month;
					obsLine.t1.day       = obsLine.t.day;
					obsLine.t1.hour      = obsLine.t.hour;
					obsLine.t1.minute    = obsLine.t.minute;
					obsLine.t1.second    = obsLine.t.second;
                    // 更新 t0
					obsLine.t0.year      = (obsLine.t - obsLine.time_Doppler).year;
					obsLine.t0.month     = (obsLine.t - obsLine.time_Doppler).month;
					obsLine.t0.day       = (obsLine.t - obsLine.time_Doppler).day;
					obsLine.t0.hour      = (obsLine.t - obsLine.time_Doppler).hour;
					obsLine.t0.minute    = (obsLine.t - obsLine.time_Doppler).minute;
					obsLine.t0.second    = (obsLine.t - obsLine.time_Doppler).second;


					if(obsLine.t.year == 0)
				    bflag = false;
			        if(obsLine.t.month  > 12 || obsLine.t.month  <= 0)
				    bflag = false;
			        if(obsLine.t.day    > 31 || obsLine.t.day    < 0)
				    bflag = false;
			        if(obsLine.t.hour   > 24 || obsLine.t.hour   < 0)
				    bflag = false;
			        if(obsLine.t.minute > 60 || obsLine.t.minute < 0)
					bflag = false;
					if(obsLine.t.second > 60 || obsLine.t.second < 0)
					bflag = false;
					m_data.push_back(obsLine); 
				}
			}
			fclose(pFile);
			return bflag;
		}
		//2022.04.18安子聪添加读取深空网实测数据
		bool TQUXBObsFile::open1(string strUXBObsFileName)
        {
			bool bflag = true;
			FILE* pFile = fopen(strUXBObsFileName.c_str(),"r+t");
			if (pFile == NULL)
				return false;
			m_data1.clear();
			char line[500];
			while(!feof(pFile))
			{
				TQUXBObsLine1 obsLine; 
				if(fgets(line,500,pFile))
				{
	                char typeofdata[100];
					char staN[100];
					char Thrid[100];
					char satN[100];
					char Fiveth[100];
					char day[100];
					char timeofday[100];
					char obs[100];
					char lilunzhi[100];
					char cancha[100];
					char elevation1[100];
					char elevation2[100];
					char shifoushiyong[100];


					sscanf(line,"%s%s%s%s%s%s%s%s%s%s%s%s%s",
						         typeofdata,
								 staN,
								 Thrid,
								 satN,
								 Fiveth,
								 day,
								 timeofday,
								 obs,
								 lilunzhi,
								 cancha,
								 elevation1,
								 elevation2,
								 shifoushiyong);
                    obsLine.typeofdata    = atoi(typeofdata);
					obsLine.staName1      = atoi(staN);
					obsLine.Thrid         = atoi(Thrid);
					obsLine.satName1      = atoi(satN);
					obsLine.Fiveth        = atoi(Fiveth);
					obsLine.day           = atoi(day);
					obsLine.timeofday     = atof(timeofday);
					obsLine.obs           = atof(obs);
					obsLine.lilunzhi      = atof(lilunzhi);
					obsLine.cancha        = atof(cancha);
					obsLine.elevation1    = atof(elevation1);
					obsLine.elevation2    = atof(elevation1);
					obsLine.shifoushiyong = atoi(shifoushiyong);

					m_data1.push_back(obsLine); 
			   }
		   }
			fclose(pFile);
			return bflag;
		}
			//2022.04.19安子聪添加读取嫦娥星历
		bool TQUXBObsFile::open2(string strUXBObsFileName)
        {
			bool bflag = true;
			FILE* pFile = fopen(strUXBObsFileName.c_str(),"r+t");
			if (pFile == NULL)
				return false;
			m_data2.clear();
			char line[500];
			fgets(line, 100, pFile);//第1行
			fgets(line, 100, pFile);//第2行
			fgets(line, 100, pFile);//第3行
			fgets(line, 100, pFile);//第4行
			fgets(line, 100, pFile);//第5行
			fgets(line, 100, pFile);//第6行
			fgets(line, 100, pFile);//第7行
			fgets(line, 100, pFile);//第8行
			fgets(line, 100, pFile);//第9行
			fgets(line, 100, pFile);//第10行
			fgets(line, 100, pFile);//第11行
			fgets(line, 100, pFile);//第12行
			//从13行开始
			while(!feof(pFile))
			{
				TimePosVel sp3file; 
				if(fgets(line,500,pFile))
				{
					string strLineLabel = line;
					if(strLineLabel.find("DATA_STOP") != strLineLabel.npos)
					{
						break;
					}
	                char year[100];
					char month[100];
					char day[100];
					char hour[100];
					char minute[100];
					char second[100];
					char posx[100];
					char posy[100];
					char posz[100];
					char velx[100];
					char vely[100];
					char velz[100];
					sscanf(line,"%*1c%4d%*1c%2d%*1c%2d%*1c%2d%*1c%2d%*1c%6lf%s%s%s%s%s%s",
						         &sp3file.t.year,
								 &sp3file.t.month,
								 &sp3file.t.day,
								 &sp3file.t.hour,
								 &sp3file.t.minute,
								 &sp3file.t.second,
								 posx,
								 posy,
								 posz,
								 velx,
								 vely,
								 velz);
					sp3file.pos.x        = atof(posx);
					sp3file.pos.y        = atof(posy);
					sp3file.pos.z        = atof(posz);
					sp3file.vel.x       = atof(velx);
					sp3file.vel.y       = atof(vely);
					sp3file.vel.z       = atof(velz);

					m_data2.push_back(sp3file); 
			   }
		   }
			fclose(pFile);
			return bflag;
		}
    }
}
