#include "StaSscFile.hpp"

namespace NUDTTK
{
	namespace SLR
	{
		StaSscFile::StaSscFile(void)
		{
			m_Epoch = 2000.0;
		}

		StaSscFile::~StaSscFile(void)
		{
		}

		// 寻找新测站记录起点: 0 - EOF; 1 - 新测站开始; 2 - 其它
		int StaSscFile::isValidSscLine(string strLine, FILE * pSscFile)
		{
			if(pSscFile != NULL) // 判断是否为文件末尾
			{
				if(feof(pSscFile))
					return 0;
			}
			if(strLine.length() < 93)
			{
				return 2;
			}
			char szUse[4];
			char szID[5];
			sscanf(strLine.c_str(),"%*28c%3c%*1c%4c",szUse, szID);
			szUse[3] = '\0';
			szID[4]  = '\0';
			int  id = atoi(szID);
			if(strcmp(szUse,"SLR") == 0 && id > 0)
				return 1;
			return 2;
		}

		bool StaSscFile::readSscLine(string strLine, SscLine& line)
		{
			if(strLine.length() < 93)
			{
				//cout<<strLine<<endl;
				return false;
			}
			SscLine sscline;
			char szID[5];
			sscanf(strLine.c_str(),"%9c%*1c%17c%*1c%3c%*1c%4c%*1c%12lf%*1c%12lf%*1c%12lf%*1c%5lf%*1c%5lf%*1c%5lf",
									sscline.szStationFlag,
									sscline.szName,
									sscline.szUse,
									szID,
								   &sscline.x,
								   &sscline.y,
								   &sscline.z,
								   &sscline.sigma_x,
								   &sscline.sigma_y,
								   &sscline.sigma_z
								   );
			sscline.szStationFlag[8] = '\0';
			sscline.szName[16] = '\0';
			sscline.szUse[2] = '\0';
			szID[4]  = '\0';
			sscline.id = atoi(szID);
			line = sscline;
			return true;
		}
		bool StaSscFile::readSscLine_14(string strLine, SscLine& line)
		{
			if(strLine.length() < 99)
			{
				//cout<<strLine<<endl;
				return false;
			}
			SscLine sscline;
			char szID[5];
			char szSat_ID[2];
			// 起始和结束时间
			int nYear_Start;
			int nDay_Start;
			int nSecond_Start;
			int nYear_End;
			int nDay_End;
			int nSecond_End;
			sscanf(strLine.c_str(),"%9c%*1c%17c%*1c%3c%*1c%4c%*1c%13lf%*1c%13lf%*1c%13lf%*1c%6lf%*1c%6lf%*1c%6lf%*1c%2c%*1c%2d%*1c%3d%*1c%5d%*1c%2d%*1c%3d%*1c%5d",
									sscline.szStationFlag,
									sscline.szName,
									sscline.szUse,
									szID,
								   &sscline.x,
								   &sscline.y,
								   &sscline.z,
								   &sscline.sigma_x,
								   &sscline.sigma_y,
								   &sscline.sigma_z,
								   szSat_ID,
								   &nYear_Start,
								   &nDay_Start,
								   &nSecond_Start,
								   &nYear_End,
								   &nDay_End,
								   &nSecond_End
								   );
			sscline.szStationFlag[8] = '\0';
			sscline.szName[16] = '\0';
			sscline.szUse[2] = '\0';
			szID[4]  = '\0';
			szSat_ID[1] = '\0';
			sscline.id = atoi(szID);
			line = sscline;
			return true;
		}
		void StaSscFile::setEpoch(double epoch)
		{
			m_Epoch = epoch;
		}

		bool StaSscFile::getStaPosVel(UTC t, int id, POS6D& posvel)
		{
			bool bFind = false;
			int count = int(m_data.size());
			if(count <= 0)
				return false;
			int i;
			for(i =  count - 1; i >= 0; i--)
			{// 倒序查找最近的测站信息, 20080121
				if(m_data[i].id == id)
				{
					bFind = true;
					break;
				}
			}
			if(!bFind)
				return false;
			posvel = m_data[i].posvel;
			UTC t_Epoch(int(m_Epoch), 1, 1, 0, 0, 0);
			double year = (t - t_Epoch) / (86400 * 365.25);
			posvel.x += posvel.vx * year;
			posvel.y += posvel.vy * year;
			posvel.z += posvel.vz * year;
			return true;
		}

		// 导出激光坐标数据列表, 用于外部使用, 2008/04/11
		bool StaSscFile::getStaSscList(vector<StaSscRecord> &staSscList)
		{
			size_t count = int(m_data.size());
			if(count <= 0)
				return false;
			staSscList.clear();
			UTC t_Epoch(int(m_Epoch), 1, 1, 0, 0, 0);
			for(size_t s_i =  0; s_i < count; s_i++)
			{
				StaSscRecord sscRecord;
				sscRecord.id   = m_data[s_i].id;
				sscRecord.name = m_data[s_i].name;
				sscRecord.t0   = t_Epoch;
				sscRecord.x    = m_data[s_i].posvel.x;
				sscRecord.y    = m_data[s_i].posvel.y;
				sscRecord.z    = m_data[s_i].posvel.z;
				sscRecord.vx   = m_data[s_i].posvel.vx;
				sscRecord.vy   = m_data[s_i].posvel.vy;
				sscRecord.vz   = m_data[s_i].posvel.vz;
				staSscList.push_back(sscRecord);
			}
			return true;
		}

		bool StaSscFile::open(string  strSscFileName)
		{
			FILE * pSscFile = fopen(strSscFileName.c_str(),"r+t");
			if(pSscFile == NULL) 
				return false;
			char szline[200];     
			m_data.clear();
			// 越过先前的 7 行
			for(int i = 0; i < 7; i++)
				fgets(szline, 200, pSscFile);
			int bFlag = 1;
			fgets(szline, 200, pSscFile);
			while(bFlag)
			{
				int nLineFlag = isValidSscLine(szline, pSscFile);
				if(nLineFlag == 0)
				{
					bFlag = false;
				}
				else if(nLineFlag == 1)
				{// 新测站开始
					// 读取本测站信息
					StaSscEnsemble sscData;
					SscLine line;
					bool bSuccess = true;
					bSuccess = bSuccess & readSscLine(szline, line); // 位置信息
					sscData.id = line.id;
					sscData.name = line.szName;
					sscData.posvel.x  = line.x;
					sscData.posvel.y  = line.y;
					sscData.posvel.z  = line.z;
					sscData.sigma.x   = line.sigma_x;
					sscData.sigma.y   = line.sigma_y;
					sscData.sigma.z   = line.sigma_z;
					fgets(szline, 200, pSscFile);
					bSuccess = bSuccess & readSscLine(szline, line); // 速度信息
					sscData.posvel.vx  = line.x;
					sscData.posvel.vy  = line.y;
					sscData.posvel.vz  = line.z;
					sscData.sigma.vx   = line.sigma_x;
					sscData.sigma.vy   = line.sigma_y;
					sscData.sigma.vz   = line.sigma_z;
					if(bSuccess)
						m_data.push_back(sscData);
				}
				else
				{
				}
				fgets(szline, 200, pSscFile);
			}
			fclose(pSscFile);
			return true;
		}
		// ITRS2014
		bool StaSscFile::open_14(string  strSscFileName)
		{
			FILE * pSscFile = fopen(strSscFileName.c_str(),"r+t");
			if(pSscFile == NULL) 
				return false;
			char szline[200];     
			m_data.clear();
			// 越过先前的 17 行
			for(int i = 0; i < 7; i++)
				fgets(szline, 200, pSscFile);
			int bFlag = 1;
			fgets(szline, 200, pSscFile);
			while(bFlag)
			{
				int nLineFlag = isValidSscLine(szline, pSscFile);
				if(nLineFlag == 0)
				{
					bFlag = false;
				}
				else if(nLineFlag == 1)
				{// 新测站开始
					// 读取本测站信息
					StaSscEnsemble sscData;
					SscLine line;
					bool bSuccess = true;
					bSuccess = bSuccess & readSscLine_14(szline, line); // 位置信息
					sscData.id = line.id;
					sscData.name = line.szName;
					sscData.posvel.x  = line.x;
					sscData.posvel.y  = line.y;
					sscData.posvel.z  = line.z;
					sscData.sigma.x   = line.sigma_x;
					sscData.sigma.y   = line.sigma_y;
					sscData.sigma.z   = line.sigma_z;
					fgets(szline, 200, pSscFile);
					bSuccess = bSuccess & readSscLine(szline, line); // 速度信息
					sscData.posvel.vx  = line.x;
					sscData.posvel.vy  = line.y;
					sscData.posvel.vz  = line.z;
					sscData.sigma.vx   = line.sigma_x;
					sscData.sigma.vy   = line.sigma_y;
					sscData.sigma.vz   = line.sigma_z;
					if(bSuccess)
						m_data.push_back(sscData);
				}
				else
				{
				}
				fgets(szline, 200, pSscFile);
			}
			fclose(pSscFile);
			return true;
		}
	}
}
