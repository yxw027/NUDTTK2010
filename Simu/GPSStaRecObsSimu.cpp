#include "GPSStaRecObsSimu.hpp"
#include "MathAlgorithm.hpp"

using namespace NUDTTK::Math;
namespace NUDTTK
{
	namespace Simu
	{
		GPSStaRecObsSimu::GPSStaRecObsSimu(void)
		{
		}

		GPSStaRecObsSimu::~GPSStaRecObsSimu(void)
		{
		}

		bool GPSStaRecObsSimu::loadSP3File(string  strSP3FileName)
		{
			return m_sp3File.open(strSP3FileName);
		}

		bool GPSStaRecObsSimu::loadCLKFile(string  strCLKFileName)
		{
			return m_clkFile.open(strCLKFileName);
		}

		void GPSStaRecObsSimu::setReceiverPos(double x, double y, double z)
		{
			m_posReceiver.x = x;
			m_posReceiver.y = y;
			m_posReceiver.z = z;
		}

		// 子程序名称： judgeGPSSignalCover   
		// 功能：根据GPS卫星位置、接收机位置和天线的仰角确定信号是否可见(未考虑卫星姿态的影响)
		// 变量类型：posGPSSat           : GPS卫星位置
		//           posRec              : 接收机位置
		//           cut_elevation       : 天线的截至高度角
		// 输入：posGPSSat, posRec, cut_elevation
		// 输出：bSee
		// 其它：
		// 语言：C++
		// 版本号：2008/8/23
		// 生成者：谷德峰
		// 修改者：
		bool GPSStaRecObsSimu::judgeGPSSignalCover(POS3D posGPSSat, POS3D posRec, double cut_elevation)
		{
			bool bSee = false;
			// 接收机到GPS位置矢量
			POS3D R = posRec - posGPSSat;
			double distance1 = sqrt(R.x * R.x + R.y * R.y + R.z * R.z);
			// GPS 到地心距离
			double distance2 = sqrt(posGPSSat.x * posGPSSat.x + posGPSSat.y * posGPSSat.y + posGPSSat.z * posGPSSat.z);
			// GPS 卫星到地球边缘的切线长度
			double DistanceEarth = distance2 * cos(13.87 * PI / 180.0);
			double value         = -vectorDot(R, posGPSSat);
			value  = value / (distance1 * distance2);
			// 由 GPS 到接收机的视线与中心线夹角
			double Sita = acos(value) * 180 / PI;
			// 区域1
			if( Sita <= 21.3 && Sita >= 13.87)
				bSee = true;
			else if(Sita < 13.87)
			{
				// 地球正面--区域2
				if(distance1 < DistanceEarth)
					bSee = true;
				// 地球背面--区域3
				else
					bSee = false;
			}
			// 区域4
			else
				bSee = false;
			// 当信号覆盖到后，考虑接收机截至高度角的影响
			if(bSee)
			{
				// 接收机位置矢量
				POS3D Vec_DLS = posRec;
				Vec_DLS = vectorNormal(Vec_DLS);
				// 由接收机到GPS卫星的视线矢量
				POS3D Vec_Look = R * (-1.0);
				Vec_Look = vectorNormal(Vec_Look);
				double value1 = vectorDot(Vec_DLS, Vec_Look);
				double Alpha = acos(value1) * 180 / PI;
				// 截至高度角限制
				if(Alpha >= (90 - cut_elevation))
					bSee = false;
			}
			return bSee;
		}

		// 子程序名称： simuRecClock   
		// 功能：采用基于Allan方差参数的两状态变量模型来表示接收机钟差漂移
		// 变量类型：p_t           : 接收机钟偏差
		//           q_t           : 钟漂移率
		//           delta         : 钟偏差和漂移率的更新时间间隔
		//           AllanVar_h_0  : 接收机钟差的白频噪声
		//           AllanVar_h_2  : 随机游走频率噪声的Allan方差参数
		// 输入：p_t,q_t,delta,AllanVar_h_0,AllanVar_h_2
		// 输出：bSee
		// 其它：
		// 语言：C++
		// 版本号：2010/02/01
		// 生成者：谷德峰
		// 修改者：
		bool GPSStaRecObsSimu::simuRecClock(double& p_t, double& q_t, double delta, double AllanVar_h_0, double AllanVar_h_2)
		{
			double c1 = 8.0 * PI * PI * AllanVar_h_2;
			double c2 = 2.0 * AllanVar_h_0;
			double cov_wq    = c1 * delta;
			double cov_wp    = c2 * delta + 1.0 / 3.0 * c1 * pow(delta, 3);
			double cov_wp_wq = 1.0 / 2.0 * c1 * pow(delta, 2);
			// 计算相关系数
			double r = cov_wp_wq / sqrt(cov_wp * cov_wq);
			double a = RandNormal();
			double b = RandNormal();
			double wq =  a                            * sqrt(cov_wq);
            double wp = (a * r + b * sqrt(1 - r * r)) * sqrt(cov_wp);
            // 更新 p_t 和 q_t
			p_t = p_t + q_t * delta + wp;
			q_t = q_t + wq;
			return true;
		}

		// 子程序名称： simuGPSObsFile   
		// 功能：仿真生成 GPS 原始观测数据文件
		// 变量类型：obsFile           : 观测数据文件
		//           t0                : 时间起点
		//           h                 : 时间步长
		//           arcLenth          : 弧段长度
		// 输入：t0, h, arcLenth
		// 输出：obsFile
		// 其它：
		// 语言：C++
		// 版本号：2012/10/09
		// 生成者：谷德峰
		// 修改者：
		bool GPSStaRecObsSimu::simuGPSObsFile(Rinex2_1_ObsFile &obsFile, string MakerName, GPST t0, double h, double arcLenth)
		{
			//FILE* pfile1 = fopen("C:\\sp3_J2000.txt","w+");
			// 将GPS卫星星历转换到 J2000 惯性系
			size_t count_sp3 = m_sp3File.m_data.size(); 
			if(count_sp3 <= 0)
			{
				printf("GPS星历文件为空!\n");
				return false;
			}
			for(size_t s_i = 0; s_i < count_sp3; s_i++)
			{
				for(SP3SatMap::iterator it = m_sp3File.m_data[s_i].sp3.begin(); it != m_sp3File.m_data[s_i].sp3.end(); ++it)
				{
					double x_ecf[3];
					double x_j2000[3];
					x_ecf[0] = it->second.pos.x * 1000;  
					x_ecf[1] = it->second.pos.y * 1000; 
					x_ecf[2] = it->second.pos.z * 1000;
					m_TimeCoordConvert.ECEF_J2000(m_sp3File.m_data[s_i].t, x_j2000, x_ecf, false);
					it->second.pos.x = x_j2000[0] / 1000;  
					it->second.pos.y = x_j2000[1] / 1000; 
					it->second.pos.z = x_j2000[2] / 1000;
				}
			}
			srand((unsigned)time(NULL));
			obsFile.m_data.clear();
			double initPhase_L1[MAX_PRN_GPS];    // 初始相位值
			double initPhase_L2[MAX_PRN_GPS];    // 初始相位值
			BYTE bySatList[MAX_PRN_GPS];        // 卫星列表
			for(int i = 0; i < MAX_PRN_GPS; i++)
			{
				bySatList[i]    = 0;
				/*initPhase_L1[i] = floor(RandNormal(0, 1000.0));
				initPhase_L2[i] = floor(RandNormal(0, 1000.0));*/
				initPhase_L1[i] = 0.0;
				initPhase_L2[i] = 0.0;
			}
			double dt  = 0.0;
			double p_t = 0.0;
			double q_t = 0.0;
			while(dt < arcLenth)
			{
				GPST t = t0 + dt;
				// 1. 接收机钟差改正
				double recClock = 0;
				if(m_simuParaDefine.bOn_RecClock)
				{
					if(dt == 0)
						recClock = 0;
					else
					{
						simuRecClock(p_t, q_t, h, m_simuParaDefine.recClockAllanVar_h_0, m_simuParaDefine.recClockAllanVar_h_2);
						recClock = p_t;
					}
				}
				GPST t_Receive = t - recClock / SPEED_LIGHT;// 真实接收时刻
				// 将概略轨道转换到 J2000 系
				double x_ecf[6];
				double x_j2000[6];
				x_ecf[0] = m_posReceiver.x;  
				x_ecf[1] = m_posReceiver.y;  
				x_ecf[2] = m_posReceiver.z;
				x_ecf[3] = 0.0;  
				x_ecf[4] = 0.0;  
				x_ecf[5] = 0.0;
				m_TimeCoordConvert.ECEF_J2000(t_Receive , x_j2000, x_ecf);

				Rinex2_1_ObsEpoch obsEpoch;
				obsEpoch.t = t; 
				obsEpoch.byEpochFlag = 0;
				obsEpoch.bySatCount  = 0;
				obsEpoch.obs.clear();
				double pError_P1[MAX_PRN_GPS];
				double pError_P2[MAX_PRN_GPS];
				double pError_L1[MAX_PRN_GPS];
				double pError_L2[MAX_PRN_GPS];
				memset(pError_P1, 0, sizeof(double) * (MAX_PRN_GPS));
				memset(pError_P2, 0, sizeof(double) * (MAX_PRN_GPS));
				memset(pError_L1, 0, sizeof(double) * (MAX_PRN_GPS));
				memset(pError_L2, 0, sizeof(double) * (MAX_PRN_GPS));
				if(m_simuParaDefine.bOn_RecObsNoise)
				{
					for(int j = 0; j < MAX_PRN_GPS; j++)
					{
						pError_P1[j] = RandNormal(0, m_simuParaDefine.noiseSigma_P1);
						pError_P2[j] = RandNormal(0, m_simuParaDefine.noiseSigma_P2);
						pError_L1[j] = RandNormal(0, m_simuParaDefine.noiseSigma_L1);
						pError_L2[j] = RandNormal(0, m_simuParaDefine.noiseSigma_L2);
					}
				}
				POSCLK posclkRec; 
				posclkRec.x = x_j2000[0];
				posclkRec.y = x_j2000[1];
				posclkRec.z = x_j2000[2];
				posclkRec.clk = 0.0;
				for(int j = 0; j < MAX_PRN_GPS; j++)
				{
					int nPRN = j;
					double delay;
					SP3Datum sp3Datum;
					if(!m_sp3File.getEphemeris_PathDelay(t, posclkRec, nPRN, delay, sp3Datum))
						continue;
					GPST t_Transmit = t_Receive - delay;
					if(judgeGPSSignalCover(sp3Datum.pos, posclkRec.getPos(), m_simuParaDefine.min_elevation))
					{// 有效 GPS 卫星, 生成观测数据
						bySatList[nPRN] = 1;
						Rinex2_1_ObsTypeList obsTypeList;
						obsTypeList.clear();
						double distance = pow(posclkRec.x - sp3Datum.pos.x, 2)
							            + pow(posclkRec.y - sp3Datum.pos.y, 2)
							            + pow(posclkRec.z - sp3Datum.pos.z, 2);
						distance = sqrt(distance);
						// 3 - GPS卫星的相对论效应
						double 	correct_GPSRelativity = 0;
						if(m_simuParaDefine.bOn_Rel_GPSSAT)
						{
							correct_GPSRelativity  = ( sp3Datum.pos.x * sp3Datum.vel.x 
													 + sp3Datum.pos.y * sp3Datum.vel.y
				                           			 + sp3Datum.pos.z * sp3Datum.vel.z) * (-2) / SPEED_LIGHT;
						}
						//// 输出PRN1的星历
						//if(j == 1)
						//{
						//	fprintf(pfile1, "%10.2f %15.4f %15.4f %15.4f %15.4f %15.4f\n", t - t0, 
						//		                                                           distance,
						//																   delay,
						//																   sp3Datum.pos.x,
						//																   sp3Datum.pos.y,
						//																   sp3Datum.pos.z);
						//}
						// 4 - GPS卫星钟差
						double 	correct_GPSClock = 0;
						if(m_simuParaDefine.bOn_Clk_GPSSAT)
						{
							CLKDatum clkDatum;
							if(!m_clkFile.getSatClock(t_Transmit, nPRN, clkDatum))
								continue;
							correct_GPSClock = clkDatum.clkBias * SPEED_LIGHT;
						}
						distance = distance + recClock - correct_GPSRelativity - correct_GPSClock;

						Rinex2_1_ObsDatum P1;
						P1.data  = distance + pError_P1[j];
						Rinex2_1_ObsDatum P2;
						P2.data  = distance + pError_P2[j];
						Rinex2_1_ObsDatum L1;
						L1.data  = distance + pError_L1[j] + initPhase_L1[nPRN] * GPS_WAVELENGTH_L1;
						L1.data  = L1.data / GPS_WAVELENGTH_L1;
						Rinex2_1_ObsDatum L2;
						L2.data  = distance + pError_L2[j] + initPhase_L2[nPRN] * GPS_WAVELENGTH_L2;
						L2.data  = L2.data / GPS_WAVELENGTH_L2;
						obsTypeList.push_back(P1);
						obsTypeList.push_back(P2);
						obsTypeList.push_back(L1);
						obsTypeList.push_back(L2);
						obsEpoch.obs.insert(Rinex2_1_SatMap::value_type(nPRN, obsTypeList));
					}
				}
				obsEpoch.bySatCount = int(obsEpoch.obs.size());
				if(obsEpoch.bySatCount > 0)
				{ 
					obsFile.m_data.push_back(obsEpoch);
				}
				dt = dt + h;
			}
			// 整理文件头
			size_t nCount   = obsFile.m_data.size();
			if(nCount == 0)
				return false;
			obsFile.m_header.bySatCount = 0;
			for(int i = 0; i < MAX_PRN_GPS; i++)
			{
				if(bySatList[i] == 1)
					obsFile.m_header.bySatCount++;
			}
			GPST tmStart = obsFile.m_data[0].t;        
			GPST tmEnd   = obsFile.m_data[nCount-1].t; 
			obsFile.m_header.tmStart = tmStart;
			obsFile.m_header.tmEnd   = tmEnd;
			obsFile.m_header.ApproxPos = m_posReceiver;
			sprintf(obsFile.m_header.szTimeType, "%-3s", "GPS");
			obsFile.m_header.byObsTypes = 4;

			obsFile.m_header.pbyObsTypeList[0] = TYPE_OBS_P1;
			obsFile.m_header.pbyObsTypeList[1] = TYPE_OBS_P2;
			obsFile.m_header.pbyObsTypeList[2] = TYPE_OBS_L1;
			obsFile.m_header.pbyObsTypeList[3] = TYPE_OBS_L2;
			sprintf(obsFile.m_header.szRinexVersion,   "     2.1            ");
			sprintf(obsFile.m_header.szFileType,       "OBSERVATION DATA    ");
			sprintf(obsFile.m_header.szSatlliteSystem, "GPS                 ");
			DayTime T_Now;
			T_Now.Now();
			sprintf(obsFile.m_header.szProgramName, "%-20s","NUDTTK");
			sprintf(obsFile.m_header.szProgramAgencyName, "%-20s","NUDT");
			sprintf(obsFile.m_header.szFileDate, "%04d-%02d-%02d %02d:%02d:%02d ",T_Now.year,T_Now.month,T_Now.day,T_Now.hour,T_Now.minute,int(T_Now.second));
			sprintf(obsFile.m_header.szMarkName,"%-60s",MakerName.c_str());
			sprintf(obsFile.m_header.szObserverName,      "%-20s","OBSERVER");
			sprintf(obsFile.m_header.szObserverAgencyName,"%-40s","NUDT");
			sprintf(obsFile.m_header.szRecNumber, "%-20s","RECNUM");
			sprintf(obsFile.m_header.szRecType,   "%-20s","POD_Antenna");
			sprintf(obsFile.m_header.szRecVersion,"%-20s","RECVERS");
			sprintf(obsFile.m_header.szAntNumber,"%-20s","ANTNUM");
			sprintf(obsFile.m_header.szAntType,  "%-20s","ANTTYPE");
			obsFile.m_header.bL1WaveLengthFact = 1;
			obsFile.m_header.bL2WaveLengthFact = 1;
			obsFile.m_header.bL5WaveLengthFact = 0;
			obsFile.m_header.Interval = h;
			char szComment[100];
			sprintf(szComment,"%-60s%20s\n", "created by gps observation simulation program.", Rinex2_1_MaskString::szComment);
			obsFile.m_header.pstrCommentList.push_back(szComment);
			//fclose(pfile1);
			return true;
		}
		// 子程序名称： simuGPSObsFile_new   
		// 功能：仿真生成 GPS 原始观测数据文件
		// 变量类型：obsFile           : 观测数据文件
		//           t0                : 时间起点
		//           h                 : 时间步长
		//           arcLenth          : 弧段长度
		// 输入：t0, h, arcLenth
		// 输出：obsFile
		// 其它：
		// 语言：C++
		// 版本号：2013/11/09
		// 生成者：谷德峰
		// 修改者：鞠冰
		// 备注：在地固系ECF下插值计算星历
		bool GPSStaRecObsSimu::simuGPSObsFile_new(Rinex2_1_ObsFile &obsFile, GPST t0, double h, double arcLenth)
		{
			FILE* pfile1 = fopen("C:\\sp3_ECF.txt","w+");
			srand((unsigned)time(NULL));
			obsFile.m_data.clear();
			double initPhase_L1[MAX_PRN_GPS];    // 初始相位值
			double initPhase_L2[MAX_PRN_GPS];    // 初始相位值
			BYTE bySatList[MAX_PRN_GPS];        // 卫星列表
			for(int i = 0; i < MAX_PRN_GPS; i++)
			{
				bySatList[i]    = 0;
				/*initPhase_L1[i] = floor(RandNormal(0, 1000.0));
				initPhase_L2[i] = floor(RandNormal(0, 1000.0));*/
				initPhase_L1[i] = 0.0;
				initPhase_L2[i] = 0.0;
			}
			double dt  = 0.0;
			double p_t = 0.0;
			double q_t = 0.0;
			while(dt < arcLenth)
			{
				GPST t = t0 + dt;
				// 1. 接收机钟差改正
				double recClock = 0;
				if(m_simuParaDefine.bOn_RecClock)
				{
					if(dt == 0)
						recClock = 0;
					else
					{
						simuRecClock(p_t, q_t, h, m_simuParaDefine.recClockAllanVar_h_0, m_simuParaDefine.recClockAllanVar_h_2);
						recClock = p_t;
					}
				}
				GPST t_Receive = t - recClock / SPEED_LIGHT;// 真实接收时刻

				Rinex2_1_ObsEpoch obsEpoch;
				obsEpoch.t = t; 
				obsEpoch.byEpochFlag = 0;
				obsEpoch.bySatCount  = 0;
				obsEpoch.obs.clear();
				double pError_P1[MAX_PRN_GPS];
				double pError_P2[MAX_PRN_GPS];
				double pError_L1[MAX_PRN_GPS];
				double pError_L2[MAX_PRN_GPS];
				memset(pError_P1, 0, sizeof(double) * (MAX_PRN_GPS));
				memset(pError_P2, 0, sizeof(double) * (MAX_PRN_GPS));
				memset(pError_L1, 0, sizeof(double) * (MAX_PRN_GPS));
				memset(pError_L2, 0, sizeof(double) * (MAX_PRN_GPS));
				if(m_simuParaDefine.bOn_RecObsNoise)
				{
					for(int j = 0; j < MAX_PRN_GPS; j++)
					{
						pError_P1[j] = RandNormal(0, m_simuParaDefine.noiseSigma_P1);
						pError_P2[j] = RandNormal(0, m_simuParaDefine.noiseSigma_P2);
						pError_L1[j] = RandNormal(0, m_simuParaDefine.noiseSigma_L1);
						pError_L2[j] = RandNormal(0, m_simuParaDefine.noiseSigma_L2);
					}
				}
				POSCLK posclkRec; 
				posclkRec.x = m_posReceiver.x;
				posclkRec.y = m_posReceiver.y;
				posclkRec.z = m_posReceiver.z;
				posclkRec.clk = recClock;
				for(int j = 0; j < MAX_PRN_GPS; j++)
				{
					int nPRN = j;
					double delay;
					SP3Datum sp3Datum;
					if(!m_sp3File.getEphemeris_PathDelay(t, posclkRec, nPRN, delay, sp3Datum))
						continue;
					GPST t_Transmit = t_Receive - delay;
					if(judgeGPSSignalCover(sp3Datum.pos, posclkRec.getPos(), m_simuParaDefine.min_elevation))
					{// 有效 GPS 卫星, 生成观测数据
						bySatList[nPRN] = 1;
						Rinex2_1_ObsTypeList obsTypeList;
						obsTypeList.clear();
						double distance = pow(posclkRec.x - sp3Datum.pos.x, 2)
							            + pow(posclkRec.y - sp3Datum.pos.y, 2)
							            + pow(posclkRec.z - sp3Datum.pos.z, 2);
						distance = sqrt(distance);
						// 3 - GPS卫星的相对论效应
						double 	correct_GPSRelativity = 0;
						if(m_simuParaDefine.bOn_Rel_GPSSAT)
						{
							correct_GPSRelativity  = ( sp3Datum.pos.x * sp3Datum.vel.x 
													 + sp3Datum.pos.y * sp3Datum.vel.y
				                           			 + sp3Datum.pos.z * sp3Datum.vel.z) * (-2) / SPEED_LIGHT;
						}
						// 输出PRN1的星历
						if(j == 1)
						{
							double x_ecf[3];
							double x_j2000[3];
							x_ecf[0] = sp3Datum.pos.x;
							x_ecf[1] = sp3Datum.pos.y;
							x_ecf[2] = sp3Datum.pos.z;
							m_TimeCoordConvert.ECEF_J2000(t_Transmit , x_j2000, x_ecf, false);
							fprintf(pfile1, "%10.2f %15.4f %15.4f %15.4f %15.4f %15.4f\n", t - t0, 
								                                                           distance,
																						   delay,
																						   x_j2000[0],
																						   x_j2000[1],
																						   x_j2000[2]);
						}
						// 4 - GPS卫星钟差
						double 	correct_GPSClock = 0;
						if(m_simuParaDefine.bOn_Clk_GPSSAT)
						{
							CLKDatum clkDatum;
							if(!m_clkFile.getSatClock(t_Transmit, nPRN, clkDatum))
								continue;
							correct_GPSClock = clkDatum.clkBias * SPEED_LIGHT;
						}
						distance = distance + recClock - correct_GPSRelativity - correct_GPSClock;

						Rinex2_1_ObsDatum P1;
						P1.data  = distance + pError_P1[j];
						Rinex2_1_ObsDatum P2;
						P2.data  = distance + pError_P2[j];
						Rinex2_1_ObsDatum L1;
						L1.data  = distance + pError_L1[j] + initPhase_L1[nPRN] * GPS_WAVELENGTH_L1;
						L1.data  = L1.data / GPS_WAVELENGTH_L1;
						Rinex2_1_ObsDatum L2;
						L2.data  = distance + pError_L2[j] + initPhase_L2[nPRN] * GPS_WAVELENGTH_L2;
						L2.data  = L2.data / GPS_WAVELENGTH_L2;
						obsTypeList.push_back(P1);
						obsTypeList.push_back(P2);
						obsTypeList.push_back(L1);
						obsTypeList.push_back(L2);
						obsEpoch.obs.insert(Rinex2_1_SatMap::value_type(nPRN, obsTypeList));
					}
				}
				obsEpoch.bySatCount = int(obsEpoch.obs.size());
				if(obsEpoch.bySatCount > 0)
				{ 
					obsFile.m_data.push_back(obsEpoch);
				}
				dt = dt + h;
			}
			// 整理文件头
			size_t nCount   = obsFile.m_data.size();
			if(nCount == 0)
				return false;
			obsFile.m_header.bySatCount = 0;
			for(int i = 0; i < MAX_PRN_GPS; i++)
			{
				if(bySatList[i] == 1)
					obsFile.m_header.bySatCount++;
			}
			GPST tmStart = obsFile.m_data[0].t;        
			GPST tmEnd   = obsFile.m_data[nCount-1].t; 
			obsFile.m_header.tmStart = tmStart;
			obsFile.m_header.tmEnd   = tmEnd;
			obsFile.m_header.ApproxPos = m_posReceiver;
			sprintf(obsFile.m_header.szTimeType, "%-3s", "GPS");
			obsFile.m_header.byObsTypes = 4;
			obsFile.m_header.pbyObsTypeList.resize(4);
			obsFile.m_header.pbyObsTypeList[0] = TYPE_OBS_P1;
			obsFile.m_header.pbyObsTypeList[1] = TYPE_OBS_P2;
			obsFile.m_header.pbyObsTypeList[2] = TYPE_OBS_L1;
			obsFile.m_header.pbyObsTypeList[3] = TYPE_OBS_L2;
			sprintf(obsFile.m_header.szRinexVersion,   "     2.1            ");
			sprintf(obsFile.m_header.szFileType,       "OBSERVATION DATA    ");
			sprintf(obsFile.m_header.szSatlliteSystem, "GPS                 ");
			DayTime T_Now;
			T_Now.Now();
			sprintf(obsFile.m_header.szProgramName, "%-20s","NUDTTK");
			sprintf(obsFile.m_header.szProgramAgencyName, "%-20s","NUDT");
			sprintf(obsFile.m_header.szFileDate, "%04d-%02d-%02d %02d:%02d:%02d ",T_Now.year,T_Now.month,T_Now.day,T_Now.hour,T_Now.minute,int(T_Now.second));
			sprintf(obsFile.m_header.szMarkName,"%-60s","unknown");
			sprintf(obsFile.m_header.szObserverName,      "%-20s","OBSERVER");
			sprintf(obsFile.m_header.szObserverAgencyName,"%-40s","NUDT");
			sprintf(obsFile.m_header.szRecNumber, "%-20s","RECNUM");
			sprintf(obsFile.m_header.szRecType,   "%-20s","POD_Antenna");
			sprintf(obsFile.m_header.szRecVersion,"%-20s","RECVERS");
			sprintf(obsFile.m_header.szAntNumber,"%-20s","ANTNUM");
			sprintf(obsFile.m_header.szAntType,  "%-20s","ANTTYPE");
			obsFile.m_header.bL1WaveLengthFact = 1;
			obsFile.m_header.bL2WaveLengthFact = 1;
			obsFile.m_header.bL5WaveLengthFact = 0;
			obsFile.m_header.Interval = h;
			char szComment[100];
			sprintf(szComment,"%-60s%20s\n", "created by gps observation simulation program.", Rinex2_1_MaskString::szComment);
			obsFile.m_header.pstrCommentList.push_back(szComment);
			fclose(pfile1);
			return true;
		}
	}
}
