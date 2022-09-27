#include "GPSLeoRecObsSimu.hpp"
#include "MathAlgorithm.hpp"
#include "RuningInfoFile.hpp"
#include "AttitudeTrans.hpp"
#include "structDef.hpp"

using namespace NUDTTK::Math;
namespace NUDTTK
{
	namespace Simu
	{
		GPSLeoRecObsSimu::GPSLeoRecObsSimu(void)
		{
		}

		GPSLeoRecObsSimu::~GPSLeoRecObsSimu(void)
		{
		}

		bool GPSLeoRecObsSimu::loadSP3File(string  strSP3FileName)
		{
			return m_sp3File.open(strSP3FileName);
		}

		bool GPSLeoRecObsSimu::loadCLKFile(string  strCLKFileName)
		{
			return m_clkFile.open(strCLKFileName);
		}

		void GPSLeoRecObsSimu::setAntPCO(double x, double y, double z)
		{
			m_pcoAnt.x = x;
			m_pcoAnt.y = y;
			m_pcoAnt.z = z;
		}

		// 子程序名称： judgeGPSSignalCover   
		// 功能：根据GPS卫星位置、接收机位置和天线的仰角确定信号是否可见
		// 变量类型：posGPSSat           : GPS卫星位置
		//           posRec              : 接收机位置
		//           POS3D Vector_z      : 天线系 Z 轴，默认为天顶方向(R)
		//           cut_elevation       : 天线的截至高度角
		// 输入：posGPSSat, posRec, cut_elevation
		// 输出：bSee
		// 其它：
		// 语言：C++
		// 版本号：2008/8/23
		// 生成者：谷德峰
		// 修改者：2019/10/28，韦春博修改，增加变量：POS3D Vector_z(姿态矩阵获得的星体-Z轴指向)；
		bool GPSLeoRecObsSimu::judgeGPSSignalCover(POS3D posGPSSat, POS3D posRec, POS3D Vector_z, double cut_elevation)
		{
			bool bSee = true;
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
				POS3D Vec_DLS;
				Vec_DLS = Vector_z;        // 天线系 Z 轴
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
		bool GPSLeoRecObsSimu::simuRecClock(double& p_t, double& q_t, double delta, double AllanVar_h_0, double AllanVar_h_2)
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
		// 功能：仿真生成双频GPS原始观测数据文件（单个导航系统）, GPS: L1/L2, BDS: B1/B3
		// 变量类型：obsFile           : 观测数据文件
		//           cSatSystem        : 卫星导航系统缩写字母集合, GPS = "G", BDS = "C"
		// 输入：
		// 输出：obsFile
		// 其它：
		// 语言：C++
		// 版本号：2012/12/19
		// 生成者：谷德峰
		// 修改者：1、在天线系与星体系不一致情况下，增加星体系到天线系下旋转矩阵，邵凯，2020.3.9
		bool GPSLeoRecObsSimu::simuGPSObsFile(Rinex2_1_ObsFile &obsFile, char cSatSystem)
		{
			double FREQUENCE_L1 = GPS_FREQUENCE_L1;
			double FREQUENCE_L2 = GPS_FREQUENCE_L2;
			if(cSatSystem == 'C')
			{
				FREQUENCE_L1 = BD_FREQUENCE_L1;
				FREQUENCE_L2 = BD_FREQUENCE_L5;				
			}				
			double WAVELENGTH_L1 = SPEED_LIGHT / FREQUENCE_L1;
            double WAVELENGTH_L2 = SPEED_LIGHT / FREQUENCE_L2;
			if(m_pvOrbList.size() < 2)
				return false;
			// 获取 GPS/BDS 卫星发射天线相位中心偏移 ( 星固系, 用于GPS/BDS发射天线相位中心修正 )			
			double    ppPCO[MAX_PRN_GPS][3];
			int       ppBlockID[MAX_PRN_GPS];
			map<int, AntCorrectionBlk> mapGPSAntCorrectionBlk;
			for(size_t s_i = 0; s_i < MAX_PRN_GPS;s_i++)
			{
				int id_PRN = int(s_i);
				m_svnavFile.getPCO(m_pvOrbList[0].t,  id_PRN, ppPCO[id_PRN][0], ppPCO[id_PRN][1], ppPCO[id_PRN][2], ppBlockID[id_PRN]);
				char szPRN[4];
				sprintf(szPRN, "%1c%02d",cSatSystem,id_PRN);
				szPRN[3] = '\0';
				AntCorrectionBlk datablk;
				if(m_AtxFile.getAntCorrectBlk(szPRN, m_sp3File.m_data.front().t, datablk))
					mapGPSAntCorrectionBlk.insert(map<int, AntCorrectionBlk>::value_type(id_PRN, datablk));
			}
			// 将GPS/BDS卫星星历转换到 J2000 惯性系
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
			for(size_t s_i = 0; s_i < m_pvOrbList.size(); s_i++)
			{
				double x_ecf[6];
				double x_j2000[6];
				x_ecf[0] = m_pvOrbList[s_i].pos.x;  
				x_ecf[1] = m_pvOrbList[s_i].pos.y;  
				x_ecf[2] = m_pvOrbList[s_i].pos.z;
				x_ecf[3] = m_pvOrbList[s_i].vel.x; 
				x_ecf[4] = m_pvOrbList[s_i].vel.y; 
				x_ecf[5] = m_pvOrbList[s_i].vel.z;
				m_TimeCoordConvert.ECEF_J2000(m_pvOrbList[s_i].t, x_j2000, x_ecf);
				m_pvOrbList[s_i].pos.x = x_j2000[0];
				m_pvOrbList[s_i].pos.y = x_j2000[1]; 
				m_pvOrbList[s_i].pos.z = x_j2000[2];
				m_pvOrbList[s_i].vel.x = x_j2000[3]; 
				m_pvOrbList[s_i].vel.y = x_j2000[4]; 
				m_pvOrbList[s_i].vel.z = x_j2000[5];
			}
			srand((unsigned)time(NULL));
			obsFile.m_data.clear();
			double initPhase_L1[MAX_PRN_GPS];    // 初始相位值
			double initPhase_L2[MAX_PRN_GPS];    // 初始相位值
			double phase_windup[MAX_PRN_GPS];    // phasewindup修正值
			BYTE bySatList[MAX_PRN_GPS];         // 卫星列表
			for(int i = 0; i < MAX_PRN_GPS; i++)
			{
				bySatList[i]    = 0;
				/*initPhase_L1[i] = floor(RandNormal(0, 1000.0));
				initPhase_L2[i] = floor(RandNormal(0, 1000.0));*/
				initPhase_L1[i] = 0.0;
				initPhase_L2[i] = 0.0;
				phase_windup[i] = DBL_MAX;
			}
			double p_t = 0.0;
			double q_t = 0.0;	
			// ——————————Test——————————//
			/*printf("t_start -> %s\n", m_pvOrbList[0].t.toString().c_str());
			printf("t_end   -> %s\n", m_pvOrbList[m_pvOrbList.size() - 1].t.toString().c_str());*/
			// ——————————Test——————————//
			for(size_t s_i = 0; s_i < m_pvOrbList.size(); s_i++)
			{ 
				// 计算星固系
				// 理想情况下，J2000惯性系下的星固系坐标轴方向[与轨道系RTN重合]：
				// X---轨道飞行方向（+T）;Z---卫星天底方向（-R）；Y---轨道法向（-N）
				// 计算星体系，天线系,PCO修正量，+ 邵凯，2020.3.9				
				POS3D exBody, eyBody, ezBody; // 计算实际情况下J2000惯性系下的星固系坐标轴方向[与轨道系RTN不完全重合]
				POS3D exAnt, eyAnt, ezAnt;    // 计算实际情况下J2000惯性系下的天线系坐标轴方向[与星固系不完全重合]
				// 计算理想情况下J2000惯性系下的星固系坐标轴方向[与轨道系RTN重合], x-轨道飞行方向(+T); z-卫星天底方向(-R); y-右手系, 轨道法向(-N) 
				POS3D exRTN = vectorNormal(m_pvOrbList[s_i].vel); // 飞行方向     
				POS3D eyRTN;
				vectorCross(eyRTN, m_pvOrbList[s_i].pos *(-1.0), exRTN); // 天底方向 x 飞行方向
				eyRTN = vectorNormal(eyRTN);                        
				POS3D  ezRTN;
				vectorCross(ezRTN, exRTN, eyRTN); // 右手系
				Matrix matPCO_Body(3, 1);
				matPCO_Body.SetElement(0, 0, m_pcoAnt.x);
				matPCO_Body.SetElement(1, 0, m_pcoAnt.y);
				matPCO_Body.SetElement(2, 0, m_pcoAnt.z);
				Matrix matPCO_RTN = matPCO_Body; // 2015/03/09, 谷德峰
				POS3D pcoAnt;
				pcoAnt.x = matPCO_RTN.GetElement(0, 0);
				pcoAnt.y = matPCO_RTN.GetElement(1, 0);
				pcoAnt.z = matPCO_RTN.GetElement(2, 0);
				POS3D offsetJ2000 = GNSSBasicCorrectFunc::correctLeoAntPCO_J2000(pcoAnt, m_pvOrbList[s_i].pos, m_pvOrbList[s_i].vel);

				Matrix matRTN2J2000(3, 3);
				matRTN2J2000.SetElement(0, 0, exRTN.x);
				matRTN2J2000.SetElement(1, 0, exRTN.y);
				matRTN2J2000.SetElement(2, 0, exRTN.z);
				matRTN2J2000.SetElement(0, 1, eyRTN.x);
				matRTN2J2000.SetElement(1, 1, eyRTN.y);
				matRTN2J2000.SetElement(2, 1, eyRTN.z);
				matRTN2J2000.SetElement(0, 2, ezRTN.x);
				matRTN2J2000.SetElement(1, 2, ezRTN.y);
				matRTN2J2000.SetElement(2, 2, ezRTN.z);
				Matrix matATT = matRTN2J2000;
				exBody.x = matATT.GetElement(0, 0);
				exBody.y = matATT.GetElement(1, 0);
				exBody.z = matATT.GetElement(2, 0);
				exBody = vectorNormal(exBody);
				eyBody.x = matATT.GetElement(0, 1);
				eyBody.y = matATT.GetElement(1, 1);
				eyBody.z = matATT.GetElement(2, 1);
				eyBody = vectorNormal(eyBody);
				ezBody.x = matATT.GetElement(0, 2);
				ezBody.y = matATT.GetElement(1, 2);
				ezBody.z = matATT.GetElement(2, 2);
				ezBody = vectorNormal(ezBody);
				Matrix matAnt = matATT * m_matAxisAnt2Body; // 这里RTN与Body一致
				Matrix matDelPCO_J2000(3, 1);               // J2000惯性系下的LEO卫星天线偏移偏差矢量
				Matrix matDelPCO_Ant(3, 1);
				matDelPCO_Ant.SetElement(0, 0, m_delPcoAnt.x);
				matDelPCO_Ant.SetElement(1, 0, m_delPcoAnt.y);
				matDelPCO_Ant.SetElement(2, 0, m_delPcoAnt.z);
				matDelPCO_J2000 = matAnt * matDelPCO_Ant;
				exAnt.x = matAnt.GetElement(0, 0);
				exAnt.y = matAnt.GetElement(1, 0);
				exAnt.z = matAnt.GetElement(2, 0);
				exAnt = vectorNormal(exAnt);
				eyAnt.x = matAnt.GetElement(0, 1);
				eyAnt.y = matAnt.GetElement(1, 1);
				eyAnt.z = matAnt.GetElement(2, 1);
				eyAnt = vectorNormal(eyAnt);
				ezAnt.x = matAnt.GetElement(0, 2);
				ezAnt.y = matAnt.GetElement(1, 2);
				ezAnt.z = matAnt.GetElement(2, 2);
				ezAnt = vectorNormal(ezAnt);

				// 1. 接收机相对论效应
				double correct_RecRelativity = 0;
				if(m_simuParaDefine.bOn_Rec_Relativity)
				{
                    correct_RecRelativity  = ( m_pvOrbList[s_i].pos.x * m_pvOrbList[s_i].vel.x 
					                         + m_pvOrbList[s_i].pos.y * m_pvOrbList[s_i].vel.y
				                             + m_pvOrbList[s_i].pos.z * m_pvOrbList[s_i].vel.z) * (-2) / SPEED_LIGHT;
				}
				// 2 - 接收机钟差改正
				double recClock = 0;
				if(m_simuParaDefine.bOn_Rec_Clock)
				{
					if(s_i == 0)
						recClock = 0;
					else
					{
						double h = m_pvOrbList[s_i].t - m_pvOrbList[s_i - 1].t;
						simuRecClock(p_t, q_t, h, m_simuParaDefine.recClockAllanVar_h_0, m_simuParaDefine.recClockAllanVar_h_2);
						recClock = p_t;
					}
				}
				/*
				    2010/10/29, 进行了修改, 原代码为

						POSCLK  posclkRec; 
						posclkRec.x = m_pvOrbList[s_i].pos.x;
						posclkRec.y = m_pvOrbList[s_i].pos.y;
						posclkRec.z = m_pvOrbList[s_i].pos.z;
						posclkRec.clk = 0.0;
						GPST t_Receive = m_pvOrbList[s_i].t;

					将观测时刻由真实时刻调整为钟差时刻
					调整后
                    观测时刻为：m_pvOrbList[s_i].t
					真实时刻为：t_Receive = m_pvOrbList[s_i].t - recClock / SPEED_LIGHT
					真实时刻的接收机位置为： pos = m_pvOrbList[s_i].pos -  m_pvOrbList[s_i].vel * recClock / SPEED_LIGHT
				*/
				POSCLK  posclkRec; 
				posclkRec.x = m_pvOrbList[s_i].pos.x - m_pvOrbList[s_i].vel.x * recClock / SPEED_LIGHT;
				posclkRec.y = m_pvOrbList[s_i].pos.y - m_pvOrbList[s_i].vel.y * recClock / SPEED_LIGHT;
				posclkRec.z = m_pvOrbList[s_i].pos.z - m_pvOrbList[s_i].vel.z * recClock / SPEED_LIGHT;
				posclkRec.clk = recClock; // 20141126, 谷德峰
				GPST t_Receive = m_pvOrbList[s_i].t - recClock / SPEED_LIGHT;// 真实接收时刻
				Rinex2_1_ObsEpoch obsEpoch;
				obsEpoch.t = m_pvOrbList[s_i].t; 
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
				if(m_simuParaDefine.bOn_Rec_ObsNoise)
				{
					for(int j = 0; j < MAX_PRN_GPS; j++)
					{
						pError_P1[j] = RandNormal(0, m_simuParaDefine.noiseSigma_P1);
						pError_P2[j] = RandNormal(0, m_simuParaDefine.noiseSigma_P2);
						pError_L1[j] = RandNormal(0, m_simuParaDefine.noiseSigma_L1);
						pError_L2[j] = RandNormal(0, m_simuParaDefine.noiseSigma_L2);
					}
				}
				for(int j = 0; j < MAX_PRN_GPS; j++)
				{
					int nPRN = j;
					double delay;
					SP3Datum sp3Datum;
					char szPRN[4];
					sprintf(szPRN, "%1c%02d",cSatSystem,nPRN);
					szPRN[3] = '\0';
					if(!m_sp3File.getEphemeris_PathDelay(m_pvOrbList[s_i].t, posclkRec, szPRN, delay, sp3Datum))
						continue;					
					GPST t_Transmit = t_Receive - delay;
					// 判断GPS卫星是否可见
					if(judgeGPSSignalCover(sp3Datum.pos, posclkRec.getPos(), ezAnt, m_simuParaDefine.min_elevation))
					{// 有效 GPS 卫星, 生成观测数据
						bySatList[nPRN] = 1;
						Rinex2_1_ObsTypeList obsTypeList;
						obsTypeList.clear();
						double distance = pow(posclkRec.x - sp3Datum.pos.x, 2)
							            + pow(posclkRec.y - sp3Datum.pos.y, 2)
							            + pow(posclkRec.z - sp3Datum.pos.z, 2);
						distance = sqrt(distance);
						POS3D E; // 记录概略展开系数(未加权)
						E.x = (posclkRec.x - sp3Datum.pos.x) / distance;
						E.y = (posclkRec.y - sp3Datum.pos.y) / distance;
						E.z = (posclkRec.z - sp3Datum.pos.z) / distance;
						//获取惯性系下太阳的位置用于 phaseWindup 和导航卫星 PCO 修正
						POS3D sunPos;
						if(m_simuParaDefine.bOn_GNSSSAT_AntPCOPCV || m_simuParaDefine.bOn_PhaseWindUp)
						{
							double jd = TimeCoordConvert::DayTime2JD(TimeCoordConvert::GPST2TDB(t_Transmit)); // 获得儒略日
							double P[3];
							m_JPLEphFile.getSunPos_Delay_EarthCenter(jd, P);// 获得 J2000 系下的太阳相对地心的位置(千米)
							sunPos.x = P[0] * 1000; // 换算成米
							sunPos.y = P[1] * 1000; // 换算成米
							sunPos.z = P[2] * 1000; // 换算成米
							// sunPos 为 J2000 惯性系坐标, 需统一转换到地固系
							double x_ecf[3];
							double x_j2000[3];
							x_j2000[0] = sunPos.x;  
							x_j2000[1] = sunPos.y; 
							x_j2000[2] = sunPos.z;
							m_TimeCoordConvert.J2000_ECEF(t_Transmit, x_j2000, x_ecf, false);
							sunPos.x = x_ecf[0];  
							sunPos.y = x_ecf[1]; 
							sunPos.z = x_ecf[2];
						}
						// 3 - GPS卫星的相对论效应
						double 	correct_GPSRelativity = 0;
						if(m_simuParaDefine.bOn_GNSSSAT_Relativity)
						{
							correct_GPSRelativity  = ( sp3Datum.pos.x * sp3Datum.vel.x 
													 + sp3Datum.pos.y * sp3Datum.vel.y
				                           			 + sp3Datum.pos.z * sp3Datum.vel.z) * (-2) / SPEED_LIGHT;
						}
						// 4 - GPS卫星钟差
						double 	correct_GPSClock = 0;
						if(m_simuParaDefine.bOn_GNSSSAT_Clk)
						{
							CLKDatum clkDatum;
							if(!m_clkFile.getSatClock(t_Transmit, nPRN, clkDatum, 3, cSatSystem))
								continue;
							correct_GPSClock = clkDatum.clkBias * SPEED_LIGHT;
						}
						// 5 GPS卫星天线 PCO 修正
						double correct_gpspco = 0;
						if(m_simuParaDefine.bOn_GNSSSAT_AntPCOPCV) 
						{
							map<int, AntCorrectionBlk>::iterator it_GPSAntCorrectionBlk = mapGPSAntCorrectionBlk.find(nPRN);
							if(it_GPSAntCorrectionBlk != mapGPSAntCorrectionBlk.end())
							{
								if(cSatSystem == 'G')								
									correct_gpspco = m_AtxFile.correctSatAntPCOPCV(it_GPSAntCorrectionBlk->second, 1, posclkRec.getPos(), sp3Datum.pos, sunPos, true);	
								else if(cSatSystem == 'C')
								{//按照block IIA卫星修正																
									if(nPRN > 5)  // IGSO卫星和MEO卫星PCO修正									
										correct_gpspco = m_AtxFile.correctSatAntPCOPCV(it_GPSAntCorrectionBlk->second, 0, posclkRec.getPos(), sp3Datum.pos, sunPos, true);														
									else  // GEO卫星PCO修正，2014/08/24，刘俊宏
									{
										POS3D vecLOS = vectorNormal(posclkRec.getPos() - sp3Datum.pos);	// 视线单位矢量，卫星指向测站																					
										POS3D axisvec_R, axisvec_T, axisvec_N;	
										axisvec_R = vectorNormal(sp3Datum.pos) * (-1.0);//轨道系坐标轴 R 对应星体系的"-Z"方向	
										POS3D eV  = vectorNormal(sp3Datum.vel);
										vectorCross(axisvec_N,axisvec_R,eV);
										axisvec_N = vectorNormal(axisvec_N);            //轨道系坐标轴 N 对应星体系的"-Y"方向	
										vectorCross(axisvec_T,axisvec_N,axisvec_R);
										axisvec_T = vectorNormal(axisvec_T);			//轨道系坐标轴 T 对应星体系的"+X"方向(即ex)										
										correct_gpspco = m_AtxFile.correctSatAntPCOPCV_YawFixed(it_GPSAntCorrectionBlk->second, 0, vecLOS, axisvec_T, axisvec_N * (-1.0), axisvec_R * (-1.0), true);												
									}							
								}
							}
						}
						// 6 - leo卫星天线相位中心偏移改正，先验值，星固系
						double correct_leopco = 0.0;
						if(m_simuParaDefine.bOn_Rec_AntPCO)
						{
							correct_leopco = -(offsetJ2000.x * E.x + offsetJ2000.y * E.y + offsetJ2000.z * E.z);
						}
						// 7 - leo卫星天线相位中心偏移偏差改正，天线系 + 邵凯，2021.05.03
						double correct_leopco_offset = 0.0;
						if(m_simuParaDefine.bOn_Rec_AntPCO_Offset)
						{
							correct_leopco_offset = -(matDelPCO_J2000.GetElement(0, 0) * E.x
										            + matDelPCO_J2000.GetElement(1, 0) * E.y
											        + matDelPCO_J2000.GetElement(2, 0) * E.z);
						}
						
						// 8 - LEO卫星天线 PCV 修正 (在轨天线校准结果)
						double correct_leopcv = 0.0;
						if(m_simuParaDefine.bOn_Rec_AntPCV)
						{// 计算天空视图
							POS3D vecLos = sp3Datum.pos - posclkRec.getPos(); // 视线矢量: 接收机位置指向GPS卫星
							POS3D vecLosXYZ;
							//vecLosXYZ.x = vectorDot(vecLos, S_X);
							//vecLosXYZ.y = vectorDot(vecLos, S_Y);
							//vecLosXYZ.z = vectorDot(vecLos, S_Z);
							vecLosXYZ.x = vectorDot(vecLos, exAnt);
							vecLosXYZ.y = vectorDot(vecLos, eyAnt);
							vecLosXYZ.z = vectorDot(vecLos, ezAnt);
							vecLosXYZ = vectorNormal(vecLosXYZ);
							double Elevation = 90 - acos(vecLosXYZ.z) * 180 / PI;
							double Azimuth = atan2(vecLosXYZ.y, vecLosXYZ.x) * 180 / PI;
							if(Azimuth < 0)
							{
								Azimuth += 360.0;// 变换到[0, 360]
							}
							correct_leopcv = m_pcvFile.getPCVValue(Elevation, Azimuth);
						}
						// 9 - 相位缠绕改正
						double correct_phasewindup_L1 = 0;
						double correct_phasewindup_L2 = 0;
						if(m_simuParaDefine.bOn_PhaseWindUp)
						{
							if(cSatSystem == 'G')
								phase_windup[nPRN] = GNSSBasicCorrectFunc::correctLeoAntPhaseWindUp(ppBlockID[nPRN], m_pvOrbList[s_i].getPosVel(), sp3Datum.pos, sunPos, phase_windup[nPRN]);
							else if(cSatSystem == 'C')//按照block IIA卫星修正
								phase_windup[nPRN] = GNSSBasicCorrectFunc::correctLeoAntPhaseWindUp(3, m_pvOrbList[s_i].getPosVel(), sp3Datum.pos, sunPos, phase_windup[nPRN]);
							//correct_phasewindup = phase_windup[nPRN] * SPEED_LIGHT / ((FREQUENCE_L1 + FREQUENCE_L2)); // 20141126, 与频率相关		
							correct_phasewindup_L1 = phase_windup[nPRN] * WAVELENGTH_L1;
							correct_phasewindup_L2 = phase_windup[nPRN] * WAVELENGTH_L2;
						}						
						distance = distance
							     + recClock
								 - correct_RecRelativity // 与解算修正符号相反
								 - correct_GPSRelativity // 与解算修正符号相反
								 - correct_GPSClock      // 与解算修正符号相反
								 - correct_gpspco        // 与解算修正符号相反
								 - correct_leopco        // 与解算修正符号相反
								 - correct_leopco_offset // 与解算修正符号相反 +
								 + correct_leopcv;       // 与解算修正符号相反
								// + correct_phasewindup;  // 与解算修正符号相反
						// 9 - 电离层延迟改正
						double ion_L1 = 0.0;
						double ion_L2 = 0.0;
						if(m_simuParaDefine.bOn_Ionosphere)
						{
							if(!m_ionFile.isEmpty())
							{
								POS3D leoPos_ECEF, gpsPos_ECEF;// 转换到地固系
								double x_ecf[3];
								double x_j2000[3];
								x_j2000[0] = posclkRec.x;  
								x_j2000[1] = posclkRec.y;  
								x_j2000[2] = posclkRec.z;
								m_TimeCoordConvert.J2000_ECEF(t_Receive,  x_j2000, x_ecf, false);
								leoPos_ECEF.x = x_ecf[0];
								leoPos_ECEF.y = x_ecf[1]; 
								leoPos_ECEF.z = x_ecf[2];
								x_j2000[0] = sp3Datum.pos.x;  
								x_j2000[1] = sp3Datum.pos.y;  
								x_j2000[2] = sp3Datum.pos.z;
								m_TimeCoordConvert.J2000_ECEF(t_Transmit, x_j2000, x_ecf, false);
								gpsPos_ECEF.x = x_ecf[0];
								gpsPos_ECEF.y = x_ecf[1]; 
								gpsPos_ECEF.z = x_ecf[2];
								GNSSBasicCorrectFunc::ionexGridCorrect_IP(m_ionFile, t_Receive, leoPos_ECEF, gpsPos_ECEF, ion_L1, 80000, FREQUENCE_L1); // 20141126, 与频率相关
								GNSSBasicCorrectFunc::ionexGridCorrect_IP(m_ionFile, t_Receive, leoPos_ECEF, gpsPos_ECEF, ion_L2, 80000, FREQUENCE_L2); // 20141126, 与频率相关
							}
						}
						// 9 - 添加观测噪声
						Rinex2_1_ObsDatum P1;
						P1.data  = distance + pError_P1[j] + correct_phasewindup_L1 + ion_L1;
						Rinex2_1_ObsDatum P2;
						P2.data  = distance + pError_P2[j] + correct_phasewindup_L2 + ion_L2;
						Rinex2_1_ObsDatum L1;
						L1.data  = distance + pError_L1[j] + correct_phasewindup_L1 - ion_L1 + initPhase_L1[nPRN] * WAVELENGTH_L1; // 20141126, 与频率相关
						L1.data  = L1.data / WAVELENGTH_L1;// 20141126, 与频率相关
						Rinex2_1_ObsDatum L2;
						L2.data  = distance + pError_L2[j] + correct_phasewindup_L2 - ion_L2 + initPhase_L2[nPRN] * WAVELENGTH_L2;// 20141126, 与频率相关
						L2.data  = L2.data / WAVELENGTH_L2;// 20141126, 与频率相关
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
			sprintf(obsFile.m_header.szTimeType, "%-3s", "GPS");
			obsFile.m_header.byObsTypes = 4;
			obsFile.m_header.pbyObsTypeList.resize(4);
			obsFile.m_header.pbyObsTypeList[0] = TYPE_OBS_P1;			
			obsFile.m_header.pbyObsTypeList[2] = TYPE_OBS_L1; // 20141126, 与频率相关
			if(cSatSystem == 'G')
			{
				obsFile.m_header.pbyObsTypeList[1] = TYPE_OBS_P2;
				obsFile.m_header.pbyObsTypeList[3] = TYPE_OBS_L2; // 20141126, 与频率相关
			}
			else if(cSatSystem == 'C')
			{
				obsFile.m_header.pbyObsTypeList[1] = TYPE_OBS_P5;
				obsFile.m_header.pbyObsTypeList[3] = TYPE_OBS_L5; // 20141126, 与频率相关
			}			
			sprintf(obsFile.m_header.szRinexVersion,   "     2.1            ");
			sprintf(obsFile.m_header.szFileType,       "OBSERVATION DATA    ");
			if(cSatSystem == 'G') // 2013/01/02, 增加北斗系统的描述
				sprintf(obsFile.m_header.szSatlliteSystem, "GPS                 ");
			else
				sprintf(obsFile.m_header.szSatlliteSystem, "C                   ");
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
			obsFile.m_header.Interval = m_pvOrbList[1].t - m_pvOrbList[0].t;
			char szComment[100];
			sprintf(szComment,"%-60s%20s\n", "created by GNSS observation simulation program.", Rinex2_1_MaskString::szComment);
			obsFile.m_header.pstrCommentList.push_back(szComment);
			return true;
		}

		// 子程序名称： simuGNSSMixedObsFile   
		// 功能：仿真生成 GNSS 混合原始观测数据文件
		// 变量类型：obsFile           : 观测数据文件
		//           strSystem         : 卫星导航系统缩写字母集合, GPS = "G", BDS = "C", 可以多个系统联合, 如"GC"
		// 输入：
		// 输出：obsFile
		// 其它：目前兼容双系统(GPS+BDS)双频(L1+L2)的数据仿真
		// 语言：C++
		// 版本号：2014/04/06
		// 生成者：谷德峰
		// 修改者：
		bool GPSLeoRecObsSimu::simuGNSSMixedObsFile(Rinex2_1_MixedObsFile &obsFile, string strSystem)
		{
			if(m_pvOrbList.size() < 2)
				return false;
			// 获取 GPS 卫星发射天线相位中心偏移 ( 星固系, 用于GPS发射天线相位中心修正 )
			double ppPCO[MAX_PRN_GPS][3];
			int    ppBlockID[MAX_PRN_GPS];
			if(strSystem.find('G') != -1) 
			{
				for(size_t s_i = 0; s_i < MAX_PRN_GPS;s_i++)
				{
					int id_PRN = int(s_i);					
					m_svnavFile.getPCO(m_pvOrbList[0].t,  id_PRN, ppPCO[id_PRN][0], ppPCO[id_PRN][1], ppPCO[id_PRN][2], ppBlockID[id_PRN]);
				}
			}//	
			// 获取 GNSS 卫星发射天线相位中心偏移 ( 星固系, 用于GPS/BDS发射天线相位中心修正 )			
			map<string, AntCorrectionBlk> mapGPSAntCorrectionBlk;
			if(strSystem.find('G') != -1)
			{
				for(size_t s_i = 0; s_i < MAX_PRN_GPS;s_i++)
				{
					int id_PRN = int(s_i);				
					char szPRN[4];
					sprintf(szPRN, "G%02d",id_PRN);
					szPRN[3] = '\0';
					AntCorrectionBlk datablk;
					if(m_AtxFile.getAntCorrectBlk(szPRN, m_sp3File.m_data.front().t, datablk))
						mapGPSAntCorrectionBlk.insert(map<string, AntCorrectionBlk>::value_type(szPRN, datablk));
				}
			}
			if(strSystem.find('C') != -1)
			{
				for(size_t s_i = 0; s_i < MAX_PRN_GPS;s_i++)
				{
					int id_PRN = int(s_i);				
					char szPRN[4];
					sprintf(szPRN, "C%02d",id_PRN);
					szPRN[3] = '\0';
					AntCorrectionBlk datablk;
					if(m_AtxFile.getAntCorrectBlk(szPRN, m_sp3File.m_data.front().t, datablk))
						mapGPSAntCorrectionBlk.insert(map<string, AntCorrectionBlk>::value_type(szPRN, datablk));
				}
			}
			// 将GNSS卫星星历转换到 J2000 惯性系
			size_t count_sp3 = m_sp3File.m_data.size(); 
			if(count_sp3 <= 0)
			{
				printf("GNSS星历文件为空!\n");
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
			for(size_t s_i = 0; s_i < m_pvOrbList.size(); s_i++)
			{
				double x_ecf[6];
				double x_j2000[6];
				x_ecf[0] = m_pvOrbList[s_i].pos.x;  
				x_ecf[1] = m_pvOrbList[s_i].pos.y;  
				x_ecf[2] = m_pvOrbList[s_i].pos.z;
				x_ecf[3] = m_pvOrbList[s_i].vel.x; 
				x_ecf[4] = m_pvOrbList[s_i].vel.y; 
				x_ecf[5] = m_pvOrbList[s_i].vel.z;
				m_TimeCoordConvert.ECEF_J2000(m_pvOrbList[s_i].t, x_j2000, x_ecf);
				m_pvOrbList[s_i].pos.x = x_j2000[0];
				m_pvOrbList[s_i].pos.y = x_j2000[1]; 
				m_pvOrbList[s_i].pos.z = x_j2000[2];
				m_pvOrbList[s_i].vel.x = x_j2000[3]; 
				m_pvOrbList[s_i].vel.y = x_j2000[4]; 
				m_pvOrbList[s_i].vel.z = x_j2000[5];
			}
			srand((unsigned)time(NULL));
			obsFile.m_data.clear();
			vector<double> initPhase_L1_List;    // 初始相位值
			vector<double> initPhase_L2_List;    // 初始相位值
			vector<double> initPhase_L5_List;    // 初始相位值
			vector<double> phase_windup;         // phasewindup修正值
            initPhase_L1_List.resize(m_sp3File.m_header.pstrSatNameList.size());
			initPhase_L2_List.resize(m_sp3File.m_header.pstrSatNameList.size());
			initPhase_L5_List.resize(m_sp3File.m_header.pstrSatNameList.size());
			phase_windup.resize(m_sp3File.m_header.pstrSatNameList.size());    
            vector<BYTE> bySatList;              // 卫星列表
			bySatList.resize(m_sp3File.m_header.pstrSatNameList.size());
			for(int i = 0; i < int(m_sp3File.m_header.pstrSatNameList.size()); i++)
			{
				bySatList[i] = 0;
				/*initPhase_L1[i] = floor(RandNormal(0, 1000.0));
				initPhase_L2[i] = floor(RandNormal(0, 1000.0));*/
				initPhase_L1_List[i] = 0.0;
				initPhase_L2_List[i] = 0.0;
				initPhase_L5_List[i] = 0.0;
				phase_windup[i]      = 0.0;
			}
			double p_t = 0.0;
			double q_t = 0.0;
			for(size_t s_i = 0; s_i < m_pvOrbList.size(); s_i++)
			{
				// 计算星固系		
				// 理想情况下，J2000惯性系下的星固系坐标轴方向[与轨道系RTN重合]：
				POS3D S_Z; // Z轴指向卫星
				POS3D S_X; // X轴沿速度方向
				POS3D S_Y; // 右手系
				int validATT = 0;			
				S_Z = vectorNormal(m_pvOrbList[s_i].pos);
				S_X = vectorNormal(m_pvOrbList[s_i].vel);	                                             
				vectorCross(S_Y, S_Z, S_X);
				S_Y = vectorNormal(S_Y);		                                          
				vectorCross(S_X, S_Y, S_Z);	
				S_X = vectorNormal(S_X);
				// 1. 接收机相对论效应
				double correct_RecRelativity = 0;
				if(m_simuParaDefine.bOn_Rec_Relativity)
				{
                    correct_RecRelativity  = ( m_pvOrbList[s_i].pos.x * m_pvOrbList[s_i].vel.x 
					                         + m_pvOrbList[s_i].pos.y * m_pvOrbList[s_i].vel.y
				                             + m_pvOrbList[s_i].pos.z * m_pvOrbList[s_i].vel.z) * (-2) / SPEED_LIGHT;
				}
				// 2 - 接收机钟差改正
				double recClock = 0;
				if(m_simuParaDefine.bOn_Rec_Clock)
				{
					if(s_i == 0)
						recClock = 0;
					else
					{
						double h = m_pvOrbList[s_i].t - m_pvOrbList[s_i - 1].t;
						simuRecClock(p_t, q_t, h, m_simuParaDefine.recClockAllanVar_h_0, m_simuParaDefine.recClockAllanVar_h_2);
						recClock = p_t;
					}
				}
				/*
				    2010/10/29, 进行了修改, 原代码为

						POSCLK  posclkRec; 
						posclkRec.x = m_pvOrbList[s_i].pos.x;
						posclkRec.y = m_pvOrbList[s_i].pos.y;
						posclkRec.z = m_pvOrbList[s_i].pos.z;
						posclkRec.clk = 0.0;
						GPST t_Receive = m_pvOrbList[s_i].t;

					将观测时刻由真实时刻调整为钟差时刻
					调整后
                    观测时刻为：m_pvOrbList[s_i].t
					真实时刻为：t_Receive = m_pvOrbList[s_i].t - recClock / SPEED_LIGHT
					真实时刻的接收机位置为： pos = m_pvOrbList[s_i].pos -  m_pvOrbList[s_i].vel * recClock / SPEED_LIGHT
				*/
				POSCLK  posclkRec; 
				posclkRec.x = m_pvOrbList[s_i].pos.x - m_pvOrbList[s_i].vel.x * recClock / SPEED_LIGHT;
				posclkRec.y = m_pvOrbList[s_i].pos.y - m_pvOrbList[s_i].vel.y * recClock / SPEED_LIGHT;
				posclkRec.z = m_pvOrbList[s_i].pos.z - m_pvOrbList[s_i].vel.z * recClock / SPEED_LIGHT;
				posclkRec.clk = 0.0;
				GPST t_Receive = m_pvOrbList[s_i].t - recClock / SPEED_LIGHT;// 真实接收时刻
				Rinex2_1_MixedObsEpoch obsEpoch;
				obsEpoch.t = m_pvOrbList[s_i].t; 
				obsEpoch.byEpochFlag = 0;
				obsEpoch.bySatCount  = 0;
				obsEpoch.obs.clear();
				vector<double> Error_C1_List;
				vector<double> Error_P1_List;
				vector<double> Error_P2_List;
				vector<double> Error_P5_List;
				vector<double> Error_L1_List;
				vector<double> Error_L2_List;
				vector<double> Error_L5_List;
				Error_C1_List.resize(m_sp3File.m_header.pstrSatNameList.size());
				Error_P1_List.resize(m_sp3File.m_header.pstrSatNameList.size());
				Error_P2_List.resize(m_sp3File.m_header.pstrSatNameList.size());
				Error_P5_List.resize(m_sp3File.m_header.pstrSatNameList.size());
				Error_L1_List.resize(m_sp3File.m_header.pstrSatNameList.size());
				Error_L2_List.resize(m_sp3File.m_header.pstrSatNameList.size());
				Error_L5_List.resize(m_sp3File.m_header.pstrSatNameList.size());
				if(m_simuParaDefine.bOn_Rec_ObsNoise)
				{
					for(int j = 0; j < int(m_sp3File.m_header.pstrSatNameList.size()); j++)
					{
						if(m_sp3File.m_header.pstrSatNameList[j][0] == 'C')
						{
							Error_C1_List[j] = RandNormal(0, m_simuParaDefine.noiseSigma_C1_bds);
							Error_P1_List[j] = RandNormal(0, m_simuParaDefine.noiseSigma_P1_bds);
							Error_P2_List[j] = RandNormal(0, m_simuParaDefine.noiseSigma_P2_bds);
							Error_P5_List[j] = RandNormal(0, m_simuParaDefine.noiseSigma_P5_bds);
							Error_L1_List[j] = RandNormal(0, m_simuParaDefine.noiseSigma_L1_bds);
							Error_L2_List[j] = RandNormal(0, m_simuParaDefine.noiseSigma_L2_bds);
							Error_L5_List[j] = RandNormal(0, m_simuParaDefine.noiseSigma_L5_bds);
						}
						else
						{
							Error_C1_List[j] = RandNormal(0, m_simuParaDefine.noiseSigma_C1);
							Error_P1_List[j] = RandNormal(0, m_simuParaDefine.noiseSigma_P1);
							Error_P2_List[j] = RandNormal(0, m_simuParaDefine.noiseSigma_P2);
							Error_L1_List[j] = RandNormal(0, m_simuParaDefine.noiseSigma_L1);
							Error_L2_List[j] = RandNormal(0, m_simuParaDefine.noiseSigma_L2);
						}
					}
				}
				else
				{
					for(int j = 0; j < int(m_sp3File.m_header.pstrSatNameList.size()); j++)
					{
						Error_C1_List[j] = 0.0;
						Error_P1_List[j] = 0.0;
						Error_P2_List[j] = 0.0;
						Error_L1_List[j] = 0.0;
						Error_L2_List[j] = 0.0;
					}
				}
				for(int j = 0; j < int(m_sp3File.m_header.pstrSatNameList.size()); j++)
				{
					if(strSystem.find(m_sp3File.m_header.pstrSatNameList[j][0]) == -1)
						continue;
					double delay;
					SP3Datum sp3Datum;
					if(!m_sp3File.getEphemeris_PathDelay(m_pvOrbList[s_i].t, posclkRec, m_sp3File.m_header.pstrSatNameList[j], delay, sp3Datum))
						continue;
					GPST t_Transmit = t_Receive - delay;
					// 2019/10/28，韦春博修改，根据不同的巡航模式判断GPS卫星是否可见
					// 在judgeGPSSignalCover函数中
					POS3D Vector_z;
					Vector_z.x = 0.0;
					Vector_z.y = 0.0;
					Vector_z.z = 0.0;
					// 判断GPS卫星是否可见
					if(judgeGPSSignalCover(sp3Datum.pos, posclkRec.getPos(),Vector_z, m_simuParaDefine.min_elevation))
					{// 有效 GNSS 卫星, 生成观测数据
						int nPRN;
						sscanf(m_sp3File.m_header.pstrSatNameList[j].c_str(), "%*1c%2d", &nPRN);
						bySatList[j] = 1;
						Rinex2_1_ObsTypeList obsTypeList;
						obsTypeList.clear();
						double distance = pow(posclkRec.x - sp3Datum.pos.x, 2)
							            + pow(posclkRec.y - sp3Datum.pos.y, 2)
							            + pow(posclkRec.z - sp3Datum.pos.z, 2);
						distance = sqrt(distance);
						POS3D E; // 记录概略展开系数(未加权)
						E.x = (posclkRec.x - sp3Datum.pos.x) / distance;
						E.y = (posclkRec.y - sp3Datum.pos.y) / distance;
						E.z = (posclkRec.z - sp3Datum.pos.z) / distance;
						POS3D sunPos;
						if(m_simuParaDefine.bOn_GNSSSAT_AntPCOPCV || m_simuParaDefine.bOn_PhaseWindUp)
						{
							double jd = TimeCoordConvert::DayTime2JD(TimeCoordConvert::GPST2TDB(t_Transmit)); // 获得儒略日
							double P[3];
							m_JPLEphFile.getSunPos_Delay_EarthCenter(jd, P);// 获得 J2000 系下的太阳相对地心的位置(千米)
							sunPos.x = P[0] * 1000; // 换算成米
							sunPos.y = P[1] * 1000; // 换算成米
							sunPos.z = P[2] * 1000; // 换算成米
						}
						// 3 - GNSS卫星的相对论效应
						double 	correct_GPSRelativity = 0;
						if(m_simuParaDefine.bOn_GNSSSAT_Relativity)
						{
							correct_GPSRelativity  = ( sp3Datum.pos.x * sp3Datum.vel.x 
													 + sp3Datum.pos.y * sp3Datum.vel.y
				                           			 + sp3Datum.pos.z * sp3Datum.vel.z) * (-2) / SPEED_LIGHT;
						}
						// 4 - GNSS卫星钟差
						double 	correct_GPSClock = 0;
						if(m_simuParaDefine.bOn_GNSSSAT_Clk)
						{
							CLKDatum clkDatum;
							if(!m_clkFile.getSatClock(t_Transmit, m_sp3File.m_header.pstrSatNameList[j] + " ", clkDatum, 3))
								continue;
							correct_GPSClock = clkDatum.clkBias * SPEED_LIGHT;
						}
						
						// 5 GNSS卫星天线 PCO 修正
						double correct_gpspco = 0;
						if(m_simuParaDefine.bOn_GNSSSAT_AntPCOPCV ) //&& m_sp3File.m_header.pstrSatNameList[j][0] == 'G'
						{
							map<string, AntCorrectionBlk>::iterator it_GPSAntCorrectionBlk = mapGPSAntCorrectionBlk.find(m_sp3File.m_header.pstrSatNameList[j]);
							if(it_GPSAntCorrectionBlk != mapGPSAntCorrectionBlk.end())
							{
								if(m_sp3File.m_header.pstrSatNameList[j][0] == 'G')								
									correct_gpspco = m_AtxFile.correctSatAntPCOPCV(it_GPSAntCorrectionBlk->second, 1, posclkRec.getPos(), sp3Datum.pos, sunPos, true);	
								else if(m_sp3File.m_header.pstrSatNameList[j][0] == 'C')
								{//按照block IIA卫星修正									
									if(nPRN > 5)  // IGSO卫星和MEO卫星PCO修正									
										correct_gpspco = m_AtxFile.correctSatAntPCOPCV(it_GPSAntCorrectionBlk->second, 0, posclkRec.getPos(), sp3Datum.pos, sunPos, true);														
									else  // GEO卫星PCO修正，2014/08/24，刘俊宏
									{
										POS3D vecLOS = vectorNormal(posclkRec.getPos() - sp3Datum.pos);	// 视线单位矢量，卫星指向测站																					
										POS3D axisvec_R, axisvec_T, axisvec_N;	
										axisvec_R = vectorNormal(sp3Datum.pos) * (-1.0);//轨道系坐标轴 R 对应星体系的"-Z"方向	
										POS3D eV  = vectorNormal(sp3Datum.vel);
										vectorCross(axisvec_N,axisvec_R,eV);
										axisvec_N = vectorNormal(axisvec_N);            //轨道系坐标轴 N 对应星体系的"-Y"方向	
										vectorCross(axisvec_T,axisvec_N,axisvec_R);
										axisvec_T = vectorNormal(axisvec_T);			//轨道系坐标轴 T 对应星体系的"+X"方向(即ex)										
										correct_gpspco = m_AtxFile.correctSatAntPCOPCV_YawFixed(it_GPSAntCorrectionBlk->second, 0, vecLOS, axisvec_T, axisvec_N * (-1.0), axisvec_R * (-1.0), true);												
									}							
								}
							}							
						}
						// 6 - leo卫星天线PCO改正
						double correct_leopco = 0;
						if(m_simuParaDefine.bOn_Rec_AntPCO && m_sp3File.m_header.pstrSatNameList[j][0] == 'G')
						{// 兼容双模 - BDS + GPS
							// 2019/10/28，韦春博修改，增加卫星姿态信息，利用卫星姿态进行天线相位中心偏移修正
							// 使用依据：判断卫星是否为对日定向巡航模式
							POS3D offsetJ2000 = GNSSBasicCorrectFunc::correctLeoAntPCO_J2000(m_pcoAnt, posclkRec.getPos(), m_pvOrbList[s_i].vel);
							correct_leopco = -(offsetJ2000.x * E.x + offsetJ2000.y * E.y + offsetJ2000.z * E.z);
						}
						if(m_simuParaDefine.bOn_Rec_AntPCV_bds && m_sp3File.m_header.pstrSatNameList[j][0] == 'C')
						{
							// 2019/10/28，韦春博修改，增加卫星姿态信息，利用卫星姿态进行天线相位中心偏移修正
							// 使用依据：判断卫星是否为对日定向巡航模式
							POS3D offsetJ2000 = GNSSBasicCorrectFunc::correctLeoAntPCO_J2000(m_pcoAnt_bds, posclkRec.getPos(), m_pvOrbList[s_i].vel);
							correct_leopco = -(offsetJ2000.x * E.x + offsetJ2000.y * E.y + offsetJ2000.z * E.z);
						}
						// 7 - LEO卫星天线 PCV 修正 (在轨天线校准结果)
						double correct_leopcv = 0;
						if(m_simuParaDefine.bOn_Rec_AntPCV && m_sp3File.m_header.pstrSatNameList[j][0] == 'G')
						{// 计算天空视图
							POS3D vecLos = sp3Datum.pos - posclkRec.getPos(); // 视线矢量: 接收机位置指向GPS卫星
							POS3D vecLosXYZ;
							vecLosXYZ.x = vectorDot(vecLos, S_X);
							vecLosXYZ.y = vectorDot(vecLos, S_Y);
							vecLosXYZ.z = vectorDot(vecLos, S_Z);
							vecLosXYZ = vectorNormal(vecLosXYZ);
							double Elevation = 90 - acos(vecLosXYZ.z) * 180 / PI;
							double Azimuth = atan2(vecLosXYZ.y, vecLosXYZ.x) * 180 / PI;
							if(Azimuth < 0)
							{
								Azimuth += 360.0;// 变换到[0, 360]
							}
							correct_leopcv = m_pcvFile.getPCVValue(Elevation, Azimuth);
						}
						if(m_simuParaDefine.bOn_Rec_AntPCV_bds && m_sp3File.m_header.pstrSatNameList[j][0] == 'C')
						{// 计算天空视图
							POS3D vecLos = sp3Datum.pos - posclkRec.getPos(); // 视线矢量: 接收机位置指向GPS卫星
							POS3D vecLosXYZ;
							vecLosXYZ.x = vectorDot(vecLos, S_X);
							vecLosXYZ.y = vectorDot(vecLos, S_Y);
							vecLosXYZ.z = vectorDot(vecLos, S_Z);
							vecLosXYZ = vectorNormal(vecLosXYZ);
							double Elevation = 90 - acos(vecLosXYZ.z) * 180 / PI;
							double Azimuth = atan2(vecLosXYZ.y, vecLosXYZ.x) * 180 / PI;
							if(Azimuth < 0)
							{
								Azimuth += 360.0;// 变换到[0, 360]
							}
							correct_leopcv = m_pcvFile_bds.getPCVValue(Elevation, Azimuth);
						}
						// 8 - 相位缠绕改正
						double correct_phasewindup_L1 = 0;
						double correct_phasewindup_L2 = 0;
						double correct_phasewindup_L5 = 0;
						if(m_simuParaDefine.bOn_PhaseWindUp)
						{
							if(m_sp3File.m_header.pstrSatNameList[j][0] == 'G')
							{
								phase_windup[j] = GNSSBasicCorrectFunc::correctLeoAntPhaseWindUp(ppBlockID[nPRN], m_pvOrbList[s_i].getPosVel(), sp3Datum.pos, sunPos, phase_windup[j]);
								//correct_phasewindup = phase_windup[j] * SPEED_LIGHT / ((GPS_FREQUENCE_L1 + GPS_FREQUENCE_L2)); // 20141126, 与频率相关
								correct_phasewindup_L1 = phase_windup[j] * GPS_WAVELENGTH_L1;
								correct_phasewindup_L2 = phase_windup[j] * GPS_WAVELENGTH_L2;

							}
							else if(m_sp3File.m_header.pstrSatNameList[j][0] == 'C' && nPRN > 5)//按照block IIA卫星修正
							{
								phase_windup[j] = GNSSBasicCorrectFunc::correctLeoAntPhaseWindUp(1, m_pvOrbList[s_i].getPosVel(), sp3Datum.pos, sunPos, phase_windup[j]);
								//correct_phasewindup = phase_windup[j] * SPEED_LIGHT / ((BD_FREQUENCE_L1 + BD_FREQUENCE_L2)); // 20141126, 与频率相关		
								correct_phasewindup_L1 = phase_windup[j] * BD_WAVELENGTH_L1;
								correct_phasewindup_L2 = phase_windup[j] * BD_WAVELENGTH_L2;
								correct_phasewindup_L5 = phase_windup[j] * BD_WAVELENGTH_L5;
							}
						}
						// 9 - 系统间偏差修正
						double  correct_sysBias = 0;
						if(m_simuParaDefine.bOn_SysBias)
						{
							if(m_sp3File.m_header.pstrSatNameList[j][0] != 'G')	
								correct_sysBias = m_simuParaDefine.sysBias;							
						}
						distance = distance
							     + recClock
								 - correct_RecRelativity // 与解算修正符号相反
								 - correct_GPSRelativity // 与解算修正符号相反
								 - correct_GPSClock      // 与解算修正符号相反
								 - correct_gpspco        // 与解算修正符号相反
								 - correct_leopco        // 与解算修正符号相反
								 + correct_leopcv
								 + correct_sysBias;      // 与解算修正符号相反
								 //+ correct_phasewindup;  // 与解算修正符号相反
						/*if(s_i <= 10)
						{
							char info[200];
							sprintf(info, "%02d   %s %20.8lf%20.8lf %20.8lf %20.8lf", s_i, m_sp3File.m_header.pstrSatNameList[j].c_str(), distance, correct_GPSClock, correct_GPSRelativity, correct_RecRelativity);
							RuningInfoFile::Add(info);
						}*/
						// 9 - 电离层延迟改正
						double ion_L1 = 0.0;
						double ion_L2 = 0.0;
						double ion_L5 = 0.0;
						if(m_simuParaDefine.bOn_Ionosphere)
						{
							if(!m_ionFile.isEmpty())
							{
								POS3D leoPos_ECEF, gpsPos_ECEF;// 转换到地固系
								double x_ecf[3];
								double x_j2000[3];
								x_j2000[0] = posclkRec.x;  
								x_j2000[1] = posclkRec.y;  
								x_j2000[2] = posclkRec.z;
								m_TimeCoordConvert.J2000_ECEF(t_Receive,  x_j2000, x_ecf, false);
								leoPos_ECEF.x = x_ecf[0];
								leoPos_ECEF.y = x_ecf[1]; 
								leoPos_ECEF.z = x_ecf[2];
								x_j2000[0] = sp3Datum.pos.x;  
								x_j2000[1] = sp3Datum.pos.y;  
								x_j2000[2] = sp3Datum.pos.z;
								m_TimeCoordConvert.J2000_ECEF(t_Transmit, x_j2000, x_ecf, false);
								gpsPos_ECEF.x = x_ecf[0];
								gpsPos_ECEF.y = x_ecf[1]; 
								gpsPos_ECEF.z = x_ecf[2];
								if(m_sp3File.m_header.pstrSatNameList[j][0] == 'G')
								{
									GNSSBasicCorrectFunc::ionexGridCorrect_IP(m_ionFile, t_Receive, leoPos_ECEF, gpsPos_ECEF, ion_L1, 80000, GPS_FREQUENCE_L1); // 20141126, 与频率相关
									GNSSBasicCorrectFunc::ionexGridCorrect_IP(m_ionFile, t_Receive, leoPos_ECEF, gpsPos_ECEF, ion_L2, 80000, GPS_FREQUENCE_L2); // 20141126, 与频率相关
								}
								else if(m_sp3File.m_header.pstrSatNameList[j][0] == 'C')
								{
									GNSSBasicCorrectFunc::ionexGridCorrect_IP(m_ionFile, t_Receive, leoPos_ECEF, gpsPos_ECEF, ion_L1, 80000, BD_FREQUENCE_L1); // 20141126, 与频率相关
									GNSSBasicCorrectFunc::ionexGridCorrect_IP(m_ionFile, t_Receive, leoPos_ECEF, gpsPos_ECEF, ion_L2, 80000, BD_FREQUENCE_L2); // 20141126, 与频率相关
									GNSSBasicCorrectFunc::ionexGridCorrect_IP(m_ionFile, t_Receive, leoPos_ECEF, gpsPos_ECEF, ion_L5, 80000, BD_FREQUENCE_L5); // 20141126, 与频率相关
								}
							}
						}					
						// 9 - 添加观测数据
						if(m_simuParaDefine.bOn_C1 || m_simuParaDefine.bOn_C1_bds)
						{// C1
							if((m_simuParaDefine.bOn_C1 && m_sp3File.m_header.pstrSatNameList[j][0] == 'G')
							|| (m_simuParaDefine.bOn_C1_bds && m_sp3File.m_header.pstrSatNameList[j][0] == 'C'))
							{
								Rinex2_1_ObsDatum C1;
								C1.data  = distance + Error_C1_List[j] + correct_phasewindup_L1 + ion_L1;
								obsTypeList.push_back(C1);
							}
							else
							{// 缺少该类型
								Rinex2_1_ObsDatum C1;
								C1.data  = DBL_MAX;
								obsTypeList.push_back(C1);
							}
						}
						if(m_simuParaDefine.bOn_P1 || m_simuParaDefine.bOn_P1_bds)
						{// P1
							if((m_simuParaDefine.bOn_P1 && m_sp3File.m_header.pstrSatNameList[j][0] == 'G')
							|| (m_simuParaDefine.bOn_P1_bds && m_sp3File.m_header.pstrSatNameList[j][0] == 'C'))
							{
								Rinex2_1_ObsDatum P1;
								P1.data  = distance + Error_P1_List[j] + correct_phasewindup_L1 + ion_L1;
								obsTypeList.push_back(P1);
							}
							else
							{// 缺少该类型
								Rinex2_1_ObsDatum P1;
								P1.data  = DBL_MAX;
								obsTypeList.push_back(P1);
							}
						}
						if(m_simuParaDefine.bOn_P2 || m_simuParaDefine.bOn_P2_bds)
						{// P2
							if((m_simuParaDefine.bOn_P2 && m_sp3File.m_header.pstrSatNameList[j][0] == 'G')
							|| (m_simuParaDefine.bOn_P2_bds && m_sp3File.m_header.pstrSatNameList[j][0] == 'C'))
							{
								Rinex2_1_ObsDatum P2;
								P2.data  = distance + Error_P2_List[j] + correct_phasewindup_L2 + ion_L2;
								obsTypeList.push_back(P2);
							}
							else
							{// 缺少该类型
								Rinex2_1_ObsDatum P2;
								P2.data  = DBL_MAX;
								obsTypeList.push_back(P2);
							}
						}
						if(m_simuParaDefine.bOn_P5_bds )
						{// P5-BDS
							if(m_sp3File.m_header.pstrSatNameList[j][0] == 'C')
							{
								Rinex2_1_ObsDatum P5;
								P5.data  = distance + Error_P5_List[j] + correct_phasewindup_L5 + ion_L5;
								obsTypeList.push_back(P5);
							}
							else
							{// 缺少该类型
								Rinex2_1_ObsDatum P5;
								P5.data  = DBL_MAX;
								obsTypeList.push_back(P5);
							}
						}
						if(m_simuParaDefine.bOn_L1 || m_simuParaDefine.bOn_L1_bds)
						{// L1-GPS/BDS
							if(m_simuParaDefine.bOn_L1 && m_sp3File.m_header.pstrSatNameList[j][0] == 'G')
							{
								Rinex2_1_ObsDatum L1;
								L1.data  = distance + Error_L1_List[j] + correct_phasewindup_L1 - ion_L1 + initPhase_L1_List[j] * GPS_WAVELENGTH_L1;
								L1.data  = L1.data / GPS_WAVELENGTH_L1;
								obsTypeList.push_back(L1);
							}
							else if(m_simuParaDefine.bOn_L1_bds && m_sp3File.m_header.pstrSatNameList[j][0] == 'C')
							{
								Rinex2_1_ObsDatum L1;
								L1.data  = distance + Error_L1_List[j] + correct_phasewindup_L1 - ion_L1 + initPhase_L1_List[j] * BD_WAVELENGTH_L1;
								L1.data  = L1.data / BD_WAVELENGTH_L1;
								obsTypeList.push_back(L1);
							}
							else
							{// 缺少该类型
								Rinex2_1_ObsDatum L1;
								L1.data  = DBL_MAX;
								obsTypeList.push_back(L1);
							}
						}
						if(m_simuParaDefine.bOn_L2 || m_simuParaDefine.bOn_L2_bds)
						{// L2-GPS/BDS
							if(m_simuParaDefine.bOn_L2 && m_sp3File.m_header.pstrSatNameList[j][0] == 'G')
							{
								Rinex2_1_ObsDatum L2;
								L2.data  = distance + Error_L2_List[j] + correct_phasewindup_L2 - ion_L2 + initPhase_L2_List[j] * GPS_WAVELENGTH_L2;
								L2.data  = L2.data / GPS_WAVELENGTH_L2;
								obsTypeList.push_back(L2);
							}
							else if(m_simuParaDefine.bOn_L1_bds && m_sp3File.m_header.pstrSatNameList[j][0] == 'C')
							{
								Rinex2_1_ObsDatum L2;
								L2.data  = distance + Error_L2_List[j] + correct_phasewindup_L2 - ion_L2 + initPhase_L2_List[j] * BD_WAVELENGTH_L2;
								L2.data  = L2.data / BD_WAVELENGTH_L2;
								obsTypeList.push_back(L2);
							}
							else
							{// 缺少该类型
								Rinex2_1_ObsDatum L2;
								L2.data  = DBL_MAX;
								obsTypeList.push_back(L2);
							}
						}
						if(m_simuParaDefine.bOn_L5_bds)
						{// L5-BDS
						    if(m_sp3File.m_header.pstrSatNameList[j][0] == 'C')
							{
								Rinex2_1_ObsDatum L5;
								L5.data  = distance + Error_L5_List[j] + correct_phasewindup_L5 - ion_L5 + initPhase_L5_List[j] * BD_WAVELENGTH_L5;
								L5.data  = L5.data / BD_WAVELENGTH_L5;
								obsTypeList.push_back(L5);
							}
							else
							{// 缺少该类型
								Rinex2_1_ObsDatum L5;
								L5.data  = DBL_MAX;
								obsTypeList.push_back(L5);
							}
						}						
						obsEpoch.obs.insert(Rinex2_1_MixedSatMap::value_type(m_sp3File.m_header.pstrSatNameList[j], obsTypeList));
					}
				}
				obsEpoch.bySatCount = int(obsEpoch.obs.size());
				if(obsEpoch.bySatCount > 0)
				{ 
					obsFile.m_data.push_back(obsEpoch);
				}
			}
			// 整理文件头
			size_t nCount   = obsFile.m_data.size();
			if(nCount == 0)
				return false;
			obsFile.m_header.bySatCount = 0;
			BYTE byGPSSatCount = 0;
			BYTE byBDSSatCount = 0;
			for(int i = 0; i < int(m_sp3File.m_header.pstrSatNameList.size()); i++)
			{
				if(bySatList[i] == 1)
				{
					obsFile.m_header.bySatCount++;
					if(m_sp3File.m_header.pstrSatNameList[i][0] == 'G')
						byGPSSatCount++;
					if(m_sp3File.m_header.pstrSatNameList[i][0] == 'C')
						byBDSSatCount++;
				}
				
			}
			GPST tmStart = obsFile.m_data[0].t;        
			GPST tmEnd   = obsFile.m_data[nCount-1].t; 
			obsFile.m_header.tmStart = tmStart;
			obsFile.m_header.tmEnd   = tmEnd;
			sprintf(obsFile.m_header.szTimeType, "%-3s", "GPS");
			obsFile.m_header.byObsTypes = 0;
			obsFile.m_header.pbyObsTypeList.clear();
			if((m_simuParaDefine.bOn_C1 && byGPSSatCount > 0)
			|| (m_simuParaDefine.bOn_C1_bds && byBDSSatCount > 0))
			{
				obsFile.m_header.pbyObsTypeList.push_back(TYPE_OBS_C1);
				obsFile.m_header.byObsTypes++;
			}
			if((m_simuParaDefine.bOn_P1 && byGPSSatCount > 0) 
			|| (m_simuParaDefine.bOn_P1_bds && byBDSSatCount > 0))
			{
				obsFile.m_header.pbyObsTypeList.push_back(TYPE_OBS_P1);
				obsFile.m_header.byObsTypes++;
			}
			if((m_simuParaDefine.bOn_P2 && byGPSSatCount > 0) 
			|| (m_simuParaDefine.bOn_P2_bds && byBDSSatCount > 0))
			{
				obsFile.m_header.pbyObsTypeList.push_back(TYPE_OBS_P2);
				obsFile.m_header.byObsTypes++;
			}
			if(m_simuParaDefine.bOn_P5_bds && byBDSSatCount > 0)
			{
				obsFile.m_header.pbyObsTypeList.push_back(TYPE_OBS_P5);
				obsFile.m_header.byObsTypes++;
			}
			if((m_simuParaDefine.bOn_L1 && byGPSSatCount > 0)
			|| (m_simuParaDefine.bOn_L1_bds && byBDSSatCount > 0))
			{
				obsFile.m_header.pbyObsTypeList.push_back(TYPE_OBS_L1);
				obsFile.m_header.byObsTypes++;
			}
			if((m_simuParaDefine.bOn_L2 && byGPSSatCount > 0)
			|| (m_simuParaDefine.bOn_L2_bds && byBDSSatCount > 0))
			{
				obsFile.m_header.pbyObsTypeList.push_back(TYPE_OBS_L2);
				obsFile.m_header.byObsTypes++;
			}
			if(m_simuParaDefine.bOn_L5_bds && byBDSSatCount > 0)
			{
				obsFile.m_header.pbyObsTypeList.push_back(TYPE_OBS_L5);
				obsFile.m_header.byObsTypes++;
			}
			sprintf(obsFile.m_header.szRinexVersion,   "     2.1            ");
			sprintf(obsFile.m_header.szFileType,       "OBSERVATION DATA    ");
			if(byGPSSatCount > 0 && byBDSSatCount > 0)// 2014/04/06, 增加混合系统的描述
				sprintf(obsFile.m_header.szSatlliteSystem, "M (MIXED)           ");
			else
			{
				if(byGPSSatCount > 0) 
					sprintf(obsFile.m_header.szSatlliteSystem, "G (GPS)             ");
				if(byBDSSatCount > 0) // 2013/01/02, 增加北斗系统的描述
					sprintf(obsFile.m_header.szSatlliteSystem, "C (BDS)             ");
			}
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
			if(byBDSSatCount > 0)
				obsFile.m_header.bL5WaveLengthFact = 1;
			obsFile.m_header.Interval = m_pvOrbList[1].t - m_pvOrbList[0].t;
			char szComment[100];
			sprintf(szComment,"%-60s%20s\n", "created by GNSS observation simulation program.", Rinex2_1_MaskString::szComment);
			obsFile.m_header.pstrCommentList.push_back(szComment);
			return true;
		}
	}
}
