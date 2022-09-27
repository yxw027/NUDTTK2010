#include "TQGNSSObsSimu.hpp"
#include "MathAlgorithm.hpp"
#include "RuningInfoFile.hpp"

using namespace NUDTTK::Math;
namespace NUDTTK
{
	namespace TQGNSSPod
	{
		TQGNSSSimu::TQGNSSSimu(void)
		{
		}

		TQGNSSSimu::~TQGNSSSimu(void)
		{
		}

		bool TQGNSSSimu::loadSP3File(string  strSP3FileName)
		{
			return m_sp3File.open(strSP3FileName);
		}

		bool TQGNSSSimu::loadCLKFile(string  strCLKFileName)
		{
			return m_clkFile.open(strCLKFileName);
		}

		bool TQGNSSSimu::loadCLKFile_rinex304(string  strCLKFileName)
		{
			return m_clkFile.open_rinex304(strCLKFileName);
		}

		void TQGNSSSimu::setAntPCO(double x, double y, double z)
		{
			m_pcoAnt.x = x;
			m_pcoAnt.y = y;
			m_pcoAnt.z = z;
		}

		// 子程序名称： judgeGNSSSignalCover   
		// 功能：根据GNSS卫星位置、接收机位置和天线的仰角确定信号是否可见(未考虑卫星姿态的影响)
		// 变量类型：heoGNSSsys_i        : 系统参数
        //           strSatName          : 卫星名字
        //           posGPSSat           : GPS卫星位置
		//           posRec              : 接收机位置
		//           freq_i              : 频率
		//           cut_elevation       : 天线的截至高度角
		// 输入：posGPSSat, posRec, cut_elevation
		// 输出：bSee
		// 其它：
		// 语言：C++
		// 版本号：2021/11/07
		// 生成者：杨诚鋆
		// 修改者：
		//20220706 童礼胜注释
		//bool TQGNSSSimu::judgeGNSSSignalCover(TQGNSS_MixedSys heoGNSSsys_i, string strSatName, POS3D posGNSSSat, POS3D posRec, double freq_i, double cut_elevation)
		//{
		//	// 根据时间+卫星名称判断卫星类型
  //          SvNavMixedLine mixedLine;
		//	if(!m_svnavMixedFile.getSvNavInfo(m_sp3File.m_data.front().t, strSatName, mixedLine))
		//	{
		//		printf("警告：%s卫星Block信息缺失.\n", strSatName);
		//		return false;
		//	}
		//	string    nameBlock;
		//	nameBlock = mixedLine.szBlock;
		//	int flag_Block = 0; // 卫星轨道类型，默认为MEO，对应flag为0  
		//	if(nameBlock.find(BLOCK_MaskString::BEIDOU_3I) != -1 
		//	|| nameBlock.find(BLOCK_MaskString::BEIDOU_3G) != -1
		//	|| nameBlock.find(BLOCK_MaskString::BEIDOU_3G_CAST) != -1
		//	|| nameBlock.find(BLOCK_MaskString::BEIDOU_3SI_CAST) != -1
		//	|| nameBlock.find(BLOCK_MaskString::BEIDOU_3SI_SECM) != -1)
		//		flag_Block = 1; // IGSO/GEO
		//	bool bSee = false;
		//	// 接收机到GNSS位置矢量
		//	POS3D R = posRec - posGNSSSat;
		//	double distance1 = sqrt(R.x * R.x + R.y * R.y + R.z * R.z);
		//	// GNSS 到地心距离
		//	double distance2 = sqrt(posGNSSSat.x * posGNSSSat.x + posGNSSSat.y * posGNSSSat.y + posGNSSSat.z * posGNSSSat.z);
		//	// GNSS 卫星到地球边缘的切线长度 1:MEO 2:GEO
		//	double DistanceEarth1 = distance2 * cos(13.23 * PI / 180.0);
		//	double DistanceEarth2 = distance2 * cos(8.71 * PI / 180.0);
		//	double value         = -vectorDot(R, posGNSSSat);
		//	value  = value / (distance1 * distance2);
		//	// 由 GNSS 到接收机的视线与中心线夹角
		//	double Sita = acos(value) * 180 / PI;			double N0=-203.98;
		//	double loss=20*log10(SPEED_LIGHT/heoGNSSsys_i.freq_L1/PI/distance1/4);
		//	double Pr;
		//	//double CN0=Pr-N0;
		//	//double testS = heoGNSSsys_i.minReceiveSignalPower-(10*log10(heoGNSSsys_i.transPowerMainlobe)+heoGNSSsys_i.satAntMaximumGain+m_simuParaDefine.recMaximumGain+20*log10(SPEED_LIGHT/heoGNSSsys_i.freq_L1/PI/distance1/4));
		//	//printf("C/N0=%f     m_simuParaDefine.recMaximumGain=%f    distance1=%f loss=%f  Pr=%f \n",CN0,m_simuParaDefine.recMaximumGain,distance1,loss,Pr);
		//	if(flag_Block == 0) //MEO 卫星可视角度判断
		//	{
		//		if(heoGNSSsys_i.bOn_SidelobeSignal)
		//		{// 旁瓣信号
		//			
		//			if(Sita >= heoGNSSsys_i.angel_meo)
		//			{
		//				if(Sita <= heoGNSSsys_i.coverAngleMainlobe/2)
		//				{// 主瓣信号
		//					Pr=10*log10(heoGNSSsys_i.transPowerMainlobe)+heoGNSSsys_i.satAntMaximumGain+m_simuParaDefine.recMaximumGain+loss;
		//					//if(heoGNSSsys_i.minReceiveSignalPower-(10*log10(heoGNSSsys_i.transPowerMainlobe)+heoGNSSsys_i.satAntMaximumGain+m_simuParaDefine.recMaximumGain+20*log(SPEED_LIGHT/heoGNSSsys_i.freq_L1/PI/distance1/4))>=m_simuParaDefine.receiverThreshold)
		//					//if(heoGNSSsys_i.transPowerMainlobe >= m_simuParaDefine.recMaximumGain)
		//					if(Pr-N0 >=m_simuParaDefine.receiverThreshold)
		//						bSee = true;
		//					else
		//						bSee = false;
		//				}
		//				else if(Sita <= heoGNSSsys_i.coverAngleSidelobe/2 && Sita > heoGNSSsys_i.coverAngleMainlobe/2)
		//				{ // 旁瓣信号可见
		//					Pr=10*log10(heoGNSSsys_i.transPowerMainlobe)-10+heoGNSSsys_i.satAntMaximumGain+m_simuParaDefine.recMaximumGain+loss;//杨诚鋆 2022.03.16修改 旁瓣信号比主瓣信号小10dB Pr=10*log10(heoGNSSsys_i.transPowerSidelobe)+heoGNSSsys_i.satAntMaximumGain+m_simuParaDefine.recMaximumGain+loss;
		//					//bSee = true;
		//					//if(heoGNSSsys_i.transPowerSidelobe >= m_simuParaDefine.recMaximumGain)
		//					if(Pr-N0 >=m_simuParaDefine.receiverThreshold)
		//						bSee = true;
		//					else
		//						bSee = false;
		//				}
		//				else
		//					bSee = false;
		//			}
		//			else
		//				bSee = false;
		//		}
		//		else
		//		{
		//			if(Sita >= heoGNSSsys_i.angel_meo)
		//			{
		//				if(Sita <= heoGNSSsys_i.coverAngleMainlobe/2)
		//				{// 主瓣信号
		//					Pr=10*log10(heoGNSSsys_i.transPowerMainlobe)+heoGNSSsys_i.satAntMaximumGain+m_simuParaDefine.recMaximumGain+loss;
		//					// 待修改，如何判断信号强度？与freq_i相关？
		//					//bSee = true;
		//					//if(heoGNSSsys_i.transPowerMainlobe >= m_simuParaDefine.recMaximumGain)
		//					if(Pr-N0 >=m_simuParaDefine.receiverThreshold)
		//						bSee = true;
		//					else
		//						bSee = false;
		//				}
		//				else
		//					bSee = false;
		//			}
		//			else
		//				bSee = false;
		//		}
		//	}
		//	else//IGSO/GEO卫星可视角度判断 flag_Block = 1
		//	{
		//		// BDS IGSO/GEO卫星与地球切线夹角为 8.71°
		//		if(heoGNSSsys_i.bOn_SidelobeSignal)
		//		{// 旁瓣信号
		//			
		//			if(Sita >= heoGNSSsys_i.angel_geo)
		//			{
		//				if(Sita <= heoGNSSsys_i.coverAngleMainlobe/2)
		//				{// 主瓣信号
		//					Pr=10*log10(heoGNSSsys_i.transPowerMainlobe)+heoGNSSsys_i.satAntMaximumGain+m_simuParaDefine.recMaximumGain+loss;
		//					// 待修改，如何判断信号强度？
		//					//bSee = true;
		//					//if(heoGNSSsys_i.transPowerMainlobe >= m_simuParaDefine.recMaximumGain)
		//					if(Pr-N0 >=m_simuParaDefine.receiverThreshold)
		//						bSee = true;
		//					else
		//						bSee = false;
		//				}
		//				else if(Sita <= heoGNSSsys_i.coverAngleSidelobe/2 && Sita > heoGNSSsys_i.coverAngleMainlobe/2)
		//				{ // 旁瓣信号可见
		//					Pr=10*log10(heoGNSSsys_i.transPowerMainlobe)-10+heoGNSSsys_i.satAntMaximumGain+m_simuParaDefine.recMaximumGain+loss;//杨诚鋆 2022.03.16修改 旁瓣信号比主瓣信号小10dB Pr=10*log10(heoGNSSsys_i.transPowerSidelobe)+heoGNSSsys_i.satAntMaximumGain+m_simuParaDefine.recMaximumGain+loss;
		//					//bSee = true;
		//					//if(heoGNSSsys_i.transPowerSidelobe >= m_simuParaDefine.recMaximumGain)
		//					if(Pr-N0 >=m_simuParaDefine.receiverThreshold)
		//						bSee = true;
		//					else
		//						bSee = false;
		//				}
		//				else
		//					bSee = false;
		//			}
		//			else
		//				bSee = false;
		//		}
		//		else
		//		{
		//			if(Sita >= heoGNSSsys_i.angel_geo)
		//			{
		//				if(Sita <= heoGNSSsys_i.coverAngleMainlobe/2)
		//				{// 主瓣信号
		//					Pr=10*log10(heoGNSSsys_i.transPowerMainlobe)+heoGNSSsys_i.satAntMaximumGain+m_simuParaDefine.recMaximumGain+loss;
		//					// 待修改，如何判断信号强度？
		//					//bSee = true;
		//					//if(heoGNSSsys_i.transPowerMainlobe >= m_simuParaDefine.recMaximumGain)
		//					if(Pr-N0 >=m_simuParaDefine.receiverThreshold)
		//						bSee = true;
		//					else
		//						bSee = false;
		//					
		//				}
		//				else
		//					bSee = false;
		//			}
		//			else
		//				bSee = false;
		//		}
		//	}
		//	// 当信号覆盖到后，考虑接收机截至高度角的影响
		//	if(bSee)
		//	{
		//		// 接收机位置矢量
		//		POS3D Vec_DLS = posRec;
		//		Vec_DLS = vectorNormal(Vec_DLS);
		//		// 由接收机到GPS卫星的视线矢量
		//		POS3D Vec_Look = R;
		//		Vec_Look = vectorNormal(Vec_Look);
		//		double value1 = vectorDot(Vec_DLS, Vec_Look);
		//		double Alpha = acos(value1) * 180 / PI;
		//		if(Alpha >= (90 - cut_elevation))
		//			bSee = false;
		//	}
		//	return bSee;
		//}


	    bool TQGNSSSimu::judgeGNSSSignalCover(TQGNSS_MixedSys heoGNSSsys_i, string strSatName, POS3D posGNSSSat, POS3D posRec, POS3D velGNSSSat, double freq_i, double cut_elevation)
		{
			// 根据时间+卫星名称判断卫星类型
            SvNavMixedLine mixedLine;
			if(!m_svnavMixedFile.getSvNavInfo(m_sp3File.m_data.front().t, strSatName, mixedLine))
			{
				printf("警告：%s卫星Block信息缺失.\n", strSatName);
				return false;
			}
			string    nameBlock;
			nameBlock = mixedLine.szBlock;
			int flag_Block = 0; // 卫星轨道类型，默认为MEO，对应flag为0  
			if(nameBlock.find(BLOCK_MaskString::BEIDOU_3I) != -1 
			|| nameBlock.find(BLOCK_MaskString::BEIDOU_3G) != -1
			|| nameBlock.find(BLOCK_MaskString::BEIDOU_3G_CAST) != -1
			|| nameBlock.find(BLOCK_MaskString::BEIDOU_3SI_CAST) != -1
			|| nameBlock.find(BLOCK_MaskString::BEIDOU_3SI_SECM) != -1)
				flag_Block = 1; // IGSO/GEO
			double AntGain = 0;//发射天线增益
			bool bSee = false;
			// 接收机到GNSS位置矢量
			POS3D R = posRec - posGNSSSat;
			double distance1 = sqrt(R.x * R.x + R.y * R.y + R.z * R.z);
			// GNSS 到地心距离
			double distance2 = sqrt(posGNSSSat.x * posGNSSSat.x + posGNSSSat.y * posGNSSSat.y + posGNSSSat.z * posGNSSSat.z);
			// GNSS 卫星到地球边缘的切线长度 1:MEO 2:GEO
			double DistanceEarth1 = distance2 * cos(13.23 * PI / 180.0);
			double DistanceEarth2 = distance2 * cos(8.71 * PI / 180.0);
			double value         = -vectorDot(R, posGNSSSat);
			value  = value / (distance1 * distance2);
			// 由 GNSS 到接收机的视线与中心线夹角
			double Sita = acos(value) * 180 / PI;			
			//double N0=-203.98;//待重新确认???????
			double N0=-205;//典型值
			double loss=20*log10(SPEED_LIGHT/heoGNSSsys_i.freq_L1/PI/distance1/4);
			//读取天线增益文件(PCV格式)
			if(m_simuParaDefine.bOn_AntFile)
			{// 计算天空视图
				// 计算星固系
				POS3D S_Z; // Z轴指向卫星
				POS3D S_X; // X轴沿速度方向
				POS3D S_Y; // 右手系
				S_Z = vectorNormal(posGNSSSat);
				S_X = vectorNormal(velGNSSSat);	                                            
				vectorCross(S_Y, S_Z, S_X);
				S_Y = vectorNormal(S_Y);		                                          
				vectorCross(S_X, S_Y, S_Z);	
				S_X = vectorNormal(S_X);
				AntPCVFile antGainFile;
				char GainFilePath[200];
				sprintf(GainFilePath,"D:\\软件开发_0406\\童礼胜备份\\NUDTTK\\release\\TQGNSS\\GPS_new.pcv");
				////2022.07.05
				//if(!antGainFile.open(GainFilePath))
				//{
				//	printf("天线增益数据无法打开!\n");
				//}
				//else
				//{
				//	printf("打开成功!\n");
				//}
				//sprintf(GainFilePath,"D:\\软件开发_0406\\童礼胜备份\\NUDTTK\\release\\TQGNSS\\GPS_new.pcv");
				//antGainFile.write(GainFilePath);
				antGainFile.open(GainFilePath);
				POS3D vecLos = posRec - posGNSSSat; // 视线矢量: 接收机位置指向GPS卫星
				POS3D vecLosXYZ;
				vecLosXYZ.x = vectorDot(vecLos, S_X);
				vecLosXYZ.y = vectorDot(vecLos, S_Y);
				vecLosXYZ.z = vectorDot(vecLos, S_Z);
				vecLosXYZ = vectorNormal(vecLosXYZ);
				//double Elevation = 90 - acos(vecLosXYZ.z) * 180 / PI;
				double Azimuth = atan2(vecLosXYZ.y, vecLosXYZ.x) * 180 / PI;
				if(Azimuth < 0)
				{
					Azimuth += 360.0;// 变换到[0, 360]
				}
				AntGain = antGainFile.getPCVValue(Sita, Azimuth);//需要定义方位角
			}
			double Pr;
			//double CN0=Pr-N0;
			//double testS = heoGNSSsys_i.minReceiveSignalPower-(10*log10(heoGNSSsys_i.transPowerMainlobe)+heoGNSSsys_i.satAntMaximumGain+m_simuParaDefine.recMaximumGain+20*log10(SPEED_LIGHT/heoGNSSsys_i.freq_L1/PI/distance1/4));
			//printf("C/N0=%f     m_simuParaDefine.recMaximumGain=%f    distance1=%f loss=%f  Pr=%f \n",CN0,m_simuParaDefine.recMaximumGain,distance1,loss,Pr);
			if(flag_Block == 0) //MEO 卫星可视角度判断
			{
				if(heoGNSSsys_i.bOn_SidelobeSignal)
				{// 旁瓣信号
					
					if(Sita >= heoGNSSsys_i.angel_meo)
					{
						if(Sita <= heoGNSSsys_i.coverAngleMainlobe/2)
						{// 主瓣信号
							//Pr=10*log10(heoGNSSsys_i.transPowerMainlobe)+heoGNSSsys_i.satAntMaximumGain+m_simuParaDefine.recMaximumGain+loss;
							Pr=10*log10(heoGNSSsys_i.transPowerMainlobe)+AntGain+m_simuParaDefine.recAntGain+loss;
								//printf("Pr =%15.9f \n", Pr);
							printf("Pr-N0 =%15.9f \n", Pr-N0);
							//if(heoGNSSsys_i.minReceiveSignalPower-(10*log10(heoGNSSsys_i.transPowerMainlobe)+heoGNSSsys_i.satAntMaximumGain+m_simuParaDefine.recMaximumGain+20*log(SPEED_LIGHT/heoGNSSsys_i.freq_L1/PI/distance1/4))>=m_simuParaDefine.receiverThreshold)
							//if(heoGNSSsys_i.transPowerMainlobe >= m_simuParaDefine.recMaximumGain)
							if(Pr-N0 >=m_simuParaDefine.receiverThreshold)
								bSee = true;
							else
								bSee = false;
						}
						else if(Sita <= heoGNSSsys_i.coverAngleSidelobe/2 && Sita > heoGNSSsys_i.coverAngleMainlobe/2)
						{ // 旁瓣信号可见
							//Pr=10*log10(heoGNSSsys_i.transPowerMainlobe)-10+heoGNSSsys_i.satAntMaximumGain+m_simuParaDefine.recMaximumGain+loss;//杨诚鋆 2022.03.16修改 旁瓣信号比主瓣信号小10dB Pr=10*log10(heoGNSSsys_i.transPowerSidelobe)+heoGNSSsys_i.satAntMaximumGain+m_simuParaDefine.recMaximumGain+loss;
							Pr=10*log10(heoGNSSsys_i.transPowerMainlobe)+AntGain+m_simuParaDefine.recAntGain+loss;
								//printf("Pr =%15.9f \n", Pr);
							printf("Pr-N0 =%15.9f \n", Pr-N0);
							//bSee = true;
							//if(heoGNSSsys_i.transPowerSidelobe >= m_simuParaDefine.recMaximumGain)
							if(Pr-N0 >=m_simuParaDefine.receiverThreshold)
								bSee = true;
							else
								bSee = false;
						}
						else
							bSee = false;
					}
					else
						bSee = false;
				}
				else
				{
					if(Sita >= heoGNSSsys_i.angel_meo)
					{
						if(Sita <= heoGNSSsys_i.coverAngleMainlobe/2)
						{// 主瓣信号
							//Pr=10*log10(heoGNSSsys_i.transPowerMainlobe)+heoGNSSsys_i.satAntMaximumGain+m_simuParaDefine.recMaximumGain+loss;//2022.05.09注释
							Pr=10*log10(heoGNSSsys_i.transPowerMainlobe)+AntGain+m_simuParaDefine.recAntGain+loss;
								//printf("Pr =%15.9f \n", Pr);
							printf("Pr-N0 =%15.9f \n", Pr-N0);
							// 待修改，如何判断信号强度？与freq_i相关？
							//bSee = true;
							//if(heoGNSSsys_i.transPowerMainlobe >= m_simuParaDefine.recMaximumGain)
							if(Pr-N0 >=m_simuParaDefine.receiverThreshold)
								bSee = true;
							else
								bSee = false;
						}
						else
							bSee = false;
					}
					else
						bSee = false;
				}
			}
			else//IGSO/GEO卫星可视角度判断 flag_Block = 1
			{
				// BDS IGSO/GEO卫星与地球切线夹角为 8.71°
				if(heoGNSSsys_i.bOn_SidelobeSignal)
				{// 旁瓣信号
					
					if(Sita >= heoGNSSsys_i.angel_geo)
					{
						if(Sita <= heoGNSSsys_i.coverAngleMainlobe/2)
						{// 主瓣信号
							//Pr=10*log10(heoGNSSsys_i.transPowerMainlobe)+heoGNSSsys_i.satAntMaximumGain+m_simuParaDefine.recMaximumGain+loss;//2022.05.09注释
							Pr=10*log10(heoGNSSsys_i.transPowerMainlobe)+AntGain+m_simuParaDefine.recAntGain+loss;
								//printf("Pr =%15.9f \n", Pr);
							printf("Pr-N0 =%15.9f \n", Pr-N0);
							// 待修改，如何判断信号强度？
							//bSee = true;
							//if(heoGNSSsys_i.transPowerMainlobe >= m_simuParaDefine.recMaximumGain)
							if(Pr-N0 >=m_simuParaDefine.receiverThreshold)
								bSee = true;
							else
								bSee = false;
						}
						else if(Sita <= heoGNSSsys_i.coverAngleSidelobe/2 && Sita > heoGNSSsys_i.coverAngleMainlobe/2)
						{ // 旁瓣信号可见
							//Pr=10*log10(heoGNSSsys_i.transPowerMainlobe)-10+heoGNSSsys_i.satAntMaximumGain+m_simuParaDefine.recMaximumGain+loss;//杨诚鋆 2022.03.16修改 旁瓣信号比主瓣信号小10dB Pr=10*log10(heoGNSSsys_i.transPowerSidelobe)+heoGNSSsys_i.satAntMaximumGain+m_simuParaDefine.recMaximumGain+loss;
							Pr=10*log10(heoGNSSsys_i.transPowerMainlobe)+AntGain+m_simuParaDefine.recAntGain+loss;//旁瓣信号判断直接从文件中获取
								//printf("Pr =%15.9f \n", Pr);
							printf("Pr-N0 =%15.9f \n", Pr-N0);
							//bSee = true;
							//if(heoGNSSsys_i.transPowerSidelobe >= m_simuParaDefine.recMaximumGain)
							if(Pr-N0 >=m_simuParaDefine.receiverThreshold)
								bSee = true;
							else
								bSee = false;
						}
						else
							bSee = false;
					}
					else
						bSee = false;
				}
				else
				{
					if(Sita >= heoGNSSsys_i.angel_geo)
					{
						if(Sita <= heoGNSSsys_i.coverAngleMainlobe/2)
						{// 主瓣信号
							//Pr=10*log10(heoGNSSsys_i.transPowerMainlobe)+heoGNSSsys_i.satAntMaximumGain+m_simuParaDefine.recMaximumGain+loss;//2022.05.09注释
							Pr=10*log10(heoGNSSsys_i.transPowerMainlobe)+AntGain+m_simuParaDefine.recAntGain+loss;//旁瓣信号判断直接从文件中获取
								//printf("Pr =%15.9f \n", Pr);
							printf("Pr-N0 =%15.9f \n", Pr-N0);
							// 待修改，如何判断信号强度？
							//bSee = true;
							//if(heoGNSSsys_i.transPowerMainlobe >= m_simuParaDefine.recMaximumGain)
							if(Pr-N0 >=m_simuParaDefine.receiverThreshold)
								bSee = true;
							else
								bSee = false;
							
						}
						else
							bSee = false;
					}
					else
						bSee = false;
				}
			}
			// 当信号覆盖到后，考虑接收机截至高度角的影响
			if(bSee)
			{
				// 接收机位置矢量
				POS3D Vec_DLS = posRec;
				Vec_DLS = vectorNormal(Vec_DLS);
				// 由接收机到GPS卫星的视线矢量
				POS3D Vec_Look = R;
				Vec_Look = vectorNormal(Vec_Look);
				double value1 = vectorDot(Vec_DLS, Vec_Look);
				double Alpha = acos(value1) * 180 / PI;
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
		bool TQGNSSSimu::simuRecClock(double& p_t, double& q_t, double delta, double AllanVar_h_0, double AllanVar_h_2)
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

		// 子程序名称： simuGNSSMixedObsFile   
		// 功能：仿真生成 GNSS 混合原始观测数据文件
		// 变量类型：obsFile           : 观测数据文件
		// 输入：
		// 输出：obsFile
		// 其它：目前兼容双系统(GPS+BDS)双频(L1+L2)的数据仿真
		// 语言：C++
		// 版本号：2014/04/06
		// 生成者：谷德峰
		// 修改者：
		bool TQGNSSSimu::simuGNSSMixedObsFile(Rinex2_1_MixedObsFile &obsFile)
		{					
			//char info[200];
			// 判断观测数据结构和星历数据结构是否为空
			if(m_pvOrbList.size() < 2)
				return false;
			if(m_sp3File.isEmpty())
			{
				printf("警告：星历数据缺失\n");
				return false;
			}
			// 星历数据
			if(m_sp3File.m_data.size() <= 0 || m_clkFile.m_data.size() <= 0)
				return false;
			obsFile.m_data.clear();
			// 根据输入的m_dataMixedSysList，仿真观测数据
            vector<BYTE> bySatList;              // 卫星列表
			bySatList.resize(m_sp3File.m_header.pstrSatNameList.size());
			for(int i = 0; i < int(m_sp3File.m_header.pstrSatNameList.size()); i++)
			{
				bySatList[i] = 0;
			}
			double p_t = 0.0;
			double q_t = 0.0;
			for(size_t s_i = 0; s_i < m_pvOrbList.size(); s_i++)
			{
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
				for(size_t i_sys = 0; i_sys < m_dataMixedSysList.size(); i_sys++)
				{ // 输入的m_dataMixedSysList循环				
					for(int j = 0; j < int(m_sp3File.m_header.pstrSatNameList.size()); j++)
					{ //输入的m_sp3File卫星列表循环
						if(m_sp3File.m_header.pstrSatNameList[j][0] == m_dataMixedSysList[i_sys].cSys)
						{// 当前卫星是需要仿真的卫星
							// 3- GNSS系统相关的观测数据误差
							double Error_P1 = 0.0;
							double Error_P2 = 0.0;
							double Error_L1 = 0.0;
							double Error_L2 = 0.0;
							if(m_simuParaDefine.bOn_Rec_ObsNoise)
							{
								Error_P1 = RandNormal(0, m_dataMixedSysList[i_sys].noiseSigma_P1);
								Error_P2 = RandNormal(0, m_dataMixedSysList[i_sys].noiseSigma_P2);
								Error_L1 = RandNormal(0, m_dataMixedSysList[i_sys].noiseSigma_L1);
								Error_L2 = RandNormal(0, m_dataMixedSysList[i_sys].noiseSigma_L2);
							}
							//Error_P1 = Error_P1 + 1.0;
							//Error_P1 = Error_P1 + 1.0;
							double delay;
							SP3Datum sp3Datum;
							if(!m_sp3File.getEphemeris_PathDelay(m_pvOrbList[s_i].t, posclkRec, m_sp3File.m_header.pstrSatNameList[j], delay, sp3Datum))
							{
								printf("%s 时延获取失败！\n", m_pvOrbList[s_i].t.toString().c_str());
								continue;
							}
							// 对 GPS 卫星星历进行地球自转改正
				            GNSSBasicCorrectFunc::correctSp3EarthRotation(delay, sp3Datum);

							GPST t_Transmit = t_Receive - delay;
							bool bOnfreq_L1 = false;  // 第一个频点数据是否可捕获
							bool bOnfreq_L2 = false;  // 第一个频点数据是否可捕获
							//2022.05.09 童礼胜注释
							/*if(m_dataMixedSysList[i_sys].bOn_P1 || m_dataMixedSysList[i_sys].bOn_L1)
							{
								bOnfreq_L1 = judgeGNSSSignalCover(m_dataMixedSysList[i_sys],m_sp3File.m_header.pstrSatNameList[j], sp3Datum.pos, posclkRec.getPos(), 
									                              m_dataMixedSysList[i_sys].freq_L1,   m_simuParaDefine.min_elevation);
							}*/
							if(m_dataMixedSysList[i_sys].bOn_P1 || m_dataMixedSysList[i_sys].bOn_L1)
							{
								bOnfreq_L1 = judgeGNSSSignalCover(m_dataMixedSysList[i_sys],m_sp3File.m_header.pstrSatNameList[j], sp3Datum.pos, posclkRec.getPos(), sp3Datum.vel,
									                              m_dataMixedSysList[i_sys].freq_L1,   m_simuParaDefine.min_elevation);
							}
							//2022.05.09 童礼胜注释
							/*if(m_dataMixedSysList[i_sys].bOn_P2 || m_dataMixedSysList[i_sys].bOn_L2)
							{
								bOnfreq_L2 = judgeGNSSSignalCover(m_dataMixedSysList[i_sys],m_sp3File.m_header.pstrSatNameList[j], sp3Datum.pos, posclkRec.getPos(), 
									                              m_dataMixedSysList[i_sys].freq_L2,   m_simuParaDefine.min_elevation);
							}*/
							if(m_dataMixedSysList[i_sys].bOn_P2 || m_dataMixedSysList[i_sys].bOn_L2)
							{
								bOnfreq_L2 = judgeGNSSSignalCover(m_dataMixedSysList[i_sys],m_sp3File.m_header.pstrSatNameList[j], sp3Datum.pos, posclkRec.getPos(), sp3Datum.vel,
									                              m_dataMixedSysList[i_sys].freq_L2,   m_simuParaDefine.min_elevation);
							}
							//if(judgeGNSSSignalCover(m_sp3File.m_header.pstrSatNameList[j], sp3Datum.pos, posclkRec.getPos(), m_simuParaDefine.min_elevation))
							if(bOnfreq_L1 || bOnfreq_L2)
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
								// 3 - GNSS卫星的相对论效应
								double 	correct_GPSRelativity = 0;
								if(m_dataMixedSysList[i_sys].bOn_GNSSSAT_Relativity)
								{
									correct_GPSRelativity  = ( sp3Datum.pos.x * sp3Datum.vel.x 
															 + sp3Datum.pos.y * sp3Datum.vel.y
				                           					 + sp3Datum.pos.z * sp3Datum.vel.z) * (-2) / SPEED_LIGHT;
								}
								// 4 - GNSS卫星钟差
								double 	correct_GPSClock = 0;
								if(m_dataMixedSysList[i_sys].bOn_GNSSSAT_Clk)
								{
									CLKDatum clkDatum;
									if(!m_clkFile.getSatClock(t_Transmit, m_sp3File.m_header.pstrSatNameList[j] + " ", clkDatum, 3))
										continue;
									correct_GPSClock = clkDatum.clkBias * SPEED_LIGHT;
								}				
								// 5 GNSS卫星天线 PCO 修正
								double correct_gpspco = 0;
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
										 + correct_sysBias;      // 与解算修正符号相反
								double WAVELENGTH_L1 =  SPEED_LIGHT / m_dataMixedSysList[i_sys].freq_L1; 
								double WAVELENGTH_L2 =  SPEED_LIGHT / m_dataMixedSysList[i_sys].freq_L2; 
								// 9 - 添加观测数据
								if(m_dataMixedSysList[i_sys].bOn_P1 && bOnfreq_L1)
								{// P1
									Rinex2_1_ObsDatum P1;
									P1.data  = distance + Error_P1;
									obsTypeList.push_back(P1);
								}
								else
								{// 缺少该类型
									Rinex2_1_ObsDatum P1;
									P1.data  = DBL_MAX;
									obsTypeList.push_back(P1);
								}
								if(m_dataMixedSysList[i_sys].bOn_P2 && bOnfreq_L1)
								{// P2
									Rinex2_1_ObsDatum P2;
									P2.data  = distance + Error_P2;
									obsTypeList.push_back(P2);
								}	
								else
								{// 缺少该类型
									Rinex2_1_ObsDatum P2;
									P2.data  = DBL_MAX;
									obsTypeList.push_back(P2);
								}
								if(m_dataMixedSysList[i_sys].bOn_L1 && bOnfreq_L2)
								{// L1
									Rinex2_1_ObsDatum L1;
									L1.data  = distance + Error_L1;
									L1.data  = L1.data / WAVELENGTH_L1;
									obsTypeList.push_back(L1);
								}
								else
								{// 缺少该类型
									Rinex2_1_ObsDatum L1;
									L1.data  = DBL_MAX;
									obsTypeList.push_back(L1);
								}
								if(m_dataMixedSysList[i_sys].bOn_L2 && bOnfreq_L2)
								{// L2
									Rinex2_1_ObsDatum L2;
									L2.data  = distance + Error_L2;
									L2.data  = L2.data / WAVELENGTH_L2;
									obsTypeList.push_back(L2);
								}
								else
								{// 缺少该类型
									Rinex2_1_ObsDatum L2;
									L2.data  = DBL_MAX;
									obsTypeList.push_back(L2);
								}
								obsEpoch.obs.insert(Rinex2_1_MixedSatMap::value_type(m_sp3File.m_header.pstrSatNameList[j], obsTypeList));
						   }
						}						
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

            if( byGPSSatCount > 0 || byBDSSatCount > 0 )
			{
				obsFile.m_header.pbyObsTypeList.push_back(TYPE_OBS_P1);
				obsFile.m_header.byObsTypes++;
			}
			if( byGPSSatCount > 0 || byBDSSatCount > 0 )
			{
				obsFile.m_header.pbyObsTypeList.push_back(TYPE_OBS_P2);
				obsFile.m_header.byObsTypes++;
			}
			if( byGPSSatCount > 0 || byBDSSatCount > 0 )
			{
				obsFile.m_header.pbyObsTypeList.push_back(TYPE_OBS_L1);
				obsFile.m_header.byObsTypes++;
			}
			 if( byGPSSatCount > 0 || byBDSSatCount > 0 )
			{
				obsFile.m_header.pbyObsTypeList.push_back(TYPE_OBS_L2);
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