#include "GPSYawAttitudeModel1995.hpp"
#include "RuningInfoFile.hpp"

namespace NUDTTK
{
	GPSYawAttitudeModel1995::GPSYawAttitudeModel1995(void)
	{
	}

	GPSYawAttitudeModel1995::~GPSYawAttitudeModel1995(void)
	{
	}

	// 子程序名称： nominalYawAttitude   
	// 功能：计算标称偏航姿态角及其变化率
	// 变量类型：id_Block            : 卫星的 block 编号
    //           sunPos              : 太阳的位置, 惯性系, 单位：米
	//           gpsPosVel           : gps卫星的轨道位置、速度, 惯性系, 单位：米
	//           yaw                 : 偏航姿态角, 单位：度
	//           yawRate             : 偏航姿态角变化率
	//           beta                : 太阳入射方向与轨道面夹角β,单位：弧度
	//           b                   : 卫星控制系统的偏航姿态角偏差, starting November 1995 the bias is set to +0.5 for all satellites, 1996 Bar Sever
	// 输入：sunPos, gpsPos
	// 输出：id_Block, yaw, yawRate, beta
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2014/11/2
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool GPSYawAttitudeModel1995::nominalYawAttitude(int id_Block, POS3D sunPos, POS6D gpsPosVel, double& yaw, double &yawRate, double &beta, double b)
	{
		// 1. 计算轨道法向矢量 n
		POS3D unit_sun    = vectorNormal(sunPos);
		POS3D unit_gpspos = vectorNormal(gpsPosVel.getPos());
		POS3D unit_gpsvel = vectorNormal(gpsPosVel.getVel());
		POS3D unit_n;
		vectorCross(unit_n, unit_gpspos, unit_gpsvel);
		unit_n = vectorNormal(unit_n);
		// 2. 计算太阳入射方向与轨道面夹角β
		beta =  PI / 2 - acos(vectorDot(unit_n, unit_sun));
		// 3. 计算GPS卫星轨道面内的yaw起始点: 午夜矢量轴; 计算轨道面内角μ
		POS3D unit_y;
		POS3D unit_midnight;
		vectorCross(unit_y, unit_n, unit_sun * (-1.0));
		unit_y = vectorNormal(unit_y);
		vectorCross(unit_midnight, unit_y, unit_n);
		unit_midnight = vectorNormal(unit_midnight);
		//double u = atan2(vectorDot(unit_gpspos, unit_y), vectorDot(unit_gpspos, unit_midnight));
		double sinu = vectorDot(unit_gpspos, unit_y);
		double cosu = vectorDot(unit_gpspos, unit_midnight);
		// 4. 计算偏航角 yaw 和 yawRate		
		if(id_Block < 4 || id_Block > 6)
			yaw = atan2(-tan(beta), sinu);		
		else
			yaw = atan2(tan(beta), -sinu);	// block IIR	
		double uRate = 0.0083 * PI / 180;
		yawRate = uRate * tan(beta) * cosu / (sinu * sinu + tan(beta) * tan(beta));
		yaw = yaw * 180.0 / PI;		
		yawRate = yawRate * 180.0 / PI;

		// 5. 计算由于 b 引入的偏航角部分 B 
		//double E = PI - acos(vectorDot(unit_sun, unit_gpspos)); // 地球-航天器-太阳张角 = pi - (卫星-地球-太阳张角) [0, pi]

		double E = acos(cos(beta) * cosu); 
		double deg_E = E * 180.0 / PI;

		//if(deg_E <= 0.5013 || deg_E >= (180.0 - 0.5013)) // 公式 3 奇异
		//	return false;

		if(deg_E <= 1.0)
			E = 1.0 * PI / 180.0;
		if(deg_E >= (180.0 - 1.0))
			E = 179.0 * PI / 180.0;

		double B = asin(0.0175 * b / sin(E)); // 公式 3
		double BRate = -0.0175 * b * cos(E) * cos(beta) * sinu * uRate / (cos(beta) * pow(sin(E), 3));
		B = B * 180.0 / PI;
		BRate = BRate * 180.0 / PI;

		yaw = yaw + B;
		yawRate = yawRate + BRate;

		//// 测试代码: 二者差异很小, 如果忽略由于 B 带来的影响, 差异标准差在 6.1626e-005, 可以忽略
		//{
		//	// GPS卫星理论坐标系
		//	POS3D vecRs =  sunPos - gpsPosVel.getPos(); 
		//	POS3D ez =  vectorNormal(gpsPosVel.getPos()) * (-1.0);
		//	POS3D ns =  vectorNormal(vecRs);
		//	POS3D ey;
		//	vectorCross(ey, ez, ns); 
		//	ey = vectorNormal(ey);
		//	POS3D ex;
		//	vectorCross(ex, ey, ez);
		//	ex = vectorNormal(ex);

		//	if(id_Block >= 4 && id_Block <= 6)
		//	{// BLK: 1=Blk I  2=Blk II 3=Blk IIA 4=Blk IIR-A 5=Blk IIR-B 6=Blk IIR-M 7=Blk IIF
		//		ex = ex * (-1.0);
		//		ey = ey * (-1.0);	
		//	}

		//	/*double cos_yaw = vectorDot(ex, unit_gpsvel);
		//	char info[200];
		//	sprintf(info, "%2d %14.4lf %14.4lf", id_Block, cos(yaw * PI / 180.0), cos_yaw);
		//	RuningInfoFile::Add(info);*/

		//	/*TimePosVel gpsPVT;
		//	gpsPVT.pos = gpsPosVel.getPos();
		//	gpsPVT.vel = gpsPosVel.getVel();
		//	POS3D ex_yaw, ey_yaw, ez_yaw;
		//	yaw2unitXYZ(gpsPVT, yaw, ex_yaw, ey_yaw, ez_yaw, false);
		//	char info[200];
		//	sprintf(info, "%2d %14.4lf %14.4lf %14.4lf %14.4lf", id_Block, yaw, vectorDot(ex,ex_yaw), vectorDot(ey,ey_yaw), vectorDot(ez,ez_yaw));
		//	RuningInfoFile::Add(info);*/
		//}

		return true;
	}

	// 子程序名称： initGYM95Info   
	// 功能：计算每颗卫星的GYM95模型基本信息, 包括 Noon Turn、Shadow Cross、Shadow Post 的时间列表
	// 变量类型：span_t         : 计算的时间粒度, 默认 30s
	//           bOn_sp3TOJ2000 : 是否需要将sp3星历转为J2000惯性系
	// 输入：span_t, m_sp3File, m_JPLEphFile, m_svnavFile
	// 输出：m_mapGYM95Info
	// 语言：C++
	// 创建者：谷德峰、刘俊宏
	// 创建时间：2014/11/5
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool GPSYawAttitudeModel1995::initGYM95Info(double span_t,bool bOn_sp3TOJ2000)
	{
		if(m_sp3File.m_data.size() == 0)
		{
			printf("GYM95模型星历数据缺失!\n");
			return false;
		}
		if(m_svnavFile.m_data.size() == 0)
		{
			printf("GYM95模型卫星类型数据缺失!\n");
			return false;
		}
		if(bOn_sp3TOJ2000)
		{//将地固系下的星历转为惯性系
			for(size_t s_i = 0; s_i < m_sp3File.m_data.size(); s_i++)
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
		}
		//首先根据m_svnavFile获取卫星的 BLOCK ID
		for(size_t s_i = 0; s_i < m_sp3File.m_header.pstrSatNameList.size(); s_i ++)
		{
			if(m_sp3File.m_header.pstrSatNameList[s_i][0] != 'G')
				continue;
			int id_Sat = SP3File::getSatPRN(m_sp3File.m_header.pstrSatNameList[s_i]);// 获取卫星的PRN编号
			double pco_x, pco_y, pco_z;
			int id_block;
			if(!m_svnavFile.getPCO(m_sp3File.m_data.front().t,  id_Sat, pco_x, pco_y, pco_z, id_block))
			{
				printf("警告：G%02d卫星BlockID信息缺失！\n",id_Sat);
				continue;
			}
			GYM95Sat gym95sat(id_block);	
			gym95sat.yawNoonTurnList.clear();
			gym95sat.yawShadowCrossList.clear();
			gym95sat.yawShadowPostList.clear();
			m_mapGYM95Info.insert(map<int, GYM95Sat>::value_type(id_Sat,gym95sat)); // 谷德峰修改
		}
		double b = 0.5;
		int sign_b = 1;
		if(b < 0)
			sign_b = -1;
		//根据m_mapGYM95Info逐颗卫星搜索yawShadowCrossList,yawNoonTurnList
		//FILE *pfile = fopen("F:\\orbit_beta.txt","w+");		
		for(map<int, GYM95Sat>::iterator it = m_mapGYM95Info.begin();it != m_mapGYM95Info.end();++it)
		{
			GYM95ShadowCrossDatum   shadowCross;
			GYM95ShadowPostDatum    shadowPost;
			GYM95NoonTurnDatum      noonTurn;			
			GPST t0_sp3 = m_sp3File.m_data.front().t;
			GPST t1_sp3 = m_sp3File.m_data.back().t;
			double max_beta = 9 * PI/180; // 可能进入noonTurn的最大beta角,适当扩大搜索范围
			if(it->second.id_Block >= 4)
				max_beta = 7 * PI/180;
			bool bOnNoonTurnIn     = false;	
			bool bOnMidnightTurnIn = false;
			bool bOnShadowIn       = false;		
			bool bOnLastPoint      = false; // 增加最后一点判断
			int sign_D = 1;                 // 用于shadowPost计算
			double yawPost_i_1,yawPost_i;   // 进入shadowPost后上一点的实际yaw和当前点的实际yaw
			int i = 0;
			while(t0_sp3 + i * span_t - t1_sp3 <= 0)
			{
				GPST t_epoch = t0_sp3 + i * span_t;
				double hour = t_epoch.hour +  t_epoch.minute/60.0 + t_epoch.second/3600.0;	
				if(fabs(t_epoch - t1_sp3) < 1e-3)
				{//最后一点,前提是t1_sp3 - t0_sp3为span_t的整数倍
					bOnLastPoint = true;					
				}
				POS3D sunPos;
				TDB t_TDB = TimeCoordConvert::GPST2TDB(t_epoch); // 获得 TDB 时间--提供太阳历参考时间									
				double jd = TimeCoordConvert::DayTime2JD(t_TDB); // 获得儒略日									
				double Pos[3];
				if(!m_JPLEphFile.getSunPos_Delay_EarthCenter(jd, Pos))
				{
					printf("%s获取太阳位置失败!\n",t_epoch.toString().c_str());
					return false;
				}
				sunPos.x = Pos[0] * 1000; 
				sunPos.y = Pos[1] * 1000; 
				sunPos.z = Pos[2] * 1000; 
				POS6D    gpsPosVel;
				SP3Datum sp3Datum;
				if(!m_sp3File.getEphemeris(t_epoch,it->first,sp3Datum))
				{
					i++; // 2014/08/26, 谷德峰修改, 防止死循环
					continue;
				}
				gpsPosVel.setPos(sp3Datum.pos);
				gpsPosVel.setVel(sp3Datum.vel);									
				double yaw,yawRate,beta;
				nominalYawAttitude(it->second.id_Block, sunPos, gpsPosVel,yaw,yawRate,beta,b);				
				double shadowFactor = 1;							
				if(!bOnShadowIn && (it->second.id_Block < 4 || it->second.id_Block > 6)) //block IIR 卫星不考虑地影
				{//step1:判断是否进入地影
					if(GNSSBasicCorrectFunc::judgeGPSEarthShadowManeuver(sunPos,sp3Datum.pos,shadowFactor))
					{						
						shadowCross.ti         = t_epoch;
						shadowCross.yaw_ti     = yaw;
						shadowCross.yawRate_ti = yawRate;
						shadowCross.t1         = t_epoch + ((sign_b * it->second.max_yawRate) - yawRate)/(sign_b * it->second.max_yawRateRate);
						bOnShadowIn            = true;						
						//printf("%02d 卫星 %s(%6.2lf) 进入地影!\n",it->first,t_epoch.toString().c_str(),hour);
						continue;
					}
				}		
				if(bOnShadowIn)
				{//step2:判断是否离开地影，并进入shawdowPost
					if(!GNSSBasicCorrectFunc::judgeGPSEarthShadowManeuver(sunPos,sp3Datum.pos,shadowFactor) || bOnLastPoint)
					{
						double    t1       = shadowCross.t1 - shadowCross.ti;
						shadowCross.te     = t_epoch;
						shadowCross.yaw_te = shadowCross.yaw_ti + shadowCross.yawRate_ti * t1 + 0.5 * sign_b * it->second.max_yawRateRate * t1 * t1
							                 + sign_b * it->second.max_yawRate * (t_epoch - shadowCross.t1);// 
						//将yaw_te,转化到[-180,180];
						if(shadowCross.yaw_te < 0)
							shadowCross.yaw_te = shadowCross.yaw_te  + 720;//转为正角度
						int n = int(floor(shadowCross.yaw_te))/180;
						double res = shadowCross.yaw_te - n * 180;
						if(n % 2 == 0)
							shadowCross.yaw_te = res;
						else
							shadowCross.yaw_te = res - 180;
						shadowCross.yawRate_te = it->second.max_yawRate;
						bOnShadowIn  = false;						
						it->second.yawShadowCrossList.push_back(shadowCross);
						//printf("%02d 卫星 %s(%6.2lf) 离开地影!\n",it->first,t_epoch.toString().c_str(),hour);
						if(bOnLastPoint)
							break;
						//进入shadowPost
						double D  = yaw - shadowCross.yaw_te - floor(( yaw - shadowCross.yaw_te)/360 + 0.5) * 360;						
						if(D < 0)
							sign_D = -1;
						shadowPost.t1 = shadowCross.te + (sign_D * it->second.max_yawRate - sign_b * it->second.max_yawRate)/(sign_D * it->second.max_yawRateRate);
						yawPost_i_1   = shadowCross.yaw_te;
						continue;
					}
				}	
				if(it->second.yawShadowPostList.size() == it->second.yawShadowCrossList.size() - 1)
				{//step3:判断是否离开shadowPost					
					double yaw_real = 0;
					if(t_epoch - shadowPost.t1 < 0)
						yaw_real = shadowCross.yaw_te + sign_b * it->second.max_yawRate * (t_epoch - shadowCross.te) + 0.5 * sign_D * it->second.max_yawRateRate * pow((t_epoch - shadowCross.te),2);
					else
					    yaw_real = shadowCross.yaw_te + sign_b * it->second.max_yawRate * (shadowPost.t1 - shadowCross.te) + 0.5 * sign_D * it->second.max_yawRateRate * pow((shadowPost.t1 - shadowCross.te),2)
						           + sign_D * it->second.max_yawRate * (t_epoch - shadowPost.t1);
					yawPost_i = yaw_real;	
					if((yawPost_i - yaw) * (yawPost_i_1 - yaw) < 0 || bOnLastPoint)
					{//离开shadowPost
						shadowPost.ti         = shadowCross.te;
						shadowPost.te         = t_epoch;
						shadowPost.yaw_ti     = shadowCross.yaw_te;
						shadowPost.yawRate_ti = shadowCross.yawRate_te;
						shadowPost.yaw_te     = yaw;
						shadowPost.yawRate_te = yawRate;
						it->second.yawShadowPostList.push_back(shadowPost);
						//printf("%02d 卫星 %s(%6.2lf) 离开shadowPost!\n",it->first,t_epoch.toString().c_str(),hour);
						if(bOnLastPoint)
							break;
						else
							continue;
					}
					yawPost_i_1 = yawPost_i;
				}
				if(fabs(beta) <= max_beta)//β角太大，不可能进入nooTurn,BLOCK II/IIA |β|= 4.9,IIR |β|=2.4
				{
					if(fabs(yawRate) >= it->second.max_yawRate && !bOnNoonTurnIn && !bOnMidnightTurnIn && !bOnShadowIn)// 进入noonturn的必要条件
					{//step4:判断是否进入noonturn					
						// 计算:卫星-地球-太阳张角
						POS3D unit_sun    = vectorNormal(sunPos);
						POS3D unit_gpspos = vectorNormal(gpsPosVel.getPos());
						double SES = acos(vectorDot(unit_sun, unit_gpspos));
						if(SES < PI/2)
						{//进入noonturn，所有卫星
							noonTurn.ti         = t_epoch;
							noonTurn.yaw_ti     = yaw;
							noonTurn.yawRate_ti = it->second.max_yawRate;
							bOnNoonTurnIn       =  true;							
							//printf("%02d 卫星 %s(%6.2lf) 进入noonTurn!\n",it->first,t_epoch.toString().c_str(),hour);
							continue;
						}
						else
						{//进入midnightturn，仅block IIR 卫星,其余卫星一定在地影内
							noonTurn.ti         = t_epoch;
							noonTurn.yaw_ti     = yaw;
							noonTurn.yawRate_ti = it->second.max_yawRate;									
							bOnMidnightTurnIn   =  true;
							//printf("%02d 卫星 %s(%6.2lf) 进入midnightturn!\n",it->first,t_epoch.toString().c_str(),hour);
							continue;
						}
					}
					if((bOnNoonTurnIn || bOnMidnightTurnIn) && (fabs(yawRate) < it->second.max_yawRate || bOnLastPoint))
					{//step5:判断是否离开noonturn,	midnightturn	
						int sign_beta = 1;
						if(beta < 0)
							sign_beta = -1;	
						if(bOnMidnightTurnIn)
							sign_beta = - sign_beta;						
						double yaw_maxRate = noonTurn.yaw_ti - sign_beta * fabs(it->second.max_yawRate) * (t_epoch - noonTurn.ti);
						//需要将yaw_maxRate 转到[-180,180]????
						if(yawRate > 0)
						{
							if(yaw_maxRate >= yaw || bOnLastPoint)
							{
								noonTurn.te    = t_epoch;																
								it->second.yawNoonTurnList.push_back(noonTurn);	
								if(bOnMidnightTurnIn)
								{
									bOnMidnightTurnIn   =  false;
									//printf("%02d 卫星 %s(%6.2lf) 离开midnightturn!\n",it->first,t_epoch.toString().c_str(),hour);
								}
								else
								{
									bOnNoonTurnIn  = false; //出了noonturn之后将bOnIn改为false
								    //printf("%02d 卫星 %s(%6.2lf) 离开noonTurn!\n",it->first,t_epoch.toString().c_str(),hour);
								}
							}
						}
						else
						{
							if(yaw_maxRate <= yaw || bOnLastPoint)
							{
								noonTurn.te    = t_epoch;															
								it->second.yawNoonTurnList.push_back(noonTurn);
								if(bOnMidnightTurnIn)
								{
									bOnMidnightTurnIn   =  false;
									//printf("%02d 卫星 %s(%6.2lf) 离开midnightturn!\n",it->first,t_epoch.toString().c_str(),hour);
								}
								else
								{
									bOnNoonTurnIn = false; // 出了noonturn之后将bOnIn改为false
								    //printf("%02d 卫星 %s(%6.2lf) 离开noonTurn!\n",it->first,t_epoch.toString().c_str(),hour);
								}
							}
						}
					}
				}				
				i++;

				//fprintf(pfile,"%02d %s %9.4lf %9.4lf %9.4lf %9.4lf %9.4lf\n",
				//				it->first,
				//				t_epoch.toString().c_str(),
				//				hour,
				//				beta * 180 / PI,
				//				yaw,
				//				yawRate,
				//				//E * 180/PI,
				//				it->second.max_yawRate);//				
				
			}
		}
		//fclose(pfile);
		return true;
	}

	// 子程序名称： acsYawAttitude   
	// 功能：姿态控制系统 attitude control subsystem (ACS) 偏航姿态角及其变化率
	// 变量类型：id_Sat                   : 卫星id 号
	//           t                        : 当前时间
	//           yaw                      : 偏航姿态角, 单位：度
	//           yawRate                  : 偏航姿态角变化率
	//           b                        : 卫星控制系统的偏航姿态角偏差, starting November 1995 the bias is set to +0.5 for all satellites, 1996 Bar Sever
	//           bUsed_NominalYawAttitude : 是否使用标称偏航姿态角及其变化率 
	// 输入：id_Sat, t
	// 输出：yaw, yawRate
	// 语言：C++
	// 创建者：谷德峰, 刘俊宏
	// 创建时间：2014/11/5
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool GPSYawAttitudeModel1995::acsYawAttitude(int id_Sat, GPST t, double& yaw, double &yawRate, double b,  bool bUsed_NominalYawAttitude)
	{
		map<int, GYM95Sat>::iterator it = m_mapGYM95Info.find(id_Sat);
		if(it == m_mapGYM95Info.end())
		{
			printf("PRN = %d 无姿态信息\n",id_Sat);
			return false;
		}
		POS3D sunPos;
		TDB t_TDB = TimeCoordConvert::GPST2TDB(t); // 获得 TDB 时间--提供太阳历参考时间									
		double jd = TimeCoordConvert::DayTime2JD(t_TDB); // 获得儒略日									
		double Pos[3];
		if(!m_JPLEphFile.getSunPos_Delay_EarthCenter(jd, Pos))
		{
			printf("无太阳位置信息\n");
			return false;
		}
		sunPos.x = Pos[0] * 1000; 
		sunPos.y = Pos[1] * 1000; 
		sunPos.z = Pos[2] * 1000; 
		POS6D    gpsPosVel;
		SP3Datum sp3Datum;
		if(!m_sp3File.getEphemeris(t,id_Sat,sp3Datum))
		{
			//printf("%02d卫星星历获取失败!\n",id_Sat);
			return false;
		}
		gpsPosVel.setPos(sp3Datum.pos);
		gpsPosVel.setVel(sp3Datum.vel);									
		double beta;
		nominalYawAttitude(it->second.id_Block, sunPos, gpsPosVel,yaw,yawRate,beta,b);

		if(bUsed_NominalYawAttitude)
			return true;
		
		int sign_b = 1;
		if(b < 0)
			sign_b = -1;

		//step1:位于noonTurn,或midnightTurn
		if(it->second.yawNoonTurnList.size() > 0)
		{
			for(size_t s_i = 0; s_i < it->second.yawNoonTurnList.size(); s_i ++)
			{
				GYM95NoonTurnDatum  noonTurn = it->second.yawNoonTurnList[s_i];
				if(noonTurn.ti - t <= 0 && noonTurn.te - t >= 0)
				{// 计算:卫星-地球-太阳张角SES
					POS3D unit_sun    = vectorNormal(sunPos);
					POS3D unit_gpspos = vectorNormal(gpsPosVel.getPos());
					double SES = acos(vectorDot(unit_sun, unit_gpspos));					
					int sign_beta = 1;
					if(beta < 0)
						sign_beta = -1;	
					if(SES > PI/2) //block IIR
						sign_beta = - sign_beta;
					yaw = noonTurn.yaw_ti - sign_beta * fabs(it->second.max_yawRate) * (t - noonTurn.ti);
					yawRate = noonTurn.yawRate_ti;
					return true;
				}
			}
		}
		//step2:位于ShadowCross
		if(it->second.yawShadowCrossList.size() > 0)
		{
			for(size_t s_i = 0; s_i < it->second.yawShadowCrossList.size(); s_i ++)
			{
				GYM95ShadowCrossDatum    shadowCross = it->second.yawShadowCrossList[s_i];
				if(shadowCross.ti - t <= 0 && shadowCross.te - t >= 0)
				{
					if(t - shadowCross.t1 < 0)
					{
						yaw = shadowCross.yaw_ti + shadowCross.yawRate_ti * (t - shadowCross.ti) + 0.5 * sign_b * it->second.max_yawRateRate * (t - shadowCross.ti) * (t - shadowCross.ti);
						yawRate = shadowCross.yawRate_ti + sign_b * it->second.max_yawRateRate * (t - shadowCross.ti);
						return true;
					}
					else
					{
						double    t1 = shadowCross.t1 - shadowCross.ti;						
						yaw  = shadowCross.yaw_ti + shadowCross.yawRate_ti * t1 + 0.5 * sign_b * it->second.max_yawRateRate * t1 * t1
							   + sign_b * it->second.max_yawRate * (t - shadowCross.t1);// 
						//将yaw_te,转化到[-180,180];
						if(yaw < 0)
							yaw = yaw  + 720;//转为正角度
						int n = int(floor(yaw))/180;
						double res = yaw - n * 180;
						if(n % 2 == 0)
							yaw = res;
						else
							yaw = res - 180;
						yawRate = it->second.max_yawRate;
						return true;
					}
					
				}
			}
		}
		//step3:位于ShadowPost
		if(it->second.yawShadowPostList.size() > 0)
		{
			for(size_t s_i = 0; s_i < it->second.yawShadowPostList.size(); s_i ++)
			{
				GYM95ShadowPostDatum    shadowPost = it->second.yawShadowPostList[s_i];
				int sign_D = 1;
				double D = yaw - it->second.yawShadowCrossList[s_i].yaw_te - floor(( yaw - it->second.yawShadowCrossList[s_i].yaw_te)/360 + 0.5) * 360;
				if(D < 0)
					sign_D = -1;
				if(shadowPost.ti - t <= 0 && shadowPost.te - t >= 0)
				{
					if(t - shadowPost.t1 < 0)
					{						
						yaw = shadowPost.yaw_ti + sign_b * it->second.max_yawRate * (t- shadowPost.ti) + 0.5 * sign_D * it->second.max_yawRateRate * pow((t - shadowPost.ti),2);		
						yawRate = sign_b * it->second.max_yawRate + sign_D * it->second.max_yawRateRate;
						return true;
					}
					else
					{ 
						yaw = shadowPost.yaw_ti + sign_b * it->second.max_yawRate * (shadowPost.t1 - shadowPost.ti) + 0.5 * sign_D * it->second.max_yawRateRate * pow((shadowPost.t1 - shadowPost.ti),2)
						      + sign_D * it->second.max_yawRate * (t - shadowPost.t1);
						yawRate = sign_D * it->second.max_yawRate;
						return true;
					}
				}
			}
		}
		//step4:其余情况，为 nominal yaw
		return true;
	}
	// 子程序名称： yaw2unitXYZ   
	// 功能：通过yaw姿态获取星固系的三轴单位矢量
	// 变量类型：gpsPVT              : 时间(GPST)位置速度	
	//           acsYaw              : 姿态控制模式下的偏航姿态角, 单位：度
	//           ex                  : 星固系X轴单位矢量
	//           ey                  : 星固系Y轴单位矢量
	//           ez                  : 星固系Z轴单位矢量
	//           bECEF               : 位置速度所使用的坐标系统，true为地固系，false 为惯性系
	// 输入：gpsPVT, acsYaw,bECEF
	// 输出：ex, ey,ez
	// 语言：C++
	// 创建者：谷德峰, 刘俊宏
	// 创建时间：2014/11/20
	// 版本时间：
	// 修改记录：
	// 备注： 
	void GPSYawAttitudeModel1995::yaw2unitXYZ(TimePosVel gpsPVT, double acsYaw, POS3D &ex, POS3D &ey, POS3D &ez, bool bECEF)
	{
		//acsYaw = -acsYaw; 纠正, 谷德峰, 20160408	
		POS3D eR,eT,eN;// 轨道系的三轴单位矢量
		if(!bECEF)
		{//对于惯性系，直接差乘获得轨道系
			eR =  vectorNormal(gpsPVT.pos) * (-1.0);// 卫星轨道系R方向
			POS3D eV =  vectorNormal(gpsPVT.vel);// 速度方向			                                             
			vectorCross(eN, eR, eV);
			eN = vectorNormal(eN);// 卫星轨道系N方向			                                          
			vectorCross(eT, eN, eR);	
			eT = vectorNormal(eT);// 卫星轨道系T方向
			
			ez = eR; // Z轴单位矢量, 卫星指向地心， 纠正 ez = eR * (-1.0), 谷德峰, 20160408		                                                            
			acsYaw = acsYaw * PI / 180; // 转换为弧度
			ex = eT * cos(acsYaw) + eN * sin(acsYaw); // ex = cos(yaw) * eT + sin(yaw) * eN方向	
			ex = vectorNormal(ex);
			vectorCross(ey, ez, ex);
			ey = vectorNormal(ey); // Y轴单位矢量
		}
		else
		{//对于地固系，需要借助时间获取轨道系
			UT1 t = m_TimeCoordConvert.GPST2UT1(gpsPVT.t);		
			m_TimeCoordConvert.getCoordinateRTNAxisVector(t, gpsPVT.getPosVel(), eR, eT, eN);
			ez = eR * (-1.0); // Z轴单位矢量, 卫星指向地心	
			eN = eN * (-1.0); 
			acsYaw = acsYaw * PI / 180; // 转换为弧度
			ex = eT * cos(acsYaw) + eN * sin(acsYaw); // ex = cos(yaw) * eT + sin(yaw) * eN方向	
			ex = vectorNormal(ex);
			vectorCross(ey, ez, ex);
			ey = vectorNormal(ey); // Y轴单位矢量
		}		 
		
	}

	// 子程序名称： get_BeiDouYawAttitudeMode   
	// 功能：获取 BDS 卫星偏航姿态 yaw-fixed: false; yaw-steering: true
	// 变量类型：inputSp3FilePath : 轨道
	// 输入：inputSp3FilePath
	// 输出：m_mapBeiDouYawMode
	// 语言：C++
	// 创建者：鞠 冰
	// 创建时间：2016/3/2
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool GPSYawAttitudeModel1995::get_BeiDouYawAttitudeMode()
	{
		m_mapBeiDouYawMode.clear();
		if(m_sp3File.m_data.empty())
		{
			printf("GPSYawAttitudeModel1995 缺少Sp3星历!\n");
			return false;
		}
		for(int i = 0; i < int(m_sp3File.m_header.pstrSatNameList.size()); i++) 
		{
			if(m_sp3File.m_header.pstrSatNameList[i][0] != 'C')
				continue;
			int nPRN = m_sp3File.getSatPRN(m_sp3File.m_header.pstrSatNameList[i]);
			if(nPRN <= 5)	// GEO卫星
			{
				// 每颗卫星的偏航姿态模式, 2016/3/1, 鞠 冰
				m_mapBeiDouYawMode.insert(map<int, bool>::value_type(nPRN, false));
			}
			else			// MEO/IGSO卫星
			{
				GPST t0 = m_sp3File.m_data.front().t;
				GPST t1 = m_sp3File.m_data.back().t;
				// 先计算beta角，再根据beta角判断IGSO和MEO卫星采用什么光压模型
				// 1、获得t0, t1时刻卫星在 J2000 系下的位置
				double x_ecf[6];
				double x_j2000[6];
				SP3Datum sp3Datum;
				m_sp3File.getEphemeris(t0, nPRN, sp3Datum, 9, 'C');
				x_ecf[0] = sp3Datum.pos.x;  
				x_ecf[1] = sp3Datum.pos.y;  
				x_ecf[2] = sp3Datum.pos.z;
				x_ecf[3] = sp3Datum.vel.x; 
				x_ecf[4] = sp3Datum.vel.y; 
				x_ecf[5] = sp3Datum.vel.z;
				m_TimeCoordConvert.ECEF_J2000(t0, x_j2000, x_ecf);
				POS3D satPos_t0, satVel_t0, satPos_t1, satVel_t1;
				satPos_t0.x = x_j2000[0];
				satPos_t0.y = x_j2000[1];
				satPos_t0.z = x_j2000[2];
				satVel_t0.x = x_j2000[3];
				satVel_t0.y = x_j2000[4];
				satVel_t0.z = x_j2000[5];
				m_sp3File.getEphemeris(t1, nPRN, sp3Datum, 9, 'C');
				x_ecf[0] = sp3Datum.pos.x;  
				x_ecf[1] = sp3Datum.pos.y;  
				x_ecf[2] = sp3Datum.pos.z;
				x_ecf[3] = sp3Datum.vel.x; 
				x_ecf[4] = sp3Datum.vel.y; 
				x_ecf[5] = sp3Datum.vel.z;
				m_TimeCoordConvert.ECEF_J2000(t1, x_j2000, x_ecf);
				satPos_t1.x = x_j2000[0];
				satPos_t1.y = x_j2000[1];
				satPos_t1.z = x_j2000[2];
				satVel_t1.x = x_j2000[3];
				satVel_t1.y = x_j2000[4];
				satVel_t1.z = x_j2000[5];
				// 2、获得t0, t1时刻太阳在 J2000 系下的位置
				TDB  t0_TDB = TimeCoordConvert::GPST2TDB(t0); // 获得 TDB 时间--提供太阳历参考时间	
				TDB  t1_TDB = TimeCoordConvert::GPST2TDB(t1); // 获得 TDB 时间--提供太阳历参考时间
				double jd_TDB_t0 = m_TimeCoordConvert.DayTime2JD(t0_TDB); // 获得儒略日
				double jd_TDB_t1 = m_TimeCoordConvert.DayTime2JD(t1_TDB); // 获得儒略日
				POS3D sunPos_J2000_t0,sunPos_J2000_t1;
				double P_J2000_t0[3],P_J2000_t1[3];
				if(!m_JPLEphFile.getSunPos_Delay_EarthCenter(jd_TDB_t0, P_J2000_t0)||!m_JPLEphFile.getSunPos_Delay_EarthCenter(jd_TDB_t1, P_J2000_t1))
				{
					printf("GPSYawAttitudeModel1995 获得太阳位置失败!\n");
					return false;
				}
				for(int i = 0; i < 3; i ++)
				{
					P_J2000_t0[i] = P_J2000_t0[i] * 1000; // 换算成米
					P_J2000_t1[i] = P_J2000_t1[i] * 1000; // 换算成米
				}							
				sunPos_J2000_t0.x = P_J2000_t0[0];
				sunPos_J2000_t0.y = P_J2000_t0[1];
				sunPos_J2000_t0.z = P_J2000_t0[2];
				sunPos_J2000_t1.x = P_J2000_t1[0];
				sunPos_J2000_t1.y = P_J2000_t1[1];
				sunPos_J2000_t1.z = P_J2000_t1[2];
				// 3. 计算t0时刻太阳入射方向与轨道面夹角β1
				POS3D unit_n;
				vectorCross(unit_n, satPos_t0, satVel_t0);
				unit_n = vectorNormal(unit_n);
				POS3D unit_sun = vectorNormal(sunPos_J2000_t0);
				double beta_t0 =  PI / 2 - acos(vectorDot(unit_n, unit_sun));
				// 4. 计算t1时刻太阳入射方向与轨道面夹角β2
				vectorCross(unit_n, satPos_t1, satVel_t1);
				unit_n = vectorNormal(unit_n);
				unit_sun = vectorNormal(sunPos_J2000_t1);				
				double beta_t1 =  PI / 2 - acos(vectorDot(unit_n, unit_sun));
				// 每颗卫星的偏航姿态模式, 2016/3/1, 鞠 冰
				if(max(fabs(beta_t0),fabs(beta_t1)) * 180 / PI < 4)		// 默认4，关闭MEO,IGSO姿态切换,2016-09-24,supice
				{
					//m_mapBeiDouYawMode.find(nPRN)->second = false;
					m_mapBeiDouYawMode.insert(map<int, bool>::value_type(nPRN, false));
				}
				else
					m_mapBeiDouYawMode.insert(map<int, bool>::value_type(nPRN, true));
			}
		}
		if(m_mapBeiDouYawMode.size() == 0)
		{
			printf("获取 BeiDou 卫星偏航姿态模式失败!\n");
			return false;
		}
		return true;
	}
}
