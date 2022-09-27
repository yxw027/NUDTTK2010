#include "GNSSYawAttitudeModel.hpp"

namespace NUDTTK
{
	const char BLOCK_MaskString::BLOCK_I[]          = "BLOCK I             ";
	const char BLOCK_MaskString::BLOCK_II[]         = "BLOCK II            ";
	const char BLOCK_MaskString::BLOCK_IIA[]        = "BLOCK IIA           ";
	const char BLOCK_MaskString::BLOCK_IIR_A[]      = "BLOCK IIR-A         ";
	const char BLOCK_MaskString::BLOCK_IIR_B[]      = "BLOCK IIR-B         ";
	const char BLOCK_MaskString::BLOCK_IIR_M[]      = "BLOCK IIR-M         ";
	const char BLOCK_MaskString::BLOCK_IIF[]        = "BLOCK IIF           ";
	const char BLOCK_MaskString::BLOCK_III_A[]      = "BLOCK IIIA          "; // GPS 三代
	const char BLOCK_MaskString::BEIDOU_2G[]        = "BEIDOU-2G           ";
	const char BLOCK_MaskString::BEIDOU_2I[]        = "BEIDOU-2I           ";
	const char BLOCK_MaskString::BEIDOU_2M[]        = "BEIDOU-2M           ";
	const char BLOCK_MaskString::BEIDOU_3M[]        = "BEIDOU-3M           ";
    const char BLOCK_MaskString::BEIDOU_3M_CAST[]   = "BEIDOU-3M-CAST      ";
	const char BLOCK_MaskString::BEIDOU_3M_SECM_A[] = "BEIDOU-3M-SECM-A    ";
	const char BLOCK_MaskString::BEIDOU_3M_SECM_B[] = "BEIDOU-3M-SECM-B    ";
	const char BLOCK_MaskString::BEIDOU_3I[]        = "BEIDOU-3I           ";
	const char BLOCK_MaskString::BEIDOU_3G[]        = "BEIDOU-3G           ";
	const char BLOCK_MaskString::BEIDOU_3G_CAST[]   = "BEIDOU-3G-CAST      ";
	const char BLOCK_MaskString::BEIDOU_3SM_CAST[]  = "BEIDOU-3SM-CAST     ";
	const char BLOCK_MaskString::BEIDOU_3SM_SECM[]  = "BEIDOU-3SM-SECM     ";
	const char BLOCK_MaskString::BEIDOU_3SI_CAST[]  = "BEIDOU-3SI-CAST     ";
	const char BLOCK_MaskString::BEIDOU_3SI_SECM[]  = "BEIDOU-3SI-SECM     ";
	const char BLOCK_MaskString::GLONASS[]          = "GLONASS             ";
	const char BLOCK_MaskString::GLONASS_M[]        = "GLONASS-M           ";
	const char BLOCK_MaskString::GLONASS_K1[]       = "GLONASS-K1          ";
	const char BLOCK_MaskString::GALILEO_1[]        = "GALILEO-1           ";
	const char BLOCK_MaskString::GALILEO_2[]        = "GALILEO-2           ";
	const char BLOCK_MaskString::GALILEO_0A[]       = "GALILEO-0A          ";
	const char BLOCK_MaskString::GALILEO_0B[]       = "GALILEO-0B          ";

	GNSSYawAttitudeModel::GNSSYawAttitudeModel(void)
	{
		m_bOnGYMInfo = false;
	}

	GNSSYawAttitudeModel::~GNSSYawAttitudeModel(void)
	{
	}

	// 子程序名称： getBetaSunAngle   
	// 功能：太阳入射方向与轨道面夹角β,单位：度
	// 变量类型：sunPos              : 太阳的位置, 惯性系, 单位：米
	//           gnssPosVel          : gnss卫星的轨道位置、速度, 惯性系, 单位：米
	//           beta                : 太阳入射方向与轨道面夹角β,单位：度
	// 输入：sunPos, gpsPos
	// 输出：beta
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2017/9/6
	// 版本时间：
	// 修改记录：
	// 备注：
	double GNSSYawAttitudeModel::getBetaSunAngle(POS3D sunPos, POS6D gnssPosVel)
	{
		// 1. 计算轨道法向矢量 n
		POS3D unit_sun     = vectorNormal(sunPos);
		POS3D unit_gnsspos = vectorNormal(gnssPosVel.getPos());
		POS3D unit_gnssvel = vectorNormal(gnssPosVel.getVel());
		POS3D unit_n;
		vectorCross(unit_n, unit_gnsspos, unit_gnssvel);
		unit_n = vectorNormal(unit_n);
		// 2. 计算太阳入射方向与轨道面夹角β
		double beta =  PI / 2 - acos(vectorDot(unit_n, unit_sun));
		return beta * 180.0 / PI;
	}
	// 子程序名称： getUOrbitAngle   
	// 功能：卫星到地心与“远日点”到地心连线的夹角u,单位：度
	// 变量类型：sunPos              : 太阳的位置, 惯性系, 单位：米
	//           gnssPosVel          : gnss卫星的轨道位置、速度, 惯性系, 单位：米
	//           u                   : 夹角, 单位：度
	// 输入：sunPos, gpsPos
	// 输出：u
	// 语言：C++
	// 创建者：邵凯
	// 创建时间：2019/8/31
	// 版本时间：
	// 修改记录：
	// 备注：
	double GNSSYawAttitudeModel::getUOrbitAngle(POS3D sunPos, POS6D gpsPosVel)
	{
		// 1. 计算轨道法向矢量 n
		POS3D unit_sun    = vectorNormal(sunPos);
		POS3D unit_gpspos = vectorNormal(gpsPosVel.getPos());
		POS3D unit_gpsvel = vectorNormal(gpsPosVel.getVel());
		POS3D unit_n;
		vectorCross(unit_n, unit_gpspos, unit_gpsvel);
		unit_n = vectorNormal(unit_n);
		// 2. 计算GPS卫星轨道面内的yaw起始点: 午夜矢量轴; 计算轨道面内角μ
		POS3D unit_y;
		POS3D unit_midnight;
		vectorCross(unit_y, unit_n, unit_sun * (-1.0));
		unit_y = vectorNormal(unit_y);
		vectorCross(unit_midnight, unit_y, unit_n);
		unit_midnight = vectorNormal(unit_midnight);
		double u = atan2(vectorDot(unit_gpspos, unit_y), vectorDot(unit_gpspos, unit_midnight));
		return u * 180.0 / PI;
	}
	// 子程序名称： getUOrbitAngle_perihelion   
	// 功能：卫星到地心与“近日点”到地心连线的夹角u,单位：度
	// 变量类型：sunPos              : 太阳的位置, 惯性系, 单位：米
	//           gnssPosVel          : gnss卫星的轨道位置、速度, 惯性系, 单位：米
	//           u                   : 夹角, 单位：度
	// 输入：sunPos, gpsPos
	// 输出：u
	// 语言：C++
	// 创建者：邵凯
	// 创建时间：2019/8/31
	// 版本时间：
	// 修改记录：
	// 备注：
	double GNSSYawAttitudeModel::getUOrbitAngle_perihelion(POS3D sunPos, POS6D gpsPosVel)
	{
		// 1. 计算轨道法向矢量 n
		POS3D unit_sun    = vectorNormal(sunPos);
		POS3D unit_gpspos = vectorNormal(gpsPosVel.getPos());
		POS3D unit_gpsvel = vectorNormal(gpsPosVel.getVel());
		POS3D unit_n;
		vectorCross(unit_n, unit_gpspos, unit_gpsvel);
		unit_n = vectorNormal(unit_n);
		// 2. 计算noon矢量轴; 计算轨道面内角μ
		POS3D unit_y;
		POS3D unit_noon;
		vectorCross(unit_y, unit_n, unit_sun);
		unit_y = vectorNormal(unit_y);
		vectorCross(unit_noon, unit_y, unit_n);
		unit_noon = vectorNormal(unit_noon);
		double u = atan2(vectorDot(unit_gpspos, unit_y), vectorDot(unit_gpspos, unit_noon));
		return u * 180.0 / PI;
	}
	// 子程序名称： nominalYawAttitude_GPS   
	// 功能：计算标称偏航姿态角及其变化率[GPS]
	// 变量类型：nameBlock           : 卫星的 block 名称
    //           sunPos              : 太阳的位置, 惯性系, 单位：米
	//           gpsPosVel           : gps卫星的轨道位置、速度, 惯性系, 单位：米
	//           yaw                 : 偏航姿态角, 单位：度
	//           yawRate             : 偏航姿态角变化率
	//           betaSun             : 太阳入射方向与轨道面夹角β,单位：度
	//           b                   : 卫星控制系统的偏航姿态角偏差, starting November 1995 the bias is set to +0.5 for all satellites, 1996 Bar Sever
	// 输入：sunPos, gpsPos
	// 输出：nameBlock, yaw, yawRate, beta
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2014/11/2
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool GNSSYawAttitudeModel::nominalYawAttitude_GPS(string nameBlock, POS3D sunPos, POS6D gpsPosVel, double& yaw, double &yawRate, double &betaSun, double b)
	{
		// 1. 计算轨道法向矢量 n
		POS3D unit_sun    = vectorNormal(sunPos);
		POS3D unit_gpspos = vectorNormal(gpsPosVel.getPos());
		POS3D unit_gpsvel = vectorNormal(gpsPosVel.getVel());
		POS3D unit_n;
		vectorCross(unit_n, unit_gpspos, unit_gpsvel);
		unit_n = vectorNormal(unit_n);
		// 2. 计算太阳入射方向与轨道面夹角β
		double beta =  PI / 2 - acos(vectorDot(unit_n, unit_sun));
		betaSun = beta * 180.0 / PI;
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
		// id_Block = block IIR
		if(nameBlock.find(BLOCK_MaskString::BLOCK_IIR_A) != -1
		|| nameBlock.find(BLOCK_MaskString::BLOCK_IIR_B) != -1
		|| nameBlock.find(BLOCK_MaskString::BLOCK_IIR_M) != -1)
			yaw = atan2(tan(beta), -sinu);	// 
		else
			yaw = atan2(-tan(beta), sinu);

		double uRate = 0.00836 * PI / 180; // uRate = 0.00836 度/s 2009,JoG
		yawRate = uRate * tan(beta) * cosu / (sinu * sinu + tan(beta) * tan(beta));
		yaw = yaw * 180.0 / PI;		
		yawRate = yawRate * 180.0 / PI;

		// 5. 计算由于 Yaw Bias 引入的偏航角部分 B 
		if(b != 0.0)
		{
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
		}
		return true;
	}
	// 子程序名称： nominalYawAttitude_GALILEO   
	// 功能：计算标称偏航姿态角及其变化率[GALILEO]
	// 变量类型：nameBlock           : 卫星的 block 名称
    //           sunPos              : 太阳的位置, 惯性系, 单位：米
	//           gpsPosVel           : gps卫星的轨道位置、速度, 惯性系, 单位：米
	//           yaw                 : 偏航姿态角, 单位：度
	//           yawRate             : 偏航姿态角变化率
	//           betaSun             : 太阳入射方向与轨道面夹角β,单位：度
	// 输入：sunPos, gpsPos
	// 输出：nameBlock, yaw, yawRate, beta, betaSun
	// 语言：C++
	// 创建者：邵凯
	// 创建时间：2019/9/9
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool GNSSYawAttitudeModel::nominalYawAttitude_GALILEO(string nameBlock, POS3D sunPos, POS6D gpsPosVel, double& yaw, double &yawRate, double &betaSun)
	{
		// 1. 计算轨道法向矢量 n
		POS3D unit_sun    = vectorNormal(sunPos);
		POS3D unit_gpspos = vectorNormal(gpsPosVel.getPos());
		POS3D unit_gpsvel = vectorNormal(gpsPosVel.getVel());
		POS3D unit_n;
		vectorCross(unit_n, unit_gpspos, unit_gpsvel);
		unit_n = vectorNormal(unit_n);
		// 2. 计算太阳入射方向与轨道面夹角β
		double beta =  PI / 2 - acos(vectorDot(unit_n, unit_sun));
		betaSun = beta * 180.0 / PI;
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
		// GALILEO卫星虽然采用与GPS卫星不同的姿态控制模式，但名义偏航角计算方式与GPS计算方式等价
		yaw = atan2(-tan(beta), sinu);
		double uRate = 0.00836 * PI / 180; // uRate = 0.00836 度/s 2009,JoG
		yawRate = uRate * tan(beta) * cosu / (sinu * sinu + tan(beta) * tan(beta));
		yaw = yaw * 180.0 / PI;		
		yawRate = yawRate * 180.0 / PI;
		return true;
	}

	// 子程序名称： nominalYawAttitude_GLONASS   
	// 功能：计算标称偏航姿态角及其变化率[GLONASS]
	// 变量类型：nameBlock           : 卫星的 block 名称
    //           sunPos              : 太阳的位置, 惯性系, 单位：米
	//           gpsPosVel           : gps卫星的轨道位置、速度, 惯性系, 单位：米
	//           yaw                 : 偏航姿态角, 单位：度
	//           yawRate             : 偏航姿态角变化率
	//           betaSun             : 太阳入射方向与轨道面夹角β,单位：度
	// 输入：sunPos, gpsPos
	// 输出：nameBlock, yaw, yawRate, beta, betaSun
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2014/11/2
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool GNSSYawAttitudeModel::nominalYawAttitude_GLONASS(string nameBlock, POS3D sunPos, POS6D gpsPosVel, double& yaw, double &yawRate, double &betaSun)
	{
		// 1. 计算轨道法向矢量 n
		POS3D unit_sun    = vectorNormal(sunPos);
		POS3D unit_gpspos = vectorNormal(gpsPosVel.getPos());
		POS3D unit_gpsvel = vectorNormal(gpsPosVel.getVel());
		POS3D unit_n;
		vectorCross(unit_n, unit_gpspos, unit_gpsvel);
		unit_n = vectorNormal(unit_n);
		// 2. 计算太阳入射方向与轨道面夹角β
		double beta =  PI / 2 - acos(vectorDot(unit_n, unit_sun));
		betaSun = beta * 180.0 / PI;
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
		// id_Block  = 4、5、6, block IIR
		if(nameBlock.find(BLOCK_MaskString::BLOCK_IIR_A) != -1
		|| nameBlock.find(BLOCK_MaskString::BLOCK_IIR_B) != -1
		|| nameBlock.find(BLOCK_MaskString::BLOCK_IIR_M) != -1)
			yaw = atan2(tan(beta), -sinu);	// 
		else
			yaw = atan2(-tan(beta), sinu);

		double uRate = 0.00836 * PI / 180; // uRate = 0.00836 度/s 2009,JoG
		yawRate = uRate * tan(beta) * cosu / (sinu * sinu + tan(beta) * tan(beta));
		yaw = yaw * 180.0 / PI;		
		yawRate = yawRate * 180.0 / PI;

		return true;
	}
	// 子程序名称： nominalYawAttitude_BDS   
	// 功能：计算标称偏航姿态角及其变化率[BDS]
	// 变量类型：nameBlock           : 卫星的 block 名称
    //           sunPos              : 太阳的位置, 惯性系, 单位：米
	//           gpsPosVel           : gps卫星的轨道位置、速度, 惯性系, 单位：米
	//           yaw                 : 偏航姿态角, 单位：度
	//           yawRate             : 偏航姿态角变化率
	//           betaSun             : 太阳入射方向与轨道面夹角β,单位：度
	// 输入：sunPos, gpsPos
	// 输出：nameBlock, yaw, yawRate, beta
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2014/11/2
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool GNSSYawAttitudeModel::nominalYawAttitude_BDS(string nameBlock, POS3D sunPos, POS6D gpsPosVel, double &yaw, double &yawRate, double &betaSun)
	{
		// 1. 计算轨道法向矢量 n
		POS3D unit_sun    = vectorNormal(sunPos);
		POS3D unit_gpspos = vectorNormal(gpsPosVel.getPos());
		POS3D unit_gpsvel = vectorNormal(gpsPosVel.getVel());
		POS3D unit_n;
		vectorCross(unit_n, unit_gpspos, unit_gpsvel);
		unit_n = vectorNormal(unit_n);
		// 2. 计算太阳入射方向与轨道面夹角β
		double beta =  PI / 2 - acos(vectorDot(unit_n, unit_sun));
		betaSun = beta * 180.0 / PI;

		if(nameBlock.find(BLOCK_MaskString::BEIDOU_2G) != -1 
			|| nameBlock.find(BLOCK_MaskString::BEIDOU_3G) != -1
			|| nameBlock.find(BLOCK_MaskString::BEIDOU_3G_CAST) != -1)
		{
			yaw = 0.0;
			yawRate = 0.0;
			return true;
		}
		// 3. 计算GPS卫星轨道面内的yaw起始点: 午夜矢量轴; 计算轨道面内角μ
		POS3D unit_y;
		POS3D unit_midnight;
		vectorCross(unit_y, unit_n, unit_sun * (-1.0));
		unit_y = vectorNormal(unit_y);
		vectorCross(unit_midnight, unit_y, unit_n);
		unit_midnight = vectorNormal(unit_midnight);
		double sinu = vectorDot(unit_gpspos, unit_y);
		double cosu = vectorDot(unit_gpspos, unit_midnight);
		
		// 4. 计算偏航角 yaw 和 yawRate		
		yaw = atan2(-tan(beta), sinu);
		//double uRate = 0.0083 * PI / 180;//uRate = |v_sat|/|r_sat| 2019 Xia. observation of BDS-2 IGSO/MEOs yaw-atitude behavior during eclipse seasons
		double v_sat_norm2 = sqrt(vectorDot(gpsPosVel.getVel(), gpsPosVel.getVel()));
		double r_sat_norm2 = sqrt(vectorDot(gpsPosVel.getPos(), gpsPosVel.getPos()));
		double uRate = v_sat_norm2 / r_sat_norm2;//v=ωR，使用这一公式时应注意，角度的单位一定要用弧度，只有角速度的单位是弧度/秒时，上述公式才成立

		yawRate = uRate * tan(beta) * cosu / (sinu * sinu + tan(beta) * tan(beta));
		yaw = yaw * 180.0 / PI;		
		yawRate = yawRate * 180.0 / PI;
		return true;
	}
		
	// 子程序名称： nominalYawAttitude_BDS_bias   
	// 功能：计算noonturn和midnightturn期间C13和C14标称偏航姿态角及其变化率[BDS]
	// 变量类型：nameBlock           : 卫星的 block 名称
    //           sunPos              : 太阳的位置, 惯性系, 单位：米
	//           gpsPosVel           : gps卫星的轨道位置、速度, 惯性系, 单位：米
	//           yaw                 : 偏航姿态角, 单位：度
	//           yawRate             : 偏航姿态角变化率
	//           betaSun             : 太阳入射方向与轨道面夹角β,单位：度
	// 输入：sunPos, gpsPos
	// 输出：nameBlock, yaw, yawRate, beta
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2014/11/2
	// 版本时间：
	// 修改记录：添加针对北斗二号C13和C14卫星的低太阳倾角的名义姿态bias偏差，张厚喆
	// 备注：	
	bool GNSSYawAttitudeModel::nominalYawAttitude_BDS_bias(string nameBlock, POS3D sunPos, POS6D gpsPosVel, double &yaw, double &yawRate, double &betaSun)
	{
		// 1. 计算轨道法向矢量 n
		POS3D unit_sun    = vectorNormal(sunPos);
		POS3D unit_gpspos = vectorNormal(gpsPosVel.getPos());
		POS3D unit_gpsvel = vectorNormal(gpsPosVel.getVel());
		POS3D unit_n;
		vectorCross(unit_n, unit_gpspos, unit_gpsvel);
		unit_n = vectorNormal(unit_n);
		// 2. 计算太阳入射方向与轨道面夹角β
		double beta =  PI / 2 - acos(vectorDot(unit_n, unit_sun));
		betaSun = beta * 180.0 / PI;
		// 3. 计算GPS卫星轨道面内的yaw起始点: 午夜矢量轴; 计算轨道面内角μ
		POS3D unit_y;
		POS3D unit_midnight;
		vectorCross(unit_y, unit_n, unit_sun * (-1.0));
		unit_y = vectorNormal(unit_y);
		vectorCross(unit_midnight, unit_y, unit_n);
		unit_midnight = vectorNormal(unit_midnight);
		double sinu = vectorDot(unit_gpspos, unit_y);
		double cosu = vectorDot(unit_gpspos, unit_midnight);
		double Rbias = 0.0;
		if(cosu > 0)//midnight-turn maneuver
			Rbias = 0.14 * PI / 180.0;
		else if(cosu < 0)//noon-turn maneuver
			Rbias = -0.14 * PI / 180.0;
		// 4. 计算偏航角 yaw 和 yawRate		
		yaw = atan2(tan(-beta + Rbias), sinu);
		//double uRate = 0.0083 * PI / 180;//uRate = |v_sat|/|r_sat| 2019 Xia. observation of BDS-2 IGSO/MEOs yaw-atitude behavior during eclipse seasons
		double v_sat_norm2 = sqrt(vectorDot(gpsPosVel.getVel(), gpsPosVel.getVel()));
		double r_sat_norm2 = sqrt(vectorDot(gpsPosVel.getPos(), gpsPosVel.getPos()));
		double uRate = v_sat_norm2 / r_sat_norm2;//v=ωR，使用这一公式时应注意，角度的单位一定要用弧度，只有角速度的单位是弧度/秒时，上述公式才成立
		yawRate = uRate * tan(beta) * cosu / (sinu * sinu + tan(beta) * tan(beta));
		yaw = yaw * 180.0 / PI;		
		yawRate = yawRate * 180.0 / PI;
		return true;
	}
	// 子程序名称： init   
	// 功能：计算每颗卫星的GYM模型基本信息, 包括 Noon Turn、Shadow Cross、Shadow Post 的时间列表
	// 变量类型：span_t         : 计算的时间粒度, 默认 30s
	//           on_J2000       : 是否需要将sp3星历转为J2000惯性系，默认为 true
	// 输入：span_t, m_sp3File, m_JPLEphFile, m_svnavMixedFile
	// 输出：m_mapGYM95Info
	// 语言：C++
	// 创建者：谷德峰、刘俊宏
	// 创建时间：2014/11/5
	// 版本时间：
	// 修改记录：1、对GPS IIF卫星姿态模型进行调整，邵凯，2019/08/31
	// 备注： 
	bool GNSSYawAttitudeModel::init(double span_t, bool on_J2000)
	{
		char info[200];
		if(m_sp3File.m_data.size() == 0)
		{
			printf("警告：GYM星历sp3缺失.\n");
			return false;
		}
		if(m_svnavMixedFile.m_data.size() == 0)
		{
			printf("警告：GYM卫星类型svnav缺失.\n");
			return false;
		}
		if(on_J2000)
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
		//首先根据svnavFile获取卫星的BLOCK信息
		for(size_t s_i = 0; s_i < m_sp3File.m_header.pstrSatNameList.size(); s_i ++)
		{
            SvNavMixedLine mixedLine;
			if(!m_svnavMixedFile.getSvNavInfo(m_sp3File.m_data.front().t, m_sp3File.m_header.pstrSatNameList[s_i], mixedLine))
			{
				printf("警告：%s卫星Block信息缺失.\n", m_sp3File.m_header.pstrSatNameList[s_i].c_str());
				continue;
			}
			GYM_MixedSat gymSat(mixedLine);	
			gymSat.gpsNoonTurnList.clear();
			gymSat.gpsShadowCrossList.clear();
			gymSat.gpsShadowPostList.clear();
			gymSat.glonassNoonTurnList.clear();
			gymSat.glonassShadowCrossList.clear();
			gymSat.galileoNoonTurnList.clear();
			gymSat.galileoShadowCrossList.clear();
			gymSat.bdsYawNominalEntryList.clear();
			gymSat.bdsOrbNormalEntryList.clear();
			gymSat.bdsNoonTurnList.clear();
			if(gymSat.max_yawRate != 0.0) // 正常所有卫星都能找到 max_yawRate
				m_mapGYMInfo.insert(map<string, GYM_MixedSat>::value_type(m_sp3File.m_header.pstrSatNameList[s_i], gymSat));
			else
				printf("警告：%s卫星GYM模型未定义.\n", m_sp3File.m_header.pstrSatNameList[s_i].c_str());
		}

		//根据 m_mapGYMInfo 逐颗卫星搜索 gpsShadowCrossList, gpsNoonTurnList
		//FILE *pfile = fopen("F:\\orbit_beta.txt","w+");		
		for(map<string, GYM_MixedSat>::iterator it = m_mapGYMInfo.begin();it != m_mapGYMInfo.end();++it)
		{
			// 处理GPS卫星的机动问题, GPS II IIA IIR 采用GYM95模型, GPS IIF采用Dilssner模型
			if(it->first[0] == 'G')
			{
				GYM_G95_ShadowCrossDatum   shadowCross;
				GYM_G95_ShadowPostDatum    shadowPost;
				GYM_G95_NoonTurnDatum      noonTurn;
				GPST t0_sp3 = m_sp3File.m_data.front().t;
				GPST t1_sp3 = m_sp3File.m_data.back().t;
				double max_beta = 9 * PI/180;     // 可能进入noonTurn的最大beta角,适当扩大搜索范围
				if(it->second.nameBlock.find(BLOCK_MaskString::BLOCK_IIR_A) != -1
				|| it->second.nameBlock.find(BLOCK_MaskString::BLOCK_IIR_B) != -1
				|| it->second.nameBlock.find(BLOCK_MaskString::BLOCK_IIR_M) != -1)
				{
					max_beta = 7 * PI/180;
				}
				bool bOnNoonTurnIn     = false;	
				bool bOnMidnightTurnIn = false;
				bool bOnShadowIn       = false;		
				bool bOnLastPoint      = false; // 增加最后一点判断
				int sign_D = 1;                 // 用于shadowPost计算
				double yawPost_i_1,yawPost_i;   // 进入shadowPost后上一点的实际yaw和当前点的实际yaw
				int i = 0;
				bool on_BLOCK_IIR = true;   // BLOCK_IIR 卫星无地影机动
				bool on_BLOCK_IIF = false;  // BLOCK_IIF 卫星地影模型与 IIA 不同
				if(it->second.nameBlock.find(BLOCK_MaskString::BLOCK_IIR_A) == -1
				&& it->second.nameBlock.find(BLOCK_MaskString::BLOCK_IIR_B) == -1
				&& it->second.nameBlock.find(BLOCK_MaskString::BLOCK_IIR_M) == -1)
					on_BLOCK_IIR = false;
				if(it->second.nameBlock.find(BLOCK_MaskString::BLOCK_IIF) != -1 
				|| it->second.nameBlock.find(BLOCK_MaskString::BLOCK_III_A) != -1 )
					on_BLOCK_IIF = true;

				while(t0_sp3 + i * span_t - t1_sp3 <= 0)
				{
					GPST t_epoch = t0_sp3 + i * span_t;
					double hour = t_epoch.hour +  t_epoch.minute/60.0 + t_epoch.second/3600.0;	
					if(fabs(t_epoch - t1_sp3) < 1e-3)
					{//最后一点,前提是t1_sp3 - t0_sp3为span_t的整数倍
						bOnLastPoint = true;					
					}
					// 获取太阳位置
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
                    // 获取卫星位置
					POS6D    gpsPosVel;
					SP3Datum sp3Datum;
					if(!m_sp3File.getEphemeris(t_epoch,it->first,sp3Datum))
					{
						i++; // 2014/08/26, 谷德峰修改, 防止死循环
						continue;
					}
					gpsPosVel.setPos(sp3Datum.pos);
					gpsPosVel.setVel(sp3Datum.vel);	

					double b = 0.5; // 常量姿态偏差
					int sign_b = 1; // 常量姿态偏差正负标识
					double beta = getBetaSunAngle(sunPos, gpsPosVel); // 太阳入射方向与轨道面夹角β,单位：度
					if(it->second.yawBiasFlag == 'P') // + 0.5
					{
						b = 0.5;
						sign_b = 1;
					}
					if(it->second.yawBiasFlag == 'N') // - 0.5
					{
						b = -0.5;
						sign_b = -1;
					}
					if(it->second.yawBiasFlag == 'Y') // nominal
					{
						if(beta >= 0)
						{
							b = -0.5;
							sign_b = -1;
						}
						else
						{
							b = 0.5;
							sign_b = 1;
						}
					}
					if(it->second.yawBiasFlag == 'A') // anti-nominal
					{
						if(beta < 0)
						{
							b = -0.5;
							sign_b = -1;
						}
						else
						{
							b = 0.5;
							sign_b = 1;
						}
					}
					if(it->second.yawBiasFlag == 'U') // no bias
					{
						b = 0.0;
						sign_b = 1;
					}
					double yaw, yawRate;
					nominalYawAttitude_GPS(it->second.nameBlock, sunPos, gpsPosVel, yaw, yawRate, beta, b);	
					beta = beta * PI / 180.0;

					double shadowFactor = 1;
					// block IIR 卫星不考虑地影
					// block IIF 卫星考虑地影，根据进出地影的偏航角计算卫星所需姿态控制速率
					if(!bOnShadowIn && (!on_BLOCK_IIR && it->second.yawBiasFlag != 'U'))
					{//step1:判断是否进入地影
						if(GNSSBasicCorrectFunc::judgeGPSEarthShadowManeuver(sunPos,sp3Datum.pos,shadowFactor))
						{						
							shadowCross.ti         = t_epoch; // 进入地影时刻
							shadowCross.yaw_ti     = yaw;
							shadowCross.yawRate_ti = yawRate;
							shadowCross.t1         = t_epoch + ((sign_b * it->second.max_yawRate) - yawRate)/(sign_b * it->second.max_yawRateRate);
							bOnShadowIn            = true;						
							if(m_bOnGYMInfo)
							{
								sprintf(info, "%s %s  %c  %10.4lf %s(%12.4lf) 进入地影shadowCross.",it->first.c_str(),it->second.nameBlock.c_str(), it->second.yawBiasFlag, it->second.max_yawRate, t_epoch.toString().c_str(),hour);
								RuningInfoFile::Add(info);
							}
							continue;
						}
					}		
					if(bOnShadowIn && !on_BLOCK_IIF)
					{//step2:判断是否离开地影，并进入shawdowPost
						if(!GNSSBasicCorrectFunc::judgeGPSEarthShadowManeuver(sunPos,sp3Datum.pos,shadowFactor) || bOnLastPoint)
						{
							double    t1       = shadowCross.t1 - shadowCross.ti;
							shadowCross.te     = t_epoch;
							shadowCross.yaw_te = shadowCross.yaw_ti + shadowCross.yawRate_ti * t1 + 0.5 * sign_b * it->second.max_yawRateRate * t1 * t1
												 + sign_b * it->second.max_yawRate * (t_epoch - shadowCross.t1);
							//将yaw_te,转化到[-180,180];
							if(shadowCross.yaw_te < 0)
								shadowCross.yaw_te = shadowCross.yaw_te + 720;//转为正角度
							int n = int(floor(shadowCross.yaw_te))/180;
							double res = shadowCross.yaw_te - n * 180;
							if(n % 2 == 0)
								shadowCross.yaw_te = res;
							else
								shadowCross.yaw_te = res - 180;
							shadowCross.yawRate_te = it->second.max_yawRate;
							bOnShadowIn  = false;						
							it->second.gpsShadowCrossList.push_back(shadowCross);
							if(m_bOnGYMInfo)
							{
								sprintf(info, "%s %s  %c  %10.4lf %s(%12.4lf) 离开shadowCross.",it->first.c_str(),it->second.nameBlock.c_str(), it->second.yawBiasFlag, it->second.max_yawRate, t_epoch.toString().c_str(),hour);
								RuningInfoFile::Add(info);
							}
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
					if(bOnShadowIn && on_BLOCK_IIF) // IIF 卫星地影模型
					{//step2:判断是否离开地影，IIF无shawdowPost
						if(!GNSSBasicCorrectFunc::judgeGPSEarthShadowManeuver(sunPos,sp3Datum.pos,shadowFactor) || bOnLastPoint)
						{
							shadowCross.te     = t_epoch;
							shadowCross.yaw_te = yaw;
							shadowCross.yawRate_te = yawRate;
							bOnShadowIn  = false;						
							it->second.gpsShadowCrossList.push_back(shadowCross);
							if(m_bOnGYMInfo)
							{
								sprintf(info, "%s %s  %c  %10.4lf %s(%12.4lf) 离开shadowCross.",it->first.c_str(),it->second.nameBlock.c_str(), it->second.yawBiasFlag, it->second.max_yawRate, t_epoch.toString().c_str(),hour);
								RuningInfoFile::Add(info);
							}
							if(bOnLastPoint)
								break;
							continue;
						}
					}
					if(it->second.gpsShadowPostList.size() == it->second.gpsShadowCrossList.size() - 1 && !on_BLOCK_IIF)
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
							it->second.gpsShadowPostList.push_back(shadowPost);
							if(m_bOnGYMInfo)
							{
								sprintf(info, "%s %s  %c  %10.4lf %s(%12.4lf) 离开地影shadowPost.",it->first.c_str(),it->second.nameBlock.c_str(), it->second.yawBiasFlag, it->second.max_yawRate, t_epoch.toString().c_str(),hour);
								RuningInfoFile::Add(info);
							}
							if(bOnLastPoint)
								break;
							else
								continue;
						}
						yawPost_i_1 = yawPost_i;
					}
					if(fabs(beta) <= max_beta)//β角太大，不可能进入nooTurn, BLOCK II/IIA |β|= 4.9,IIR |β|=2.4
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
								if(m_bOnGYMInfo)
								{
									sprintf(info, "%s %s  %c  %10.4lf %s(%12.4lf) 进入noonTurn.",it->first.c_str(),it->second.nameBlock.c_str(), it->second.yawBiasFlag, it->second.max_yawRate, t_epoch.toString().c_str(),hour);
									RuningInfoFile::Add(info);
								}
								continue;
							}
							else
							{//进入midnightturn，仅block IIR 卫星,其余卫星一定在地影内
								noonTurn.ti         = t_epoch;
								noonTurn.yaw_ti     = yaw;
								noonTurn.yawRate_ti = it->second.max_yawRate;									
								bOnMidnightTurnIn   =  true;
								if(m_bOnGYMInfo)
								{
									sprintf(info, "%s %s  %c  %10.4lf %s(%12.4lf) 进入midnightturn.",it->first.c_str(),it->second.nameBlock.c_str(), it->second.yawBiasFlag, it->second.max_yawRate, t_epoch.toString().c_str(),hour);
									RuningInfoFile::Add(info);
								}
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
							//需要将yaw_maxRate 转到[-180,180]
							if(yawRate > 0)
							{
								if(yaw_maxRate >= yaw || bOnLastPoint)
								{
									noonTurn.te    = t_epoch;																
									it->second.gpsNoonTurnList.push_back(noonTurn);	
									if(bOnMidnightTurnIn)
									{
										bOnMidnightTurnIn   =  false;
										if(m_bOnGYMInfo)
										{
											sprintf(info, "%s %s  %c  %10.4lf %s(%12.4lf) 离开midnightturn.",it->first.c_str(),it->second.nameBlock.c_str(), it->second.yawBiasFlag, it->second.max_yawRate, t_epoch.toString().c_str(),hour);
											RuningInfoFile::Add(info);
										}
									}
									else
									{
										bOnNoonTurnIn  = false; //出了noonturn之后将bOnIn改为false
										if(m_bOnGYMInfo)
										{
											sprintf(info, "%s %s  %c  %10.4lf %s(%12.4lf) 离开noonTurn.",it->first.c_str(),it->second.nameBlock.c_str(), it->second.yawBiasFlag, it->second.max_yawRate, t_epoch.toString().c_str(),hour);
											RuningInfoFile::Add(info);
										}
									}
								}
							}
							else
							{
								if(yaw_maxRate <= yaw || bOnLastPoint)
								{
									noonTurn.te    = t_epoch;															
									it->second.gpsNoonTurnList.push_back(noonTurn);
									if(bOnMidnightTurnIn)
									{
										bOnMidnightTurnIn   =  false;
										if(m_bOnGYMInfo)
										{
											sprintf(info, "%s %s  %c  %10.4lf %s(%12.4lf) 离开midnightturn.",it->first.c_str(),it->second.nameBlock.c_str(), it->second.yawBiasFlag, it->second.max_yawRate, t_epoch.toString().c_str(),hour);
											RuningInfoFile::Add(info);
										}
									}
									else
									{
										bOnNoonTurnIn = false; // 出了noonturn之后将bOnIn改为false
										if(m_bOnGYMInfo)
										{
											sprintf(info, "%s %s  %c  %10.4lf %s(%12.4lf) 离开noonTurn.",it->first.c_str(),it->second.nameBlock.c_str(), it->second.yawBiasFlag, it->second.max_yawRate, t_epoch.toString().c_str(),hour);
											RuningInfoFile::Add(info);
										}
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
			// 处理GLONASS卫星的机动问题, GLONASS-M 卫星采用 Dilssner 模型
			if(it->first[0] == 'R')
			{
				GYM_R11_ShadowCrossDatum   shadowCross;
				GYM_R11_NoonTurnDatum      noonTurn;
				GPST t0_sp3 = m_sp3File.m_data.front().t;
				GPST t1_sp3 = m_sp3File.m_data.back().t;
				double max_beta = 5 * PI/180;     // 可能进入noonTurn的最大beta角,适当扩大搜索范围
				bool bOnNoonTurnIn     = false;	
				bool bOnMidnightTurnIn = false;
				bool bOnShadowIn       = false;		
				bool bOnLastPoint      = false; // 增加最后一点判断
				int i = 0;

				while(t0_sp3 + i * span_t - t1_sp3 <= 0)
				{
					GPST t_epoch = t0_sp3 + i * span_t;
					double hour = t_epoch.hour +  t_epoch.minute/60.0 + t_epoch.second/3600.0;	
					if(fabs(t_epoch - t1_sp3) < 1e-3)
					{//最后一点,前提是t1_sp3 - t0_sp3为span_t的整数倍
						bOnLastPoint = true;					
					}
					// 获取太阳位置
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
                    // 获取卫星位置
					POS6D    gpsPosVel;
					SP3Datum sp3Datum;
					if(!m_sp3File.getEphemeris(t_epoch,it->first,sp3Datum))
					{
						i++; // 2014/08/26, 谷德峰修改, 防止死循环
						continue;
					}
					gpsPosVel.setPos(sp3Datum.pos);
					gpsPosVel.setVel(sp3Datum.vel);	

					int sign_b = 1; // 常量姿态偏差正负标识
					double beta = getBetaSunAngle(sunPos, gpsPosVel); // 太阳入射方向与轨道面夹角β,单位：度
					double yaw, yawRate;
					nominalYawAttitude_GLONASS(it->second.nameBlock, sunPos, gpsPosVel, yaw, yawRate, beta);
					beta = beta * PI / 180.0;
					double shadowFactor = 1;
					// GLONASS 卫星考虑地影，进地影后即以最大姿态速度进行姿态调整
					if(!bOnShadowIn)
					{//step1:判断是否进入地影
						if(GNSSBasicCorrectFunc::judgeGPSEarthShadowManeuver(sunPos,sp3Datum.pos,shadowFactor))
						{						
							shadowCross.ti         = t_epoch; // 进入地影时刻
							shadowCross.yaw_ti     = yaw;
							shadowCross.yawRate_ti = yawRate;
							bOnShadowIn            = true;						
							if(m_bOnGYMInfo)
							{
								sprintf(info, "%s %s  %c  %10.4lf %s(%12.4lf) 进入地影shadowCross.",it->first.c_str(),it->second.nameBlock.c_str(), it->second.yawBiasFlag, it->second.max_yawRate, t_epoch.toString().c_str(),hour);
								RuningInfoFile::Add(info);
							}
							continue;
						}
					}		
					if(bOnShadowIn) // IIF 卫星地影模型
					{//step2:判断是否离开地影，并进入shawdowPost （GLONASS 无shawdowPost）
						if(!GNSSBasicCorrectFunc::judgeGPSEarthShadowManeuver(sunPos,sp3Datum.pos,shadowFactor) || bOnLastPoint)
						{
							shadowCross.te     = t_epoch;
							shadowCross.yaw_te = yaw;
							shadowCross.yawRate_te = yawRate;
							double t1 = abs(shadowCross.yaw_te - shadowCross.yaw_ti)/it->second.max_yawRate;
							shadowCross.t1 = shadowCross.ti + t1;
							bOnShadowIn  = false;						
							it->second.glonassShadowCrossList.push_back(shadowCross);
							if(m_bOnGYMInfo)
							{
								sprintf(info, "%s %s  %c  %10.4lf %s(%12.4lf) 离开shadowCross.",it->first.c_str(),it->second.nameBlock.c_str(), it->second.yawBiasFlag, it->second.max_yawRate, t_epoch.toString().c_str(),hour);
								RuningInfoFile::Add(info);
							}
							if(bOnLastPoint)
								break;
							continue;
						}
					}
					if(fabs(beta) <= max_beta)//β角太大，不可能进入nooTurn β< 2.0
					{
						if(fabs(yawRate) >= it->second.max_yawRate && !bOnNoonTurnIn && !bOnMidnightTurnIn && !bOnShadowIn)// 进入noonturn的必要条件
						{//step4:判断是否进入 noonturn					
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
								if(m_bOnGYMInfo)
								{
									sprintf(info, "%s %s  %c  %10.4lf %s(%12.4lf) 进入noonTurn.",it->first.c_str(),it->second.nameBlock.c_str(), it->second.yawBiasFlag, it->second.max_yawRate, t_epoch.toString().c_str(),hour);
									RuningInfoFile::Add(info);
								}
								continue;
							}
						}
						if((bOnNoonTurnIn) && (fabs(yawRate) < it->second.max_yawRate || bOnLastPoint))
						{//step5:判断是否离开noonturn	
							int sign_beta = 1;
							if(beta < 0)
								sign_beta = -1;	
							if(bOnMidnightTurnIn)
								sign_beta = - sign_beta;						
							double yaw_maxRate = noonTurn.yaw_ti - sign_beta * fabs(it->second.max_yawRate) * (t_epoch - noonTurn.ti);
							//需要将yaw_maxRate 转到[-180,180]
							if(yawRate > 0)
							{
								if(yaw_maxRate >= yaw || bOnLastPoint)
								{
									noonTurn.te    = t_epoch;																
									it->second.glonassNoonTurnList.push_back(noonTurn);	
									bOnNoonTurnIn  = false; //出了noonturn之后将bOnIn改为false
									if(m_bOnGYMInfo)
									{
										sprintf(info, "%s %s  %c  %10.4lf %s(%12.4lf) 离开noonTurn.",it->first.c_str(),it->second.nameBlock.c_str(), it->second.yawBiasFlag, it->second.max_yawRate, t_epoch.toString().c_str(),hour);
										RuningInfoFile::Add(info);
									}
								}
							}
							else
							{
								if(yaw_maxRate <= yaw || bOnLastPoint)
								{
									noonTurn.te    = t_epoch;															
									it->second.glonassNoonTurnList.push_back(noonTurn);
									bOnNoonTurnIn = false; // 出了noonturn之后将bOnIn改为false
									if(m_bOnGYMInfo)
									{
										sprintf(info, "%s %s  %c  %10.4lf %s(%12.4lf) 离开noonTurn.",it->first.c_str(),it->second.nameBlock.c_str(), it->second.yawBiasFlag, it->second.max_yawRate, t_epoch.toString().c_str(),hour);
										RuningInfoFile::Add(info);
									}
								}
							}
						}
					}				
					i++;
				}
			}
			// 处理GALILEO卫星的机动问题, 动态动偏姿态控制模型
			if(it->first[0] == 'E')
			{
				GYM_E08_ShadowCrossDatum   shadowCross;
				GYM_E08_NoonTurnDatum      noonTurn;
				GPST t0_sp3 = m_sp3File.m_data.front().t;
				GPST t1_sp3 = m_sp3File.m_data.back().t;
				double max_beta = 5 * PI/180;     // 可能进入noonTurn的最大beta角,适当扩大搜索范围
				bool bOnNoonTurnIn     = false;	
				bool bOnShadowIn       = false;		
				bool bOnLastPoint      = false; // 增加最后一点判断
				int i = 0;
				while(t0_sp3 + i * span_t - t1_sp3 <= 0)
				{
					GPST t_epoch = t0_sp3 + i * span_t;
					double hour = t_epoch.hour +  t_epoch.minute/60.0 + t_epoch.second/3600.0;	
					if(fabs(t_epoch - t1_sp3) < 1e-3)
					{//最后一点,前提是t1_sp3 - t0_sp3为span_t的整数倍
						bOnLastPoint = true;					
					}
					// 获取太阳位置
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
                    // 获取卫星位置
					POS6D    gpsPosVel;
					SP3Datum sp3Datum;
					if(!m_sp3File.getEphemeris(t_epoch,it->first,sp3Datum))
					{
						i++; // 2014/08/26, 谷德峰修改, 防止死循环
						continue;
					}
					gpsPosVel.setPos(sp3Datum.pos);
					gpsPosVel.setVel(sp3Datum.vel);	

					int sign_b = 1; // 正负标识
					double beta = getBetaSunAngle(sunPos, gpsPosVel);       // 太阳入射方向与轨道面夹角β,单位：度 
					double u = getUOrbitAngle_perihelion(sunPos, gpsPosVel);// 卫星到地心与“近日点”到地心连线的夹角u,单位：度
					double yaw, yawRate;
					nominalYawAttitude_GALILEO(it->second.nameBlock, sunPos, gpsPosVel, yaw, yawRate, beta);
					beta = beta * PI / 180.0;
					// 根据太阳高度角和轨道角计算太阳单位矢量，Sx,Sy,Sz
					double Sy = -sin(beta);
					double shadowFactor = 1;
					// GALILEO 卫星考虑地影
					if(!bOnShadowIn)
					{//step1:判断是否进入地影
						if(GNSSBasicCorrectFunc::judgeGPSEarthShadowManeuver(sunPos,sp3Datum.pos,shadowFactor))
						{						
							shadowCross.ti         = t_epoch; // 进入地影时刻
							shadowCross.yaw_ti     = yaw;
							shadowCross.yawRate_ti = yawRate;
							if(Sy < 0)
								sign_b = -1;
							shadowCross.sign_ti = sign_b;
							bOnShadowIn            = true;						
							if(m_bOnGYMInfo)
							{
								sprintf(info, "%s %s  %c  %10.4lf %s(%12.4lf) 进入地影shadowCross.",it->first.c_str(),it->second.nameBlock.c_str(), it->second.yawBiasFlag, it->second.max_yawRate, t_epoch.toString().c_str(),hour);
								RuningInfoFile::Add(info);
							}
							continue;
						}
					}		
					if(bOnShadowIn) 
					{//step2:判断是否离开地影，无shawdowPost
						if(!GNSSBasicCorrectFunc::judgeGPSEarthShadowManeuver(sunPos,sp3Datum.pos,shadowFactor) || bOnLastPoint)
						{
							shadowCross.te     = t_epoch;
							shadowCross.yaw_te = yaw;
							shadowCross.yawRate_te = yawRate;
							bOnShadowIn  = false;						
							it->second.galileoShadowCrossList.push_back(shadowCross);
							if(m_bOnGYMInfo)
							{
								sprintf(info, "%s %s  %c  %10.4lf %s(%12.4lf) 离开shadowCross.",it->first.c_str(),it->second.nameBlock.c_str(), it->second.yawBiasFlag, it->second.max_yawRate, t_epoch.toString().c_str(),hour);
								RuningInfoFile::Add(info);
							}
							if(bOnLastPoint)
								break;
							continue;
						}
					}
					if(fabs(beta) <= max_beta)//β角太大，不可能进入nooTurn, β< 2.0 for IOV; β<4.1 for
					{
						if(fabs(u) < 15 && !bOnNoonTurnIn && !bOnShadowIn)// 进入noonturn的必要条件
						{//step4:判断是否进入 noonturn					
							noonTurn.ti         = t_epoch;
							noonTurn.yaw_ti     = yaw;
							noonTurn.yawRate_ti = it->second.max_yawRate;
							if(Sy < 0)
								noonTurn.sign_ti = -1;
							else
								noonTurn.sign_ti = 1;
							bOnNoonTurnIn       =  true;							
							if(m_bOnGYMInfo)
							{
								sprintf(info, "%s %s  %c  %10.4lf %s(%12.4lf) 进入noonTurn.",it->first.c_str(),it->second.nameBlock.c_str(), it->second.yawBiasFlag, it->second.max_yawRate, t_epoch.toString().c_str(),hour);
								RuningInfoFile::Add(info);
							}
							continue;
						}
						if((bOnNoonTurnIn) && (fabs(u) >= 15 || bOnLastPoint))
						{//step5:判断是否离开noonturn	
							noonTurn.te    = t_epoch;															
							it->second.galileoNoonTurnList.push_back(noonTurn);
							bOnNoonTurnIn = false; // 出了noonturn之后将bOnIn改为false
							if(m_bOnGYMInfo)
							{
								sprintf(info, "%s %s  %c  %10.4lf %s(%12.4lf) 离开noonTurn.",it->first.c_str(),it->second.nameBlock.c_str(), it->second.yawBiasFlag, it->second.max_yawRate, t_epoch.toString().c_str(),hour);
								RuningInfoFile::Add(info);
							}
						}
					}				
					i++;
				}
			}
			// 处理BDS卫星的机动问题, 采用BYM15模型
			//if(it->first[0] == 'C')
			//{
			//	GPST t0_sp3 = m_sp3File.m_data.front().t;
			//	GPST t1_sp3 = m_sp3File.m_data.back().t;
			//	if(it->second.nameBlock.find(BLOCK_MaskString::BEIDOU_2G) != -1
			//	|| it->second.nameBlock.find(BLOCK_MaskString::BEIDOU_3G) != -1)
			//		continue; // GEO 不处理, 始终 yaw = 0.0
			//	int i = 0;

			//	vector<GPST>   epochList;
			//	vector<double> yawList;
			//	vector<double> betaList;
			//	vector<int>    markList; // 1: yaw nominal; 0: orbit nominal

			//	while(t0_sp3 + i * span_t - t1_sp3 <= 0)
			//	{
			//		GPST t_epoch = t0_sp3 + i * span_t;
			//		double hour = t_epoch.hour +  t_epoch.minute/60.0 + t_epoch.second/3600.0;	
			//		
			//		POS3D sunPos;
			//		TDB t_TDB = TimeCoordConvert::GPST2TDB(t_epoch); // 获得 TDB 时间--提供太阳历参考时间									
			//		double jd = TimeCoordConvert::DayTime2JD(t_TDB); // 获得儒略日									
			//		double Pos[3];
			//		if(!m_JPLEphFile.getSunPos_Delay_EarthCenter(jd, Pos))
			//		{
			//			printf("%s获取太阳位置失败!\n",t_epoch.toString().c_str());
			//			return false;
			//		}
			//		sunPos.x = Pos[0] * 1000; 
			//		sunPos.y = Pos[1] * 1000; 
			//		sunPos.z = Pos[2] * 1000; 
			//		POS6D bdsPosVel;
			//		SP3Datum sp3Datum;
			//		if(!m_sp3File.getEphemeris(t_epoch,it->first,sp3Datum))
			//		{
			//			i++; // 2014/08/26, 谷德峰修改, 防止死循环
			//			continue;
			//		}
			//		bdsPosVel.setPos(sp3Datum.pos);
			//		bdsPosVel.setVel(sp3Datum.vel);
			//		double yaw;
			//		double yawRate;
			//		double beta;
			//		nominalYawAttitude_BDS(it->second.nameBlock, sunPos, bdsPosVel, yaw, yawRate, beta);
			//		
			//		yawList.push_back(yaw);
			//		betaList.push_back(beta);
			//		epochList.push_back(t_epoch);

			//		// J Geod (2015) Estimating the yaw-attitude of BDS IGSO and MEO satellites
			//		// Since the Sun elevation varies less than 1? per day, the orbit-normal attitude mode could last for about eight days each cycle.
			//		// 寻找Entry时间点
			//		if(markList.size() == 0)
			//		{
			//			// 初始点的判断, 1-yaw nominal; 0-orbit nominal
			//			if(fabs(beta) <= it->second.min_betaBDSYaw2Orb)
			//				markList.push_back(0);
			//			else
			//				markList.push_back(1);
			//		}
			//		else
			//		{
			//			int id_time = int(markList.size());
			//			// 通常如果卫星进入Entry, 1天内不可能再跳出Entry, 要持续几天时间
			//			if(fabs(beta) <= it->second.min_betaBDSYaw2Orb && markList[id_time - 1] == 1)
			//			{
			//				markList.push_back(0); // OrbNormalEntry
			//				GYM_C15_OrbNormalEntryDatum OrbNormalEntryDatum;
			//				OrbNormalEntryDatum.tBeta = t_epoch;
			//				OrbNormalEntryDatum.id_time = id_time;
			//				OrbNormalEntryDatum.yaw_tBeta = yaw;
			//				it->second.bdsOrbNormalEntryList.push_back(OrbNormalEntryDatum);
			//			}
			//			else if(fabs(beta) > it->second.min_betaBDSYaw2Orb && markList[id_time - 1] == 0)
			//			{
			//				markList.push_back(1); // YawNominalEntry
			//				GYM_C15_YawNominalEntryDatum yawNominalEntryDatum;
			//				yawNominalEntryDatum.tBeta = t_epoch;
			//				yawNominalEntryDatum.id_time = id_time;
			//				yawNominalEntryDatum.yaw_tBeta = 0.0;
			//				it->second.bdsYawNominalEntryList.push_back(yawNominalEntryDatum);
			//			}
			//			else
			//			{// 保持与前一时刻相同
			//				markList.push_back(markList[id_time - 1]); 
			//			}
			//		}
			//		i++;
			//	}
			//	// 计算机动信息, 并清除未完成的机动
			//	size_t s_i = 0;
			//	while(s_i < it->second.bdsOrbNormalEntryList.size())
			//	{
			//		// 从 id_time 开始寻找最接近 0 的 yaw
			//		int id_t0 = -1;
			//		for(int j = it->second.bdsOrbNormalEntryList[s_i].id_time + 1; j < int(markList.size()); j++)
			//		{
			//			// 减小过程, 且接近目标0时跳出
			//			if(fabs(yawList[j]) - fabs(yawList[j-1]) > 0.0 && fabs(yawList[j]) <= 20.0)
			//			{//
			//				id_t0 = j-1;
			//				it->second.bdsOrbNormalEntryList[s_i].t0 = epochList[j - 1];
			//				it->second.bdsOrbNormalEntryList[s_i].yaw_t0 = yawList[j - 1];
			//				double span_maneuver = fabs(it->second.bdsOrbNormalEntryList[s_i].yaw_t0) / it->second.max_yawRate;
			//				it->second.bdsOrbNormalEntryList[s_i].t1 = it->second.bdsOrbNormalEntryList[s_i].t0 + span_maneuver;
			//				it->second.bdsOrbNormalEntryList[s_i].yaw_t1 = 0.0;
			//				if(m_bOnGYMInfo)
			//				{
			//					double hour = it->second.bdsOrbNormalEntryList[s_i].t0.hour +  it->second.bdsOrbNormalEntryList[s_i].t0.minute/60.0 + it->second.bdsOrbNormalEntryList[s_i].t0.second/3600.0;	
			//					sprintf(info, "%s %s  %c  %10.4lf %s(%12.4lf) yaw nominal -> orbit normal, 耗时%.2fs.",it->first.c_str(),it->second.nameBlock.c_str(), it->second.yawBiasFlag, it->second.max_yawRate, it->second.bdsOrbNormalEntryList[s_i].t0.toString().c_str(),hour, span_maneuver);
			//					RuningInfoFile::Add(info);
			//				}
			//				break;
			//			}
			//		}
			//		// 如果未找到, 说明尚未达到机动发生时刻, 删除该机
			//		if(id_t0 == -1)
			//		{
			//			it->second.bdsOrbNormalEntryList.erase(it->second.bdsOrbNormalEntryList.begin() + s_i);
			//			continue;
			//		}
			//		s_i++;
			//	}
			//	// 计算机动信息
			//	s_i = 0;
			//	while(s_i < it->second.bdsYawNominalEntryList.size())
			//	{
			//		// 从 id_time 开始寻找最接近 0 的 yaw
			//		int id_t0 = -1;
			//		for(int j = it->second.bdsYawNominalEntryList[s_i].id_time + 1; j < int(markList.size()); j++)
			//		{
			//			// 减小过程, 且接近目标0时跳出
			//			if(fabs(yawList[j]) - fabs(yawList[j-1]) > 0.0 && fabs(yawList[j]) <= 20.0)
			//			{//
			//				id_t0 = j-1;
			//				it->second.bdsYawNominalEntryList[s_i].t0 = epochList[j - 1];
			//				it->second.bdsYawNominalEntryList[s_i].yaw_t0 = yawList[j - 1];
			//				double span_maneuver = fabs(it->second.bdsYawNominalEntryList[s_i].yaw_t0) / it->second.max_yawRate;
			//				it->second.bdsYawNominalEntryList[s_i].t1 = it->second.bdsYawNominalEntryList[s_i].t0 + span_maneuver;
			//				it->second.bdsYawNominalEntryList[s_i].yaw_t1 = it->second.bdsYawNominalEntryList[s_i].yaw_t0;
			//				if(m_bOnGYMInfo)
			//				{
			//					double hour = it->second.bdsYawNominalEntryList[s_i].t0.hour +  it->second.bdsYawNominalEntryList[s_i].t0.minute/60.0 + it->second.bdsYawNominalEntryList[s_i].t0.second/3600.0;	
			//					sprintf(info, "%s %s  %c  %10.4lf %s(%12.4lf) orbit normal -> yaw nominal, 耗时%.2fs.",it->first.c_str(),it->second.nameBlock.c_str(), it->second.yawBiasFlag, it->second.max_yawRate, it->second.bdsYawNominalEntryList[s_i].t0.toString().c_str(),hour, span_maneuver);
			//					RuningInfoFile::Add(info);
			//				}
			//				break;
			//			}
			//		}
			//		// 如果未找到, 说明尚未达到机动发生时刻, 删除该机
			//		if(id_t0 == -1)
			//		{
			//			it->second.bdsYawNominalEntryList.erase(it->second.bdsYawNominalEntryList.begin() + s_i);
			//			continue;
			//		}
			//		s_i++;
			//	}
			//}
			// 处理BDS卫星的机动问题, BDS2采用BYM15模型，BDS3采用动态偏航模型
			if(it->first[0] == 'C')
			{
				GPST t_Attchange_C06(2017, 03, 01, 0, 0, 0.0);
				GPST t_Attchange_C13(2016, 03, 29, 0, 0, 0.0);
				GPST t_Attchange_C14(2016, 10, 01, 0, 0, 0.0);
				GYM_C15_NoonTurnDatum      noonTurn;//BDS3动态偏航模型与Galileo类似，包含midnightturn
				GPST t0_sp3 = m_sp3File.m_data.front().t;
				GPST t1_sp3 = m_sp3File.m_data.back().t;
				double max_beta = 3.0;     // 可能进入noonTurn的最大beta角,适当扩大搜索范围
				bool bOnNoonTurnIn     = false;	
				bool bOnMidnightTurnIn = false;		
				bool bOnLastPoint      = false; // 增加最后一点判断
				if(it->second.nameBlock.find(BLOCK_MaskString::BEIDOU_2G) != -1
				|| it->second.nameBlock.find(BLOCK_MaskString::BEIDOU_3G) != -1
				|| it->second.nameBlock.find(BLOCK_MaskString::BEIDOU_3G_CAST) != -1)
					continue; // GEO 不处理, 始终 yaw = 0.0
				bool on_BDS3 = false;  // BDS3、BDS2 C06\C13\C14考虑noonturn、midnightturn				
				if(it->second.nameBlock.find(BLOCK_MaskString::BEIDOU_3I) != -1
				|| it->second.nameBlock.find(BLOCK_MaskString::BEIDOU_3M_CAST) != -1
				|| it->second.nameBlock.find(BLOCK_MaskString::BEIDOU_3M_SECM_A) != -1
				|| it->second.nameBlock.find(BLOCK_MaskString::BEIDOU_3M_SECM_B) != -1
				|| (it->first.find("C06") != -1 && (t0_sp3 - t_Attchange_C06) > 0)
				|| (it->first.find("C13") != -1 && (t0_sp3 - t_Attchange_C13) > 0)
				|| (it->first.find("C14") != -1 && (t0_sp3 - t_Attchange_C14) > 0)
				|| it->first.find("C16") != -1)
					on_BDS3 = true;
				
				int i = 0;

				vector<GPST>   epochList;
				vector<double> yawList;
				vector<double> betaList;
				vector<int>    markList; // 1: yaw nominal; 0: orbit nominal

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
					POS6D bdsPosVel;
					SP3Datum sp3Datum;
					if(!m_sp3File.getEphemeris(t_epoch,it->first,sp3Datum))
					{
						i++; // 2014/08/26, 谷德峰修改, 防止死循环
						continue;
					}
					bdsPosVel.setPos(sp3Datum.pos);
					bdsPosVel.setVel(sp3Datum.vel);

					double yaw, yawRate, beta;
					nominalYawAttitude_BDS(it->second.nameBlock, sunPos, bdsPosVel, yaw, yawRate, beta);
					
					//移动，考虑C13、C14卫星beta<0.14时的调整，张厚喆，20210407
					//yawList.push_back(yaw);
					//betaList.push_back(beta);
					//epochList.push_back(t_epoch);
					double u = getUOrbitAngle(sunPos, bdsPosVel);// 卫星到地心与“远日点”到地心连线的夹角μ
					double v_sat_norm2 = sqrt(vectorDot(bdsPosVel.getVel(), bdsPosVel.getVel()));
					double r_sat_norm2 = sqrt(vectorDot(bdsPosVel.getPos(), bdsPosVel.getPos()));
					double uRate = (v_sat_norm2 / r_sat_norm2) * 180.0 / PI;//v=ωR，使用这一公式时应注意，角度的单位一定要用弧度，只有角速度的单位是弧度/秒时，上述公式才成立
					int sign_b = 1; // 正负标识

					// J Geod (2015) Estimating the yaw-attitude of BDS IGSO and MEO satellites
					// Since the Sun elevation varies less than 1? per day, the orbit-normal attitude mode could last for about eight days each cycle.
					// 寻找Entry时间点
					if(markList.size() == 0)
					{
						// 初始点的判断, 1-yaw nominal; 0-orbit nominal
						if(fabs(beta) <= it->second.min_betaBDSYaw2Orb)
							markList.push_back(0);
						else
							markList.push_back(1);
					}
					else
					{
						int id_time = int(markList.size());
						// 通常如果卫星进入Entry, 1天内不可能再跳出Entry, 要持续几天时间
						if(fabs(beta) <= it->second.min_betaBDSYaw2Orb && markList[id_time - 1] == 1)
						{
							markList.push_back(0); // OrbNormalEntry
							GYM_C15_OrbNormalEntryDatum OrbNormalEntryDatum;
							OrbNormalEntryDatum.tBeta = t_epoch;
							OrbNormalEntryDatum.id_time = id_time;
							OrbNormalEntryDatum.yaw_tBeta = yaw;
							it->second.bdsOrbNormalEntryList.push_back(OrbNormalEntryDatum);
						}
						else if(fabs(beta) > it->second.min_betaBDSYaw2Orb && markList[id_time - 1] == 0)
						{
							markList.push_back(1); // YawNominalEntry
							GYM_C15_YawNominalEntryDatum yawNominalEntryDatum;
							yawNominalEntryDatum.tBeta = t_epoch;
							yawNominalEntryDatum.id_time = id_time;
							yawNominalEntryDatum.yaw_tBeta = 0.0;
							it->second.bdsYawNominalEntryList.push_back(yawNominalEntryDatum);
						}
						else
						{// 保持与前一时刻相同
							markList.push_back(markList[id_time - 1]); 
						}
					}
					//移动至此处，张厚喆，20210407
					yawList.push_back(yaw);
					betaList.push_back(beta);
					epochList.push_back(t_epoch);

					// 北斗三号 卫星考虑Noon-turn和Midnight-turn maneuvers
					if((fabs(beta) <= max_beta) && on_BDS3)//β角太大，不可能进入noonTurn, β< 3.0 I06 and BDS3
					{
						if(fabs(u) < 6 && !bOnMidnightTurnIn)// 进入Midnightturn的必要条件
						{//step4:判断是否进入 noonturn					
							noonTurn.ti         = t_epoch;
							if((fabs(beta) <= 0.14)
								&&((it->first.find("C13") != -1 && (t_epoch - t_Attchange_C13) > 0.0)
								|| (it->first.find("C14") != -1 && (t_epoch - t_Attchange_C14) > 0.0)))
								nominalYawAttitude_BDS_bias(it->second.nameBlock, sunPos, bdsPosVel, yaw, yawRate, beta);
							noonTurn.yaw_ti     = yaw;
							noonTurn.yawRate_ti = yawRate/*it->second.max_yawRate*/;
							noonTurn.u_ti       = u;
							noonTurn.uRate_ti   = uRate;
							if(yaw < 0)
								noonTurn.sign_ti = -1;
							else
								noonTurn.sign_ti = 1;
							bOnMidnightTurnIn    =  true;							
							if(m_bOnGYMInfo)
							{
								sprintf(info, "%s %s  %c  %10.4lf %s(%12.4lf) 进入MidnightTurn.",it->first.c_str(),it->second.nameBlock.c_str(), it->second.yawBiasFlag, it->second.max_yawRate, t_epoch.toString().c_str(),hour);
								RuningInfoFile::Add(info);
							}
							continue;
						}
						if(fabs(u) > 174 && !bOnNoonTurnIn)// 进入noonturn的必要条件
						{//step4:判断是否进入 noonturn					
							noonTurn.ti         = t_epoch;
							if((fabs(beta) <= 0.14)
								&&((it->first.find("C13") != -1 && (t_epoch - t_Attchange_C13) > 0.0)
								|| (it->first.find("C14") != -1 && (t_epoch - t_Attchange_C14) > 0.0)))
								nominalYawAttitude_BDS_bias(it->second.nameBlock, sunPos, bdsPosVel, yaw, yawRate, beta);
							noonTurn.yaw_ti     = yaw;
							noonTurn.yawRate_ti = yawRate/*it->second.max_yawRate*/;
							noonTurn.u_ti       = u;
							noonTurn.uRate_ti   = uRate;
							if(yaw < 0)
								noonTurn.sign_ti = -1;
							else
								noonTurn.sign_ti = 1;
							bOnNoonTurnIn       =  true;							
							if(m_bOnGYMInfo)
							{
								sprintf(info, "%s %s  %c  %10.4lf %s(%12.4lf) 进入noonTurn.",it->first.c_str(),it->second.nameBlock.c_str(), it->second.yawBiasFlag, it->second.max_yawRate, t_epoch.toString().c_str(),hour);
								RuningInfoFile::Add(info);
							}
							continue;
						}
						if((bOnMidnightTurnIn) && (fabs(u) >= 6 || bOnLastPoint))
						{//step5:判断是否离开Midnightturn	
							noonTurn.te    = t_epoch;															
							it->second.bdsNoonTurnList.push_back(noonTurn);
							bOnMidnightTurnIn = false; // 出了Midnightturn之后将bOnIn改为false
							if(m_bOnGYMInfo)
							{
								sprintf(info, "%s %s  %c  %10.4lf %s(%12.4lf) 离开MidnightTurn.",it->first.c_str(),it->second.nameBlock.c_str(), it->second.yawBiasFlag, it->second.max_yawRate, t_epoch.toString().c_str(),hour);
								RuningInfoFile::Add(info);
							}
						}
						if((bOnNoonTurnIn) && (fabs(u) <= 174 || bOnLastPoint))
						{//step5:判断是否离开noonturn	
							noonTurn.te    = t_epoch;															
							it->second.bdsNoonTurnList.push_back(noonTurn);
							bOnNoonTurnIn = false; // 出了noonturn之后将bOnIn改为false
							if(m_bOnGYMInfo)
							{
								sprintf(info, "%s %s  %c  %10.4lf %s(%12.4lf) 离开noonTurn.",it->first.c_str(),it->second.nameBlock.c_str(), it->second.yawBiasFlag, it->second.max_yawRate, t_epoch.toString().c_str(),hour);
								RuningInfoFile::Add(info);
							}
						}
					}
					i++;
				}
				// 计算机动信息, 并清除未完成的机动
				size_t s_i = 0;
				while(s_i < it->second.bdsOrbNormalEntryList.size())
				{
					// 从 id_time 开始寻找最接近 0 的 yaw
					int id_t0 = -1;
					for(int j = it->second.bdsOrbNormalEntryList[s_i].id_time + 1; j < int(markList.size()); j++)
					{
						// 减小过程, 且接近目标0时跳出
						if(fabs(yawList[j]) - fabs(yawList[j-1]) > 0.0 && fabs(yawList[j]) <= 20.0)
						{//
							id_t0 = j-1;
							it->second.bdsOrbNormalEntryList[s_i].t0 = epochList[j - 1];
							it->second.bdsOrbNormalEntryList[s_i].yaw_t0 = yawList[j - 1];
							double span_maneuver = fabs(it->second.bdsOrbNormalEntryList[s_i].yaw_t0) / it->second.max_yawRate;
							it->second.bdsOrbNormalEntryList[s_i].t1 = it->second.bdsOrbNormalEntryList[s_i].t0 + span_maneuver;
							it->second.bdsOrbNormalEntryList[s_i].yaw_t1 = 0.0;
							if(m_bOnGYMInfo)
							{
								double hour = it->second.bdsOrbNormalEntryList[s_i].t0.hour +  it->second.bdsOrbNormalEntryList[s_i].t0.minute/60.0 + it->second.bdsOrbNormalEntryList[s_i].t0.second/3600.0;	
								sprintf(info, "%s %s  %c  %10.4lf %s(%12.4lf) yaw nominal -> orbit normal, 耗时%.2fs.",it->first.c_str(),it->second.nameBlock.c_str(), it->second.yawBiasFlag, it->second.max_yawRate, it->second.bdsOrbNormalEntryList[s_i].t0.toString().c_str(),hour, span_maneuver);
								RuningInfoFile::Add(info);
							}
							break;
						}
					}
					// 如果未找到, 说明尚未达到机动发生时刻, 删除该机
					if(id_t0 == -1)
					{
						it->second.bdsOrbNormalEntryList.erase(it->second.bdsOrbNormalEntryList.begin() + s_i);
						continue;
					}
					s_i++;
				}
				// 计算机动信息
				s_i = 0;
				while(s_i < it->second.bdsYawNominalEntryList.size())
				{
					// 从 id_time 开始寻找最接近 0 的 yaw
					int id_t0 = -1;
					for(int j = it->second.bdsYawNominalEntryList[s_i].id_time + 1; j < int(markList.size()); j++)
					{
						// 减小过程, 且接近目标0时跳出
						if(fabs(yawList[j]) - fabs(yawList[j-1]) > 0.0 && fabs(yawList[j]) <= 20.0)
						{//
							id_t0 = j-1;
							it->second.bdsYawNominalEntryList[s_i].t0 = epochList[j - 1];
							it->second.bdsYawNominalEntryList[s_i].yaw_t0 = yawList[j - 1];
							double span_maneuver = fabs(it->second.bdsYawNominalEntryList[s_i].yaw_t0) / it->second.max_yawRate;
							it->second.bdsYawNominalEntryList[s_i].t1 = it->second.bdsYawNominalEntryList[s_i].t0 + span_maneuver;
							it->second.bdsYawNominalEntryList[s_i].yaw_t1 = it->second.bdsYawNominalEntryList[s_i].yaw_t0;
							if(m_bOnGYMInfo)
							{
								double hour = it->second.bdsYawNominalEntryList[s_i].t0.hour +  it->second.bdsYawNominalEntryList[s_i].t0.minute/60.0 + it->second.bdsYawNominalEntryList[s_i].t0.second/3600.0;	
								sprintf(info, "%s %s  %c  %10.4lf %s(%12.4lf) orbit normal -> yaw nominal, 耗时%.2fs.",it->first.c_str(),it->second.nameBlock.c_str(), it->second.yawBiasFlag, it->second.max_yawRate, it->second.bdsYawNominalEntryList[s_i].t0.toString().c_str(),hour, span_maneuver);
								RuningInfoFile::Add(info);
							}
							break;
						}
					}
					// 如果未找到, 说明尚未达到机动发生时刻, 删除该机
					if(id_t0 == -1)
					{
						it->second.bdsYawNominalEntryList.erase(it->second.bdsYawNominalEntryList.begin() + s_i);
						continue;
					}
					s_i++;
				}
			}
		}
		//fclose(pfile);
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
	void GNSSYawAttitudeModel::yaw2unitXYZ(TimePosVel gpsPVT, double acsYaw, POS3D &ex, POS3D &ey, POS3D &ez, bool bECEF)
	{
		//acsYaw = -acsYaw; // 纠正, 谷德峰, 20160408	
		POS3D eR,eT,eN;     // 轨道系的三轴单位矢量
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
			//*************************************************
			m_TimeCoordConvert.getCoordinateRTNAxisVector(t, gpsPVT.getPosVel(), eR, eT, eN);
			ez = eR * (-1.0); // Z轴单位矢量, 卫星指向地心	
			eN = eN * (-1.0); 
			//eN = eN * (1.0); 
			acsYaw = acsYaw * PI / 180; // 转换为弧度
			ex = eT * cos(acsYaw) + eN * sin(acsYaw); // ex = cos(yaw) * eT + sin(yaw) * eN方向	
			ex = vectorNormal(ex);
			vectorCross(ey, ez, ex);
			ey = vectorNormal(ey); // Y轴单位矢量
		}		 
	}

	// 子程序名称： gpsACSYawAttitude   
	// 功能：计算姿态控制系统 attitude control subsystem (ACS) 偏航姿态角及其变化率
	// 变量类型：nameSat                  : 卫星名称
	//           t                        : 当前时间
	//           yaw                      : 偏航姿态角, 单位：度
	//           yawRate                  : 偏航姿态角变化率
	//           betaSun                  : 太阳入射方向与轨道面夹角β,单位：度
	//           uOrbit                   : 轨道角β,单位：度
	//           bUsed_NominalYawAttitude : 是否使用标称偏航姿态角及其变化率 
	//           on_J2000                 : 是否需要将sp3星历转为J2000惯性系，默认为 true
	// 输入：nameSat, t
	// 输出：yaw, yawRate
	// 语言：C++
	// 创建者：谷德峰, 刘俊宏
	// 创建时间：2014/11/5
	// 版本时间：
	// 修改记录：1、增加轨道角u的计算，邵凯，2019/8/31
	// 备注： 
	int GNSSYawAttitudeModel::gpsACSYawAttitude(string nameSat, GPST t, double& yaw, double &yawRate, double &betaSun, double &uOrbit, bool bUsed_NominalYawAttitude)
	{
		if(nameSat[0] != 'G')
			return TYPE_GYM_UNKNOWN;
		map<string, GYM_MixedSat>::iterator it = m_mapGYMInfo.find(nameSat);
		if(it == m_mapGYMInfo.end())
		{
			printf("%s gpsACSYawAttitude失败.\n", nameSat.c_str());
			return TYPE_GYM_UNKNOWN;
		}
		POS3D sunPos;
		TDB t_TDB = TimeCoordConvert::GPST2TDB(t); // 获得 TDB 时间--提供太阳历参考时间									
		double jd = TimeCoordConvert::DayTime2JD(t_TDB); // 获得儒略日									
		double Pos[3];
		if(!m_JPLEphFile.getSunPos_Delay_EarthCenter(jd, Pos))
		{
			printf("gpsACSYawAttitude无法获得太阳位置.\n");
			return TYPE_GYM_UNKNOWN;
		}
		sunPos.x = Pos[0] * 1000; 
		sunPos.y = Pos[1] * 1000; 
		sunPos.z = Pos[2] * 1000; 
		POS6D    gpsPosVel;
		SP3Datum sp3Datum;
		// 使初始化后，m_sp3File为惯性系坐标
		if(!m_sp3File.getEphemeris(t, nameSat, sp3Datum))
		{
			printf("gpsACSYawAttitude无法获得GNSS卫星位置.\n");
			return TYPE_GYM_UNKNOWN;
		}
		gpsPosVel.setPos(sp3Datum.pos);
		gpsPosVel.setVel(sp3Datum.vel);	
		double b = 0.5;
		int sign_b = 1;
		double beta = getBetaSunAngle(sunPos, gpsPosVel);
		double u = getUOrbitAngle(sunPos, gpsPosVel);
		uOrbit = u;
		if(it->second.yawBiasFlag == 'P') // + 0.5
		{
			b = 0.5;
			sign_b = 1;
		}
		if(it->second.yawBiasFlag == 'N') // - 0.5
		{
			b = -0.5;
			sign_b = -1;
		}
		if(it->second.yawBiasFlag == 'Y') // nominal
		{
			if(beta >= 0)
			{
				b = -0.5;
				sign_b = -1;
			}
			else
			{
				b = 0.5;
				sign_b = 1;
			}
		}
		if(it->second.yawBiasFlag == 'A') // anti-nominal
		{
			if(beta < 0)
			{
				b = -0.5;
				sign_b = -1;
			}
			else
			{
				b = 0.5;
				sign_b = 1;
			}
		}
		if(it->second.yawBiasFlag == 'U') // no bias
		{
			b = 0.0;
			sign_b = 1;
		}
		nominalYawAttitude_GPS(it->second.nameBlock, sunPos, gpsPosVel, yaw, yawRate, betaSun, b);

		if(bUsed_NominalYawAttitude)
			return TYPE_GYM_YAWNOMINAL;
		
		//step1:位于noonTurn,或midnightTurn
		if(it->second.gpsNoonTurnList.size() > 0)
		{
			for(size_t s_i = 0; s_i < it->second.gpsNoonTurnList.size(); s_i ++)
			{
				GYM_G95_NoonTurnDatum  noonTurn = it->second.gpsNoonTurnList[s_i];
				if(noonTurn.ti - t <= 0 && noonTurn.te - t >= 0)
				{// 计算:卫星-地球-太阳张角SES
					POS3D unit_sun    = vectorNormal(sunPos);
					POS3D unit_gpspos = vectorNormal(gpsPosVel.getPos());
					double SES = acos(vectorDot(unit_sun, unit_gpspos));					
					int sign_beta = 1;
					if(betaSun < 0)
						sign_beta = -1;	
					if(SES > PI/2) //block IIR
						sign_beta = - sign_beta;
					yaw = noonTurn.yaw_ti - sign_beta * fabs(it->second.max_yawRate) * (t - noonTurn.ti);
					yawRate = noonTurn.yawRate_ti;
					return TYPE_GYM_NOONTURN;
				}
			}
		}
		//step2:位于ShadowCross
		if(it->second.gpsShadowCrossList.size() > 0)
		{
			for(size_t s_i = 0; s_i < it->second.gpsShadowCrossList.size(); s_i ++)
			{
				GYM_G95_ShadowCrossDatum shadowCross = it->second.gpsShadowCrossList[s_i];
				if(it->second.nameBlock.find(BLOCK_MaskString::BLOCK_IIF) != -1
				|| it->second.nameBlock.find(BLOCK_MaskString::BLOCK_III_A) != -1)
				{ // IIF 卫星： BLOCK_III_A
					yawRate = (shadowCross.yaw_te - shadowCross.yaw_ti)/(shadowCross.te - shadowCross.ti);
					if(shadowCross.ti - t <= 0 && shadowCross.te - t >= 0)
					{
						double t_ti = t - shadowCross.ti;
						yaw = shadowCross.yaw_ti + t_ti * yawRate;
						return TYPE_GYM_SHADOWCROSS;
					}
				}
				else
				{ // IIA 卫星
					if(shadowCross.ti - t <= 0 && shadowCross.te - t >= 0)
					{
						if(t - shadowCross.t1 < 0)
						{
							yaw = shadowCross.yaw_ti + shadowCross.yawRate_ti * (t - shadowCross.ti) + 0.5 * sign_b * it->second.max_yawRateRate * (t - shadowCross.ti) * (t - shadowCross.ti);
							yawRate = shadowCross.yawRate_ti + sign_b * it->second.max_yawRateRate * (t - shadowCross.ti);
							return TYPE_GYM_SHADOWCROSS;
						}
						else
						{
							double t1 = shadowCross.t1 - shadowCross.ti;						
							yaw  = shadowCross.yaw_ti + shadowCross.yawRate_ti * t1 + 0.5 * sign_b * it->second.max_yawRateRate * t1 * t1
								 + sign_b * it->second.max_yawRate * (t - shadowCross.t1);
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
							return TYPE_GYM_SHADOWCROSS;
						}
					}
				}
			}
		}
		//step3:位于ShadowPost
		if(it->second.gpsShadowPostList.size() > 0)
		{
			for(size_t s_i = 0; s_i < it->second.gpsShadowPostList.size(); s_i ++)
			{
				GYM_G95_ShadowPostDatum    shadowPost = it->second.gpsShadowPostList[s_i];
				int sign_D = 1;
				double D = yaw - it->second.gpsShadowCrossList[s_i].yaw_te - floor(( yaw - it->second.gpsShadowCrossList[s_i].yaw_te)/360 + 0.5) * 360;
				if(D < 0)
					sign_D = -1;
				if(shadowPost.ti - t <= 0 && shadowPost.te - t >= 0)
				{
					if(t - shadowPost.t1 < 0)
					{						
						yaw = shadowPost.yaw_ti + sign_b * it->second.max_yawRate * (t- shadowPost.ti) + 0.5 * sign_D * it->second.max_yawRateRate * pow((t - shadowPost.ti),2);		
						yawRate = sign_b * it->second.max_yawRate + sign_D * it->second.max_yawRateRate;
						return TYPE_GYM_SHADOWPOST;
					}
					else
					{ 
						yaw = shadowPost.yaw_ti + sign_b * it->second.max_yawRate * (shadowPost.t1 - shadowPost.ti) + 0.5 * sign_D * it->second.max_yawRateRate * pow((shadowPost.t1 - shadowPost.ti),2)
						      + sign_D * it->second.max_yawRate * (t - shadowPost.t1);
						yawRate = sign_D * it->second.max_yawRate;
						return TYPE_GYM_SHADOWPOST;
					}
				}
			}
		}
		//step4:其余情况，为 nominal yaw
		return TYPE_GYM_YAWNOMINAL;
	}
	
	// 子程序名称： glonassACSYawAttitude   
	// 功能：姿态控制系统 attitude control subsystem (ACS) 偏航姿态角及其变化率
	// 变量类型：nameSat                  : 卫星名称
	//           t                        : 当前时间
	//           yaw                      : 偏航姿态角, 单位：度
	//           yawRate                  : 偏航姿态角变化率
	//           betaSun                  : 太阳入射方向与轨道面夹角β,单位：度
	//           uOrbit                   : 轨道角β,单位：度
	//           bUsed_NominalYawAttitude : 是否使用标称偏航姿态角及其变化率 
	// 输入：nameSat, t
	// 输出：yaw, yawRate
	// 语言：C++
	// 创建者：谷德峰, 刘俊宏
	// 创建时间：2014/11/5
	// 版本时间：
	// 修改记录：1、增加轨道角计算，邵凯，2019/8/31
	// 备注： 
	int GNSSYawAttitudeModel::glonassACSYawAttitude(string nameSat, GPST t, double& yaw, double &yawRate, double &betaSun, double &uOrbit, bool bUsed_NominalYawAttitude)
	{
		if(nameSat[0] != 'R')
			return TYPE_GYM_UNKNOWN;
		map<string, GYM_MixedSat>::iterator it = m_mapGYMInfo.find(nameSat);
		if(it == m_mapGYMInfo.end())
		{
			printf("%s glonassACSYawAttitude失败.\n", nameSat.c_str());
			return TYPE_GYM_UNKNOWN;
		}
		POS3D sunPos;
		TDB t_TDB = TimeCoordConvert::GPST2TDB(t); // 获得 TDB 时间--提供太阳历参考时间									
		double jd = TimeCoordConvert::DayTime2JD(t_TDB); // 获得儒略日									
		double Pos[3];
		if(!m_JPLEphFile.getSunPos_Delay_EarthCenter(jd, Pos))
		{
			printf("gpsACSYawAttitude无法获得太阳位置.\n");
			return TYPE_GYM_UNKNOWN;
		}
		sunPos.x = Pos[0] * 1000; 
		sunPos.y = Pos[1] * 1000; 
		sunPos.z = Pos[2] * 1000; 
		POS6D    gpsPosVel;
		SP3Datum sp3Datum;
		// 使初始化后，m_sp3File为惯性系坐标
		if(!m_sp3File.getEphemeris(t,nameSat,sp3Datum))
		{
			return TYPE_GYM_UNKNOWN;
		}
		gpsPosVel.setPos(sp3Datum.pos);
		gpsPosVel.setVel(sp3Datum.vel);	

		double beta = getBetaSunAngle(sunPos, gpsPosVel);
		double u = getUOrbitAngle(sunPos, gpsPosVel);
		uOrbit = u;
		nominalYawAttitude_GLONASS(it->second.nameBlock, sunPos, gpsPosVel,yaw, yawRate, betaSun);
		if(bUsed_NominalYawAttitude)
			return TYPE_GYM_YAWNOMINAL;
		//step1:位于noonTurn,或midnightTurn
		if(it->second.glonassNoonTurnList.size() > 0)
		{
			for(size_t s_i = 0; s_i < it->second.glonassNoonTurnList.size(); s_i ++)
			{
				GYM_R11_NoonTurnDatum  noonTurn = it->second.glonassNoonTurnList[s_i];
				if(noonTurn.ti - t <= 0 && noonTurn.te - t >= 0)
				{// 计算:卫星-地球-太阳张角SES
					POS3D unit_sun    = vectorNormal(sunPos);
					POS3D unit_gpspos = vectorNormal(gpsPosVel.getPos());
					double SES = acos(vectorDot(unit_sun, unit_gpspos));					
					int sign_beta = 1;
					if(betaSun < 0)
						sign_beta = -1;	
					yaw = noonTurn.yaw_ti - sign_beta * fabs(it->second.max_yawRate) * (t - noonTurn.ti);
					yawRate = noonTurn.yawRate_ti;
					return TYPE_GYM_NOONTURN;
				}
			}
		}
		//step2:位于ShadowCross
		if(it->second.glonassShadowCrossList.size() > 0)
		{
			for(size_t s_i = 0; s_i < it->second.glonassShadowCrossList.size(); s_i ++)
			{
				GYM_R11_ShadowCrossDatum shadowCross = it->second.glonassShadowCrossList[s_i];
				if(shadowCross.ti - t <= 0 && shadowCross.t1 - t >= 0) // [ti t1]
				{
					int sign_yawRate = 1;
					if(shadowCross.yawRate_ti < 0)
						sign_yawRate = -1;
					double t_ti = t - shadowCross.ti;
					yawRate = it->second.max_yawRate;
					yaw = shadowCross.yaw_ti + sign_yawRate * t_ti * yawRate;
					return TYPE_GYM_SHADOWCROSS;
				}
				else if(shadowCross.t1 - t <= 0 && shadowCross.te - t >= 0) // [t1 te]
				{
					yawRate = shadowCross.yawRate_te;
					yaw = shadowCross.yaw_te;
					return TYPE_GYM_SHADOWCROSS;
				}
			}
		}
		//step4:其余情况，为 nominal yaw
		return TYPE_GYM_YAWNOMINAL;
	}

	// 子程序名称： galileoACSYawAttitude   
	// 功能：姿态控制系统 attitude control subsystem (ACS) 偏航姿态角及其变化率
	// 变量类型：nameSat                  : 卫星名称
	//           t                        : 当前时间
	//           yaw                      : 偏航姿态角, 单位：度
	//           yawRate                  : 偏航姿态角变化率
	//           betaSun                  : 太阳入射方向与轨道面夹角β,单位：度
	//           uOrbit                   : 轨道角β,单位：度
	//           bUsed_NominalYawAttitude : 是否使用标称偏航姿态角及其变化率 
	// 输入：nameSat, t
	// 输出：yaw, yawRate
	// 语言：C++
	// 创建者：邵凯
	// 创建时间：2019/9/9
	// 版本时间：
	// 修改记录：
	// 备注： 
	int GNSSYawAttitudeModel::galileoACSYawAttitude(string nameSat, GPST t, double& yaw, double &yawRate, double &betaSun, double &uOrbit, bool bUsed_NominalYawAttitude)
	{
		if(nameSat[0] != 'E')
			return TYPE_GYM_UNKNOWN;
		map<string, GYM_MixedSat>::iterator it = m_mapGYMInfo.find(nameSat);
		if(it == m_mapGYMInfo.end())
		{
			printf("%s galileoACSYawAttitude失败.\n", nameSat.c_str());
			return TYPE_GYM_UNKNOWN;
		}
		POS3D sunPos;
		TDB t_TDB = TimeCoordConvert::GPST2TDB(t); // 获得 TDB 时间--提供太阳历参考时间									
		double jd = TimeCoordConvert::DayTime2JD(t_TDB); // 获得儒略日									
		double Pos[3];
		if(!m_JPLEphFile.getSunPos_Delay_EarthCenter(jd, Pos))
		{
			printf("gpsACSYawAttitude无法获得太阳位置.\n");
			return TYPE_GYM_UNKNOWN;
		}
		sunPos.x = Pos[0] * 1000; 
		sunPos.y = Pos[1] * 1000; 
		sunPos.z = Pos[2] * 1000; 
		POS6D    gpsPosVel;
		SP3Datum sp3Datum;
		// 使初始化后，m_sp3File为惯性系坐标
		if(!m_sp3File.getEphemeris(t,nameSat,sp3Datum))
		{
			return TYPE_GYM_UNKNOWN;
		}
		gpsPosVel.setPos(sp3Datum.pos);
		gpsPosVel.setVel(sp3Datum.vel);	

		double beta = getBetaSunAngle(sunPos, gpsPosVel);
		beta = beta*PI/180;
		double u = getUOrbitAngle_perihelion(sunPos, gpsPosVel);
		uOrbit = u;
		u = u *PI/180;
		nominalYawAttitude_GALILEO(it->second.nameBlock, sunPos, gpsPosVel, yaw, yawRate, betaSun);
		// 利用卫星-地球-太阳张角SES代替轨道角u进行计算
		double Sx = -sin(u)*cos(beta);
		double Sy = -sin(beta);
		double Sz = -cos(u)*cos(beta);
		double beta_x = 15 * PI/180; // 弧度
		double beta_y = 2 * PI/180;
		if(bUsed_NominalYawAttitude)
			return TYPE_GYM_YAWNOMINAL;
		//step1:位于noonTurn,或midnightTurn
		if(it->second.galileoNoonTurnList.size() > 0)
		{
			for(size_t s_i = 0; s_i < it->second.galileoNoonTurnList.size(); s_i ++)
			{
				GYM_E08_NoonTurnDatum  noonTurn = it->second.galileoNoonTurnList[s_i];
				if(noonTurn.ti - t <= 0 && noonTurn.te - t >= 0)
				{
					// 使用平滑后的Shy代替Sy
					double Shy = 0.5*noonTurn.sign_ti*sin(beta_y)+0.5*Sy+0.5*cos(PI*fabs(Sx)/sin(beta_x))*(noonTurn.sign_ti*sin(beta_y)-Sy);
					yaw = atan2(Shy, Sx);
					yaw = yaw * 180.0 / PI;		
					yawRate = noonTurn.yawRate_ti;
					return TYPE_GYM_NOONTURN;
				}
			}
		}
		//step2:位于ShadowCross
		if(it->second.galileoShadowCrossList.size() > 0)
		{
			for(size_t s_i = 0; s_i < it->second.galileoShadowCrossList.size(); s_i ++)
			{
				GYM_E08_ShadowCrossDatum shadowCross = it->second.galileoShadowCrossList[s_i];
				if(shadowCross.ti - t <= 0 && shadowCross.te - t >= 0) // [ti te]
				{
					// 使用平滑后的Shy代替Sy
					double Shy = 0.5*shadowCross.sign_ti*sin(beta_y)+0.5*Sy+0.5*cos(PI*fabs(Sx)/sin(beta_x))*(shadowCross.sign_ti*sin(beta_y)-Sy);
					yaw = atan2(Shy, Sx);
					yaw = yaw * 180.0 / PI;		
					yawRate = shadowCross.yawRate_ti;
					return TYPE_GYM_SHADOWCROSS;
				}
			}
		}
		//step4:其余情况，为 nominal yaw
		return TYPE_GYM_YAWNOMINAL;
	}
	// 子程序名称： bdsYawAttitude   
	// 功能：姿态控制系统 attitude control subsystem 偏航姿态角及其变化率
	// 变量类型：nameSat                  : 卫星名称
	//           t                        : 当前时间
	//           yaw                      : 偏航姿态角, 单位：度
	//           yawRate                  : 偏航姿态角变化率
	//           beta                     : 太阳入射方向与轨道面夹角β,单位：度
	//           uOrbit                   : 轨道角β,单位：度
	//           bUsed_NominalYawAttitude : 是否使用标称偏航姿态角及其变化率 
	// 输入：nameSat, t
	// 输出：yaw, yawRate
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2017/8/30
	// 版本时间：
	// 修改记录：
	// 备注：
	int GNSSYawAttitudeModel::bdsYawAttitude(string nameSat, GPST t, double &yaw, double &yawRate, double &betaSun, double &uOrbit, bool bUsed_NominalYawAttitude)
	{
		if(nameSat[0] != 'C')
			return TYPE_GYM_UNKNOWN;
		map<string, GYM_MixedSat>::iterator it = m_mapGYMInfo.find(nameSat);
		if(it == m_mapGYMInfo.end())
		{
			printf("%s gpsACSYawAttitude失败.\n", nameSat.c_str());
			return TYPE_GYM_UNKNOWN;
		}
		
		POS3D sunPos;
		TDB t_TDB = TimeCoordConvert::GPST2TDB(t); // 获得 TDB 时间--提供太阳历参考时间									
		double jd = TimeCoordConvert::DayTime2JD(t_TDB); // 获得儒略日									
		double Pos[3];
		if(!m_JPLEphFile.getSunPos_Delay_EarthCenter(jd, Pos))
		{
			printf("gpsACSYawAttitude无法获得太阳位置.\n");
			return TYPE_GYM_UNKNOWN;
		}
		sunPos.x = Pos[0] * 1000; 
		sunPos.y = Pos[1] * 1000; 
		sunPos.z = Pos[2] * 1000; 
		POS6D    bdsPosVel;
		SP3Datum sp3Datum;
		if(!m_sp3File.getEphemeris(t,nameSat,sp3Datum))
		{
			return TYPE_GYM_UNKNOWN;
		}
		bdsPosVel.setPos(sp3Datum.pos);
		bdsPosVel.setVel(sp3Datum.vel);

		if(it->second.nameBlock.find(BLOCK_MaskString::BEIDOU_2G) != -1
		|| it->second.nameBlock.find(BLOCK_MaskString::BEIDOU_3G) != -1
		|| it->second.nameBlock.find(BLOCK_MaskString::BEIDOU_3G_CAST) != -1)
		{
			yaw = 0.0;
			yawRate = 0.0;
            betaSun = getBetaSunAngle(sunPos, bdsPosVel);
			return TYPE_GYM_ORBNORMAL;
		}
		double u = getUOrbitAngle(sunPos, bdsPosVel);
		uOrbit = u;
		nominalYawAttitude_BDS(it->second.nameBlock, sunPos, bdsPosVel, yaw, yawRate, betaSun);

		if(bUsed_NominalYawAttitude)
			return TYPE_GYM_YAWNOMINAL;

		if(it->second.bdsYawNominalEntryList.size() == 0)
		{
			if(it->second.bdsOrbNormalEntryList.size() == 0)
			{
				// orbit normal: 整个区间没有机动, 判断当前点beta, 如果在4度以内, 即整个弧段都处于零偏
				if(fabs(betaSun) <= it->second.min_betaBDSYaw2Orb)
				{
					yaw = 0.0;
					yawRate = 0.0;
					return TYPE_GYM_ORBNORMAL;
				}
				// yaw nominal: 整个区间没有机动, 判断当前点beta, 如果在4度以外, 即整个弧段都处于动偏
				return TYPE_GYM_YAWNOMINAL;
			}
			//  由于没有 YawNominalEntry, OrbNormalEntry 至多也只出现1次
			else
			{
				// orbit normal: 如果当前点在OrbNormalEntry之后,则处于零偏
				if(t - it->second.bdsOrbNormalEntryList[0].t0 >= 0)
				{
					yaw = 0.0;
					yawRate = 0.0;
					return TYPE_GYM_ORBNORMAL;
				}
				// yaw nominal: 如果当前点在OrbNormalEntry之前,则处于动偏
				return TYPE_GYM_YAWNOMINAL;
			}
		}
		else
		{
			// 由于没有 OrbNormalEntry, YawNominalEntry 至多也只出现1次
			if(it->second.bdsOrbNormalEntryList.size() == 0)
			{
				// orbit normal: 如果当前点在YawNominalEntry之前,则处于零偏
				if(t - it->second.bdsYawNominalEntryList[0].t0 < 0)
				{
					yaw = 0.0;
					yawRate = 0.0;
					return TYPE_GYM_ORBNORMAL;
				}
				// yaw nominal: 如果当前点在YawNominalEntry之后,则处于动偏
				return TYPE_GYM_YAWNOMINAL;
			}
			else 
			{
				// 寻找bdsYawNominalEntryList左端点
				int i0_bdsYawNominalEntry = -1;
				for(size_t s_i = 0; s_i < it->second.bdsYawNominalEntryList.size(); s_i++)
				{
					if(t - it->second.bdsYawNominalEntryList[s_i].t0 >= 0)
					{
						i0_bdsYawNominalEntry = int(s_i); // 左端点
					}
				}
				// 由于右侧是 YawNominalEntry, 前面的 OrbNormalEntry 不可能连续出现, 找到一个即可
				if(i0_bdsYawNominalEntry == -1)
				{
					int i0_bdsOrbNormalEntry = -1;
					for(size_t s_i = 0; s_i < it->second.bdsOrbNormalEntryList.size(); s_i++)
					{
						if(it->second.bdsOrbNormalEntryList[s_i].t0 - it->second.bdsYawNominalEntryList[0].t0 > 0)
							continue; // 在bdsYawNominalEntryList首点右侧的均不考虑
						i0_bdsOrbNormalEntry = int(s_i);
						// yaw nominal: 如果当前点在OrbNormalEntry之前,则处于动偏
						if(t - it->second.bdsOrbNormalEntryList[s_i].t0 < 0)  
						{
							return TYPE_GYM_YAWNOMINAL;
						}
						// orbit normal: 如果当前点在OrbNormalEntry之后,则处于零偏
						else
						{
							yaw = 0.0;
							yawRate = 0.0;
							return TYPE_GYM_ORBNORMAL;
						}
					}
					if(i0_bdsOrbNormalEntry == -1)
					{// orbit normal: 未找到OrbNormalEntry, 由于未进入YawNominalEntry, 则处于零偏
						yaw = 0.0;
						yawRate = 0.0;
						return TYPE_GYM_ORBNORMAL;
					}
				}
				// 由于左侧是 YawNominalEntry, 前面的 OrbNormalEntry 不可能连续出现, 找到一个即可
				else
				{
					int i0_bdsOrbNormalEntry = -1;
					for(size_t s_i = 0; s_i < it->second.bdsOrbNormalEntryList.size(); s_i++)
					{
						if(it->second.bdsOrbNormalEntryList[s_i].t0 - it->second.bdsYawNominalEntryList[i0_bdsYawNominalEntry].t0 < 0)
							continue;// 在bdsYawNominalEntryList当前点左侧的均不考虑
						i0_bdsOrbNormalEntry = int(s_i);
						// yaw nominal: 如果当前点在OrbNormalEntry之前,则处于动偏
						if(t - it->second.bdsOrbNormalEntryList[s_i].t0 < 0)
						{
							return TYPE_GYM_YAWNOMINAL;
						}
						// orbit normal: 如果当前点在OrbNormalEntry之后,则处于零偏
						else
						{
							yaw = 0.0;
							yawRate = 0.0;
							return TYPE_GYM_ORBNORMAL;
						}
					}
					if(i0_bdsOrbNormalEntry == -1)
					{// yaw nominal: 未找到OrbNormalEntry, 由于已完成YawNominalEntry, 则处于动偏
						return TYPE_GYM_YAWNOMINAL;
					}
				}
			}
		}
		return TYPE_GYM_YAWNOMINAL;
	}
	
	// 子程序名称： bdsYawAttitude_continuous   
	// 功能：姿态控制系统 attitude control subsystem 连续偏航姿态角及其变化率
	// 变量类型：nameSat                  : 卫星名称
	//           t                        : 当前时间
	//           yaw                      : 偏航姿态角, 单位：度
	//           yawRate                  : 偏航姿态角变化率
	//           beta                     : 太阳入射方向与轨道面夹角β,单位：弧度
	//           uOrbit                   : 轨道角β,单位：度
	// 输入：nameSat, t
	// 输出：yaw, yawRate
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2017/8/30
	// 版本时间：
	// 修改记录：添加北斗三号和北斗二号C06\C13\C14连续动偏模型，张厚喆，2021/04/01，C13和C14的noonTurn.yaw_ti在init进行常值偏差修正，防止姿控反向
	// 备注：
	int GNSSYawAttitudeModel::bdsYawAttitude_continuous(string nameSat, GPST t, double &yaw, double &yawRate, double &betaSun, double &uOrbit)
	{
		if(nameSat[0] != 'C')
			return TYPE_GYM_UNKNOWN;
		map<string, GYM_MixedSat>::iterator it = m_mapGYMInfo.find(nameSat);
		if(it == m_mapGYMInfo.end())
		{
			printf("%s bdsACSYawAttitude_continuous失败.\n", nameSat.c_str());
			return TYPE_GYM_UNKNOWN;
		}
		
		POS3D sunPos;
		TDB t_TDB = TimeCoordConvert::GPST2TDB(t); // 获得 TDB 时间--提供太阳历参考时间									
		double jd = TimeCoordConvert::DayTime2JD(t_TDB); // 获得儒略日									
		double Pos[3];
		if(!m_JPLEphFile.getSunPos_Delay_EarthCenter(jd, Pos))
		{
			printf("bdsACSYawAttitude_continuous无法获得太阳位置.\n");
			return TYPE_GYM_UNKNOWN;
		}
		sunPos.x = Pos[0] * 1000; 
		sunPos.y = Pos[1] * 1000; 
		sunPos.z = Pos[2] * 1000; 
		POS6D    bdsPosVel;
		SP3Datum sp3Datum;
		if(!m_sp3File.getEphemeris(t,nameSat,sp3Datum))
		{
			return TYPE_GYM_UNKNOWN;
		}
		bdsPosVel.setPos(sp3Datum.pos);
		bdsPosVel.setVel(sp3Datum.vel);

		if(it->second.nameBlock.find(BLOCK_MaskString::BEIDOU_3G) != -1)
		{
			yaw = 0.0;
			yawRate = 0.0;
            betaSun = getBetaSunAngle(sunPos, bdsPosVel);
			return TYPE_GYM_ORBNORMAL;
		}
		double u = getUOrbitAngle(sunPos, bdsPosVel);
		uOrbit = u;
		nominalYawAttitude_BDS(it->second.nameBlock, sunPos, bdsPosVel, yaw, yawRate, betaSun);

		//if(bUsed_NominalYawAttitude)
		//	return TYPE_GYM_YAWNOMINAL;
		//step1:位于noonTurn,或midnightTurn
		if(it->second.bdsNoonTurnList.size() > 0)
		{
			double t_max = 5740.0;//IGSO
			if(it->second.nameBlock.find(BLOCK_MaskString::BEIDOU_2M) != -1 
				|| it->second.nameBlock.find(BLOCK_MaskString::BEIDOU_3M) != -1 
				|| it->second.nameBlock.find(BLOCK_MaskString::BEIDOU_3M_CAST) != -1 
				|| it->second.nameBlock.find(BLOCK_MaskString::BEIDOU_3M_SECM_A) != -1 
				|| it->second.nameBlock.find(BLOCK_MaskString::BEIDOU_3M_SECM_B) != -1 )
				t_max = 3090.0;
			for(size_t s_i = 0; s_i < it->second.bdsNoonTurnList.size(); s_i ++)
			{
				GYM_C15_NoonTurnDatum  noonTurn = it->second.bdsNoonTurnList[s_i];
				if(noonTurn.ti - t <= 0 && noonTurn.te - t >= 0)
				{
					if(noonTurn.u_ti > 0 && u < 0)//正午机动，当前历元轨道角超过180度
						u = 360 + u;
					yaw = 90.0 * noonTurn.sign_ti + (noonTurn.yaw_ti - 90.0 * noonTurn.sign_ti) * cos(2.0 * PI * (u - noonTurn.u_ti)/(t_max * noonTurn.uRate_ti));
					//printf("%s %s %8.2f %8.2f %8.2f %10.8f\n",it->first.c_str(), t.toString().c_str(), noonTurn.yaw_ti, u, noonTurn.u_ti, noonTurn.uRate_ti);
					yawRate = noonTurn.yawRate_ti;
					return TYPE_GYM_NOONTURN;
				}
			}
		}
		return TYPE_GYM_YAWNOMINAL;
	}
}