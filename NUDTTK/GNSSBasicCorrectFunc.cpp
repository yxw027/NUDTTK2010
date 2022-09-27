#include "GNSSBasicCorrectFunc.hpp"
#include "Matrix.hpp"
#include "constDef.hpp"
#include "TimeCoordConvert.hpp"
#include "RuningInfoFile.hpp"
#include "MathAlgorithm.hpp"

namespace NUDTTK
{
	GNSSBasicCorrectFunc::GNSSBasicCorrectFunc(void)
	{
	}

	GNSSBasicCorrectFunc::~GNSSBasicCorrectFunc(void)
	{
	}

	// 子程序名称： correctSp3EarthRotation   
	// 功能：根据信号传播时间, 对 GPS 卫星的精密星历进行地球自转修正 (当采用ITRF坐标系时需要)
	// 变量类型：delay           : 信号传播延迟时间( > 0), 单位: 秒
	//           sp3Datum        : GPS卫星星历
	// 输入：sp3Datum, delay
	// 输出：sp3Datum
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/02/09
	// 版本时间：
	// 修改记录：
	// 备注： 
	void GNSSBasicCorrectFunc::correctSp3EarthRotation(double delay, SP3Datum& sp3Datum)
	{
		// 将发射时刻 k - 1 的坐标系转换到接收时刻 k 的坐标系
		// 地球本身自西向东旋转(绕 Z 轴逆时针旋转), 坐标系逆时针旋转 delay * EARTH_W(弧度), 相当于坐标旋转 - delay * EARTH_W
		Matrix matRotate = TimeCoordConvert::rotate(delay * EARTH_W, 3);
		Matrix matPos(3,1);
		matPos.SetElement(0, 0, sp3Datum.pos.x);
		matPos.SetElement(1, 0, sp3Datum.pos.y);
		matPos.SetElement(2, 0, sp3Datum.pos.z);
		Matrix matVel(3,1);
		matVel.SetElement(0, 0, sp3Datum.vel.x);
		matVel.SetElement(1, 0, sp3Datum.vel.y);
		matVel.SetElement(2, 0, sp3Datum.vel.z);
		// 由于地固系的旋转近似匀速的, 因此两个时刻的坐标系间相对没有旋转
		matPos = matRotate * matPos;
		matVel = matRotate * matVel;
		sp3Datum.pos.x = matPos.GetElement(0,0);
		sp3Datum.pos.y = matPos.GetElement(1,0);
		sp3Datum.pos.z = matPos.GetElement(2,0);
		sp3Datum.vel.x = matVel.GetElement(0,0);
		sp3Datum.vel.y = matVel.GetElement(1,0);
		sp3Datum.vel.z = matVel.GetElement(2,0);
	}

	// 子程序名称： correctLeoAntPCO_J2000   
	// 功能：对 LEO 卫星的天线相位中心偏移进行修正(J2000惯性系下)
	// 变量类型：pcoAnt          : LEO卫星的天线偏移(星固系)
	//           posLeo          : LEO卫星位置, [J2000惯性系]
	//           velLeo          : LEO卫星速度, [J2000惯性系]
	//           offsetJ2000     : LEO卫星的天线偏移改正量, [J2000惯性系]
	// 输入：pcoAnt, posLeo, velLeo
	// 输出：offsetJ2000
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/07/13
	// 版本时间：
	// 修改记录：
	// 备注： 
	POS3D GNSSBasicCorrectFunc::correctLeoAntPCO_J2000(POS3D pcoAnt, POS3D posLeo, POS3D velLeo)
	{
        POS3D ex  = vectorNormal(velLeo);
		POS3D ez  = vectorNormal(posLeo *(-1.0));   // 飞行方向     
		POS3D ey;
		vectorCross(ey, ez, ex); // 天底方向 x 飞行方向
		ey = vectorNormal(ey);                        
		vectorCross(ex, ey, ez); // 右手系 
		ex = vectorNormal(ex);
		POS3D offsetJ2000 = ex * pcoAnt.x + ey * pcoAnt.y + ez * pcoAnt.z;
		return offsetJ2000;
	}

	// 子程序名称： correctLeoAntPCO_ECEF   
	// 功能：对 LEO 卫星的天线相位中心偏移进行修正(地固系下), 考虑到了“轨道坐标系的定义是与 J2000 惯性系捆绑的”的影响
	// 变量类型：t               : 轨道时间, 用于计算轨道系
    //           pcoAnt          : LEO卫星的天线偏移(星固系)
	//           posLeo          : LEO卫星位置, [地固系]
	//           velLeo          : LEO卫星速度, [地固系]
	//           offsetECEF      : LEO卫星的天线偏移改正量, [地固系]
	// 输入：t, pcoAnt, posLeo, velLeo
	// 输出：offsetECEF
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2009/12/30
	// 版本时间：
	// 修改记录：
	// 备注： 
	POS3D  GNSSBasicCorrectFunc::correctLeoAntPCO_ECEF(UT1 t, POS3D pcoAnt, POS3D posLeo, POS3D velLeo)
	{
		POS6D posvelLeo;
		posvelLeo.setPos(posLeo);
		posvelLeo.setVel(velLeo);
		POS3D S_R;
		POS3D S_T;
		POS3D S_N;
		TimeCoordConvert::getCoordinateRTNAxisVector(t, posvelLeo, S_R, S_T, S_N);
		POS3D  ex = S_T;           // 飞行方向
		POS3D  ey = S_N *(-1.0);   // 天底方向 x 飞行方向
		POS3D  ez = S_R *(-1.0);   // 天底方向
		POS3D offsetECEF = ex * pcoAnt.x + ey * pcoAnt.y + ez * pcoAnt.z;
		return offsetECEF;
	}

	// 子程序名称： correctGPSAntPCO   
	// 功能：对GPS卫星的天线相位中心偏移进行修正
	// 变量类型：id_Block           : GPS卫星类型
    //           gpsPCO             : GPS卫星的天线偏移(星固系)
	//           receiverPos        : 接收机位置
	//           gpsPos             : GPS卫星位置
	//           sunPos             : 太阳位置
	// 输入：id_Block, gpsPCO, receiverPos, gpsPos, sunPos
	// 输出：correctdistance
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2007/07/08
	// 版本时间：
	// 修改记录：
	// 备注： 
	double GNSSBasicCorrectFunc::correctGPSAntPCO(int id_Block, POS3D gpsPCO, POS3D receiverPos, POS3D gpsPos, POS3D sunPos)
	{
		// 计算GPS卫星星体系
		double correctdistance = 0;
		POS3D  vecRs =  sunPos - gpsPos; // 太阳矢量, 由GPS卫星指向太阳     
		POS3D ez =  vectorNormal(gpsPos) * (-1.0);
		POS3D ns =  vectorNormal(vecRs);
		POS3D ey;
		vectorCross(ey, ez, ns); 
		ey = vectorNormal(ey);
		POS3D ex;
		vectorCross(ex, ey, ez);
		ex = vectorNormal(ex);
		if(id_Block >= 4 && id_Block <= 6)
		{
			ex = ex * (-1.0);
			ey = ey * (-1.0);	
		}
		POS3D d = ex * (gpsPCO.x) + ey * gpsPCO.y + ez * gpsPCO.z;
        POS3D vecLos = vectorNormal(receiverPos - gpsPos);
		correctdistance = vectorDot(vecLos, d); // 将 GPS 天线相位中心偏移矢量 d 向视线矢量 vecLook 投影
		return correctdistance;
	}

	// 子程序名称： correctLeoAntPCO_YawAttitudeModel   
	// 功能：对卫星的天线相位中心偏移进行修正(采用偏航姿态控制)
	// 变量类型：pcoAnt       :  天线偏移(星固系)
	//           posLeo       :  卫星在地心系下位置速度
	//           posSun       :  太阳位置
	// 输入：pcoAnt, posLeo, posSun
	// 输出：offset
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2013/12/04
	// 版本时间：
	// 修改记录：
	// 备注： 
	POS3D GNSSBasicCorrectFunc::correctLeoAntPCO_YawAttitudeModel(POS3D pcoAnt, POS3D posLeo, POS3D posSun)
	{
		POS3D  vecRs =  posSun - posLeo; // 太阳矢量, 由卫星指向太阳     
		POS3D ez =  vectorNormal(posLeo) * (-1.0);
		POS3D ns =  vectorNormal(vecRs);
		POS3D ey;
		vectorCross(ey, ez, ns); 
		ey = vectorNormal(ey);
		POS3D ex;
		vectorCross(ex, ey, ez);
		ex = vectorNormal(ex);
		POS3D offset = ex * pcoAnt.x + ey * pcoAnt.y + ez * pcoAnt.z;
		return offset;
	}

	// 子程序名称： correctPhaseWindUp   
	// 功能：对相位缠绕进行修正
	// 变量类型：id_Block           : GPS卫星类型
	//           receiverPos        : 接收机位置速度
	//           unitXr             : north unit vector at receiver（地面测站接收机）
	//           unitYr             : west unit vector at receiver（地面测站接收机）
	//           gpsPos             : GPS卫星位置
	//           sunPos             : 太阳位置
	//           prev               : 同一弧段, 前一时刻修正相位值, (如果为DBL_MAX, 说明当前时刻为起点)
	//           windup             : 同一弧段, 当前时刻修正相位值, 返回值, [-1, 1]
	// 输入：id_Block, gpsPCO, receiverPos, gpsPos, sunPos
	// 输出：windup
	// 语言：C++
	// 创建者：涂佳, 谷德峰
	// 创建时间：2012/05/01
	// 版本时间：
	// 修改记录：
	// 备注： 参考GPSTK进行了连续化
	double GNSSBasicCorrectFunc::correctPhaseWindUp(int id_Block, POS3D receiverPos, POS3D unitXr, POS3D unitYr, POS3D gpsPos, POS3D sunPos, double prev)
	{
		// 计算GPS卫星星体系
		double windup = 0;
		POS3D  vecRs =  sunPos - gpsPos; // 太阳矢量, 由 GPS 卫星指向太阳     
		POS3D ez =  vectorNormal(gpsPos) * (-1.0);
		POS3D ns =  vectorNormal(vecRs);
		POS3D ey;
		vectorCross(ey, ez, ns); 
		ey = vectorNormal(ey);
		POS3D ex;
		vectorCross(ex, ey, ez);
		ex = vectorNormal(ex);
		if(id_Block >= 4 && id_Block <= 6)
		{// BLK: 1=Blk I  2=Blk II 3=Blk IIA 4=Blk IIR-A 5=Blk IIR-B 6=Blk IIR-M 7=Blk IIF
			ex = ex * (-1.0);
			ey = ey * (-1.0);	
		}
		/*if(vectorDot(vecRs, ex) < 0)
		{
			ex = ex * (-1.0);
			ey = ey * (-1.0);	
		}*/
		POS3D vecLos = vectorNormal(receiverPos - gpsPos);
		POS3D k_ey_GPS;
		POS3D k_ey_Rec;
		vectorCross(k_ey_GPS, vecLos, ey);
		vectorCross(k_ey_Rec, vecLos, unitYr); 
		POS3D D_R = unitXr  - vecLos * (unitXr.x * vecLos.x + unitXr.y * vecLos.y + unitXr.z * vecLos.z) + k_ey_Rec;
		POS3D D_T = ex - vecLos * (ex.x * vecLos.x + ex.y * vecLos.y + ex.z * vecLos.z) - k_ey_GPS;
		POS3D DD;
		vectorCross(DD, D_T, D_R);
		double theta = vectorDot(vecLos, DD);
		int sign = 1;
		/*if(theta > 0)
			sign = 1;
		if(theta == 0)
			sign = 0;*/
		if(theta < 0)
			sign = -1;
		D_T = vectorNormal(D_T);
		D_R = vectorNormal(D_R);
		double cos_fai = D_T.x * D_R.x + D_T.y * D_R.y + D_T.z * D_R.z;
		cos_fai = cos_fai >  1 ?  1 : cos_fai; // 2013/07/04, 确保 -1 <= cos_fai <= 1
		cos_fai = cos_fai < -1 ? -1 : cos_fai;
		windup = sign * acos(cos_fai) / (2 * PI);
		if(prev != DBL_MAX && prev != 0.0)
		{// 连续化, 20140427
			double d = windup - prev;
			windup -= int(d + (d < 0.0 ? -0.5 : 0.5));
		}
		//correctdistance = windup * SPEED_LIGHT / ((GPS_FREQUENCE_L1 + GPS_FREQUENCE_L2));
		return windup;
	}

	// 子程序名称： correctLeoAntPhaseWindUp   
	// 功能：对低轨卫星的天线相位缠绕进行修正
	// 变量类型：id_Block           : GPS卫星类型
	//           receiverPosVel     : 接收机位置速度
	//           gpsPos             : GPS卫星位置
	//           sunPos             : 太阳位置
	// 输入：id_Block, gpsPCO, receiverPos, gpsPos, sunPos
	// 输出：correctdistance
	// 语言：C++
	// 创建者：涂佳, 谷德峰
	// 创建时间：2012/05/01
	// 版本时间：
	// 修改记录：
	// 备注： 
	double GNSSBasicCorrectFunc::correctLeoAntPhaseWindUp(int id_Block, POS6D receiverPosVel, POS3D gpsPos, POS3D sunPos, double prev)
	{
		// 计算LEO卫星星体系
		POS3D ex_LEO = vectorNormal(receiverPosVel.getVel());
		POS3D ez_LEO = vectorNormal(receiverPosVel.getPos() *(-1.0));
		POS3D ey_LEO;
		vectorCross(ey_LEO, ez_LEO, ex_LEO); 
		ey_LEO = vectorNormal(ey_LEO);    
		POS3D unitXr = ex_LEO;
		POS3D unitYr = ey_LEO *(-1.0); 
        return correctPhaseWindUp(id_Block, receiverPosVel.getPos(), unitXr, unitYr, gpsPos, sunPos, prev);
        // 以下代码为合并之前的有效代码, 20130923
		//// 计算GPS卫星星体系
		//double correctdistance = 0;
		//POS3D  vecRs =  sunPos - gpsPos; // 太阳矢量, 由GPS卫星指向太阳     
		//POS3D ez =  vectorNormal(gpsPos) * (-1.0);
		//POS3D ns =  vectorNormal(vecRs);
		//POS3D ey;
		//vectorCross(ey, ez, ns); 
		//ey = vectorNormal(ey);
		//POS3D ex;
		//vectorCross(ex, ey, ez);
		//ex = vectorNormal(ex);
		//if(id_Block >= 4 && id_Block <= 6)
		//{
		//	ex = ex * (-1.0);
		//	ey = ey * (-1.0);	
		//}
		//// 计算LEO卫星星体系
		//POS3D ex_LEO = vectorNormal(receiverPosVel.getVel());
		//POS3D ez_LEO = vectorNormal(receiverPosVel.getPos() *(-1.0));
		//POS3D ey_LEO;
		//vectorCross(ey_LEO, ez_LEO, ex_LEO); 
		//ey_LEO = vectorNormal(ey_LEO);                        
		//vectorCross(ex_LEO, ey_LEO, ez_LEO);
		//POS3D vecLos = vectorNormal(receiverPosVel.getPos() - gpsPos);
		//POS3D k_ey_GPS;
		//POS3D k_ey_LEO;
		//vectorCross(k_ey_GPS, vecLos, ey);
		//vectorCross(k_ey_LEO, vecLos, ey_LEO * (-1));
		//POS3D D_GPS = ex - vecLos * (ex.x * vecLos.x + ex.y * vecLos.y + ex.z * vecLos.z) - k_ey_GPS;
		//POS3D D_LEO = ex_LEO  - vecLos * (ex_LEO.x * vecLos.x + ex_LEO.y * vecLos.y + ex_LEO.z * vecLos.z) + k_ey_LEO;
		//POS3D DD;
		//vectorCross(DD, D_GPS, D_LEO);
		//double theta = vectorDot(vecLos, DD);
		//int sign = 0;
		//if(theta > 0)
		//	sign = 1;
		//if(theta == 0)
		//	sign = 0;
		//if(theta < 0)
		//	sign = -1;
		//D_GPS = vectorNormal(D_GPS);
		//D_LEO = vectorNormal(D_LEO);
		//double cos_fai = D_GPS.x * D_LEO.x + D_GPS.y * D_LEO.y + D_GPS.z * D_LEO.z;
		//cos_fai = cos_fai >  1 ?  1 : cos_fai; // 2013/07/04, 确保 -1 <= cos_fai <= 1
		//cos_fai = cos_fai < -1 ? -1 : cos_fai;
		//correctdistance = sign * acos(cos_fai);
		//correctdistance = (-1.0) * correctdistance * SPEED_LIGHT / ((GPS_FREQUENCE_L1 + GPS_FREQUENCE_L2) *(2 * PI));
		//return correctdistance;
	}
	// 相位缠绕修正，不根据GNSS卫星类型进行区分，统一计算GPS卫星轨道系
	double GNSSBasicCorrectFunc::correctLeoAntPhaseWindUp_GNSS(POS6D receiverPosVel, POS3D gpsPos, POS3D sunPos, double prev)
	{
		double windup = 0;
		// 计算GPS卫星星体系，所有类型卫星一致
		POS3D vecRs =  sunPos - gpsPos; // 太阳矢量, 由GPS卫星指向太阳     
		POS3D ez =  vectorNormal(gpsPos) * (-1.0);
		POS3D ns =  vectorNormal(vecRs);
		POS3D ey;
		vectorCross(ey, ez, ns); 
		ey = vectorNormal(ey);
		POS3D ex;
		vectorCross(ex, ey, ez);
		ex = vectorNormal(ex);
		// 计算LEO卫星星体系
		POS3D ex_LEO = vectorNormal(receiverPosVel.getVel());
		POS3D ez_LEO = vectorNormal(receiverPosVel.getPos() *(-1.0));
		POS3D ey_LEO;
		vectorCross(ey_LEO, ez_LEO, ex_LEO); 
		ey_LEO = vectorNormal(ey_LEO);    
		POS3D unitXr = ex_LEO;
		POS3D unitYr = ey_LEO *(-1.0);  
		// 计算相位缠绕
		POS3D vecLos = vectorNormal(receiverPosVel.getPos() - gpsPos);
		POS3D k_ey_GPS;
		POS3D k_ey_Rec;
		vectorCross(k_ey_GPS, vecLos, ey);
		vectorCross(k_ey_Rec, vecLos, unitYr); 
		POS3D D_R = unitXr  - vecLos * (unitXr.x * vecLos.x + unitXr.y * vecLos.y + unitXr.z * vecLos.z) + k_ey_Rec;
		POS3D D_T = ex - vecLos * (ex.x * vecLos.x + ex.y * vecLos.y + ex.z * vecLos.z) - k_ey_GPS;
		POS3D DD;
		vectorCross(DD, D_T, D_R);
		double theta = vectorDot(vecLos, DD);
		int sign = 1;
		/*if(theta > 0)
			sign = 1;
		if(theta == 0)
			sign = 0;*/
		if(theta < 0)
			sign = -1;
		D_T = vectorNormal(D_T);
		D_R = vectorNormal(D_R);
		double cos_fai = D_T.x * D_R.x + D_T.y * D_R.y + D_T.z * D_R.z;
		cos_fai = cos_fai >  1 ?  1 : cos_fai; // 2013/07/04, 确保 -1 <= cos_fai <= 1
		cos_fai = cos_fai < -1 ? -1 : cos_fai;
		windup = sign * acos(cos_fai) / (2 * PI);
		if(prev != DBL_MAX && prev != 0.0)
		{// 连续化, 20140427
			//double d = windup - prev;
			//windup -= int(d + (d < 0.0 ? -0.5 : 0.5));
			windup = windup + floor(prev - windup + 0.5); // in cycle; 参考 rtklib 
		}
		return windup;
	}
	// 低轨卫星接收GEO卫星相位缠绕修正
	double GNSSBasicCorrectFunc::correctLeoAntPhaseWindUp_GEO(POS6D receiverPosVel, POS3D gpsPos, POS3D sunPos, double prev)
	{
		double windup = 0;
		// 计算GEO卫星星体系，一直零偏姿态
		POS3D vecRs =  sunPos - gpsPos; // 太阳矢量, 由GPS卫星指向太阳  
		POS3D ex = vectorNormal(vecRs); // 速度方向
		POS3D ez =  vectorNormal(gpsPos) * (-1.0); // 对地
		POS3D ey;
		vectorCross(ey, ez, ex); 
		ey = vectorNormal(ey);
		vectorCross(ex, ey, ez);
		ex = vectorNormal(ex);
		// 计算LEO卫星星体系
		POS3D ex_LEO = vectorNormal(receiverPosVel.getVel());
		POS3D ez_LEO = vectorNormal(receiverPosVel.getPos() *(-1.0));
		POS3D ey_LEO;
		vectorCross(ey_LEO, ez_LEO, ex_LEO); 
		ey_LEO = vectorNormal(ey_LEO);    
		POS3D unitXr = ex_LEO;
		POS3D unitYr = ey_LEO *(-1.0);  
		// 计算相位缠绕
		POS3D vecLos = vectorNormal(receiverPosVel.getPos() - gpsPos);
		POS3D k_ey_GPS;
		POS3D k_ey_Rec;
		vectorCross(k_ey_GPS, vecLos, ey);
		vectorCross(k_ey_Rec, vecLos, unitYr); 
		POS3D D_R = unitXr  - vecLos * (unitXr.x * vecLos.x + unitXr.y * vecLos.y + unitXr.z * vecLos.z) + k_ey_Rec;
		POS3D D_T = ex - vecLos * (ex.x * vecLos.x + ex.y * vecLos.y + ex.z * vecLos.z) - k_ey_GPS;
		POS3D DD;
		vectorCross(DD, D_T, D_R);
		double theta = vectorDot(vecLos, DD);
		int sign = 1;
		/*if(theta > 0)
			sign = 1;
		if(theta == 0)
			sign = 0;*/
		if(theta < 0)
			sign = -1;
		D_T = vectorNormal(D_T);
		D_R = vectorNormal(D_R);
		double cos_fai = D_T.x * D_R.x + D_T.y * D_R.y + D_T.z * D_R.z;
		cos_fai = cos_fai >  1 ?  1 : cos_fai; // 2013/07/04, 确保 -1 <= cos_fai <= 1
		cos_fai = cos_fai < -1 ? -1 : cos_fai;
		windup = sign * acos(cos_fai) / (2 * PI);
		if(prev != DBL_MAX && prev != 0.0)
		{// 连续化, 20140427
			//double d = windup - prev;
			//windup -= int(d + (d < 0.0 ? -0.5 : 0.5));
			windup = windup + floor(prev - windup + 0.5); // in cycle; 参考 rtklib 
		}
		return windup;
	}
	// 子程序名称： correctStaAntPhaseWindUp   
	// 功能：对地面测站接收机相位缠绕进行修正
	// 变量类型：id_Block           : GPS卫星类型
	//           receiverPos        : 接收机位置
	//           gpsPos             : GPS卫星位置
	//           sunPos             : 太阳位置
	// 输入：id_Block, gpsPCO, receiverPos, gpsPos, sunPos
	// 输出：correctdistance
	// 语言：C++
	// 创建者：涂佳, 谷德峰 
	// 创建时间：2012/05/01
	// 版本时间：
	// 修改记录：
	// 备注： 
	double GNSSBasicCorrectFunc::correctStaAntPhaseWindUp(int id_Block, POS3D receiverPos, POS3D gpsPos, POS3D sunPos, double prev)
	{
		// 计算天线东北天坐标系
		POS3D vecU; // 垂直径向
		vecU = receiverPos;
		POS3D vecN; // 北方向
		vecN.x = 0;
		vecN.y = 0;
		vecN.z = EARTH_R; 							
		POS3D vecE; // 东方向
		vectorCross(vecE,vecN,vecU);
		vectorCross(vecN,vecU,vecE);
		POS3D unitXr = vectorNormal(vecN); // 北方向
		POS3D unitYr = vectorNormal(vecE) *(-1.0); // 西方向
        return correctPhaseWindUp(id_Block, receiverPos, unitXr, unitYr, gpsPos, sunPos, prev);
	}
	// 子程序名称： correctPhaseWindUp_GYM95   
	// 功能：对相位缠绕进行修正
	// 变量类型：
	//           unitXr             : north unit vector at receiver（地面测站接收机）
	//           unitYr             : west unit vector at receiver（地面测站接收机）
	//           vecLos             : 视线矢量，卫星指向测站
	//           ex                 : 星固系X轴单位矢量
	//           ey                 : 星固系Y轴单位矢量
	//           ez                 : 星固系Z轴单位矢量
	//           prev               : 同一弧段, 前一时刻修正相位值, (如果为DBL_MAX, 说明当前时刻为起点)
	//           windup             : 同一弧段, 当前时刻修正相位值, 返回值, [-1, 1]
	// 输入：unitXr,unitYr,vecLos,ex,ey,ez, prev
	// 输出：windup
	// 语言：C++
	// 创建者：涂佳, 谷德峰, 刘俊宏
	// 创建时间：2014/11/20
	// 版本时间：
	// 修改记录：
	// 备注： 参考GPSTK进行了连续化
	double GNSSBasicCorrectFunc::correctPhaseWindUp_GYM95(POS3D unitXr, POS3D unitYr,POS3D vecLos, POS3D ex, POS3D ey, POS3D ez, double prev)
	{
		// 计算GPS卫星星体系
		double windup = 0;		
		vecLos = vectorNormal(vecLos); //单位化
		POS3D k_ey_GPS;
		POS3D k_ey_Rec;
		vectorCross(k_ey_GPS, vecLos, ey);
		vectorCross(k_ey_Rec, vecLos, unitYr); 
		POS3D D_R = unitXr  - vecLos * (unitXr.x * vecLos.x + unitXr.y * vecLos.y + unitXr.z * vecLos.z) + k_ey_Rec;
		POS3D D_T = ex - vecLos * (ex.x * vecLos.x + ex.y * vecLos.y + ex.z * vecLos.z) - k_ey_GPS;
		POS3D DD;
		vectorCross(DD, D_T, D_R);
		double theta = vectorDot(vecLos, DD);
		int sign = 1;
		if(theta < 0)
			sign = -1;
		D_T = vectorNormal(D_T);
		D_R = vectorNormal(D_R);
		double cos_fai = D_T.x * D_R.x + D_T.y * D_R.y + D_T.z * D_R.z;
		cos_fai = cos_fai >  1 ?  1 : cos_fai; // 2013/07/04, 确保 -1 <= cos_fai <= 1
		cos_fai = cos_fai < -1 ? -1 : cos_fai;
		windup = sign * acos(cos_fai) / (2 * PI);
		if(prev != DBL_MAX && prev != 0.0)
		{// 连续化, 20140427
			double d = windup - prev;
			windup -= int(d + (d < 0.0 ? -0.5 : 0.5));
		}		
		return windup;
	}
	// 子程序名称： correctLeoAntPhaseWindUp_GYM95   
	// 功能：对低轨卫星的天线相位缠绕进行修正
	// 变量类型：
	//           receiverPosVel     : 接收机位置速度
	//           vecLos             : 视线矢量，卫星指向测站
	//           ex                 : 星固系X轴单位矢量
	//           ey                 : 星固系Y轴单位矢量
	//           ez                 : 星固系Z轴单位矢量
	// 输入：receiverPosVel,vecLos,ex,ey,ez
	// 输出：correctdistance
	// 语言：C++
	// 创建者：涂佳, 谷德峰,刘俊宏
	// 创建时间：2014/11/20
	// 版本时间：
	// 修改记录：
	// 备注： 
	double GNSSBasicCorrectFunc::correctLeoAntPhaseWindUp_GYM95(POS6D receiverPosVel, POS3D vecLos, POS3D ex, POS3D ey, POS3D ez, double prev)
	{
		// 计算LEO卫星星体系
		POS3D ex_LEO = vectorNormal(receiverPosVel.getVel());
		POS3D ez_LEO = vectorNormal(receiverPosVel.getPos() *(-1.0));
		POS3D ey_LEO;
		vectorCross(ey_LEO, ez_LEO, ex_LEO); 
		ey_LEO = vectorNormal(ey_LEO);    
		POS3D unitXr = ex_LEO;
		POS3D unitYr = ey_LEO *(-1.0); 
        return correctPhaseWindUp_GYM95(unitXr, unitYr,vecLos, ex, ey, ez, prev);       
	}
	// 子程序名称： correctStaAntPhaseWindUp_GYM95   
	// 功能：对低轨卫星的天线相位缠绕进行修正
	// 变量类型：
	//           receiverPosVel     : 接收机位置速度
	//           vecLos             : 视线矢量，卫星指向测站
	//           ex                 : 星固系X轴单位矢量
	//           ey                 : 星固系Y轴单位矢量
	//           ez                 : 星固系Z轴单位矢量
	// 输入：receiverPosVel,vecLos,ex,ey,ez
	// 输出：correctdistance
	// 语言：C++
	// 创建者：涂佳, 谷德峰,刘俊宏
	// 创建时间：2014/11/20
	// 版本时间：
	// 修改记录：
	// 备注： 
	double GNSSBasicCorrectFunc::correctStaAntPhaseWindUp_GYM95(POS3D receiverPos, POS3D vecLos, POS3D ex, POS3D ey, POS3D ez, double prev)
	{
		// 计算天线东北天坐标系
		POS3D vecU; // 垂直径向
		vecU = receiverPos;
		POS3D vecN; // 北方向
		vecN.x = 0;
		vecN.y = 0;
		vecN.z = EARTH_R; 							
		POS3D vecE; // 东方向
		vectorCross(vecE,vecN,vecU);
		vectorCross(vecN,vecU,vecE);
		POS3D unitXr = vectorNormal(vecN); // 北方向
		POS3D unitYr = vectorNormal(vecE) *(-1.0); // 西方向
        return correctPhaseWindUp_GYM95(unitXr, unitYr,vecLos, ex, ey, ez, prev);       
	}

	//   子程序名称： ionIpp   
	//   作用：根据GPS卫星位置、接收机位置和计算穿刺点的位置（经度, 纬度, 视线仰角）
	//   类型：leoPos          : 接收机位置 （地固系）
	//         gpsPos          : GPS卫星位置（地固系）
	//         H0              : 穿刺点的高度
	//         latitude_ipp    : 穿刺点的纬度,     度
	//         longitude_ipp   : 穿刺点的经度,     度
	//         elevation_ipp   : 穿刺点的观测仰角, 度
	//   输入：leoPos, gpsPos, H0
	//   输出：latitude_ipp, longitude_ipp,  elevation_ipp
	//   其它：
	//   语言：C++
	//   版本号：2008.8.23
	//   生成者：涂先勤(提供matlab算法), 谷德峰(C++翻译)
	//   修改者：
	bool GNSSBasicCorrectFunc::ionIpp(POS3D leoPos, POS3D gpsPos, double H0, double &latitude_ipp, double &longitude_ipp, double &elevation_ipp)
	{
		// 计算接收机的经纬度
		POLARCOORD leoRFL;
		TimeCoordConvert::Cartesian2Polar(leoPos, leoRFL);
		double fai_s   = leoRFL.fai   * PI / 180.0; // 弧度
		double lamda_s = leoRFL.lamda * PI / 180.0; // 弧度
		// 计算接收机的观测仰角 elevation_s
		POS3D v_los = gpsPos - leoPos;
		v_los = vectorNormal(v_los);
		POS3D v_r   = leoPos;
		v_r = vectorNormal(v_r);
		double value = vectorDot(v_los, v_r);
		double elevation_s = PI / 2.0 - acos(value); // 弧度
		// 计算接收机到 GPS 卫星的视线方位角 sita_s
		POS3D S_V;       // 垂直径向
		S_V =  leoPos;
		POS3D S_N;       // 北方向
		S_N.x = 0;
		S_N.y = 0;
		S_N.z = EARTH_R; // 北极点
		POS3D S_E;       // 东方向
		vectorCross(S_E,S_N,S_V);
		vectorCross(S_N,S_V,S_E);
		S_E = vectorNormal(S_E);
		S_N = vectorNormal(S_N);
		S_V = vectorNormal(S_V);
		POS3D LOS_ENU;
		LOS_ENU.x = vectorDot(v_los, S_E); 
		LOS_ENU.y = vectorDot(v_los, S_N); 
		LOS_ENU.z = vectorDot(v_los, S_V);
		double sita_s = atan2(LOS_ENU.x, LOS_ENU.y); // tan(sita_s) = 东 / 北
		// 计算接收机与穿刺点在地心处所张的夹角 psai
		double h_s  = sqrt(leoPos.x * leoPos.x + leoPos.y * leoPos.y + leoPos.z * leoPos.z) - EARTH_R;
		double psai = PI / 2 - asin((EARTH_R + h_s) * cos(elevation_s) /  (EARTH_R + H0)) - elevation_s;
		// 穿刺点处观测仰角
		elevation_ipp = psai + elevation_s;
		// 穿刺点处纬度
		latitude_ipp  = asin(sin(fai_s) * cos(psai) + cos(fai_s) * sin(psai) * cos(sita_s)); // 弧度
		// 穿刺点处经度
		longitude_ipp = lamda_s + asin( sin(psai) * sin(sita_s) / cos(latitude_ipp)); // 弧度
		// 换算成度
		elevation_ipp = elevation_ipp * 180 / PI;
		latitude_ipp  = latitude_ipp  * 180 / PI;
		longitude_ipp = longitude_ipp * 180 / PI;
		return true;
	}

	// 子程序名称： judgeGPSEarthShadowManeuver_moon   
	// 功能：地影机动判断,增加月影判断
	// 变量类型：sunPos              : 太阳的位置, 单位：米
	//           moonPos             : 月球位置，单位：米
	//           gpsPos              : gps卫星的轨道位置, 单位：米
	//           factor              : 阴影因子
	// 输入：sunPos, gpsPos
	// 输出：factor
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2014/05/04
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool GNSSBasicCorrectFunc::judgeGPSEarthShadowManeuver_moon(POS3D sunPos, POS3D moonPos, POS3D gpsPos, double &factor)
	{
		// 地球阴影计算
		const double Rs = 6.96E+08; // 太阳半径
		const double Rm = 1738000;  // 月球半径
		const double Re = 6371000;  // 地球半径
		POS3D  DELTA_S = sunPos - gpsPos;  // 卫星至太阳的矢量
		double r   = sqrt(gpsPos.x * gpsPos.x + gpsPos.y * gpsPos.y + gpsPos.z * gpsPos.z);
		double fai = asin(gpsPos.z / r);   // 计算地心纬度
		double delta_s = sqrt(DELTA_S.x * DELTA_S.x + DELTA_S.y * DELTA_S.y + DELTA_S.z * DELTA_S.z);
		double sita_ES = acos(-vectorDot(gpsPos, DELTA_S)/(r * delta_s)); // 地球-卫星-太阳张角
		double alpha_S = asin(Rs/delta_s); // 上视太阳
		double alpha_E = asin(Re/r);       // 上视地球
		double beta_E = (sita_ES * sita_ES + alpha_S * alpha_S - alpha_E * alpha_E) / (2.0 * sita_ES); 
		double AE = 0; // 太阳被地球所蚀面积
		// 1 判断卫星是否位于地球本影区
		bool bEarth_self_shadow = false;
		if(vectorDot(gpsPos, DELTA_S) < 0      // 2009/05/26 修改符号
		&& fabs(alpha_E - alpha_S) >= sita_ES)
		{
			bEarth_self_shadow = true;
			AE = PI * alpha_S * alpha_S;
		}
		// 2 判断卫星是否位于地球半影区
		bool bEarth_half_shadow = false;
		if(vectorDot(gpsPos, DELTA_S) < 0     // 2009/05/26 修改符号
		&& (alpha_E + alpha_S) > sita_ES
		&& fabs(alpha_E - alpha_S) < sita_ES)
		{
			bEarth_half_shadow = true;
			AE = alpha_S * alpha_S * acos(beta_E / alpha_S)
			   + alpha_E * alpha_E * acos((sita_ES - beta_E) / alpha_E)
			   - sita_ES * sqrt(alpha_S * alpha_S - beta_E * beta_E);
		}
	   // 月球阴影计算
		POS3D  DELTA_M = moonPos - gpsPos; // 卫星至月球的矢量, 注意与r的符号相反
		double r_m = sqrt(DELTA_M.x * DELTA_M.x + DELTA_M.y * DELTA_M.y + DELTA_M.z * DELTA_M.z);
		double delta_m = sqrt(DELTA_M.x * DELTA_M.x + DELTA_M.y * DELTA_M.y + DELTA_M.z * DELTA_M.z);
		double sita_MS = acos(vectorDot(DELTA_M, DELTA_S)/(delta_m * delta_s)); // 月球-卫星-太阳张角
		double alpha_M = asin(Rm/r_m);
		double beta_M = (sita_MS * sita_MS + alpha_S * alpha_S - alpha_M * alpha_M) / (2.0 * sita_MS); // 参考李济生教材
		double AM = 0; // 太阳被月球所蚀面积
		// 1 判断卫星是否位于月球本影区
		bool bMoon_self_shadow = false;
		if(vectorDot(DELTA_M, DELTA_S) > 0
		&& alpha_S <= alpha_M
		&& fabs(alpha_M - alpha_S) >= sita_MS)
		{
			bMoon_self_shadow = true;
			AM = PI * alpha_S * alpha_S;
		}
		// 2 判断卫星在月球伪本影区
		bool bMoon_false_shadow = false;
		if(vectorDot(DELTA_M, DELTA_S) < 0
		&& alpha_S > alpha_M
		&& fabs(alpha_M - alpha_S) >= sita_MS)
		{
			bMoon_false_shadow = true;
			AM = PI * alpha_M * alpha_M;
		}
		// 3 判断卫星在月球半影区
		bool bMoon_half_shadow = false;
		if(vectorDot(DELTA_M, DELTA_S) < 0
		&& (alpha_M + alpha_S) > sita_MS
		&& fabs(alpha_M - alpha_S) < sita_MS)
		{
			bMoon_half_shadow = true;
			AM = alpha_S * alpha_S * acos(beta_M / alpha_S)
			   + alpha_M * alpha_M * acos((sita_MS - beta_M) / alpha_M)
			   - sita_MS * sqrt(alpha_S * alpha_S - beta_M * beta_M);
		}
		// 计算阴影因子， factor = 1.0 代表光照
		factor = 1.0 - max(AE, AM) / (PI * alpha_S * alpha_S);
		//// 计算阴影因子, factor = 1.0 代表光照
		//factor = 1.0 - AE / (PI * alpha_S * alpha_S);
		if(bEarth_self_shadow || bEarth_half_shadow || bMoon_self_shadow || bMoon_false_shadow || bMoon_half_shadow)
			return true;
		else
			return false;
	}

	// 子程序名称： judgeGPSEarthShadowManeuver   
	// 功能：地影机动判断
	// 变量类型：sunPos              : 太阳的位置, 单位：米
	//           gpsPos              : gps卫星的轨道位置, 单位：米
	//           factor              : 阴影因子
	// 输入：sunPos, gpsPos
	// 输出：factor
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2014/05/04
	// 版本时间：
	// 修改记录：
	// 备注： 
	bool GNSSBasicCorrectFunc::judgeGPSEarthShadowManeuver(POS3D sunPos, POS3D gpsPos, double &factor)
	{
		// 地球阴影计算
		const double Rs = 6.96E+08; // 太阳半径
		const double Re = 6371000;  // 地球半径
		POS3D  DELTA_S = sunPos - gpsPos;  // 卫星至太阳的矢量
		double r   = sqrt(gpsPos.x * gpsPos.x + gpsPos.y * gpsPos.y + gpsPos.z * gpsPos.z);
		double fai = asin(gpsPos.z / r);   // 计算地心纬度
		double delta_s = sqrt(DELTA_S.x * DELTA_S.x + DELTA_S.y * DELTA_S.y + DELTA_S.z * DELTA_S.z);
		double sita_ES = acos(-vectorDot(gpsPos, DELTA_S)/(r * delta_s)); // 地球-卫星-太阳张角
		double alpha_S = asin(Rs/delta_s); // 上视太阳
		double alpha_E = asin(Re/r);       // 上视地球
		double beta_E = (sita_ES * sita_ES + alpha_S * alpha_S - alpha_E * alpha_E) / (2.0 * sita_ES); 
		double AE = 0; // 太阳被地球所蚀面积
		// 1 判断卫星是否位于地球本影区
		bool bEarth_self_shadow = false;
		if(vectorDot(gpsPos, DELTA_S) < 0      // 2009/05/26 修改符号
		&& fabs(alpha_E - alpha_S) >= sita_ES)
		{
			bEarth_self_shadow = true;
			AE = PI * alpha_S * alpha_S;
		}
		// 2 判断卫星是否位于地球半影区
		bool bEarth_half_shadow = false;
		if(vectorDot(gpsPos, DELTA_S) < 0     // 2009/05/26 修改符号
		&& (alpha_E + alpha_S) > sita_ES
		&& fabs(alpha_E - alpha_S) < sita_ES)
		{
			bEarth_half_shadow = true;
			AE = alpha_S * alpha_S * acos(beta_E / alpha_S)
			   + alpha_E * alpha_E * acos((sita_ES - beta_E) / alpha_E)
			   - sita_ES * sqrt(alpha_S * alpha_S - beta_E * beta_E);
		}
		// 计算阴影因子, factor = 1.0 代表光照
		factor = 1.0 - AE / (PI * alpha_S * alpha_S);
		if(bEarth_self_shadow || bEarth_half_shadow)
			return true;
		else
			return false;
	}

	// 子程序名称： judgeGPSNoonManeuver   
	// 功能：正午机动判断
	// 变量类型：sunPos              : 太阳的位置, 单位：米
	//           gpsPos              : gps卫星的轨道位置, 单位：米
	//           gpsPosVel           : gps卫星的轨道位置和速度, 单位：米，米/秒
	// 输入：sunPos, gpsPos
	// 输出：
	// 语言：C++
	// 创建者：谷德峰
	// 创建时间：2014/05/04
	// 版本时间：
	// 修改记录：2014/11/14,增加β角计算，因为只有在β角较小时才会进入noonTurn
	// 备注： 
	bool GNSSBasicCorrectFunc::judgeGPSNoonManeuver(POS3D sunPos, POS6D gpsPosVel)
	{
		POS3D unit_sun    = vectorNormal(sunPos);
		POS3D unit_gpspos = vectorNormal(gpsPosVel.getPos());			
		POS3D unit_gpsvel = vectorNormal(gpsPosVel.getVel());
		POS3D unit_n;
		vectorCross(unit_n, unit_gpspos, unit_gpsvel);
		unit_n = vectorNormal(unit_n);
		//  计算太阳入射方向与轨道面夹角β
		double beta =  PI / 2 - acos(vectorDot(unit_n, unit_sun));
		// 卫星-地球-太阳张角, [0, pi], = pi - E	
		double sita = acos(vectorDot(unit_sun, unit_gpspos)); 
		if(fabs(sita * 180) / PI <= 5 && fabs(beta) * 180 / PI < 10 ) // 正午
			return true;
		else
			return false;
	}
	// 子程序名称：getPrioriAlpha_Montenbruck   
	// 功能：根据高度计算先验电离层比例因子
	// 变量类型：h          : 目标高度
	//           h_IonLayer : 单层电离层高度, 默认350km
	// 输入：h, h_IonLayer
	// 输出：
	// 语言：C++
	// 创建者：高永飞
	// 创建时间：2014/10/28
	// 版本时间：
	// 修改记录：
	// 备注：参考Oliver Montenbruck "Ionospheric Correction for GPS Tracking of LEO Satellites"
	double GNSSBasicCorrectFunc::ionPrioriAlpha(double h, double h_IonLayer)
	{
		double ScaleHeight = 100000;
		return 0.5 * (exp(1.0) - exp(1 - exp((h_IonLayer - h) / ScaleHeight))) / (exp(1.0) - exp(1 - exp(h_IonLayer / ScaleHeight)));
	}

	// 子程序名称： ionexGridCorrect_alpha   
	// 功能： 计算IONEX网格电离层修正量（比例因子可以从外部输入）
	// 变量类型：ionFile                  : 电离层文件, 提供VTEC插值结果
    //           t                        : 时间
	//           leoPos                   : 目标位置, 地固系
    //           gpsPos                   : gps卫星位置, 地固系
	//           alpha                    : 电离层比例因子
	//           value                    : 电离层修正量
	//           d_mapFun_r_ip            : 投影高度偏导数
	//           addLayerHeight           : 附加的单层电离层投影高度, 默认80km
	//           frequence                : 频率, 默认L1
	// 输入：t, recPos，gpsPos，alpha
	// 输出：value
	// 语言：C++
	// 创建者：高永飞
	// 创建时间：2014/10/28
	// 版本时间：
	// 修改记录：
	// 备注：
	bool GNSSBasicCorrectFunc::ionexGridCorrect_IP_alpha(Ionex1_0_File &ionFile, GPST t, POS3D leoPos, POS3D gpsPos, double alpha, double& value, double &d_mapFun_r_ip, double addLayerHeight, double frequence)
	{
		POLARCOORD polarLeo;
		TimeCoordConvert::Cartesian2Polar(leoPos, polarLeo);
		double LayerHeight = polarLeo.r + addLayerHeight;// 计算等效电离层高度
		// 计算经纬度latLeo、lonLeo
		double latLeo = polarLeo.fai   * PI / 180.0; // 弧度
		double lonLeo = polarLeo.lamda * PI / 180.0; // 弧度
		// 计算地心、穿刺点、目标点张角
		POS3D v_los = gpsPos - leoPos;
		v_los = vectorNormal(v_los);
		POS3D v_r   = leoPos * (-1);
		v_r = vectorNormal(v_r);
		double angle_Earth_Ipp_Leo = acos(vectorDot(v_los, v_r)); // 弧度
		double angle_Earth_Leo_Ipp = asin(sin(angle_Earth_Ipp_Leo) / LayerHeight * polarLeo.r); // 穿刺点处的投影角
		double angle_Leo_Earth_Ipp = PI - angle_Earth_Ipp_Leo - angle_Earth_Leo_Ipp; // 计算目标点、地心、穿刺点的张角
		// 计算GPS卫星相对与Leo的方位角
		POS3D S_V;       // 垂直径向
		S_V =  leoPos;
		POS3D S_N;       // 北方向
		S_N.x = 0;
		S_N.y = 0;
		S_N.z = EARTH_R; // 北极点
		POS3D S_E;       // 东方向
		vectorCross(S_E,S_N,S_V);
		vectorCross(S_N,S_V,S_E);
		S_E = vectorNormal(S_E);
		S_N = vectorNormal(S_N);
		S_V = vectorNormal(S_V);
		POS3D LOS_ENU;
		LOS_ENU.x = vectorDot(v_los, S_E); 
		LOS_ENU.y = vectorDot(v_los, S_N); 
		LOS_ENU.z = vectorDot(v_los, S_V);
		double azimuth = atan2(LOS_ENU.x, LOS_ENU.y); // tan(azimuth) = 东 / 北
		double latIpp = PI / 2 - acos(cos(PI / 2 - latLeo) * cos(angle_Leo_Earth_Ipp) + sin(PI / 2 - latLeo) * sin(angle_Leo_Earth_Ipp) * cos(azimuth));// 计算穿刺点纬度
		double lonIpp = lonLeo + asin(sin(angle_Leo_Earth_Ipp) * sin(azimuth) / cos(latIpp));// 计算穿刺点经度
		latIpp = latIpp * 180 / PI;
		lonIpp = lonIpp * 180 / PI;
		double VTEC;
		if(!ionFile.getVTEC(t, latIpp, lonIpp, VTEC))
			return false;
		double mapFun = 1 / cos(angle_Earth_Leo_Ipp);
		double TEC = mapFun * VTEC * 1E16;
		value = alpha * 40.3 / pow(frequence , 2 ) * TEC;

		// 计算映射函数对投影高度的导数
		double Es   = angle_Earth_Ipp_Leo - PI / 2; // [pi / 2 , pi]
		double r_s  = polarLeo.r;
		double r_IP = LayerHeight;
		//double mapFun_2 = 1 / sqrt(1 - pow(cos(Es) * r_s / r_IP, 2));
		d_mapFun_r_ip = -0.5 / sqrt(pow(1 - pow(cos(Es) * r_s / r_IP, 2), 3)) * 2 * cos(Es) * r_s * pow(1 / r_IP, 2);
		d_mapFun_r_ip *= alpha * 40.3 / pow(frequence , 2 ) * VTEC * 1E16;

		/*char info[100];
		sprintf(info, "%10.2lf %16.4lf %16.8lf", Es * 180 / PI, mapFun, d_mapFun_r_IP);
		RuningInfoFile::Add(info);*/

		return true;
	}

	bool GNSSBasicCorrectFunc::ionexGridCorrect_alpha(Ionex1_0_File &ionFile, GPST t, POS3D leoPos, POS3D gpsPos, double alpha, double& value, double frequence)
	{
		POLARCOORD polarLeo;
		TimeCoordConvert::Cartesian2Polar(leoPos, polarLeo);
		// 计算地心、穿刺点、目标点张角
		POS3D v_los = gpsPos - leoPos;
		v_los = vectorNormal(v_los);
		POS3D v_r   = leoPos * (-1);
		v_r = vectorNormal(v_r);
		double angle_Earth_Leo_Gps = acos(vectorDot(v_los, v_r)); // 弧度
		// 计算经纬度latLeo、lonLeo
		double latLeo = polarLeo.fai;   // 度
		double lonLeo = polarLeo.lamda; // 度
		double VTEC;
		if(!ionFile.getVTEC(t, latLeo, lonLeo, VTEC))
			return false;
		double Es = angle_Earth_Leo_Gps - PI / 2; // [pi / 2 , pi]
		double mapFun = 2.037 / (sqrt(sin(Es) * sin(Es) + 0.076) + sin(Es));
		double TEC = mapFun * VTEC * 1E16;
		value = alpha * 40.3 / pow(frequence , 2 ) * TEC;
		return true;
	}

	// 子程序名称： ionexGridCorrect_IP   
	// 功能： 计算IONEX网格电离层修正量（比例因子采用先验值）
	// 变量类型：ionFile                  : 电离层文件, 提供VTEC插值结果
    //           t                        : 时间
	//           leoPos                   : 目标位置, 地固系
    //           gpsPos                   : gps卫星位置, 地固
	//           value                    : 电离层修正量
	//           addLayerHeight           : 附加的单层电离层投影高度, 默认80km
	//           frequence                : 频率, 默认L1
	// 输入：t, recPos，gpsPos，alpha
	// 输出：value
	// 语言：C++
	// 创建者：高永飞
	// 创建时间：2014/10/28
	// 版本时间：
	// 修改记录：
	// 备注：
	bool GNSSBasicCorrectFunc::ionexGridCorrect_IP(Ionex1_0_File &ionFile, GPST t, POS3D leoPos, POS3D gpsPos, double& value, double addLayerHeight, double frequence)
	{
		// 参考Oliver Montenbruck "Ionospheric Correction for GPS Tracking of LEO Satellites"
		double r = sqrt(leoPos.x * leoPos.x + leoPos.y * leoPos.y + leoPos.z * leoPos.z);
		double H0 = 350; // 350km
		double ScaleHeight = 100;
		double alpha = 0.5 * (exp(1.0) - exp(1 - exp((H0 * 1000.0 + ionFile.m_header.BaseRadius * 1000.0 - r) / (ScaleHeight * 1000.0)))) / (exp(1.0) - exp(1 - exp(H0 * 1.0 / 100.0)));
		return ionexGridCorrect_IP_alpha(ionFile, t, leoPos, gpsPos, alpha, value, addLayerHeight, frequence);
	}
	// 子程序名称： ionexGridCorrect_IP_High_Order   
	// 功能： 计算IONEX网格电离层修正量（比例因子采用先验值）
	// 变量类型：ionFile                  : 电离层文件, 提供VTEC插值结果
    //           t                        : 时间
	//           leoPos                   : 目标位置, 地固系
    //           gpsPos                   : gps卫星位置, 地固
	//           value                    : 电离层修正量
	//           addLayerHeight           : 附加的单层电离层投影高度, 默认80km
	// 输入：t, recPos，gpsPos，alpha
	// 输出：value
	// 语言：C++
	// 创建者：邵凯
	// 创建时间：2021/10/03
	bool GNSSBasicCorrectFunc::ionexGridCorrect_IP_High_Order(Ionex1_0_File &ionFile, GPST t, POS3D leoPos, POS3D gpsPos, double& value, double addLayerHeight)
	{
		// 参考Oliver Montenbruck "Ionospheric Correction for GPS Tracking of LEO Satellites"
		double r = sqrt(leoPos.x * leoPos.x + leoPos.y * leoPos.y + leoPos.z * leoPos.z);
		double H0 = 350; // 350km
		double ScaleHeight = 100;
		//printf("%f \n",ionFile.m_header.BaseRadius);
		//getchar();
		double alpha = 0.5 * (exp(1.0) - exp(1 - exp((H0 * 1000.0 + ionFile.m_header.BaseRadius * 1000.0 - r) / (ScaleHeight * 1000.0)))) / (exp(1.0) - exp(1 - exp(H0 * 1.0 / 100.0)));
		//printf("%f \n",alpha);
		//getchar();
		return ionexGridCorrect_Iono_High_Order(ionFile, t, leoPos, gpsPos, alpha, value, addLayerHeight);
	}
	// 子程序名称： ionexGridCorrect_Iono_High_Order   
	// 功能： 计算IONEX网格电离层修正量（比例因子可以从外部输入），高阶（二、三）
	// 变量类型：ionFile                  : 电离层文件, 提供VTEC插值结果
    //           t                        : 时间
	//           leoPos                   : 目标位置, 地固系
    //           gpsPos                   : gps卫星位置, 地固系
	//           alpha                    : 电离层比例因子
	//           value                    : 电离层修正量
	//           addLayerHeight           : 附加的单层电离层投影高度, 默认80km
	// 输入：t, recPos，gpsPos，alpha
	// 输出：value
	// 语言：C++
	// 创建者：邵凯
	// 创建时间：2021/10/03
	// 版本时间：
	// 修改记录：参考ionexGridCorrect_IP_alpha代码和RINEX_RO软件，依赖igrf13syn.f获得地球磁场矢量. 邵凯，2021/10/03
	// 备注：
	bool GNSSBasicCorrectFunc::ionexGridCorrect_Iono_High_Order(Ionex1_0_File &ionFile, GPST t, POS3D leoPos, POS3D gpsPos, double alpha, double& value, double addLayerHeight)
	{
		// 以下计算二阶、三阶电离层修正量，需要依赖于地磁：I2 = e*A/(pow(frequence , 3 )*2*pi*m)*BJ*TEC
		// 第一步，参考ionexGridCorrect_IP_alpha代码，获得TEC
		POLARCOORD polarLeo;
		TimeCoordConvert::Cartesian2Polar(leoPos, polarLeo);
		double LayerHeight = polarLeo.r + addLayerHeight;  // 计算等效电离层高度
		// 计算经纬度latLeo、lonLeo
		double latLeo = polarLeo.fai   * PI / 180.0; // 弧度
		double lonLeo = polarLeo.lamda * PI / 180.0; // 弧度
		// 计算地心、穿刺点、目标点张角
		POS3D v_los = gpsPos - leoPos;
		v_los = vectorNormal(v_los);
		POS3D v_r   = leoPos * (-1);
		v_r = vectorNormal(v_r);
		double angle_Earth_Ipp_Leo = acos(vectorDot(v_los, v_r)); // 弧度
		double angle_Earth_Leo_Ipp = asin(sin(angle_Earth_Ipp_Leo) / LayerHeight * polarLeo.r); // 穿刺点处的投影角
		double angle_Leo_Earth_Ipp = PI - angle_Earth_Ipp_Leo - angle_Earth_Leo_Ipp; // 计算目标点、地心、穿刺点的张角
		// 计算GPS卫星相对于Leo的方位角
		POS3D S_V;       // 垂直径向
		S_V =  leoPos;
		POS3D S_N;       // 北方向
		S_N.x = 0;
		S_N.y = 0;
		S_N.z = EARTH_R; // 北极点
		POS3D S_E;       // 东方向
		vectorCross(S_E,S_N,S_V);
		vectorCross(S_N,S_V,S_E);
		S_E = vectorNormal(S_E);
		S_N = vectorNormal(S_N);
		S_V = vectorNormal(S_V);
		POS3D LOS_ENU;
		LOS_ENU.x = vectorDot(v_los, S_E); 
		LOS_ENU.y = vectorDot(v_los, S_N); 
		LOS_ENU.z = vectorDot(v_los, S_V);
		double azimuth = atan2(LOS_ENU.x, LOS_ENU.y); // tan(azimuth) = 东 / 北
		double latIpp = PI / 2 - acos(cos(PI / 2 - latLeo) * cos(angle_Leo_Earth_Ipp) + sin(PI / 2 - latLeo) * sin(angle_Leo_Earth_Ipp) * cos(azimuth));// 计算穿刺点纬度
		double lonIpp = lonLeo + asin(sin(angle_Leo_Earth_Ipp) * sin(azimuth) / cos(latIpp));// 计算穿刺点经度
		latIpp = latIpp * 180 / PI;
		lonIpp = lonIpp * 180 / PI;
		double VTEC;
		if(!ionFile.getVTEC(t, latIpp, lonIpp, VTEC))
			return false;
		double mapFun = 1 / cos(angle_Earth_Leo_Ipp);
		double TEC = mapFun * VTEC * 1E16;
		//value = alpha * 40.3 / pow(frequence , 2 ) * TEC; // 一阶电离层修正量
		// 第二步，参考RINEX_RO软件、igrf13.f获得穿刺点处地磁矢量: Nm、Em、Um, 计算BJ
		int isv = 0;
		double dateAD = t.decimalYear();
		int itype = 2; 	
		double altIpp = LayerHeight/1000.0; // km，等效电离层高度即穿刺点高度
		// latIpp :[0 189]; lonIpp :[0 360]
		//lonIpp = lonIpp + 180;
		//latIpp = latIpp + 90;
		double Nm = 0.0;
		double Em = 0.0;
		double Um = 0.0;
		double Fm = 0.0;  // nT
		// main-field,geocentric
		IGRF13SYN(&isv, &dateAD, &itype, &altIpp, &latIpp, &lonIpp, &Nm, &Em, &Um, &Fm);  
		//To compute the projection vector between geomagnetic and satellite-receiver vector in the pierce point
		//printf("%f %f %f %f %f \n",altIpp, TEC, lonIpp, latIpp, Nm);
		//BLH blh; // 计算卫星大地坐标
		//TimeCoordConvert::XYZ2BLH(leoPos, blh);
		double xyz[3]={0.0},dx[3]={0.0};
	    double Norm_dx;
		//Vector in the satellite receiver direction
		dx[0] = gpsPos.x - leoPos.x;
		dx[1] = gpsPos.y - leoPos.y;
		dx[2] = gpsPos.z - leoPos.z;
		Norm_dx = sqrt(pow(dx[0],2) + pow(dx[1],2) + pow(dx[2],2) );
		dx[0] = dx[0]/Norm_dx;
		dx[1] = dx[1]/Norm_dx;
		dx[2] = dx[2]/Norm_dx;
		 // To compute terrestrial local vector to be used in the geomagnetic propagation vector
	    POS3D Nt, Et, Ut;
        TimeCoordConvert::Local_Terrestrial_Vector(leoPos.x, leoPos.y, leoPos.z, Nt, Et, Ut);
		//Local geomagnetic coordinates aligned with terrestrial system
		xyz[0] = Nm*Nt.x + Em*Et.x + Um*Ut.x;
		xyz[1] = Nm*Nt.y + Em*Et.y + Um*Ut.y;
		xyz[2] = Nm*Nt.z + Em*Et.z + Um*Ut.z;
		//Projection vector between geomagnetic and satellite-receiver vector in the pierce point
		double BtJ = (xyz[0]*dx[0] + xyz[1]*dx[1] + xyz[2]*dx[2])*1.0e-9;  // nT -> T (Tesla)
		// 第三步，计算二阶电离层修正量
		double value1 = 0.0;
		double  e = 1.60218E-19,  //Electron charge - Coloumb
        A = 80.6,         // Constant  m^3/s^2
        me = 9.10939E-31,  //electron Mass - kg
        //const1 = (e*A)/(pow(frequence,3)*2.0*PI*me);
		const1 = (e*A)/(2.0*PI*me);
		value1 = const1*BtJ*TEC;
		//第四步，计算三阶电离层修正量
		double value2 = 0.0;
  //      double conste = ( 3.0*pow(A,2)*0.66 )/( 8.0*pow(frequence,4));
		//double conste = ( 3.0*pow(A,2)*0.66 )/( 8.0);
  //      //electron density maximum in function of the TEC
  //      double Ne_max = (((20.0-6.0)*1.0E12 )/((4.55-1.38)*1.0E18));
		//value2 = conste * Ne_max * TEC;
		//第五步，计算总的电离层修正量
		value = value1 + value2;
		return true;
	}

	//   子程序名称： RelativityCorrect   
	//   作用：广义相对论修正，在一个统一的地心坐标系下进行
	//   类型：
	//         satPos          :  卫星在地心系位置（米）
	//         staPos          :  测站在地心系位置（米）
	//         gamma           :  相对论修正因子，一般情况下取1
	//   输入：satPos, staPos
	//   输出：dR_relativity
	//   其它：
	//   语言： C++
	//   版本号：2013.7.10
	//   生成者：谷德峰
	//   修改者：
	//   修改记录：2013.12.5,去除太阳引力的影响
	double GNSSBasicCorrectFunc::graRelativityCorrect(POS3D satPos, POS3D staPos, double gamma)
	{
		double cc = pow(SPEED_LIGHT, 2); // 光速的平方
		// 太阳引力场引起的相对论修正			
		POS3D  V_sat_sta = satPos - staPos;
		double r   = sqrt(V_sat_sta.x * V_sat_sta.x + V_sat_sta.y * V_sat_sta.y + V_sat_sta.z * V_sat_sta.z); // 卫星到观测站的距离
		// 地球引力场引起的相对论修正
		double r1 = sqrt(satPos.x * satPos.x + satPos.y * satPos.y + satPos.z * satPos.z); // 地心到卫星的距离
		double r2 = sqrt(staPos.x * staPos.x + staPos.y * staPos.y + staPos.z * staPos.z); // 地心到观测站的距离
		double dR2 = (GM_EARTH / cc) * log((r1 + r2 + r)/(r1 + r2 - r));
		double dR_relativity = (1 + gamma) * dR2;
		return dR_relativity; 

	 }
	 // 相位缠绕修正，GPS卫星轨道系
	 double GNSSBasicCorrectFunc::correctLeoAntPhaseWindUp_new(POS6D receiverPosVel, POS3D gpsPos, POS3D sunPos, double prev)
	 {
	  double windup = 0;
	  // 计算GPS卫星星体系，所有类型卫星一致
	  POS3D vecRs =  sunPos - gpsPos; // 太阳矢量, 由GPS卫星指向太阳     
	  POS3D ez =  vectorNormal(gpsPos) * (-1.0);
	  POS3D ns =  vectorNormal(vecRs);
	  POS3D ey;
	  vectorCross(ey, ez, ns); 
	  ey = vectorNormal(ey);
	  POS3D ex;
	  vectorCross(ex, ey, ez);
	  ex = vectorNormal(ex);
	  // 计算LEO卫星星体系
	  POS3D ex_LEO = vectorNormal(receiverPosVel.getVel());
	  POS3D ez_LEO = vectorNormal(receiverPosVel.getPos() *(-1.0));
	  POS3D ey_LEO;
	  vectorCross(ey_LEO, ez_LEO, ex_LEO); 
	  ey_LEO = vectorNormal(ey_LEO);    
	  POS3D unitXr = ex_LEO;
	  POS3D unitYr = ey_LEO *(-1.0);  
	  // 计算相位缠绕
	  POS3D vecLos = vectorNormal(receiverPosVel.getPos() - gpsPos);
	  POS3D k_ey_GPS;
	  POS3D k_ey_Rec;
	  vectorCross(k_ey_GPS, vecLos, ey);
	  vectorCross(k_ey_Rec, vecLos, unitYr); 
	  POS3D D_R = unitXr  - vecLos * (unitXr.x * vecLos.x + unitXr.y * vecLos.y + unitXr.z * vecLos.z) + k_ey_Rec;
	  POS3D D_T = ex - vecLos * (ex.x * vecLos.x + ex.y * vecLos.y + ex.z * vecLos.z) - k_ey_GPS;
	  POS3D DD;
	  vectorCross(DD, D_T, D_R);
	  double theta = vectorDot(vecLos, DD);
	  int sign = 1;
	  /*if(theta > 0)
	   sign = 1;
	  if(theta == 0)
	   sign = 0;*/
	  if(theta < 0)
	   sign = -1;
	  D_T = vectorNormal(D_T);
	  D_R = vectorNormal(D_R);
	  double cos_fai = D_T.x * D_R.x + D_T.y * D_R.y + D_T.z * D_R.z;
	  cos_fai = cos_fai >  1 ?  1 : cos_fai; // 2013/07/04, 确保 -1 <= cos_fai <= 1
	  cos_fai = cos_fai < -1 ? -1 : cos_fai;
	  windup = sign * acos(cos_fai) / (2 * PI);
	  if(prev != DBL_MAX && prev != 0.0)
	  {// 连续化, 20140427
	   //double d = windup - prev;
	   //windup -= int(d + (d < 0.0 ? -0.5 : 0.5));
	   windup = windup + floor(prev - windup + 0.5); // in cycle; 参考 rtklib 
	  }
	  return windup;
	 }
}
