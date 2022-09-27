#include "LLRPreproc.hpp"
#include <math.h>
#include "LeoGPSObsPreproc.hpp"
#include "TimeCoordConvert.hpp"
#include "OceanTidesLoading.hpp"
#include "SolidTides.hpp"
#include "GNSSBasicCorrectFunc.hpp"

using namespace NUDTTK::SpaceborneGPSPreproc;
namespace NUDTTK
{
	namespace SLR
	{
		LLRPreproc::LLRPreproc(void)
		{
		}

		LLRPreproc::~LLRPreproc(void)
		{
		}
		// 子程序名称： relativityCorrect 
		// 功能：广义相对论修正, 在一个统一的太阳质心坐标系下进行
		// 变量类型：earthPos        :  地球在太阳质心系位置（米）
		//           lunarStaPos     :  月球角反射器在太阳质心系位置（米）
		//           earthStaPos     :  地球观测站在太阳质心系位置（米）
		//           Gamma           :  相对论修正因子，一般情况下取1
		// 输入： earthPos, lunarStaPos, earthStaPos
		// 输出： dR_relativity
		// 语言：C++
		// 创建者：邵凯
		// 创建时间：2021/10/22
		// 版本时间：
		// 修改记录：
		// 备注：
		double LLRPreproc::relativityCorrect(POS3D earthPos, POS3D lunarStaPos, POS3D earthStaPos, double Gamma)
		{
			double cc = pow(SPEED_LIGHT, 2); // 光速的平方
			POS3D  v_lunarSta_earthSta = lunarStaPos - earthStaPos;
			double r   = sqrt(v_lunarSta_earthSta.x * v_lunarSta_earthSta.x + v_lunarSta_earthSta.y * v_lunarSta_earthSta.y 
				         + v_lunarSta_earthSta.z * v_lunarSta_earthSta.z); // 月球角反射器到地球观测站的距离
			// 太阳引力场引起的相对论修正
			double r1 = sqrt(lunarStaPos.x * lunarStaPos.x + lunarStaPos.y * lunarStaPos.y + lunarStaPos.z * lunarStaPos.z); // 太阳质心到月球角反射器的距离
			double r2 = sqrt(earthStaPos.x * earthStaPos.x + earthStaPos.y * earthStaPos.y + earthStaPos.z * earthStaPos.z); // 太阳质心到地球观测站的距离
			double dR1 = (GM_SUN / cc) * log((r1 + r2 + r)/(r1 + r2 - r));
			// 地球引力场引起的相对论修正
			POS3D  v_earth_lunarSta = earthPos - lunarStaPos;
			r1  = sqrt(v_earth_lunarSta.x * v_earth_lunarSta.x + v_earth_lunarSta.y * v_earth_lunarSta.y 
				            + v_earth_lunarSta.z * v_earth_lunarSta.z); // 地球到月球角反射器的距离
			POS3D  v_earth_earthSta = earthPos - earthStaPos;
			r2  = sqrt(v_earth_earthSta.x * v_earth_earthSta.x + v_earth_earthSta.y * v_earth_earthSta.y 
				         + v_earth_earthSta.z * v_earth_earthSta.z); // 地球到地球观测站的距离
			double dR2 = (GM_EARTH / cc) * log((r1 + r2 + r)/(r1 + r2 - r));
			double dR_relativity = (1 + Gamma) * (dR1 + dR2);  // 
			return dR_relativity; 
		}

	}
}
