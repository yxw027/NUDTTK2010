#pragma once
#include "Sinex2_0_File.hpp"
#include "constDef.hpp"
#include <string>
#include <vector>

namespace NUDTTK
{
	struct TROZPDHeader
	{
		char            szFirstchar[1 + 1];            // %,( )
		char            szSecondchar[1 + 1];           // =,(+)
		char            szDocType[3 + 1];          // 文件种类，SNX  A3
		double          Version;                   // 文件格式版本，1X,F4.2
		char            szFileAgency[3 + 1];       // 生成文件的机构，1X,A3
		SinexFileTime   FileTime;                  // 生成文件的时间，1X,I2.2,':',I3.3,':',I5.5
		char            szDataAgency[3 + 1];       // 提供数据生成该文件的机构，1X,A3
		SinexFileTime   StartTimeSolut;            // 参与解算数据的起始时间，1X,I2.2,':',I3.3,':',I5.5
		SinexFileTime   EndTimeSolut;              // 参与解算数据的结束时间，1X,I2.2,':',I3.3,':',I5.5
		char            szObsCode[1 + 1];          // 观测资料，1X,A1，P(GNSS),C(Combined techniques),D(DORIS),L(SLR),M(LLR),R(VLBI)		
		char            szSolutCont[4 + 1];        // 测站名，1X,A4;"MIX"多个测站
		TROZPDHeader()
		{
			memset(this,0,sizeof(TROZPDHeader));					
		}
	};
	struct TroDes
	{		
		int                sampInterval;            // 1X,22I
		int                troInterval;             // 1X,22I
		double             eleCutoff;               // 1X,F22.1
		char               szMapFunc[22 + 1];       // 1X,A22
		vector<string>     pstrSolutField;          // 对流层产品的种类
		vector<string>     pstrSolutFieldNor;       // 常用的对流层产品种类
		bool               bMapFunc;                // 是否有MapFunc的描述

		TroDes()
		{		
			sampInterval = INT_MAX;
			troInterval  = INT_MAX;
			eleCutoff    = DBL_MAX;
			bMapFunc     = false;
			pstrSolutField.clear();                
			pstrSolutFieldNor.resize(6);           // 目前常用的为6种产品(4种产品类型)
			pstrSolutFieldNor[0] = "TROTOT";
			pstrSolutFieldNor[1] = "STDEV";
			pstrSolutFieldNor[2] = "TGNTOT";
			pstrSolutFieldNor[3] = "STDEV";
			pstrSolutFieldNor[4] = "TGETOT";
			pstrSolutFieldNor[5] = "STDEV";

		}	
	};
	// 测站坐标结构
	struct TroStaPos
	{
		char			szSiteCode[4 + 1];         // 测站名，1X,A4
		char			szPointCode[2 + 1];        // 1X,A2 
		char			szSolutID[4 + 1];          // 解算编号，1X,A4
		char			szObsCode[1 + 1];          // 观测资料，1X,A1
		POS3D           pos;                       // 测站坐标，3(1X,F12.3)
		char            szRefSys[6 + 1];           // 测站坐标的参考系统，1X,A6
		char            szRemark[5 + 1];           // origin of the coordinates,1X,A5 
		POS3D           posSTD;                    // 测站的标准差,mm,3(1X,I2)
		int             counterAC;                 // AC 的个数
		TroStaPos()
		{
			memset(this,0,sizeof(TroStaPos));
		}
 	};
	typedef map<string, TroStaPos>      TroStaPosMap;

	struct TroSolut
	{//数据结构可能有很多种情况，暂时仅考虑常用的一种情况，TROTOT STDEV  TGNTOT  STDEV  TGETOT  STDEV
		char            szMarker[4 + 1];          // 测站名
		SinexFileTime   EpochTime;                // 历元时刻
		double          TROTOT;                   // 天顶延迟总和
		double          TROTOTSTD;                // 天顶延迟总和标准差
		double          TGNTOT;                   // 天顶延迟N向梯度总和
		double          TGNTOTSTD;                // 天顶延迟N向梯度总和标准差
		double          TGETOT;                   // 天顶延迟E向梯度总和
		double          TGETOTSTD;                // 天顶延迟E向梯度总和标准差
		TroSolut()
		{
			memset(this,0,sizeof(TroSolut));
		}
	};
	typedef vector<TroSolut>               TroSolutList;
	typedef map<string, TroSolutList>      TroSolutMap;

	struct TROZPDData
	{
		FileRef                       m_FileRef;
		vector<string>                m_Comment;		
		vector<InputAck>              m_InputAck;		
		vector<SiteID>                m_SiteID;
		vector<SiteData>              m_SiteData;
		SiteRecAntARPEpochMap         m_SiteRec;
		SiteRecAntARPEpochMap         m_SiteAnt;
		RecAntPCOMap                  m_SiteGPSPCO;		
		SiteRecAntARPEpochMap         m_SiteAntARP;
		TroDes                        m_TroDes;	
		TroStaPosMap                  m_StaPos;
		TroSolutMap                   m_TroSolut;

		
		TROZPDData()
		{			
			m_FileRef = FileRef::FileRef();
			m_Comment.clear();		
			m_InputAck.clear();			
			m_SiteID.clear();			
			m_SiteRec.clear();
			m_SiteAnt.clear();
			m_SiteGPSPCO.clear();			
			m_SiteAntARP.clear();
			m_TroDes.pstrSolutField.clear();
			m_StaPos.clear();
			m_TroSolut.clear();
		}
	};

	class TROZPDFile
	{
	public:
		TROZPDFile();
	public:
		~TROZPDFile();
	public:
		void    clear();
		bool    isEmpty();
		bool    open(string  strTROZPDFileName);
		bool    write(string strTROZPDFileName);
		bool    getTroZPDSolut(string name, UTC t, TroSolut &t_tro,double t_forecast = 3600.0);
	public:
		TROZPDHeader         m_header;
		TROZPDData           m_data;
	};

}