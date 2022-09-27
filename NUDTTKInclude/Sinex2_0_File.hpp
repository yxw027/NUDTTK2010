#pragma once
//#include "constDef.hpp"
#include "structDef.hpp"
#include <string>
#include <vector>
#include <map>
#include "Matrix.hpp"
#include <limits>
#include <windows.h>

//using namespace std;
//  Copyright 2014, The National University of Defense Technology at ChangSha
namespace NUDTTK
{
	struct Sinex2_0_BlockLabel
	{
		//static const char   szFileID[];
		static const char   szFileRef[];           // 文件信息
		static const char   szFileComment[];
		static const char   szInputHistory[];
		static const char   szInputFiles[];
		static const char   szInputAck[];
		static const char   szInputAck_NOE[];
		static const char   szNutationData[];
		static const char   szPrecessionData[];
		static const char   szSourceID[];
		static const char   szSiteID[];
		static const char   szSiteData[];
		static const char   szSiteRec[];
		static const char   szSiteAnt[];
		static const char   szSiteGPSPCO[];
		static const char   szSiteGALPCO[];
		static const char   szSiteAntARP[];        //天线位置UNE
		static const char   szSatID[];     
		static const char   szSatPCO[];  
		static const char   szSolutEpochs[]; 
		static const char   szBiasEpochs[]; 
		static const char   szSolutStatistics[]; 
		static const char   szSolutEst[];          // 估计值
		static const char   szSolutApri[];          // 先验值
		static const char   szSolutMatrixEstLCORR[]; 
		static const char   szSolutMatrixEstUCORR[]; 
		static const char   szSolutMatrixEstLCOVA[]; 
		static const char   szSolutMatrixEstUCOVA[];
		static const char   szSolutMatrixEstLINFO[]; 
		static const char   szSolutMatrixEstUINFO[]; 
		static const char   szSolutMatrixApriLCORR[]; 
		static const char   szSolutMatrixApriUCORR[]; 
		static const char   szSolutMatrixApriLCOVA[]; 
		static const char   szSolutMatrixApriUCOVA[]; 
		static const char   szSolutMatrixApriLINFO[]; 
		static const char   szSolutMatrixApriUINFO[]; 
		static const char   szSolutNEQVector[]; 
		static const char   szSolutNEQMatrixL[];
		static const char   szSolutNEQMatrixU[];		
		static const char   szBlockSeparator[];			 
		///以下内容用于FileRefBlock
		static const char   szDes[];                // 信息种类，1X,A18 
		static const char   szOutput[];             // 输出文件
		static const char   szInput[];              // 描述文件内容
		static const char   szContact[];            // 邮件
		static const char   szSoftware[];           // 软件
		static const char   szHardware[];           // 硬件
		static const char   szRefFrame[];           // 参考框架
		static const char   szObsCount[];          // 观测数据个数1X,A30 
		static const char   szObsUnknown[];        // 观测数据个数
		///以下内容用于SolutStatisticsBlock
		static const char   szObsFreedom[];        // 
		static const char   szFreedom[];           // 
		static const char   szSampInterval[];      // 采样间隔，秒
		static const char   szSampIntervalUnit[];  // 采样间隔，秒
		static const char   szSquareRes[];         // 残差平方和
		static const char   szSquaredRes[];        // 残差平方和
		static const char   szPhaseSigma[];        // 相位权重
		static const char   szCodeSigma[];         // 伪距权重
		static const char   szVarFactor[];         // szSqrResiduals/szObsFreedom
		static const char   szWSquareRes[];        // 带权重的残差平方和
		// 以下内容用于TROZPD文件
		static const char   szTroDescription[];     
		static const char   szTroStaCoordinate[]; 
		static const char   szTroCentersInfo[];
		static const char   szTroCentersModel[];
		static const char   szTroSolution[];  
		// 以下内容用于TROZPD TRODES文件
		static const char   szTROSampInterval[];		// 数据采样间隔，1X,A29 ，second
		static const char   szTroInterval[];			// 对流层采样间隔
		static const char   szEleCutoff[];           // 高度截止角
		static const char   szTroMapFunc[];          // 对流层估计使用的映射函数        
		static const char   szSolutField1[];         // 对流层产品的种类,最多放7个产品名，用空格隔开
		static const char   szSolutField2[];         // 对流层产品的种类	
	};

	struct SinexFileTime //表示UTC时间
	{
		int			year;						  //年,最后两位，<=50,+2000;>50,+1900
		int			doy;						  //年积日
		int			second;						  //天内秒
		UTC         toUTC();
		SinexFileTime()
		{
			year   = 0;
			doy    = 0;
			second = 0;
		}		
	};

	struct Sinex2_0_HeaderLine
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
		int             EstParaCount;              // 估计的参数个数，1X,I5.5
		char            szConstCode[1 + 1];        // 约束条件，1X,A1 
		vector<char>    pszSolutCont;              // 解算的参数总类，6(1X,A1)		
		Sinex2_0_HeaderLine()
		{
			memset(this,0,sizeof(Sinex2_0_HeaderLine));			
			pszSolutCont.resize(6);	
			pszSolutCont[0] = ' ';
			pszSolutCont[1] = ' ';
			pszSolutCont[2] = ' ';
			pszSolutCont[3] = ' ';
			pszSolutCont[4] = ' ';
			pszSolutCont[5] = ' ';
		}
		//char            szMarkerName[4 + 1];       // 测站名，1X,A4

	};
	//Sinex文件的相关信息结构
	struct FileRef
	{		
		char			    szDesInfo[60 + 1];      // 信息，1X,A60 
		char			    szOutputInfo[60 + 1];   // 信息，1X,A60 
		char			    szInputInfo[60 + 1];    // 信息，1X,A60 
		char			    szContactInfo[60 + 1];  // 信息，1X,A60 
		char			    szSoftwareInfo[60 + 1]; // 信息，1X,A60 
		char			    szHardwareInfo[60 + 1]; // 信息，1X,A60 
		char			    szRefFrameInfo[60 + 1]; // 信息，1X,A60 

		bool                bBlockUse;              // 是否有这个数据模块           
		FileRef()
		{
			memset(this,0,sizeof(FileRef));			
			bBlockUse = false;
		}
	};
	// 输入文件结构
	struct InputFile
	{
		char            szFileAgency[3 + 1];       // 生成文件的机构，1X,A3
		SinexFileTime   FileTime;                  // 生成文件的时间，1X,I2.2,':',I3.3,':',I5.5
		char            szFileName[29 + 1];        // 文件名， 1X,A29
		char            szFileDes[32 + 1];         // 文件描述， 1X,A32 
		InputFile()
		{
			memset(this,0,sizeof(InputFile));
		}
	};
	//INPUT/ACKNOWLEDGEMENTS 信息结构
	struct InputAck
	{		
		char			szAgencyCode[3 + 1];       // 机构，1X,A3
		char			szAgencyDes[75 + 1];       // 信息，1X,A75  
		InputAck()
		{
			memset(this,0,sizeof(InputAck));
		}
	};
	// 岁差章动结构
	struct Nut_Pre
	{
		char			szModelCode[8 + 1];        // 岁差或章动模型，1X,A8 
		char			szModelDes[70 + 1];        // 模型描述，1X,A70
		Nut_Pre()
		{
			memset(this,0,sizeof(Nut_Pre));
		}
	};
	struct SiteID
	{
		char			szSiteCode[4 + 1];         // 测站名，1X,A4
		char			szPointCode[2 + 1];        // 1X,A2 
		char			szMonumentID[9 + 1];       // 测站编号，1X,A9
		char			szObsCode[1 + 1];          // 观测资料
		char			szStaDes[22 + 1];          // 测站位置描述， 1X,A22 
		int				LonDegree;                 // 测站经度，度，0,360，1X,I3,
		int				LonMinute;                 // 测站经度，分，1X,I2,
		double			LonSecond;                 // 测站经度，秒，1X,F4.1
		int				LatDegree;                 // 测站纬度，度  +-90
		int				LatMinute;                 // 测站纬度，分
		double			LatSecond;                 // 测站纬度，秒
		double			Height;                    // 测站高，m, 1X,F7.1
		SiteID()
		{
			memset(this,0,sizeof(SiteID));
		}
	};
	struct SiteRecAntARPEpoch
	{
		char			szSiteCode[4 + 1];         // 测站名，1X,A4
		char			szPointCode[2 + 1];        // 1X,A2 
		char			szSolutID[4 + 1];          // 解算编号，1X,A4
		char			szObsCode[1 + 1];          // 观测资料
		SinexFileTime   StartTimeSolut;			   // 参与解算数据的起始时间，1X,I2.2,':',I3.3,':',I5.5
		SinexFileTime   EndTimeSolut;			   // 参与解算数据的结束时间，1X,I2.2,':',I3.3,':',I5.5

		SinexFileTime   MeanTime;                  // 平均时间，用于Solut/BiasEpoch

		char			szType[20 + 1];            // 接收机或天线类型， 1X,A20   ，用于RecAnt
		char			szSerial[5 + 1];           // 接收机或天线序号， 1X,A5    ，用于RecAnt
		char			szFirmware[11 + 1];        // 接收机硬件， 1X,A11   ，用于Rec

		char            szRefSys[3 + 1];           // 参考系统，UNE，XYZ     ,用于ARP
		ENU             AntARP;                    // 天线相对于接收机的位置 ,用于ARP

		
		SiteRecAntARPEpoch()
		{
			memset(this,0,sizeof(SiteRecAntARPEpoch));
		}
	};
	typedef map<string, SiteRecAntARPEpoch>      SiteRecAntARPEpochMap;  
	                                               // 接收机或解算EpochMap 
	struct SiteData
	{
		char			szSiteCode[4 + 1];         // 测站名，1X,A4
		char			szPointCode[2 + 1];        // 1X,A2 
		char			szSolutID[4 + 1];          // 解算编号，1X,A4		
		char			szSiteCodeIn[4 + 1];       // 测站名，1X,A4
		char			szPointCodeIn[2 + 1];      // 1X,A2 
		char			szSolutIDIn[4 + 1];        // 解算编号，1X,A4
		char			szObsCodeIn[1 + 1];        // 观测资料
		SinexFileTime   StartTimeSolutIn;		   // 参与解算数据的起始时间，1X,I2.2,':',I3.3,':',I5.5
		SinexFileTime   EndTimeSolutIn;			   // 参与解算数据的结束时间，1X,I2.2,':',I3.3,':',I5.5
		SinexFileTime   FileTimeIn;                // 输入文件生成时间
		char            szAgencyCodeIn[3 + 1];       // 生成输入文件的机构			
		SiteData()
		{
			memset(this,0,sizeof(SiteData));
		}
		
	};	     

	typedef map<BYTE, ENU>      RecFreqPCOMap;      // 接收机不同频率PCOMap
	struct SitePCO
	{
		char			szAntType[20 + 1];         // 天线类型， 1X,A20 
		char			szAntSerial[5 + 1];        // 天线序号， 1X,A5
		RecFreqPCOMap   PCO;                       // L1,PCO，1X,F6.4 ,L2,PCO，1X,F6.4	
		char            PCVCorModel[10 + 1];       // 天线PCV模型		
		//对于Galileo,有三行，分别是：L1和L5；L6和L7；L8
	};
	typedef map<string, SitePCO>      RecAntPCOMap;// 接收机天线PCO,方便根据天线类型索引。
	                                               // 但根据格式说明，若天线类型相同，序号不同时PCO可能不同，此时这种表示方法不妥。20140907，刘俊宏
	                                               // 目前还没有发现这种情况
	struct SourceID
	{
		char            szSourceCode[4 + 1];       // 1X,A4
		char            szIERSDes[8 + 1];          // 1X,A8
		char            szICRFDes[16 + 1];         // 1X,A16
		char            szComments[68 + 1];        // 1X,A68
		SourceID()
		{
			memset(this,0,sizeof(SourceID));
		}	//	
	};

	struct SatID
	{
		char			szSiteCode[4 + 1];         // 卫星编号，1X,A4
		char			szPRN[2 + 1];              // PRN，1X,A2 
		char			szCorparID[9 + 1];         // 发射信息，1X,A9
		char			szObsCode[1 + 1];          // 观测资料
		SinexFileTime   StartTimeSolut;			   // 参与解算数据的起始时间，1X,I2.2,':',I3.3,':',I5.5
		SinexFileTime   EndTimeSolut;			   // 参与解算数据的结束时间，1X,I2.2,':',I3.3,':',I5.5		
		char			szAntType[20 + 1];         // 天线类型， 1X,A20 
		SatID()
		{
			memset(this,0,sizeof(SatID));
		}	//	
	};
	typedef map<string, SatID>      SatIDMap;      // 卫星IDMap,方便将PRN号与SiteCode关联起来<系统标记+PRN(例如G01)，SatID>

	typedef map<BYTE, POS3D>      SatFrePCOMap;    // 卫星PCOMap
	struct SatPCO
	{
		char			szSiteCode[4 + 1];         // 卫星编号CNNN，1X,A4
		SatFrePCOMap    PCO;
		char            PCVCorModel[10 + 1];       // 天线PCV修正模型
		char            PCVType[1 + 1];            // PCV类型 
		char            PCVAppModel[1 + 1];        // PCV修正模型类型 F--full PCV model applied，E-elevation-dependent		
	};
	typedef map<string, SatPCO>   SatPCOMap;       // 卫星PCOMap,根据卫星CNNN号索引

	struct SolutSatistics
	{		
		int                 obsCount;              // 1X,F22.15 
		int                 unknowns;
		int                 freedoms;
		int                 sampInterval;
		double              squareRes;
		double              phaseSigma;
		double              codeSigma;
		double              varFactor;
		double              wSquareRes;
		bool                bBlockUse;             // 是否有这个数据模块            
		SolutSatistics()
		{
			obsCount     = INT_MAX;
			unknowns   = INT_MAX;
			freedoms   = INT_MAX;
			sampInterval = INT_MAX;
			squareRes    = DBL_MAX;
			phaseSigma   = DBL_MAX;
			codeSigma    = DBL_MAX;
			varFactor    = DBL_MAX;
			wSquareRes   = DBL_MAX;
			bBlockUse    = false;
			//memset(this,0,sizeof(SolutSatistics));
		}	
	};
	struct SolutVector
	{
		int				Index;                     // 估计参数的序号， 1X,I5
		char			ParaType[6 + 1];           // 参数类型，1X,A6 
		char			szSiteCode[4 + 1];         // 测站名，1X,A4
		char			szPointCode[2 + 1];        // 1X,A2 
		char			szSolutID[4 + 1];          // 解算编号，1X,A4		
		SinexFileTime   EpochTime;				   // 历元的有效时间，1X,I2.2,':',I3.3,':',I5.5
		char            Unit[4 + 1];               // 单位，1X,A4 
		char            szConstCode[1 + 1];        // 约束条件，1X,A1 
		double          ParaValue;                 // 参数的估计值或先验值，1X,E21.15 

		double          ParaSTD;                   // 估计值或先验值的标准差，1X,E11.6,仅SolutEst和SolutApriori
		SolutVector()
		{
			memset(this,0,sizeof(SolutVector));
			ParaValue = DBL_MAX;
			ParaSTD   = DBL_MAX;
		}	
	};
	//struct EOPdata
	//{		
	//	double  pm_x;    // 单位: 角秒
	//	double  pm_y;    // 单位: 角秒    
	//	double  ut1_utc; // 单位: 秒
	//};//
	struct Sinex2_0_Data
	{
		FileRef                       m_FileRef;
		vector<string>                m_Comment;
		vector<Sinex2_0_HeaderLine>   m_InputHistory;
		vector<InputFile>             m_InputFile;
		vector<InputAck>              m_InputAck;
		vector<Nut_Pre>               m_Nutaion;
		vector<Nut_Pre>               m_Precession;
		vector<SourceID>              m_SourceID;
		vector<SiteID>                m_SiteID;
		vector<SiteData>              m_SiteData;
		SiteRecAntARPEpochMap         m_SiteRec;
		SiteRecAntARPEpochMap         m_SiteAnt;
		RecAntPCOMap                  m_SiteGPSPCO;
		RecAntPCOMap                  m_SiteGALPCO;      //暂不处理
		SiteRecAntARPEpochMap         m_SiteAntARP;
		SatIDMap                      m_SatID;
		SatPCOMap                     m_SatPCO;
		SiteRecAntARPEpochMap         m_SolutEpoch;
		SiteRecAntARPEpochMap         m_BiasEpoch;
		SolutSatistics                m_SolutSatistics;
		vector<SolutVector>           m_SolutEst;
		vector<SolutVector>           m_SolutApri;
		vector<SolutVector>           m_SolutNEQVector;
		Matrix                        m_MatrixEst;        // 相关系数矩阵或协方差或INFO	
		Matrix                        m_MatrixApri;       // 相关系数矩阵或协方差或INFO	
		Matrix                        m_MatrixNEQ;        // NEQ矩阵
		int                           MatrixEstType;      // 1-LCORR,2-UCORR,3-LCOVA,4-UCOVA,5-LINFO,6-UINFO,0-无此矩阵
		int                           MatrixApriType;     // 1-LCORR,2-UCORR,3-LCOVA,4-UCOVA,5-LINFO,6-UINFO,0-无此矩阵
		int                           MatrixNEQType;      // 1-L,2-U,0-无此矩阵
		Sinex2_0_Data()
		{
			//memset(this,0,sizeof(Sinex2_0_Data));//含有map的结果不能使用这个命令，否则map不能使用
			m_FileRef = FileRef::FileRef();
			m_Comment.clear();
			m_InputHistory.clear();
			m_InputFile.clear();
			m_InputAck.clear();
			m_Nutaion.clear();
			m_Precession.clear();
			m_SourceID.clear();
			m_SiteID.clear();
			m_SiteData.clear();
			m_SiteRec.clear();
			m_SiteAnt.clear();
			m_SiteGPSPCO.clear();
			m_SiteGALPCO.clear();    
			m_SiteAntARP.clear();
			m_SatID.clear();
			m_SatPCO.clear();
			m_SolutEpoch.clear();
			m_BiasEpoch.clear();
			m_SolutSatistics = SolutSatistics::SolutSatistics();
			m_SolutEst.clear();
			m_SolutApri.clear();
			m_SolutNEQVector.clear();		
			MatrixEstType     = 0;
			MatrixApriType    = 0;
			MatrixNEQType     = 0;
		}
	};

	class Sinex2_0_File
	{
	public:
		Sinex2_0_File(void);
	public:
		~Sinex2_0_File(void);
	public:
		void    	 clear();
		bool    	 isEmpty();
		static void  stringEraseFirstZero(const char* szFloat, string& strFloat);
		bool   		 open(string  strSNXFileName);
		bool   		 write(string strSNXFileName);
		bool   		 getStaPos(string name, POS3D &pos, POS3D &posSTD,bool bEst = true);		
		bool   		 getRecAntARP(string name, ENU  &antARP);
		bool   		 getGPSRecAntPCO(string name, BYTE  freIndex, ENU &antPCO);		
		bool   		 getSatPCOApri(string satName, BYTE freIndex, POS3D  &satPCOApri);
		bool   		 getSatPCOEst(string satName, POS3D  &satPCOEst, POS3D &satPCOSTD, string pointCode = "LC");
		bool   		 getEOPParaEst();
	public:
		Sinex2_0_HeaderLine         m_header;
		Sinex2_0_Data               m_data;
	};

}