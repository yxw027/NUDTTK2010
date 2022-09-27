#pragma once
#include"structDef.hpp"
#include <vector>
#include <map>
#include "Matrix.hpp"

namespace NUDTTK
{
	namespace Math
	{
		// 行号
		struct PE_ROW
		{
			//每个行元素用Nxx_row记录，double只记录index大于当前行号的元素（对称）
			map<int, double>  Nxx_row;
			double           nx;
		};

		struct PE_ELIEDROW
		{
			double           Naa;
			map<int, double>  Nab;
			double           na;
		};

		// 每消去一维需要用vector进行保存Nxx、Nxb、nx

		// 每个参数索引号对应的时间区间, 用于参数消去的排序,由外部输入
		struct PE_INTERVAL
		{
			int   id; // 索引号 
			GPST  t0; // 参数时间区间起点
			GPST  t1; // 参数时间区间终点
		};

		// 使用步骤: 1.输入m_intervalList; 2.调用init函数进行排序; 3.调用plusElement_Nxx和plusElement_nx进行赋值; 4.调用main_ParaElimination参数消去求解
		class ParaEliminationNEQ
		{
		public:
			ParaEliminationNEQ(void);
		public:
			~ParaEliminationNEQ(void);

		public:
			vector<PE_INTERVAL> m_intervalList; // 外部输入，用于参数消去的排序使用, 生成 m_sequenceList 和 m_idList
			void init();
			void plusElement_Nxx(int id_row, int id_col, double value); // 法方程元素叠加
			void plusElement_nx(int id_row, double value); // 法方程元素叠加
			bool main_ParaElimination(Matrix &dx);        // 输出解算结果, 调整到原始顺序

        private:
			vector<int>           m_sequenceList;  // 使用时输入原始id, 获得参数消去顺序 sequence
			vector<int>           m_idList;        // 使用时输入序号se, 获得参数id
			vector<PE_ROW>        m_data;          // 记录原始的法方程数据
			map<int, double>      m_dx;            // 记录更新的解         
			vector<PE_ELIEDROW>   m_EliedDataList; // 记录已消去的数据, 用于还原
		};
	}
}
