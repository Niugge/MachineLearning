// 本程序对三维散点数据进行椭球拟合
//六个面的六个点。多了行不通
/*
* @参考：http://www.51hei.com/bbs/dpj-97791-1.html
*
*/
#include "stdio.h"
#include "string.h"
#include "math.h"

#define MATRIX_SIZE 7
#define u8 unsigned char

double m_matrix[MATRIX_SIZE][MATRIX_SIZE + 1];
int m = MATRIX_SIZE;
int n = MATRIX_SIZE + 1;
double m_result[MATRIX_SIZE];

void DispMatrix(void);

double Abs(double a)
{
	return a<0 ? -a : a;
}

u8 Equal(double a, double b)
{
	return Abs(a - b) < 1e-6;
}

void ResetMatrix(void)
{
	int row, column;

	for (row = 0; row<m; row++) {
		for (column = 0; column<n; column++)
			m_matrix[row][column] = 0.0f;
	}
}

void CalcData_Input(double x, double y, double z)
{
	double V[MATRIX_SIZE];
	int row, column;

	V[0] = x*x;
	V[1] = y*y;
	V[2] = z*z;
	V[3] = x;
	V[4] = y;
	V[5] = z;
	V[6] = 1.0;

	//构建VxVt矩阵(Vt为V的转置)，并进行累加
	for (row = 0; row<MATRIX_SIZE; row++) {
		for (column = 0; column<MATRIX_SIZE; column++) {
			m_matrix[row][column] += V[row] * V[column];
		}
	}
}

void SwapRow(int row1, int row2)
{
	int column;
	double tmp;

	for (column = 0; column<n; column++) {
		tmp = m_matrix[row1][column];
		m_matrix[row1][column] = m_matrix[row2][column];
		m_matrix[row2][column] = tmp;
	}
}

void MoveBiggestElement2Top(int s_row, int s_column)
{
	int row, column;

	for (row = s_row + 1; row<m; row++) {
		if (Abs(m_matrix[s_row][s_column])<Abs(m_matrix[row][s_column])) {
			SwapRow(s_row, row);
		}
	}
}

//高斯消元法，求行阶梯型矩阵
u8 Matrix_GaussElimination(void)
{
	int row, column, i, j;
	double tmp;
	static u8 count = 1;

	for (row = 0, column = 0; row<m - 1 && column<n - 1; row++, column++) {
		//将当前列最大的一行移上来
		MoveBiggestElement2Top(row, column);

		//整列都为0
		if (Equal(m_matrix[row][column], 0.0f)) {
			printf("qiyi matrix:%d %d\r\n", row, column);
			//DispMatrix();
			//return 0;
			row--;
			continue;
		}

		//高斯消元
		for (i = row + 1; i<m; i++) {
			if (Equal(m_matrix[i][column], 0.0f))
				continue;        //为0，无需处理

			tmp = m_matrix[i][column] / m_matrix[row][column];

			for (j = column; j<n; j++) {
				m_matrix[i][j] -= m_matrix[row][j] * tmp;
			}
		}

		printf("第%d次消元：\r\n", count++);
		DispMatrix();
		printf("\r\n");
	}

	return 1;
}

//求行最简型矩阵
int Matrix_RowSimplify(void)
{
	int c = n;//返回值，表示(解的任意常量数+1)；
			  //
	int row, column, k, s, t;
	float tmp;
	static u8 count = 1;
	//
	for (row = 1, column = 1; row<m && column<n; row++, column++)
	{
		if (Equal(m_matrix[row][column], 0))//平移，找出本行第一个非零；
		{
			row--;
			continue;
		}
		//
		c--;//少一个常量；
			//
			//这里不化成对角矩阵为1的矩阵，为了防止输入的数据较大的时候，求出的解为接近于0值的情况
			//tmp = 1 / m_matrix[row][column];
			//for(k=column;k<n;k++)//前面的"0"就不处理了；
			//m_matrix[row][k] *= tmp;
			//

			//化上三角矩阵为对角矩阵[/i][/i][/i]  [i][i][i] 
		if (row == m - 1)
			m_matrix[row][column] = 0.0f;        //强制为0，释放一个自由度，否则很难有解

		for (s = 0; s<row; s++)//下面的0也不用处理；
		{
			if (Equal(m_matrix[s][column], 0))
				continue;//已经为0；
						 //
			tmp = m_matrix[s][column] / m_matrix[row][column];
			for (t = column; t<n; t++)
				m_matrix[s][t] -= m_matrix[row][t] * tmp;
			//
		}
		printf("第%d次简化：\r\n", count++);
		DispMatrix();
		printf("\r\n");
	}

	DispMatrix();
	printf("\r\n");
	//
	return c;
}

void Matrix_Solve(double* C, double* sol)
{
	int row, column, i;
	int any_sol[MATRIX_SIZE];

	//找出任意解的位置
	memset(any_sol, 0, MATRIX_SIZE);
	for (row = 0, column = 0; row<m && column<n - 1; row++, column++) {
		if (Equal(m_matrix[row][column], 0.0f)) {
			any_sol[column] = 1;        //记录任意解的位置
			row--;        //右移1列
		}
	}

	//求解
	row = 0;
	for (column = 0; column<n - 1; column++) {
		if (any_sol[column] == 1) {        //任意解
			sol[column] = C[column];
		}
		else {
			sol[column] = m_matrix[row][n - 1];
			//加上任意解
			for (i = column + 1; i<n - 1; i++) {
				if (any_sol[i] == 1 && !Equal(m_matrix[row][i], 0.0f)) {
					sol[column] -= m_matrix[row][i] * C[i];
				}
			}

			sol[column] /= m_matrix[row][column];        //除以对角线元素

			row++;
		}
	}
}

void DispMatrix(void)
{
	int row, column;

	for (row = 0; row<m; row++) {
		for (column = 0; column<n; column++) {
			printf("%9.3f ", m_matrix[row][column]);
		}
		printf("\r\n");
	}
}

void Calc_Process(double radius)
{
	double C[MATRIX_SIZE];
	double Res[MATRIX_SIZE];
	int i;
	double k;

	ResetMatrix();

	//这里输入任意个点的罗盘(加速度)参数，尽量在球面上均匀分布(可使用APM的六面采集法采集)
	/*CalcData_Input(7, -7, -2);
	CalcData_Input(-1, -7, -2);
	CalcData_Input(3, 3, -2);
	CalcData_Input(3, -17, -2);
	CalcData_Input(3, -7, 4);
	CalcData_Input(3, -7, -8*/

	CalcData_Input(0, 0, -2.3);
	CalcData_Input(0, 0, 1.9);
	CalcData_Input(0, 2, 0);
	CalcData_Input(0, -2, 0);
	CalcData_Input(2, 0, 0);
	CalcData_Input(-2, 0, 0);
	//CalcData_Input(2.2, 0, 0);
	//CalcData_Input(-1.8, 0, 0);

	printf("原始矩阵：\r\n");
	DispMatrix();

	Matrix_GaussElimination();
	Matrix_RowSimplify();

	for (i = 0; i<MATRIX_SIZE; i++) {
		C[i] = 1000.0f;
	}

	Matrix_Solve(C, Res);

	printf("a:%.2f b:%.2f c:%.2f d:%.2f e:%.2f f:%.2f g:%.2f\r\n", Res[0], Res[1], Res[2], Res[3], Res[4], Res[5], Res[6]);

	k = (Res[3] * Res[3] / Res[0] + Res[4] * Res[4] / Res[1] + Res[5] * Res[5] / Res[2] - 4 * Res[6]) / (4 * radius*radius);

	m_result[0] = sqrt(Res[0] / k);
	m_result[1] = sqrt(Res[1] / k);
	m_result[2] = sqrt(Res[2] / k);
	m_result[3] = Res[3] / (2 * Res[0]);
	m_result[4] = Res[4] / (2 * Res[1]);
	m_result[5] = Res[5] / (2 * Res[2]);

	printf("Xo:%f Yo:%f Zo:%f Xg:%f Yg:%f Zg:%f C:%f\r\n", m_result[3], m_result[4], m_result[5], m_result[0], m_result[1], m_result[2], k);
	while (1);
}


int main()
{
	Calc_Process(2.0);
	return 0;
}
//http://www.51hei.com/bbs/dpj-97791-1.html