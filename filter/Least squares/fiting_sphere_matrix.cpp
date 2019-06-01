/*
 *
 *			球体拟合算法，使用矩阵
 *
 *  @use:	Lease Squares		.LS
 *
 *	@refer：https://blog.csdn.net/woniu199166/article/details/79459807?utm_source=blogxgwz5
 *
 */
#include <Windows.h>
#include <math.h>
#include <stdio.h> 
#include <stdlib.h> 

#define MAX 10 

void Input(double *matrix[], int m, int n, double A[4][4]);
void Output(double *matrix[], int m, int n, double IA[4][4]);
void InputExample();
void Inverse(double *matrix1[], double *matrix2[], int n, double d);
double AlCo(double* matrix[], int jie, int row, int column);
double Determinant(double *matrix[], int n);
double Cofactor(double* matrix[], int jie, int row, int column);

void PrintMTX(double A[4][4])
{
	for (int i = 0; i< 4; i++)
	{
		printf("\n");
		for (int j = 0; j< 4; j++)
			printf("%10G ", A[i][j]);
	}
	printf("\n");
}
void main()
{
	double *matrix1[MAX], *matrix2[MAX];
	double d;
	int n;
	//printf("请输入行列式的阶数:"); 
	//scanf("%d",&n); 
	n = 4;
	//InputExample(); 
	//printf("开始输入矩阵:\n"); 
	int i, j;
	double Y[4], fac[4], array[12][3], MTX[4][4], IMTX[4][4];
	ZeroMemory(MTX, sizeof(MTX));
	ZeroMemory(IMTX, sizeof(IMTX));
	ZeroMemory(Y, sizeof(Y));
	ZeroMemory(fac, sizeof(fac));
	//以下的数据是我用SolidWorks画出来的球，我在球上任意取了12个点
	array[0][0] = -70.08; array[0][1] = 29.27; array[0][2] = 6.04;
	array[1][0] = -72.37; array[1][1] = 24.19; array[1][2] = -1.64;
	array[2][0] = -65.1; array[2][1] = 13.6; array[2][2] = 0.58;
	array[3][0] = -60.11; array[3][1] = 13.51; array[3][2] = 4.93;
	array[4][0] = -61.12; array[4][1] = 27.69; array[4][2] = 12.16;
	array[5][0] = -65;    array[5][1] = 22.32; array[5][2] = -11.14;
	array[6][0] = -61.44; array[6][1] = 18.31; array[6][2] = -10.46;
	array[7][0] = -56.37; array[7][1] = 15.920; array[7][2] = -7.78;
	array[8][0] = -53.27; array[8][1] = 15.95; array[8][2] = -5.39;
	array[9][0] = -51.64; array[9][1] = 16.53; array[9][2] = -3.82;
	array[10][0] = -50.99; array[10][1] = 16.92; array[10][2] = -3.12;
	array[11][0] = -58.94; array[11][1] = 15.16; array[11][2] = 7.64;
	for (i = 0; i < 12; i++)
	{
		Y[0] += pow(array[i][0], 3) + pow(array[i][1], 2)*array[i][0] + pow(array[i][2], 2)*array[i][0];
		Y[1] += pow(array[i][1], 3) + pow(array[i][0], 2)*array[i][1] + pow(array[i][2], 2)*array[i][1];
		Y[2] += pow(array[i][2], 3) + pow(array[i][0], 2)*array[i][2] + pow(array[i][1], 2)*array[i][2];
		Y[3] -= pow(array[i][0], 2) + pow(array[i][1], 2) + pow(array[i][2], 2);
	}
	for (j = 0; j< 12; j++)
	{
		MTX[0][0] += pow(array[j][0], 2);
		MTX[0][1] += array[j][0] * array[j][1];
		MTX[0][2] += array[j][0] * array[j][2];
		MTX[0][3] -= array[j][0];
		MTX[1][1] += pow(array[j][1], 2);
		MTX[1][2] += array[j][1] * array[j][2];
		MTX[1][3] -= array[j][1];
		MTX[2][2] += pow(array[j][2], 2);
		MTX[2][3] -= array[j][2];
	}
	MTX[1][0] = MTX[0][1];
	MTX[2][0] = MTX[0][2];
	MTX[2][1] = MTX[1][2];
	MTX[3][0] = MTX[0][3];
	MTX[3][1] = MTX[1][3];
	MTX[3][2] = MTX[2][3];
	MTX[3][3] = 12.0;
	Input(matrix1, n, n, MTX);
	PrintMTX(MTX);
	printf("\nY[0] =  %f ", Y[0]);
	printf("\nY[1] =  %f ", Y[1]);
	printf("\nY[2] =  %f ", Y[2]);
	printf("\nY[3] =  %f \n", Y[3]);
	d = Determinant(matrix1, n);
	if (d == 0) printf("这个矩阵不可逆。\n");
	else
	{
		Inverse(matrix1, matrix2, n, d);
		printf("它的逆矩阵是:\n");
		Output(matrix2, n, n, IMTX);
		for (i = 0; i< 4; i++)
		{
			printf("\n");
			for (int j = 0; j< 4; j++)
			{
				double tt = 0;
				tt = MTX[i][0] * IMTX[0][j] + MTX[i][1] * IMTX[1][j] + MTX[i][2] * IMTX[2][j] + MTX[i][3] * IMTX[3][j];
				printf("%5.2f \t", tt);
			}
		}printf("\n");
		printf("\n解算出来的系数是：");
		for (i = 0; i < 4; i++)
		{
			fac[0] += IMTX[0][i] * Y[i];
			fac[1] += IMTX[1][i] * Y[i];
			fac[2] += IMTX[2][i] * Y[i];
			fac[3] += IMTX[3][i] * Y[i];
			//printf("\nfac[%d] =  %f ",i,fac[i]);
		}
		printf("\na =  %f ", fac[0] / 2.0);
		printf("\nb =  %f ", fac[1] / 2.0);
		printf("\nc =  %f ", fac[2] / 2.0);
		//printf("\nfac[3] =  %f ",fac[3]/2.0);
		//for (i = 0;i < 4;i++)
		//{
		//    printf("\nfac[%d] =  %f ",i,fac[i]);
		//}
		//printf("\n");
		double R;
		R = pow(fac[0] / 2.0, 2) + pow(fac[1] / 2.0, 2) + pow(fac[2] / 2.0, 2) - fac[3];
		printf("\nR = %f ", pow(R, 0.5));
		Sleep(1);
		getchar();
	}
	//return 0; 
}

void Input(double *matrix[], int m, int n, double A[4][4])
{
	int i, j;
	for (i = 0; i<m; i++)
		matrix[i] = (double *)malloc(n*sizeof(double));
	//printf("Please input the matrix:\n"); 
	for (i = 0; i<m; i++)
		for (j = 0; j<n; j++)
		{
			//double tt = ;
			*(matrix[i] + j) = A[i][j];
		}
}

void Output(double *matrix[], int m, int n, double IA[4][4])
{
	int i, j;
	for (i = 0; i<m; i++)
	{
		for (j = 0; j<n; j++)
		{
			IA[i][j] = *(matrix[i] + j);
			printf("%10G \t", IA[i][j]);
		}
		printf("\n");
	}
}

void InputExample()
{
	printf("数据间用Tab键间隔，输完一行按回车。例如：\n");
	printf("12 26 48\n");
	printf("-1 24 10\n");
	printf("2 -6 14\n");
}

void Inverse(double *matrix1[], double *matrix2[], int n, double d)
{
	int i, j;
	for (i = 0; i<n; i++)
		matrix2[i] = (double *)malloc(n*sizeof(double));
	for (i = 0; i<n; i++)
		for (j = 0; j<n; j++)
			*(matrix2[j] + i) = (AlCo(matrix1, n, i, j) / d);
}

double Determinant(double* matrix[], int n)
{
	double result = 0, temp;
	int i;
	if (n == 1)
		result = (*matrix[0]);
	else
	{
		for (i = 0; i<n; i++)
		{
			temp = AlCo(matrix, n, n - 1, i);
			result += (*(matrix[n - 1] + i))*temp;
		}
	}
	return result;
}

double AlCo(double* matrix[], int jie, int row, int column)
{
	double result;
	if ((row + column) % 2 == 0)
		result = Cofactor(matrix, jie, row, column);
	else result = (-1)*Cofactor(matrix, jie, row, column);
	return result;
}

double Cofactor(double* matrix[], int jie, int row, int column)
{
	double result;
	int i, j;
	double* smallmatr[MAX - 1];
	for (i = 0; i<jie - 1; i++)
		smallmatr[i] = (double*)malloc(sizeof(double)*(jie - 1));
	for (i = 0; i<row; i++)
		for (j = 0; j<column; j++)
			*(smallmatr[i] + j) = *(matrix[i] + j);
	for (i = row; i<jie - 1; i++)
		for (j = 0; j<column; j++)
			*(smallmatr[i] + j) = *(matrix[i + 1] + j);
	for (i = 0; i<row; i++)
		for (j = column; j<jie - 1; j++)
			*(smallmatr[i] + j) = *(matrix[i] + j + 1);
	for (i = row; i<jie - 1; i++)
		for (j = column; j<jie - 1; j++)
			*(smallmatr[i] + j) = *(matrix[i + 1] + j + 1);
	result = Determinant(smallmatr, jie - 1);
	return result;
}