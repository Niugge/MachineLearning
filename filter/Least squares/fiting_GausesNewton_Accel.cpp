/*
 *			���ٶȼ���������㷨
 *
 *  @use:	Lease Squares		.LS	
 *	@use:	Gause-Newton
 *	@use:	Levenberg-Marquardt
 *	@use:	��˹��Ԫ��
 *  @refer:	https://blog.csdn.net/loveuav/article/details/81592870
 *
 *
 *	accel sampe:	up down left right front back	, 6 faces
 *
 *	fitting:	ax  ay  az
 *	measure:	axm aym azm
 *
 *	relations:	
 *				ax = scale_x(axm + offset_x);
 *				ay = scale_y(aym + offset_y);
 *				az = scale_z(azm + offset_z);
 *
 *				r^2 = ax^2 + ay^2 + az^2;
 *
 *				beta = [scale_x, scale_y, scale_z, offset_x, offset_y, offset_z]
 *				e(beta) = r^2 - ax^2 + ay^2 + az^2;
 *
 *				E(beta) = e0(beta)^2 + e1(beta)^2 + ... + e(k-1)(beta)^2;
 */

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <iostream>


using namespace std;

typedef unsigned int uint32_t;
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef int int32_t;
typedef signed char int8_t;
typedef short int int16_t;

#define		MATRIX_SIZE			6

#define		LM_ALGORITHM_USE	1

#define		ConstrainFloat(val, min, max)	(val < min ? min : val > max ? max : val)

double	 lm_lambda = 0.1;

typedef struct {
	float x;
	float y;
	float z;
}Vector3f_t;

static void ResetMatrices(float JtR[6], float JtJ[6][6])
{
	int16_t j, k;
	for (j = 0; j<6; j++)
	{
		JtR[j] = 0.0f;
		for (k = 0; k<6; k++)
		{
			JtJ[j][k] = 0.0f;
		}
	}
}

/*
 *	@����JtR  JtJ
 *
 */
static void UpdateMatrices(float JtR[6], float JtJ[6][6], float beta[6], float data[3])
{
	int16_t j, k;
	float dx, b;
	float residual = 1.0;
	float jacobian[6];

	for (j = 0; j<3; j++)
	{
		b = beta[3 + j];
		dx = (float)data[j] - beta[j];
		//����в� (���������̵����)
		residual -= b*b*dx*dx;

		//�����ſɱȾ���
		jacobian[j] = 2.0f*b*b*dx;
		jacobian[3 + j] = -2.0f*b*dx*dx;
	}

	for (j = 0; j<6; j++)
	{
		//���㺯���ݶ�
		JtR[j] += jacobian[j] * residual;

		for (k = 0; k<6; k++)
		{
			//����Hessian���󣨼���ʽ��ʡ�Զ���ƫ���������ſɱȾ�������ת�õĳ˻�
			JtJ[j][k] += jacobian[j] * jacobian[k];
		}
	}
}

static void MovMaxRowsToUp(float root[MATRIX_SIZE], float mat[MATRIX_SIZE][MATRIX_SIZE], uint8_t row)
{
	float temp;
	int16_t i, j = -1;

	temp = mat[row][row];

	for (i = row + 1; i < MATRIX_SIZE; i++) {
		if (temp < mat[i][row]) {
			j = i;
			temp = mat[i][row];
		}
	}

	if (j > 0) {
		temp = root[row];
		root[row] = root[j];
		root[j] = temp;

		for (i = 0; i < MATRIX_SIZE; i++) {
			temp = mat[row][i];
			mat[row][i] = mat[j][i];
			mat[j][i] = temp;
		}
	}
}

static int GaussEliminateSolveDelta(float root[MATRIX_SIZE], float mat[MATRIX_SIZE][MATRIX_SIZE], float delta[MATRIX_SIZE])
{
	int16_t i, j, k;
	float mu;

	//����LM����
#if LM_ALGORITHM_USE
	for (i = 0; i<6; i++)
	{
		mat[i][i] += lm_lambda;
	}
#endif

	//�����Ԫ�������Է�����ת��Ϊ�����Ƿ�����
	for (i = 0; i<MATRIX_SIZE; i++)
	{
		MovMaxRowsToUp(root, mat, i);

		if (0 == mat[i][i]) {
			return -1;
		}

		//��JtJ[i][i]��Ϊ0����������JtJ[i][i]���µ�Ԫ����Ϊ0
		for (j = i + 1; j<MATRIX_SIZE; j++)
		{
			mu = mat[j][i] / mat[i][i];

			root[j] -= mu * root[i];

			for (k = i; k<MATRIX_SIZE; k++)
			{
				mat[j][k] -= mu * mat[i][k];
			}

		}
	}

	//�ش��õ�������Ľ�
	for (i = MATRIX_SIZE - 1; i >= 0; i--)
	{
		root[i] /= mat[i][i];
		mat[i][i] = 1.0f;

		for (j = 0; j<i; j++)
		{
			root[j] -= mat[j][i] * root[i];
			mat[j][i] = 0;
		}
	}

	for (i = 0; i<MATRIX_SIZE; i++)
	{
		delta[i] = root[i];
	}

	return 0;
}


void GaussNewton(Vector3f_t inputData[6], Vector3f_t* offset, Vector3f_t* scale, float length)
{
	uint32_t cnt = 0;
	double   eps = 0.000000001;
	double   change = 100.0;
	float    data[3];
	float    beta[6];      //���̽�
	float    delta[6];     //��������
	float    JtR[6];       //�ݶȾ���
	float    JtJ[6][6];    //Hessian����

#if LM_ALGORITHM_USE
	
	double   changeTemp = 100.0;
#endif
						   //�趨���̽��ֵ
	beta[0] = beta[1] = beta[2] = 0;
	beta[3] = beta[4] = beta[5] = 1 / length;

	//��ʼ����������������С��epsʱ�������㣬�õ����̽������Ž�
	while (change > eps)
	{
		//�����ʼ��
		ResetMatrices(JtR, JtJ);

		//�������̺������ݶ�JtR��Hessian����JtJ
		for (uint8_t i = 0; i<6; i++)
		{
			data[0] = inputData[i].x;
			data[1] = inputData[i].y;
			data[2] = inputData[i].z;
			UpdateMatrices(JtR, JtJ, beta, data);
		}

		//��˹��Ԫ����ⷽ�̣�JtJ * delta = JtR���õ�delta
		GaussEliminateSolveDelta(JtR, JtJ, delta);

		//�����������
#if LM_ALGORITHM_USE
		changeTemp =	
#else
		change =
#endif
					delta[0] * delta[0] +\
					delta[1] * delta[1] +\
					delta[2] * delta[2] +\
					delta[3] * delta[3] +\
					delta[4] * delta[4] +\
					delta[5] * delta[5] ;

		//���ݲ�����С����LM����
		//LM����Ϊ0ʱ���㷨�˻�Ϊ��˹ţ�ٷ���LM���ӱ��ʱ���˻�Ϊ������С���ݶ��½���
#if LM_ALGORITHM_USE
		if (changeTemp < change)
		{
			//LM���Ӽ�С
			lm_lambda /= 3;
#endif
			//���·��̽�
			for (uint8_t i = 0; i<6; i++)
			{
				beta[i] -= delta[i];
			}
#if LM_ALGORITHM_USE
			change = changeTemp;
		}
		else
		{
			//LM��������
			lm_lambda *= 3;
			lm_lambda = ConstrainFloat(lm_lambda, 0, 1e10);
		}
#endif
		//���Ƶ�������
		if (cnt++ > 100)
			break;
	}

	//����У׼����
	scale->x = beta[3] * length;
	scale->y = beta[4] * length;
	scale->z = beta[5] * length;
	offset->x = beta[0];
	offset->y = beta[1];
	offset->z = beta[2];
}


void DisplayMatirx(Vector3f_t inputData[6])
{
	for (int i = 0; i < MATRIX_SIZE; i++) {
		printf("��%d�����ݣ�\n", i+1);
		printf("\t\tax:%10f\tay:%10f\taz:%10f\r\n", inputData[i].x, inputData[i].y, inputData[i].z);
	}
	printf("\n");
}

void DisplayResult(Vector3f_t offset, Vector3f_t scale)
{
	printf("ƫ������\n");
	printf("\tx_offset:%10f\ty_offset:%10f\tz_offset:%10f\n", offset.x, offset.y, offset.z);

	printf("����ֵ��\n");
	printf("\tx_scale:%10f\ty_scale:%10f\tz_scale:%10f\n\n\n", scale.x, scale.y, scale.z);
}

int main()
{
	
	Vector3f_t	accel_offset;
	Vector3f_t	accel_scale;

	Vector3f_t	accel_saw[MATRIX_SIZE] = {	{-0.011308, -0.006936, -1.014725},\
											{ 0.001049, -0.010036,  1.009912},\
											{-1.011354, -0.108833, -0.112941},\
											{ 0.972251,  0.046388,  0.031283},\
											{-0.027451,  0.984027, -0.004315},\
											{-0.058122, -1.009629,  0.004295} };

	DisplayMatirx(accel_saw);

	GaussNewton(accel_saw, &accel_offset, &accel_scale, 1);

	DisplayResult(accel_offset, accel_scale);


	return 0;
}