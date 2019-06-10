#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <iostream>
#include "cJSON.h"
//#include "md5.h"
#include "md5_2.h"

using namespace std;

typedef unsigned int uint32_t;
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef int int32_t;
typedef signed char int8_t;
typedef short int int16_t;

namespace variable1
{
	uint32_t xx = 1;
	uint32_t yy = 2;
}

namespace variable2
{
	uint32_t xx = 3;
	uint32_t yy = 4;
}

template<class T>
T Max(T a,T b)
{
	return a > b ? a : b;
}

#define		_STR_(_A)	#_A
#define		sdsds	"1""2"

class CA {
public:
	CA()
	{
		a = 0;
		b = 0;
		printf("默认构造函数！\n");
	}
	~CA()
	{
		a = 0;
		b = 0;
		printf("默认析构函数！\n");
	}
	CA(int c,int d)
	{
		a = c;
		b = d;
		printf("两个两个参数的构造函数！\n");
	}


	int operator() (int val)const
	{
		cout << "val is " << val <<"  "<<__LINE__<< endl;
		return val > 0 ? val : -val;
		
	}

	int operator() (int val1, int val2)const
	{
		int c = val1 + val2;
		cout << "c is " << c << endl;
		return c;
	}
	CA operator++ (int)
	{
		++a;
		printf("%s | %s | %s | %d,++operator\n", __DATE__, __TIME__, __FUNCTION__, __LINE__);//,__TIME__,__DATE__%s%s%s
		return CA(a,b);
	}


private:
	int a;
	int b;
};

typedef struct 
{
	int a;
	int add(int a, int b)
	{
		printf("dsd = %d\n", a + b);
		return 0;
	}
}MyStruct;

#define SDF 1
#define SDF 2

void fuzhi(float a[3])
{
	a[0] = 1.1;
	a[1] = 1.2;
	a[2] = 1.3;
}

void ref(int &a)
{
	a = 10+a;
}

typedef struct {
	int a;
}a_t;

double GpsDistance(int32_t nLng1, int32_t nLat1, int32_t nLng2, int32_t nLat2)
{
	double dLat1, dLat2, dDeltLng, R, dInAngelRad;
	double dPi = 3.1415926;

	R = 63781370.0; //单位: 分米dm
	dLat1 = nLat1*dPi / 180.0 / (1.0e7f);
	dLat2 = nLat2*dPi / 180.0 / (1.0e7f);
	dDeltLng = (nLng2 - nLng1)*dPi / 180.0 / (1.0e7f);

	dInAngelRad = acos(cos(dLat2 - dLat1) + cos(dLat1)*cos(dLat2)*(cos(dDeltLng) - (double)1.0));

	R = R*dInAngelRad;

	return R;
}

a_t at = { 1 };

void change_value(void)
{
	a_t *pat = &at;

	pat->a = 2;

}

class test1 {
public:
	int a;
	int ret()
	{
		return 1;
	}
};
class test2 {
public:
	int b;
};

union {
	float a;
	float b;
}data_u;

struct {
	float a;
	float b;
}data_s;

class myclass{
	int value;
public:
	void set_value1(int v) {
		value = v;
	}
	void set_value2(int &v) {
		value = v;
		v = 9999;
	}
	void print_value() {
		printf("value = %d\r\n,", value);
	}

	int a;
	myclass(int v):a(v){
		printf("a = %d\r\n", a);
	}
};

class CA1
{
public:
	CA1() { printf("构造函数1\r\n"); }
	CA1(int v) { printf("构造函数2，value is %d\r\n", v); }
public:
	int _ca_value;
	void set_value() { printf("class A!\r\n"); }
};

class CB
{
public:
	int _cb_value;
	void set_value() { printf("class B!\r\n"); }
};

class CD:public CA1, public CB
{
public:
	int _cd_value;
	void set_value() { CA1::set_value(); printf("class c!\r\n"); }
};

void array_sort_ascending(float *array_in, uint32_t len, float *array_out)
{
	float temp;
	uint32_t i, j;

	for (i = 0; i < len - 1; i++) {
		for (j = i + 1; j < len; j++)
			if (array_in[i] > array_in[j]) {
				temp = array_in[j];
				array_in[j] = array_in[i];
				array_in[i] = temp;
			}
	}
	memcpy(array_out, array_in, len*sizeof(float));
}

void array_sort_descending(float *array_in, uint32_t len, float *array_out)
{
	float temp;
	uint32_t i, j;

	for (i = 0; i < len - 1; i++) {
		for (j = i + 1; j < len; j++)
			if (array_in[i] < array_in[j]) {
				temp = array_in[j];
				array_in[j] = array_in[i];
				array_in[i] = temp;
			}
	}
	memcpy(array_out, array_in, len*sizeof(float));
}

typedef struct chain_table{
	int value;
	struct chain_table *next;
}chain_table_t;

void create_table()
{
	uint32_t data,i;
	chain_table_t *head,*r,*s;
	head = (chain_table_t*)malloc(sizeof(chain_table_t));
	r = head;

	for (i = 1; i < 5; i++) {
		s = (chain_table_t*)malloc(sizeof(chain_table_t));
		scanf("%d",&data);
		s->value = data;
		r->next = s;
		r = s;
	}
	r->next = NULL;

	i = 0;
	chain_table_t *t = head->next;
	while (t) {
		i++;
		printf("table[%d]->value = %d\r\n", i, t->value);
		t = t->next;
	}
	
}

typedef struct Node{
	int32_t value;
	struct Node *next;
}node_t;

typedef enum param_type_e {
	/* globally-known parameter types */
	PARAM_TYPE_INT32 = 0,
	PARAM_TYPE_FLOAT,

	PARAM_TYPE_UNKNOWN = 0xffffffff
} param_type_t;

union param_value_u {
	int32_t		i;
	float		f;
};

typedef struct param_info_s {
	//const char	*name;
	volatile param_type_t	type;
	volatile union param_value_u val;
}PARAM_INFO, *PPARAM_INFO;

int StrToInt(const char *str)
{
	int result = 0;
	int flag = 1;
	// 判断字符串是否是NULL 
	if (str == NULL)
	{
		return -1;
	}
	// 跳过前面的空格 
	while ((*str) == ' ')
		str++;
	// 判断字符串正负号
	if ((*str) == '-')
	{
		flag = -1;
		str++;
	}
	if ((*str) == '+')
	{
		flag = 1;
		str++;
	}
	// 跳过字符串前面的0 
	while ((*str) == '0')
	{
		str++;
	}
	// 遍历剩下字符串，进行字符转数字 
	while ((*str) != '\0')
	{
		// 判断字符是否合法 
		if ((*str >= '0') && (*str <= 'f'))
		{
			int temp = result;
			// 正数处理 
			if (flag == 1)
			{
				result = (result * 10) + (*str - '0');
			}
			// 负数处理 
			else if (flag == -1)
			{
				result = (result * 10) - (*str - '0');
			}
			// 溢出判断 
			if (((flag == 1) && (result < temp)) || ((flag == -1) && (result > temp)))
				return -1;
			str++;
		}
		else
		{
			return -1;
		}
	}
	return result;
}

class cRet_add {
public:
	cRet_add(int val)
	{
		this->val = val;
	}
private:
	int val;
public:
	int & add_val(int a,int b) 
	{
		//int c = a + b;
		return val;
	}
	int get_val()
	{
		return val;
	}
};
#if 0
char *string = "{\"family\":[\"father\",\"mother\",\"brother\",\"sister\",\"somebody\"]}";
//从缓冲区中解析出JSON结构
cJSON *json = cJSON_Parse(string);
cJSON *node = NULL;
//cJOSN_GetObjectItem 根据key来查找json节点 若果有返回非空
node = cJSON_GetObjectItem(json, "family");
if (node == NULL)
{
	printf("family node == NULL\n");
}
else
{
	printf("found family node\n");
}
node = cJSON_GetObjectItem(json, "famil");
if (node == NULL)
{
	printf("famil node == NULL\n");
}
else
{
	printf("found famil node\n");
}
//判断是否有key是string的项 如果有返回1 否则返回0
if (1 == cJSON_HasObjectItem(json, "family"))
{
	printf("found family node\n");
}
else
{
	printf("not found family node\n");
}
if (1 == cJSON_HasObjectItem(json, "famil"))
{
	printf("found famil node\n");
}
else
{
	printf("not found famil node\n");
}

node = cJSON_GetObjectItem(json, "family");
if (node->type == cJSON_Array)
{
	printf("array size is %d\n", cJSON_GetArraySize(node));
}
//非array类型的node 被当做array获取size的大小是未定义的行为 不要使用
cJSON *tnode = NULL;
int size = cJSON_GetArraySize(node);
int i;
for (i = 0; i<size; i++)
{
	tnode = cJSON_GetArrayItem(node, i);
	if (tnode->type == cJSON_String)
	{
		printf("value[%d]:%s\n", i, tnode->valuestring);
	}
	else
	{
		printf("node' type is not string\n");
	}
}

cJSON_ArrayForEach(tnode, node)
{
	if (tnode->type == cJSON_String)
	{
		printf("int forEach: vale:%s\n", tnode->valuestring);
	}
	else
	{
		printf("node's type is not string\n");
	}
}


//用char* 模拟一个JSON字符串
char *json_string = "{\"test_1\":\"0\",\"test_2\":\"1\",\"test_3\":\"2\"}";


//JSON字符串到cJSON格式
cJSON* cjson = cJSON_Parse(json_string);
//判断cJSON_Parse函数返回值确定是否打包成功
if (cjson == NULL) {
	printf("json pack into cjson error...");
}
else {//打包成功调用cJSON_Print打印输出
	cJSON_Print(cjson);
}

//获取字段值
//cJSON_GetObjectltem返回的是一个cJSON结构体所以我们可以通过函数返回结构体的方式选择返回类型！
char* test_1_string = cJSON_GetObjectItem(cjson, "test_1")->valuestring;
char* test_2_string = cJSON_GetObjectItem(cjson, "test_2")->valuestring;
char* test_3_string = cJSON_GetObjectItem(cjson, "test_3")->valuestring;

//打印输出
printf("%s\n", test_1_string);
printf("%s\n", test_2_string);
printf("%s\n", test_3_string);

//delete cjson
#endif

#define MAX 10 

double Determinant(double* matrix[], int n);

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

double AlCo(double* matrix[], int jie, int row, int column)
{
	double result;
	if ((row + column) % 2 == 0)
		result = Cofactor(matrix, jie, row, column);
	else result = (-1)*Cofactor(matrix, jie, row, column);
	return result;
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

void Inverse(double *matrix1[], double *matrix2[], int n, double d)
{
	int i, j;
	for (i = 0; i<n; i++)
		matrix2[i] = (double *)malloc(n*sizeof(double));
	for (i = 0; i<n; i++)
		for (j = 0; j<n; j++)
			*(matrix2[j] + i) = (AlCo(matrix1, n, i, j) / d);
}

void Input(double *matrix[], int m, int n, double A[2][2])
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
void Output(double *matrix[], int m, int n, double IA[2][2])
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


class persion
{
public:
	persion(int age);

	void set_age(int age)
	{
		this->age = age;
	}

	int get_age(void) { return age; }

	bool operator==(persion& p) const
	{
		if (this->age == p.age) return true;

		return false;
	}

private:
	int age;
};

persion::persion(int age)
{
	this->age = age;
}

void ArrayToHexString(uint8_t *buf, uint32_t len, uint8_t *out)
{
#define	IntToASCII(c)	(c)>9?((c)+0x37):((c)+0x30);

	uint32_t i;
	uint8_t temp;

	for (i = 0; i < len; i++) {
		temp = buf[i]&0xf0;
		out[3 * i + 0] = IntToASCII(temp>>4);
		temp = buf[i]&0x0f;
		out[3 * i + 1] = IntToASCII(temp);
		/* space */
		out[3 * i + 2] = 0x20;
	}

	out[3 * i] = 0;

}

#define	PACK_BUF_LENGTH	100

#define SetGear		"{\"method\":\"SetCameraGear\",\"params\":{\"Gear\":\"%s\"},\"id\":%d}"

#define	StickCalibStatus	"{\"method\":\"PhoneSettingParam \",\"params\":{\"Type\":%d,\"Length\":%d,\"Data\":[%d,%d,%d]}}"

#define	StickCalibCmd		"{\"method\":\"PhoneSettingParam \",\"params\":{\"Type\":%d,\"Length\":%d,\"Data\":[%d]}}"

#if 0
int main()
{
#if 1
	uint8_t paket_buf[PACK_BUF_LENGTH];

	//snprintf((char *)paket_buf, PACK_BUF_LENGTH, StickCalibStatus, 1, 3, 12, 13, 14);
	snprintf((char *)paket_buf, PACK_BUF_LENGTH, StickCalibCmd, 1, 1, 2);

	printf("%s\n", paket_buf);

	//从缓冲区中解析出JSON结构
	cJSON * json = cJSON_Parse((char *)paket_buf);

	//将传入的JSON结构转化为字符串 并打印
	char *json_data = NULL;
	printf("%s\n", json_data = cJSON_Print(json));

	printf("json node!, %s %s 0x%x\r\n", json->string, json->valuestring, json->type);

	cJSON *node = NULL;
	cJSON *subnode = NULL;
#if 0
	//cJOSN_GetObjectItem 根据key来查找json节点 若果有返回非空
	node = cJSON_GetObjectItem(json, "method");
	if (node == NULL)
	{
		printf("method node == NULL\n");
	}
	else {
		printf("find method node!, %s %s 0x%x\r\n", node->string, node->valuestring, node->type);
	}

	node = cJSON_GetObjectItem(json, "params");
	if (node == NULL)
	{
		printf("params node == NULL\n");
	}
	else {
		printf("find params node!, %s %s 0x%x\r\n", node->string, node->valuestring, node->type);
	}

	
	subnode = cJSON_GetObjectItem(node, "Type");
	if (subnode == NULL)
	{
		printf("Type node == NULL\n");
	}
	else {
		printf("find Type node!, %s %d 0x%x\r\n", subnode->string, subnode->valueint, subnode->type);
	}

	subnode = cJSON_GetObjectItem(node, "Length");
	if (subnode == NULL)
	{
		printf("Length node == NULL\n");
	}
	else {
		printf("find Length node!, %s %d 0x%x\r\n", subnode->string, subnode->valueint, subnode->type);
	}

	subnode = cJSON_GetObjectItem(node, "Data");
	if (subnode == NULL)
	{
		printf("Data node == NULL\n");
	}
	else {
		printf("find Data node!, %s %d 0x%x\r\n", subnode->string, subnode->valueint, subnode->type);
	}
#endif
	uint32_t data_num;
	node = cJSON_GetObjectItem(json, "params");
	subnode = cJSON_GetObjectItem(node, "Length");
	if (NULL != subnode) {
		data_num = subnode->valueint;
		printf("data length:%d\n", data_num);
	}

	subnode = cJSON_GetObjectItem(node, "Data");
	cJSON *sub2node = subnode->child;
	for (uint8_t i = 0; i < 3; i++) {
		if (sub2node != NULL)
			printf("node %d value:%d \n", i ,sub2node->valueint);
		else {
			break;
		}

		sub2node = sub2node->next;
	}
	
	

	free(json_data);
	//将JSON结构所占用的数据空间释放
	cJSON_Delete(json);
#endif
#if 0
	cJSON *json = cJSON_CreateObject();

	cJSON_AddStringToObject(json, "method", "PhoneSettingParam");

	cJSON *obj;
	cJSON_AddItemToObject(json, "params", obj = cJSON_CreateObject());
	cJSON_AddNumberToObject(obj, "Type", 1);
	cJSON_AddItemToObject(obj, "Length", cJSON_CreateNumber(8));

	int num[3] = { 12,13,14 };
	cJSON *intArray = cJSON_CreateIntArray(num, 3);
	cJSON_AddItemToObject(obj, "Data", intArray);


	char *buf = cJSON_Print(json);
	printf("%s\n", buf);
	printf("\n%d\n", strlen(buf));
	
	free(buf);
	
	cJSON_Delete(json);
#endif
	return 0;
}

#endif



typedef void(*pfun)(int, int);

int* add(int x, int y)
{
	printf(" %d %d\r\n",x,y);
	int sum = x + y;
	return &sum;
}

typedef struct {
	int x;
	int y;
	int z;
	int* (*add)(int,int);
}mag_data_t;

char *buf[] = 
{
	"123",
	"345",
	"345",
	"345",
	"345",
	"345"
};


int main()
{
	
	int i;
	unsigned char encrypt[] = "niuhongfang";//21232f297a57a5a743894a0e4a801fc3
	unsigned char mac[16];
#if 0
	//MD5_CTX md5;
	//MD5Init(&md5);
	//MD5Update(&md5, encrypt, strlen((char *)encrypt));
	//MD5Final(&md5, decrypt);
	MD5Sum(encrypt, strlen((char *)encrypt), mac);
	printf("加密前:%s\n加密后:", encrypt);
	for (i = 0; i<16; i++)
	{
		printf("%02x", mac[i]);
	}
	printf("\n");
#else
	md5_sum(encrypt, strlen((char *)encrypt), mac);
	printf("加密前:%s\n加密后:", encrypt);
	for (i = 0; i<16; i++)
	{
		printf("%02x", mac[i]);
	}
	printf("\n"); 
	//getchar();
#endif

	static double AeroWPLLA_data[6000];

	printf("size:%ld\n", sizeof(AeroWPLLA_data));

	return 0;
}


