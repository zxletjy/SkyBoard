#include "math.h"

//��֤����ֵ����Ч��
float safe_asin(float v)
{
    if (isnan(v)) {
        return 0.0;
    }
    if (v >= 1.0f) {
        return M_PI/2;
    }
    if (v <= -1.0f) {
        return -M_PI/2;
    }
    return asinf(v);
}

//16λ�޷����������޷�
uint16_t constrain_uint16(uint16_t amt, uint16_t low, uint16_t high)
{
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

//32λ�������޷�
int32_t constrain_int32(int32_t amt, int32_t low, int32_t high) {
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

//�Ƕ�ת����
float radians(float deg) {
	return deg * DEG_TO_RAD;
}

//����ת�Ƕ�
float degrees(float rad) {
	return rad * RAD_TO_DEG;
}

//��ƽ��
float sq(float v) {
	return v*v;
}

//3ά��������
float pythagorous3(float a, float b, float c) {
	return sqrtf(sq(a)+sq(b)+sq(c));
}

//4ά��������
float pythagorous4(float a, float b, float c, float d) {
	return sqrtf(sq(a)+sq(b)+sq(c)+sq(d));
}
