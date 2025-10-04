#include "TD.h"
#include "math.h"
#include "main.h"
#include "Can_receive.h"

TD Y_td;
TD X_td;


void TD_init(TD *TT, float r, float h1, float h2)
{
	TT->xx1 = 0; // ��ʼ�� xx1
	TT->xx2 = 0; // ��ʼ�� xx2
	TT->fh1 = 0; // ��ʼ�� fh1
	TT->r = r;
	TT->h1 = h1;
	TT->h2 = h2;
}


/*�źź���Sign*/
int16_t Sign(float x)
{
	if (x > 1E-6f) // 1*10��-6��
		return 1;
	else if (x < 1E-6f)
		return -1;
	else
		return 0;
}

/*�����Ժ��������亯��Fsg*/
int16_t Fsg(float x, float d)
{
	float output = 0;
	output = (Sign(x + d) - Sign(x - d)) / 2;
	return output;
}

/*���ٿ����ۺ����ź���*/
float Fhan(float x1, float x2, float r, float h)
{
	float d, a, a0, a1, a2, y;

	d = r * h * h;
	if (fabs(d) < 1E-6f)
	{
		d = 1E-6f; // ���������
	}

	a0 = h * x2;

	y = x1 + a0;

	if (fabs(y) < 1E-6f)
	{
		y = 1E-6f; // ���� y �ӽ���ʱ�������ȶ���
	}

	a1 = sqrt(d * (d + 8.0f * fabs(y))); // ʹ�� fabs ȷ���Ǹ�

	a2 = a0 + Sign(y) * (a1 - d) * 0.5f;

	a = (a0 + y) * Fsg(y, d) + a2 * (1 - Fsg(y, d));

	if (fabs(a) < 1E-6f)
	{
		a = 1E-6f; // ���� a �ӽ���ʱ�������ȶ���
	}

	return (-r) * (a / d) * Fsg(a, d) - r * Sign(a) * (1 - Fsg(a, d));
}

/*
���� r���ò��������˸���΢���������档
�ϴ�� r ֵ����ʹϵͳ�������Ӧ�仯���������ֵ���ܵ���ϵͳ���ȶ�������񵴡�
ͨ������£�����ѡ��һ����Խ�С��ֵ��ʼ���ԣ����� r = 5 ���� r = 10��Ȼ�����ʵ��Ч��������

���� h1������������ڲ������еķ��������йأ�Ӱ����ϵͳ�Ķ�̬���ԡ�
һ����ԣ�h1 Ӧ��С�ڵ��� 1�����ҿ��Ը��ݾ���Ӧ�ó������е�����
���ڽǶȷ�Χ�� 0 �� 180 ����������Գ��Դ� h1 = 0.5 ��ʼ���۲�����ֲ��ʵ�������

���� h2������ʱ�䲽������˵�ǲ������ڵı������ӣ���ֱ��Ӱ�쵽״̬���µ��ٶȡ�
�������ϵͳ���Խϸߵ�Ƶ�����У��� 1kHz������ô����ѡ���С��ʱ�䲽���������ӣ�
��֮����ѡ��ϴ��ֵ�����ǵ��Ƕȵı仯�ʲ����ر�ߣ����Դ� h2 = 0.01 ���� h2 = 0.005 ��ʼ���ԡ�
*/
void Tracking_Differentiator(TD *TT, float target, float r, float h1, float h2)
{
	if (isnan(target))
	{
		// ���� target Ϊ NaN �����
		target = 0; // ��������һ�������Ĭ��ֵ
	}
	// ���� Fhan ����ʱȷ�� d ��Ϊ��
	float d = r * h1 * h1;
	if (fabs(d) < 1E-6f)
	{
		d = 1E-6f;
	}

	TT->fh1 = Fhan(TT->xx1 - target, TT->xx2, r, h1);

	// ����״̬����
	TT->xx1 = TT->xx1 + h2 * TT->xx2;
	TT->xx2 = TT->xx2 + h2 * TT->fh1;
}

void TD_INIT(void)
{
	/*ң����*/ // ���� r ��С����Ӧ�ٶ������
	TD_init(&Y_td, 12000.0f, 0.5f, 0.005f);//36000.0f
	TD_init(&X_td, 25000.0f, 0.5f, 0.005f); // 50000��Ӧ�м�̧��߼�������
}


void td_task(void)
{
	Tracking_Differentiator(&Y_td, Y_target, Y_td.r, Y_td.h1, Y_td.h2);
    Tracking_Differentiator(&X_td, X_target, X_td.r, X_td.h1, X_td.h2);
}

