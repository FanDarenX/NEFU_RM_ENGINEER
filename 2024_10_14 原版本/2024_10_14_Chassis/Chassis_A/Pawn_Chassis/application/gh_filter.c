#include "gh_filter.h"
#include "string.h"

#define ABS(x) (((x) > 0) ? (x) : (-(x)))

void g_h_fliter(float data,_GH_Filter_Struct* gh_param)
{
    float x_pred;
    float residual;

    //Ô¤²â
    x_pred = gh_param->x_est + ( gh_param->dx * gh_param->dt );

    //¸üĞÂ
    residual = data - x_pred;
    gh_param->dx  = gh_param->dx + gh_param->h * ( residual / gh_param->dt );
    gh_param->x_est = x_pred + gh_param->g * residual;

    gh_param->x_now=gh_param->x_est;
}


