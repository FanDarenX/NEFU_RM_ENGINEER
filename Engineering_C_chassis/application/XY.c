#include "XY.h"
#include "pid.h"
#include "TD.h"

pid_struct_t X_speed_pid = {0};
pid_struct_t X_angle_pid = {0};
pid_struct_t Y_speed_pid = {0};
pid_struct_t Y_angle_pid = {0};

float X_target;
float Y_target;

float Multiple_X = 36.0 / 18.0 * 36.0f;
float Multiple_Y = 36.0 / 14.0 * 51.5f;
float Y_safe_multiple = 36.0 / 14.0 * 34.0f;

int Y_safe_flag = 0;

void XY_pid_init(void)
{
    Pid_Init(&X_speed_pid,
             10,   // Kp
             0.05, // Ki
             1,    // Kd
             200,  // Imaxout
             9998  // Outmax
    );
    Pid_Init(&X_angle_pid,
             0.1, // Kp
             0,   // Ki
             0,   // Kd
             200, // Imaxout
             8192 // Outmax
    );
    /*Y*/
    /*0x206*/
    Pid_Init(&Y_speed_pid,
             10,   // Kp
             0.05, // Ki
             1,    // Kd
             200,  // Imaxout
             9998  // Outmax
    );
    Pid_Init(&Y_angle_pid,
             0.1, // Kp
             0,   // Ki
             0,   // Kd
             200, // Imaxout
             8192 // Outmax
    );
}

void XY_location(float X_target, float Y_target) // 参数是编码值8192倍数// 内部应包含判定先x后y的角度顺序,看Y_target（内含绝对值）与Y_save的，决定X_target是否为零
{
    /*前伸到机械臂水平面投影安全距离*/
    if (Y_moto.total_angle <= Y_safe) // Y_moto.total_angle > Y_safe + Y_target - 10000 &&
        Y_safe_flag = 0;
    else
        Y_safe_flag = 1;

    // float compensate_Y = -4096.0 * 36.0 / 14.0; // 半个齿
    pid_calc(&Y_angle_pid, Y_target, Y_moto.total_angle); //+ compensate_Y
    pid_calc(&Y_speed_pid, Y_angle_pid.output, Y_moto.speed);

    pid_calc(&X_angle_pid, X_target, X_moto.total_angle);
    pid_calc(&X_speed_pid, X_angle_pid.output, X_moto.speed);
    XY_CURRENT(Y_speed_pid.output, X_speed_pid.output);
}

void XY_task()
{
    XY_location(X_td.h1, Y_td.h1);
}