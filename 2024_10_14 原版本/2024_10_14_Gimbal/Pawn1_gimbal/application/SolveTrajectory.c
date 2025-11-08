///*
//@brief: 弹道解算 适配陈君的rm_vision
//@author: CodeAlan  华南师大Vanguard战队
//*/
//// 近点只考虑水平方向的空气阻力
////TODO 完整弹道模型
////TODO 适配英雄机器人弹道解算
//#include <math.h>
//#include <stdio.h>
//#include <stdarg.h>
//#include "string.h"
//#include "cmsis_os.h"
#include "SolveTrajectory.h"
//#include "gimbal_task.h"
//#include "ins_task.h"
//#include "shoot.h"
//#include "referee.h"
//struct SolveTrajectoryParams st;
//struct tar_pos tar_position[4]; //最多只有四块装甲板
//float t_time = 0.2f; // 飞行时间
//float aim_x= 0, aim_y = 0, aim_z = 0; // aim point 落点，传回上位机用于可视化
//float pitch_1 = 0; //输出控制量 pitch绝对角度 弧度
//float yaw_1 = 0;   //输出控制量 yaw绝对角度 弧度
//int   ama=2;
//float pitch_add=0;
//float yaw_add=0;
//float distance_aim=0;
//float distance_board=0;
//float yaw_board = 0;
//float fire_angle = 0;//
//float board_angle = 0;//开火时板子与中心的夹角
//float fire_angle_flag = 0;//转换过来的应当开火角度  绝对值
//float board_x =0;
//float board_y =0;
//float text=0;
//float last_xw;
//float last_yw;
//float last_zw;
//float text_pitch=0.2;
//float text_angle1=0.06;
//float text_angle2=-0.03;
//void solvetrajectory_Init(struct SolveTrajectoryParams *ST);

//void solvetrajectory_task(void const * argument)
//{
//		vTaskDelay(200);
//		//定义参数
//	  solvetrajectory_Init(&st);
//		while (1)
//		{
//			if(st.xw==0&&st.yw==0&&st.zw==0)
//			{
//				 st.xw=last_xw;
//				 st.yw=last_yw;
//				 st.zw=last_zw;
//			}	
//			last_xw=st.xw;
//			last_yw=st.yw;
//			last_zw=st.zw;
//			
//			autoSolveTrajectory(&pitch_1, &yaw_1, &aim_x, &aim_y, &aim_z,&yaw_board,&board_x,&board_y);
//			//  /////////////////    云台自瞄控制量 
//			if(gimbal_control.gimbal_PC_data.tracking){
//				gimbal_control.gimbal_PC_data.aim_yaw=yaw_1;
//				gimbal_control.gimbal_PC_data.aim_pitch=pitch_1;			
//			} else{
//				gimbal_control.gimbal_PC_data.aim_yaw=gimbal_control.gimbal_yaw_motor.absolute_angle;
//				gimbal_control.gimbal_PC_data.aim_pitch=gimbal_control.gimbal_pitch_motor.absolute_angle;					
//			}
//			//////////////////
//			distance_aim=sqrt(aim_y*aim_y+aim_x*aim_x);
//			distance_board=sqrt(board_y*board_y+board_x*board_x);
//			st.distance=distance_board;
//			fire_angle = yaw_board-yaw_1;//还需考虑方向
//			board_angle = st.v_yaw*(t_time+0.1);//还需考虑方向
//			fire_angle_flag = st.v_yaw*asinf(st.r1/distance_board*sinf(fabs(board_angle)))/(fabsf(st.v_yaw));//为正值 需判断旋转方向
//		  ///////////////////////////////////根据车辆实际数据求正负////////   
//			
////			////////////////////////敌方顺时针旋转////////////
////			text=fire_angle-fire_angle_flag;
////			if(st.v_yaw<0)
////			{
////				if(fire_angle+fire_angle_flag<text_angle1&&fire_angle+fire_angle_flag>text_angle2)
////					shoot_control.fire_flag=1;
////				else shoot_control.fire_flag=0;
////			}
////			else 
////			{
////				if(fire_angle-fire_angle_flag<text_angle1&&fire_angle-fire_angle_flag>text_angle2)
////					shoot_control.fire_flag=1;
////				else shoot_control.fire_flag=0;
////			}
//			vTaskDelay(10);
//		}
//}

//void solvetrajectory_Init(struct SolveTrajectoryParams *ST)
//{
//		ST->k = 0.092;//0.092
//		ST->bullet_type =  BULLET_17;
//		ST->current_v = 26.0f;
//		ST->current_pitch = INS_angle[0];//gimbal_control.gimbal_INT_angle_point[0];
//		ST->current_yaw = INS_angle[2];
//		ST->xw = 0.0;
//		ST->yw = 0;
//		ST->zw = 0;

//		ST->vxw = 0;
//		ST->vyw = 0;
//		ST->vzw = 0;
//		ST->v_yaw = 0;
//		ST->tar_yaw = 0;
//		ST->r1 = 0.2;
//		ST->r2 = 0.2;
//		ST->dz = 0.0;
//		ST->bias_time = 0;
//		ST->s_bias = 0.0;
//		ST->z_bias = 0.13f;
//		ST->armor_id = ARMOR_HERO;
//		ST->armor_num = ARMOR_NUM_NORMAL;	
//}

///*
//@brief 单方向空气阻力弹道模型
//@param s:m 距离
//@param v:m/s 速度
//@param angle:rad 角度
//@return z:m
//*/
//float monoDirectionalAirResistanceModel(float s, float v, float angle)
//{
//    float z;
//    //t为给定v与angle时的飞行时间
//    t_time = (float)((exp(st.k * s) - 1) / (st.k * v * cos(angle)));
//    //z为给定v与angle时的高度
//    z = (float)(v * sin(angle) * t_time - GRAVITY * t_time * t_time / 2);
//   // printf("model %f %f\n", t, z);
//    return z;
//}

///*
//@brief 完整弹道模型
//@param s:m 距离
//@param v:m/s 速度
//@param angle:rad 角度
//@return z:m
//*/
////TODO 完整弹道模型
//float completeAirResistanceModel(float s, float v, float angle)
//{
//    


//}

///*
//@brief pitch轴解算
//@param s:m 距离
//@param z:m 高度
//@param v:m/s
//@return angle_pitch:rad
//*/
//float pitchTrajectoryCompensation(float s, float z, float v)
//{
//    float z_temp, z_actual, dz;
//    float angle_pitch;
//    int i = 0;
//    z_temp = z;
//    for (i = 0; i < 20; i++)
//    {
//        angle_pitch = atan2(z_temp, s); // rad
//        z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
//        dz = 0.3*(z - z_actual);
//        z_temp = z_temp + dz;
//        if (fabsf(dz) < 0.0002)
//        {
//            break;
//        }
//    }
//		return angle_pitch;
//}

///*
//@brief 根据最优决策得出被击打装甲板 自动解算弹道
//@param pitch:rad  传出pitch
//@param yaw:rad    传出yaw
//@param aim_x:传出aim_x  打击目标的x
//@param aim_y:传出aim_y  打击目标的y
//@param aim_z:传出aim_z  打击目标的z
//*/
//float pitch_temp_now=0;
//void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z,float *yaw_1,float *x,float *y)
//{
//    // 线性预测
//    float timeDelay = 0.01 + t_time;
//    st.tar_yaw += st.v_yaw * timeDelay;
//    //计算四块装甲板的位置
//    //装甲板id顺序，以四块装甲板为例，逆时针编号
//    //      2
//    //   3     1
//    //      0
//	
//		int use_1 = 1;
//		int i = 0;
//    int idx = 0; // 选择的装甲板	
//    //armor_num = ARMOR_NUM_BALANCE 为平衡步兵
//    if (st.armor_num == ARMOR_NUM_BALANCE) {
//        for (i = 0; i<2; i++) {
//            float tmp_yaw = st.tar_yaw + i * PI;
//            float r = st.r1;
//            tar_position[i].x = st.xw - r*cos(tmp_yaw);
//            tar_position[i].y = st.yw - r*sin(tmp_yaw);
//            tar_position[i].z = st.zw;
//            tar_position[i].yaw = tmp_yaw;
//        }

//				//因为是平衡步兵 只需判断两块装甲板即可
//        float yaw_diff_min  = fabsf(*yaw - tar_position[0].yaw);  
//        float temp_yaw_diff = fabsf(*yaw - tar_position[1].yaw); 
//        if (temp_yaw_diff < yaw_diff_min)
//        {
//            yaw_diff_min = temp_yaw_diff;
//            idx = 1;
//        }


//    } else if (st.armor_num == ARMOR_NUM_OUTPOST) {  //前哨站
//        for (i = 0; i<3; i++) {
//            float tmp_yaw = st.tar_yaw + i * 2.0f * PI/3.0f;  // 2/3PI
//            float r =  (st.r1 + st.r2)/2;   //理论上r1=r2 这里取个平均值
//            tar_position[i].x = st.xw - r*cos(tmp_yaw);
//            tar_position[i].y = st.yw - r*sin(tmp_yaw);
//            tar_position[i].z = st.zw;
//            tar_position[i].yaw = tmp_yaw;
//        }

//        //TODO 选择最优装甲板 选板逻辑你们自己写，这个一般给英雄用
//    } else {
//        for (i = 0; i<4; i++) {
//            float tmp_yaw = st.tar_yaw + i * PI/2.0f;
//            float r = use_1 ? st.r1 : st.r2;
//            tar_position[i].x = st.xw - r*cos(tmp_yaw);
//            tar_position[i].y = st.yw - r*sin(tmp_yaw);
//            tar_position[i].z = use_1 ? st.zw : st.zw + st.dz;
//            tar_position[i].yaw = tmp_yaw;
//            use_1 = !use_1;
//        }

//				/*******************2种常见决策方案******************/
////        // 1.计算距离最近的装甲板
////        float dis_diff_min = sqrt(tar_position[0].x * tar_position[0].x + tar_position[0].y * tar_position[0].y);
////       	int idx = 0;
////        for (i = 1; i<4; i++)
////        {
////        		float temp_dis_diff = sqrt(tar_position[i].x * tar_position[0].x + tar_position[i].y * tar_position[0].y);
////        		if (temp_dis_diff < dis_diff_min)
////        		{
////        			dis_diff_min = temp_dis_diff;
////        			idx = i;
////       		  }
////        }

//        // 2.计算枪管到目标装甲板yaw最小的那个装甲板
//        float yaw_diff_min = fabsf(*yaw - tar_position[0].yaw);
//        for (i = 1; i<4; i++) {
//            float temp_yaw_diff = fabsf(*yaw - tar_position[i].yaw);
//            if (temp_yaw_diff < yaw_diff_min)
//            {
//                yaw_diff_min = temp_yaw_diff;
//                idx = i;
//            }
//        }
//    }

//    *aim_z = tar_position[idx].z + st.vzw * timeDelay;
//    *aim_x = tar_position[idx].x + st.vxw * timeDelay;
//    *aim_y = tar_position[idx].y + st.vyw * timeDelay;
//		//*pitch=(float)(sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y)));
//    //这里符号给错了
//		float dis = sqrt((*aim_x) * (*aim_x) + (*aim_y) * (*aim_y));
//		if(dis<=2.0f) st.z_bias = 0.06f;
//		else if(dis<=3.0f) st.z_bias = 0.10f;
//		else if(dis<=4.0f) st.z_bias = 0.13f;
//		else if(dis<=5.0f) st.z_bias = 0.18f;
//		else st.z_bias = 0.22f;
//    *pitch = -pitchTrajectoryCompensation(dis +st.s_bias,
//          *aim_z + st.z_bias, st.current_v);
//    *yaw = (float)(atan2(st.yw ,st.xw ));
//		*yaw_1=(float)(atan2(tar_position[idx].y ,tar_position[idx].x));
//		*x = tar_position[idx].x;
//		*y = tar_position[idx].y;
//		ama=5;
//}

//// 从坐标轴正向看向原点，逆时针方向为正

