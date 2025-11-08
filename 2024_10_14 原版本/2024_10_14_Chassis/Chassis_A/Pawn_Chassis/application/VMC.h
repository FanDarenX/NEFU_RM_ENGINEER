#include "struct_typedef.h"
#include "gimbal_task.h"
#include "ins_task.h"

extern fp32 VMC_solve_J1(fp32 a,fp32 h);
extern fp32 VMC_solve_J2(fp32 a,fp32 h);
extern fp32 VMC_solve_J3(fp32 a,fp32 h);
extern fp32 VMC_solve_J4(fp32 a,fp32 h);
extern fp32 VMC_solve_invJ1(fp32 a,fp32 h);
extern fp32 VMC_solve_invJ2(fp32 a,fp32 h);
extern fp32 VMC_solve_invJ3(fp32 a,fp32 h);
extern fp32 VMC_solve_invJ4(fp32 a,fp32 h);
extern void Forward_kinematic_solution(chassis_control_t *feedback_update,
                                       fp32 Q1,fp32 S1,fp32 Q4,fp32 S4,fp32 A1,fp32 A4,uint8_t ce)	;//ce 1为L 0为R
extern void inverse_kinematics(chassis_control_t *feedback_update,fp32 angle,fp32 high,uint8_t ce);	//ce 1ΪL 0ΪR
extern fp32 Differential_solution(fp32 number);
extern void gh_L0_seita_init(void);
extern void gh_L0_seita_L_update(void);
extern void gh_L0_seita_R_update(void);
extern fp32 F_foot_Cal(fp32 F, fp32 Tp, fp32 ddzm,
	fp32 L0, fp32 dL0, fp32 ddL0,
	fp32 seita, fp32 dseita, fp32 ddseita
);


