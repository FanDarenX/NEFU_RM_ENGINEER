//
// Created by RM UI Designer
// Dynamic Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_figure_t ui_g_now_figures[11];
extern ui_interface_string_t ui_g_now_strings[10];
extern uint8_t ui_g_dirty_figure[11];
extern uint8_t ui_g_dirty_string[10];

#define ui_g_Ungroup_Cup_pitch_angle ((ui_interface_number_t *)&(ui_g_now_figures[0]))
#define ui_g_Ungroup_save_M ((ui_interface_round_t *)&(ui_g_now_figures[1]))
#define ui_g_Ungroup_save_L ((ui_interface_round_t *)&(ui_g_now_figures[2]))
#define ui_g_Ungroup_save_R ((ui_interface_round_t *)&(ui_g_now_figures[3]))
#define ui_g_Ungroup_Yaw_num ((ui_interface_number_t *)&(ui_g_now_figures[4]))
#define ui_g_Ungroup_Pitch_num ((ui_interface_number_t *)&(ui_g_now_figures[5]))
#define ui_g_Ungroup_Roll_num ((ui_interface_number_t *)&(ui_g_now_figures[6]))
#define ui_g_Ungroup_Z_num ((ui_interface_number_t *)&(ui_g_now_figures[7]))
#define ui_g_Ungroup_Y_num ((ui_interface_number_t *)&(ui_g_now_figures[8]))
#define ui_g_Ungroup_X_num ((ui_interface_number_t *)&(ui_g_now_figures[9]))
#define ui_g_Ungroup_reset ((ui_interface_rect_t *)&(ui_g_now_figures[10]))

#define ui_g_Ungroup_Direction (&(ui_g_now_strings[0]))
#define ui_g_Ungroup_Cup_pitch (&(ui_g_now_strings[1]))
#define ui_g_Ungroup_Mode (&(ui_g_now_strings[2]))
#define ui_g_Ungroup_Keys_can_mode (&(ui_g_now_strings[3]))
#define ui_g_Ungroup_Z (&(ui_g_now_strings[4]))
#define ui_g_Ungroup_Y (&(ui_g_now_strings[5]))
#define ui_g_Ungroup_X (&(ui_g_now_strings[6]))
#define ui_g_Ungroup_Yaw (&(ui_g_now_strings[7]))
#define ui_g_Ungroup_Pitch (&(ui_g_now_strings[8]))
#define ui_g_Ungroup_Roll (&(ui_g_now_strings[9]))

#ifdef MANUAL_DIRTY
#define ui_g_Ungroup_Cup_pitch_angle_dirty (ui_g_dirty_figure[0])
#define ui_g_Ungroup_save_M_dirty (ui_g_dirty_figure[1])
#define ui_g_Ungroup_save_L_dirty (ui_g_dirty_figure[2])
#define ui_g_Ungroup_save_R_dirty (ui_g_dirty_figure[3])
#define ui_g_Ungroup_Yaw_num_dirty (ui_g_dirty_figure[4])
#define ui_g_Ungroup_Pitch_num_dirty (ui_g_dirty_figure[5])
#define ui_g_Ungroup_Roll_num_dirty (ui_g_dirty_figure[6])
#define ui_g_Ungroup_Z_num_dirty (ui_g_dirty_figure[7])
#define ui_g_Ungroup_Y_num_dirty (ui_g_dirty_figure[8])
#define ui_g_Ungroup_X_num_dirty (ui_g_dirty_figure[9])
#define ui_g_Ungroup_reset_dirty (ui_g_dirty_figure[10])

#define ui_g_Ungroup_Direction_dirty (ui_g_dirty_string[0])
#define ui_g_Ungroup_Cup_pitch_dirty (ui_g_dirty_string[1])
#define ui_g_Ungroup_Mode_dirty (ui_g_dirty_string[2])
#define ui_g_Ungroup_Keys_can_mode_dirty (ui_g_dirty_string[3])
#define ui_g_Ungroup_Z_dirty (ui_g_dirty_string[4])
#define ui_g_Ungroup_Y_dirty (ui_g_dirty_string[5])
#define ui_g_Ungroup_X_dirty (ui_g_dirty_string[6])
#define ui_g_Ungroup_Yaw_dirty (ui_g_dirty_string[7])
#define ui_g_Ungroup_Pitch_dirty (ui_g_dirty_string[8])
#define ui_g_Ungroup_Roll_dirty (ui_g_dirty_string[9])
#endif

void ui_init_g();
void ui_update_g();

#endif // UI_g_H
