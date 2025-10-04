//
// Created by RM UI Designer
// Dynamic Edition
//

#include "string.h"
#include "ui_interface.h"
#include "ui_g.h"

#define TOTAL_FIGURE 11
#define TOTAL_STRING 10

ui_interface_figure_t ui_g_now_figures[TOTAL_FIGURE];
ui_interface_string_t ui_g_now_strings[TOTAL_STRING];
uint8_t ui_g_dirty_figure[TOTAL_FIGURE];
uint8_t ui_g_dirty_string[TOTAL_STRING];
#ifndef MANUAL_DIRTY
ui_interface_figure_t ui_g_last_figures[TOTAL_FIGURE];
ui_interface_string_t ui_g_last_strings[TOTAL_STRING];
#endif

void ui_init_g()
{
    ui_g_Ungroup_Cup_pitch_angle->figure_tpye = 6;
    ui_g_Ungroup_Cup_pitch_angle->layer = 1;
    ui_g_Ungroup_Cup_pitch_angle->font_size = 20;
    ui_g_Ungroup_Cup_pitch_angle->start_x = 274;
    ui_g_Ungroup_Cup_pitch_angle->start_y = 817;
    ui_g_Ungroup_Cup_pitch_angle->color = 2;
    ui_g_Ungroup_Cup_pitch_angle->number = 0;
    ui_g_Ungroup_Cup_pitch_angle->width = 2;

    ui_g_Ungroup_save_M->figure_tpye = 2;
    ui_g_Ungroup_save_M->layer = 1;
    ui_g_Ungroup_save_M->r = 50;
    ui_g_Ungroup_save_M->start_x = 965;
    ui_g_Ungroup_save_M->start_y = 54;
    ui_g_Ungroup_save_M->color = 6;
    ui_g_Ungroup_save_M->width = 3;

    ui_g_Ungroup_save_L->figure_tpye = 2;
    ui_g_Ungroup_save_L->layer = 1;
    ui_g_Ungroup_save_L->r = 51;
    ui_g_Ungroup_save_L->start_x = 860;
    ui_g_Ungroup_save_L->start_y = 54;
    ui_g_Ungroup_save_L->color = 6;
    ui_g_Ungroup_save_L->width = 3;

    ui_g_Ungroup_save_R->figure_tpye = 2;
    ui_g_Ungroup_save_R->layer = 1;
    ui_g_Ungroup_save_R->r = 50;
    ui_g_Ungroup_save_R->start_x = 1068;
    ui_g_Ungroup_save_R->start_y = 55;
    ui_g_Ungroup_save_R->color = 6;
    ui_g_Ungroup_save_R->width = 3;

    ui_g_Ungroup_Yaw_num->figure_tpye = 6;
    ui_g_Ungroup_Yaw_num->layer = 3;
    ui_g_Ungroup_Yaw_num->font_size = 20;
    ui_g_Ungroup_Yaw_num->start_x = 185;
    ui_g_Ungroup_Yaw_num->start_y = 541;
    ui_g_Ungroup_Yaw_num->color = 8;
    ui_g_Ungroup_Yaw_num->number = 0;
    ui_g_Ungroup_Yaw_num->width = 2;

    ui_g_Ungroup_Pitch_num->figure_tpye = 6;
    ui_g_Ungroup_Pitch_num->layer = 3;
    ui_g_Ungroup_Pitch_num->font_size = 20;
    ui_g_Ungroup_Pitch_num->start_x = 185;
    ui_g_Ungroup_Pitch_num->start_y = 590;
    ui_g_Ungroup_Pitch_num->color = 8;
    ui_g_Ungroup_Pitch_num->number = 0;
    ui_g_Ungroup_Pitch_num->width = 2;

    ui_g_Ungroup_Roll_num->figure_tpye = 6;
    ui_g_Ungroup_Roll_num->layer = 3;
    ui_g_Ungroup_Roll_num->font_size = 20;
    ui_g_Ungroup_Roll_num->start_x = 185;
    ui_g_Ungroup_Roll_num->start_y = 648;
    ui_g_Ungroup_Roll_num->color = 8;
    ui_g_Ungroup_Roll_num->number = 0;
    ui_g_Ungroup_Roll_num->width = 2;

    ui_g_Ungroup_Z_num->figure_tpye = 6;
    ui_g_Ungroup_Z_num->layer = 3;
    ui_g_Ungroup_Z_num->font_size = 20;
    ui_g_Ungroup_Z_num->start_x = 1559;
    ui_g_Ungroup_Z_num->start_y = 653;
    ui_g_Ungroup_Z_num->color = 8;
    ui_g_Ungroup_Z_num->number = 0;
    ui_g_Ungroup_Z_num->width = 2;

    ui_g_Ungroup_Y_num->figure_tpye = 6;
    ui_g_Ungroup_Y_num->layer = 3;
    ui_g_Ungroup_Y_num->font_size = 20;
    ui_g_Ungroup_Y_num->start_x = 1559;
    ui_g_Ungroup_Y_num->start_y = 593;
    ui_g_Ungroup_Y_num->color = 8;
    ui_g_Ungroup_Y_num->number = 0;
    ui_g_Ungroup_Y_num->width = 2;

    ui_g_Ungroup_X_num->figure_tpye = 6;
    ui_g_Ungroup_X_num->layer = 3;
    ui_g_Ungroup_X_num->font_size = 20;
    ui_g_Ungroup_X_num->start_x = 1559;
    ui_g_Ungroup_X_num->start_y = 531;
    ui_g_Ungroup_X_num->color = 8;
    ui_g_Ungroup_X_num->number = 0;
    ui_g_Ungroup_X_num->width = 2;

    ui_g_Ungroup_reset->figure_tpye = 1;
    ui_g_Ungroup_reset->layer = 4;
    ui_g_Ungroup_reset->start_x = 822;
    ui_g_Ungroup_reset->start_y = 246-90;
    ui_g_Ungroup_reset->color = 6;
    ui_g_Ungroup_reset->width = 5;
    ui_g_Ungroup_reset->end_x = 972 + 30;
    ui_g_Ungroup_reset->end_y = 389-90;

    ui_g_Ungroup_Direction->figure_tpye = 7;
    ui_g_Ungroup_Direction->layer = 1;
    ui_g_Ungroup_Direction->font_size = 50;
    ui_g_Ungroup_Direction->start_x = 884;
    ui_g_Ungroup_Direction->start_y = 866;
    ui_g_Ungroup_Direction->color = 6;
    ui_g_Ungroup_Direction->str_length = 3;
    ui_g_Ungroup_Direction->width = 5;
    strcpy(ui_g_Ungroup_Direction->string, "DIR");

    ui_g_Ungroup_Cup_pitch->figure_tpye = 7;
    ui_g_Ungroup_Cup_pitch->layer = 1;
    ui_g_Ungroup_Cup_pitch->font_size = 20;
    ui_g_Ungroup_Cup_pitch->start_x = 39;
    ui_g_Ungroup_Cup_pitch->start_y = 816;
    ui_g_Ungroup_Cup_pitch->color = 2;
    ui_g_Ungroup_Cup_pitch->str_length = 10;
    ui_g_Ungroup_Cup_pitch->width = 2;
    strcpy(ui_g_Ungroup_Cup_pitch->string, "Cup_pitch:");

    ui_g_Ungroup_Mode->figure_tpye = 7;
    ui_g_Ungroup_Mode->layer = 1;
    ui_g_Ungroup_Mode->font_size = 20;
    ui_g_Ungroup_Mode->start_x = 41;
    ui_g_Ungroup_Mode->start_y = 753;
    ui_g_Ungroup_Mode->color = 1;
    ui_g_Ungroup_Mode->str_length = 5;
    ui_g_Ungroup_Mode->width = 2;
    strcpy(ui_g_Ungroup_Mode->string, "Mode:");

    ui_g_Ungroup_Keys_can_mode->figure_tpye = 7;
    ui_g_Ungroup_Keys_can_mode->layer = 1;
    ui_g_Ungroup_Keys_can_mode->font_size = 20;
    ui_g_Ungroup_Keys_can_mode->start_x = 153;
    ui_g_Ungroup_Keys_can_mode->start_y = 753;
    ui_g_Ungroup_Keys_can_mode->color = 6;
    ui_g_Ungroup_Keys_can_mode->str_length = 12;
    ui_g_Ungroup_Keys_can_mode->width = 2;
    strcpy(ui_g_Ungroup_Keys_can_mode->string, "Chassis_mode");

    ui_g_Ungroup_Z->figure_tpye = 7;
    ui_g_Ungroup_Z->layer = 2;
    ui_g_Ungroup_Z->font_size = 20;
    ui_g_Ungroup_Z->start_x = 1490;
    ui_g_Ungroup_Z->start_y = 653;
    ui_g_Ungroup_Z->color = 8;
    ui_g_Ungroup_Z->str_length = 2;
    ui_g_Ungroup_Z->width = 2;
    strcpy(ui_g_Ungroup_Z->string, "Z:");

    ui_g_Ungroup_Y->figure_tpye = 7;
    ui_g_Ungroup_Y->layer = 2;
    ui_g_Ungroup_Y->font_size = 20;
    ui_g_Ungroup_Y->start_x = 1488;
    ui_g_Ungroup_Y->start_y = 592;
    ui_g_Ungroup_Y->color = 8;
    ui_g_Ungroup_Y->str_length = 2;
    ui_g_Ungroup_Y->width = 2;
    strcpy(ui_g_Ungroup_Y->string, "Y:");

    ui_g_Ungroup_X->figure_tpye = 7;
    ui_g_Ungroup_X->layer = 2;
    ui_g_Ungroup_X->font_size = 20;
    ui_g_Ungroup_X->start_x = 1491;
    ui_g_Ungroup_X->start_y = 533;
    ui_g_Ungroup_X->color = 8;
    ui_g_Ungroup_X->str_length = 2;
    ui_g_Ungroup_X->width = 2;
    strcpy(ui_g_Ungroup_X->string, "X:");

    ui_g_Ungroup_Yaw->figure_tpye = 7;
    ui_g_Ungroup_Yaw->layer = 2;
    ui_g_Ungroup_Yaw->font_size = 20;
    ui_g_Ungroup_Yaw->start_x = 39;
    ui_g_Ungroup_Yaw->start_y = 541;
    ui_g_Ungroup_Yaw->color = 8;
    ui_g_Ungroup_Yaw->str_length = 4;
    ui_g_Ungroup_Yaw->width = 2;
    strcpy(ui_g_Ungroup_Yaw->string, "Yaw:");

    ui_g_Ungroup_Pitch->figure_tpye = 7;
    ui_g_Ungroup_Pitch->layer = 2;
    ui_g_Ungroup_Pitch->font_size = 20;
    ui_g_Ungroup_Pitch->start_x = 41;
    ui_g_Ungroup_Pitch->start_y = 590;
    ui_g_Ungroup_Pitch->color = 8;
    ui_g_Ungroup_Pitch->str_length = 6;
    ui_g_Ungroup_Pitch->width = 2;
    strcpy(ui_g_Ungroup_Pitch->string, "Pitch:");

    ui_g_Ungroup_Roll->figure_tpye = 7;
    ui_g_Ungroup_Roll->layer = 2;
    ui_g_Ungroup_Roll->font_size = 20;
    ui_g_Ungroup_Roll->start_x = 41;
    ui_g_Ungroup_Roll->start_y = 645;
    ui_g_Ungroup_Roll->color = 8;
    ui_g_Ungroup_Roll->str_length = 5;
    ui_g_Ungroup_Roll->width = 2;
    strcpy(ui_g_Ungroup_Roll->string, "Roll:");

    uint32_t idx = 0;
    for (int i = 0; i < TOTAL_FIGURE; i++)
    {
        ui_g_now_figures[i].figure_name[2] = idx & 0xFF;
        ui_g_now_figures[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_figures[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_figures[i].operate_tpyel = 1;
#ifndef MANUAL_DIRTY
        ui_g_last_figures[i] = ui_g_now_figures[i];
#endif
        ui_g_dirty_figure[i] = 1;
        idx++;
    }
    for (int i = 0; i < TOTAL_STRING; i++)
    {
        ui_g_now_strings[i].figure_name[2] = idx & 0xFF;
        ui_g_now_strings[i].figure_name[1] = (idx >> 8) & 0xFF;
        ui_g_now_strings[i].figure_name[0] = (idx >> 16) & 0xFF;
        ui_g_now_strings[i].operate_tpyel = 1;
#ifndef MANUAL_DIRTY
        ui_g_last_strings[i] = ui_g_now_strings[i];
#endif
        ui_g_dirty_string[i] = 1;
        idx++;
    }

    ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, ui_g_now_strings, ui_g_dirty_string, TOTAL_FIGURE, TOTAL_STRING);

    for (int i = 0; i < TOTAL_FIGURE; i++)
    {
        ui_g_now_figures[i].operate_tpyel = 2;
    }
    for (int i = 0; i < TOTAL_STRING; i++)
    {
        ui_g_now_strings[i].operate_tpyel = 2;
    }
}

void ui_update_g()
{
#ifndef MANUAL_DIRTY
    for (int i = 0; i < TOTAL_FIGURE; i++)
    {
        if (memcmp(&ui_g_now_figures[i], &ui_g_last_figures[i], sizeof(ui_g_now_figures[i])) != 0)
        {
            ui_g_dirty_figure[i] = 1;
            ui_g_last_figures[i] = ui_g_now_figures[i];
        }
    }
    for (int i = 0; i < TOTAL_STRING; i++)
    {
        if (memcmp(&ui_g_now_strings[i], &ui_g_last_strings[i], sizeof(ui_g_now_strings[i])) != 0)
        {
            ui_g_dirty_string[i] = 1;
            ui_g_last_strings[i] = ui_g_now_strings[i];
        }
    }
#endif
    ui_scan_and_send(ui_g_now_figures, ui_g_dirty_figure, ui_g_now_strings, ui_g_dirty_string, TOTAL_FIGURE, TOTAL_STRING);
}
