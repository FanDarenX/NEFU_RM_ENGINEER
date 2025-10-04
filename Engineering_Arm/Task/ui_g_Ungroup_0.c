//
// Created by RM UI Designer
//

#include "ui_g_Ungroup_0.h"

#define FRAME_ID 0
#define GROUP_ID 0
#define START_ID 0
#define OBJ_NUM 1
#define FRAME_OBJ_NUM 1

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_g_Ungroup_0;
ui_interface_rect_t *ui_g_Ungroup_NewRect = (ui_interface_rect_t *)&(ui_g_Ungroup_0.data[0]);

void _ui_init_g_Ungroup_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_g_Ungroup_0.data[i].figure_name[0] = FRAME_ID;
        ui_g_Ungroup_0.data[i].figure_name[1] = GROUP_ID;
        ui_g_Ungroup_0.data[i].figure_name[2] = i + START_ID;
        ui_g_Ungroup_0.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_g_Ungroup_0.data[i].operate_tpyel = 0;
    }

    ui_g_Ungroup_NewRect->figure_tpye = 1;
    ui_g_Ungroup_NewRect->layer = 0;
    ui_g_Ungroup_NewRect->start_x = 822;
    ui_g_Ungroup_NewRect->start_y = 246;
    ui_g_Ungroup_NewRect->color = 6;
    ui_g_Ungroup_NewRect->width = 5;
    ui_g_Ungroup_NewRect->end_x = 972+30;
    ui_g_Ungroup_NewRect->end_y = 389;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

void _ui_update_g_Ungroup_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_g_Ungroup_0.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

void _ui_remove_g_Ungroup_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_g_Ungroup_0.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}
