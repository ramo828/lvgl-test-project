// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.1
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#ifndef _SQUARELINE_PROJECT_UI_H
#define _SQUARELINE_PROJECT_UI_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined __has_include
#if __has_include("lvgl.h")
#include "lvgl.h"
#elif __has_include("lvgl/lvgl.h")
#include "lvgl/lvgl.h"
#else
#include "lvgl.h"
#endif
#else
#include "lvgl.h"
#endif

#include "ui_helpers.h"
#include "ui_events.h"

// SCREEN: ui_window
void ui_window_screen_init(void);
extern lv_obj_t * ui_window;
extern lv_obj_t * ui_scale;
extern lv_obj_t * ui_value;
extern lv_obj_t * ui_red;
extern lv_obj_t * ui_green;
extern lv_obj_t * ui_blue;
extern lv_obj_t * ui_brigtness;
extern lv_obj_t * ui_display;
extern lv_obj_t * ui_brighttext;
void ui_event_setbutton(lv_event_t * e);
extern lv_obj_t * ui_setbutton;
extern lv_obj_t * ui_bttext;
void ui_event_nextscreen(lv_event_t * e);
extern lv_obj_t * ui_nextscreen;
extern lv_obj_t * ui_Label9;
extern lv_obj_t * ui_backlight;
extern lv_obj_t * ui_backlightvalue1;
void ui_event_autoback(lv_event_t * e);
extern lv_obj_t * ui_autoback;
extern lv_obj_t * ui_autobacktext;
void ui_event_rgbbutton(lv_event_t * e);
extern lv_obj_t * ui_rgbbutton;
extern lv_obj_t * ui_rgbtext;
// CUSTOM VARIABLES

// SCREEN: ui_window1
void ui_window1_screen_init(void);
extern lv_obj_t * ui_window1;
void ui_event_back(lv_event_t * e);
extern lv_obj_t * ui_back;
extern lv_obj_t * ui_Label6;
extern lv_obj_t * ui_Container2;
extern lv_obj_t * ui_qrpanel;
extern lv_obj_t * ui_backlightvalue;
// CUSTOM VARIABLES

// SCREEN: ui_window2
void ui_window2_screen_init(void);
extern lv_obj_t * ui_window2;
extern lv_obj_t * ui_Panel3;
extern lv_obj_t * ui_redslider;
extern lv_obj_t * ui_greenslider;
extern lv_obj_t * ui_blueslider;
extern lv_obj_t * ui_reddis;
extern lv_obj_t * ui_greendis;
extern lv_obj_t * ui_bluedis;
void ui_event_back2(lv_event_t * e);
extern lv_obj_t * ui_back2;
extern lv_obj_t * ui_backtext;
// CUSTOM VARIABLES

// EVENTS
extern lv_obj_t * ui____initial_actions0;

// FONTS
LV_FONT_DECLARE(ui_font_Font1);

// UI INIT
void ui_init(void);

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif
