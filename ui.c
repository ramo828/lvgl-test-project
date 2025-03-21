// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.1
// LVGL version: 8.3.11
// Project name: SquareLine_Project

#include "ui.h"
#include "ui_helpers.h"

///////////////////// VARIABLES ////////////////////

// SCREEN: ui_window
void ui_window_screen_init(void);
lv_obj_t * ui_window;
lv_obj_t * ui_scale;
lv_obj_t * ui_value;
lv_obj_t * ui_red;
lv_obj_t * ui_green;
lv_obj_t * ui_blue;
lv_obj_t * ui_brigtness;
lv_obj_t * ui_display;
lv_obj_t * ui_brighttext;
void ui_event_setbutton(lv_event_t * e);
lv_obj_t * ui_setbutton;
lv_obj_t * ui_bttext;
void ui_event_nextscreen(lv_event_t * e);
lv_obj_t * ui_nextscreen;
lv_obj_t * ui_Label9;
lv_obj_t * ui_backlight;
lv_obj_t * ui_backlightvalue1;
void ui_event_autoback(lv_event_t * e);
lv_obj_t * ui_autoback;
lv_obj_t * ui_autobacktext;
void ui_event_rgbbutton(lv_event_t * e);
lv_obj_t * ui_rgbbutton;
lv_obj_t * ui_rgbtext;
// CUSTOM VARIABLES

// SCREEN: ui_window1
void ui_window1_screen_init(void);
lv_obj_t * ui_window1;
void ui_event_back(lv_event_t * e);
lv_obj_t * ui_back;
lv_obj_t * ui_Label6;
lv_obj_t * ui_Container2;
lv_obj_t * ui_qrpanel;
lv_obj_t * ui_backlightvalue;
// CUSTOM VARIABLES

// SCREEN: ui_window2
void ui_window2_screen_init(void);
lv_obj_t * ui_window2;
lv_obj_t * ui_Panel3;
lv_obj_t * ui_redslider;
lv_obj_t * ui_greenslider;
lv_obj_t * ui_blueslider;
lv_obj_t * ui_reddis;
lv_obj_t * ui_greendis;
lv_obj_t * ui_bluedis;
void ui_event_back2(lv_event_t * e);
lv_obj_t * ui_back2;
lv_obj_t * ui_backtext;
// CUSTOM VARIABLES

// EVENTS
lv_obj_t * ui____initial_actions0;

// IMAGES AND IMAGE SETS

///////////////////// TEST LVGL SETTINGS ////////////////////
#if LV_COLOR_DEPTH != 16
    #error "LV_COLOR_DEPTH should be 16bit to match SquareLine Studio's settings"
#endif
#if LV_COLOR_16_SWAP !=0
    #error "LV_COLOR_16_SWAP should be 0 to match SquareLine Studio's settings"
#endif

///////////////////// ANIMATIONS ////////////////////

///////////////////// FUNCTIONS ////////////////////
void ui_event_setbutton(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        show_widget(e);
    }
}

void ui_event_nextscreen(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_window1, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_window1_screen_init);
    }
}

void ui_event_autoback(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        backlight_auto(e);
    }
}

void ui_event_rgbbutton(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_window2, LV_SCR_LOAD_ANIM_FADE_ON, 500, 1, &ui_window2_screen_init);
    }
}

void ui_event_back(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_window, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_window_screen_init);
    }
}

void ui_event_back2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_window, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_window_screen_init);
    }
}

///////////////////// SCREENS ////////////////////

void ui_init(void)
{
    lv_disp_t * dispp = lv_disp_get_default();
    lv_theme_t * theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED),
                                               false, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);
    ui_window_screen_init();
    ui_window1_screen_init();
    ui_window2_screen_init();
    ui____initial_actions0 = lv_obj_create(NULL);
    lv_disp_load_scr(ui_window);
}
