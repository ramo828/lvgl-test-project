#include <lvgl.h>
#include <extra/libs/qrcode/lv_qrcode.h>
#include <TFT_eSPI.h>
#include "ui.h"

#define LV_HOR_RES 320
#define LV_VER_RES 240
#define LV_COLOR_DEPTH 16

TFT_eSPI tft = TFT_eSPI();
TFT_eSPI *lcd = &tft;

#define LDR 34
#define RED 4
#define GREEN 16
#define BLUE 17
#define BACKLIGHT_PIN 27  // TFT ekran arka ışık pinini belirleyin (örnek pin)

bool hidden_state = false;
bool auto_back_state = false;
int value = 0;
int backlight_value = 0;
int current_backlight = 0;  // Geçerli arka ışık değeri

static lv_color_t buf[LV_HOR_RES * LV_VER_RES / 3];
static lv_disp_draw_buf_t disp_buf;

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = area->x2 - area->x1 + 1;
  uint32_t h = area->y2 - area->y1 + 1;

  lcd->startWrite();
  lcd->setAddrWindow(area->x1, area->y1, w, h);
  lcd->pushColors((uint16_t *)color_p, w * h, true);
  lcd->endWrite();

  lv_disp_flush_ready(disp);
}

void qr() {
  const char *data = "https://github.com/ramo828";
  lv_obj_t *qr = lv_qrcode_create(ui_qrpanel, 175, lv_color_hex3(0x000), lv_color_hex3(0xFFF));
  lv_qrcode_update(qr, data, strlen(data));
  lv_obj_center(qr);
}

void set_rgb(char flag, int brightness) {
  if (flag == 'R') analogWrite(RED, brightness);
  if (flag == 'G') analogWrite(GREEN, brightness);
  if (flag == 'B') analogWrite(BLUE, brightness);
}

void my_touchpad_read(lv_indev_drv_t *indev, lv_indev_data_t *data) {
  uint16_t touchX, touchY;
  bool touched = tft.getTouch(&touchX, &touchY);
  if (touched) {
    data->state = LV_INDEV_STATE_PR;
    data->point.x = touchX;
    data->point.y = touchY;
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

void get_ldr_value() {
  value = analogRead(LDR);
  value = map(value, 0, 4095, 100, 0);
}

// Yumuşak geçişli backlight fonksiyonu
void set_backlight_smooth(int target_brightness) {
  target_brightness = map(target_brightness, 0, 100, 0, 255);

  // Küçük adımlarla parlaklık değişimi
  if (current_backlight < target_brightness) {
    current_backlight += min(5, target_brightness - current_backlight);  // 5 birimlik artış
  } else if (current_backlight > target_brightness) {
    current_backlight -= min(5, current_backlight - target_brightness);  // 5 birimlik azalış
  }

  analogWrite(BACKLIGHT_PIN, current_backlight);
}

void show_widget(lv_event_t *e) {
  hidden_state = !hidden_state;
  if (hidden_state) {
    lv_label_set_text(ui_bttext, "Show");
    lv_obj_add_flag(ui_scale, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_label_set_text(ui_bttext, "Hide");
    lv_obj_clear_flag(ui_scale, LV_OBJ_FLAG_HIDDEN);
  }
}

void backlight_auto(lv_event_t *e) {
  auto_back_state = !auto_back_state;
  if (auto_back_state) {
    lv_label_set_text(ui_autobacktext, "Manual");
    lv_obj_add_state(ui_backlight, LV_STATE_DISABLED);
  } else {
    lv_label_set_text(ui_autobacktext, "Auto");
    lv_obj_clear_state(ui_backlight, LV_STATE_DISABLED);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);

  tft.begin();
  tft.setRotation(1);
  uint16_t touch_cal_data[5] = { 260, 3375, 378, 3366, 1 };
  tft.setTouch(touch_cal_data);

  lv_init();
  lv_disp_draw_buf_init(&disp_buf, buf, NULL, LV_HOR_RES * LV_VER_RES / 10);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.draw_buf = &disp_buf;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.hor_res = LV_HOR_RES;
  disp_drv.ver_res = LV_VER_RES;
  lv_disp_drv_register(&disp_drv);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  ui_init();
  qr();
}

void loop() {
  lv_timer_handler();
  get_ldr_value();
  int brightness = lv_slider_get_value(ui_brigtness);
  int r_slider = lv_slider_get_value(ui_redslider);
  int g_slider = lv_slider_get_value(ui_greenslider);
  int b_slider = lv_slider_get_value(ui_blueslider);
  bool r_switch = lv_obj_has_state(ui_red, LV_STATE_CHECKED);
  bool g_switch = lv_obj_has_state(ui_green, LV_STATE_CHECKED);
  bool b_switch = lv_obj_has_state(ui_blue, LV_STATE_CHECKED);
  lv_label_set_text(ui_reddis, String(r_slider).c_str());
  lv_label_set_text(ui_greendis, String(g_slider).c_str());
  lv_label_set_text(ui_bluedis, String(b_slider).c_str());

  if (!r_switch) {
    Serial.println("aktiv");
    // set_rgb('R', r_slider);
    // set_rgb('G', g_slider);
    // set_rgb('B', b_slider);
    Serial.println(r_slider);
    Serial.println(g_slider);
    Serial.println(b_slider);

  }
  int backlight_val = lv_arc_get_value(ui_backlight);
  int scaled_value = map(value, 60, 100, 0, 100);
  scaled_value = constrain(scaled_value, 0, 100);  // Değerin 0-100 aralığında kalmasını sağla
  lv_arc_set_value(ui_scale, value);
  lv_label_set_text(ui_value, String(value).c_str());
  lv_label_set_text(ui_brighttext, String(brightness).c_str());
  lv_label_set_text(ui_backlightvalue1, auto_back_state ? String(scaled_value).c_str() : String(backlight_val).c_str());

  brightness = map(brightness, 0, 255, 255, 0);

  // Yumuşak geçişli backlight kontrolü
  if (auto_back_state) {
    set_backlight_smooth(scaled_value);  // Otomatik modda yumuşak geçiş
  } else {
    set_backlight_smooth(backlight_val);  // Manuel mod
  }

  if (r_switch) set_rgb('R', brightness);
  else set_rgb('R', 0);

  if (g_switch) set_rgb('G', brightness);
  else set_rgb('G', 0);

  if (b_switch) set_rgb('B', brightness);
  else set_rgb('B', 0);
}
