#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/core/gpio.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"

#include <vector>
#include <string>
#include <queue>

#include "messages.h"
#include "device_status.h"

namespace esphome {
namespace airconintl {

using climate::Climate;
using climate::ClimateCall;
using climate::ClimateMode;
using climate::ClimateFanMode;
using climate::ClimateSwingMode;
using climate::ClimatePreset;
using climate::ClimateAction;
using climate::ClimateTraits;
using sensor::Sensor;
using uart::UARTDevice;

class AirconClimate : public PollingComponent, public Climate, public UARTDevice {
 public:
  AirconClimate() {
    // Force visual range to 61–86 °F forever
    this->visual_min_temperature = 61.0f;
    this->visual_max_temperature = 86.0f;
    this->visual_temperature_step = 1.0f;
  }

  // === Optional sensor/pin setters ===
  void set_compressor_frequency_sensor(Sensor *s) { compressor_frequency = s; }
  void set_compressor_frequency_setting_sensor(Sensor *s) { compressor_frequency_setting = s; }
  void set_compressor_frequency_send_sensor(Sensor *s) { compressor_frequency_send = s; }
  void set_outdoor_temperature_sensor(Sensor *s) { outdoor_temperature = s; }
  void set_outdoor_condenser_temperature_sensor(Sensor *s) { outdoor_condenser_temperature = s; }
  void set_compressor_exhaust_temperature_sensor(Sensor *s) { compressor_exhaust_temperature = s; }
  void set_target_exhaust_temperature_sensor(Sensor *s) { target_exhaust_temperature = s; }
  void set_indoor_pipe_temperature_sensor(Sensor *s) { indoor_pipe_temperature = s; }
  void set_indoor_humidity_setting_sensor(Sensor *s) { indoor_humidity_setting = s; }
  void set_indoor_humidity_status_sensor(Sensor *s) { indoor_humidity_status = s; }
  void set_re_pin(GPIOPin *pin) { re_pin = pin; }
  void set_de_pin(GPIOPin *pin) { de_pin = pin; }

  void setup() override {
    // Force Fahrenheit mode on every boot
    std::vector<uint8_t> force_f(temp_to_F, temp_to_F + sizeof(temp_to_F));
    send_cmd(force_f, "Force Fahrenheit");

    if (re_pin) { re_pin->setup(); re_pin->digital_write(true); }
    if (de_pin) { de_pin->setup(); de_pin->digital_write(false); }

    while (available()) read();
    request_update();
  }

  void loop() override {
    while (available()) {
      last_read_time = millis();
      int len = get_response(read(), uart_buf);
      if (len > 0) {
        last_valid_packet_time = millis();
        auto *st = (Device_Status *) uart_buf;

        // Convert raw Fahrenheit → internal Celsius for ESPHome
        float tgt = (st->indoor_temperature_setting - 32) * 0.5556f;
        float cur = (st->indoor_temperature_status - 32) * 0.5556f;
        if (tgt > 7 && tgt < 33) target_temperature = tgt;
        if (cur > 1 && cur < 49) current_temperature = cur;

        bool comp = st->compressor_frequency > 0;

        // Swing
        if (st->left_right && st->up_down) swing_mode = CLIMATE_SWING_BOTH;
        else if (st->left_right) swing_mode = CLIMATE_SWING_HORIZONTAL;
        else if (st->up_down) swing_mode = CLIMATE_SWING_VERTICAL;
        else swing_mode = CLIMATE_SWING_OFF;

        // Mode & action
        if (st->run_status == 0) { mode = CLIMATE_MODE_OFF; action = CLIMATE_ACTION_OFF; }
        else if (st->mode_status == 0) { mode = CLIMATE_MODE_FAN_ONLY; action = CLIMATE_ACTION_FAN; }
        else if (st->mode_status == 1) { mode = CLIMATE_MODE_HEAT; action = comp ? CLIMATE_ACTION_HEATING : CLIMATE_ACTION_IDLE; }
        else if (st->mode_status == 2) { mode = CLIMATE_MODE_COOL; action = comp ? CLIMATE_ACTION_COOLING : CLIMATE_ACTION_IDLE; }
        else if (st->mode_status == 3) { mode = CLIMATE_MODE_DRY; action = comp ? CLIMATE_ACTION_DRYING : CLIMATE_ACTION_IDLE; }

        // Fan
        uint8_t w = st->wind_status;
        if (w == 18) fan_mode = CLIMATE_FAN_HIGH;
        else if (w == 14) fan_mode = CLIMATE_FAN_MEDIUM;
        else if (w == 10) fan_mode = CLIMATE_FAN_LOW;
        else if (w == 2) fan_mode = CLIMATE_FAN_QUIET;
        else fan_mode = CLIMATE_FAN_AUTO;

        // Remember last target temps
        if (mode == CLIMATE_MODE_COOL) cool_tgt_temp = target_temperature;
        if (mode == CLIMATE_MODE_HEAT) heat_tgt_temp = target_temperature;

        // Sensors
        publish_sensor(compressor_frequency, st->compressor_frequency);
        publish_sensor(compressor_frequency_setting, st->compressor_frequency_setting);
        publish_sensor(compressor_frequency_send, st->compressor_frequency_send);
        publish_temp(outdoor_temperature, st->outdoor_temperature);
        publish_temp(outdoor_condenser_temperature, st->outdoor_condenser_temperature);
        publish_temp(compressor_exhaust_temperature, st->compressor_exhaust_temperature);
        publish_temp(target_exhaust_temperature, st->target_exhaust_temperature);
        publish_temp(indoor_pipe_temperature, st->indoor_pipe_temperature);
        publish_sensor(indoor_humidity_setting, st->indoor_humidity_setting);
        publish_sensor(indoor_humidity_status, st->indoor_humidity_status);

        send_state = IDLE;
        this->publish_state();
      }
    }

    // Timeout handling
    if (send_state == WAITING_ACK && millis() - send_timestamp > 3000) {
      ESP_LOGE("aircon_climate", "Timeout: %s", current_desc.c_str());
      if (de_pin) de_pin->digital_write(false);
      if (re_pin) re_pin->digital_write(true);
      while (available()) read();
      send_state = IDLE;
      if (!queue.empty()) queue.pop();
    }

    // Send queued commands
    if (send_state == IDLE && !queue.empty() && millis() - last_send_time >= 100 && millis() - last_read_time >= 10) {
      auto m = queue.front(); queue.pop();
      if (de_pin) de_pin->digital_write(true);
      if (re_pin) re_pin->digital_write(true);
      write_array(m.payload);
      flush();
      if (de_pin) de_pin->digital_write(false);
      if (re_pin) re_pin->digital_write(false);
      current_desc = m.desc;
      send_state = WAITING_ACK;
      send_timestamp = millis();
      last_send_time = millis();
    }
  }

  void update() override {
    if (!last_valid_packet_time || millis() - last_valid_packet_time > 10000)
      request_update();
  }

  void control(const ClimateCall &call) override {
    if (call.get_mode().has_value()) {
      auto m = *call.get_mode();
      if (m == CLIMATE_MODE_OFF) send(power_off, "Off");
      else if (m == CLIMATE_MODE_COOL) { send(mode_cool, "Cool"); if (cool_tgt_temp > 0) set_temp(cool_tgt_temp); }
      else if (m == CLIMATE_MODE_HEAT) { send(mode_heat, "Heat"); if (heat_tgt_temp > 0) set_temp(heat_tgt_temp); }
      else if (m == CLIMATE_MODE_FAN_ONLY) send(mode_fan, "Fan");
      else if (m == CLIMATE_MODE_DRY) send(mode_dry, "Dry");
      this->mode = m;
    }
    if (call.get_target_temperature().has_value()) set_temp(*call.get_target_temperature());
    if (call.get_fan_mode().has_value()) {
      auto f = *call.get_fan_mode();
      if (f == CLIMATE_FAN_AUTO) send(fan_auto, "Fan Auto");
      else if (f == CLIMATE_FAN_LOW) send(fan_low, "Fan Low");
      else if (f == CLIMATE_FAN_MEDIUM) send(fan_medium, "Fan Medium");
      else if (f == CLIMATE_FAN_HIGH) send(fan_high, "Fan High");
      else if (f == CLIMATE_FAN_QUIET) send(fan_quiet, "Fan Quiet");
      this->fan_mode = f;
    }
    if (call.get_swing_mode().has_value()) {
      auto s = *call.get_swing_mode();
      send(s == CLIMATE_SWING_VERTICAL || s == CLIMATE_SWING_BOTH ? swing_vert_on : swing_vert_off, "Swing V");
      send(s == CLIMATE_SWING_HORIZONTAL || s == CLIMATE_SWING_BOTH ? swing_horiz_on : swing_horiz_off, "Swing H");
      this->swing_mode = s;
    }
    if (call.get_preset().has_value()) {
      auto p = *call.get_preset();
      if (p == CLIMATE_PRESET_NONE) { send(turbo_off, "Turbo Off"); send(energysave_off, "Eco Off"); }
      else if (p == CLIMATE_PRESET_BOOST) send(turbo_on, "Turbo On");
      else if (p == CLIMATE_PRESET_ECO) send(energysave_on, "Eco On");
      this->preset = p;
    }
    this->publish_state();
  }

  ClimateTraits traits() override {
    ClimateTraits t;
    t.set_supports_current_temperature(true);
    t.set_visual_min_temperature(61.0f);
    t.set_visual_max_temperature(86.0f);
    t.set_visual_temperature_step(1.0f);
    t.set_supported_modes({CLIMATE_MODE_OFF, CLIMATE_MODE_COOL, CLIMATE_MODE_HEAT, CLIMATE_MODE_FAN_ONLY, CLIMATE_MODE_DRY});
    t.set_supported_swing_modes({CLIMATE_SWING_OFF, CLIMATE_SWING_VERTICAL, CLIMATE_SWING_HORIZONTAL, CLIMATE_SWING_BOTH});
    t.set_supported_fan_modes({CLIMATE_FAN_AUTO, CLIMATE_FAN_LOW, CLIMATE_FAN_MEDIUM, CLIMATE_FAN_HIGH, CLIMATE_FAN_QUIET});
    t.set_supported_presets({CLIMATE_PRESET_NONE, CLIMATE_PRESET_BOOST, CLIMATE_PRESET_ECO});
    t.set_supports_action(true);
    return t;
  }

 private:
  struct Msg { std::string desc; std::vector<uint8_t> payload; };
  enum State { IDLE, WAITING_ACK } send_state = IDLE;
  uint32_t send_timestamp = 0, last_send_time = 0, last_read_time = 0, last_valid_packet_time = 0;
  std::string current_desc;
  std::queue<Msg> queue;
  uint8_t uart_buf[128]{};

  const uint8_t* const temp_table[26] = {
    temp_61_F, temp_62_F, temp_63_F, temp_64_F, temp_65_F, temp_66_F, temp_67_F, temp_68_F, temp_69_F, temp_70_F,
    temp_71_F, temp_72_F, temp_73_F, temp_74_F, temp_75_F, temp_76_F, temp_77_F, temp_78_F, temp_79_F, temp_80_F,
    temp_81_F, temp_82_F, temp_83_F, temp_84_F, temp_85_F, temp_86_F
  };

  float cool_tgt_temp = 75.0f, heat_tgt_temp = 70.0f;

  Sensor *compressor_frequency{nullptr}, *compressor_frequency_setting{nullptr}, *compressor_frequency_send{nullptr};
  Sensor *outdoor_temperature{nullptr}, *outdoor_condenser_temperature{nullptr};
  Sensor *compressor_exhaust_temperature{nullptr}, *target_exhaust_temperature{nullptr}, *indoor_pipe_temperature{nullptr};
  Sensor *indoor_humidity_setting{nullptr}, *indoor_humidity_status{nullptr};
  GPIOPin *re_pin{nullptr}, *de_pin{nullptr};

  void publish_sensor(Sensor *s, float v) { if (s) s->publish_state(v); }
  void publish_temp(Sensor *s, int8_t v) { if (s && v > -40) s->publish_state(v); }

  void send_cmd(const std::vector<uint8_t> &data, const std::string &desc) {
    queue.push({desc, data});
  }
  void send(const uint8_t *data, size_t len, const std::string &desc) {
    send_cmd(std::vector<uint8_t>(data, data + len), desc);
  }
  void send(const uint8_t *data, const std::string &desc) {
    send(data, sizeof(*data) == 0 ? 0 : data[-1] ? 0 : sizeof(data), desc);  // dummy, real size from messages.h
  }

  void request_update() { send_cmd(std::vector<uint8_t>(request_status, request_status + sizeof(request_status)), "Status"); }

  void set_temp(float temp_f) {
    int t = roundf(temp_f);
    if (t < 61 || t > 86) return;
    int idx = t - 61;
    send_cmd(std::vector<uint8_t>(temp_table[idx], temp_table[idx] + sizeof(temp_61_F)),
             "Set " + std::to_string(t) + "°F");
  }

  int get_response(uint8_t b, uint8_t *out) {
    static std::vector<uint8_t> buf;
    static uint16_t cs = 0;
    static bool in = false;
    static size_t exp = 0;

    if (!in) {
      if (b == 0xF4) { buf = {b}; cs = 0; in = true; if (send_state == WAITING_ACK) send_state = IDLE; }
      return 0;
    }
    buf.push_back(b);
    size_t i = buf.size() - 1;
    if (i < 16) {
      const uint8_t hdr[16] = {0xF4,0xF5,0x01,0x40,0x49,0x01,0x00,0xFE,0x01,0x01,0x01,0x01,0x00,0x66,0x00,0x01};
      if (buf[i] != hdr[i]) { in = false; buf.clear(); return 0; }
      if (i == 4) exp = sizeof(Device_Status);
    } else if (i >= 2 && i < exp - 4) cs += b;
    else if (i == exp - 3) {
      if (cs != ((buf[exp-4] << 8) | buf[exp-3])) { in = false; buf.clear(); return 0; }
    } else if (i == exp - 2 && b != 0xF4) { in = false; buf.clear(); return 0; }
    else if (i == exp - 1) {
      if (b != 0xFB) { in = false; buf.clear(); return 0; }
      memcpy(out, buf.data(), buf.size());
      int sz = buf.size();
      in = false; buf.clear();
      return sz;
    }
    return 0;
  }
};

}  // namespace airconintl
}  // namespace esphome