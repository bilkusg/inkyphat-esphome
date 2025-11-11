#include "inkyphat_epaper.h"
#include <bitset>
#include <cinttypes>
#include "esphome/core/application.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include <freertos/FreeRTOS.h>
#include "esp_task_wdt.h"

namespace esphome {
namespace inkyphat_epaper {

static const char *const TAG = "inkyphat_epaper";

static const uint8_t LUT_SIZE_BLIX = 70;

static const uint8_t FULL_UPDATE_LUT_BLIX[LUT_SIZE_BLIX] = {
                0b01001000, 0b10100000, 0b00010000, 0b00010000, 0b00010011, 0b00000000, 0b00000000,
                0b01001000, 0b10100000, 0b10000000, 0b00000000, 0b00000011, 0b00000000, 0b00000000,
                0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
                0b01001000, 0b10100101, 0b00000000, 0b10111011, 0b00000000, 0b00000000, 0b00000000,
                0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
                0x10, 0x04, 0x04, 0x04, 0x04,
                0x10, 0x04, 0x04, 0x04, 0x04,
                0x04, 0x08, 0x08, 0x10, 0x10,
                0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00,
};
static const uint8_t FULL_UPDATE_LUT_RED_BLIX[LUT_SIZE_BLIX] = {
                0b01001000, 0b10100000, 0b00010000, 0b00010000, 0b00010011, 0b00000000, 0b00000000,
                0b01001000, 0b10100000, 0b10000000, 0b00000000, 0b00000011, 0b00000000, 0b00000000,
                0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
                0b01001000, 0b10100101, 0b00000000, 0b10111011, 0b00000000, 0b00000000, 0b00000000,
                0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b00000000,
                0x40, 0x0C, 0x20, 0x0C, 0x06,
                0x10, 0x08, 0x04, 0x04, 0x06,
                0x04, 0x08, 0x08, 0x10, 0x10,
                0x02, 0x02, 0x02, 0x40, 0x20,
                0x02, 0x02, 0x02, 0x02, 0x02,
                0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x00, 0x00
};
static const uint8_t PARTIAL_UPDATE_LUT_BLIX[LUT_SIZE_BLIX] = { // THIS PROBABLY DOESN'T WORK
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // LUT0: BB:     VS 0 ~7
    0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // LUT1: BW:     VS 0 ~7
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // LUT2: WB:     VS 0 ~7
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // LUT3: WW:     VS 0 ~7
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // LUT4: VCOM:   VS 0 ~7
    0x0A, 0x00, 0x00, 0x00, 0x00,              // TP0 A~D RP0
    0x00, 0x00, 0x00, 0x00, 0x00,              // TP1 A~D RP1
    0x00, 0x00, 0x00, 0x00, 0x00,              // TP2 A~D RP2
    0x00, 0x00, 0x00, 0x00, 0x00,              // TP3 A~D RP3
    0x00, 0x00, 0x00, 0x00, 0x00,              // TP4 A~D RP4
    0x00, 0x00, 0x00, 0x00, 0x00,              // TP5 A~D RP5
    0x00, 0x00, 0x00, 0x00, 0x00,              // TP6 A~D RP6
};
void InkyphatEPaperBase::setup() {
  this->init_internal_(this->get_buffer_length_());
  this->setup_pins_();
  this->spi_setup();
  this->reset_();
  this->initialize();
}
void InkyphatEPaperBase::setup_pins_() {
  this->dc_pin_->setup();  // OUTPUT
  this->dc_pin_->digital_write(false);
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();  // OUTPUT
    this->reset_pin_->digital_write(true);
  }
  if (this->busy_pin_ != nullptr) {
    this->busy_pin_->setup();  // INPUT
  }
}
float InkyphatEPaperBase::get_setup_priority() const { return setup_priority::PROCESSOR; }
void InkyphatEPaperBase::command(uint8_t value) {
  this->start_command_();
  this->write_byte(value);
  this->end_command_();
}
void InkyphatEPaperBase::data(uint8_t value) {
  this->start_data_();
  this->write_byte(value);
  this->end_data_();
}

// write a command followed by one or more bytes of data.
// The command is the first byte, length is the total including cmd.
void InkyphatEPaperBase::cmd_data(const uint8_t *c_data, size_t length) {
  this->dc_pin_->digital_write(false);
  this->enable();
  this->write_byte(c_data[0]);
  this->dc_pin_->digital_write(true);
  this->write_array(c_data + 1, length - 1);
  this->disable();
}

bool InkyphatEPaperBase::wait_until_idle_() {
  if (this->busy_pin_ == nullptr || !this->busy_pin_->digital_read()) {
    return true;
  }

  const uint32_t start = millis();
  esp_err_t err ;
  //err = esp_task_wdt_delete(NULL);
  //if (err != ESP_OK ) {
    //ESP_LOGE(TAG,"Failed to delete timer watchdog  error %d",err);
  //}
  //err = esp_task_wdt_deinit();
  //if (err != ESP_OK ) {
    //ESP_LOGE(TAG,"Failed to deinit timer watchdog  error %d",err);
  //}
  esp_task_wdt_config_t config = {
    .timeout_ms = 50000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  //err= esp_task_wdt_reconfigure(&config);
  //if (err != ESP_OK ) {
    //ESP_LOGE(TAG,"Failed to init timer watchdog  error %d",err);
  //}
  //esp_task_wdt_add(NULL);
  while (this->busy_pin_->digital_read()) {
    if (millis() - start > this->idle_timeout_()) {
      ESP_LOGE(TAG, "Long wait for busy pin %d",millis() - start);
    }
    err= esp_task_wdt_reset();
    if (err != ESP_OK ) {
     ESP_LOGE(TAG,"Failed to init timer watchdog  error %d",err);
    }
    yield();
    vTaskDelay(pdMS_TO_TICKS(1000));
    vTaskDelay(1 / portTICK_PERIOD_MS);
    delay(1000);
  }
  config.timeout_ms = 5000;
  //err= esp_task_wdt_reconfigure(&config);
  //if (err != ESP_OK ) {
    //ESP_LOGE(TAG,"Failed to init timer watchdog  error %d",err);
  //}
  return true;
}
void InkyphatEPaperBase::update() {
  this->do_update_();
  this->display();
}
void InkyphatEPaperBase::start_command_() {
  this->dc_pin_->digital_write(false);
  this->enable();
}
void InkyphatEPaperBase::end_command_() { this->disable(); }
void InkyphatEPaperBase::start_data_() {
  this->dc_pin_->digital_write(true);
  this->enable();
}
void InkyphatEPaperBase::end_data_() { this->disable(); }
void InkyphatEPaperBase::on_safe_shutdown() { if (false) this->deep_sleep(); }

// ========================================================
//                      BWR starts here 
// ========================================================
bool InkyphatEPaperBWR::isBusy() {
   return  this->busy_pin_->digital_read();
}
InkyphatEPaperBWR::InkyphatEPaperBWR(int modeli) : model_(InkyphatEPaperBWRModel(modeli)) {}
void InkyphatEPaperBWR::fill(Color color) {
// A rotten hack, but we use this as a way of doing special stuff by sending invalid colours
  int buflen = this->get_buffer_length_();
  uint32_t c1 = 0;
  uint32_t c2 = 0;

// 0 0 is black
// 0 1 is off-white
// 1 0 is white
// 1 1 is red
  if ((color.white == 255) && (color.red == 255) && (color.green == 255) && (color.blue == 255)) {
    // black 00
    c1 = 0;
    c2 = 0;
  } else if (( color.red > 0 ) && ( color.green == 0) && (color.blue == 0)) {
    // red 11
    c1 = 0xff;
    c2 = 0xff;
  } else  {
    // white 10 for all other colours
    c1 = 0xff;
    c2 = 0;
  }

  uint32_t i=0;
  for( i;i < buflen / 2 ;i++)
  {
     this->buffer_[i] = c1;
  }
  for( i;i < buflen ; i++)
  {
     this->buffer_[i] = c2;
  }  
}

void HOT InkyphatEPaperBWR::draw_absolute_pixel_internal(int x, int y, Color color) {
  if (x >= this->get_width_internal() || y >= this->get_height_internal() || x < 0 || y < 0)
    return;

  const uint32_t buf_half_len = this->get_buffer_length_() / 2u;

  const uint32_t pos = (x + y * this->get_width_controller()) / 8u;
  const uint8_t subpos = x & 0x07;
  if ((color.white == 255) && (color.red == 255) && (color.green == 255) && (color.blue == 255)) {
    // black 00
    this->buffer_[pos] &= ~(0x80 >> subpos);
    this->buffer_[pos + buf_half_len] &= ~(0x80 >> subpos);
  } else if (( color.red > 0 ) && ( color.green == 0) && (color.blue == 0)) {
    // red 11
    this->buffer_[pos] |= 0x80 >> subpos;
    this->buffer_[pos + buf_half_len] |= 0x80 >> subpos;
  } else  {
    // white 10 for all other colours
    this->buffer_[pos] |= 0x80 >> subpos;
    this->buffer_[pos + buf_half_len] &= ~(0x80 >> subpos);
  }

}

uint32_t InkyphatEPaperBWR::get_buffer_length_() {
  return this->get_width_controller() * this->get_height_internal() / 4u;
}  // black and red buffer


void InkyphatEPaperBWR::initialize() {
  // Achieve display intialization
  this->init_display_();
  // If a reset pin is configured, eligible displays can be set to deep sleep
  // between updates, as recommended by the hardware provider
  if (this->reset_pin_ != nullptr) {
    if (false) {
        this->deep_sleep_between_updates_ = true;
        ESP_LOGI(TAG, "Set the display to deep sleep");
        this->deep_sleep();
    }
  }
}
void InkyphatEPaperBWR::init_display_() {
    if (this->reset_pin_ != nullptr) {
      this->reset_pin_->digital_write(false);
      delay(10);
      this->reset_pin_->digital_write(true);
      delay(10);
      this->wait_until_idle_();
    }

    this->command(0x12);  // SWRESET
    this->wait_until_idle_();

  // COMMAND DRIVER OUTPUT CONTROL
  this->command(0x01);
  this->data(this->get_height_internal() - 1);
  this->data((this->get_height_internal() - 1) >> 8);
  this->data(0x00);  // ? GD = 0, SM = 0, TB = 0

  // COMMAND BOOSTER SOFT START CONTROL
  this->command(0x0C);
  this->data(0xD7);
  this->data(0xD6);
  this->data(0x9D);

  // COMMAND WRITE VCOM REGISTER
  this->command(0x2C);
  this->data(0xA8);

  // COMMAND SET DUMMY LINE PERIOD
  this->command(0x3A);
  this->data(0x1A);

  // COMMAND SET GATE TIME
  this->command(0x3B);
  this->data(0x08);  // 2µs per row

  // COMMAND DATA ENTRY MODE SETTING
  this->command(0x11);
      this->data(0x03);  // from top left to bottom right
      // RAM content option for Display Update
      this->command(0x21); this->data(0x00); this->data(0x80);
}
void InkyphatEPaperBWR::dump_config() {
  LOG_DISPLAY("", "Inkyphat E-Paper", this);
  switch (this->model_) {
    case INKYPHAT_EPAPER_2_13_IN_V2:
      ESP_LOGCONFIG(TAG, "  Model: 2.13inV2 (INKYPHAT)");
      break;
  }
  ESP_LOGCONFIG(TAG, "  Full Update Every: %" PRIu32, this->full_update_every_);
  ESP_LOGCONFIG(TAG, "  BLIX component RED LUT v1");
  LOG_PIN("  Reset Pin: ", this->reset_pin_);
  LOG_PIN("  DC Pin: ", this->dc_pin_);
  LOG_PIN("  Busy Pin: ", this->busy_pin_);
  LOG_UPDATE_INTERVAL(this);
}
void HOT InkyphatEPaperBWR::display() {
  uint32_t buf_len_half = this->get_buffer_length_() >> 1;
  bool full_update =  this->at_update_ == 0;
  bool prev_full_update = this->at_update_ == 1;

  //ESP_LOGI(TAG, "Update display");
  if (this->deep_sleep_between_updates_) {
    ESP_LOGI(TAG, "Wake up the display");
    this->reset_();
    this->wait_until_idle_();
    this->init_display_();
  }

  if (!this->wait_until_idle_()) {
    this->status_set_warning();
    ESP_LOGI(TAG, "timeout waiting for idle");
    return;
  }
  // ESP_LOGI(TAG,"Display is idle and ready for commands");
  if (this->full_update_every_ >= 1) {
    if (full_update != prev_full_update) {
          this->write_lut_(full_update ? FULL_UPDATE_LUT_RED_BLIX : PARTIAL_UPDATE_LUT_BLIX, LUT_SIZE_BLIX);
      }
    }
    this->at_update_ = (this->at_update_ + 1) % this->full_update_every_;

    // Set VCOM for full or partial update
    this->command(0x2C);
    this->data(full_update ? 0x55 : 0x26);

    if (!full_update) {
      // Enable "ping-pong"
      this->command(0x37);
      this->data(0x00);
      this->data(0x00);
      this->data(0x00);
      this->data(0x00);
      this->data(0x40);
      this->data(0x00);
      this->data(0x00);
      this->command(0x22);
      this->data(0xc0);
      this->command(0x20);
  }

  // Border waveform
      this->command(0x3C);
      this->data(0x0);
      this->command(0x3C);
      // BLIX WAS this->data(full_update ? 0x03 : 0x01);
      const uint32_t border_color_code_black = 0b00000000;
      const uint32_t border_color_code_white = 0b00000001;
      const uint32_t border_color_code_red = 0b00000110;
      uint32_t border_color_code = border_color_code_white;
      
      //this->data(full_update ?  0b01110011 : 0x01);
      this->data(full_update ?  border_color_code : 0x01);

  // Set x & y regions we want to write to (full)
      // COMMAND SET RAM X ADDRESS START END POSITION
      this->command(0x44);
      this->data(0x00);
      this->data((this->get_width_internal() - 1) >> 3);
      // COMMAND SET RAM Y ADDRESS START END POSITION
      this->command(0x45);
      this->data(0x00);
      this->data(0x00);
      this->data(this->get_height_internal() - 1);
      this->data((this->get_height_internal() - 1) >> 8);

      // COMMAND SET RAM X ADDRESS COUNTER
      this->command(0x4E);
      this->data(0x00);
      // COMMAND SET RAM Y ADDRESS COUNTER
      this->command(0x4F);
      this->data(0x00);
      this->data(0x00);

  if (!this->wait_until_idle_()) {
    this->status_set_warning();
    return;
  }

  // COMMAND WRITE RAM
    this->command(0x24);
    this->start_data_();
    this->write_array(this->buffer_, buf_len_half);
    this->end_data_();
    this->command(0x26);
    this->start_data_();
    this->write_array(this->buffer_ + buf_len_half, buf_len_half);
    this->end_data_();
  // COMMAND DISPLAY UPDATE CONTROL 2
  this->command(0x22);
      this->data(full_update ? 0xC7 : 0x0C);

  // COMMAND MASTER ACTIVATION
  this->command(0x20);
  // COMMAND TERMINATE FRAME READ WRITE
  this->command(0xFF);

  this->status_clear_warning();

  // ESP_LOGI(TAG,"Display  commands sent");
  if (this->deep_sleep_between_updates_) {
    ESP_LOGI(TAG, "Set the display back to deep sleep after update");
    this->deep_sleep();
    ESP_LOGI(TAG, "Deep sleep concluded");
  }
}
int InkyphatEPaperBWR::get_width_internal() {
      return 122;
}
// The controller of the 2.13" displays has a buffer larger than screen size
int InkyphatEPaperBWR::get_width_controller() {
      return 128;
}
int InkyphatEPaperBWR::get_height_internal() {
      return 250;
}
void InkyphatEPaperBWR::write_lut_(const uint8_t *lut, const uint8_t size) {
  // COMMAND WRITE LUT REGISTER
  this->command(0x32);
  for (uint8_t i = 0; i < size; i++)
    this->data(lut[i]);
}
void InkyphatEPaperBWR::set_full_update_every(uint32_t full_update_every) {
  this->full_update_every_ = full_update_every;
}

uint32_t InkyphatEPaperBWR::idle_timeout_() {
  return 60000; // need long timeout with red colour too
}

}  // namespace inkyphat_epaper
}  // namespace esphome
