#include "emon.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <utility>

#define TEST_WITH_DAC

namespace esphome {
namespace emon {

static const char *const TAG = "emon";

static TaskHandle_t s_task_handle;

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata,
                                     void *user_data) {
  BaseType_t must_yield = pdFALSE;
  vTaskNotifyGiveFromISR(s_task_handle, &must_yield);

  return (must_yield == pdTRUE);
}

// void Emon::calcVI(int crossings, int timeout) {}

void Emon::setup() {
  memset(this->adc_conv_, 0xcc, READ_LEN);

  s_task_handle = xTaskGetCurrentTaskHandle();

  adc_continuous_handle_cfg_t adc_config = {
      .max_store_buf_size = ADC_MAX_BUF_SIZE,
      .conv_frame_size = READ_LEN,
  };

  esp_err_t err = adc_continuous_new_handle(&adc_config, &this->adc_handle_);
  ESP_LOGI("emon", "%d", err);
  ESP_ERROR_CHECK(err);

  adc_digi_pattern_config_t adc_pattern[2] = {{
                                                  .atten = this->v_attenuation_,
                                                  .channel = this->v_channel_,
                                                  .unit = this->v_adc_unit_,
                                                  .bit_width = ADC_BIT_WIDTH,
                                              },
                                              {
                                                  .atten = this->i_attenuation_,
                                                  .channel = this->i_channel_,
                                                  .unit = this->i_adc_unit_,
                                                  .bit_width = ADC_BIT_WIDTH,
                                              }};

  adc_continuous_config_t dig_cfg = {
      .pattern_num = 2,
      .adc_pattern = adc_pattern,
      .sample_freq_hz = ADC_SAMPLE_RATE_HZ,
      .conv_mode = ADC_CONV_SINGLE_UNIT_1,  // TODO: Assign based on declared pins
      .format = ADC_OUTPUT_TYPE,
  };

  ESP_ERROR_CHECK(adc_continuous_config(this->adc_handle_, &dig_cfg));

  adc_continuous_evt_cbs_t adc_cbs = {
      .on_conv_done = s_conv_done_cb,
      .on_pool_ovf = nullptr,
  };
  ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(this->adc_handle_, &adc_cbs, NULL));
  ESP_ERROR_CHECK(adc_continuous_start(this->adc_handle_));
}

void Emon::loop() {
  esp_err_t ret = ESP_OK;
  uint32_t ret_num = 0;
  uint32_t notif = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
  if (notif == 0) {
    return;
  }
  ret = adc_continuous_read(this->adc_handle_, this->adc_conv_, READ_LEN, &ret_num, 0);
  if (ret == ESP_ERR_TIMEOUT) {
    return;
  }
  if (ret != ESP_OK) {
    return;
  }

  uint16_t v_idx_ = 0;
  uint16_t i_idx_ = 0;

  // ESP_LOGD(TAG, "ret is %x, ret_num is %" PRIu32 " bytes, num_readings is %" PRIu32, ret, ret_num,
  //          ret_num / SOC_ADC_DIGI_RESULT_BYTES);
  for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
    adc_digi_output_data_t *p = (adc_digi_output_data_t *) &this->adc_conv_[i];
    uint16_t unit = ADC_GET_UNIT(p);
    uint16_t chan = ADC_GET_CHANNEL(p);
    uint16_t data = ADC_GET_DATA(p);
    /* Check the channel number validation, the data is invalid if the channel num exceed the maximum channel */
    if (chan > SOC_ADC_CHANNEL_NUM(unit)) {
      ESP_LOGW(TAG, "Invalid data [%" PRIu16 "_%" PRIx16 "]", chan, data);
      continue;
    }
    if (unit == this->v_adc_unit_ && chan == this->v_channel_) {
      this->v_data_[v_idx_++] = data;
    } else if (unit == this->i_adc_unit_ && chan == this->i_channel_) {
      this->i_data_[i_idx_++] = data;
    } else {
      ESP_LOGW(TAG, "Unknown channel [%" PRIu16 "_%" PRIx16 "]", chan, data);
    }
  }

  if (v_idx_ != BUF_LEN || i_idx_ != BUF_LEN) {
    ESP_LOGW(TAG, "Data not complete, v_idx_=%d, i_idx_=%d", v_idx_, i_idx_);
    return;
  }

  uint16_t v_max = 0;
  uint16_t v_min = 1 << ADC_BIT_WIDTH;
  for (int i = 0; i < BUF_LEN; i++) {
    if (this->v_data_[i] > v_max) {
      v_max = this->v_data_[i];
    }
    if (this->v_data_[i] < v_min) {
      v_min = this->v_data_[i];
    }
  }
  uint16_t zero = (v_max + v_min) / 2;
  uint16_t eps = (v_max - zero) * 0.05;

  int first;
  for (first = 0; first < BUF_LEN; first++) {
    if ((this->v_data_[first] < zero + eps) && (this->v_data_[first] > zero - eps)) {
      break;
    }
  }
  if (first == BUF_LEN) {
    ESP_LOGW(TAG, "Voltage data not stable");
    return;
  }

  int last = first;
  for (int j = BUF_LEN - 1; j >= first; j--) {
    if (((this->v_data_[j] - this->v_data_[first]) * (this->v_data_[j - 1] - this->v_data_[first])) <= 0) {
      last = j;
      break;
    }
  }

  if (last == first) {
    ESP_LOGW(TAG, "Voltage data not stable");
    return;
  }

  for (int j = first; j < BUF_LEN; j++) {
    this->v_offset_ += (this->v_data_[j] - this->v_offset_) / 1024;
    this->i_offset_ += (this->i_data_[j] - this->i_offset_) / 1024;
  }

  uint64_t v_rms = 0;
  uint64_t i_rms = 0;
  int64_t p = 0;

  for (int j = first; j < BUF_LEN; j++) {
    this->v_data_[j] -= this->v_offset_;
    this->i_data_[j] -= this->i_offset_;
  }

  for (int j = first; j <= last; j++) {
    v_rms += this->v_data_[j] * this->v_data_[j];
    i_rms += this->i_data_[j] * this->i_data_[j];
    p += this->v_data_[j] * this->i_data_[j];
  }

  float v_out = this->v_cal_ * sqrt(v_rms / (last - first + 1));
  float i_out = this->i_cal_ * sqrt(i_rms / (last - first + 1));
  float p_out = this->v_cal_ * this->i_cal_ * p / (last - first + 1);
  float s_out = v_out * i_out;
  float pf;
  if (s_out == 0) {
    pf = 1;
  } else {
    pf = p_out / s_out;
  }

  ESP_LOGI(TAG, "Vrms=%.2f, Irms=%.2f, P=%.2f, S=%.2f, PF=%.2f", v_out, i_out, p_out, s_out, pf);
  ESP_LOGD(TAG, "v_offset_=%d, i_offset_=%d, first=%d, last=%d", this->v_offset_, this->i_offset_, first, last);
}

void Emon::dump_config() { LOG_SENSOR("", "Emon", this); }

}  // namespace emon
}  // namespace esphome
