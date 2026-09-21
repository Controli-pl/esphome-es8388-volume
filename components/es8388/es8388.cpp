#include "es8388.h"

#include <algorithm>
#include <cmath>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome::es8388 {

static const char *const TAG = "es8388";

#define ES8388_ERROR_FAILED(func) \
  if (!(func)) { \
    this->mark_failed(); \
    return; \
  }

#define ES8388_ERROR_CHECK(func) \
  if (!(func)) { \
    return false; \
  }

namespace {

// Local volume-law calibration for the ES8388 ceiling speaker.
// 1.0 keeps the stock linear mapping. Lower values make the lower part of
// the Snapcast/HA slider louder. 0.55 is a good start for the 24% vs 70%
// mismatch reported for this installation.
constexpr float ES8388_VOLUME_CURVE = 0.55f;
constexpr uint8_t ES8388_DAC_VOLUME_MUTE = 192;
constexpr uint8_t ES8388_DAC_VOLUME_MAX = 0;

uint8_t volume_to_dac_attenuation(float volume) {
  volume = std::clamp(volume, 0.0f, 1.0f);
  if (volume <= 0.0f)
    return ES8388_DAC_VOLUME_MUTE;

  const float corrected = std::pow(volume, ES8388_VOLUME_CURVE);
  const float attenuation =
      static_cast<float>(ES8388_DAC_VOLUME_MUTE) * (1.0f - corrected);
  return static_cast<uint8_t>(std::lround(std::clamp(
      attenuation, static_cast<float>(ES8388_DAC_VOLUME_MAX),
      static_cast<float>(ES8388_DAC_VOLUME_MUTE))));
}

float dac_attenuation_to_volume(uint8_t attenuation) {
  const float corrected = 1.0f -
                          std::clamp(static_cast<float>(attenuation) /
                                         static_cast<float>(ES8388_DAC_VOLUME_MUTE),
                                     0.0f, 1.0f);
  if (corrected <= 0.0f)
    return 0.0f;
  return std::pow(corrected, 1.0f / ES8388_VOLUME_CURVE);
}

}  // namespace

void ES8388::setup() {
  this->set_mute_state_(true);

  ES8388_ERROR_FAILED(this->write_byte(ES8388_MASTERMODE, 0x00));
  ES8388_ERROR_FAILED(this->write_byte(ES8388_CONTROL2, 0x50));
  ES8388_ERROR_FAILED(this->write_byte(ES8388_CHIPPOWER, 0x00));
  ES8388_ERROR_FAILED(this->write_byte(ES8388_CONTROL1, 0x12));

  ES8388_ERROR_FAILED(this->write_byte(ES8388_DACCONTROL1, 0x18));
  ES8388_ERROR_FAILED(this->write_byte(ES8388_DACCONTROL2, 0x02));
  ES8388_ERROR_FAILED(this->write_byte(ES8388_DACCONTROL16, 0x00));
  ES8388_ERROR_FAILED(this->write_byte(ES8388_DACCONTROL17, 0x90));
  ES8388_ERROR_FAILED(this->write_byte(ES8388_DACCONTROL20, 0x90));
  ES8388_ERROR_FAILED(this->write_byte(ES8388_DACCONTROL21, 0x80));
  ES8388_ERROR_FAILED(this->write_byte(ES8388_DACCONTROL23, 0x00));

  ES8388_ERROR_FAILED(this->write_byte(ES8388_ADCPOWER, 0xFF));
  ES8388_ERROR_FAILED(this->write_byte(ES8388_ADCCONTROL1, 0x00));
  ES8388_ERROR_FAILED(this->write_byte(ES8388_ADCCONTROL3, 0x02));
  ES8388_ERROR_FAILED(this->write_byte(ES8388_ADCCONTROL4, 0x0D));
  ES8388_ERROR_FAILED(this->write_byte(ES8388_ADCCONTROL5, 0x02));
  ES8388_ERROR_FAILED(this->write_byte(ES8388_ADCCONTROL8, 0x00));
  ES8388_ERROR_FAILED(this->write_byte(ES8388_ADCCONTROL9, 0x00));
  ES8388_ERROR_FAILED(this->write_byte(ES8388_ADCCONTROL10, 0xE2));
  ES8388_ERROR_FAILED(this->write_byte(ES8388_ADCCONTROL11, 0xA0));
  ES8388_ERROR_FAILED(this->write_byte(ES8388_ADCCONTROL12, 0x12));
  ES8388_ERROR_FAILED(this->write_byte(ES8388_ADCCONTROL13, 0x06));
  ES8388_ERROR_FAILED(this->write_byte(ES8388_ADCCONTROL14, 0xC3));
  ES8388_ERROR_FAILED(this->write_byte(ES8388_DACCONTROL21, 0x80));

  ES8388_ERROR_FAILED(this->write_byte(ES8388_CHIPPOWER, 0xF0));
  delay(1);
  ES8388_ERROR_FAILED(this->write_byte(ES8388_CHIPPOWER, 0x00));

  this->set_mute_state_(false);
  ES8388_ERROR_FAILED(this->write_byte(ES8388_ADCCONTROL7, 0x60));
  ES8388_ERROR_FAILED(this->write_byte(ES8388_DACCONTROL3, 0x20));
  ES8388_ERROR_FAILED(this->write_byte(ES8388_ADCPOWER, 0x09));

#ifdef USE_SELECT
  if (this->dac_output_select_ != nullptr) {
    auto dac_power = this->get_dac_power();
    if (dac_power.has_value()) {
      if (this->dac_output_select_->has_index(dac_power.value())) {
        this->dac_output_select_->publish_state(dac_power.value());
      } else {
        ESP_LOGW(TAG, "Unknown DAC output power value: %d", dac_power.value());
      }
    }
  }

  if (this->adc_input_mic_select_ != nullptr) {
    auto mic_input = this->get_mic_input();
    if (mic_input.has_value()) {
      if (this->adc_input_mic_select_->has_index(mic_input.value())) {
        this->adc_input_mic_select_->publish_state(mic_input.value());
      } else {
        ESP_LOGW(TAG, "Unknown ADC input mic value: %d", mic_input.value());
      }
    }
  }
#endif
}

void ES8388::dump_config() {
  ESP_LOGCONFIG(TAG, "ES8388 Audio Codec:");
  LOG_I2C_DEVICE(this);
#ifdef USE_SELECT
  LOG_SELECT("  ", "DacOutputSelect", this->dac_output_select_);
  LOG_SELECT("  ", "ADCInputMicSelect", this->adc_input_mic_select_);
#endif
  if (this->is_failed()) {
    ESP_LOGCONFIG(TAG, "  Failed to initialize");
  }
}

bool ES8388::set_volume(float volume) {
  volume = clamp(volume, 0.0f, 1.0f);
  const uint8_t value = volume_to_dac_attenuation(volume);

  ESP_LOGD(TAG,
           "Setting ES8388 DAC volume: slider=%.3f curve=%.2f attenuation=0x%02X",
           volume, ES8388_VOLUME_CURVE, value);
  ES8388_ERROR_CHECK(this->write_byte(ES8388_DACCONTROL4, value));
  ES8388_ERROR_CHECK(this->write_byte(ES8388_DACCONTROL5, value));
  return true;
}

float ES8388::volume() {
  uint8_t value;
  if (!this->read_byte(ES8388_DACCONTROL4, &value))
    return 0.0f;
  return dac_attenuation_to_volume(value);
}

bool ES8388::set_mute_state_(bool mute_state) {
  uint8_t value = 0;
  this->is_muted_ = mute_state;

  ES8388_ERROR_CHECK(this->read_byte(ES8388_DACCONTROL3, &value));
  ESP_LOGV(TAG, "Read ES8388_DACCONTROL3: 0x%02X", value);

  if (mute_state) {
    value |= ES8388_DACCONTROL3_DAC_MUTE;
  } else {
    value &= ~ES8388_DACCONTROL3_DAC_MUTE;
  }

  ESP_LOGV(TAG, "Setting ES8388_DACCONTROL3 to 0x%02X (muted: %s)", value,
           YESNO(mute_state));
  return this->write_byte(ES8388_DACCONTROL3, value);
}

bool ES8388::set_dac_output(DacOutputLine line) {
  uint8_t reg_out1 = 0;
  uint8_t reg_out2 = 0;
  uint8_t dac_power = 0;

  switch (line) {
    case DAC_OUTPUT_LINE1:
      reg_out1 = 0x1E;
      dac_power = ES8388_DAC_OUTPUT_LOUT1_ROUT1;
      break;
    case DAC_OUTPUT_LINE2:
      reg_out2 = 0x1E;
      dac_power = ES8388_DAC_OUTPUT_LOUT2_ROUT2;
      break;
    case DAC_OUTPUT_BOTH:
      reg_out1 = 0x1E;
      reg_out2 = 0x1E;
      dac_power = ES8388_DAC_OUTPUT_BOTH;
      break;
    default:
      ESP_LOGE(TAG, "Unknown DAC output line: %d", line);
      return false;
  }

  ESP_LOGV(TAG,
           "DAC output config:\n"
           "  DACPOWER: 0x%02X\n"
           "  DACCONTROL24/25: 0x%02X\n"
           "  DACCONTROL26/27: 0x%02X",
           dac_power, reg_out1, reg_out2);

  ES8388_ERROR_CHECK(this->write_byte(ES8388_DACCONTROL24, reg_out1));
  ES8388_ERROR_CHECK(this->write_byte(ES8388_DACCONTROL25, reg_out1));
  ES8388_ERROR_CHECK(this->write_byte(ES8388_DACCONTROL26, reg_out2));
  ES8388_ERROR_CHECK(this->write_byte(ES8388_DACCONTROL27, reg_out2));
  return this->write_byte(ES8388_DACPOWER, dac_power);
}

optional<ES8388::DacOutputLine> ES8388::get_dac_power() {
  uint8_t dac_power;
  if (!this->read_byte(ES8388_DACPOWER, &dac_power)) {
    this->status_momentary_warning("dacpower_read");
    return {};
  }

  switch (dac_power) {
    case ES8388_DAC_OUTPUT_LOUT1_ROUT1:
      return DAC_OUTPUT_LINE1;
    case ES8388_DAC_OUTPUT_LOUT2_ROUT2:
      return DAC_OUTPUT_LINE2;
    case ES8388_DAC_OUTPUT_BOTH:
      return DAC_OUTPUT_BOTH;
    default:
      return {};
  }
}

bool ES8388::set_adc_input_mic(AdcInputMicLine line) {
  uint8_t mic_input = 0;

  switch (line) {
    case ADC_INPUT_MIC_LINE1:
      mic_input = ES8388_ADC_INPUT_LINPUT1_RINPUT1;
      break;
    case ADC_INPUT_MIC_LINE2:
      mic_input = ES8388_ADC_INPUT_LINPUT2_RINPUT2;
      break;
    case ADC_INPUT_MIC_DIFFERENCE:
      mic_input = ES8388_ADC_INPUT_DIFFERENCE;
      break;
    default:
      ESP_LOGE(TAG, "Unknown ADC input mic line: %d", line);
      return false;
  }

  ESP_LOGV(TAG, "Setting ES8388_ADCCONTROL2 to 0x%02X", mic_input);
  return this->write_byte(ES8388_ADCCONTROL2, mic_input);
}

optional<ES8388::AdcInputMicLine> ES8388::get_mic_input() {
  uint8_t mic_input;
  if (!this->read_byte(ES8388_ADCCONTROL2, &mic_input)) {
    this->status_momentary_warning("adccontrol2_read");
    return {};
  }

  switch (mic_input) {
    case ES8388_ADC_INPUT_LINPUT1_RINPUT1:
      return ADC_INPUT_MIC_LINE1;
    case ES8388_ADC_INPUT_LINPUT2_RINPUT2:
      return ADC_INPUT_MIC_LINE2;
    case ES8388_ADC_INPUT_DIFFERENCE:
      return ADC_INPUT_MIC_DIFFERENCE;
    default:
      return {};
  }
}

}  // namespace esphome::es8388
