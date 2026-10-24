#include <cstdlib>

#include "selfdrive/pandad/pandad.h"
#include "openpilot/cereal/messaging/messaging.h"
#include "common/swaglog.h"

void PandaSafety::configureSafetyMode(bool is_onroad) {
  // --- EPS parked-bench mode (isolated, reversible) ---------------------------
  // When the bench launch sets EPS_BENCH, hold SAFETY_ALLOUTPUT (param 0) instead
  // of the car's safety model. ALLOUTPUT: controls_allowed=true (steer TX allowed
  // at standstill), relay OPEN/intercept (default branch of set_safety_mode -> the
  // camera's bus-2 0x0E4 is isolated from the EPS), and param 0 => disable_forwarding
  // (the camera's 0x0E4 is not bridged to bus 0 either). Bus-0 car traffic (speed
  // 0x158, sensors, 0x18F) stays native. No CarParams / ControlsReady dependency,
  // so controlsd/card need not run. See eps_bench/README.md.
  static const bool eps_bench = (std::getenv("EPS_BENCH") != nullptr);
  if (eps_bench) {
    if (is_onroad && !safety_configured_) {
      LOGW("EPS_BENCH: setting SAFETY_ALLOUTPUT (param 0) - PARKED STEER BENCH ONLY");
      panda_->set_safety_model(cereal::CarParams::SafetyModel::ALL_OUTPUT, 0U);
      safety_configured_ = true;
    } else if (!is_onroad) {
      safety_configured_ = false;
    }
    return;
  }

  if (is_onroad && !safety_configured_) {
    updateMultiplexingMode();

    auto car_params = fetchCarParams();
    if (!car_params.empty()) {
      LOGW("got %lu bytes CarParams", car_params.size());
      setSafetyMode(car_params);
      safety_configured_ = true;
    }
  } else if (!is_onroad) {
    initialized_ = false;
    safety_configured_ = false;
    log_once_ = false;
  }
}

void PandaSafety::updateMultiplexingMode() {
  // Initialize to ELM327 without OBD multiplexing for initial fingerprinting
  if (!initialized_) {
    prev_obd_multiplexing_ = false;
    panda_->set_safety_model(cereal::CarParams::SafetyModel::ELM327, 1U);
    initialized_ = true;
  }

  // Switch between multiplexing modes based on the OBD multiplexing request
  bool obd_multiplexing_requested = params_.getBool("ObdMultiplexingEnabled");
  if (obd_multiplexing_requested != prev_obd_multiplexing_) {
    const uint16_t safety_param = obd_multiplexing_requested ? 0U : 1U;
    panda_->set_safety_model(cereal::CarParams::SafetyModel::ELM327, safety_param);
    prev_obd_multiplexing_ = obd_multiplexing_requested;
    params_.putBool("ObdMultiplexingChanged", true);
  }
}

std::string PandaSafety::fetchCarParams() {
  if (!params_.getBool("FirmwareQueryDone")) {
    return {};
  }

  if (!log_once_) {
    LOGW("Finished FW query, Waiting for params to set safety model");
    log_once_ = true;
  }

  if (!params_.getBool("ControlsReady")) {
    return {};
  }
  return params_.get("CarParams");
}

void PandaSafety::setSafetyMode(const std::string &params_string) {
  AlignedBuffer aligned_buf;
  capnp::FlatArrayMessageReader cmsg(aligned_buf.align(params_string.data(), params_string.size()));
  cereal::CarParams::Reader car_params = cmsg.getRoot<cereal::CarParams>();

  auto safety_configs = car_params.getSafetyConfigs();
  uint16_t alternative_experience = car_params.getAlternativeExperience();

  cereal::CarParams::SafetyModel safety_model = safety_configs[0].getSafetyModel();
  uint16_t safety_param = safety_configs[0].getSafetyParam();

  LOGW("setting safety model: %d, param: %d, alternative experience: %d", (int)safety_model, safety_param, alternative_experience);
  panda_->set_alternative_experience(alternative_experience);
  panda_->set_safety_model(safety_model, safety_param);
}
