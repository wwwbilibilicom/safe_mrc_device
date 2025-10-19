/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

#include "safe_mrc_device/safe_mrc/safeguarder.hpp"

#include "chrono"

namespace safe_mrc {

Safeguarder::Safeguarder(const std::string& port) : port_(port) {
  rs485_serial_ = std::make_unique<RS485Serial>(port_);
  master_rs485_device_collection_ =
      std::make_unique<RS485DeviceCollection>(*rs485_serial_);
  mrc_component_ = std::make_unique<MRCComponent>(*rs485_serial_);
}

void Safeguarder::init_mrcs(const std::vector<MRCType>& mrc_types,
                            const std::vector<uint8_t>& rs485_ids) {
  mrc_component_->init_mrc_devices(mrc_types, rs485_ids);
  register_mrc_device_collection(*mrc_component_);
  master_rs485_device_collection_->enable_rx_thread();
}

void Safeguarder::register_mrc_device_collection(
    SafeMRCDeviceCollection& device_collection) {
  for (const auto& [id, device] :
       device_collection.get_device_collection().get_devices()) {
    master_rs485_device_collection_->add_device(device);
  }
  sub_mrc_device_collections_.push_back(&device_collection);
}

void Safeguarder::enable_all() {
  for (SafeMRCDeviceCollection* device_collection :
       sub_mrc_device_collections_) {
    device_collection->enable_all();
  }
}

void Safeguarder::disable_all() {
  for (SafeMRCDeviceCollection* device_collection :
       sub_mrc_device_collections_) {
    device_collection->disable_all();
  }
}

void Safeguarder::set_zero_all() {
  for (SafeMRCDeviceCollection* device_collection :
       sub_mrc_device_collections_) {
    device_collection->set_zero_all();
  }
}

void Safeguarder::refresh_all() {
  for (SafeMRCDeviceCollection* device_collection :
       sub_mrc_device_collections_) {
    device_collection->refresh_all();
  }
}

void Safeguarder::set_rx_timeout_us(int timeout_us) {
  rs485_serial_->setTimeout(timeout_us);
}

}  // namespace safe_mrc
