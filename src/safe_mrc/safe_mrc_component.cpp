/**************************************************
 * Copyright (c) 2025 Wenbo Li
 * University of Science and Technology of China
 *
 * This file is part of the SafeMRC project.
 * Distributed under the MIT License.
 **************************************************/

 #include "safe_mrc_device/safe_mrc/safe_mrc_component.hpp"

namespace safe_mrc {
MRCComponent::MRCComponent(RS485Serial& rs485_serial)
    : SafeMRCDeviceCollection(rs485_serial) {}

void MRCComponent::init_mrc_devices(std::vector<MRCType>mrc_types, std::vector<uint8_t> rs485_ids) {
    mrcs_.reserve(mrc_types.size());
    for (size_t i = 0; i < mrc_types.size(); i++) {
        mrcs_.emplace_back(mrc_types[i], rs485_ids[i]);
        auto mrc_device = std::make_shared<SafeMRCRS485Device>(mrcs_.back());
        get_device_collection().add_device(mrc_device);
    }
}

}
