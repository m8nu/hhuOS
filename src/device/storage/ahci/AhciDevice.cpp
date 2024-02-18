#include "AhciDevice.h"

#include "device/storage/ahci/AhciController.h"

namespace Device::Storage {

AhciDevice::AhciDevice(AhciController &controller, const AhciController::DeviceInfo &deviceInfo) : controller(controller), info(deviceInfo) {}

uint32_t AhciDevice::getSectorSize() {
    return info.sectorSize;
}

uint64_t AhciDevice::getSectorCount() {
    return info.maxSectorsLba28;
}

uint32_t AhciDevice::read(uint8_t *buffer, uint32_t startSector, uint32_t sectorCount) {
    return controller.read
}

uint32_t AhciDevice::write(const uint8_t *buffer, uint32_t startSector, uint32_t sectorCount) {
    return 
}

}