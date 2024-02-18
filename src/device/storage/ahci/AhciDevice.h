
#ifndef HHUOS_AHCIDEVICE_H
#define HHUOS_AHCIDEVICE_H

#include <cstdint>

#include "device/storage/StorageDevice.h"
#include "AhciController.h"

namespace Device::Storage {

class AhciDevice : public StorageDevice {

public:
    /**
     * Constructor.
     */
    AhciDevice(AhciController &controller, const AhciController::DeviceInfo &deviceInfo);

    /**
     * Copy Constructor.
     */
    AhciDevice(const AhciDevice &other) = delete;

    /**
     * Assignment operator.
     */
    AhciDevice &operator=(const AhciDevice &other) = delete;

    /**
     * Destructor.
     */
    ~AhciDevice() override = default;

    /**
     * Overriding function from StorageDevice.
     */
    uint32_t getSectorSize() override;

    /**
     * Overriding function from StorageDevice.
     */
    uint64_t getSectorCount() override;

    /**
     * Overriding function from StorageDevice.
     */
    uint32_t read(uint8_t *buffer, uint32_t startSector, uint32_t sectorCount) override;

    /**
     * Overriding function from StorageDevice.
     */
    uint32_t write(const uint8_t *buffer, uint32_t startSector, uint32_t sectorCount) override;

private:

    AhciController &controller;
    AhciController::DeviceInfo info;
};

}

#endif
