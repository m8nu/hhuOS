#include "AhciController.h"

#include "lib/util/async/Thread.h"
#include "device/storage/ChsConverter.h"
#include "kernel/system/System.h"
#include "kernel/service/StorageService.h"
#include "kernel/service/InterruptService.h"
#include "device/pci/Pci.h"
#include "device/pci/PciDevice.h"
#include "kernel/log/Logger.h"
#include "kernel/process/ThreadState.h"
#include "kernel/service/MemoryService.h"
#include "lib/util/base/Exception.h"
#include "lib/util/collection/Array.h"
#include "lib/util/base/Address.h"
#include "lib/util/base/Constants.h"
#include "lib/util/base/String.h"
#include "lib/util/time/Timestamp.h"
#include "device/interrupt/InterruptRequest.h"
#include "kernel/interrupt/InterruptVector.h"
#include "lib/util/collection/ArrayList.h"
#include "lib/util/collection/Iterator.h"
#include <chrono>

namespace Kernel {
    class Logger;
}

namespace Device::Storage {
    Kernel::Logger AhciController::log = Kernel::Logger::get("AHCI");
    HBA_MEM *hbaMem;
    virtual_port_addr vpa[32]; //Virtual port addresses of each port
    uint32_t lba_capacity[32]; //LBA capacity of each port
    uint32_t sector_size[32]; //sector size of each port in bytes (512 or 4096)

    char* strip_and_swap(char *str, int len){
        char *str2 = new char[len+1];
        int k;
        for (k=0; k<len; k+=2){
            str2[k] = str[k + 1];
            str2[k+1] = str[k];
        };
        char *check = &str2[len-1];
        str2[len] = 0;
        while (*check == ' '){
            if (check == str2) break;
            *check-- = 0;
        };
        return str2;
    }

    HBA_MEM *MapAHCIRegisters(uint32_t baseAddress) {
        auto &memoryService = Kernel::System::getService<Kernel::MemoryService>();
        void *mappedAddress = memoryService.mapIO(baseAddress, (uint32_t)4096);

        if (mappedAddress == nullptr) {
            AhciController::log.error("Failed to map AHCI registers");
            return nullptr;
        }
        return reinterpret_cast<HBA_MEM *>(mappedAddress);
    }

    AhciController::AhciController(const PciDevice &device) {
        auto &memoryService = Kernel::System::getService<Kernel::MemoryService>();

        //enable bus mastering
        uint16_t command = device.readWord(Pci::COMMAND);
        command |= Pci::BUS_MASTER | Pci::IO_SPACE;
        device.writeWord(Pci::COMMAND, command);

        //ABAR auslesen und mappen
        uint32_t abar = device.readDoubleWord(Pci::BASE_ADDRESS_5);
        hbaMem = MapAHCIRegisters(abar);
        log.info("Initializing controller [0x%x:0x%x]", device.getVendorId(), device.getDeviceId());

        //Ownership übernehmen (AHCI Version >= 1.2)
        biosHandoff(device);

        //AHCI Modus aktivieren
        if(enableAHCIController(device) == -1){
            log.error("AHCI Controller could not be enabled");
            return;
        } else {
            log.info("AHCI Controller [0x%x] runs in AHCI-Mode", device.getDeviceId()); 
        }

        //CAP.NP auslesen um Anzahl der Ports zu ermitteln, die vom HBA unterstützt werden
        uint8_t numPortsAllowed = (hbaMem->cap & 0x1F) + 1;

        for (uint8_t i = 0; i < 1; i++) { //i < AHCI_MAX_PORTS
            if(hbaMem->pi & (1 << i)){

                //Speicher für Command List und FIS reservieren
                port_rebase(&hbaMem->ports[i], i);

                //Clear errors
                hbaMem->ports[i].serr = 0xFFFFFFFF;

                //Clear interrupt status
                hbaMem->ports[i].is = 0xFFFFFFFF;

                //Enable interrupts
                hbaMem->ports[i].ie = 0xFFFFFFFF;

                //Detect what device is connected to the port
                int dt = check_type(&hbaMem->ports[i]);
			    if (dt == AHCI_DEV_SATA){
                    identifyDevice(&hbaMem->ports[i], i);

                    test_read_write(i, 2048, 5);
                    test_read_write(i, 10240, 5);
                    test_read_write(i, 20480, 5);

                }else if (dt == AHCI_DEV_SATAPI){
                    log.info("SATAPI drive found at port %d", i);
			    }else if (dt == AHCI_DEV_SEMB){
			    	log.info("SEMB drive found at port %d", i);
			    }else if (dt == AHCI_DEV_PM){
			    	log.info("PM drive found at port %d", i);
			    }else{
			    	log.info("No drive found at port %d", i);
			    }
            }
		}
    }

    //FYSOS Kapitel Identy Device
    int AhciController::identifyDevice(HBA_PORT *port, uint8_t portno) {
        auto &memoryService = Kernel::System::getService<Kernel::MemoryService>();

        port->is = (uint32_t) -1;		// Clear pending interrupt bits

        //get unused command slot
        uint32_t slot = find_cmdslot(port);

        HBA_CMD_HEADER *cmdheader = (HBA_CMD_HEADER*) vpa[portno].cmdList;
        cmdheader += slot;
        cmdheader->prdtl = 1;	// PRDT entries count
        cmdheader->pmp = 0;		// Port multiplier value
	    cmdheader->a = 0;   	// ATAPI
	    cmdheader->w = 0;		// Write, 1: H2D, 0: D2H
	    cmdheader->p = 0;		// Prefetchable 0
        cmdheader->r = 0;		// Reset
	    cmdheader->b = 0;	    // BIST
	    cmdheader->c = 0;	    // Clear busy upon R_OK 0
        cmdheader->cfl = 5;    // Command FIS length | sizeof(FIS_REG_H2D) / 4;

        HBA_CMD_TBL *cmdtbl = (HBA_CMD_TBL*) vpa[portno].commandTable[slot];
        //FIS
        FIS_REG_H2D *cmdfis = (FIS_REG_H2D*) cmdtbl->cfis;
        cmdfis->fis_type = FIS_TYPE_REG_H2D; //0x27
        cmdfis->c = 1; //Command
        cmdfis->command = ATA_IDENTIFY_DEVICE; //0xEC
        cmdfis->featurel = 0x00;
        cmdfis->featureh = 0x00;
        cmdfis->lba0     = 0x00;
        cmdfis->lba1     = 0x00;
        cmdfis->lba2     = 0x00;
        cmdfis->device   = 0xA0; //(1 << 6);
        cmdfis->lba3     = 0x00;
        cmdfis->lba4     = 0x00;
        cmdfis->lba5     = 0x00;
        cmdfis->control  = 0x08;
        cmdfis->rsv0     = 0x00;

        auto dba = reinterpret_cast<uint32_t*>(memoryService.mapIO(512));
        auto dbaphy = reinterpret_cast<uint32_t>(memoryService.getPhysicalAddress(dba));
        cmdtbl->prdt_entry[0].dba = dbaphy;
        cmdtbl->prdt_entry[0].dbc = 0x1FF; //512 bytes

        //Ensure that device is not busy
        while (port->tfd & (ATA_DEV_BUSY | ATA_DEV_DRQ)); 
                                                                        
        //Issue command
        port->ci = 1 << slot;

        //Wait for command completion
        while (1) {
            if ((port->ci & (1 << slot)) == 0) 
                break;
            if (port->is & HBA_PxIS_TFES) { //Task file error
                log.error("Read disk error in Identify Device\n");
                return false;
            }
        }

        SATA_ident_t *SATA_Identify_info =(SATA_ident_t*)(dba);

        auto mdl = strip_and_swap((char*) SATA_Identify_info->model, 40);
        auto serial = strip_and_swap((char*) SATA_Identify_info->serial_no, 20);
        auto fw = strip_and_swap((char*) SATA_Identify_info->fw_rev, 8);

        lba_capacity[portno] = SATA_Identify_info->lba_capacity;
        sector_size[portno] = SATA_Identify_info->sector_bytes;

        log.info("Found SATA drive on port [%d]: %s %s (Firmware: [%s])", portno, mdl, serial, fw);
        return 0;
    }

    bool AhciController::read(HBA_PORT *port,int portno, uint32_t startl, uint32_t starth, uint32_t count, void* buffer){
        auto &memoryService = Kernel::System::getService<Kernel::MemoryService>();  

        // Clear pending interrupt bits
        port->is = (uint32_t) -1;
        int spin = 0;
        int slot = find_cmdslot(port);
        if (slot == -1)
            return false;

        HBA_CMD_HEADER *cmdheader = (HBA_CMD_HEADER*) vpa[portno].cmdList;
        cmdheader += slot;
        cmdheader->prdtl = (uint16_t)((count-1)/16) + 1;	// PRDT entries count
        cmdheader->pmp = 0;		// Port multiplier value
	    cmdheader->a = 0;   	// ATAPI
	    cmdheader->w = 0;		// Write
	    cmdheader->p = 0;		// Prefetchable 0
        cmdheader->r = 0;		// Reset
	    cmdheader->b = 0;	    // BIST
	    cmdheader->c = 0;	    // Clear busy upon R_OK 0
        cmdheader->cfl = sizeof(FIS_REG_H2D) / 4;    // Command FIS length | sizeof(FIS_REG_H2D) / 4;

        HBA_CMD_TBL *cmdtbl = (HBA_CMD_TBL*) vpa[portno].commandTable[slot];

        auto bufphy = memoryService.getPhysicalAddress(buffer);
        // 8K bytes (16 sectors) per PRDT
        int i = 0;
        for (i = 0; i < cmdheader->prdtl - 1; i++){
            cmdtbl->prdt_entry[i].dba = (uint32_t)bufphy;
            cmdtbl->prdt_entry[i].dbc = 8191; //zero-based byte count
            bufphy = bufphy + 4*1024; // 4K bytes
            count -= 16;
        }

        cmdtbl->prdt_entry[i].dba = (uint32_t)bufphy;
        cmdtbl->prdt_entry[i].dbc = (count * 512) - 1; // 512 bytes per sector

        FIS_REG_H2D *cmdfis = (FIS_REG_H2D*) cmdtbl->cfis;
        cmdfis->fis_type = FIS_TYPE_REG_H2D; //0x27
        cmdfis->c = 1; //Command
        cmdfis->command = ATA_CMD_READ_DMA_EX; //0x25;
        cmdfis->featurel = 0x00;
        cmdfis->featureh = 0x00;
        cmdfis->device   = 0xA0;	// LBA mode
        cmdfis->lba0     = (uint8_t)startl;
        cmdfis->lba1     = (uint8_t)(startl >> 8);
        cmdfis->lba2     = (uint8_t)(startl >> 16);
        cmdfis->lba3     = (uint8_t)(startl >> 24);
        cmdfis->lba4     = (uint8_t)starth;
        cmdfis->lba5     = (uint8_t)(starth >> 8);
        cmdfis->control  = 0x08;

        cmdfis->countl = count & 0xFF;
        cmdfis->counth = (count >> 8) & 0xFF;
        cmdfis->rsv0     = 0x00;

        // The below loop waits until the port is no longer busy before issuing the command
        while ((port->tfd & (ATA_DEV_BUSY | ATA_DEV_DRQ)) && spin < 1000000) {
            spin++;
        }

        if (spin == 1000000) {
            log.error("Port is hung");
            return false;
        }

        port->ci = 1 << slot;	// Issue command

        // Wait for completion
        while (1) {
            // In some longer duration reads, it may be helpful to spin on the DPS bit
            // in the PxIS port field as well (1 << 5)
            if ((port->ci & (1 << slot)) == 0) 
                break;
            if (port->is & HBA_PxIS_TFES) {	// Task file error
                log.error("Read disk error");
                return false;
            }
        }

        // Check again
        if (port->is & HBA_PxIS_TFES) {
            log.error("Read disk error");
            return false;
        }

        return true;
    }

    bool AhciController::readOneSector(HBA_PORT *port, int portno, uint32_t startl, uint32_t starth){
        auto &memoryService = Kernel::System::getService<Kernel::MemoryService>();
        port->is = (uint32_t) -1;		// Clear pending interrupt bits

        //get unused command slot
        uint32_t slot = find_cmdslot(port);

        HBA_CMD_HEADER *cmdheader = (HBA_CMD_HEADER*) vpa[portno].cmdList;
        cmdheader += slot;
        cmdheader->prdtl = 1;	// PRDT entries count
        cmdheader->pmp = 0;		// Port multiplier value
	    cmdheader->a = 0;   	// ATAPI
	    cmdheader->w = 0;		// Write, 1: H2D, 0: D2H
	    cmdheader->p = 0;		// Prefetchable 0
        cmdheader->r = 0;		// Reset
	    cmdheader->b = 0;	    // BIST
	    cmdheader->c = 0;	    // Clear busy upon R_OK 0
        cmdheader->cfl = 5;    // Command FIS length | sizeof(FIS_REG_H2D) / 4;

        HBA_CMD_TBL *cmdtbl = (HBA_CMD_TBL*) vpa[portno].commandTable[slot];

        //FIS
        FIS_REG_H2D *cmdfis = (FIS_REG_H2D*) cmdtbl->cfis;
        cmdfis->fis_type = FIS_TYPE_REG_H2D; //0x27
        cmdfis->c = 1; //Command
        cmdfis->command = ATA_CMD_READ_DMA_EX; //0xEC
        cmdfis->countl   = 0x01;
        cmdfis->counth   = 0x00;
        cmdfis->featurel = 0x00;
        cmdfis->featureh = 0x00;
        cmdfis->device   = 0xA0;
        cmdfis->lba0     = (uint8_t)startl;
        cmdfis->lba1     = (uint8_t)(startl >> 8);
        cmdfis->lba2     = (uint8_t)(startl >> 16);
        cmdfis->lba3     = (uint8_t)(startl >> 24);
        cmdfis->lba4     = (uint8_t)starth;
        cmdfis->lba5     = (uint8_t)(starth >> 8);
        cmdfis->control  = 0x08;
        cmdfis->rsv0     = 0x00;

        auto dba = reinterpret_cast<uint32_t*>(memoryService.mapIO(512));
        auto dbaphy = reinterpret_cast<uint32_t>(memoryService.getPhysicalAddress(dba));
        cmdtbl->prdt_entry[0].dba = dbaphy;
        cmdtbl->prdt_entry[0].dbc = 0x1FF; //511 bytes

        //Ensure that device is not busy
        while (port->tfd & (ATA_DEV_BUSY | ATA_DEV_DRQ)); 
                                                                        
        //Issue command
        port->ci = 1 << slot;

        //Wait for command completion
        while (1) {
            if ((port->ci & (1 << slot)) == 0) 
                break;
            if (port->is & HBA_PxIS_TFES) { //Task file error
                log.error("Read disk error\n");
                return false;
            }
        }

        //print the buffer
        for(int i = 0; i < 512 / 4; i++){
            log.info("buffer[%d]: %x", i, dba[i]);
        }
        return true;
    }

    bool AhciController::writeOneSector(HBA_PORT *port, int portno, uint32_t startl, uint32_t starth){
        auto &memoryService = Kernel::System::getService<Kernel::MemoryService>();
        port->is = (uint32_t) -1;		// Clear pending interrupt bits

        //get unused command slot
        uint32_t slot = find_cmdslot(port);

        HBA_CMD_HEADER *cmdheader = (HBA_CMD_HEADER*) vpa[0].cmdList;
        cmdheader += slot;
        cmdheader->prdtl = 1;	// PRDT entries count
        cmdheader->pmp = 0;		// Port multiplier value
        cmdheader->a = 0;   	// ATAPI
        cmdheader->w = 1;		// Write, 1: H2D, 0: D2H
        cmdheader->p = 0;		// Prefetchable 0
        cmdheader->r = 0;		// Reset
        cmdheader->b = 0;	    // BIST
        cmdheader->c = 0;	    // Clear busy upon R_OK 0
        cmdheader->cfl = 5;    // Command FIS length | sizeof(FIS_REG_H2D) / 4;

        HBA_CMD_TBL *cmdtbl = (HBA_CMD_TBL*) vpa[0].commandTable[slot];

        //FIS
        FIS_REG_H2D *cmdfis = (FIS_REG_H2D*) cmdtbl->cfis;
        cmdfis->fis_type = FIS_TYPE_REG_H2D; //0x27
        cmdfis->c = 1; //Command
        cmdfis->command = ATA_CMD_WRITE_DMA_EX; //0xEC
        cmdfis->countl   = 0x01;
        cmdfis->counth   = 0x00;
        cmdfis->featurel = 0x00;
        cmdfis->featureh = 0x00;
        cmdfis->device   = 0xA0;
        cmdfis->lba0     = (uint8_t)startl;
        cmdfis->lba1     = (uint8_t)(startl >> 8);
        cmdfis->lba2     = (uint8_t)(startl >> 16);
        cmdfis->lba3     = (uint8_t)(startl >> 24);
        cmdfis->lba4     = (uint8_t)starth;
        cmdfis->lba5     = (uint8_t)(starth >> 8);
        cmdfis->rsv0     = 0x00;

        auto dba = reinterpret_cast<uint32_t*>(memoryService.mapIO(512));

        for(int i = 0; i < 512 / 4; i++){
            dba[i] = 0x12345678;
        }

        auto dbaphy = reinterpret_cast<uint32_t>(memoryService.getPhysicalAddress(dba));
        cmdtbl->prdt_entry[0].dba = dbaphy;
        cmdtbl->prdt_entry[0].dbc = 0x1FF; //512 bytes

        //Ensure that device is not busy
        while (port->tfd & (ATA_DEV_BUSY | ATA_DEV_DRQ));

        //Issue command
        port->ci = 1 << slot;

        //Wait for command completion
        while (1) {
            if ((port->ci & (1 << slot)) == 0) 
                break;
            if (port->is & HBA_PxIS_TFES) { //Task file error
                log.error("Write disk error\n");
                return false;
            }
        }
        return true;
    }

    bool AhciController::write(HBA_PORT *port,int portno, uint32_t startl, uint32_t starth, uint32_t count, void* buffer){
        auto &memoryService = Kernel::System::getService<Kernel::MemoryService>();

        hbaMem->is = (uint32_t) -1;		// Clear pending interrupt bits
        int slot = find_cmdslot(port);
        if (slot == -1)
            return false;

        HBA_CMD_HEADER *cmdheader = (HBA_CMD_HEADER*) vpa[portno].cmdList;
        cmdheader += slot;
        cmdheader->prdtl = (uint16_t)((count-1)/16) + 1;	// PRDT entries count
        cmdheader->pmp = 0;		// Port multiplier value
        cmdheader->a = 0;   	// ATAPI
        cmdheader->w = 1;		// Write
	    cmdheader->p = 0;		// Prefetchable 0
        cmdheader->r = 0;		// Reset
	    cmdheader->b = 0;	    // BIST
	    cmdheader->c = 0;	    // Clear busy upon R_OK 0
        cmdheader->cfl = sizeof(FIS_REG_H2D) / 4;    // Command FIS length | sizeof(FIS_REG_H2D) / 4;

        HBA_CMD_TBL *cmdtbl = (HBA_CMD_TBL*) vpa[portno].commandTable[slot];

        auto bufphy = memoryService.getPhysicalAddress(buffer);
        int i = 0;
        for (i = 0; i < cmdheader->prdtl - 1; i++){
            cmdtbl->prdt_entry[i].dba = (uint32_t)bufphy;
            cmdtbl->prdt_entry[i].dbc = 8191;
            bufphy = (uint8_t*)bufphy + 4*1024;
            count -= 16;
        }

        cmdtbl->prdt_entry[i].dba = (uint32_t)bufphy;
        cmdtbl->prdt_entry[i].dbc = (count * 512) - 1; // 512 bytes per sector

        FIS_REG_H2D *cmdfis = (FIS_REG_H2D*) cmdtbl->cfis;
        cmdfis->fis_type = FIS_TYPE_REG_H2D;
        cmdfis->command = ATA_CMD_WRITE_DMA_EX; //0x35
        cmdfis->c = 1;	// Command
        cmdfis->featurel = 0x00;
        cmdfis->featureh = 0x00;
        cmdfis->device   = 0xA0;	// LBA mode
        cmdfis->lba0     = (uint8_t)startl;
        cmdfis->lba1     = (uint8_t)(startl >> 8);
        cmdfis->lba2     = (uint8_t)(startl >> 16);
        cmdfis->lba3     = (uint8_t)(startl >> 24);
        cmdfis->lba4     = (uint8_t)starth;
        cmdfis->lba5     = (uint8_t)(starth >> 8);
        cmdfis->control  = 0x08;

        cmdfis->countl = count & 0xFF;
        cmdfis->counth = (count >> 8) & 0xFF;
        cmdfis->rsv0     = 0x00;

        // The below loop waits until the port is no longer busy before issuing the command
        while ((port->tfd & (ATA_DEV_BUSY | ATA_DEV_DRQ)));

        port->ci = 1 << slot;	// Issue command

        // Wait for completion
        while (1) {
            // In some longer duration reads, it may be helpful to spin on the DPS bit
            // in the PxIS port field as well (1 << 5)
            if ((port->ci & (1 << slot)) == 0) 
                break;
            if (port->is & HBA_PxIS_TFES) {	// Task file error
                log.error("Write disk error");
                return false;
            }
        }

        // Check again
        if (port->is & HBA_PxIS_TFES) {
            log.error("Write disk error");
            return false;
        }

        return true;
    }

    void AhciController::initializeAvailableControllers() {
        Util::Array<PciDevice> devices = Pci::search(Pci::Class::MASS_STORAGE, PCI_SUBCLASS_AHCI);
        for (const auto &device: devices) {
            AhciController *controller = new AhciController(device);
            controller->plugin();
        }
    }

    uint32_t AhciController::find_cmdslot(HBA_PORT *port) {
        //get count of command slots
        uint32_t numCmdSlots = ((hbaMem->cap >> 8) & 0x1F) + 1;
        uint32_t slots = (port->sact | port->ci);

        for (uint32_t i = 0; i < numCmdSlots; i++) {
            if ((slots & 1) == 0)
                return i;
            slots >>= 1;
        }
        log.error("Cannot find free command list entry");
        return -1;
    }

    //FYSOS  
    int AhciController::biosHandoff(const Device::PciDevice &device) {
        if (hbaMem->vs >= 0x10200) {
            if (hbaMem->cap2 & 0x1) {
                hbaMem->bohc = hbaMem->bohc | (1 << 1);
                Util::Async::Thread::sleep(Util::Time::Timestamp::ofMilliseconds(25));

                if (hbaMem->bohc & (1 << 4)) {
                    Util::Async::Thread::sleep(Util::Time::Timestamp::ofMilliseconds(2000));
                }

                if ((hbaMem->bohc & ((1 << 4) | (1 << 1) | (1 << 0))) != (1 << 1)) {
                    log.info("BIOS/OS Handoff failed for device: [0x%x]", device.getDeviceId());
                    return -1;
                }

                //Clear the OOC bit
                hbaMem->bohc = hbaMem->bohc & ~(1 << 3);

            } else {
                log.info("BIOS/OS Handoff not supported for device: [0x%x]", device.getDeviceId());
                return 0;
            }
        } else{
            log.info("AHCI Version < 1.2 for device: [0x%x]. skip BIOS/OS Handoff", device.getDeviceId());
            return 0;
        }
        return 0;
    }
    
    //FYSOS
    int AhciController::enableAHCIController(const Device::PciDevice &device) {
        hbaMem->ghc |= (1 << 31);
        Util::Async::Thread::sleep(Util::Time::Timestamp::ofMilliseconds(25));
        if (!(hbaMem->ghc & (1 << 31))) {
            log.error("AHCI not enabled for device: %x", device.getDeviceId());
            hbaMem->ghc |= (1 << 31);
            Util::Async::Thread::sleep(Util::Time::Timestamp::ofMilliseconds(25));
            if (!(hbaMem->ghc & (1 << 31))) {
                log.error("AHCI not enabled for device: %x (2nd try)", device.getDeviceId());
                return -1;
            }
        }
        return 0;
    }

    int AhciController::check_type(HBA_PORT *port) {
	    uint32_t ssts = port->ssts;
    
	    uint8_t ipm = (ssts >> 8) & 0x0F;
	    uint8_t det = ssts & 0x0F;
    
        // Check drive status
	    if (det != HBA_PORT_DET_PRESENT)
	    	return AHCI_DEV_NULL;
	    if (ipm != HBA_PORT_IPM_ACTIVE)
	    	return AHCI_DEV_NULL;
    
	    switch (port->sig){
	        case SATA_SIG_ATAPI:
	        	return AHCI_DEV_SATAPI;
	        case SATA_SIG_SEMB:
                return AHCI_DEV_SEMB;
	        case SATA_SIG_PM:
	        	return AHCI_DEV_PM;
	        default:
	        	return AHCI_DEV_SATA;
	    }
    }

    int AhciController::hbaReset(){
        int timeout = 0;

        //set GHC.HR to 1
        hbaMem->ghc |= (1 << 0);

        //wait until GHC.HR is cleared
        while(hbaMem->ghc & (1 << 0) && timeout < 1000){
            Util::Async::Thread::sleep(Util::Time::Timestamp::ofMilliseconds(1));
            timeout++;
        }

        if(timeout >= 1000){
            log.error("HBA Reset failed");
            return -1;
        }
        log.info("HBA Reset done");
        return 0;
    }

    void AhciController::portReset(HBA_PORT *port) {

        //write 1 PxSCTL.DET to 1
        port->sctl |= (1 << 0);
        Util::Async::Thread::sleep(Util::Time::Timestamp::ofMilliseconds(5));
        //clear PxSCTL.DET
        port->sctl  &= ~(1 << 0);

        //wait until PxSSTS.DET is 3
        while((port->ssts & 0x0F) != HBA_PORT_DET_PRESENT){
            Util::Async::Thread::sleep(Util::Time::Timestamp::ofMilliseconds(5));
        }

        port->serr = 0xFFFFFFFF;
    }

    void AhciController::start_cmd(HBA_PORT *port) {
        // Wait until CR (bit15) is cleared
        while (port->cmd & HBA_PxCMD_CR);

        // Set FRE (bit4) and ST (bit0)
        port->cmd |= HBA_PxCMD_FRE;
        port->cmd |= HBA_PxCMD_ST;
    }

    void AhciController::stop_cmd(HBA_PORT *port) {
        // Clear ST (bit0)
        port->cmd &= ~HBA_PxCMD_ST;

        // Clear FRE (bit4)
        port->cmd &= ~HBA_PxCMD_FRE;

        // Wait until FR (bit14), CR (bit15) are cleared
        while(1) {
            if (port->cmd & HBA_PxCMD_FR)
                continue;
            if (port->cmd & HBA_PxCMD_CR)
                continue;
            break;
        }
    }
    
    //OSDEV
    void AhciController::port_rebase(HBA_PORT *port, int portno){
        auto &memoryService = Kernel::System::getService<Kernel::MemoryService>(); 

        AhciController::stop_cmd(port); //Stop command engine

        //Command List
        auto cmdList = reinterpret_cast<uint32_t*>(memoryService.mapIO(sizeof(HBA_CMD_HEADER) * 32));
        vpa[portno].cmdList = cmdList; 
        auto cmdListPhysicalAdress = reinterpret_cast<uint32_t>(memoryService.getPhysicalAddress(cmdList));
        port->clb = cmdListPhysicalAdress;
        port->clbu = 0;

        //FIS
        auto fis = reinterpret_cast<uint32_t*>(memoryService.mapIO(256));
        vpa[portno].fis = fis;
        auto fisPhysicalAdress = reinterpret_cast<uint32_t>(memoryService.getPhysicalAddress(fis));
        port->fb = fisPhysicalAdress;
        port->fbu = 0;

        //Command Table
        HBA_CMD_HEADER *cmdheader = (HBA_CMD_HEADER*) vpa[portno].cmdList;
        for(int i=0;i<32;i++){
            cmdheader[i].prdtl = 8;	// 8 prdt entries per command table // 256 bytes per command table, 64+16+48+16*8
            // Command table offset: 40K + 8K*portno + cmdheader_index*256
            auto cmdtbl = reinterpret_cast<uint32_t*>(memoryService.mapIO(256));
            vpa[portno].commandTable[i] = cmdtbl;
            auto cmdtblPhysicalAdress = reinterpret_cast<uint32_t>(memoryService.getPhysicalAddress(cmdtbl));
            cmdheader[i].ctba = cmdtblPhysicalAdress;
            cmdheader[i].ctbau = 0;
        }
        AhciController::start_cmd(port);
    }

    void AhciController::plugin() {
        auto &interruptService = Kernel::System::getService<Kernel::InterruptService>();
        interruptService.assignInterrupt(Kernel::InterruptVector::AHCI, *this);
        interruptService.allowHardwareInterrupt(Device::InterruptRequest::AHCI);
    }

    void AhciController::trigger(const Kernel::InterruptFrame &frame) {
        log.info("AHCI Interrupt triggered");
    }

    void AhciController::test_read_write(int portno, uint64_t sector, int repeat) {
        int readtimes[repeat];
        int writetimes[repeat];
        auto &memoryService = Kernel::System::getService<Kernel::MemoryService>();

        auto buffer_write = reinterpret_cast<uint32_t*>(memoryService.mapIO(512 * sector));
        auto buffer_read = reinterpret_cast<uint32_t*>(memoryService.mapIO(512 * sector));
        for(uint64_t i = 0; i < (512 * sector) / 4; i++){
            buffer_write[i] = 0x87654321;
        }
        for(int i = 0; i < repeat; i++){
            int startl = 50;
            uint64_t sec1 = sector;
            uint64_t sec2 = sector;
        
            Util::Time::Timestamp start_write = Util::Time::getSystemTime();
            while(sec1 > 0){
                write(&hbaMem->ports[portno], portno, startl, 0, 128, buffer_write);
                buffer_write += 128;
                startl += 128;
                sec1 -= 128;
            }
            Util::Time::Timestamp end_write = Util::Time::getSystemTime();
        

            startl = 50;
            Util::Time::Timestamp start_read = Util::Time::getSystemTime();
            while(sec2 > 0){
                read(&hbaMem->ports[portno], portno, startl, 0, 128, buffer_read);
                buffer_read += 128;
                startl += 128;
                sec2 -= 128;
            };
            Util::Time::Timestamp end_read = Util::Time::getSystemTime();

            uint32_t sw = start_write.toMilliseconds();
            uint32_t ew = end_write.toMilliseconds();
            uint32_t sr = start_read.toMilliseconds();
            uint32_t er = end_read.toMilliseconds();

            writetimes[i] = ew - sw;
            readtimes[i] = er - sr;
        }
        double avg_write = 0;
        double avg_read = 0;
        for(int i = 0; i < repeat; i++){
            avg_write += writetimes[i];
            avg_read += readtimes[i];
        }
        avg_write = avg_write / repeat;
        avg_read = avg_read / repeat;

        log.info("%d repetitions, %d sectors transfered ",repeat, sector);
        log.info("Average write time: %d ms", (int)avg_write);
        log.info("Average read time: %d ms", (int)avg_read);
    }
}