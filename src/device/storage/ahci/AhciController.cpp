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

namespace Kernel {
    class Logger;
}

namespace Device::Storage {
    Kernel::Logger AhciController::log = Kernel::Logger::get("AHCI");
    HBA_MEM *hbaMem;
    virtual_port_addr vpa[32]; //Virtual port addresses of each port
    uint32_t lba_capacity[32]; //LBA capacity of each port
    uint32_t sector_size[32]; //sector size of each port in bytes (512 or 4096)
    bool supports48LBA[32]; //true if port supports 48-bit LBA

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

        //ABAR auslesen und mappen
        uint32_t abar = device.readDoubleWord(Pci::BASE_ADDRESS_5);
        hbaMem = MapAHCIRegisters(abar);
        log.info("device id: %x, vendor id: %x", device.getDeviceId(), device.getVendorId());
        log.info("ahci version: %x", hbaMem->vs);

        //Ownership übernehmen (AHCI Version >= 1.2)
        biosHandoff(device);

        //AHCI Modus aktivieren
        if(enableAHCIController(device) == -1){
            log.error("AHCI Controller could not be enabled");
            return;
        }

        //CAP.NP auslesen um Anzahl der Ports zu ermitteln, die vom HBA unterstützt werden
        uint8_t numPortsAllowed = (hbaMem->cap & 0x1F) + 1;
        log.info("Number of ports allowed: %u", numPortsAllowed);

        for (uint8_t i = 0; i < 1; i++) { //i < AHCI_MAX_PORTS
            if(hbaMem->pi & (1 << i)){

                portReset(&hbaMem->ports[i]);

                log.info("Port Status: %x", hbaMem->ports[i].ssts);
                log.info("Port Control: %x", hbaMem->ports[i].sctl);

                //Speicher für Command List und FIS reservieren
                port_rebase(&hbaMem->ports[i], i);

                //Clear errors
                hbaMem->ports[i].serr = 0xFFFFFFFF;

                //Clear interrupt status
                hbaMem->ports[i].is = 0xFFFFFFFF;

                //Enable interrupts
                hbaMem->ports[i].ie = 0xFFFFFFFF;

                log.info("Port Control: %x", hbaMem->ports[i].sctl);
                
                //Detect what device is connected to the port
                int dt = check_type(&hbaMem->ports[i]);
			    if (dt == AHCI_DEV_SATA){
                    log.info("SATA drive found at port %d", i);
                    log.info("Port Status: %x", hbaMem->ports[i].ssts);
                    identifyDevice(&hbaMem->ports[i], i);

                    writeOneSector(&hbaMem->ports[i], i, 500, 0);
                    readOneSector(&hbaMem->ports[i], i, 500, 0);

                    //auto buffer = reinterpret_cast<uint32_t*>(memoryService.mapIO(512));
                    //read(&hbaMem->ports[i], i, 500, 0, 1, buffer);

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
        log.info("slot: %u", slot);

        //HBA_CMD_HEADER *cmdheader = (HBA_CMD_HEADER*)(port->clb);
        HBA_CMD_HEADER *cmdheader = (HBA_CMD_HEADER*) vpa[portno].cmdList;
        log.info("identify clb: %x", port->clb);
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

        //Byte count field is 0 based reprensentation of 512 bytes, the size of data we expect on return
        cmdheader->prdbc = 511;

        HBA_CMD_TBL *cmdtbl = (HBA_CMD_TBL*) vpa[portno].commandTable[slot];
        //HBA_CMD_TBL *cmdtbl = (HBA_CMD_TBL*)(cmdheader->ctba);
        //FIS
        FIS_REG_H2D *cmdfis = (FIS_REG_H2D*) cmdtbl->cfis;
        //FIS_REG_H2D *cmdfis = (FIS_REG_H2D*)(cmdtbl->cfis);
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

        log.info("port->ie: %x", port->ie);
        log.info("port->is: %x", port->is);

        SATA_ident_t *SATA_Identify_info =(SATA_ident_t*)(dba);

        auto mdl = strip_and_swap((char*) SATA_Identify_info->model, 40);
        auto serial = strip_and_swap((char*) SATA_Identify_info->serial_no, 20);
        auto fw = strip_and_swap((char*) SATA_Identify_info->fw_rev, 8);

        lba_capacity[portno] = SATA_Identify_info->lba_capacity;
        sector_size[portno] = SATA_Identify_info->sector_bytes;
        supports48LBA[portno] = SATA_Identify_info->command_set_2 & (1 << 10);

        log.info("Found SATA drive: %s on port [%d]: %s (Firmware: [%s])", serial, portno, mdl, fw);
        return 0;
    }

    bool AhciController::read(HBA_PORT *port,int portno, uint32_t startl, uint32_t starth, uint32_t count, void* buffer){
        auto &memoryService = Kernel::System::getService<Kernel::MemoryService>();    

        port->is = (uint32_t) -1;		// Clear pending interrupt bits
        int spin = 0;
        int slot = find_cmdslot(port);
        if (slot == -1)
            return false;

        HBA_CMD_HEADER *cmdheader = (HBA_CMD_HEADER*) vpa[portno].cmdList;
        cmdheader += slot;
        cmdheader->cfl = sizeof(FIS_REG_H2D) / sizeof(uint32_t);	// Command FIS size
        cmdheader->w = 0;		// Read from device
        cmdheader->prdtl = (uint16_t)((count - 1) >> 4) + 1;	// PRDT entries count

        HBA_CMD_TBL *cmdtbl = (HBA_CMD_TBL*) vpa[portno].commandTable[slot];


        auto bufphy = memoryService.getPhysicalAddress(buffer);
        // 8K bytes (16 sectors) per PRDT
        int i = 0;
        for (i = 0; i < cmdheader->prdtl - 1; i++){
            cmdtbl->prdt_entry[i].dba = (uint32_t)(uint64_t)bufphy;
            cmdtbl->prdt_entry[i].dbau = 0; // (uint32_t)((uint64_t)buffer >> 32);
            cmdtbl->prdt_entry[i].dbc = 0x2000 - 1;
            cmdtbl->prdt_entry[i].i = 1;
            bufphy = (uint8_t*)bufphy + 0x2000;
            count -= 16;
        }

        cmdtbl->prdt_entry[i].dba = (uint32_t)(uint64_t)bufphy;
        cmdtbl->prdt_entry[i].dbau = 0; //(uint32_t)((uint64_t)buffer >> 32);
        cmdtbl->prdt_entry[i].dbc = (count << 9) - 1; // 512 bytes per sector
        cmdtbl->prdt_entry[i].i = 1;

        FIS_REG_H2D *cmdfis = (FIS_REG_H2D*) cmdtbl->cfis;
        cmdfis->fis_type = FIS_TYPE_REG_H2D;
        cmdfis->c = 1;	// Command
        cmdfis->command = ATA_CMD_READ_DMA_EX;

        cmdfis->lba0 = (uint8_t)startl;
        cmdfis->lba1 = (uint8_t)(startl >> 8);
        cmdfis->lba2 = (uint8_t)(startl >> 16);
        cmdfis->device = 1 << 6;	// LBA mode

        cmdfis->lba3 = (uint8_t)(startl >> 24);
        cmdfis->lba4 = (uint8_t)starth;
        cmdfis->lba5 = (uint8_t)(starth >> 8);

        cmdfis->countl = count & 0xFF;
        cmdfis->counth = (count >> 8) & 0xFF;

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
        log.info("slot: %u", slot);

        //HBA_CMD_HEADER *cmdheader = (HBA_CMD_HEADER*)(port->clb);
        HBA_CMD_HEADER *cmdheader = (HBA_CMD_HEADER*) vpa[0].cmdList;
        log.info("identify clb: %x", port->clb);
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

        HBA_CMD_TBL *cmdtbl = (HBA_CMD_TBL*) vpa[0].commandTable[slot];

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

        log.info("port->ie: %x", port->ie);
        log.info("port->is: %x", port->is);

        //print the buffer
        log.info("Buffer: %x", dba[0]);
        log.info("Buffer: %x", dba[1]);
        log.info("Buffer: %x", dba[2]);
        log.info("Buffer: %x", dba[3]);
        log.info("Buffer: %x", dba[4]);

    }

    bool AhciController::writeOneSector(HBA_PORT *port, int portno, uint32_t startl, uint32_t starth){
        auto &memoryService = Kernel::System::getService<Kernel::MemoryService>();
        port->is = (uint32_t) -1;		// Clear pending interrupt bits

        //get unused command slot
        uint32_t slot = find_cmdslot(port);
        log.info("slot: %u", slot);

        //HBA_CMD_HEADER *cmdheader = (HBA_CMD_HEADER*)(port->clb);
        HBA_CMD_HEADER *cmdheader = (HBA_CMD_HEADER*) vpa[0].cmdList;
        log.info("identify clb: %x", port->clb);
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
        cmdfis->control  = 0x08;
        cmdfis->rsv0     = 0x00;

        auto dba = reinterpret_cast<uint32_t*>(memoryService.mapIO(512));
        dba[0] = 0x12345678;
        dba[1] = 0x87654321;
        dba[2] = 0x12345678;
        dba[3] = 0x87654321;
        dba[4] = 0x12345678;

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
                log.error("Write disk error\n");
                return false;
            }
        }

        log.info("port->ie: %x", port->ie);
        log.info("port->is: %x", port->is);

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
        cmdheader->cfl = sizeof(FIS_REG_H2D) / sizeof(uint32_t);	// Command FIS size
        cmdheader->w = 1;		// Write to device
        cmdheader->prdtl = (uint16_t)((count - 1) >> 4) + 1;	// PRDT entries count

        HBA_CMD_TBL *cmdtbl = (HBA_CMD_TBL*) vpa[portno].commandTable[slot];

        auto bufphy = memoryService.getPhysicalAddress(buffer);
        int i = 0;
        for (i = 0; i < cmdheader->prdtl - 1; i++){
            cmdtbl->prdt_entry[i].dba = (uint32_t)(uint64_t)bufphy;
            cmdtbl->prdt_entry[i].dbau = 0; // (uint32_t)((uint64_t)buffer >> 32);
            cmdtbl->prdt_entry[i].dbc = 8*1024 - 1;
            cmdtbl->prdt_entry[i].i = 1;
            bufphy = (uint8_t*)bufphy + 4*1024;
            count -= 16;
        }

        cmdtbl->prdt_entry[i].dba = (uint32_t)(uint64_t)bufphy;
        cmdtbl->prdt_entry[i].dbau = 0; //(uint32_t)((uint64_t)buffer >> 32);
        cmdtbl->prdt_entry[i].dbc = (count << 9) - 1; // 512 bytes per sector
        cmdtbl->prdt_entry[i].i = 1;

        FIS_REG_H2D *cmdfis = (FIS_REG_H2D*) cmdtbl->cfis;
        cmdfis->fis_type = FIS_TYPE_REG_H2D;
        cmdfis->c = 1;	// Command
        cmdfis->command = ATA_CMD_WRITE_DMA_EX;

        cmdfis->lba0 = (uint8_t)startl;
        cmdfis->lba1 = (uint8_t)(startl >> 8);
        cmdfis->lba2 = (uint8_t)(startl >> 16);
        cmdfis->device = 1 << 6;	// LBA mode

        cmdfis->lba3 = (uint8_t)(startl >> 24);
        cmdfis->lba4 = (uint8_t)starth;
        cmdfis->lba5 = (uint8_t)(starth >> 8);

        cmdfis->countl = count & 0xFF;
        cmdfis->counth = (count >> 8) & 0xFF;

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
            //controller->plugin();
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
                    log.info("BIOS/OS Handoff failed for device: %u", device.getDeviceId());
                    return -1;
                }

                //Clear the OOC bit
                hbaMem->bohc = hbaMem->bohc & ~(1 << 3);

            } else {
                log.info("BIOS/OS Handoff not supported for device: %u", device.getDeviceId());
                return 0;
            }
        } else{
            log.info("AHCI Version < 1.2 for device: %x. skip BIOS/OS Handoff", device.getDeviceId());
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
        log.info("Port Reset done");
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
        //HBA_CMD_HEADER *cmdheader = (HBA_CMD_HEADER*)(port->clb);
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

}