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

    HBA_MEM *MapAHCIRegisters(uint32_t baseAddress) {
        auto &memoryService = Kernel::System::getService<Kernel::MemoryService>();
        void *mappedAddress = memoryService.mapIO(baseAddress, sizeof(HBA_MEM));

        if (mappedAddress == nullptr) {
            AhciController::log.error("Failed to map AHCI registers");
            return nullptr;
        }
        return reinterpret_cast<HBA_MEM *>(mappedAddress);
    }

    AhciController::AhciController(const PciDevice &device) {

        //ABAR auslesen und mappen
        uint32_t abar = device.readDoubleWord(Pci::BASE_ADDRESS_5);
        hbaMem = MapAHCIRegisters(abar);
        log.info("device id: %x, vendor id: %x", device.getDeviceId(), device.getVendorId());

        //Ownership übernehmen (AHCI Version >= 1.2)
        biosHandoff(device);

        //AHCI Modus aktivieren
        if(enableAHCIController(device) == -1){
            log.error("AHCI Controller could not be enabled");
            return;
        }

        //Interrupts deaktivieren
        hbaMem->ghc &= ~(1 << 1);

        //CAP.NP auslesen um Anzahl der Ports zu ermitteln, die vom HBA unterstützt werden
        uint32_t numPortsAllowed = (hbaMem->cap & 0x1F) + 1;


        for (uint8_t i = 0; i < AHCI_MAX_PORTS; i++) { 
            if(hbaMem->pi & (1 << i)){

                setPortinIdleState(&hbaMem->ports[i]);

                //Speicher für Command List und FIS reservieren
                port_rebase(&hbaMem->ports[i], i);

                //Enable receiving FISes
                hbaMem->ports[i].cmd |= (1 << 4);

                //Clear errors
                hbaMem->ports[i].serr = 0xFFFFFFFF;

                //Clear interrupt status
                hbaMem->ports[i].is = 0xFFFFFFFF;

                //Detect what device is connected to the port
                int dt = check_type(&hbaMem->ports[i]);
			    if (dt == AHCI_DEV_SATA){
                    log.info("SATA drive found at port %d", i);
                    identifyDevice(&hbaMem->ports[i]);
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
    int AhciController::identifyDevice(HBA_PORT *port) {
        auto &memoryService = Kernel::System::getService<Kernel::MemoryService>();
        //get unused command slot
        uint32_t slot = find_cmdslot(port);

        HBA_CMD_HEADER *cmdheader = (HBA_CMD_HEADER*)(port->clb);
        cmdheader += slot;
        cmdheader->prdtl = 1;	// PRDT entries count
        cmdheader->pmp = 0;		// Port multiplier value
	    cmdheader->a = 0;		// ATAPI
	    cmdheader->w = 0;		// Write, 1: H2D, 0: D2H
	    cmdheader->p = 0;		// Prefetchable
        cmdheader->r = 0;		// Reset
	    cmdheader->b = 0;	    // BIST
	    cmdheader->c = 0;	    // Clear busy upon R_OK
        cmdheader->cfl = 5;    // Command FIS length

        //Byte count field is 0 based reprensentation of 512 bytes, the size of data we expect on return
        cmdheader->prdbc = 512;

        HBA_CMD_TBL *cmdtbl = (HBA_CMD_TBL*)(cmdheader->ctba);

        //FIS
        FIS_REG_H2D *cmdfis = (FIS_REG_H2D*)(&cmdtbl->cfis);
        cmdfis->fis_type = FIS_TYPE_REG_H2D; //0x27
        cmdfis->c = 1; //Command
        cmdfis->command = ATA_IDENTIFY_DEVICE; //0xEC
        cmdfis->featurel = 0x00;
        cmdfis->featureh = 0x00;
        cmdfis->lba0     = 0x00;
        cmdfis->lba1     = 0x00;
        cmdfis->lba2     = 0x00;
        cmdfis->device   = 0xA0;
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
        port->ci = 1;

        //Wait for command completion
        while (1) {
            if ((port->ci & 1) == 0) 
                break;
            if (port->is & HBA_PxIS_TFES) { //Task file error
                log.error("Read disk error\n");
                return false;
            }
        }

        //get data from memory
        FIS_DATA *data = (FIS_DATA*)(&cmdtbl->cfis);
        log.info("FIS_TYPE: %x", data->fis_type);
    }

    void AhciController::initializeAvailableControllers() {
        Util::Array<PciDevice> devices = Pci::search(Pci::Class::MASS_STORAGE, PCI_SUBCLASS_AHCI);
        for (const auto &device: devices) {
            AhciController *controller = new AhciController(device);
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
        log.info("Cannot find free command list entry");
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
            log.info("AHCI Version < 1.2 for device: %u. skip BIOS/OS Handoff", device.getDeviceId());
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
        Util::Async::Thread::sleep(Util::Time::Timestamp::ofMilliseconds(1));
        //clear PxSCTL.DET
        port->sctl  &= ~(1 << 0);

        //wait until PxSSTS.DET is 3
        while((port->ssts & 0x0F) != HBA_PORT_DET_PRESENT){
            Util::Async::Thread::sleep(Util::Time::Timestamp::ofMilliseconds(1));
        }
        log.info("Port Reset done");
    }

    //start command engine (OSDEV)
    void AhciController::start_cmd(uint32_t port) {
        // Wait until CR (bit15) is cleared
        while (hbaMem->ports[port].cmd & HBA_PxCMD_CR);

        // Set FRE (bit4) and ST (bit0)
        hbaMem->ports[port].cmd |= HBA_PxCMD_FRE;
        hbaMem->ports[port].cmd |= HBA_PxCMD_ST;
    }

    //Stop command engine (OSDEV)
    void AhciController::stop_cmd(uint32_t port) {
        // Clear ST (bit0)
        hbaMem->ports[port].cmd &= ~HBA_PxCMD_ST;

        // Wait until FR (bit14), CR (bit15) are cleared
        while(1) {
            if (hbaMem->ports[port].cmd & HBA_PxCMD_FR)
                continue;
            if (hbaMem->ports[port].cmd & HBA_PxCMD_CR)
                continue;
            break;
        }

        // Clear FRE (bit4)
        hbaMem->ports[port].cmd &= ~HBA_PxCMD_FRE;
    }


    void AhciController::setPortinIdleState(HBA_PORT *port) {
        uint32_t timeout = 0;
        port->cmd &= ~HBA_PxCMD_ST;
        while (port->cmd & HBA_PxCMD_CR && timeout < 500){
            Util::Async::Thread::sleep(Util::Time::Timestamp::ofMilliseconds(1));
            timeout++;
        }

        if(timeout >= 500){
            log.error("Port in Idle State failed. Trying Port reset");
            portReset(port);
            return;
        }

        timeout = 0;
        port->cmd &= ~HBA_PxCMD_FRE;
        while (port->cmd & HBA_PxCMD_FR && timeout < 500){
            Util::Async::Thread::sleep(Util::Time::Timestamp::ofMilliseconds(1));
            timeout++;
        }

        if(timeout >= 500){
            log.error("Port in Idle State failed. Trying Port reset");
            portReset(port);
            return;
        }
    }
    
    //OSDEV
    void AhciController::port_rebase(HBA_PORT *port, int portno){
        auto &memoryService = Kernel::System::getService<Kernel::MemoryService>(); 

        AhciController::stop_cmd(portno); //Stop command engine

        //Command List
        auto cmdList = reinterpret_cast<uint32_t*>(memoryService.mapIO(sizeof(HBA_CMD_HEADER) * 32));
        auto cmdListPhysicalAdress = reinterpret_cast<uint32_t>(memoryService.getPhysicalAddress(cmdList));
        port->clb = cmdListPhysicalAdress;
        port->clbu = 0;

        //FIS
        auto fis = reinterpret_cast<uint32_t*>(memoryService.mapIO(sizeof(HBA_FIS)));
        auto fisPhysicalAdress = reinterpret_cast<uint32_t>(memoryService.getPhysicalAddress(fis));
        port->fb = fisPhysicalAdress;
        port->fbu = 0;

        //Command Table
        HBA_CMD_HEADER *cmdheader = (HBA_CMD_HEADER*)(port->clb);
        for(int i=0;i<32;i++){
            cmdheader[i].prdtl = 8;	// 8 prdt entries per command table // 256 bytes per command table, 64+16+48+16*8

            // Command table offset: 40K + 8K*portno + cmdheader_index*256
            auto cmdtbl = reinterpret_cast<uint32_t*>(memoryService.mapIO(256));
            auto cmdtblPhysicalAdress = reinterpret_cast<uint32_t>(memoryService.getPhysicalAddress(cmdtbl));
            cmdheader[i].ctba = cmdtblPhysicalAdress;
            cmdheader[i].ctbau = 0;

        }
        
        AhciController::start_cmd(portno);

    }

    //OSDEV
    bool AhciController::read(HBA_PORT *port, uint32_t startl, uint32_t starth, uint32_t count, uint16_t *buf){
        auto &memoryService = Kernel::System::getService<Kernel::MemoryService>(); 
	    port->is = (uint32_t) -1;		// Clear pending interrupt bits
	    int spin = 0; // Spin lock timeout counter
	    int slot = AhciController::find_cmdslot(port);
	    if (slot == -1)
		    return false;

        log.info("check_type: %u", check_type(0));

	    HBA_CMD_HEADER *cmdheader = (HBA_CMD_HEADER*)port->clb;
	    cmdheader += slot;
	    cmdheader->cfl = sizeof(FIS_REG_H2D)/sizeof(uint32_t);	// Command FIS size
	    cmdheader->w = 0;		// Read from device
	    cmdheader->prdtl = (uint16_t)((count-1)>>4) + 1;	// PRDT entries count
 
        log.info("PRDT entries count: %u", cmdheader->prdtl);
        
        auto cmdtbladdr = memoryService.mapIO(sizeof(HBA_CMD_TBL) + (cmdheader->prdtl-1)*sizeof(HBA_PRDT_ENTRY));
        cmdheader->ctba = reinterpret_cast<uint32_t>(memoryService.getPhysicalAddress(cmdtbladdr));
        HBA_CMD_TBL *cmdtbl = (HBA_CMD_TBL*)(cmdheader->ctba);
        

	    // 8K bytes (16 sectors) per PRDT
        int i;
	    for (i=0; i<cmdheader->prdtl-1; i++){

		    cmdtbl->prdt_entry[i].dba = (uint32_t) buf;
		    cmdtbl->prdt_entry[i].dbc = 8*1024-1;	// 8K bytes (this value should always be set to 1 less than the actual value)
		    cmdtbl->prdt_entry[i].i = 1;
		    buf += 4*1024;	// 4K words
		    count -= 16;	// 16 sectors
	    }
        
	    // Last entry
	    cmdtbl->prdt_entry[i].dba = (uint32_t) buf;
	    cmdtbl->prdt_entry[i].dbc = (count<<9)-1;	// 512 bytes per sector
	    cmdtbl->prdt_entry[i].i = 1;


	    // Setup command
	    FIS_REG_H2D *cmdfis = (FIS_REG_H2D*)(&cmdtbl->cfis);
 
	    cmdfis->fis_type = FIS_TYPE_REG_H2D;
	    cmdfis->c = 1;	// Command
	    cmdfis->command = ATA_CMD_READ_DMA_EX;
 
	    cmdfis->lba0 = (uint8_t)startl;
	    cmdfis->lba1 = (uint8_t)(startl>>8);
	    cmdfis->lba2 = (uint8_t)(startl>>16);
	    cmdfis->device = 1<<6;	// LBA mode
 
	    cmdfis->lba3 = (uint8_t)(startl>>24);
	    cmdfis->lba4 = (uint8_t)starth;
	    cmdfis->lba5 = (uint8_t)(starth>>8);
    
	    cmdfis->countl = count & 0xFF;
	    cmdfis->counth = (count >> 8) & 0xFF;
    
	    // The below loop waits until the port is no longer busy before issuing a new command
	    while ((port->tfd & (ATA_DEV_BUSY | ATA_DEV_DRQ)) && spin < 1000000){
		    spin++;
	    }
	    if (spin == 1000000){
		    AhciController::log.error("Port is hung\n");
		    return false;
        }


	    port->ci = 1<<slot;	// Issue command

        log.info("Port->ci: %x , slot %u", port->ci, slot);

	    // Wait for completion
	    while (1){
		// In some longer duration reads, it may be helpful to spin on the DPS bit 
		// in the PxIS port field as well (1 << 5)
		    if ((port->ci & (1<<slot)) == 0) 
			    break;
		    if (port->is & HBA_PxIS_TFES){ // Task file error
			    log.error("Read disk error\n");
			    return false;
		    }
	    }
 
	    // Check again
	    if (port->is & HBA_PxIS_TFES){
		    log.error("Read disk error\n");
		    return false;
	    }
        
	    return true;
    }

    void AhciController::plugin() {
        //TODO: Implement
    }

    void AhciController::trigger(const Kernel::InterruptFrame &frame) {
        //TODO: Implement
    }

}
