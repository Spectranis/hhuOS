//
// Created by mthiel on 06.11.23.
//

#include "Ne2000.h"

#include "device/pci/Pci.h"
#include "device/pci/PciDevice.h"


#include "lib/util/collection/Array.h"
#include "lib/util/base/Address.h"
#include "lib/util/time/Timestamp.h"
#include "lib/util/async/Thread.h"

/*
 * Include necessary kernel modules
 */
#include "kernel/service/InterruptService.h"
#include "kernel/service/MemoryService.h"
#include "kernel/log/Logger.h"
#include "kernel/system/System.h"
#include "kernel/service/NetworkService.h"


namespace Kernel {
    struct InterruptFrame;
    enum InterruptVector : uint8_t;
}  // namespace Kernel

namespace Device::Network {
    Kernel::Logger Ne2000::log = Kernel::Logger::get("Ne2000");

/*
 * ToDo Implement Hardware Initialization method
 */
Ne2000::Ne2000(const PciDevice &pciDevice) : pciDevice(pciDevice) {
    log.info("Configure Ne2000 PCI registers");

    uint16_t command = pciDevice.readWord( Pci::COMMAND );
    uint8_t buffer[32];
    command |= Pci::BUS_MASTER | Pci::IO_SPACE;
    pciDevice.writeWord(Pci::COMMAND, command);

    // ~0x3
    uint16_t ioBaseAddress = pciDevice.readDoubleWord(Pci::BASE_ADDRESS_0) & ~0x3;
    baseRegister = IoPort(ioBaseAddress);
    /*
     * ToDo Power on ne2000
     */
    log.info("Ne2000: Start device");
    baseRegister.writeByte(COMMAND, STA);


    log.info("Ne2000: Reset device");
    /* RD2 set -> abort all DMA */
    baseRegister.writeByte(COMMAND, RD2 | STP);

    log.debug("Ne2000: Block ISR");
    /* Set Interrupt Status Register to 0xFF */
    baseRegister.writeByte(ISR, ISR_PRX | ISR_PTX | ISR_RXE | ISR_TXE | ISR_OVW | ISR_CNT | ISR_RDC | ISR_RST);
    log.debug("Ne2000: Stop and Block DMA ");
    baseRegister.writeByte(COMMAND, STP | RD2);
    /* Initialize Data Configuration register */
    baseRegister.writeByte(DCR, DCR_LS | DCR_FT1 | DCR_AR);

    /* Clear Count Register 0 and 1 */
    baseRegister.writeByte(RBCR0, 0x20);
    baseRegister.writeByte(RBCR1, 0);
    /* Set RSAR1/2 to 0 */
    baseRegister.writeByte(RSAR0, 0);
    baseRegister.writeByte(RSAR1, 0);
    /* Start and Set Remote DMA to remote read */
    baseRegister.writeByte(COMMAND, STA | RD0);
    /* Receive Accept RUNT Packets, Broadcast and Multicast */
    baseRegister.writeByte(RCR, RCR_AR | RCR_AB | RCR_AM);

    /* Disable Loopback mode */
    baseRegister.writeByte(TCR, TCR_LB1);

    /* Read 32 Bit from IOPort */
    for (int i = 0; i < 32; i++){
        buffer[i] = baseRegister.readByte(0x11);
    }
/*
        baseRegister.writeByte(COMMAND, RD2 | PS0 | STP);
        baseRegister.writeByte(PAR0, 0x12);
        buffer[0] = baseRegister.readByte(PAR0);
        buffer[1] = baseRegister.readByte(PAR1);
        buffer[2] = baseRegister.readByte(PAR2);
        buffer[3] = baseRegister.readByte(PAR3);
        buffer[4] = baseRegister.readByte(PAR4);
        buffer[5] = baseRegister.readByte(PAR5);
*/
    log.info("NE2000: MAC:%02x:%02x:%02x:%02x:%02x:%02x",
             buffer[0],
             buffer[1],
             buffer[2],
             buffer[3],
             buffer[4],
             buffer[5]);

    Util::Async::Thread::sleep(Util::Time::Timestamp::ofMilliseconds(10000000000));

    baseRegister.writeByte(TPSR, 0x40);
    baseRegister.writeByte(PSTART, 0x46);
    baseRegister.writeByte(BNRY, 0x46);
    baseRegister.writeByte(PSTOP, 0x60);

    /* Mask Interrupts */
    baseRegister.writeByte(IMR, IMR_PRXE | IMR_PTXE | IMR_RXEE | IMR_TXEE | IMR_OVWE);

    baseRegister.writeByte(COMMAND, RD2 | PS0 | STP);

    /* Write Physical Address from Buffer[0..6] into PAR0 .. PAR5 */
    for (int8_t i = 0; i < 6; i ++){
        baseRegister.writeByte(PAR0+i, buffer[i]);
    }

    baseRegister.writeByte(RSAR0, 0xff);
    baseRegister.writeByte(RSAR1, 0xff);
    baseRegister.writeByte(RBCR0, 0xff);
    baseRegister.writeByte(RBCR1, 0xff);
    baseRegister.writeByte(TCR, 0xff);
    baseRegister.writeByte(DCR, 0xff);
    baseRegister.writeByte(IMR, 0xff);

    /* Set Dataconfiguration FIFO Threshold to 4 Bytes, set loopback to normal operation and auto initialize remote */
    baseRegister.writeByte(DCR, DCR_AR | DCR_FT1 | DCR_LS);

    int8_t nextPackage = baseRegister.readByte(PSTART) + 0x1;
    baseRegister.writeByte(CURR, nextPackage);

    /* COMMAND: Start and  complete Remote DMA */
    baseRegister.writeByte(COMMAND, STA | RD2);

    /* Receive Config Accept Broadcast and Accept Multicast */
    baseRegister.writeByte(RCR, RCR_AB | RCR_AM);


    log.info("Ne2000: Configure Buffer");
    /*
         * Initialization Packet Reception:
         *   - reserve 64k byte or 32k word address space for receive buffer ring
         *   - PSTART and PSTOP define the boundaries where the buffer resides
         * BufferRing Initialization:
         *  Two static registers:
         *  - Page Start Register
         *  - Page Stop Register
         *  Two working registers:
         *  - Current Page Register (CPR) -> "Write Pointer"
         *      Points to the first buffer
         *      used to store a packet and is used to restore the DMA for writing status to the Buffer Ring
         *      or to restore DMA address in the event of a RUNT packet, a CRCm or Frame Alignment Error
         *  - Boundary Pointer Register -> "Read Pointer"
         *     Points to the first packet in the Ring not yet read by the host
         *     If local DMA address ever reaches Boundary the reception is aborted
         *     Used to initialize the Remote DMA for removing a packet and is advanced when a packet is remove.
         */

    /*
     * ToDo Initialize buffer
     * Initialize transmit buffer */
    //baseRegister.writeByte(PSTOP, 0x1);
    //baseRegister.writeByte(BNRY, 0x1-1);
    /* Set Transmit Page Start Register */
    //baseRegister.writeByte(TPSR, transmitBuffer);

    /*
        baseRegister.writeByte(PSTART, //bufferstart);
        baseRegister.writeByte(BNDRY, //r buf end);
        baseRegister.writeByte(PSTOP, //buf end);
        */


    /* Enable & start NIC */
    baseRegister.writeByte(COMMAND,  RD2 | STA);
    /* Reset TCR */
    baseRegister.writeByte(TCR, 0x00 );
    /* Allow broadcast */
    baseRegister.writeByte(RCR, RCR_AB);

}

/*
 * Initialize cards that are supported by this driver
 */
void Ne2000::initializeAvailableCards() {
    auto &networkService = Kernel::System::getService<Kernel::NetworkService>();
    auto devices = Pci::search(VENDOR_ID, DEVICE_ID);

    for (const auto &device:devices){
        auto *ne2000 = new Ne2000(device);
        ne2000->plugin();
        networkService.registerNetworkDevice(ne2000, "eth");
    }
}


/*
 * ToDo read MacAddress form PAR0 - PAR5 registers
 */
Util::Network::MacAddress Ne2000::getMacAddress() const {

    uint8_t buffer[6];

    /* Go to P2 to read PAR0-5, STOP and block Remote RMA */
    baseRegister.writeByte(COMMAND, RD2 | PS0 | STP);
        /* Read MAC from P1 PAR0 - PAR5 */
    buffer[0] = baseRegister.readByte(PAR0);
    buffer[1] = baseRegister.readByte(PAR1);
    buffer[2] = baseRegister.readByte(PAR2);
    buffer[3] = baseRegister.readByte(PAR3);
    buffer[4] = baseRegister.readByte(PAR4);
    buffer[5] = baseRegister.readByte(PAR5);

    /* COMMAND: Start and complete Remote DMA */
    baseRegister.writeByte(COMMAND, STA | RD2);

    /* Receive Config Accept Broadcast and Accept Multicast */
    baseRegister.writeByte(RCR, RCR_AB | RCR_AM);

    return Util::Network::MacAddress(buffer);
}

/*
 * ToDo Resolve Interrupt trigger
 */
void Ne2000::trigger(const Kernel::InterruptFrame &frame) {
        auto interrupt = baseRegister.readWord(ISR);

        // Handle Packet received
        if(interrupt & ISR_PRX){
            //ToDo: Transfer Packet into ReceiveBuffer
            /*
             * 1. Reset PRX Bit in ISR
             * 2. DMA Packet from Local Mem to PC
             * 3. Inform High level software of received packet
             * 4. Is Ringbuffer empty?
             *      yes: return
             *      no: jump to 1.
             */
        }
        // Handle Packet Transmitted
        else if (interrupt & ISR_PTX){
            // ToDo: Delete Packet from SendBuffer
            /*
             * Reset PTX Bit in ISR
             * Read Transmit Status Register
             * Inform High level software of Good/Bad Transmission
             * Is Transmission Queue Empty?
             *  No transmit next packet in queue
             *  Update Queue Pointers
             */
        }
        //Handle Receive Error
        else if (interrupt & ISR_RXE){
            // ToDo: Handle Receive Error
        }
        //Handle Transmit Error
        else if(interrupt & ISR_TXE){
            // ToDo: Handle Transmit Error
        }
        //Handle Overwrite Error
        else if(interrupt & ISR_OVW){
            // ToDo: Handle Overwrite Error
        }
        //Handle Counter Overflow
        else if(interrupt & ISR_CNT){
            //ToDo: Handle Counter Overflow
        }

    }

/*
 * ToDo
 */
void Ne2000::handleBufferOverflow(){
        return;
    }

/*
 * ToDo
 */
void Ne2000::handleIncomingPacket(const uint8_t *packet, uint32_t length) {
    /*
     * Buffer Ring Structure ->
     *   Buffer page fixed length 256 Bytes
     *   Consists of N pages
     * Location is in PageStart (PSTART) and PageStop (PSTOP) register
     * Assignment is handled in the NIC
     *   - provides linking of buffer pages for long packets
     *   - recovery of buffers when a packet is rejected
     *   - recirculation of buffer pages that have been read by the host
     * Handling:
     *   DMA treats list of buffers as logical ring, whenever the DMA address reaches the Page Stip Address,
     *   the DMA is reset to Page Start Address
     *
     * PacketReception:
     *   begins storing packet at the location pointed by current page register
     *   save offset of 4 bytes in the first buffer to allow room for storing receive status for the packet
     * PacketLinking:
     *   if packet length > 256 - 4 bytes -> DMA performs a forward link to the next buffer to store the remainder of the packet
     *   Case maximal lenght packet: 6 contiguous Pages will be linked
     *
     *   Steps before linking 2 comparisons:
     *   1) test equality between DMA address of the next buffer and the contents of the PageStopRegister
     *      if equal: buffer management logic will restore DMA to the first buffer in the Receive Buffer Ring
     *   2) Test equality between the DMA address of the next buffer address and the contents of the Boundary Pointer Register
     *     if equal: abort reception
     *   else: if Buffer address does not match Boundary Pointer || PageStopAddress the link to the next buffer is performed
     *
     *   Linking Buffers:
     *   Check: address equality to PSTOP and BoundaryPointer
     *     if not equal: allow DMA to use next buffer
     *
     */

    /*
     * 1) Read and store value of TXP in the CR
     * 2) Issue STOP Command to the NIC -> Set STP ind CR -> Write 21H to CR will stop NIC
     * 3) Wait for at least 1.6ms
     * 4) Clear NIC RBCR0 and RBCR1
     * 5) Read stored value of TXP (see 1) )
     * 6)
     */

        return;
    }

/*
 * ToDo
 */
void Ne2000::handleOutgoingPacket(const uint8_t *packet, uint32_t length) {
        /*
         * Ready to Transmit?
         *  No: Queue packet and return
         *  Yes:
         *      DMA Packet from PC to Local mem
         *      Transmit Packet
         *      return
         */

        /*
         * ISR Services the
         */
        return;
    }

void Ne2000::sendPacket(const uint8_t *packet, uint32_t length) {
    /*
     * 1. Set COMMAND Register  to start and nodma (0x22)
     * 2. RBCR0/1 are loaded with the packetsize
     * 3. Remote DMA complete bit is cleared by writing a 1 in bit 6 of ISR
     * 4. RSAR0/1 are loaded with 0x00 (low) and target page number (high) -> Chip is rdy to receive packet data and store it in the ring buffer for emission
     * 5. COMMAND register gets set to "start" and "remote write DMA" (0x12)
     * 6. Packet data is now written to the "data port" (register 0x10) of the NIC in a loop or using an "outsx" if available) -> NIC will then update its remote DMA logic after each written 16-bit value and places bytes in transmit buffer
     * 7. Poll ISR until bit 6 remote DMA completed is set
     */

    baseRegister.writeByte(COMMAND, PS0 | RD2 | STA);
    /* 2. Load RBCR0/1 with packet size */
    /* length & 0xFF write Byte 1+2 into RBCR0 */
    baseRegister.writeByte(RBCR0, length & 0xFF);
    /* Bitshift 8 to the right -> write Byte 3+4 into RBCR1 */
    baseRegister.writeByte(RBCR1, length >> 8);
    /* 3. Clear remote DMA complete BIT */
    baseRegister.writeByte(ISR, ISR_RDC);
    /* 4. */
    baseRegister.writeByte(RSAR0, 0x00);
    baseRegister.writeByte(RSAR1, 0x40); //Transmitbuffer

    /* 5. Set CR to "Start" and "Remote write DMA" */
    baseRegister.writeByte(COMMAND, PS0 | RD1 | STA);
    /* Write Packetdata to data port */
    for(uint32_t i = 0; i < length; i++){
        baseRegister.writeByte(0x10, packet[i]);
    }
    baseRegister.writeByte(TBCR0, length & 0xFF);
    baseRegister.writeByte(TBCR1, length >> 8);
    baseRegister.writeByte(COMMAND, TXP | RD2 | STA);

    log.debug("NE2000: transmitted packet");

    /* Poll ISR until RDMA is completed */
    while(baseRegister.readByte(ISR) != ISR_RDC ){
        Util::Async::Thread::sleep(Util::Time::Timestamp::ofMilliseconds(1));
    }
}

/*
 * ToDo
 */
void Ne2000::plugin() {
        auto &interruptService = Kernel::System::getService<Kernel::InterruptService>();
        interruptService.allowHardwareInterrupt(pciDevice.getInterruptLine());
        interruptService.assignInterrupt(static_cast<Kernel::InterruptVector>(pciDevice.getInterruptLine() + 32), *this);
    }
}