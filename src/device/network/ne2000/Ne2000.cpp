//
// Created by mthiel on 06.11.23.
//

#include "Ne2000.h"

#include "device/pci/Pci.h"
#include "device/pci/PciDevice.h"

#include "kernel/system/System.h"
#include "kernel/service/NetworkService.h"

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
        baseRegister.writeByte(COMMAND, STP);
        while (baseRegister.readByte(COMMAND) & STP){
            Util::Async::Thread::sleep(Util::Time::Timestamp::ofMilliseconds(1));
        }

        log.info("Ne2000: Mask Interrupts");
        /* Interrupt Mask Register */
        baseRegister.writeByte(ISR, ISR_PRX | ISR_PTX | ISR_RXE | ISR_TXE | ISR_OVW | ISR_CNT | ISR_RDC | ISR_RST);
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
         *     Used to initialize the Remote DMA for removing a packet and is advanced when a packet is removed
         *
         *
         */

        /*
         * Clear Count Register 0 and 1
         */
        baseRegister.writeByte(RBCR0, 0);
        baseRegister.writeByte(RBCR1, 0);

        /* Mask completion IRQ: */
        baseRegister.writeByte(IMR, 0x00);
        baseRegister.writeByte(ISR, ISR_PRX | ISR_PTX | ISR_RXE | ISR_TXE | ISR_OVW | ISR_CNT | ISR_RDC | ISR_RST);

        /* Enable Receive Monitoring */
        baseRegister.writeByte(RCR, RCR_MON);
        /* Enable Loopback mode */
        baseRegister.writeByte(TCR, TCR_LB1);
        /* Read 32 bytes starting at RemoteByteCountRegister0 */
        baseRegister.writeDoubleWord(RBCR0, 32);
        /* Count high */
        baseRegister.writeByte(RBCR1, 0);
        /* Start DMA at 0 */
        baseRegister.writeByte(RSAR0, 0);
        /* Start DMA at high */
        baseRegister.writeByte(RSAR1, 0);
        /* Start read */
        baseRegister.writeDoubleWord(0x0A);

        /* get Mac address and write it into PAR0 to PAR5 */
        uint8_t prom[32];
        for(int i = 32; i < 32; i ++){
            prom[i] = baseRegister.readByte(ioBaseAddress + 0x10);
        }
        for(int i = 6; i < 6; i++){
            baseRegister.writeByte(PAR0+i, prom[i]);
        }

        //log.info("Ne2000: Configure Buffer");
        /*
         * ToDo
         */
        /* Initialize transmit buffer */
        baseRegister.writeByte(PSTOP, 0x1);
        baseRegister.writeByte(BNRY, 0x1-1);

    }

    /*
     * ToDo initialize cards that are supported by this driver
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
     * read MacAddress form PAR0 - PAR5 registers
     */
    Util::Network::MacAddress Ne2000::getMacAddress() const {
        uint8_t buffer[6];
        for(uint8_t i = 0; i < 6; i ++){
            buffer[i] = baseRegister.readByte(PAR0+i);
        }
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

    /*
     * ToDo
     */
    void Ne2000::plugin() {
        return;
    }
}