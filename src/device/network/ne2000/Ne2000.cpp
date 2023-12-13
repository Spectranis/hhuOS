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
/**
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

/**
 * Initialize Ne2000
 */
Ne2000::Ne2000(const PciDevice &pciDevice) : pciDevice(pciDevice) {
    uint16_t command = pciDevice.readWord( Pci::COMMAND );
    uint16_t buffer[32];
    command |= Pci::BUS_MASTER | Pci::IO_SPACE;
    pciDevice.writeWord(Pci::COMMAND, command);

    uint16_t ioBaseAddress = pciDevice.readDoubleWord(Pci::BASE_ADDRESS_0) & ~0x3;
    baseRegister = IoPort(ioBaseAddress);

    log.info("Start device");
    baseRegister.writeByte(COMMAND, STA);

    /**  1) CR Register = 21h */
    log.info("Reset device");
    baseRegister.writeByte(COMMAND, RD2 | STP);

    /**  2) Initialize DCR */
    log.info("Init DCR");
    /** Initialize Data Configuration register */
    baseRegister.writeByte(P0_DCR, DCR_LS | DCR_FT1 | DCR_AR);

    /**  3) Clear RBCR0, RBCR1 */
    /** Clear Count Register 0 and 1 */
    log.info("Clear RBCR0/1");
    baseRegister.writeByte(P0_RBCR0, 0x00);
    baseRegister.writeByte(P0_RBCR1, 0x00);

    /**  4) Initialize RCR */
    /** Receive Accept RUNT Packets, Broadcast and Multicast */
    log.info("Init RCR");
    baseRegister.writeByte(P0_RCR, RCR_AR | RCR_AB | RCR_AM);

    /**  5) Set NIC to LOOPBACK -> TCR = 02/04h */
    log.info("Set NIC to Loopback");
    baseRegister.writeByte(P0_TCR, TCR_LB0);

    /**  6) Initialize Receive Buffer Ring ( BNDRY, PSTART, PSTOP ) */
    log.info("Init Receive Buffer Ring");
    /** ToDo: Fix Buffer Init */
    /** https://github.com/torokernel/torokernel/blob/7d6df4c40fa4cc85febd5fd5799404592ffdff53/rtl/drivers/Ne2000.pas
     * Line 200 ff */
    baseRegister.writeByte(P0_TPSR, 0x40);
    baseRegister.writeByte(P0_PSTART, 0x46);
    /** At Initialization PSR is written into CPR and BPR*/
    baseRegister.writeByte(P0_BNRY, 0x46);
    baseRegister.writeByte(P0_PSTOP, 0x60);

    /**  7) Clear ISR */
    log.info("Clear ISR");
    /** Set Interrupt Status Register to 0xFF */
    baseRegister.writeByte(P0_ISR, 0xFF);

    /**  8) Initialize IMR */
    log.info("Init IMR");
    baseRegister.writeByte(P0_IMR, IMR_PRXE | IMR_PTXE | IMR_RXEE | IMR_TXEE | IMR_OVWE);

    /**  9) Switch to P1 -> CR = 61h */
    log.info("Switch to P1");
    baseRegister.writeByte(COMMAND, 0x61);

    /**  9) i) Initialize Physical Address Register: PAR0-PAR5 */
    /** Read 32 Bit from IOPort */
    log.debug("Get proc");
    for (int i = 0; i < 32; i++){
        buffer[i] = baseRegister.readByte(0x10);
    }
    log.info("MAC:%02x:%02x:%02x:%02x:%02x:%02x",
              buffer[0],
              buffer[2],
              buffer[4],
              buffer[6],
              buffer[8],
              buffer[10]);

    /** Write Mac Address into PAR0-PAR5 */
    baseRegister.writeByte(P1_PAR0, buffer[0]);
    baseRegister.writeByte(P1_PAR1, buffer[2]);
    baseRegister.writeByte(P1_PAR2, buffer[4]);
    baseRegister.writeByte(P1_PAR3, buffer[6]);
    baseRegister.writeByte(P1_PAR4, buffer[8]);
    baseRegister.writeByte(P1_PAR5, buffer[10]);

    /**  9) ii) Initialize Multicast Address Register: MAR0-MAR7 with 0xFF
     * https://github.com/torokernel/torokernel/blob/7d6df4c40fa4cc85febd5fd5799404592ffdff53/rtl/drivers/Ne2000.pas
     * line 213 ff
     * */
    log.info("Init Multicast Address Register");
    baseRegister.writeByte(P1_MAR0, 0xFF);
    baseRegister.writeByte(P1_MAR1, 0xFF);
    baseRegister.writeByte(P1_MAR2, 0xFF);
    baseRegister.writeByte(P1_MAR3, 0xFF);
    baseRegister.writeByte(P1_MAR4, 0xFF);
    baseRegister.writeByte(P1_MAR5, 0xFF);
    baseRegister.writeByte(P1_MAR6, 0xFF);
    baseRegister.writeByte(P1_MAR7, 0xFF);

    /**  9) iii) Initialize Current Pointer: CURR
     * 10.0 Internal Registers (P28) CURR init with same value as PSTART */
    log.info("Init CURR");
    int8_t nextPackage = baseRegister.readByte(P0_PSTART)+0x1;
    baseRegister.writeByte(P1_CURR, nextPackage);

    log.info("CurrentPageRegister: %02x", baseRegister.readByte(P1_CURR));

    /** 10) Put NIC in START Mode -> CR = 22H */
    log.info("Start NIC");
    baseRegister.writeByte(COMMAND, RD2 | STA);

    /** 11) Initialize TCR for intended value */
    log.info("Init TC");
    baseRegister.writeByte(P0_TCR, 0x00);

    log.info("NIC initialized");

    //Util::Async::Thread::sleep(Util::Time::Timestamp::ofMilliseconds(100000000));
}

void Ne2000::resetNe200() {

  baseRegister.writeByte(COMMAND, PS0 | STP | RD2);

  baseRegister.writeByte(COMMAND, PS1 | STP | RD2);

  baseRegister.writeByte(COMMAND, PS0 | STP);
}
/**
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

/**
 * Read MacAddress form PAR0 - PAR5 registers
 */
Util::Network::MacAddress Ne2000::getMacAddress() const {
    uint8_t buffer[6];

    /** Go to P2, STOP and block Remote DMA */
    baseRegister.writeByte(COMMAND, RD2 | PS0 | STP);
    /** Read MAC from PAR0 - PAR5 */
    buffer[0] = baseRegister.readByte(P1_PAR0);
    buffer[1] = baseRegister.readByte(P1_PAR1);
    buffer[2] = baseRegister.readByte(P1_PAR2);
    buffer[3] = baseRegister.readByte(P1_PAR3);
    buffer[4] = baseRegister.readByte(P1_PAR4);
    buffer[5] = baseRegister.readByte(P1_PAR5);

    /** COMMAND: Start and complete Remote DMA */
    baseRegister.writeByte(COMMAND, STA | RD2);

    /** Receive Config Accept Broadcast and Accept Multicast */
    baseRegister.writeByte(P0_RCR, RCR_AB | RCR_AM);

    return Util::Network::MacAddress(buffer);
}

/**
 * ToDo Resolve Interrupt trigger
 */
void Ne2000::trigger(const Kernel::InterruptFrame &frame) {
        auto interrupt = baseRegister.readWord(P0_ISR);
        /** Disable Interrupt and Save PC Registers */
        /** Read ISR Status */
        /** Packet Reception? */
        /** Then
         * Reset PRX Bit in ISR
         * DMA Packet from Local Mem to PC
         * Inform High Level Software of Received Packet
         * Is Receive Buffer Ring Empty : return
         * Else Repeat
         * */
        /** Packet Transmission? */
        /** Then
         * Reset PTX Bit in ISR
         * Read TSR
         * Inform High Level Software of Good/Bad Transmission
         * Is queue Empty: return
         * Transmit Next Packet in Queue
         * Update Queue Pointer
         * */
        /** ELSE
         * SEND EOI to 8259
         * Enable Interrupt
         * Return
         * */

        if(interrupt & ISR_PRX){
          while (baseRegister.readByte(P0_ISR) & ISR_PRX){
            /** Reset PRX Bit */
            baseRegister.writeByte(P0_ISR, ISR_PRX);
            /** Read Packet */
            //processReceivedPacket();
          }
        }


        /**
         * Clear Interrupt by writing a 1 into the corresponding bit
         */
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

/**
 * ToDo
 */
void Ne2000::handleBufferOverflow(){
        return;
    }

/**
 * ToDo
 */
void Ne2000::handleIncomingPacket(const uint8_t *packet, uint32_t length) {
    /**
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

    /**
     * 1) Read and store value of TXP in the CR
     * 2) Issue STOP Command to the NIC -> Set STP ind CR -> Write 21H to CR will stop NIC
     * 3) Wait for at least 1.6ms
     * 4) Clear NIC RBCR0 and RBCR1
     * 5) Read stored value of TXP (see 1)
     * 6)
     */


        return;
    }

/**
 * ToDo
 */
void Ne2000::handleOutgoingPacket(const uint8_t *packet, uint32_t length) {
        if(1 == 2){

        }

        /**
         * Ready to Transmit?
         *  No: Queue packet and return
         *  Yes:
         *      DMA Packet from PC to Local mem
         *      Transmit Packet
         *      return
         */

        /**
         * ISR Services the
         */

        return;
    }

void Ne2000::readPacket(){

  /** Go To P1, Start and Block Remote DMA */
  baseRegister.writeByte(COMMAND, STA | PS0 | RD2);
  auto current = baseRegister.readByte(P1_CURR);
  baseRegister.writeByte(COMMAND, STA | RD2);

  /** while
   * https://github.com/torokernel/torokernel/blob/7d6df4c40fa4cc85febd5fd5799404592ffdff53/rtl/drivers/Ne2000.pas
   * Line 243 ff
   * */
  baseRegister.writeByte(P0_RBCR0, 4);
  baseRegister.writeByte(P0_RBCR1, 0);
  baseRegister.writeByte(P0_RSAR0, 0);
  baseRegister.writeByte(P0_RBCR1, 0x00);

  baseRegister.writeByte(COMMAND, STA | RD0);
  /** Read from I/O Port */
  auto rsr = baseRegister.readByte(0x10);
  auto next = baseRegister.readByte(0x10);
  auto packet_length = baseRegister.readByte(0x10);
  packet_length = packet_length + baseRegister.readByte(0x10) << 8;

  baseRegister.writeByte(P0_ISR, ISR_RDC);
  if ((rsr | 31) == 1 && packet_length <= 1532){
    /** && next >= PSTART && next <= PSTOP */
    //auto data = int[packet_length];
    for (int i = 0; i < packet_length; i ++){
      //data[i] = baseRegister.readByte(0x10);
    }
  }


}
/**
 * ToDo
 */
void Ne2000::sendPacket(const uint8_t *packet, uint32_t length) {
    log.debug("Prepare Packet Send: Set CR to 0x22");
    baseRegister.writeByte(COMMAND,  RD2 | STA);

    log.debug("Write PacketSize into RBCR0/1");
    /** length & 0xFF write Byte 1+2 into RBCR0 */
    baseRegister.writeByte(P0_RBCR0, length & 0xFF);
    /** Bitshift 8 to the right -> write Byte 3+4 into RBCR1 */
    baseRegister.writeByte(P0_RBCR1, length >> 8);


    log.debug("Clear remote DMA complete bit");
    baseRegister.writeByte(P0_ISR, ISR_RDC);

    log.debug("Load RSAR0/1 with low and target page number");
    baseRegister.writeByte(P0_RSAR0, 0x00);
    baseRegister.writeByte(P0_RSAR1, 0x40); //Transmitbuffer


    log.debug("Set CR to start and Remote write DMA");
    baseRegister.writeByte(COMMAND, PS0 | RD1 | STA);

    log.debug("Write PacketData into DataPort");
    /** Write Packetdata to data port */
    for(uint32_t i = 0; i < length; i++){
        baseRegister.writeByte(0x10, packet[i]);
    }

    /**
    baseRegister.writeByte(P0_TBCR0, length & 0xFF);
    baseRegister.writeByte(P0_TBCR1, length >> 8);
    baseRegister.writeByte(COMMAND, TXP | RD2 | STA);
    */

    log.debug("Poll ISR until Bit 6 remote DMA complete is set");
    while(baseRegister.readByte(P0_ISR) != ISR_RDC ){
        Util::Async::Thread::sleep(Util::Time::Timestamp::ofMilliseconds(1));
    }
    log.debug("NE2000: transmitted packet");
}

/**
 * ToDo
 */
void Ne2000::plugin() {
        auto &interruptService = Kernel::System::getService<Kernel::InterruptService>();
        interruptService.allowHardwareInterrupt(pciDevice.getInterruptLine());
        interruptService.assignInterrupt(static_cast<Kernel::InterruptVector>(pciDevice.getInterruptLine() + 32), *this);
    }
}