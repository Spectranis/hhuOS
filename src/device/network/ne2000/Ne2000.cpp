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
  uint16_t command = pciDevice.readWord(Pci::COMMAND);
  uint8_t buffer[32];
  command |= Pci::BUS_MASTER | Pci::IO_SPACE;
  pciDevice.writeWord(Pci::COMMAND, command);

  uint16_t ioBaseAddress = pciDevice.readDoubleWord(Pci::BASE_ADDRESS_0) & ~0x3;
  baseRegister = IoPort(ioBaseAddress);

  log.info("Start NIC");
  baseRegister.writeByte(COMMAND, STA);

  /**  1) CR Register = 21h */
  log.debug("Reset device");
  baseRegister.writeByte(COMMAND, RD2 | STP);

  /**  2) Initialize DCR */
  log.debug("Init DCR");
  /** Initialize Data Configuration register */
  baseRegister.writeByte(P0_DCR, DCR_LS | DCR_FT1 | DCR_AR);

  /**  3) Clear RBCR0, RBCR1 */
  /** Clear Count Register 0 and 1 */
  log.debug("Clear RBCR0/1");
  baseRegister.writeByte(P0_RBCR0, 0x00);
  baseRegister.writeByte(P0_RBCR1, 0x00);

  /**  4) Initialize RCR */
  /** Receive Accept RUNT Packets, Broadcast and Multicast */
  log.debug("Init RCR");
  baseRegister.writeByte(P0_RCR, RCR_AR | RCR_AB | RCR_AM);

  /**  5) Set NIC to LOOPBACK -> TCR = 02/04h */
  log.debug("Set NIC to Loopback");
  baseRegister.writeByte(P0_TCR, TCR_LB0);

  /**  6) Initialize Receive Buffer Ring ( BNDRY, PSTART, PSTOP ) */
  log.debug("Init Receive Buffer Ring");
  /** ToDo: Fix Buffer Init */
  /** https://github.com/torokernel/torokernel/blob/7d6df4c40fa4cc85febd5fd5799404592ffdff53/rtl/drivers/Ne2000.pas
     * Line 200 ff */
  baseRegister.writeByte(P0_TPSR, 0x40);
  baseRegister.writeByte(P0_PSTART, 0x46);
  /** At Initialization PSR is written into CPR and BPR*/
  baseRegister.writeByte(P0_BNRY, 0x46);
  baseRegister.writeByte(P0_PSTOP, 0x60);

  /**  7) Clear ISR */
  log.debug("Clear ISR");
  /** Set Interrupt Status Register to 0xFF */
  baseRegister.writeByte(P0_ISR, 0xFF);

  /**  8) Initialize IMR */
  log.debug("Init IMR");
  baseRegister.writeByte(P0_IMR,
                         IMR_PRXE | IMR_PTXE | IMR_RXEE | IMR_TXEE | IMR_OVWE);

  /**  9) Switch to P1 -> CR = 61h */
  log.debug("Switch to P1");
  baseRegister.writeByte(COMMAND, 0x61);

  /**  9) i) Initialize Physical Address Register: PAR0-PAR5 */
  /** Read 32 Bit from IOPort */
  log.debug("Read I/O and get MAC Address");
  for (int i = 0; i < 32; i++) {
    buffer[i] = baseRegister.readByte(0x10);
  }
  log.debug("MAC:%02x:%02x:%02x:%02x:%02x:%02x",
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
  log.debug("Init Multicast Address Register");
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
  log.debug("Init CURR");
  int8_t nextPackage = baseRegister.readByte(P0_PSTART) + 0x1;
  baseRegister.writeByte(P1_CURR, nextPackage);

  log.debug("CurrentPageRegister: %02x", baseRegister.readByte(P1_CURR));

  /** 10) Put NIC in START Mode -> CR = 22H */
  log.debug("Start NIC");
  baseRegister.writeByte(COMMAND, RD2 | STA);

  /** 11) Initialize TCR for intended value */
  log.debug("Init TC");
  baseRegister.writeByte(P0_TCR, 0x00);

  log.info("NIC initialized");
}

/**
 * Needed to reset the Ne2000 before every receive/send call
 */
void Ne2000::resetNe2000() {
  // https://github.com/gammy/dosbox_ne2000/blob/master/src/hardware/ne2000.cpp

  baseRegister.writeByte(COMMAND, 0x00);
  baseRegister.writeByte(P0_ISR, 0x00);
  baseRegister.writeByte(P0_IMR, 0x00);
  baseRegister.writeByte(P0_DCR, 0x00);
  baseRegister.writeByte(P0_TCR, 0x00);
  baseRegister.writeByte(P0_TSR, 0x00);
  baseRegister.writeByte(P0_RSR, 0x00);

  baseRegister.writeByte(COMMAND, PS1 | STP | RD2);

  baseRegister.writeByte(COMMAND, PS0 | STP);
}
/**
 * Initialize cards that are supported by this driver
 */
void Ne2000::initializeAvailableCards() {
  auto &networkService = Kernel::System::getService<Kernel::NetworkService>();
  auto devices = Pci::search(VENDOR_ID, DEVICE_ID);

  for (const auto &device : devices) {
    auto *ne2000 = new Ne2000(device);
    ne2000->plugin();
    networkService.registerNetworkDevice(ne2000, "eth");
  }
}

/**
 * Read MacAddress form PAR0 - PAR5 registers
 */
Util::Network::MacAddress Ne2000::getMacAddress() const {

  /** Go to P2, STOP and block Remote DMA */
  baseRegister.writeByte(COMMAND, RD2 | PS0 | STP);
  /** Read MAC from PAR0 - PAR5 */
  uint8_t buffer[6] = {
      baseRegister.readByte(P1_PAR0),
      baseRegister.readByte(P1_PAR1),
      baseRegister.readByte(P1_PAR2),
      baseRegister.readByte(P1_PAR3),
      baseRegister.readByte(P1_PAR4),
      baseRegister.readByte(P1_PAR5),
  };


  /** COMMAND: Start and complete Remote DMA */
  baseRegister.writeByte(COMMAND, STA | RD2);

  /** Receive Config Accept Broadcast and Accept Multicast */
  baseRegister.writeByte(P0_RCR, RCR_AB | RCR_AM);
  return Util::Network::MacAddress(buffer);
}

/**
 * ToDo Handle Interrupt trigger
 */
void Ne2000::trigger(const Kernel::InterruptFrame &frame) {
  /** Disable Interrupts */
  baseRegister.writeByte(P0_IMR, 0x00);

  auto interrupt = baseRegister.readByte(P0_ISR);
  log.info("Interrupt Trigger %02x", interrupt);
  /** Packet Received Interrupt */
  if (interrupt & ISR_PRX) {
    log.info("Process Received Packet");
    processReceivedPackets();
    /** Clear Interrupt */
    baseRegister.writeByte(P0_ISR, ISR_PRX);
  }
  /** ToDo Packet Transmit Interrupt */
  if (interrupt & ISR_PTX) {
    baseRegister.writeByte(P0_ISR, ISR_RDC);
  }


  /** Send EOI (EndOfInterrupt) and enable Interrupt*/
  baseRegister.writeByte(P0_IMR, IMR_PRXE | IMR_PTXE | IMR_RXEE | IMR_TXEE | IMR_OVWE | IMR_CNTE | IMR_RDCE);
}

/**
 * Called by Interrupt
 * https://github.com/torokernel/torokernel/blob/7d6df4c40fa4cc85febd5fd5799404592ffdff53/rtl/drivers/Ne2000.pas
 * Line 243 ff
 *
 */
void Ne2000::processReceivedPackets() {

  /** Go To P0, Start and Block Remote DMA -> 0x22 */
  baseRegister.writeByte(COMMAND, 0x22);
  auto current = baseRegister.readByte(P1_CURR);
  baseRegister.writeByte(COMMAND, 0x22);

  log.info("Process Packet");
  while (current not_eq nextPacket) {
    log.info("While entered");
    baseRegister.writeByte(P0_RBCR0, 4);
    baseRegister.writeByte(P0_RBCR1, 0);
    baseRegister.writeByte(P0_RSAR0, 0);
    baseRegister.writeByte(P0_RBCR1, 0x00);

    baseRegister.writeByte(COMMAND, STA | RD0);
    /** Read from I/O Port */
    auto rsr = baseRegister.readByte(0x10);
    auto next = baseRegister.readByte(0x10);
    uint32_t packet_length = baseRegister.readByte(0x10);
    packet_length = packet_length + (baseRegister.readByte(0x10) << 8);

    baseRegister.writeByte(P0_ISR, ISR_RDC);
    if (((rsr & 31) == 1) && (next >= PSTART) && (next <= PSTOP) &&
        (packet_length <= 1522)) {
      uint8_t buffer[(packet_length) * sizeof(uint8_t)];

      baseRegister.writeByte(P0_RBCR0, packet_length);
      baseRegister.writeByte(P0_RBCR1, packet_length << 8);

      baseRegister.writeByte(P0_RBCR0, 4);
      baseRegister.writeByte(P0_RBCR1, nextPacket);

      baseRegister.writeByte(COMMAND, STA | 0x0);
      log.info("Read Packet");
      for (int i = 0; i < packet_length; i++) {
        buffer[i] = (baseRegister.readByte(0x10));
      }

      /** Packet Received */
      baseRegister.writeByte(P0_ISR, 0x40);
      if (next == PSTOP)
        nextPacket = PSTART;
      else
        nextPacket = next;

      /** Call NetworkDevice handleIncomingPacket */
      handleIncomingPacket(buffer, packet_length);
    }
    if (nextPacket == PSTART)
      baseRegister.writeByte(P0_BNRY, PSTOP - 1);
    else
      baseRegister.writeByte(P0_BNRY, nextPacket - 1);

    /** Start, NODMA, 40 */
    baseRegister.writeByte(COMMAND, 0x42);
    current = baseRegister.readByte(P1_CURR);
    baseRegister.writeByte(COMMAND, 0x42);
  }
  log.info("Packets Processed");
}

/**
 * Check if TXP register equals 0
 * -> NIC is ready to transmit
 */
bool Ne2000::readyToTransmit() {
  auto status = baseRegister.readByte(COMMAND);
  log.info("Interrupt Trigger %02x", status);
  return (status & 0x22);
}

/**
 *
 * @param packet
 * @param length
 */
void Ne2000::handleOutgoingPacket(const uint8_t *packet, uint32_t length) {
  /** Check if Ne2000 is rdy to transmit */
  log.info("Call Send Packet");
  /*while (!readyToTransmit()) {
    Util::Async::Thread::sleep(Util::Time::Timestamp::ofMilliseconds(1));
  }*/
  log.info("Ready to transmit");

  sendPacketToNIC(packet, length);

  /*const auto &nextPck = getNextOutgoingPacket();
  if(nextPck.length not_eq 0x00 && nextPck.buffer not_eq nullptr){
    sendPacketToNIC(nextPck.buffer, nextPck.length);
    freePacketBuffer(nextPck.buffer);
  }*/

}

void Ne2000::sendPacketToNIC(const uint8_t *packet, uint32_t packet_length){
  //resetNe2000();

  // ToDO Check if needed (?)
  /*
  auto &memoryService = Kernel::System::getService<Kernel::MemoryService>();
  auto physicalAddress = memoryService.getPhysicalAddress(const_cast<uint8_t *>(packet));
  */
  // ToDo read https://datasheetspdf.com/pdf-file/549767/NationalSemiconductor/DP83901A/1
  /** Set Command Register to 0x22 */

  /** Set DCR to

  /** Patch read before write */

  /** Send Config */
  //baseRegister.writeByte(COMMAND, STA | RD2);
  log.info("Write PacketSize into RBCR0/1 %u", packet_length);
  /** Length & 0xFF write Byte 1+2 into RBCR0 */
  baseRegister.writeByte(P0_RBCR0, packet_length & 0xFF);
  /** Bitshift 8 to the right -> write Byte 3+4 into RBCR1 */
  baseRegister.writeByte(P0_RBCR1, packet_length >> 8);

  /*log.info("Clear remote DMA complete bit");
    baseRegister.writeByte(P0_ISR, ISR_RDC);
  */

  log.info("Load RSAR0/1 with low and target page number");
  baseRegister.writeByte(P0_RSAR0, 0x00);
  log.info("SendStartAddress %u", sendStartPage);
  baseRegister.writeByte(P0_RSAR1, sendStartPage);

  log.info("Set CR to start and Remote write DMA");
  //baseRegister.writeByte(COMMAND, RD1 | TXP | STA);
  baseRegister.writeByte(COMMAND, RD1 | STA);

  log.info("Write PacketData into DataPort");
  /** Write Packetdata to data port */
  for (uint32_t i = 0; i < packet_length; i++) {
    baseRegister.writeByte(0x10, packet[i]);
  }
  for(uint32_t i = 3;  i < packet_length-4; i = i+4){
    log.info("%02x %02x %02x %02x", packet[i-3], packet[i-2], packet[i-1], packet[i]);
  }
  /** Pad Packet if length < 64 Bytes */
  /*if(packet_length < 64){
    for(uint8_t i = 0; i < 64 - packet_length; i++){
      baseRegister.writeByte(0x10, 0x00);
    }
  }*/
  baseRegister.writeByte(P0_TPSR, 0x04);
  baseRegister.writeByte(P0_TBCR0, packet_length & 0xFF);
  baseRegister.writeByte(P0_TBCR1, packet_length >> 8);

  baseRegister.writeByte(COMMAND, RD2 | STA | TXP);

  log.info("Poll ISR until Bit 6 remote DMA complete is set");
  while ((baseRegister.readByte(P0_ISR) & ISR_RDC) == 0) {
    Util::Async::Thread::sleep(Util::Time::Timestamp::ofMilliseconds(1));
  }

  /** Handle and accept occurring interrupt */
  //baseRegister.writeByte(P0_ISR, ISR_RDC);

  /** Don't do Send Packet  -> crashes system */
  //baseRegister.writeByte(COMMAND, STA | RD2);
  log.info("Transmitted packet");
}

void Ne2000::plugin() {
  auto &interruptService = Kernel::System::getService<Kernel::InterruptService>();
  interruptService.allowHardwareInterrupt(pciDevice.getInterruptLine());
  interruptService.assignInterrupt(static_cast<Kernel::InterruptVector>(pciDevice.getInterruptLine() + 32), *this);
}
}