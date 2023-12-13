//
// Created by mthiel on 06.11.23.
//

#ifndef HHUOS_NE2000_H
#define HHUOS_NE2000_H

#include <cstdint>

#include "device/network/NetworkDevice.h"
#include "kernel/interrupt/InterruptHandler.h"
#include "device/cpu/IoPort.h"
#include "lib/util/network/MacAddress.h"
#include "device/pci/Pci.h"
#include "device/pci/PciDevice.h"
#include "kernel/system/System.h"
#include "kernel/service/NetworkService.h"
#include "lib/util/time/Timestamp.h"
#include "lib/util/async/Thread.h"
#include "kernel/service/MemoryService.h"
#include "kernel/service/InterruptService.h"
#include "kernel/log/Logger.h"
#include "lib/util/collection/Array.h"
#include "lib/util/base/Address.h"


namespace Kernel {
    class Logger;
    struct InterruptFrame;
}  // namespace Kernel

namespace Device::Network{

class Ne2000 : public NetworkDevice, Kernel::InterruptHandler {
public:

    /**
     * Default Constructor.
     */
    explicit Ne2000(const PciDevice &pciDevice);

    /**
     * Copy Constructor
     */
    Ne2000(const Ne2000 &other) = delete;

    /**
     * Assignment operator
     */
    Ne2000 &operator=(const Ne2000 &other) = delete;

    /**
     * Destructor
     */
    virtual ~Ne2000() override = default;

    /**
     * Initialize cards that are supported by this driver
     */
    static void initializeAvailableCards();

    [[nodiscard]] Util::Network::MacAddress getMacAddress() const override;

    void plugin() override;

    /**
     * Handle incoming Interrupts
     */
    void trigger(const Kernel::InterruptFrame &frame) override;

protected:
  void handleOutgoingPacket(const uint8_t *packet, uint32_t length) override;

  void sendPacket(const uint8_t *packet, uint32_t length);

  void processReceivedPacket();
  void readPacket();

  void resetNe2000();

private:

  /**
     * https://github.com/hisilicon/qemu/blob/master/hw/net/ne2000-pci.c
     * https://github.com/hisilicon/qemu/blob/master/hw/vfio/pci-quirks.c#L1006
     * #define PCI_VENDOR_ID_REALTEK 0x10ec
   */
  static const constexpr uint16_t VENDOR_ID   = 0x10ec;

  /**
     * https://github.com/hisilicon/qemu/blob/master/include/hw/pci/pci.h#L48
     * #define PCI_DEVICE_ID_REALTEK_8029       0x8029
   */
  static const constexpr uint16_t DEVICE_ID   = 0x8029;

  static const constexpr uint32_t RBufferSize = 0x0000; // ToDo
  static const constexpr uint32_t TBufferSize = 0x0000; // ToDo
  uint16_t MaxPacketSize = 256*6-4;

  uint16_t *receiveBuffer{};
  uint16_t *transmitBuffer{};
  PciDevice pciDevice;
  IoPort baseRegister = IoPort(0x00);

  static Kernel::Logger log; // ToDo


  /**
     * PageRegister as defined in DP8390D
     */
    enum PageRegisters : uint8_t {
        COMMAND     = 0x00,         /** R|W COMMAND used for P0, P1, P2 */
        /** P0 Write */
        P0_PSTART   = 0x01,         /** W Page Start Register  */
        P0_PSTOP    = 0x02,         /** W Page Stop Register  */
        P0_BNRY     = 0x03,         /** R|W Boundary Pointer  P0 */
        P0_TPSR     = 0x04,         /** W Transmit Page Start Address  */
        P0_TBCR0    = 0x05,         /** W Transmit Byte Count Register 0  */
        P0_TBCR1    = 0x06,         /** W Transmit Byte Count Register 1  */
        P0_ISR      = 0x07,         /** R|W Interrupt Status Register P0 */
        P0_RSAR0    = 0x08,         /** W Remote Start Address Register 0 */
        P0_RSAR1    = 0x09,         /** W Remote Start Address Register 1 */
        P0_RBCR0    = 0x0A,         /** W Remote Byte Count Register 0 */
        P0_RBCR1    = 0x0B,         /** W Remote Byte Count Register 1 */
        P0_RCR      = 0x0C,         /** W Receive Configuration Register */
        P0_TCR      = 0x0D,         /** W Transmit Configuration Register*/
        P0_DCR      = 0x0E,         /** W Data Configuration Register */
        P0_IMR      = 0x0F,         /** W Interrupt Mask Register */
        /** P0 Read*/
        P0_CLDA0    = 0x01,         /** R Current Local DMA Address 0  */
        P0_CLDA1    = 0x02,         /** R Current Local DMA Addresse 1  */
        P0_TSR      = 0x04,         /** R Transmit Status Register  */
        P0_NCR      = 0x05,         /** R Number of Collisions Register  */
        P0_FIFO     = 0x06,         /** R FIFO */
        P0_CRDA0    = 0x08,         /** R Current Remote DMA Address 0 */
        P0_CRDA1    = 0x09,         /** R Current Remote DMA Address 1 */
        /** R 0x0A and 0x0B are reserved */
        P0_RSR      = 0x0C,         /** R Receive Status Register */
        P0_CNTR0    = 0x0D,         /** R Tally Counter 0 (Frame Alignment Errors) */
        P0_CNTR1    = 0x0E,         /** R Tally Counter 1 (CRC Errors) */
        P0_CNTR2    = 0x0F,         /** R Tally Counter 2 (Missed Packet Error) */
        /** P1 */
        P1_PAR0     = 0x01,         /** R|W Physical Address Register 0 */
        P1_PAR1     = 0x02,         /** R|W Physical Address Register 1 */
        P1_PAR2     = 0x03,         /** R|W Physical Address Register 2 */
        P1_PAR3     = 0x04,         /** R|W Physical Address Register 3 */
        P1_PAR4     = 0x05,         /** R|W Physical Address Register 4 */
        P1_PAR5     = 0x06,         /** R|W Physical Address Register 5 */
        P1_CURR     = 0x07,         /** R|W Current Page Register */
        P1_MAR0     = 0x08,         /** R|W Multicast Address Register 0 */
        P1_MAR1     = 0x09,         /** R|W Multicast Address Register 1 */
        P1_MAR2     = 0x0A,         /** R|W Multicast Address Register 2 */
        P1_MAR3     = 0x0B,         /** R|W Multicast Address Register 3 */
        P1_MAR4     = 0x0C,         /** R|W Multicast Address Register 4 */
        P1_MAR5     = 0x0D,         /** R|W Multicast Address Register 5 */
        P1_MAR6     = 0x0E,         /** R|W Multicast Address Register 6 */
        P1_MAR7     = 0x0F,         /** R|W Multicast Address Register 7 */
        /** P2 Write */
        P2_CLDA0    = 0x01,         /** W Current Local DMA Address 0 */
        P2_CLDA1    = 0x02,         /** W Current Local DMA Address 1 */
        P2_RNPP     = 0x03,         /** R|W Remote Next Packet Pointer */
        /** 0x04 W Reserved */
        P2_LNPP     = 0x05,         /** R|W Local Next Packet Pointer */
        P2_UPPER    = 0x06,         /** R|W Address Counter (Upper) */
        P2_LOWER    = 0x07,         /** R|W Address Counter (Lower) */

        /** W C - F Reserved */
        /** P2 Read */
        P2_PSTART   = 0x01,         /** R Page Start Register */
        P2_PSTOP    = 0x02,         /** R Page Stop Register */
        P2_TPSR     = 0x04,         /** R Transmit Page Start Address */
        /** 8 - B Reserved */
        P2_RCR      = 0x0C,         /** R Receive Configuration Register */
        P2_TCR      = 0x0D,         /** R Transmit Configuration Register */
        P2_DCR      = 0x0E,         /** R Data Configuration Register */
        P2_IMR      = 0x0F          /** R Interrupt Mask Register */
    };

    /**
     * Command Register
     */
    enum CommandRegister : uint8_t{
        STP         = 0x01,         /** STOP */
        STA         = 0x02,         /** START */
        TXP         = 0x04,         /** Transmit Packet */
        RD0         = 0x08,         /** Remote DMA Command 0 */
        RD1         = 0x10,         /** Remote DMA Command 1 */
        RD2         = 0x20,         /** Remote DMA Command 2*/
        PS0         = 0x40,         /** Page Select PS0 */
        PS1         = 0x80          /** Page Select PS1 */
    };

    /**
     * Interrupt status register
     */
    enum InterruptStatusRegister : uint8_t{
        ISR_PRX     = 0x01,         /** Packet Received */
        ISR_PTX     = 0x02,         /** Packet Transmitted */
        ISR_RXE     = 0x04,         /** Receive Error */
        ISR_TXE     = 0x08,         /** Transmit Error */
        ISR_OVW     = 0x10,         /** Overwrite Error */
        ISR_CNT     = 0x20,         /** Counter Overflow */
        ISR_RDC     = 0x40,         /** Remote DMA Complete */
        ISR_RST     = 0x80          /** Reset Status*/
    };

    /**
     * Interrupt mask register
     */
    enum InterruptMaskRegister : uint8_t{
        IMR_PRXE    = 0x01,         /** Packet Received Interrupt Enable */
        IMR_PTXE    = 0x02,         /** Packet Transmit Interrupt Enable */
        IMR_RXEE    = 0x04,         /** Receive Error Interrupt Enable */
        IMR_TXEE    = 0x08,         /** Transmit Error Interrupt Enable */
        IMR_OVWE    = 0x10,         /** Overwrite Warning Interrupt Enable */
        IMR_CNTE    = 0x20,         /** Counter Overflow Interrupt Enable */
        IMR_RDCE    = 0x40          /** DMA Complete Interrupt Enable */
        /** 0x80 reserved */
    };

    /**
     * Data configuration register
     */
    enum DataConfigurationRegister : uint8_t {
        DCR_WTS     = 0x01,         /** Word Transfer Select */
        DCR_BOS     = 0x02,         /** Byte Order Select */
        DCR_LAS     = 0x04,         /** Long Address Select */
        DCR_LS      = 0x08,         /** Loop-back Select */
        DCR_AR      = 0x10,         /** Auto-Initialize Remote */
        DCR_FT0     = 0x20,         /** FIFO Threshold Select 0 */
        DCR_FT1     = 0x40,         /** FIFO Threshold Select 1 */
        /** 0x80 not defined */
    };

    /**
     * Transmit Configuration Register
     */
    enum TransmitConfigurationRegister : uint8_t {
        TCR_CRC     = 0x01,         /** Inhibit CRC */
        TCR_LB0     = 0x02,         /** Encoded Loop-back Control */
        TCR_LB1     = 0x04,         /** Encoded Loop-back Control */
        TCR_ATD     = 0x08,         /** Auto Transmit Disable */
        TCR_OFST    = 0x10,         /** Collision Offset Enable */
        /** 0x20, 0x40, 0x80  Reserved */
    };

    /**
     * Transmit Status Register
     */
    enum TransmitStatusRegister : uint8_t {
        TSR_PTX     = 0x01,         /** Packet Transmit */
        TSR_COL     = 0x02,         /** Transmit Collided */
        TSR_ABT     = 0x04,         /** Transmit Aborted */
        TSR_CRS     = 0x08,         /** Carrier Sense Lost */
        TSR_FU      = 0x10,         /** FIFO Under-run */
        TSR_CDH     = 0x20,         /** CD Heartbeat */
        TSR_OWC     = 0x40          /** Out of Window Collision */
    };

    /**
     * Receive Configuration Register
     */
    enum ReceiveConfigurationRegister : uint8_t {
        RCR_SEP     = 0x01,         /** Save Errored Packets */
        RCR_AR      = 0x02,         /** Accept Runt Packets */
        RCR_AB      = 0x04,         /** Accept Broadcast */
        RCR_AM      = 0x08,         /** Accept Multicast */
        RCR_PRO     = 0x10,         /** Promiscuous Physical */
        RCR_MON     = 0x20,         /** Monitor Mode */
        /** 0x40 and 0x80 are reserved */
    };

    /**
     * Receive Status Register
     */
    enum ReceiveStatusRegister : uint8_t {
        RSR_PRX     = 0x01,         /** Packet Received Intact */
        RSR_CRC     = 0x02,         /** CRC Error */
        RSR_FAE     = 0x04,         /** Frame Alignment Error */
        RSR_FO      = 0x08,         /** FIFO Overrun */
        RSR_MPA     = 0x10,         /** Missed Packet*/
        RSR_PHY     = 0x20,         /** Physical/Multicast Address */
        RSR_DIS     = 0x40,         /** Receiver Disabled */
        RSR_DFR     = 0x80          /** Deferring */
    };


    };
}

#endif //HHUOS_NE2000_H
