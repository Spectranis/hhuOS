//
// Created by mthiel on 06.11.23.
//

#ifndef HHUOS_NE2000_H
#define HHUOS_NE2000_H

#include <cstdint>

#include "device/network/NetworkDevice.h"
#include "device/pci/PciDevice.h"
#include "kernel/interrupt/InterruptHandler.h"
#include "device/cpu/IoPort.h"
#include "lib/util/network/MacAddress.h"

namespace Kernel {
    class Logger;
    struct InterruptFrame;
}  // namespace Kernel

namespace Device::Network{

class Ne2000 : public NetworkDevice, Kernel::InterruptHandler {
public:
        /**
         * Implement Methods of ../NetworkDevice.h
         */

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

        /*+
         * Destructor
         */
        virtual ~Ne2000() override = default;

        /*
         * driverInitialize
         */
        static void initializeAvailableCards();

        /*
         * ToDo:
         * DiverInitializion
         * DriverSend
         * PCtoNIC
         * InterruptService
         * ReceivePackets
         */

        [[nodiscard]] Util::Network::MacAddress getMacAddress() const override;
        /*
         * ToDo
         */
        void plugin() override;
        /*
         * ToDo
         */
        void trigger(const Kernel::InterruptFrame &frame) override;
        void handleBufferOverflow();
        void handleIncomingPacket(const uint8_t *packet, uint32_t length);
        void handleOutgoingPacket(const uint8_t *packet, uint32_t length) override;
    protected:


    private:

    uint16_t COMMAND = 0x300;            /* R|W COMMAND used for P0, P1, P2 */
    uint16_t ISR     = 0x307;      /* R|W Interrupt Status Register P0 */
    uint16_t BNRY    = 0x303;     /* R|W Boundary Pointer  P0 */

    /*
         * Accessed during NIC operations
         * Gets called if:
         *  PS0 = 0, PS1 = 0
         */
        enum Page0write : uint16_t {
            PSTART = 0x301,      /* W Page Start Register  */
            PSTOP  = 0x302,      /* W Page Stop Register  */
            TPSR   = 0x304,      /* W Transmit Page Start Address  */
            TBCR0  = 0x305,      /* W Transmit Byte Count Register 0  */
            TBCR1  = 0x306,      /* W Transmit Byte Count Register 1  */
            RSAR0  = 0x308,      /* W Remote Start Address Register 0 */
            RSAR1  = 0x309,      /* W Remote Start Address Register 1 */
            RBCR0  = 0x30A,      /* W Remote Byte Count Register 0 */
            RBCR1  = 0x30B,      /* W Remote Byte Count Register 1 */
            RCR    = 0x30C,      /* W Receive Configuration Register */
            TCR    = 0x30D,      /* W Transmit Configuration Register*/
            DCR    = 0x30E,      /* W Data Configuration Register */
            IMR    = 0x30F       /* W Interrupt Mask Register */
        };

        enum Page0Read : uint16_t {
            CLDA0  = 0x301,      /* R Current Local DMA Address 0  */
            CLDA1  = 0x302,      /* R Current Local DMA Addresse 1  */
            TSR    = 0x304,      /* R Transmit Status Register  */
            NCR    = 0x305,      /* R Number of Collisions Register  */
            FIFO   = 0x306,      /* R FIFO */
            CRDA0  = 0x308,      /* R Current Remote DMA Address 0 */
            CRDA1  = 0x309,      /* R Current Remote DMA Address 1 */
            /* R 0x0A and 0x0B are reserved */
            RSR    = 0x30C,      /* R Receive Status Register */
            CNTR0  = 0x30D,      /* R Tally Counter 0 (Frame Alignment Errors) */
            CNTR1  = 0x30E,      /* R Tally Counter 1 (CRC Errors) */
            CNTR2  = 0x30F       /* R Tally Counter 2 (Missed Packet Error) */
        };

        /*
         * Primarily used for Initialization
         * Gets called if:
         *  PS0 = 1, PS1 = 0
         */
        enum Page1 : uint16_t {
            PAR0   = 0x301,      /* R|W Physical Address Register 0 */
            PAR1   = 0x302,      /* R|W Physical Address Register 1 */
            PAR2   = 0x303,      /* R|W Physical Address Register 2 */
            PAR3   = 0x304,      /* R|W Physical Address Register 3 */
            PAR4   = 0x305,      /* R|W Physical Address Register 4 */
            PAR5   = 0x306,      /* R|W Physical Address Register 5 */
            CURR   = 0x307,      /* R|W Current Page Register */
            MAR0   = 0x308,      /* R|W Multicast Address Register 0 */
            MAR1   = 0x309,      /* R|W Multicast Address Register 1 */
            MAR2   = 0x30A,      /* R|W Multicast Address Register 2 */
            MAR3   = 0x30B,      /* R|W Multicast Address Register 3 */
            MAR4   = 0x30C,      /* R|W Multicast Address Register 4 */
            MAR5   = 0x30D,      /* R|W Multicast Address Register 5 */
            MAR6   = 0x30E,      /* R|W Multicast Address Register 6 */
            MAR7   = 0x30F       /* R|W Multicast Address Register 7 */
        };

        /*
         * Register page2
         * Gets called if:
         *  PS0 = 0, PS1 = 1
         * ToDo document P2
         */
        enum Page2Write : uint16_t {
            P2_CLDA0    = 0x301,     /* W Current Local DMA Address 0 */
            P2_CLDA1    = 0x302,     /* W Current Local DMA Address 1 */
            RNPP        = 0x303,     /* R|W Remote Next Packet Pointer */
            /* 0x04 W Reserved */
            LNPP        = 0x305,     /* R|W Local Next Packet Pointer */
            UPPER       = 0x306,     /* R|W Address Counter (Upper) */
            LOWER       = 0x307,     /* R|W Address Counter (Lower) */
            /* 8 - B Reserved */
            /* W C - F Reserved */
        };

        enum Page2Read : uint16_t {
            P2_PSTART  = 0x301,     /* R Page Start Register */
            P2_PSTOP   = 0x302,     /* R Page Stop Register */
            P2_TPSR    = 0x304,     /* R Transmit Page Start Address */
            P2_RCR     = 0x30C,     /* R Receive Configuration Register */
            P2_TCR     = 0x30D,     /* R Transmit Configuration Register */
            P2_DCR     = 0x30E,     /* R Data Configuration Register */
            P2_IMR     = 0x30F,     /* R Interrupt Mask Register */
        };
        /*
         * Command Register
         */
        enum Command : uint8_t{
            STP = 0x01,          /* STOP */
            STA = 0x02,          /* START */
            TXP = 0x04,          /* Transmit Packet */
            RD0 = 0x08,          /* Remote DMA Command 0 */
            RD1 = 0x10,          /* Remote DMA Command 1 */
            RD2 = 0x20,          /* Remote DMA Command 2*/
            PS0 = 0x40,          /* Page Select PS0 */
            PS1 = 0x80           /* Page Select PS1 */
        };

        /*
         * Interrupt status register
         */
        enum InterruptStatusRegister : uint8_t{
            ISR_PRX = 0x01,          /* Packet Received */
            ISR_PTX = 0x02,          /* Packet Transmitted */
            ISR_RXE = 0x04,          /* Receive Error */
            ISR_TXE = 0x08,          /* Transmit Error */
            ISR_OVW = 0x10,          /* Overwrite Error */
            ISR_CNT = 0x20,          /* Counter Overflow */
            ISR_RDC = 0x40,          /* Remote DMA Complete */
            ISR_RST = 0x80           /* Reset Status*/
        };

        /*
         * Interrupt mask register
         */
        enum InterruptMaskRegister : uint8_t{
            IMR_PRXE = 0x01,         /* Packet Received Interrupt Enable */
            IMR_PTXE = 0x02,         /* Packet Transmit Interrupt Enable */
            IMR_RXEE = 0x04,         /* Receive Error Interrupt Enable */
            IMR_TXEE = 0x08,         /* Transmit Error Interrupt Enable */
            IMR_OVWE = 0x10,         /* Overwrite Warning Interrupt Enable */
            IMR_CNTE = 0x20,         /* Counter Overflow Interrupt Enable */
            IMR_RDCE = 0x40          /* DMA Complete Interrupt Enable */
            /* 0x7 reserved */
        };

        /*
         * Data configuration register
         */
        enum DataConfigurationRegister : uint8_t {
            DCR_WTS = 0x01,          /* Word Transfer Select */
            DCR_BOS = 0x02,          /* Byte Order Select */
            DCR_LAS = 0x04,          /* Long Address Select */
            DCR_LS  = 0x08,          /* Loop-back Select */
            DCR_AR  = 0x10,          /* Auto-Initialize Remote */
            DCR_FT0 = 0x20,          /* FIFO Threshold Select 0 */
            DCR_FT1 = 0x40,          /* FIFO Threshold Select 1 */
            /* 0x7 not defined */
        };

        enum TransmitConfigurationRegister : uint8_t {
            TCR_CRC  = 0x01,         /* Inhibit CRC */
            TCR_LB0  = 0x02,         /* Encoded Loop-back Control */
            TCR_LB1  = 0x04,         /* Encoded Loop-back Control */
            TCR_ATD  = 0x08,         /* Auto Transmit Disable */
            TCR_OFST = 0x10,         /* Collision Offset Enable */
            /* 5-7 Reserved */
        };

        enum TransmitStatusRegister : uint8_t {
            TSR_PTX = 0x01,          /* Packet Transmit */
            TSR_COL = 0x02,          /* Transmit Collided */
            TSR_ABT = 0x04,          /* Transmit Aborted */
            TSR_CRS = 0x08,          /* Carrier Sense Lost */
            TSR_FU  = 0x10,          /* FIFO Under-run */
            TSR_CDH = 0x20,          /* CD Heartbeat */
            TSR_OWC = 0x40           /* Out of Window Collision */
        };

        enum ReceiveConfigurationRegister : uint8_t {
            RCR_SEP = 0x01,          /* Save Errored Packets */
            RCR_AR  = 0x02,          /* Accept Runt Packets */
            RCR_AB  = 0x04,          /* Accept Broadcast */
            RCR_AM  = 0x08,          /* Accept Multicast */
            RCR_PRO = 0x10,          /* Promiscuous Physical */
            RCR_MON = 0x20,          /* Monitor Mode */
            /* 6 and 7 are reserved */
        };

        enum ReceiveStatusRegister : uint8_t {
            RSR_PRX = 0x01,          /* Packet Received Intact */
            RSR_CRC = 0x02,          /* CRC Error */
            RSR_FAE = 0x04,          /* Frame Alignment Error */
            RSR_FO  = 0x08,          /* FIFO Overrun */
            RSR_MPA = 0x10,          /* Missed Packet*/
            RSR_PHY = 0x20,          /* Physical/Multicast Address */
            RSR_DIS = 0x40,          /* Receiver Disabled */
            RSR_DFR = 0x80           /* Deferring */
        };

        struct PacketHeader {// ToDo

        };

        bool isTransmitDescriptorAvailable();

        //ToDo
        void setTransmitAddress(void *buffer);

        void setPacketSize(uint32_t size);

        void sendPacket(const uint8_t *packet, uint32_t length);

        void processReceivedPacket();

        PciDevice pciDevice;

        /*
         * ToDo
         * Pagesize 256 Byte
         * n pages
         */
        uint16_t *receiveBuffer{};
        uint16_t *transmitBuffer{};

        IoPort baseRegister = IoPort(0x00);

        static Kernel::Logger log; // ToDo

        /*
         * https://github.com/hisilicon/qemu/blob/master/hw/net/ne2000-pci.c
         * https://github.com/hisilicon/qemu/blob/master/hw/vfio/pci-quirks.c#L1006
         * #define PCI_VENDOR_ID_REALTEK 0x10ec
         */
        static const constexpr uint16_t VENDOR_ID   = 0x10ec; // ToDo
        /*
         * https://github.com/hisilicon/qemu/blob/master/include/hw/pci/pci.h#L48
         * #define PCI_DEVICE_ID_REALTEK_8029       0x8029
         */
        static const constexpr uint16_t DEVICE_ID   = 0x8029; // ToDo

        static const constexpr uint16_t NIC_ID      = 0x0000; // ToDo
        static const constexpr uint32_t RBufferSize = 0x0000; // ToDo
        static const constexpr uint32_t TBufferSize = 0x0000; // ToDo
        int16_t MaxPacketSize = 256*6-4;
    };
}



#endif //HHUOS_NE2000_H
