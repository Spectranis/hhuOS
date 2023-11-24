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
}

namespace Device::Network{
class Ne2000 : public NetworkDevice, Kernel::InterruptHandler {
    public:
        /**
         * Implement Methods of ../NetworkDevice.h
         */

        /**
         * Default Constructor
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
    protected:


    private:


    uint16_t COMMAND = 0x300;            /* R|W COMMAND used for P0, P1, P2 */
    uint16_t ISR     = COMMAND+0x7,      /* R|W Interrupt Status Register P0 */
    uint16_t BNRY    = COMMAND+0x3,      /* R|W Boundary Pointer  P0 */

    /*
         * Accessed during NIC operations
         * Gets called if:
         *  PS0 = 0, PS1 = 0
         */
        enum Page0write : uint16_t {
            PSTART = COMMAND+0x1       /* W Page Start Register  */
            PSTIO  = COMMAND+0x2,      /* W Page Stop Register  */
            TPSR   = COMMAND+0x4,      /* W Transmit Page Start Address  */
            TBCR0  = COMMAND+0x5,      /* W Transmit Byte Count Register 0  */
            TBCR1  = COMMAND+0x6,      /* W Transmit Byte Count Register 1  */
            RSAR0  = COMMAND+0x8,      /* W Remote Start Address Register 0 */
            RSAR1  = COMMAND+0x9,      /* W Remote Start Address Register 1 */
            RBCR0  = COMMAND+0xA,      /* W Remote Byte Count Register 0 */
            RBCR1  = COMMAND+0xB,      /* W Remote Byte Count Register 1 */
            RCR    = COMMAND+0xC,      /* W Receive Configuration Register */
            TCR    = COMMAND+0xD,      /* W Transmit Configuration Register*/
            DCR    = COMMAND+0xE,      /* W Data Configuration Register */
            IMR    = COMMAND+0xF       /* W Interrupt Mask Register */
        };

        enum Page0Read : uint16_t {
            CLDA0  = COMMAND+0x1,      /* R Current Local DMA Address 0  */
            CLDA1  = COMMAND+0x2,      /* R Current Local DMA Addresse 1  */
            TSR    = COMMAND+0x4,      /* R Transmit Status Register  */
            NCR    = COMMAND+0x5,      /* R Number of Collisions Register  */
            FIFO   = COMMAND+0x6,      /* R FIFO */
            CRDA0  = COMMAND+0x8,      /* R Current Remote DMA Address 0 */
            CRDA1  = COMMAND+0x9,      /* R Current Remote DMA Address 1 */
            /* R 0x0A and 0x0B are reserved */
            RSR    = COMMAND+0xC,      /* R Receive Status Register */
            CNTR0  = COMMAND+0xD,      /* R Tally Counter 0 (Frame Alignment Errors) */
            CNTR1  = COMMAND+0xE,      /* R Tally Counter 1 (CRC Errors) */
            CNTR2  = COMMAND+0xF       /* R Tally Counter 2 (Missed Packet Error) */
        };

        /*
         * Primarily used for Initialization
         * Gets called if:
         *  PS0 = 1, PS1 = 0
         */
        enum Page1 : uint16_t {
            PAR0   = COMMAND+0x1,      /* R|W Physical Address Register 0 */
            PAR1   = COMMAND+0x2,      /* R|W Physical Address Register 1 */
            PAR2   = COMMAND+0x3,      /* R|W Physical Address Register 2 */
            PAR3   = COMMAND+0x4,      /* R|W Physical Address Register 3 */
            PAR4   = COMMAND+0x5,      /* R|W Physical Address Register 4 */
            PAR5   = COMMAND+0x6,      /* R|W Physical Address Register 5 */
            CURR   = COMMAND+0x7,      /* R|W Current Page Register */
            MAR0   = COMMAND+0x8,      /* R|W Multicast Address Register 0 */
            MAR1   = COMMAND+0x9,      /* R|W Multicast Address Register 1 */
            MAR2   = COMMAND+0xA,      /* R|W Multicast Address Register 2 */
            MAR3   = COMMAND+0xB,      /* R|W Multicast Address Register 3 */
            MAR4   = COMMAND+0xC,      /* R|W Multicast Address Register 4 */
            MAR5   = COMMAND+0xD,      /* R|W Multicast Address Register 5 */
            MAR6   = COMMAND+0xE,      /* R|W Multicast Address Register 6 */
            MAR7   = COMMAND+0xF       /* R|W Multicast Address Register 7 */
        };

        /*
         * Register page2
         * Gets called if:
         *  PS0 = 0, PS1 = 1
         * ToDo document P2
         */
        enum Page2Write : uint16_t {
            CLDA0   = COMMAND+0x1,     /* W Current Local DMA Address 0 */
            CLDA1   = COMMAND+0x2,     /* W Current Local DMA Address 1 */
            RNPP    = COMMAND+0x3,     /* R|W Remote Next Packet Pointer */
            /* 0x04 W Reserved */
            LNPP    = COMMAND+0x5,     /* R|W Local Next Packet Pointer */
            UPPER   = COMMAND+0x6,     /* R|W Address Counter (Upper) */
            LOWER   = COMMAND+0x7,     /* R|W Address Counter (Lower) */
            /* 8 - B Reserved */
            /* W C - F Reserved */
        };

        enum Page2Read : uint16_t {
            PSTART  = COMMAND+0x1,     /* R Page Start Register */
            PSTOP   = COMMAND+0x2,     /* R Page Stop Register */
            TPSR    = COMMAND+0x4,     /* R Transmit Page Start Address */
            RCR     = COMMAND+0xC,     /* R Receive Configuration Register */
            TCR     = COMMAND+0xD,     /* R Transmit Configuration Register */
            DCR     = COMMAND+0xE,     /* R Data Configuration Register */
            IMR     = COMMAND+0xF,     /* R Interrupt Mask Register */
        };
        /*
         * Command Register
         */
        enum Command : uint8_t{
            STP = 0x0,          /* STOP */
            STA = 0x1,          /* START */
            TXP = 0x2,          /* Transmit Packet */
            RD0 = 0x3,          /* Remote DMA Command 0 */
            RD1 = 0x4,          /* Remote DMA Command 1 */
            RD2 = 0x5,          /* Remote DMA Command 2*/
            PS0 = 0x6,          /* Page Select PS0 */
            PS1 = 0x7           /* Page Select PS1 */
        };

        /*
         * Interrupt status register
         */
        enum InterruptStatusRegister : uint8_t{
            ISR_PRX = 0x0,          /* Packet Received */
            ISR_PTX = 0x1,          /* Packet Transmitted */
            ISR_RXE = 0x2,          /* Receive Error */
            ISR_TXE = 0x3,          /* Transmit Error */
            ISR_OVW = 0x4,          /* Overwrite Error */
            ISR_CNT = 0x5,          /* Counter Overflow */
            ISR_RDC = 0x6,          /* Remote DMA Complete */
            ISR_RST = 0x7           /* Reset Status*/
        };

        /*
         * Interrupt mask register
         */
        enum InterruptMaskRegister : uint8_t{
            IMR_PRXE = 0x0,         /* Packet Received Interrupt Enable */
            IMR_PTXE = 0x1,         /* Packet Transmit Interrupt Enable */
            IMR_RXEE = 0x2,         /* Receive Error Interrupt Enable */
            IMR_TXEE = 0x3,         /* Transmit Error Interrupt Enable */
            IMR_OVWE = 0x4,         /* Overwrite Warning Interrupt Enable */
            IMR_CNTE = 0x5,         /* Counter Overflow Interrupt Enable */
            IMR_RDCE = 0x6          /* DMA Complete Interrupt Enable */
            /* 0x7 reserved */
        };

        /*
         * Data configuration register
         */
        enum DataConfigurationRegister : uint8_t {
            DCR_WTS = 0x0,          /* Word Transfer Select */
            DCR_BOS = 0x1,          /* Byte Order Select */
            DCR_LAS = 0x2,          /* Long Address Select */
            DCR_LS  = 0x3,          /* Loop-back Select */
            DCR_AR  = 0x4,          /* Auto-Initialize Remote */
            DCR_FT0 = 0x5,          /* FIFO Threshold Select 0 */
            DCR_FT1 = 0x6,          /* FIFO Threshold Select 1 */
            /* 0x7 not defined */
        };

        enum TransmitConfigurationRegister : uint8_t {
            TCR_CRC  = 0x0,         /* Inhibit CRC */
            TCR_LB0  = 0x1,         /* Encoded Loop-back Control */
            TCR_LB1  = 0x2,         /* Encoded Loop-back Control */
            TCR_ATD  = 0x3,         /* Auto Transmit Disable */
            TCR_OFST = 0x4,         /* Collision Offset Enable */
            /* 5-7 Reserved */
        };

        enum TransmitStatusRegister : uint_8 {
            TSR_PTX = 0x0,          /* Packet Transmit */
            TSR_COL = 0x2,          /* Transmit Collided */
            TSR_ABT = 0x3,          /* Transmit Aborted */
            TSR_CRS = 0x4,          /* Carrier Sense Lost */
            TSR_FU  = 0x5,          /* FIFO Under-run */
            TSR_CDH = 0x6,          /* CD Heartbeat */
            TSR_OWC = 0x7           /* Out of Window Collision */
        };

        enum ReceiveConfigurationRegister : uint_8 {
            RCR_SEP = 0x0,          /* Save Errored Packets */
            RCR_AR  = 0x1,          /* Accept Runt Packets */
            RCR_AB  = 0x2,          /* Accept Broadcast */
            RCR_AM  = 0x3,          /* Accept Multicast */
            RCR_PRO = 0x4,          /* Promiscuous Physical */
            RCR_MON = 0x5,          /* Monitor Mode */
            /* 6 and 7 are reserved */
        };

        enum ReceiveStatusRegister : uint_8{
            RSR_PRX = 0x0,          /* Packet Received Intact */
            RSR_CRC = 0x1,          /* CRC Error */
            RSR_FAE = 0x2,          /* Frame Alignment Error */
            RSR_FO  = 0x3,          /* FIFO Overrun */
            RSR_MPA = 0x4,          /* Missed Packet*/
            RSR_PHY = 0x5,          /* Physical/Multicast Address */
            RSR_DIS = 0x6,          /* Receiver Disabled */
            RSR_DFR = 0x7           /* Deferring */
        };

        struct PacketHeader {// ToDo

        };

        bool isTransmitDescriptorAvailable();

        //ToDo
        void setTransmitAddress(void *buffer);

        void setPacketSize(uint32_t size);

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

        static Kernel::Logger ne2k_log; // ToDo

        static const constexpr uint16_t VENDOR_ID   = 0x0000; // ToDo
        static const constexpr uint16_t NIC_ID      = 0x0000; // ToDo
        static const constexpr uint32_t RBufferSize = 0x0000; // ToDo
        static const constexpr uint32_t TBufferSize = 0x0000; // ToDo
        int16_t MaxPacketSize = 256*6-4;
    };
}



#endif //HHUOS_NE2000_H
