/*
 * Copyright (C) 2018-2022 Heinrich-Heine-Universitaet Duesseldorf,
 * Institute of Computer Science, Department Operating Systems
 * Burak Akguel, Christian Gesse, Fabian Ruhland, Filip Krakowski, Michael Schoettner
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */

#include <cstdint>

#include "device/bios/Bios.h"
#include "device/graphic/lfb/vesa/VesaBiosExtensions.h"
#include "kernel/multiboot/MultibootLinearFrameBufferProvider.h"
#include "device/graphic/terminal/lfb/LinearFrameBufferTerminalProvider.h"
#include "device/graphic/terminal/cga/ColorGraphicsAdapterProvider.h"
#include "lib/util/reflection/InstanceFactory.h"
#include "kernel/system/System.h"
#include "kernel/multiboot/Multiboot.h"
#include "kernel/multiboot/MultibootTerminalProvider.h"
#include "device/hid/Keyboard.h"
#include "lib/util/stream/InputStreamReader.h"
#include "lib/util/file/tar/Archive.h"
#include "filesystem/tar/ArchiveDriver.h"
#include "lib/util/file/File.h"
#include "lib/util/stream/BufferedReader.h"
#include "kernel/service/FilesystemService.h"
#include "filesystem/memory/MemoryDriver.h"
#include "lib/util/stream/FileInputStream.h"
#include "lib/util/cpu/CpuId.h"
#include "device/port/parallel/ParallelPort.h"
#include "lib/util/stream/FileOutputStream.h"
#include "lib/util/async/Process.h"
#include "kernel/service/MemoryService.h"
#include "kernel/service/SchedulerService.h"
#include "kernel/memory/MemoryStatusNode.h"
#include "device/power/apm/ApmMachine.h"
#include "kernel/service/PowerManagementService.h"
#include "device/pci/Pci.h"
#include "device/storage/floppy/FloppyController.h"
#include "device/storage/ide/IdeController.h"
#include "kernel/service/StorageService.h"
#include "filesystem/fat/FatDriver.h"
#include "device/sound/speaker/PcSpeakerNode.h"
#include "filesystem/memory/NullNode.h"
#include "filesystem/memory/ZeroNode.h"
#include "filesystem/memory/RandomNode.h"
#include "filesystem/process/ProcessDriver.h"
#include "device/hid/Mouse.h"
#include "device/hid/Ps2Controller.h"
#include "lib/util/stream/FileReader.h"
#include "filesystem/memory/MountsNode.h"
#include "device/debug/FirmwareConfiguration.h"
#include "filesystem/qemu/FirmwareConfigurationDriver.h"
#include "device/network/loopback/Loopback.h"
#include "kernel/service/NetworkService.h"
#include "BuildConfig.h"
#include "GatesOfHell.h"
#include "kernel/service/ProcessService.h"
#include "lib/util/async/FunctionPointerRunnable.h"
#include "lib/util/async/Thread.h"
#include "device/power/acpi/Acpi.h"
#include "network/ip4/Ip4Datagram.h"
#include "network/ethernet/EthernetDatagram.h"
#include "network/NumberUtil.h"
#include "network/icmp/IcmpSocket.h"
#include "network/icmp/IcmpDatagram.h"
#include "asm_interface.h"
#include "device/graphic/lfb/LinearFrameBufferProvider.h"
#include "device/graphic/terminal/TerminalProvider.h"
#include "device/port/serial/SerialPort.h"
#include "device/power/default/DefaultMachine.h"
#include "filesystem/core/Filesystem.h"
#include "kernel/log/Logger.h"
#include "kernel/process/Thread.h"
#include "lib/util/Exception.h"
#include "lib/util/data/Array.h"
#include "lib/util/memory/String.h"
#include "lib/util/stream/ByteArrayOutputStream.h"
#include "lib/util/stream/PrintWriter.h"
#include "lib/util/system/System.h"
#include "lib/util/time/Timestamp.h"
#include "network/Datagram.h"
#include "network/MacAddress.h"
#include "network/NetworkAddress.h"
#include "network/NetworkStack.h"
#include "network/ethernet/EthernetHeader.h"
#include "network/ethernet/EthernetSocket.h"
#include "network/icmp/EchoHeader.h"
#include "network/icmp/IcmpHeader.h"
#include "network/ip4/Ip4Address.h"
#include "network/ip4/Ip4Header.h"
#include "network/ip4/Ip4Module.h"
#include "network/ip4/Ip4NetworkMask.h"
#include "network/ip4/Ip4PortAddress.h"
#include "network/ip4/Ip4Route.h"
#include "network/ip4/Ip4Socket.h"
#include "network/udp/UdpDatagram.h"
#include "network/udp/UdpSocket.h"

namespace Device {
class Machine;
}  // namespace Device

Kernel::Logger GatesOfHell::log = Kernel::Logger::get("GatesOfHell");

void GatesOfHell::enter() {
    const auto logLevel = Kernel::Multiboot::hasKernelOption("log_level") ? Kernel::Multiboot::getKernelOption("log_level") : "info";
    Kernel::Logger::setLevel(logLevel);

    auto &bootloaderCopyInformation = Kernel::Multiboot::getCopyInformation();
    if (!bootloaderCopyInformation.success) {
        log.error("Bootloader information has not been copied successfully -> Undefined behaviour may occur...");
    }
    log.info("Bootloader: [%s], Multiboot info size: [%u/%u Byte]", static_cast<const char*>(Kernel::Multiboot::getBootloaderName()), bootloaderCopyInformation.copiedBytes, bootloaderCopyInformation.targetAreaSize);
    log.info("%u MiB of physical memory detected", Kernel::System::getService<Kernel::MemoryService>().getMemoryStatus().totalPhysicalMemory / 1024 / 1024);

    if (Util::Cpu::CpuId::isAvailable()) {
        log.info("CPU vendor: %s", static_cast<const char*>(Util::Cpu::CpuId::getVendorString()));

        const auto info = Util::Cpu::CpuId::getCpuInfo();
        log.info("CPU info: Family [%u], Model [%u], Stepping [%u], type [%u]", info.family, info.model, info.stepping, info.type);

        const auto features = Util::Cpu::CpuId::getCpuFeatures();
        Util::Memory::String featureString;
        for (uint32_t i = 0; i < features.length(); i++) {
            featureString += Util::Cpu::CpuId::getFeatureAsString(features[i]);
            if (i < features.length() - 1) {
                featureString += ",";
            }
        }
        log.info("CPU features: %s", static_cast<const char*>(featureString));
    }

    if (Device::Acpi::isAvailable()) {
        const auto &acpiCopyInformation = Device::Acpi::getCopyInformation();
        log.info("ACPI support detected (Table size: [%u/%u Byte])", acpiCopyInformation.copiedBytes, acpiCopyInformation.targetAreaSize);

        const auto &rsdp = Device::Acpi::getRsdp();
        const auto vendor = Util::Memory::String(reinterpret_cast<const uint8_t*>(rsdp.oemId), sizeof(Device::Acpi::Rsdp::oemId));
        log.info("ACPI vendor: [%s], ACPI version: [%s]", static_cast<const char*>(vendor), rsdp.revision == 0 ? "1.0" : ">=2.0");

        const auto tables = Device::Acpi::getAvailableTables();
        Util::Memory::String featureString;
        for (uint32_t i = 0; i < tables.length(); i++) {
            featureString += tables[i];
            if (i < tables.length() - 1) {
                featureString += ",";
            }
        }
        log.info("ACPI tables: %s", static_cast<const char*>(featureString));
    }

    if (Device::Bios::isAvailable()) {
        log.info("BIOS detected");
        Device::Bios::init();
    }

    Device::Pci::scan();

    initializeStorage();

    initializeFilesystem();

    initializePorts();

    initializeTerminal();

    initializePs2Devices();

    initializeNetwork();

    initializePowerManagement();

    Kernel::Logger::addOutputStream(*new Util::Stream::FileOutputStream("/device/log"));
    enablePortLogging();

    mountDevices();

    printBanner();

    /*auto &ethernetThread = Kernel::Thread::createKernelThread("Ethernet-Test", Kernel::System::getService<Kernel::ProcessService>().getKernelProcess(), new Util::Async::FunctionPointerRunnable([](){
        uint8_t addressBuffer[6] = {0, 0, 0, 0, 0, 0};
        auto senderAddress = Network::MacAddress(addressBuffer);
        auto receiverAddress = Network::MacAddress(addressBuffer);
        auto datagram = Network::Ethernet::EthernetDatagram(reinterpret_cast<const uint8_t*>("Hello, World!"), 14, receiverAddress, static_cast<Network::Ethernet::EthernetHeader::EtherType>(0));

        auto sender = Network::Ethernet::EthernetSocket();
        sender.bind(senderAddress);
        auto receiver = Network::Ethernet::EthernetSocket();
        receiver.bind(receiverAddress);

        auto logger = Kernel::Logger::get("Ethernet-Test");
        for (uint32_t i = 0; i < 10; i++) {
            sender.send(datagram);
            auto *receivedDatagram = receiver.receive();
            log.info("Received datagram from [%s]: '%s'", static_cast<const char*>(receivedDatagram->getRemoteAddress().toString()), receivedDatagram->getData());
            delete receivedDatagram;
            Util::Async::Thread::sleep(Util::Time::Timestamp(1, 0));
        }
    }));
    Kernel::System::getService<Kernel::SchedulerService>().ready(ethernetThread);*/

    /*auto &ip4Thread = Kernel::Thread::createKernelThread("IP4-Test", Kernel::System::getService<Kernel::ProcessService>().getKernelProcess(), new Util::Async::FunctionPointerRunnable([](){
        auto senderAddress = Network::Ip4::Ip4Address("127.0.0.1");
        auto receiverAddress = Network::Ip4::Ip4Address("127.0.0.1");
        auto datagram = Network::Ip4::Ip4Datagram(reinterpret_cast<const uint8_t*>("Hello, World!"), 14, receiverAddress, static_cast<Network::Ip4::Ip4Header::Protocol>(0));

        auto sender = Network::Ip4::Ip4Socket();
        sender.bind(senderAddress);
        auto receiver = Network::Ip4::Ip4Socket();
        receiver.bind(receiverAddress);

        auto logger = Kernel::Logger::get("IP4-Test");
        for (uint32_t i = 0; i < 10; i++) {
            sender.send(datagram);
            auto *receivedDatagram = receiver.receive();
            log.info("Received datagram from [%s]: '%s'", static_cast<const char*>(receivedDatagram->getRemoteAddress().toString()), receivedDatagram->getData());
            delete receivedDatagram;
            Util::Async::Thread::sleep(Util::Time::Timestamp(1, 0));
        }
    }));
    Kernel::System::getService<Kernel::SchedulerService>().ready(ip4Thread);*/

    /*auto &udpThread = Kernel::Thread::createKernelThread("UDP-Test", Kernel::System::getService<Kernel::ProcessService>().getKernelProcess(), new Util::Async::FunctionPointerRunnable([](){
        auto senderAddress = Network::Ip4::Ip4PortAddress(Network::Ip4::Ip4Address("127.0.0.1"), 1797);
        auto receiverAddress = Network::Ip4::Ip4PortAddress(Network::Ip4::Ip4Address("127.0.0.1"), 8821);
        auto datagram = Network::Udp::UdpDatagram(reinterpret_cast<const uint8_t*>("Hello, World!"), 14, receiverAddress);

        auto sender = Network::Udp::UdpSocket();
        sender.bind(senderAddress);
        auto receiver = Network::Udp::UdpSocket();
        receiver.bind(receiverAddress);

        auto logger = Kernel::Logger::get("UDP-Test");
        for (uint32_t i = 0; i < 10; i++) {
            sender.send(datagram);
            auto *receivedDatagram = receiver.receive();
            log.info("Received datagram from [%s]: '%s'", static_cast<const char*>(receivedDatagram->getRemoteAddress().toString()), receivedDatagram->getData());
            delete receivedDatagram;
            Util::Async::Thread::sleep(Util::Time::Timestamp(1, 0));
        }
    }));
    Kernel::System::getService<Kernel::SchedulerService>().ready(udpThread);*/

    /*auto &pingThread = Kernel::Thread::createKernelThread("Ping-Test", Kernel::System::getService<Kernel::ProcessService>().getKernelProcess(), new Util::Async::FunctionPointerRunnable([](){
        auto senderAddress = Network::Ip4::Ip4Address("127.0.0.1");
        auto receiverAddress = Network::Ip4::Ip4Address("127.0.0.1");

        auto echoHeader = Network::Icmp::EchoHeader();

        auto sender = Network::Icmp::IcmpSocket();
        sender.bind(senderAddress);
        auto receiver = Network::Icmp::IcmpSocket();
        receiver.bind(receiverAddress);

        auto logger = Kernel::Logger::get("Ping-Test");
        for (uint32_t i = 0; i < 10; i++) {
            bool validReply = false;
            auto packet = Util::Stream::ByteArrayOutputStream();
            echoHeader.setIdentifier(0);
            echoHeader.setSequenceNumber(i);
            echoHeader.write(packet);
            Network::NumberUtil::writeUnsigned32BitValue(Util::Time::getSystemTime().toMilliseconds(), packet);

            auto datagram = Network::Icmp::IcmpDatagram(packet.getBuffer(), packet.getPosition(), receiverAddress, Network::Icmp::IcmpHeader::ECHO_REQUEST, 0);
            sender.send(datagram);

            do {
                auto *receivedDatagram = reinterpret_cast<Network::Icmp::IcmpDatagram*>(receiver.receive());
                if (receivedDatagram->getType() == Network::Icmp::IcmpHeader::ECHO_REPLY) {
                    echoHeader.read(*receivedDatagram);
                    if (echoHeader.getSequenceNumber() == i) {
                        validReply = true;
                        auto sourceTimestamp = Network::NumberUtil::readUnsigned32BitValue(*receivedDatagram);
                        auto currentTimestamp = Util::Time::getSystemTime().toMilliseconds();
                        log.info("[%u bytes] from [%s] (Sequence number: [%u], Time: [%u ms])", receivedDatagram->getDataLength(),
                                 static_cast<const char *>(receivedDatagram->getRemoteAddress().toString()),
                                 echoHeader.getSequenceNumber(), currentTimestamp - sourceTimestamp);
                    }
                }
                delete receivedDatagram;
            } while (!validReply);

            Util::Async::Thread::sleep(Util::Time::Timestamp(1, 0));
        }
    }));
    Kernel::System::getService<Kernel::SchedulerService>().ready(pingThread);*/

    Util::Async::Process::execute(Util::File::File("/initrd/bin/shell"), Util::File::File("/device/terminal"), Util::File::File("/device/terminal"), Util::File::File("/device/terminal"), "shell", Util::Data::Array<Util::Memory::String>(0));

    log.info("Starting scheduler!");
    Kernel::System::getService<Kernel::SchedulerService>().startScheduler();

    Util::Exception::throwException(Util::Exception::ILLEGAL_STATE, "Once you entered the gates of hell, you are not allowed to leave!");
}

void GatesOfHell::initializeTerminal() {
    log.info("Initializing graphical terminal");

    if (Device::Graphic::VesaBiosExtensions::isAvailable()) {
        log.info("VESA graphics detected");
        Util::Reflection::InstanceFactory::registerPrototype(new Device::Graphic::VesaBiosExtensions(true));
    }

    if (Device::Graphic::ColorGraphicsAdapterProvider::isAvailable()) {
        log.info("CGA graphics detected");
        Util::Reflection::InstanceFactory::registerPrototype(new Device::Graphic::ColorGraphicsAdapterProvider(true));
    }

    Device::Graphic::LinearFrameBufferProvider *lfbProvider = nullptr;
    Device::Graphic::TerminalProvider *terminalProvider;

    if (Kernel::Multiboot::hasKernelOption("lfb_provider")) {
        auto providerName = Kernel::Multiboot::getKernelOption("lfb_provider");
        log.info("LFB provider set to [%s] -> Starting initialization", static_cast<const char*>(providerName));
        lfbProvider = reinterpret_cast<Device::Graphic::LinearFrameBufferProvider*>(Util::Reflection::InstanceFactory::createInstance(providerName));
    } else if (Kernel::MultibootLinearFrameBufferProvider::isAvailable()) {
        log.info("LFB provider is not set -> Using with multiboot values");
        lfbProvider = new Kernel::MultibootLinearFrameBufferProvider();
    }

    if (lfbProvider != nullptr) {
        auto mode = lfbProvider->searchMode(800, 600, 32);
        lfbProvider->initializeLinearFrameBuffer(mode, "lfb");
    }

    if (Kernel::Multiboot::hasKernelOption("terminal_provider")) {
        auto providerName = Kernel::Multiboot::getKernelOption("terminal_provider");
        log.info("Terminal provider set to [%s] -> Starting initialization", static_cast<const char*>(providerName));
        terminalProvider = reinterpret_cast<Device::Graphic::TerminalProvider*>(Util::Reflection::InstanceFactory::createInstance(providerName));
    } else if (lfbProvider != nullptr) {
        log.info("Terminal provider is not set -> Using LFB terminal");
        auto lfbFile = Util::File::File("/device/lfb");
        terminalProvider = new Device::Graphic::LinearFrameBufferTerminalProvider(lfbFile);
    }  else if (Kernel::MultibootTerminalProvider::isAvailable()) {
        log.info("Terminal provider is not set and LFB is not available -> Using multiboot values");
        terminalProvider = new Kernel::MultibootTerminalProvider();
    } else {
        Util::Exception::throwException(Util::Exception::ILLEGAL_STATE, "Unable to find a suitable graphics driver for this machine!");
    }

    auto resolution = terminalProvider->searchMode(100, 37, 24);
    terminalProvider->initializeTerminal(resolution, "terminal");

    delete terminalProvider;
    delete lfbProvider;

    // Open first file descriptors for Util::System::in, Util::System::out and Util::System::error
    Util::File::open("/device/terminal");
    Util::File::open("/device/terminal");
    Util::File::open("/device/terminal");
}

void GatesOfHell::enablePortLogging() {
    if (!Kernel::Multiboot::hasKernelOption("log_ports")) {
        return;
    }

    const auto ports = Kernel::Multiboot::getKernelOption("log_ports").split(",");
    for (const auto &port : ports) {
        auto file = Util::File::File("/device/" + port.toLowerCase());
        if (!file.exists()) {
            log.error("Port [%s] not present", static_cast<const char*>(port));
            return;
        }

        auto *stream = new Util::Stream::FileOutputStream(file);
        Kernel::Logger::addOutputStream(*stream);
    }
}

void GatesOfHell::initializeFilesystem() {
    log.info("Initializing filesystem");
    Kernel::System::registerService(Kernel::FilesystemService::SERVICE_ID, new Kernel::FilesystemService());
    auto &filesystemService = Kernel::System::getService<Kernel::FilesystemService>();
    auto &storageService = Kernel::System::getService<Kernel::StorageService>();

    Util::Reflection::InstanceFactory::registerPrototype(new Filesystem::Fat::FatDriver());

    bool rootMounted = false;
    if (Kernel::Multiboot::hasKernelOption("root")) {
        auto rootOptions = Kernel::Multiboot::getKernelOption("root").split(",");
        if (rootOptions.length() >= 2) {
            const auto &deviceName = rootOptions[0];
            const auto &driverName = rootOptions[1];

            if (storageService.isDeviceRegistered(deviceName)) {
                log.info("Mounting [%s] to root using driver [%s]", static_cast<const char*>(deviceName), static_cast<const char*>(driverName));
                rootMounted = filesystemService.mount(deviceName, "/", driverName);
                if (!rootMounted) {
                    log.error("Failed to mount root filesystem");
                }
            } else {
                log.error("Device [%s] is not available", static_cast<const char*>(deviceName));
            }
        } else {
            log.error("Invalid options for root filesystem given");
        }
    }

    if (!rootMounted) {
        log.info("Mounting virtual filesystem as root filesystem");
        auto *rootDriver = new Filesystem::Memory::MemoryDriver();
        filesystemService.getFilesystem().mountVirtualDriver("/", rootDriver);
    }

    auto *deviceDriver = new Filesystem::Memory::MemoryDriver();
    filesystemService.createDirectory("/device");
    filesystemService.getFilesystem().mountVirtualDriver("/device", deviceDriver);

    auto *processDriver = new Filesystem::Process::ProcessDriver();
    filesystemService.createDirectory("/process");
    filesystemService.getFilesystem().mountVirtualDriver("/process", processDriver);

    filesystemService.createFile("/device/log");
    deviceDriver->addNode("/", new Filesystem::Memory::NullNode());
    deviceDriver->addNode("/", new Filesystem::Memory::ZeroNode());
    deviceDriver->addNode("/", new Filesystem::Memory::RandomNode());
    deviceDriver->addNode("/", new Filesystem::Memory::MountsNode());
    deviceDriver->addNode("/", new Kernel::MemoryStatusNode("memory"));
    deviceDriver->addNode("/", new Device::Sound::PcSpeakerNode("speaker"));

    if (Kernel::Multiboot::isModuleLoaded("initrd")) {
        log.info("Initial ramdisk detected -> Mounting [%s]", "/initrd");
        auto module = Kernel::Multiboot::getModule("initrd");
        auto *tarArchive = new Util::File::Tar::Archive(module.start);
        auto *tarDriver = new Filesystem::Tar::ArchiveDriver(*tarArchive);

        filesystemService.createDirectory("/initrd");
        filesystemService.getFilesystem().mountVirtualDriver("/initrd", tarDriver);
    }

    if (Device::FirmwareConfiguration::isAvailable()) {
        auto *fwCfg = new Device::FirmwareConfiguration();
        auto *qemuDriver = new Filesystem::Qemu::FirmwareConfigurationDriver(*fwCfg);
        filesystemService.createDirectory("/qemu");
        filesystemService.getFilesystem().mountVirtualDriver("/qemu", qemuDriver);
    }
}

void GatesOfHell::initializePs2Devices() {
    auto *ps2Controller = Device::Ps2Controller::initialize();
    auto *keyboard = Device::Keyboard::initialize(*ps2Controller);
    auto *mouse = Device::Mouse::initialize(*ps2Controller);

    if (keyboard == nullptr) {
        // Register a null node as keyboard, so that the system can at least still boot up
        auto *node = new Filesystem::Memory::NullNode("keyboard");
        auto &filesystem = Kernel::System::getService<Kernel::FilesystemService>().getFilesystem();
        auto &driver = filesystem.getVirtualDriver("/device");
        driver.addNode("/", node);
    } else {
        keyboard->plugin();
    }

    if (mouse != nullptr) {
        mouse->plugin();
    }
}

void GatesOfHell::initializePorts() {
    Device::SerialPort::initializeAvailablePorts();
    Device::ParallelPort::initializeAvailablePorts();
}

void GatesOfHell::printBanner() {
    auto bannerFile = Util::File::File("/initrd/banner.txt");
    if (bannerFile.exists()) {
        auto bannerStream = Util::Stream::FileInputStream(bannerFile);
        auto bannerReader = Util::Stream::InputStreamReader(bannerStream);
        auto bufferedReader = Util::Stream::BufferedReader(bannerReader);

        auto banner = bufferedReader.read(bannerFile.getLength());
        Util::System::out << Util::Memory::String::format(static_cast<const char*>(banner),
                                               BuildConfig::getVersion(),
                                               BuildConfig::getBuildDate(),
                                               BuildConfig::getGitBranch(),
                                               BuildConfig::getGitRevision()) << Util::Stream::PrintWriter::endl << Util::Stream::PrintWriter::flush;
    } else {
        printDefaultBanner();
    }
}

void GatesOfHell::printDefaultBanner() {
    Util::System::out << "Welcome to hhuOS!" << Util::Stream::PrintWriter::endl
           << "Version: " << BuildConfig::getVersion() << " (" << BuildConfig::getGitBranch() << ")" << Util::Stream::PrintWriter::endl
           << "Git revision: " << BuildConfig::getGitRevision() << Util::Stream::PrintWriter::endl
           << "Build date: " << BuildConfig::getBuildDate() << Util::Stream::PrintWriter::endl << Util::Stream::PrintWriter::endl << Util::Stream::PrintWriter::flush;
}

void GatesOfHell::initializePowerManagement() {
    Device::Machine *machine;
    if (Device::ApmMachine::isAvailable()) {
        log.info("APM is available");
        machine = new Device::ApmMachine();
    } else {
        machine = new Device::DefaultMachine();
    }

    auto *powerManagementService = new Kernel::PowerManagementService(machine);
    Kernel::System::registerService(Kernel::PowerManagementService::SERVICE_ID, powerManagementService);
}

void GatesOfHell::initializeStorage() {
    Device::Storage::IdeController::initializeAvailableControllers();

    if (Device::Storage::FloppyController::isAvailable()) {
        auto *floppyController = new Device::Storage::FloppyController();
        floppyController->initializeAvailableDrives();
    }
}

void GatesOfHell::initializeNetwork() {
    Kernel::System::registerService(Kernel::NetworkService::SERVICE_ID, new Kernel::NetworkService());
    auto &networkService = Kernel::System::getService<Kernel::NetworkService>();
    auto *loopback = new Device::Network::Loopback("loopback");
    networkService.registerNetworkDevice(loopback);
    networkService.getNetworkStack().getIp4Module().registerInterface(Network::Ip4::Ip4Address("127.0.0.1"), Network::Ip4::Ip4Address("127.0.0.0"), Network::Ip4::Ip4NetworkMask(8), *loopback);
    networkService.setDefaultRoute(Network::Ip4::Ip4Route(Network::Ip4::Ip4Address("127.0.0.1"), Network::Ip4::Ip4NetworkMask(8), loopback->getIdentifier()));

    /*auto *echoRequest = "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x08\x00\x45\x00" \
                        "\x00\x54\x5c\x09\x40\x00\x40\x01\xe0\x9d\x7f\x00\x00\x01\x7f\x00" \
                        "\x00\x01\x08\x00\xea\x85\x00\x02\x00\x01\xba\xa6\x7f\x63\x00\x00" \
                        "\x00\x00\x0d\x9a\x07\x00\x00\x00\x00\x00\x10\x11\x12\x13\x14\x15" \
                        "\x16\x17\x18\x19\x1a\x1b\x1c\x1d\x1e\x1f\x20\x21\x22\x23\x24\x25" \
                        "\x26\x27\x28\x29\x2a\x2b\x2c\x2d\x2e\x2f\x30\x31\x32\x33\x34\x35" \
                        "\x36\x37\x00\x00\x00\x00";*/

    /*auto *dnsRequest = "\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x08\x00\x45\x00" \
                    "\x00\x4f\x37\x5e\x00\x00\x40\x11\x45\x3e\x7f\x00\x00\x01\x7f\x00" \
                    "\x00\x01\xee\x17\x00\x35\x00\x3b\x4b\x5f\xfb\x45\x01\x20\x00\x01" \
                    "\x00\x00\x00\x00\x00\x01\x06\x67\x6f\x6f\x67\x6c\x65\x03\x63\x6f" \
                    "\x6d\x00\x00\x01\x00\x01\x00\x00\x29\x04\xd0\x00\x00\x00\x00\x00" \
                    "\x0c\x00\x0a\x00\x08\xa5\xa8\xee\x49\x1c\x30\xf4\x7e\x00\x00\x00" \
                    "\x00";*/


    /*auto *udpTest = "\xa0\x36\x9f\x6b\xc0\x99\x90\x1b\x0e\x9a\xc7\x07\x08\x00\x45\x00" \
                    "\x00\x1e\x00\x01\x00\x00\x40\x11\xf9\x40\xc0\xa8\x00\x1f\xc0\xa8" \
                    "\x00\x1e\x00\x14\x00\x0a\x00\x0a\x35\xc5\x48\x69\x00\x00\x00\x00";*/

    // loopback->sendPacket(reinterpret_cast<const uint8_t*>(udpTest), 48);

    /* auto *content = "Hello, World!";
    Network::Udp::UdpModule::writePacket(1797, 8821, Network::Ip4::Ip4Address("127.0.0.1"), reinterpret_cast<const uint8_t*>(content), 14);*/
}

void GatesOfHell::mountDevices() {
    auto mountFile = Util::File::File("/system/mount_table");
    if (!mountFile.exists()) {
        return;
    }

    auto &filesystemService = Kernel::System::getService<Kernel::FilesystemService>();
    auto fileReader = Util::Stream::FileReader(mountFile);
    auto reader = Util::Stream::BufferedReader(fileReader);

    Util::Memory::String line = reader.readLine();
    while (!line.isEmpty()) {
        if (line.beginsWith("#")) {
            line = reader.readLine();
            continue;
        }

        auto split = line.split(" ");
        if (split.length() < 3) {
            log.error("Invalid line in /system/mount_table");
            line = reader.readLine();
            continue;
        }

        auto success = filesystemService.mount(split[0], split[1], split[2]);
        if (!success) {
            log.error("Failed to mount [%s] to [%s]", static_cast<const char*>(split[0]), static_cast<const char*>(split[1]));
        }

        line = reader.readLine();
    }
}
