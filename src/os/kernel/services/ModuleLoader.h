#ifndef __ModuleLoader_include__
#define __ModuleLoader_include__


#include <kernel/Module.h>
#include "kernel/KernelService.h"
#include "kernel/KernelSymbols.h"

/**
 * @author Filip Krakowski
 */
class ModuleLoader : public KernelService {

public:

    enum class Status : uint32_t {
        OK              = 0x00,
        MISSING_DEP     = 0x01,
        INVALID         = 0x02,
        WRONG_TYPE      = 0x03,
        ERROR           = 0x04
    };

    ModuleLoader() = default;

    ModuleLoader(const ModuleLoader &other) = delete;

    ModuleLoader &operator=(const ModuleLoader &other) = delete;

    /**
     * Loads a new module into memory and initializes it.
     *
     * @param file The module's file
     * @return A Status indicating if the operation succeeded
     */
    Status load(File *file);

    static constexpr const char* SERVICE_NAME = "ModuleLoader";

private:

    HashMap<String, Module*> modules;

};


#endif
