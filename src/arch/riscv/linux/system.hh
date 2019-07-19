/*
 * Copyright (c) 2019 Yifei Liu
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met: redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer;
 * redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution;
 * neither the name of the copyright holders nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Yifei Liu
 *          Lin Cheng
 *          Xihao Cheng
 *          Cheng Tan
 */
#ifndef __ARCH_LINUX_RISCV_SYSTEM_HH__
#define __ARCH_LINUX_RISCV_SYSTEM_HH__
#include <inttypes.h>

#include <memory>
#include <string>
#include <vector>

#include "arch/riscv/system.hh"
#include "kern/linux/events.hh"
#include "mem/fs_translating_port_proxy.hh"
#include "params/LinuxRiscvSystem.hh"
#include "params/RiscvSystem.hh"
#include "sim/core.hh"
#include "sim/mem_state.hh"
#include "sim/sim_object.hh"
#include "sim/system.hh"

typedef uint64_t hwaddr;
class LinuxRiscvSystem : public RiscvSystem
{
    protected:
        /**
        * PC based event to skip the dprink() call and emulate its
        * functionality
        */
        Linux::DebugPrintkEvent *debugPrintkEvent;
        //ObjectFile* bootloader;
        std::string commandLine;
        /** Bootloaders */
        std::vector<std::unique_ptr<ObjectFile>> bootLoaders;
        /**
        * Pointer to the bootloader object
        */
        ObjectFile *bootldr;
        /**
        * Get a boot loader that matches the kernel.
        *
        * @param obj Kernel binary
        * @return Pointer to boot loader ObjectFile or nullptr if there
        *         is no matching boot loader.
        */
        ObjectFile *getBootLoader(ObjectFile *const obj);
        FSTranslatingPortProxy virtProxy;
    public:
        Addr ZeroPGE() const   { return KernelStart + 0x0A000; }
        Addr Param() const { return ZeroPGE() + 0x0; }
        Addr CommandLine() const { return Param() + 0x0; }
        typedef LinuxRiscvSystemParams Params;
        const Params *
        params() const{
            return dynamic_cast<const Params *>(_params);
        }
        // initialize DTB in memory
        void initDTB();
        LinuxRiscvSystem(Params *p);
        ~LinuxRiscvSystem();
        // initialize the system
        virtual void initState();
        std::shared_ptr<MemState> memState;
    private:
        Addr KernelStart; // Lookup the symbol swapper_pg_dir
        /** Event to halt the simulator if the kernel calls panic()  */
        PCEvent *kernelPanicEvent;
        /** Event to halt the simulator if the kernel calls oopses  */
        PCEvent *kernelOopsEvent;
        /**
        * PC based event to skip udelay(<time>) calls and quiesce the
        * processor for the appropriate amount of time. This is not
        * functionally required but does speed up simulation.
        */
        Linux::UDelayEvent *uDelaySkipEvent;

        /** Another PC based skip event for const_udelay(). Similar to the
        * udelay skip, but this function precomputes the first multiply that
        * is done in the generic case since the parameter is known at compile
        * time. Thus we need to do some division to get back to us.
        */
        Linux::UDelayEvent *constUDelaySkipEvent;
};

#endif // __ARCH_LINUX_RISCV_SYSTEM_HH__
