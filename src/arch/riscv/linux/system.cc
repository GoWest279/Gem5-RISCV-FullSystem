
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
/*#include <algorithm>
#include <cstddef>
#include <iostream>
#include <iterator>
#include <map>
#include <string>
#include <vector>

*/

#include "arch/riscv/isa_traits.hh"
#include "arch/riscv/linux/system.hh"
#include "arch/riscv/utility.hh"
#include "base/loader/dtb_object.hh"
#include "base/loader/elf_object.hh"
#include "base/loader/object_file.hh"
#include "base/loader/symtab.hh"
#include "base/logging.hh"
#include "base/random.hh"
#include "cpu/base.hh"
#include "cpu/pc_event.hh"
#include "cpu/thread_context.hh"
#include "debug/Loader.hh"
#include "debug/Stack.hh"
#include "mem/fs_translating_port_proxy.hh"
#include "mem/physical.hh"
#include "sim/aux_vector.hh"
#include "sim/full_system.hh"
#include "sim/mem_state.hh"
#include "sim/stat_control.hh"
#include "sim/system.hh"

//using namespace Linux;
using namespace std;
using namespace RiscvISA;
using namespace Linux;

LinuxRiscvSystem::LinuxRiscvSystem(Params *p)
    : RiscvSystem(p),
      commandLine(p->boot_osflags),
      bootLoaders(), bootldr(),
      virtProxy(getSystemPort(), p->cache_line_size)
{
    _resetVect = kernel->textBase();//kernel->entryPoint();
}

LinuxRiscvSystem::~LinuxRiscvSystem()
{
   // delete bootloader;
}

void LinuxRiscvSystem::initDTB()
{
    ObjectFile *dtb_file = createObjectFile(params()->dtb_filename, true);
        if (!dtb_file)
                fatal("couldn't load DTB file: %s\n", params()->dtb_filename);
        else
                std::cout << "start load DTB file" << params()->dtb_filename
                                                                   << "\n";

        dtb_file->setTextBase(0x1020);
        dtb_file->loadSections(physProxy);
        delete dtb_file;
}

void
LinuxRiscvSystem::initState()
{
    RiscvSystem::initState();
        LinuxRiscvSystem::initDTB();

        threadContexts[0]->setIntReg(11, 0x1020);
        uint8_t  UART_REG_LINESTAT = 96;
    physProxy.writeBlob(0x10000005, (uint8_t *) &UART_REG_LINESTAT, 1);

    if (params()->early_kernel_symbols) {
        kernel->loadGlobalSymbols(kernelSymtab, 0, 0, loadAddrMask);
        kernel->loadGlobalSymbols(debugSymbolTable, 0, 0, loadAddrMask);
    }
}

LinuxRiscvSystem *
LinuxRiscvSystemParams::create()
{
    return new LinuxRiscvSystem(this);
}
