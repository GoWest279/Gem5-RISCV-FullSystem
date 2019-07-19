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
//#include <cstdio>
#include "dev/riscv/Riscv.hh"

#include <deque>
#include <string>
#include <vector>

#include "arch/riscv/system.hh"
#include "config/the_isa.hh"
#include "cpu/intr_control.hh"

//#include "sim/system.hh"

using namespace std;
using namespace TheISA;

Riscv::Riscv(const Params *p)
    : Platform(p), system(p->system)
{
    printf("*****************Riscv::Riscv*************\n");
    for (int i = 0; i < Riscv::Max_CPUs; i++)
        intr_sum_type[i] = 0;
}
void Riscv::init()
{
    printf("*****************Riscv::init**************\n");
    RiscvSystem *riscvSystem = dynamic_cast<RiscvSystem *>(system);
    assert(riscvSystem);
//    riscvSystem->setIntrFreq(io->freqeuncy());
}


void Riscv::postConsoleInt() {
    //southBridge->ioApic->signalInterrupt(4);
    //southBridge->pic1->signalInterrupt(4);
}
void Riscv::clearConsoleInt() {
    warn_once("Don't know what interrupt to clear for console.\n");
    //panic("Need implementation\n");
}
void Riscv::postPciInt(int line) {
    //southBridge->ioApic->signalInterrupt(line);
    printf("Test postRiscvInt");
}
void Riscv::clearPciInt(int line) {
    warn_once("Tried to clear PCI interrupt %d\n", line);
}

void Riscv::serialize(CheckpointOut &cp) const
{
    SERIALIZE_ARRAY(intr_sum_type, Riscv::Max_CPUs);
}

void Riscv::unserialize(CheckpointIn &cp)
{
    UNSERIALIZE_ARRAY(intr_sum_type, Riscv::Max_CPUs);
}


Riscv * RiscvParams::create()
{
    return new Riscv(this);
}
