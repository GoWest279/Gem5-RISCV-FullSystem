# Copyright (c) 2019 Yifei Liu
# All rights reserved.
#
# The license below extends only to copyright in the software and shall
# not be construed as granting a license to any other intellectual
# property including but not limited to intellectual property relating
# to a hardware implementation of the functionality of the software
# licensed hereunder.  You may use the software subject to the license
# terms below provided that you ensure that this notice is replicated
# unmodified and in its entirety in all distributions of the software,
# modified or unmodified, in source code or in binary form.
#
# Copyright (c) 2006-2008 The Regents of The University of Michigan
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met: redistributions of source code must retain the above copyright
# notice, this list of conditions and the following disclaimer;
# redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution;
# neither the name of the copyright holders nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Authors: Yifei Liu
#          Lin Cheng
#          Xihao Cheng
#          Cheng Tan
from __future__ import print_function
from __future__ import absolute_import
import optparse
import sys
import os
import m5
from m5.defines import buildEnv
from m5.objects import *
from m5.objects.Device import IsaFake
from m5.objects.Platform import Platform
from m5.objects.Terminal import Terminal
from m5.objects.Uart import Uart8250
from m5.params import *
from m5.proxy import *


from m5.util import *
from m5.util.fdthelper import *
addToPath('../')
from common.FSConfig import *
from common.SysPaths import *
from common.Benchmarks import *
from common import Simulation
from common import CacheConfig
from common import MemConfig
from common import CpuConfig
from common import BPConfig
from common.Caches import *
from common import Options
from common import PlatformConfig
addToPath('/home/yliu/gem5/gem5_ad2039/src/dev')
addToPath('/home/yliu/gem5/gem5_ad2039/src/dev/serial')
#from Uart import Uart8250
##
class MemBus(SystemXBar):
    badaddr_responder = BadAddr()
    default = Self.badaddr_responder.pio

def init_fs(self, membus):
    IO_address_space_base = 0x80000000000
    self.riscv = Riscv() # system.riscv is our definiation
    self.iobus = IOXBar()
    self.riscv.attachIO(self.iobus)
    self.intrctrl = IntrControl()
    self.iobridge = Bridge(delay='50ns',ranges = \
                    [AddrRange(IO_address_space_base, Addr.max)])
    self.iobridge.master = self.iobus.slave
    self.iobridge.slave = self.membus.master

def createCPU(self):
    # Create a CPU for the system """
    # This defaults to one simple atomic CPU. Using other CPU models
    # and using timing memory is possible as well.
    # Also, changing this to using multiple CPUs is also possible
    # Note: If you use multiple CPUs, then the BIOS config needs to be
    #       updated as well.
    self.cpu = AtomicSimpleCPU()
    self.mem_mode = 'atomic'
    self.cpu.createThreads()

def createCacheHierarchy(self):
    """ Create a simple cache heirarchy with the caches from part1 """
    # Create an L1 instruction and data caches and an MMU cache
    # The MMU cache caches accesses from the inst and data TLBs
    self.cpu.icache_port = self.membus.slave
    self.cpu.dcache_port = self.membus.slave


def createMemoryControllers(self):
    """ Create the memory controller for the system """

        # Just create a controller for the first range, assuming the memory
        # size is < 3GB this will work. If it's > 3GB or if you want to use
        # mulitple or interleaved memory controllers then this should be
        # updated accordingly
    self.mem_cntrl = DDR3_1600_8x8(range = self.mem_ranges[0], \
                                   port = self.membus.master)

def setupInterrupts(self):
    """ Create the interrupt controller for the CPU """
    # create the interrupt controller for the CPU, connect to the membus
    self.cpu.createInterruptController()

        # For x86 only, make sure the interrupts are connected to the memory
        # Note: these are directly connected to the memory bus, not cached
        #self.cpu.interrupts[0].pio = self.membus.master
        #self.cpu.interrupts[0].int_master = self.membus.slave
        #self.cpu.interrupts[0].int_slave = self.membus.master
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
## Entry

self = LinuxRiscvSystem()
self.clk_domain = SrcClockDomain()
self.clk_domain.clock = '3GHz'
self.clk_domain.voltage_domain = VoltageDomain()
#self.iobus = IOXBar()
self.membus = SystemXBar()
self.membus.badaddr_responder = BadAddr()
self.membus.default = self.membus.badaddr_responder.pio

mem_size = '8192MB'
self.mem_ranges = [ AddrRange(start=0,size='4GB'),
                    AddrRange(0xC0000000, 0xFFFF0000)]


# Set up the system port for functional access from the simulator
self.system_port = self.membus.slave

# This will initialize most of the x86-specific system parameters
# This includes things like the I/O, multiprocessor support, BIOS...
init_fs(self, self.membus)
# bbl4: berkely boot loader with customerized load
# bbl3: berkely boot loader with original load
# bbl6: berkely boot loader with linux kernel
self.kernel = binary\
            ('/home/yliu/A00gem5/Gem5-RISCV-FullSystem/configs/example/bbl3')
self.dtb_filename = \
            '/home/yliu/A00gem5/Gem5-RISCV-FullSystem/configs/example/cpu.dtb'
boot_options = ['earlyprintk=ttyS0,384000',
                'console=ttyS0,384000',
                'lpj=7999923',
                'root=/dev/hda1']
self.boot_osflags = ' '.join(boot_options)


# Create the CPU for our system.
createCPU(self)

# Create the cache heirarchy for the system.
createCacheHierarchy(self)

# Create the memory controller for the sytem
createMemoryControllers(self)

# Set up the interrupt controllers for the system (x86 specific)
setupInterrupts(self)

#self.com_1 = Uart8250()
#self.terminal = Terminal()
#self.com_1.pio = bus.master
# set up the root SimObject and start the simulation
root = Root(full_system = True, system = self)

# instantiate all of the objects we've created above
m5.instantiate()
print("Beginning simulation!")
exit_event = m5.simulate()
