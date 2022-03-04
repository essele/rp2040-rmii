# rp2040-rmii

An ethernet MAC implementation for the RP2040 / Pico

This is very much work-in-progress and will develop over time, currently working is:

- Pico clocked at 100MHz
- No need for a fully dedicated core
- 100BASE-TX send and receive, both half & full duplex
- 10BASE-T send and receive, both half & full duplex
- PIO based mdio interface
- Link state & speed monitoring
- Incoming packets checksummed using DMA snooping
- Additional IP checksum module using INTERP0 (25% faster than LWIP versions)

Still to do:

- LWIP integration
- Switching of PIO code on speed change
- Support for 150MHz clocking
- Option to drop IRQ's and dedicate a core to this

## Background

This was orginially inspired by [Pico RMII Ethernet](https://github.com/sandeepmistry/pico-rmii-ethernet) by Sandeep Mistry. 

Who knew you could directly attach a PHY to a microcontroller! But the RP2040 is quite special!

There is also another implementation which provided some additional insight - [pico-rmiieth](https://github.com/strags/pico-rmiieth) by Strags.

These are both great bits of work but (in my view) they both have significant flaws caused by them using an unmodified LAN8720 breakout board. This means that you have very limited options for clocking, and then you end up in all sorts of mess.

Sandeep's implementation actually clocks the Pico from the clock on the LAN8720 board through a GPIO. This is great for aligning the PIO and the PHY clock, however this then limits the Pico to 50Mhz (quite a drop!) and then you have a real challenge that you only have a single cycle in the PIO for each dibit. This means you can't detect the packet end signial in the PIO and you have to use interrupts, which will always lag, so you then don't know where the end of the packet is, and you have to search for it by finding the checksum. Personally I think this is horrible, I don't think you'll have enough processing time (especially at 50MHz) to do this between packets on a busy network, and the checksum approach has a number of flaws.

Strags' implementation overclocks the Pico to 250MHz and then samples the input with a 5 cycles/clock approach in the PIO. This is interesting and should work, but you're outside of the specs for the RP2040 (by quite a margin.) Plus they've missed a trick on packet end detection, so they have the same checksum search issue as Sandeep.

These are great implementations, and very inspiring ... I never would have thought of this! Plus they both work with a completely unmodified breakout board, meaning it's much easier for people to test and experiment with.

## My Approach

My intended use for this is actually a custom board, so I'm not ultimately constrained by a fixed approach to clocking because of the breakout board.

I believe there are two ways this could be clocked...

1. Feeding the 50MHz LAN8720 clock into the RP2040 XIN pin, this can then be fed to the PLL and you can recreate all the normal clocks (scaling up to 1200MHz should give you the ability to generate 48MHz, 100MHz, 125Mhz etc. quite easily.)
2. Generating the 50MHz clock on the RP2040 (gpout) and feeding it to the PHY.

In either case you need to clock the CPU in the RP2040 at a multiple of 50MHz and it must be at least 100MHz to allow the packet end detection in the PIO. So you are really limited to 100MHz, 150MHz, 200MHz etc. All my testing is at 100MHz currently, although I do plan to include 150MHz support.

I've used the second approach for my testing, you can make a minor tweak to the breakout board to get this to work.

TODO: include details and photo of the mod.

This approach means the (a) the PHY and the RP2040 are synchronised, and (b) you have at least two cycles per dibit in the PIO state machine, this means you can detect the end of the packet cleanly and not need to do any searching for checksums.

The DMA engine on the RP2040 includes a snooping system which can create an ethernet checksum, this works perfectly given packet end detection so we can accurately transfer the right number of bytes, and effective have zero CPU used for computing ethernet checksums.

It would also have been nice to be able to use the DMA snooping system for building the checksum on transmits, but sadly there's only one and it's more important (from a timing perspective) to have it done on receive.

I've also built an IP checksum module with uses the INTERP0 unit on the RP2040 ... sadly you can't DMA to this device, but it does support two 16bit additions in a single cycle which makes it slightly quicker than any other approach I could fine. My testing using O3 optimisation shows it to be approaching 30% faster than the fasted version included with LWIP. This was with a mixed workload that varied sizes and alignments.

It's very much work-in-progress.
