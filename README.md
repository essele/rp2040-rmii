# rp2040-rmii

An ethernet MAC implementation for the RP2040 / Pico

This is very much work-in-progress and will develop over time, currently working is:

- Pico clocked at 100MHz or 150MHz
- No need for a fully dedicated core
- Works with LAN8720
- LWIP Integration
- 100BASE-TX send and receive, both half & full duplex
- 10BASE-T send and receive, both half & full duplex
- TX inter-packet-gap honored for full duplex
- Half duplex TX carrier detection and inter-packet-gap (see below)
- Both TX & RX use 32bit DMA to improve performance
- PIO based mdio interface
- Auto detection of phy type and address
- Link state & speed monitoring
- Automatic switching of PIO code on 10/100 speed change
- Incoming packets checksummed using DMA snooping
- Additional IP checksum module using INTERP0 (25% faster than LWIP versions)

Still to do:

- Support for RTL8201 
- Option to drop IRQ's and dedicate a core to this

Current WIP Status:

- 150MHz working (compile time decision)
- However, IPG at 10meg/half not long enough ... run out of PIO instructions!
- Some checksum and overrun errors when flood pinging (not convinced this isn't jumper-wire related)
- Other possible cause is phase different between generated 50MHz, there shouldn't be one any more so need to test for this.
- Started #define's for chosing LWIP or not.
- Started #define's for debugging.

## Background

This was orginially inspired by [Pico RMII Ethernet](https://github.com/sandeepmistry/pico-rmii-ethernet) by Sandeep Mistry. 

Who knew you could directly attach a PHY to a microcontroller! But the RP2040 is quite special!

There is also another implementation which provided some additional insight - [pico-rmiieth](https://github.com/strags/pico-rmiieth) by Strags.

These are both great bits of work but (in my view) they both have significant flaws caused by using an unmodified LAN8720 breakout board. This means that you have very limited options for clocking, and then you end up in all sorts of mess.

Sandeep's implementation actually clocks the Pico from the clock on the LAN8720 board through a GPIO. This is great for aligning the PIO and the PHY clock, however this then limits the Pico to 50Mhz (quite a drop!) and then you have a real challenge that you only have a single cycle in the PIO for each dibit. This means you can't detect the packet end signal in the PIO and you have to use interrupts, which will always lag, so you then don't know where the end of the packet is, and you have to search for it by finding the checksum. 
Personally I think this is horrible, I don't think you'll have enough processing time (especially at 50MHz) to do this between packets on a busy network, and the checksum approach has a number of flaws. Plus I think it's impossible to transmit at 100Mbps (although this depends on my understading of the TXEN requirements being correct.) 

Strags' implementation overclocks the Pico to 250MHz and then samples the input with a 5 cycles/clock approach in the PIO. This is interesting and should work, but you're outside of the specs for the RP2040 (by quite a margin.) Plus they've missed a trick on packet end detection, so they have the same checksum search issue as Sandeep -- although this could be fixed.

These are great implementations, and very inspiring ... I never would have thought of this! Plus they both work with a completely unmodified breakout board, meaning it's much easier for people to test and experiment with.

## My Approach

My intended use for this is actually a custom board, so I'm not ultimately constrained by a fixed approach to clocking because of the breakout board.

I believe there are two ways this could be clocked...

1. Feeding the 50MHz LAN8720 clock into the RP2040 XIN pin, this can then be fed to the PLL and you can recreate all the normal clocks (scaling up to 1200MHz should give you the ability to generate 48MHz, 100MHz, 125Mhz etc. quite easily.) This removes the need for a crystal for the RP2040, but you do need one for the PHY.
2. Generating the 50MHz clock on the RP2040 (gpout) and feeding it to the PHY. This means you can use a standard 12MHz crystal and have the usb bootloader still work.

In either case you need to clock the CPU in the RP2040 at a multiple of 50MHz and it must be at least 100MHz to allow the packet end detection in the PIO. So you are really limited to 100MHz, 150MHz, 200MHz etc. Most of my testing has been at 100MHz, although 150MHz is now working and seems stable.

I've used the second approach for my testing, you can make a minor tweak to the breakout board to get this to work.

TODO: include details and photo of the mod.

This approach means that (a) the PHY and the RP2040 are synchronised, and (b) you have at least two cycles per dibit in the PIO state machine, this means you can detect the end of the packet cleanly and not need to do any searching for checksums.

The DMA engine on the RP2040 includes a snooping system which can create an ethernet checksum, this works perfectly given packet end detection so we can accurately transfer the right number of bytes, and effective have zero CPU used for computing ethernet checksums.

It would also have been nice to be able to use the DMA snooping system for building the checksum on transmits, but sadly there's only one and it's more important (from a timing perspective) to have it done on receive, and shareing isn't really an option.

I've also built an IP checksum module with uses the INTERP0 unit on the RP2040 ... sadly you can't DMA to this device, but it does support two 16bit additions in a single cycle which makes it slightly quicker than any other approach I could find. My testing using O3 optimisation shows it to be approaching 30% faster than the fastest version included with LWIP. This was with a mixed workload that varied sizes and alignments.

**It's very much work-in-progress -- I'm making sweeping changes to the code regularly, so it's more for reference than anything else at the moment.**

(I'm just about to move onto LWIP support so I can start testing the IP side.)
