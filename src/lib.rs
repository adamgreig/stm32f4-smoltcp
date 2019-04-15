#![no_std]

pub extern crate stm32f4xx_hal as hal;
use self::hal::stm32::{ETHERNET_DMA, ETHERNET_MAC, ETHERNET_PTP, RCC};

use smoltcp::{
    self,
    phy::{self, DeviceCapabilities},
    time::Instant,
    wire::EthernetAddress,
};

pub const ETH_BUF_SIZE: usize = 1536;

/// Transmit Descriptor representation
///
/// * tdes0: ownership bit and transmit settings
/// * tdes1: transmit buffer lengths
/// * tdes2: transmit buffer address
/// * tdes3: not used
///
/// Note that Copy and Clone are derived to support initialising an array of TDes,
/// but you may not move a TDes after its address has been given to the ETH_DMA engine.
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct TDes {
    tdes0: u32,
    tdes1: u32,
    tdes2: u32,
    tdes3: u32,
    tdes4: u32,
    tdes5: u32,
    tdes6: u32,
    tdes7: u32,
}

impl TDes {
    pub const fn new() -> TDes {
        TDes {
            tdes0: 0,
            tdes1: 0,
            tdes2: 0,
            tdes3: 0,
            tdes4: 0,
            tdes5: 0,
            tdes6: 0,
            tdes7: 0,
        }
    }

    /// Initialises this TDes to point at the given buffer.
    pub fn init(&mut self, tdbuf: &[u32]) {
        // Set FS and LS on each descriptor: each will hold a single full segment.
        self.tdes0 = (1 << 29) | (1 << 28);
        // Store pointer to associated buffer.
        self.tdes2 = tdbuf.as_ptr() as u32;
        // No second buffer.
        self.tdes3 = 0;
    }

    /// Mark this TDes as end-of-ring.
    pub fn set_end_of_ring(&mut self) {
        self.tdes0 |= 1 << 21;
    }

    /// Return true if the TDes is not currently owned by the DMA
    pub fn available(&self) -> bool {
        self.tdes0 & (1 << 31) == 0
    }

    /// Release this RDes back to DMA engine for transmission
    pub unsafe fn release(&mut self) {
        self.tdes0 |= 1 << 31;
    }

    /// Set the length of data in the buffer pointed to by this TDes
    pub unsafe fn set_length(&mut self, length: usize) {
        self.tdes1 = (length as u32) & 0x1FFF;
    }

    /// Access the buffer pointed to by this descriptor
    pub unsafe fn buf_as_slice_mut(&mut self) -> &mut [u8] {
        core::slice::from_raw_parts_mut(self.tdes2 as *mut _, self.tdes1 as usize & 0x1FFF)
    }

    pub unsafe fn buf_as_slice(&self) -> &[u8] {
        core::slice::from_raw_parts(self.tdes2 as *mut u8, self.tdes1 as usize & 0x1FFF)
    }

    /// Enable timestamping this outgoing packet
    pub unsafe fn set_timestamping(&mut self) {
        self.tdes0 |= 1 << 25;
    }

    /// Returns the timestamp associated with this transmitted packet if available
    pub unsafe fn get_timestamp(&self) -> Option<u64> {
        match self.available() && (self.tdes0 & (1 << 17) != 0) {
            true => Some(((self.tdes7 as u64) << 32) | (self.tdes6 as u64)),
            false => None,
        }
    }

    /// Enable interrupt on transmission completion
    pub unsafe fn set_interrupt(&mut self) {
        self.tdes0 |= 1 << 30;
    }

    /// Returns if this packet is set to interrupt on completion
    pub unsafe fn get_interrupt(&self) -> bool {
        self.available() && self.tdes0 & (1 << 30) != 0
    }

    /// Check if the buffer contains a PTP-over-UDP event packet
    ///
    /// Specifically checks for IPv4 ethertype, UDP protocol, and UDP destination port 319
    pub unsafe fn is_ptp_event(&self) -> bool {
        let buf = self.buf_as_slice();
        buf[12] == 0x08 && buf[13] == 0x00 && buf[23] == 0x11 && buf[36] == 0x01 && buf[37] == 0x3f
    }

    /// Check if the buffer contains a PTP-over-UDP SYNC packet
    pub unsafe fn is_ptp_sync(&self) -> bool {
        let buf = self.buf_as_slice();
        self.is_ptp_event() && buf[42] == 0x00
    }

    /// Check if the buffer contains a PTP-over-UDP DELAY_REQ packet
    pub unsafe fn is_ptp_delay_req(&self) -> bool {
        let buf = self.buf_as_slice();
        self.is_ptp_event() && buf[42] == 0x01
    }
}

/// Store a ring of TDes and associated buffers
pub struct TDesRing<'a> {
    pub td: &'a mut [TDes],
    pub tbuf: &'a mut [[u32; ETH_BUF_SIZE / 4]],
    pub tdidx: usize,
}

impl<'a> TDesRing<'a> {
    /// Initialise this TDesRing
    ///
    /// The current memory address of the buffers inside this TDesRing will be stored in the
    /// descriptors, so ensure the TDesRing is not moved after initialisation.
    pub fn init(&mut self) {
        for (td, tdbuf) in self.td.iter_mut().zip(self.tbuf.iter()) {
            td.init(&tdbuf[..]);
        }
        self.td.last_mut().unwrap().set_end_of_ring();
    }

    /// Return the address of the start of the TDes ring
    pub fn ptr(&self) -> *const TDes {
        self.td.as_ptr()
    }

    /// Return true if a TDes is available for use
    pub fn available(&self) -> bool {
        self.td[self.tdidx].available()
    }

    /// Return the next available TDes if any are available, otherwise None
    pub fn next(&mut self) -> Option<&mut TDes> {
        if self.available() {
            let len = self.td.len();
            let rv = Some(&mut self.td[self.tdidx]);
            self.tdidx = (self.tdidx + 1) % len;
            rv
        } else {
            None
        }
    }

    /// Returns the most recently transmitted TDes with interrupt enabled, if any
    pub fn last_interrupt(&self) -> Option<&TDes> {
        for off in 1..(self.td.len()+1) {
            let mut idx: isize = (self.tdidx - off) as isize;
            if idx < 0 {
                idx += self.td.len() as isize;
            }
            let rv = self.td[idx as usize];
            match unsafe { rv.get_interrupt() } {
                true => { return Some(&self.td[idx as usize]) },
                false => (),
            }
        }
        None
    }
}

/// Receive Descriptor representation
///
/// * rdes0: ownership bit and received packet metadata
/// * rdes1: receive buffer lengths and settings
/// * rdes2: receive buffer address
/// * rdes3: not used
///
/// Note that Copy and Clone are derived to support initialising an array of TDes,
/// but you may not move a TDes after its address has been given to the ETH_DMA engine.
#[derive(Copy, Clone)]
#[repr(C, packed)]
pub struct RDes {
    rdes0: u32,
    rdes1: u32,
    rdes2: u32,
    rdes3: u32,
    rdes4: u32,
    rdes5: u32,
    rdes6: u32,
    rdes7: u32,
}

impl RDes {
    pub const fn new() -> RDes {
        RDes {
            rdes0: 0,
            rdes1: 0,
            rdes2: 0,
            rdes3: 0,
            rdes4: 0,
            rdes5: 0,
            rdes6: 0,
            rdes7: 0,
        }
    }

    /// Initialises this RDes to point at the given buffer.
    pub fn init(&mut self, rdbuf: &[u32]) {
        // Mark each RDes as owned by the DMA engine.
        self.rdes0 = 1 << 31;
        // Store length of and pointer to associated buffer.
        self.rdes1 = rdbuf.len() as u32 * 4;
        self.rdes2 = rdbuf.as_ptr() as u32;
        // No second buffer.
        self.rdes3 = 0;
    }

    /// Mark this RDes as end-of-ring.
    pub fn set_end_of_ring(&mut self) {
        self.rdes1 |= 1 << 15;
    }

    /// Return true if the RDes is not currently owned by the DMA
    pub fn available(&self) -> bool {
        self.rdes0 & (1 << 31) == 0
    }

    /// Release this RDes back to the DMA engine
    pub unsafe fn release(&mut self) {
        self.rdes0 |= 1 << 31;
    }

    /// Access the buffer pointed to by this descriptor
    pub unsafe fn buf_as_slice(&self) -> &[u8] {
        core::slice::from_raw_parts(self.rdes2 as *const _, (self.rdes0 >> 16) as usize & 0x3FFF)
    }

    /// Access the buffer pointed to by this descriptor
    pub unsafe fn buf_as_slice_mut(&mut self) -> &mut [u8] {
        core::slice::from_raw_parts_mut(self.rdes2 as *mut _, (self.rdes0 >> 16) as usize & 0x3FFF)
    }

    /// Returns the timestamp associated with this received packet, if available
    pub unsafe fn get_timestamp(&self) -> Option<u64> {
        match self.available() && (self.rdes0 & (1 << 7) != 0) {
            true => Some(((self.rdes7 as u64) << 32) | (self.rdes6 as u64)),
            false => None,
        }
    }

    /// Check if the buffer contains a PTP-over-UDP event packet
    ///
    /// Specifically checks for IPv4 ethertype, UDP protocol, and UDP destination port 319
    pub unsafe fn is_ptp_event(&self) -> bool {
        let buf = self.buf_as_slice();
        buf[12] == 0x08 && buf[13] == 0x00 && buf[23] == 0x11 && buf[36] == 0x01 && buf[37] == 0x3f
    }
}

/// Store a ring of RDes and associated buffers
pub struct RDesRing<'a> {
    pub rd: &'a mut [RDes],
    pub rbuf: &'a mut [[u32; ETH_BUF_SIZE / 4]],
    pub rdidx: usize,
}

impl<'a> RDesRing<'a> {
    /// Initialise this RDesRing
    ///
    /// The current memory address of the buffers inside this TDesRing will be stored in the
    /// descriptors, so ensure the TDesRing is not moved after initialisation.
    pub fn init(&mut self) {
        for (rd, rdbuf) in self.rd.iter_mut().zip(self.rbuf.iter()) {
            rd.init(&rdbuf[..]);
        }
        self.rd.last_mut().unwrap().set_end_of_ring();
    }

    /// Return the address of the start of the RDes ring
    pub fn ptr(&self) -> *const RDes {
        self.rd.as_ptr()
    }

    /// Return true if a RDes is available for use
    pub fn available(&self) -> bool {
        self.rd[self.rdidx].available()
    }

    /// Return the next available RDes if any are available, otherwise None
    pub fn next(&mut self) -> Option<&mut RDes> {
        if self.available() {
            let len = self.rd.len();
            let rv = Some(&mut self.rd[self.rdidx]);
            self.rdidx = (self.rdidx + 1) % len;
            rv
        } else {
            None
        }
    }
}

/// Ethernet device driver
pub struct EthernetDevice {
    rdring: &'static mut RDesRing<'static>,
    tdring: &'static mut TDesRing<'static>,
    eth_mac: ETHERNET_MAC,
    eth_dma: ETHERNET_DMA,
    eth_ptp: ETHERNET_PTP,
    phy_addr: u8,
}

impl EthernetDevice {
    /// Create a new uninitialised EthernetDevice.
    ///
    /// You must move in ETH_MAC, ETH_DMA, and they are then kept by the device.
    ///
    /// You may only call this function once; subsequent calls will panic.
    pub fn new(
        eth_mac: ETHERNET_MAC,
        eth_dma: ETHERNET_DMA,
        eth_ptp: ETHERNET_PTP,
        rdring: &'static mut RDesRing,
        tdring: &'static mut TDesRing,
        phy_addr: u8,
    ) -> EthernetDevice {
        EthernetDevice {
            rdring,
            tdring,
            eth_mac,
            eth_dma,
            eth_ptp,
            phy_addr,
        }
    }

    /// Initialise the ethernet driver.
    ///
    /// Sets up the descriptor structures, sets up the peripheral clocks and GPIO configuration,
    /// and configures the ETH MAC and DMA peripherals.
    ///
    /// Brings up the PHY and then blocks waiting for a network link.
    pub fn init(&mut self, addr: EthernetAddress) {
        self.tdring.init();
        self.rdring.init();

        self.init_peripherals(addr);

        self.phy_reset();
        self.phy_init();
    }

    pub fn link_established(&mut self) -> bool {
        self.phy_poll_link()
    }

    pub fn block_until_link(&mut self) {
        while !self.link_established() {}
    }

    /// Resume suspended TX DMA operation
    pub fn resume_tx_dma(&mut self) {
        if self.eth_dma.dmasr.read().tps().is_suspended() {
            self.eth_dma.dmatpdr.write(|w| w.tpd().poll());
        }
    }

    /// Resume suspended RX DMA operation
    pub fn resume_rx_dma(&mut self) {
        if self.eth_dma.dmasr.read().rps().is_suspended() {
            self.eth_dma.dmarpdr.write(|w| w.rpd().poll());
        }
    }

    /// Enable receive interrupt
    pub fn enable_rx_interrupt(&mut self) {
        self.eth_dma
            .dmaier
            .modify(|_, w| w.nise().set_bit().rie().set_bit());
    }

    /// Disable receive interrupt
    pub fn disable_rx_interrupt(&mut self) {
        self.eth_dma.dmaier.modify(|_, w| w.rie().clear_bit());
    }

    /// Enable transmit interrupt
    pub fn enable_tx_interrupt(&mut self) {
        self.eth_dma
            .dmaier
            .modify(|_, w| w.nise().set_bit().tie().set_bit());
    }

    /// Disable transmit interrupt
    pub fn disable_tx_interrupt(&mut self) {
        self.eth_dma.dmaier.modify(|_, w| w.tie().clear_bit());
    }

    /// Sets up the device peripherals.
    fn init_peripherals(&mut self, mac: EthernetAddress) {
        cortex_m::interrupt::free(|_| {
            let rcc = unsafe { &(*RCC::ptr()) };

            // Reset ETH_MAC and ETH_DMA
            rcc.ahb1rstr.modify(|_, w| w.ethmacrst().reset());
            rcc.ahb1rstr.modify(|_, w| w.ethmacrst().clear_bit());
        });

        self.eth_dma.dmabmr.modify(|_, w| w.sr().reset());
        while self.eth_dma.dmabmr.read().sr().is_reset() {}

        // Enable PTP timestamping
        self.eth_ptp.ptptscr.write(|w| {
            w.tse().set_bit()
             .tsfcu().set_bit()
             .tsssr().set_bit()
             .tssarfe().set_bit()
        });

        // Set sub-second increment to 20ns and initial addend to HCLK/(1/20ns) (HCLK=100MHz)
        self.eth_ptp.ptpssir.write(|w| {
            unsafe { w.stssi().bits(20) }
        });
        self.eth_ptp.ptptsar.write(|w| {
            unsafe { w.tsa().bits(1<<31) }
        });
        self.eth_ptp.ptptscr.modify(|_, w| {
            w.ttsaru().set_bit()
        });
        while self.eth_ptp.ptptscr.read().ttsaru().bit_is_set() {}

        // Initialise timestamp
        self.eth_ptp.ptptscr.modify(|_, w| {
            w.tssti().set_bit()
        });
        while self.eth_ptp.ptptscr.read().tssti().bit_is_set() {}

        // Set MAC address
        let mac = mac.as_bytes();
        self.eth_mac.maca0lr.write(|w| {
            w.maca0l().bits(
                u32::from(mac[0])
                    | u32::from(mac[1]) << 8
                    | u32::from(mac[2]) << 16
                    | u32::from(mac[3]) << 24,
            )
        });
        self.eth_mac
            .maca0hr
            .write(|w| w.maca0h().bits(u16::from(mac[4]) | u16::from(mac[5]) << 8));

        // Enable RX and TX. We'll set link speed and duplex at link-up.
        self.eth_mac
            .maccr
            .write(|w| w.re().enabled().te().enabled().cstf().enabled());

        // Tell the ETH DMA the start of each ring
        self.eth_dma
            .dmatdlar
            .write(|w| w.stl().bits(self.tdring.ptr() as u32));
        self.eth_dma
            .dmardlar
            .write(|w| w.srl().bits(self.rdring.ptr() as u32));

        // Set DMA bus mode
        self.eth_dma
            .dmabmr
            .modify(|_, w| w.aab().aligned().pbl().pbl1().edfe().enabled());

        // Flush TX FIFO
        self.eth_dma.dmaomr.write(|w| w.ftf().flush());
        while self.eth_dma.dmaomr.read().ftf().is_flush() {}

        // Set DMA operation mode to store-and-forward and start DMA
        self.eth_dma.dmaomr.write(|w| {
            w.rsf()
                .store_forward()
                .tsf()
                .store_forward()
                .st()
                .started()
                .sr()
                .started()
        });
    }

    /// Read a register over SMI.
    fn smi_read(&mut self, reg: u8) -> u16 {
        // Use PHY address 00000, set register address, set clock to HCLK/102, start read.
        self.eth_mac.macmiiar.write(|w| {
            w.mb()
                .busy()
                .pa()
                .bits(self.phy_addr)
                .cr()
                .cr_150_168()
                .mr()
                .bits(reg)
        });

        // Wait for read
        while self.eth_mac.macmiiar.read().mb().is_busy() {}

        // Return result
        self.eth_mac.macmiidr.read().md().bits()
    }

    /// Write a register over SMI.
    fn smi_write(&mut self, reg: u8, val: u16) {
        // Use PHY address 00000, set write data, set register address, set clock to HCLK/102,
        // start write operation.
        self.eth_mac.macmiidr.write(|w| w.md().bits(val));
        self.eth_mac.macmiiar.write(|w| {
            w.mb()
                .busy()
                .pa()
                .bits(self.phy_addr)
                .mw()
                .write()
                .cr()
                .cr_150_168()
                .mr()
                .bits(reg)
        });

        while self.eth_mac.macmiiar.read().mb().is_busy() {}
    }

    /// Reset the connected PHY and wait for it to come out of reset.
    fn phy_reset(&mut self) {
        self.smi_write(0x00, 1 << 15);
        while self.smi_read(0x00) & (1 << 15) == (1 << 15) {}
    }

    /// Command connected PHY to initialise.
    fn phy_init(&mut self) {
        self.smi_write(0x00, 1 << 12);
    }

    /// Poll PHY to determine link status.
    fn phy_poll_link(&mut self) -> bool {
        let bsr = self.smi_read(0x01);
        let bcr = self.smi_read(0x00);
        let lpa = self.smi_read(0x05);

        // No link without autonegotiate
        if bcr & (1 << 12) == 0 {
            return false;
        }
        // No link if link is down
        if bsr & (1 << 2) == 0 {
            return false;
        }
        // No link if remote fault
        if bsr & (1 << 4) != 0 {
            return false;
        }
        // No link if autonegotiate incomplete
        if bsr & (1 << 5) == 0 {
            return false;
        }
        // No link if other side can't do 100Mbps full duplex
        if lpa & (1 << 8) == 0 {
            return false;
        }

        // Got link. Configure MAC to 100Mbit/s and full duplex.
        self.eth_mac
            .maccr
            .modify(|_, w| w.fes().fes100().dm().full_duplex());

        true
    }
}

pub struct TxToken(*mut EthernetDevice);
pub struct RxToken(*mut EthernetDevice);

impl phy::TxToken for TxToken {
    fn consume<R, F>(self, _timestamp: Instant, len: usize, f: F) -> smoltcp::Result<R>
    where
        F: FnOnce(&mut [u8]) -> smoltcp::Result<R>,
    {
        // There can only be a single EthernetDevice and therefore all TxTokens are wrappers
        // to a raw pointer to it. Unsafe required to dereference this pointer and call
        // the various TDes methods.
        assert!(len <= ETH_BUF_SIZE);
        unsafe {
            let tdes = (*self.0).tdring.next().unwrap();
            tdes.set_length(len);
            let result = f(tdes.buf_as_slice_mut());

            // Check if this frame should be timestamped.
            if tdes.is_ptp_event() {
                tdes.set_timestamping();
                tdes.set_interrupt();
            }

            tdes.release();
            (*self.0).resume_tx_dma();

            result
        }
    }
}

impl phy::RxToken for RxToken {
    fn consume<R, F>(self, _timestamp: Instant, f: F) -> smoltcp::Result<R>
    where
        F: FnOnce(&[u8]) -> smoltcp::Result<R>,
    {
        // There can only be a single EthernetDevice and therefore all RxTokens are wrappers
        // to a raw pointer to it. Unsafe required to dereference this pointer and call
        // the various RDes methods.
        unsafe {
            let rdes = (*self.0).rdring.next().unwrap();
            // For PTP events (SYNC/DELAY_REQ), we write the received timestamp into the UDP
            // data at a known offset and fix the UDP checksum.
            if rdes.is_ptp_event() {
                let ts = match rdes.get_timestamp() {
                    Some(ts) => ts,
                    None => 0,
                };
                let buf = rdes.buf_as_slice_mut();
                // Fix up UDP checksum for changed data
                let check = ((buf[14+20+6] as u16) << 8) | (buf[14+20+7] as u16);
                let mut check = (!check) as u32;
                check += (((ts >>  0) & 0xFFFF) as u16).to_be() as u32;
                check += (((ts >> 16) & 0xFFFF) as u16).to_be() as u32;
                check += (((ts >> 32) & 0xFFFF) as u16).to_be() as u32;
                check += (((ts >> 48) & 0xFFFF) as u16).to_be() as u32;
                check = (check >> 16) + (check & 0xFFFF);
                let check = !(check as u16);
                let check = if check == 0 { 0xFFFF } else { check };
                buf[14+20+6] = (check >> 8) as u8;
                buf[14+20+7] = check as u8;
                // Write timestamp as little-endian u64
                buf[14 + 20 + 8 + 4 + 0] = (ts >> 32) as u8;
                buf[14 + 20 + 8 + 4 + 1] = (ts >> 40) as u8;
                buf[14 + 20 + 8 + 4 + 2] = (ts >> 48) as u8;
                buf[14 + 20 + 8 + 4 + 3] = (ts >> 56) as u8;
                buf[14 + 20 + 8 + 4 + 4] = (ts >>  0) as u8;
                buf[14 + 20 + 8 + 4 + 5] = (ts >>  8) as u8;
                buf[14 + 20 + 8 + 4 + 6] = (ts >> 16) as u8;
                buf[14 + 20 + 8 + 4 + 7] = (ts >> 24) as u8;
            }
            let result = f(rdes.buf_as_slice());
            rdes.release();
            (*self.0).resume_rx_dma();
            result
        }
    }
}

// Implement the smoltcp Device interface
impl<'a> phy::Device<'a> for EthernetDevice {
    type RxToken = RxToken;
    type TxToken = TxToken;

    fn capabilities(&self) -> DeviceCapabilities {
        let mut caps = DeviceCapabilities::default();
        caps.max_transmission_unit = 1500;
        caps.max_burst_size = Some(core::cmp::min(self.rdring.rd.len(), self.tdring.td.len()));
        caps
    }

    fn receive(&mut self) -> Option<(RxToken, TxToken)> {
        if self.rdring.available() && self.tdring.available() {
            Some((RxToken(self), TxToken(self)))
        } else {
            None
        }
    }

    fn transmit(&mut self) -> Option<TxToken> {
        if self.tdring.available() {
            Some(TxToken(self))
        } else {
            None
        }
    }
}
