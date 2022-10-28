#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::v2::{OutputPin, StatefulOutputPin};
use embedded_time::rate::*;
use enc28j60::{smoltcp_phy::Phy, Enc28j60};
use panic_probe as _;
use rp_pico::hal::{self, gpio, pac, prelude::*, spi};
use smoltcp::{
    iface::{EthernetInterfaceBuilder, NeighborCache},
    socket::{SocketSet, TcpSocket, TcpSocketBuffer},
    time::Instant,
    wire::{EthernetAddress, IpAddress, IpCidr, Ipv4Address},
};
/* Configuration */
const SRC_MAC: [u8; 6] = [0x20, 0x18, 0x03, 0x01, 0x00, 0x00];

#[rp_pico::entry]
fn main() -> ! {
    info!("Program start");

    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();

    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);

    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let sio = hal::Sio::new(pac.SIO);

    let pins = rp_pico::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );
    let mut led = pins.led.into_push_pull_output();
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().integer());

    //SPI
    let spi = {
        let _spi_sclk = pins.gpio2.into_mode::<gpio::FunctionSpi>();
        let _spi_mosi = pins.gpio3.into_mode::<gpio::FunctionSpi>();
        let _spi_miso = pins.gpio4.into_mode::<gpio::FunctionSpi>();
        let spi = spi::Spi::<_, _, 8>::new(pac.SPI0);

        spi.init(
            &mut pac.RESETS,
            clocks.peripheral_clock.freq(),
            1_000_000u32.Hz(),
            &embedded_hal::spi::MODE_0,
        )
    };
    debug!("SPI initialized");

    // ENC28J60
    let enc28j60 = {
        let mut ncs = pins.gpio21.into_push_pull_output();
        let _ = ncs.set_high();
        let mut reset = pins.gpio22.into_push_pull_output();
        let _ = reset.set_high();

        Enc28j60::new(
            spi,
            ncs,
            enc28j60::Unconnected,
            reset,
            &mut delay,
            7168,
            SRC_MAC,
        )
        .ok()
        .unwrap()
    };
    debug!("Enc28j60 initialized");

    // PHY Wrapper
    let mut rx_buf = [0u8; 1024];
    let mut tx_buf = [0u8; 1024];
    let mut eth = Phy::new(enc28j60, &mut rx_buf, &mut tx_buf);
    debug!("Phy Wrapper created");

    // Ethernet interface
    let local_addr = Ipv4Address::new(192, 168, 5, 2);
    let ip_addr = IpCidr::new(IpAddress::from(local_addr), 24);
    let mut ip_addrs = [ip_addr];
    let mut neighbor_storage = [None; 16];
    let neighbor_cache = NeighborCache::new(&mut neighbor_storage[..]);
    let ethernet_addr = EthernetAddress(SRC_MAC);
    let mut iface = EthernetInterfaceBuilder::new(&mut eth)
        .ethernet_addr(ethernet_addr)
        .ip_addrs(&mut ip_addrs[..])
        .neighbor_cache(neighbor_cache)
        .finalize();
    debug!("Ethernet initialized");

    // Sockets
    let mut server_rx_buffer = [0; 2048];
    let mut server_tx_buffer = [0; 2048];
    let server_socket = TcpSocket::new(
        TcpSocketBuffer::new(&mut server_rx_buffer[..]),
        TcpSocketBuffer::new(&mut server_tx_buffer[..]),
    );
    let mut sockets_storage = [None, None];
    let mut sockets = SocketSet::new(&mut sockets_storage[..]);
    let server_handle = sockets.add(server_socket);
    debug!("Sockets initialized");

    let mut count: u64 = 0;
    loop {
        match iface.poll(&mut sockets, Instant::from_millis(0)) {
            Ok(b) => {
                if b {
                    let mut socket = sockets.get::<TcpSocket>(server_handle);
                    if !socket.is_open() {
                        socket.listen(80).unwrap();
                    }

                    if socket.can_send() {
                        // Toggle LED, without using ToggleableOutputPin in case it isn't implemented.
                        let _ = match led.is_set_high() {
                            Result::Ok(true) => led.set_low(),
                            _ => led.set_high(),
                        };
                        count += 1;
                        debug!("tcp:80 send");
                        socket
                            .send_slice("HTTP/1.1 200 OK\r\n\r\nHello!".as_bytes())
                            .unwrap();

                        debug!("tcp:80 close");
                        socket.close();
                    }
                }
            }
            Err(e) => {
                crate::todo!();
                // debug!(core::format_args!("{:?}", e));
            }
        }
    }
}
