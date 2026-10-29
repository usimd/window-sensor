//! OTA transport: MCUmgr SMP over a BLE GATT service, backed by the MCUboot slots in RRAM.

use defmt::{info, warn};
use embassy_nrf::nvmc::Nvmc;
use embedded_storage::nor_flash::{ErrorType, NorFlash, ReadNorFlash};
use trouble_host::prelude::*;
use window_sensor::smp;

use crate::SettingsFlash;

/// Largest notification payload with the default 251-byte packet pool (ATT MTU 247).
pub const SMP_CHUNK: usize = 244;

#[gatt_server]
pub struct OtaServer {
    pub smp: SmpService,
}

/// MCUmgr SMP service; the UUIDs are fixed by mcumgr, smpclient and nRF Connect Device Manager.
#[gatt_service(uuid = "8d53dc1d-1db7-4cd3-868b-8a527460aa84")]
pub struct SmpService {
    #[characteristic(
        uuid = "da2e7828-fbce-4e01-ae9e-261174997c48",
        write_without_response,
        notify
    )]
    pub data: heapless::Vec<u8, SMP_CHUNK>,
}

/// NorFlash view of the RRAMC shared with settings persistence; locks per operation.
#[derive(Clone, Copy)]
pub struct SharedFlash(pub &'static SettingsFlash);

impl ErrorType for SharedFlash {
    type Error = <Nvmc<'static> as ErrorType>::Error;
}

impl ReadNorFlash for SharedFlash {
    const READ_SIZE: usize = <Nvmc<'static> as ReadNorFlash>::READ_SIZE;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        self.0.lock(|flash| flash.borrow_mut().read(offset, bytes))
    }

    fn capacity(&self) -> usize {
        self.0.lock(|flash| flash.borrow().capacity())
    }
}

impl NorFlash for SharedFlash {
    const WRITE_SIZE: usize = <Nvmc<'static> as NorFlash>::WRITE_SIZE;
    const ERASE_SIZE: usize = <Nvmc<'static> as NorFlash>::ERASE_SIZE;

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        self.0.lock(|flash| flash.borrow_mut().erase(from, to))
    }

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        self.0.lock(|flash| flash.borrow_mut().write(offset, bytes))
    }
}

pub type SmpServer = smp::Server<SharedFlash>;

/// Serves SMP requests until the client disconnects; returns true if it requested a reset.
pub async fn serve<P: PacketPool>(
    server: &OtaServer<'_>,
    conn: &GattConnection<'_, '_, P>,
    smp: &mut SmpServer,
) -> bool {
    let data = &server.smp.data;
    let mut reassembler = smp::Reassembler::default();
    let mut response = [0u8; smp::MAX_FRAME];
    loop {
        let event = match conn.next().await {
            GattConnectionEvent::Disconnected { reason } => {
                info!("[OTA] client disconnected: {}", reason);
                return smp.reset_requested();
            }
            GattConnectionEvent::Gatt { event } => event,
            _ => continue,
        };

        let mut len = 0;
        if let GattEvent::Write(write) = &event
            && write.handle() == data.handle
        {
            write.with_data(|_, bytes| {
                if let Some(frame) = reassembler.push(bytes) {
                    len = smp.handle(frame, &mut response);
                }
            });
        }
        if let Ok(reply) = event.accept() {
            reply.send().await;
        }
        if len == 0 {
            continue;
        }

        let chunk = usize::from(conn.raw().att_mtu())
            .saturating_sub(3)
            .clamp(20, SMP_CHUNK);
        for part in response[..len].chunks(chunk) {
            if data.notify_raw(conn, part, false).await.is_err() {
                warn!("[OTA] notification failed");
                break;
            }
        }
        if smp.reset_requested() {
            return true;
        }
    }
}
