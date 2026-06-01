//! BLE advertising policy independent of the actual radio backend.

use crate::telemetry::{EnvReading, PacketCounter, WindowStatus};

pub const STATE_BURST_COUNT: u8 = 10;
pub const STATE_BURST_INTERVAL_MS: u32 = 100;
pub const HEARTBEAT_INTERVAL_S: u32 = 300;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(all(target_arch = "arm", target_os = "none"), derive(defmt::Format))]
pub enum AdvertisingEvent {
    StateBurst {
        packet_id: u8,
        repeats: u8,
        interval_ms: u32,
        status: WindowStatus,
    },
    Heartbeat {
        packet_id: u8,
        env: EnvReading,
        status: WindowStatus,
    },
}

/// High-level BLE scheduler state.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AdvertisingScheduler {
    packet_counter: PacketCounter,
    status: WindowStatus,
    heartbeat_interval_s: u32,
    next_heartbeat_at_ms: u64,
}

impl AdvertisingScheduler {
    pub fn new(now_ms: u64, heartbeat_interval_s: u32) -> Self {
        Self {
            packet_counter: PacketCounter::new(),
            status: WindowStatus {
                state: crate::classifier::WindowState::Closed,
                tampered: false,
                problem: false,
            },
            heartbeat_interval_s: heartbeat_interval_s.clamp(60, 3600),
            next_heartbeat_at_ms: now_ms + heartbeat_interval_s.clamp(60, 3600) as u64 * 1000,
        }
    }

    pub fn on_state_changed(&mut self, status: WindowStatus) -> AdvertisingEvent {
        self.status = status;
        AdvertisingEvent::StateBurst {
            packet_id: self.packet_counter.next_packet_id(),
            repeats: STATE_BURST_COUNT,
            interval_ms: STATE_BURST_INTERVAL_MS,
            status,
        }
    }

    pub fn on_tamper(&mut self) -> AdvertisingEvent {
        self.status.tampered = true;
        AdvertisingEvent::StateBurst {
            packet_id: self.packet_counter.next_packet_id(),
            repeats: STATE_BURST_COUNT,
            interval_ms: STATE_BURST_INTERVAL_MS,
            status: self.status,
        }
    }

    pub fn clear_tamper_if_closed(&mut self) {
        if self.status.state == crate::classifier::WindowState::Closed {
            self.status.tampered = false;
        }
    }

    pub fn due_heartbeat(&mut self, now_ms: u64, env: EnvReading) -> Option<AdvertisingEvent> {
        if now_ms < self.next_heartbeat_at_ms {
            return None;
        }

        let packet_id = self.packet_counter.next_packet_id();
        while self.next_heartbeat_at_ms <= now_ms {
            self.next_heartbeat_at_ms += self.heartbeat_interval_s as u64 * 1000;
        }

        Some(AdvertisingEvent::Heartbeat {
            packet_id,
            env,
            status: self.status,
        })
    }

    pub const fn status(&self) -> WindowStatus {
        self.status
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::classifier::WindowState;

    fn env() -> EnvReading {
        EnvReading {
            temperature_cdeg: 2150,
            humidity_cpct: 5100,
            battery_soc_pct: 92,
        }
    }

    #[test]
    fn state_change_emits_burst_policy() {
        let mut scheduler = AdvertisingScheduler::new(0, HEARTBEAT_INTERVAL_S);
        let event = scheduler.on_state_changed(WindowStatus {
            state: WindowState::Tilt,
            tampered: false,
            problem: false,
        });

        assert_eq!(
            event,
            AdvertisingEvent::StateBurst {
                packet_id: 0,
                repeats: STATE_BURST_COUNT,
                interval_ms: STATE_BURST_INTERVAL_MS,
                status: WindowStatus {
                    state: WindowState::Tilt,
                    tampered: false,
                    problem: false,
                },
            }
        );
    }

    #[test]
    fn heartbeat_is_silent_until_due() {
        let mut scheduler = AdvertisingScheduler::new(0, HEARTBEAT_INTERVAL_S);
        assert_eq!(scheduler.due_heartbeat(299_000, env()), None);
        assert!(matches!(
            scheduler.due_heartbeat(300_000, env()),
            Some(AdvertisingEvent::Heartbeat { packet_id: 0, .. })
        ));
    }

    #[test]
    fn tamper_persists_until_closed() {
        let mut scheduler = AdvertisingScheduler::new(0, HEARTBEAT_INTERVAL_S);
        let tamper = scheduler.on_tamper();
        assert!(matches!(
            tamper,
            AdvertisingEvent::StateBurst {
                status: WindowStatus { tampered: true, .. },
                ..
            }
        ));

        scheduler.on_state_changed(WindowStatus {
            state: WindowState::Open,
            tampered: true,
            problem: false,
        });
        scheduler.clear_tamper_if_closed();
        assert!(scheduler.status().tampered);

        scheduler.on_state_changed(WindowStatus {
            state: WindowState::Closed,
            tampered: true,
            problem: false,
        });
        scheduler.clear_tamper_if_closed();
        assert!(!scheduler.status().tampered);
    }

    #[test]
    fn heartbeat_catches_up_after_long_sleep() {
        let mut scheduler = AdvertisingScheduler::new(0, HEARTBEAT_INTERVAL_S);
        assert!(scheduler.due_heartbeat(905_000, env()).is_some());
        assert_eq!(scheduler.due_heartbeat(905_001, env()), None);
        assert!(scheduler.due_heartbeat(1_200_000, env()).is_some());
    }
}
