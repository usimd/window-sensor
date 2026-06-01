//! Battery SoC estimation logic, independent of ADC/heater hardware wiring.

/// Typical SHT4x heater current at 20 mW / 3.6 V.
pub const HEATER_CURRENT_20MW_MA: u16 = 6;
/// Typical SHT4x heater current at 110 mW / 3.6 V.
pub const HEATER_CURRENT_110MW_MA: u16 = 31;
/// Typical SHT4x heater current at 200 mW / 3.6 V.
pub const HEATER_CURRENT_200MW_MA: u16 = 56;
/// Shipping default: 110 mW SHT4x heater pulse.
pub const DEFAULT_HEATER_CURRENT_MA: u16 = HEATER_CURRENT_110MW_MA;
const MAX_DROP_PER_UPDATE_PCT: u8 = 10;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(all(target_arch = "arm", target_os = "none"), derive(defmt::Format))]
pub struct BatteryMeasurement {
    pub open_mv: u16,
    pub load_mv: u16,
    pub load_current_ma: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(all(target_arch = "arm", target_os = "none"), derive(defmt::Format))]
pub struct BatteryState {
    pub percent: u8,
    pub voltage_mv: u16,
    pub resistance_mohm: u32,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BatteryEstimator {
    reported_percent: u8,
    best_resistance_mohm: u32,
}

impl BatteryEstimator {
    pub const fn new(initial_percent: u8) -> Self {
        Self {
            reported_percent: initial_percent,
            best_resistance_mohm: 0,
        }
    }

    pub const fn from_state(initial_percent: u8, best_resistance_mohm: u32) -> Self {
        Self {
            reported_percent: initial_percent,
            best_resistance_mohm,
        }
    }

    pub const fn current_percent(&self) -> u8 {
        self.reported_percent
    }

    pub const fn best_resistance_mohm(&self) -> u32 {
        self.best_resistance_mohm
    }

    pub fn update_from_measurement(&mut self, measurement: BatteryMeasurement) -> BatteryState {
        let resistance_mohm = internal_resistance_mohm(measurement);
        let raw_percent = estimate_percent(measurement.open_mv, resistance_mohm);
        let next_percent = smooth_monotonic_percent(self.reported_percent, raw_percent);
        self.reported_percent = next_percent;
        self.best_resistance_mohm = match self.best_resistance_mohm {
            0 => resistance_mohm,
            current => current.min(resistance_mohm),
        };

        BatteryState {
            percent: next_percent,
            voltage_mv: measurement.open_mv,
            resistance_mohm,
        }
    }

    pub fn update_from_open_voltage(&mut self, open_mv: u16) -> BatteryState {
        let raw_percent = open_voltage_percent(open_mv);
        let next_percent = smooth_monotonic_percent(self.reported_percent, raw_percent);
        self.reported_percent = next_percent;

        BatteryState {
            percent: next_percent,
            voltage_mv: open_mv,
            resistance_mohm: self.best_resistance_mohm,
        }
    }
}

pub fn internal_resistance_mohm(measurement: BatteryMeasurement) -> u32 {
    if measurement.load_current_ma == 0 || measurement.open_mv <= measurement.load_mv {
        return 0;
    }

    let delta_mv = (measurement.open_mv - measurement.load_mv) as u32;
    (delta_mv * 1000) / measurement.load_current_ma as u32
}

pub fn lowest_resistance_measurement(
    measurements: &[BatteryMeasurement],
) -> Option<BatteryMeasurement> {
    measurements
        .iter()
        .copied()
        .min_by_key(|measurement| internal_resistance_mohm(*measurement))
}

fn estimate_percent(open_mv: u16, resistance_mohm: u32) -> u8 {
    let voltage_cap = open_voltage_percent(open_mv);
    if resistance_mohm == 0 {
        return voltage_cap;
    }

    voltage_cap.min(interpolate_percent(
        resistance_mohm,
        &[
            (3_000, 100),
            (5_000, 97),
            (8_000, 92),
            (12_000, 84),
            (18_000, 72),
            (25_000, 60),
            (40_000, 42),
            (60_000, 24),
            (90_000, 10),
            (150_000, 0),
        ],
    ))
}

fn open_voltage_percent(open_mv: u16) -> u8 {
    interpolate_percent(
        u32::from(open_mv),
        &[
            (2900, 0),
            (3000, 5),
            (3100, 12),
            (3200, 28),
            (3300, 55),
            (3400, 80),
            (3500, 92),
            (3600, 100),
        ],
    )
}

fn interpolate_percent(value: u32, points: &[(u32, u8)]) -> u8 {
    let Some(&(first_x, first_y)) = points.first() else {
        return 0;
    };

    if value <= first_x {
        return first_y;
    }

    for window in points.windows(2) {
        let &[(x0, y0), (x1, y1)] = window else {
            continue;
        };

        if value <= x1 {
            let span = x1 - x0;
            if span == 0 {
                return y1;
            }

            let offset = value - x0;
            let y0 = i32::from(y0);
            let y1 = i32::from(y1);
            let interpolated = y0 + (((y1 - y0) * offset as i32) / span as i32);
            return interpolated.clamp(0, 100) as u8;
        }
    }

    points.last().map(|&(_, y)| y).unwrap_or(0)
}

fn smooth_monotonic_percent(previous: u8, raw: u8) -> u8 {
    let monotonic = raw.min(previous);
    let floor = previous.saturating_sub(MAX_DROP_PER_UPDATE_PCT);
    monotonic.max(floor)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn resistance_is_calculated_from_voltage_drop() {
        let measurement = BatteryMeasurement {
            open_mv: 3600,
            load_mv: 3445,
            load_current_ma: DEFAULT_HEATER_CURRENT_MA,
        };

        assert_eq!(internal_resistance_mohm(measurement), 5000);
    }

    #[test]
    fn fresh_cell_maps_to_full_charge() {
        let mut estimator = BatteryEstimator::new(100);
        let state = estimator.update_from_measurement(BatteryMeasurement {
            open_mv: 3650,
            load_mv: 3620,
            load_current_ma: DEFAULT_HEATER_CURRENT_MA,
        });

        assert_eq!(state.percent, 100);
        assert!(state.resistance_mohm < 5000);
    }

    #[test]
    fn deep_drop_is_smoothed_to_ten_percent_steps() {
        let mut estimator = BatteryEstimator::new(100);
        let state = estimator.update_from_measurement(BatteryMeasurement {
            open_mv: 3500,
            load_mv: 700,
            load_current_ma: DEFAULT_HEATER_CURRENT_MA,
        });

        assert_eq!(state.percent, 90);
    }

    #[test]
    fn charge_never_increases_again() {
        let mut estimator = BatteryEstimator::new(50);
        let state = estimator.update_from_measurement(BatteryMeasurement {
            open_mv: 3650,
            load_mv: 3620,
            load_current_ma: DEFAULT_HEATER_CURRENT_MA,
        });

        assert_eq!(state.percent, 50);
    }

    #[test]
    fn empty_cell_uses_voltage_floor() {
        let mut estimator = BatteryEstimator::new(12);
        let state = estimator.update_from_measurement(BatteryMeasurement {
            open_mv: 2700,
            load_mv: 2500,
            load_current_ma: DEFAULT_HEATER_CURRENT_MA,
        });

        assert_eq!(state.percent, 2);
    }

    #[test]
    fn open_voltage_only_fallback_tracks_end_of_life_knee() {
        let mut estimator = BatteryEstimator::new(100);

        let plateau = estimator.update_from_open_voltage(3580);
        assert_eq!(plateau.percent, 98);

        let knee = estimator.update_from_open_voltage(3200);
        assert_eq!(knee.percent, 88);
    }

    #[test]
    fn resistance_curve_is_smoother_than_coarse_buckets() {
        let mut estimator = BatteryEstimator::new(100);
        let state = estimator.update_from_measurement(BatteryMeasurement {
            open_mv: 3600,
            load_mv: 3290,
            load_current_ma: DEFAULT_HEATER_CURRENT_MA,
        });

        assert_eq!(state.resistance_mohm, 10000);
        assert_eq!(state.percent, 90);
    }

    #[test]
    fn auto_calibration_prefers_lowest_resistance_measurement() {
        let best = lowest_resistance_measurement(&[
            BatteryMeasurement {
                open_mv: 3600,
                load_mv: 3450,
                load_current_ma: DEFAULT_HEATER_CURRENT_MA,
            },
            BatteryMeasurement {
                open_mv: 3600,
                load_mv: 3520,
                load_current_ma: DEFAULT_HEATER_CURRENT_MA,
            },
            BatteryMeasurement {
                open_mv: 3600,
                load_mv: 3490,
                load_current_ma: DEFAULT_HEATER_CURRENT_MA,
            },
        ])
        .unwrap();

        assert_eq!(best.load_mv, 3520);
        assert!(internal_resistance_mohm(best) < 3_000);
    }

    #[test]
    fn estimator_tracks_best_resistance_seen() {
        let mut estimator = BatteryEstimator::new(100);

        estimator.update_from_measurement(BatteryMeasurement {
            open_mv: 3600,
            load_mv: 3500,
            load_current_ma: DEFAULT_HEATER_CURRENT_MA,
        });
        estimator.update_from_measurement(BatteryMeasurement {
            open_mv: 3600,
            load_mv: 3550,
            load_current_ma: DEFAULT_HEATER_CURRENT_MA,
        });

        assert!(estimator.best_resistance_mohm() > 0);
        assert_eq!(estimator.best_resistance_mohm(), 1612);
    }
}
