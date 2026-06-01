use std::collections::HashMap;

use btsensor::Reading;
use btsensor::bthome::events::{ButtonEventType, Event};
use btsensor::bthome::v2::{BtHomeV2, Element};
use uuid::Uuid;
use window_sensor::bthome;
use window_sensor::classifier::WindowState;
use window_sensor::telemetry::{
    AD_TYPE_SERVICE_DATA_UUID16, EnvReading, WindowStatus, encode_button_advertisement,
    encode_heartbeat_advertisement, encode_state_advertisement,
};

fn decode_bthome_v2(ad: &[u8]) -> BtHomeV2 {
    assert_eq!(ad[1], AD_TYPE_SERVICE_DATA_UUID16);
    let service_data = HashMap::from([(
        Uuid::from_u128(0x0000fcd2_0000_1000_8000_00805f9b34fb),
        ad[4..usize::from(ad[0]) + 1].to_vec(),
    )]);

    match Reading::decode(&service_data) {
        Some(Reading::BtHomeV2(reading)) => reading,
        other => panic!("unexpected decoded reading: {:?}", other),
    }
}

#[test]
fn state_advertisement_decodes_with_btsensor() {
    let mut buf = [0u8; 32];
    let len = encode_state_advertisement(
        &mut buf,
        7,
        WindowStatus {
            state: WindowState::Tilt,
            tampered: true,
            problem: true,
        },
    );

    let decoded = decode_bthome_v2(&buf[..len]);
    assert!(decoded.trigger_based);
    assert_eq!(
        decoded.elements,
        vec![
            Element::PacketId(7),
            Element::GenericBoolean(true),
            Element::Problem(true),
            Element::Tamper(true),
            Element::WindowOpen(true),
        ]
    );
}

#[test]
fn heartbeat_advertisement_decodes_with_btsensor() {
    let mut buf = [0u8; 32];
    let len = encode_heartbeat_advertisement(
        &mut buf,
        42,
        EnvReading {
            temperature_cdeg: 2350,
            humidity_cpct: 6520,
            battery_soc_pct: 88,
        },
        WindowStatus {
            state: WindowState::Closed,
            tampered: false,
            problem: false,
        },
    );

    let decoded = decode_bthome_v2(&buf[..len]);
    assert!(decoded.trigger_based);
    assert_eq!(
        decoded.elements,
        vec![
            Element::PacketId(42),
            Element::Battery(88),
            Element::TemperatureSmall(2350),
            Element::Humidity(6520),
            Element::GenericBoolean(false),
            Element::Problem(false),
            Element::Tamper(false),
            Element::WindowOpen(false),
        ]
    );
}

#[test]
fn button_advertisement_decodes_with_btsensor() {
    let mut buf = [0u8; 32];
    let len = encode_button_advertisement(&mut buf, 11, bthome::ButtonEvent::TriplePress);

    let decoded = decode_bthome_v2(&buf[..len]);
    assert!(decoded.trigger_based);
    assert_eq!(
        decoded.elements,
        vec![
            Element::PacketId(11),
            Element::ButtonEvent(Some(ButtonEventType::TriplePress)),
        ]
    );
    assert_eq!(
        decoded.elements[1].event(),
        Some(Event::Button(Some(ButtonEventType::TriplePress)))
    );
}
