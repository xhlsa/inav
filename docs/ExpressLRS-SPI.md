# ExpressLRS SPI Receiver

This document covers the onboard SPI ExpressLRS (SX1280, 2.4 GHz) receiver support ported from Betaflight into iNav.

## Supported Boards and Hardware

The SPI ExpressLRS receiver driver is available on target boards that define hardware support for the onboard SPI receiver (`USE_RX_EXPRESSLRS`). The onboard hardware uses the Semtech SX1280 2.4 GHz transceiver connected over SPI.

Firmware builds are compiled for a specific ExpressLRS major version. The firmware build running on the flight controller must match the major version of ExpressLRS running on the transmitter module:
* Transmitters running **ELRS 3.x** require an **ELRS3** firmware build.
* Transmitters running **ELRS 4.x** require an **ELRS4** firmware build.

## Configuration

### Receiver Type

On flight controller targets with an integrated SPI ExpressLRS receiver, `receiver_type` defaults to `SPI`. It can also be set explicitly in the CLI:

```
set receiver_type = SPI
```

### Binding and Bind Phrase

ExpressLRS binding can be configured either by setting a binding phrase or by using the binding command.

#### Binding Phrase

You can set an ExpressLRS binding phrase via the CLI setting `expresslrs_bind_phrase`:

```
set expresslrs_bind_phrase = your phrase here
save
```

* The binding phrase is identical to the one configured on your transmitter.
* The phrase is **case-sensitive** and supports up to a maximum of 32 characters.
* CLI entry preserves the exact case entered (it is not converted to uppercase). Running `get expresslrs_bind_phrase` echoes the phrase back in the exact case typed.
* The receiver UID is derived from an MD5 hash of this exact phrase at boot.

#### Binding via CLI or Bind Button

If `expresslrs_bind_phrase` is left empty (`""`), the receiver can learn a UID during binding:

1. Put the receiver into bind mode by entering the CLI command:
```
bind_rx
```
(Alternatively, press the board's hardware bind button if available.)
2. Initiate the bind procedure from your transmitter's ExpressLRS Lua script.
3. Once bound, the learned UID is automatically saved to storage when the craft is disarmed.

The onboard receiver LED blinks while searching and turns solid when connected.

### Regulatory Domain

The regulatory domain must match your transmitter module's domain setting. This is configured via `expresslrs_domain`:

```
set expresslrs_domain = ISM2400
save
```

Allowed values:
* `ISM2400` (default)
* `CE2400` (EU LBT variant)

### Model Match

ExpressLRS model match is configured using `expresslrs_model_id`:

```
set expresslrs_model_id = 255
save
```

* Range: `0` to `255`.
* Default: `255` (disables model match).
* Set to a specific ID from `0` to `255` to match the model ID set on the transmitter.

## Supported Packet Rates and Switch Modes

### Packet Rates

The following 2.4 GHz packet rates are supported:
* **F1000**
* **F500**
* **500 Hz**
* **250 Hz**
* **150 Hz**
* **50 Hz**

*Note on LoRa coding rate:* ExpressLRS 3.3+ uses LoRa coding rate 4/8 for 250 Hz, 150 Hz, and 50 Hz. This port configures coding rate 4/7 (matching Betaflight). If 250 Hz, 150 Hz, or 50 Hz fails to connect while 500 Hz or F500 connects, this coding rate divergence is the first suspect.

### Switch Modes and Channels

* **Switch Modes:** `Hybrid` and `Wide` switch modes are supported.
* **Channels:** 16 channels are supported. ExpressLRS places AUX1 (channel 5) on the low-latency switch; the craft must be configured to **arm on AUX1**.

## Unsupported Features

The following features are not supported:
* Full-resolution 8/12/16-channel switch modes, 100 Hz Full, 333 Hz Full, D250, and D500 (these utilize a 13-byte packet format not implemented in the SPI receiver driver).
* Configuring iNav from the transmitter Lua script (MSP over ELRS is not ported).
* 900 MHz operation (the onboard hardware is 2.4 GHz SX1280 only).

## Channels and Link Statistics

* Channels 15 and 16 carry Link Quality (LQ) and RSSI.
* Link statistics (RSSI in dBm, LQ, SNR, and transmitter power) are available for the OSD.
* RSSI can be read via `rssi_source`.

## Telemetry

CRSF telemetry to the transmitter is supported over the radio link. The receiver transmits flight controller telemetry back to the radio, including:
* Battery voltage
* GPS data
* Attitude
* Flight mode

## Debug Modes

Receiver performance and diagnostics can be monitored using `debug_mode`:

* `set debug_mode = ELRS_PHASELOCK`: Logs phase synchronization to blackbox. `debug[1]` displays filtered phase offset, which settles near 0 when synchronized.
* `set debug_mode = ELRS_SPI`: Displays lost-connection count, RSSI, SNR, and LQ.

## Differences from Betaflight

This port has two key user-visible behavioral differences compared to Betaflight:

1. **Failsafe is driven by RC packets only:** Betaflight also counts sync packets as signal present, which can cause the receiver to hold the last stick positions if the RC link drops while sync packets continue to be received. In this iNav port, only accepted RC packets reset the signal loss timeout.
2. **Settings are never saved while armed:** Betaflight saves learned packet rates or UIDs whenever they change, which writes to internal flash and stalls the CPU. In this iNav port, settings (such as the UID learned from `bind_rx`) are never saved while armed, and are only written once disarmed.
