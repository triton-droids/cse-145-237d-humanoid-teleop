# Power and Battery

Each wearable node is fully self-powered. There is no shared power bus between
nodes; every IMU+MCU node carries its own cell, charger, and boost converter so
nodes can be placed, swapped, and charged independently.

## Per-Node Power Chain

```text
18650 cell (2500 mAh)
    -> TP4056 protected charger module (charge + cell protection)
    -> MT3608 boost converter (set to 5V)
    -> power switch (on the node protoboard)
    -> ESP32-S3 5V / VBUS pin
    -> onboard ESP32-S3 LDO -> 3V3
    -> 3V3 powers the MCU and the BNO085 (Vin)
```

## Components

| Part | Role | Notes |
|---|---|---|
| 18650 lithium cell, 2500 mAh | Energy storage | One cell per node, nominal ~3.7V (3.0–4.2V range) |
| TP4056 module (protected) | Charging + cell protection | Includes the DW01 + dual-MOSFET protection IC, so the cell has over-discharge and over-current cutoff. USB charge input. |
| MT3608 | Boost converter | Steps the single-cell voltage up to **5V** to feed the ESP32-S3 5V/VBUS input |

## Notes

- The boost target is 5V into the board's 5V/VBUS pin (not the 3V3 pin); the
  ESP32-S3's onboard regulator produces the 3V3 rail that powers both the MCU
  and the BNO085. Do not also drive 3V3 from an external source while the 5V
  rail is powered.
- A power switch on the MT3608 output (mounted on the node protoboard) is the
  node on/off control. See the physical assembly notes in
  [`README.md`](README.md#physical-node-assembly).
- The TP4056 protection circuit guards the cell, but charge the node from the
  TP4056 USB input, not from the ESP32-S3 USB port, unless you have confirmed
  the wiring keeps the two USB paths separate.
- The BNO085 wiring and full-UART transport are documented in
  [`README.md`](README.md#bno085-transport).

## Open / Unconfirmed

- Measured runtime per charge at the streaming report rate is not yet recorded.
- Any inline fusing between the cell and the TP4056 is not yet documented here.
