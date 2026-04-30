# Robot control protocol

The robot now uses a framed binary protocol over UART for the firmware link. The older JSON message format is kept below as a reference and for simulator tooling that still uses it.


## Binary UART protocol

Each packet is a raw byte frame with this layout:

- `start`: `0xA5`
- `nonce`: `uint8`
- `size`: `uint16` (payload size)
- `header_checksum`: CRC-8 over the first 4 header bytes (`start`, `nonce`, `size_low`, `size_high`)
- `payload`: `size` bytes

The header checksum uses CRC-8 with polynomial `0x07`, initial value `0x00`, and no final xor. Payload bytes are not checksummed by the header.


### Command payloads

Each command payload starts with a `type` byte that specifies the command type.

#### Move command

Payload bytes:

- `type`: `uint8` (value = `1`)
- `left_power`: `float32`
- `right_power`: `float32`

#### Claw command

Payload bytes:

- `type`: `uint8` (value = `2`)
- `action`: `uint8` (`0`: close, non-zero: open)

#### Subscribe command

Payload bytes:

- `type`: `uint8` (value = `3`)


### Measurement payloads

Payload bytes:

- `timestamp`: `int64`
- `lidar_count`: `uint16`
- `lidar_count` repeated entries of:
  - `angle`: `uint16` (degrees / 64)
  - `distance`: `uint16` (millimeters / 4)
- `encoders.left_ticks`: `int32`
- `encoders.right_ticks`: `int32`

The full measurement payload is wrapped in the same framed packet format as commands.


## JSON protocol

Packet framing is expected to be handled by the transport layer, so the JSON messages are sent as raw strings without additional framing.


### Commands

Each command is a JSON object with a `command` field specifying the type of command.

#### Move command

```json
{
  "command": "move",
  "left_power": 0.5,
  "right_power": 0.5
}
```

- `left_power` and `right_power` are floats in the range `[-1.0, 1.0]` representing the power applied to the left and right motors respectively.


#### Claw command

```json
{
  "command": "claw",
  "action": "open"  // or "close"
}
```

- `action` specifies whether to open or close the claws.


#### Subscription command

```json
{
  "command": "subscribe"
}
```

- Subscribes the client to receive sensor measurements.


### Sensor measurements

```json
{
  "timestamp": 123456789,
  "lidar": [
    { "angle": 0.0, "distance": 100.0 },
    { "angle": 1.0, "distance": 95.0 },
    ...
  ],
  "encoders": {
    "left_ticks": 1000,
    "right_ticks": 1000
  }
}
```
