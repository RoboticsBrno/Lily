# Protocol for sending commands to the visualizer

## Overview

The visualizer receives data from a byte stream and displays it on the screen. The data is sent in the form of commands, which consist of a command byte and a variable number of arguments, strictly depending on the command. The commands do not offer any feedback. Data frame synchronization is achieved by using the COBS algorithm.

## Command structure

The command structure is as follows:

| Overhead | Command byte | Arguments   | Delimiter (0x00) |
|----------|--------------|-------------|------------------|
| 1 byte   | 1 byte       | 0-254 bytes | 1 byte           |

The number of arguments and their meaning depends on the command byte. The delimiter is always `0x00`.

Arguments at the end of the command can be optional. If an argument is optional, it is marked with a `?` in the argument list.

## Argument types

The arguments can be of the following types:

| Type id | Type           | Size (bytes) | Description             |
|---------|----------------|--------------|-------------------------|
| `0x00`  | `nil`          | 0            | Null                    |
| `0x01`  | `uint8`        | 1            | Unsigned 8-bit integer  |
| `0x02`  | `int8`         | 1            | Signed 8-bit integer    |
| `0x03`  | `uint16`       | 2            | Unsigned 16-bit integer |
| `0x04`  | `int16`        | 2            | Signed 16-bit integer   |
| `0x05`  | `uint32`       | 4            | Unsigned 32-bit integer |
| `0x06`  | `int32`        | 4            | Signed 32-bit integer   |
| `0x07`  | `string`       | variable     | Null-terminated string  |
| `0x08`  | `rgb332`       | 1            | 8-bit rgb color         |
| `0x09`  | `type`         | 1            | Type identifier         |

## Command list

The following commands are available:

| Command byte | Name             | Description                                                                 |
|--------------|------------------|-----------------------------------------------------------------------------|
| `0x01`       | `CLEAR_SCREEN`   | Clears the screen.                                                          |
| `0x02`       | `CLEAR_LAYER`    | Clears the specified layer.                                                 |
| `0x10`       | `DRAW_POINT`     | Draws a point.                                                              |
| `0x11`       | `DRAW_LINE`      | Draws a line.                                                               |
| `0x12`       | `DRAW_RECTANGLE` | Draws a rectangle.                                                          |
| `0x13`       | `DRAW_CIRCLE`    | Draws a circle.                                                             |
| `0x30`       | `KV_SET`         | Sets a key-value pair.                                                      |
| `0x31`       | `KV_DELETE`      | Deletes a key-value pair.                                                   |
| `0x32`       | `KV_CLEAR`       | Clears the key-value store.                                                 |
| `0x40`       | `CONFIG_ADD`     | Adds a configuration item.                                                  |
| `0x41`       | `CONFIG_SET`     | Sets a configuration item.                                                  |

## Command details

### `CLEAR_SCREEN`

Clears the screen.

No arguments.

### `CLEAR_LAYER`

Clears the specified layer.

| Argument | Type    | Description |
|----------|---------|-------------|
| `layer`  | `uint8` | Layer index |

### `DRAW_POINT`

Draws a point.

| Argument  | Type       | Description            |
|-----------|------------|------------------------|
| `x`       | `uint16`   | X coordinate           |
| `y`       | `uint16`   | Y coordinate           |
| `layer`   | `uint8`    | Layer index            |
| `color?`  | `rgb332`   | Color (default: black) |
| `radius?` | `uint16`   | Radius (default: 1)    |

### `DRAW_LINE`

Draws a line.

| Argument | Type       | Description                      |
|----------|------------|----------------------------------|
| `x1`     | `uint16`   | X coordinate of the first point  |
| `y1`     | `uint16`   | Y coordinate of the first point  |
| `x2`     | `uint16`   | X coordinate of the second point |
| `y2`     | `uint16`   | Y coordinate of the second point |
| `layer`  | `uint8`    | Layer index                      |
| `color?` | `rgb332`   | Color (default: black)           |
| `width?` | `uint16`   | Width (default: 1)               |

### `DRAW_RECTANGLE`

Draws a rectangle.

| Argument | Type       | Description                         |
|----------|------------|-------------------------------------|
| `x1`     | `uint16`   | X coordinate of the first point     |
| `y1`     | `uint16`   | Y coordinate of the first point     |
| `x2`     | `uint16`   | X coordinate of the second point    |
| `y2`     | `uint16`   | Y coordinate of the second point    |
| `layer`  | `uint8`    | Layer index                         |
| `color?` | `rgb332`   | Color (default: black)              |
| `width?` | `uint16`   | Width (default: 1)                  |

### `DRAW_CIRCLE`

Draws a circle.

| Argument | Type       | Description                      |
|----------|------------|----------------------------------|
| `x`      | `uint16`   | X coordinate of the center point |
| `y`      | `uint16`   | Y coordinate of the center point |
| `r`      | `uint16`   | Radius                           |
| `layer`  | `uint8`    | Layer index                      |
| `color?` | `rgb332`   | Color (default: black)           |
| `width?` | `uint16`   | Width (default: 1)               |

### `KV_SET`

Sets a key-value pair.

| Argument | Type     | Description |
|----------|----------|-------------|
| `key`    | `string` | Key         |
| `T`      | `type`   | Type        |
| `value`  | `T`      | Value       |

### `KV_DELETE`

Deletes a key-value pair.

| Argument | Type     | Description |
|----------|----------|-------------|
| `key`    | `string` | Key         |

### `KV_CLEAR`

Clears the key-value store.

No arguments.

### `CONFIG_ADD`

Adds a configuration item.

| Argument | Type     | Description |
|----------|----------|-------------|
| `key`    | `string` | Key         |
| `T`      | `type`   | Type        |
| `value`  | `T`      | Value       |

### `CONFIG_SET`

Sets a configuration item. The configuration item must already exist. The value is interpreted as the type of the configuration item.

| Argument | Type     | Description |
|----------|----------|-------------|
| `key`    | `string` | Key         |
| `value`  | `T`      | Value       |
