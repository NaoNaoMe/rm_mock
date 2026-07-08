# rm_mock: A Software MCU for RM Classic

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

**rm_mock** is a software stand-in for an embedded controller that speaks the
[RM Classic](https://github.com/NaoNaoMe/RM-Classic) protocol over TCP. It lets you
**rehearse and learn the RM Classic remote workflow with no hardware** — no MCU,
no serial cable, no flashing.

RM Classic, opened in **LocalNet** mode, connects to the mock as if it were a real
device reached over a WiFi-UART bridge. The mock holds an **address space** and
answers the RM binary protocol (SLIP framing, CRC-8, ReadInfo / StartLog / StopLog /
SetTimeStep / Write / SetAddr / ReadDump, plus autonomous log frames) just like real
firmware. A sample counter even runs on its own so live monitoring looks real.

## Requirements

- Python 3.7+ (standard library only — no dependencies)

## Usage

1. **Start the mock:**

   ```
   python rm_mock.py
   ```

   Options: `--host 127.0.0.1` `--port 5005` `--quiet`.

   To stop: press **Enter** in its console, or **Ctrl+C**. Launched in the
   background or via a pipe (no interactive console), it just keeps running —
   stop it by ending the process; no stdin workaround is needed.

2. **Point RM Classic at it** — in RM Classic, set the connection to **LocalNet**:

   | Setting        | Value         |
   |----------------|---------------|
   | Address / Port | `127.0.0.1` / `5005` |
   | Password       | `0x0000FFFF`  |
   | Address width  | `Byte4` (4 bytes) |

3. **Load the view** — open `MockTest--RmSample.rmxml`. The version string
   **`RmSample`** confirms a successful connection.

4. **Try it** — `test_count` increments continuously. Write **`CountDisable`** to
   freeze it and **`CountEnable`** to resume (named write macros in the view).

You can also drive the mock through RM Classic's remote (TCP) interface from a
script — see the RM Classic repository for the command catalog (`help`).

## Address Space

The mock exposes a flat RAM region (base `0xFEF00000`, 16 KiB). The sample symbols
mirror a small reference firmware:

| Symbol            | Address      | Size | Notes                                        |
|-------------------|--------------|------|----------------------------------------------|
| `test_count`      | `0xFEF00014` | 4    | Free-running counter (~1 per ms)             |
| `isCountUp`       | `0xFEF00018` | 1    | Gate: `1` = count, `0` = freeze `test_count` |
| `isResetRquested` | `0xFEF0000C` | 1    | Reset-request flag (held in RAM)             |

## Files

| File                          | Purpose                                          |
|-------------------------------|--------------------------------------------------|
| `rm_mock.py`                  | The mock (TCP server, RM protocol slave)         |
| `MockTest--RmSample.rmxml`    | RM Classic view: monitored vars + write macros   |
| `MockTest.rmmap`              | Symbol map (`symbol,address,offset,size`)         |

## Protocol

The wire format is documented in
[RM_Protocol_Specification.md](https://github.com/NaoNaoMe/RM-Classic/blob/main/RM_Protocol_Specification.md)
in the RM Classic repository. The mock implements the MCU (slave) side of it.

## Related

- [RM Classic](https://github.com/NaoNaoMe/RM-Classic) — the host application.
- [rm_embedded](https://github.com/NaoNaoMe/rm_embedded) — firmware library for real MCUs.

## License

MIT — see [LICENSE](LICENSE).
