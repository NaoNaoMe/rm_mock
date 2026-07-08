"""
RM protocol MCU-side mock (slave) over TCP, for dry-run rehearsal without hardware.

RM Classic, opened in LocalNet mode, connects to this mock as a TCP client and
speaks the same SLIP-framed RM binary protocol it would speak to a real MCU:

    rmApplication.exe   ->  loadview / open <pass>,LocalNet,Byte4,<baud>,<ip>,<port>
                            -> (TCP) -> this mock

The mock holds a flat ADDRESS SPACE (RH850 RAM region) and implements the slave
handlers from rm_core.c: ReadInfo, StartLog, StopLog, SetTimeStep, Write, SetAddr,
ReadDump, plus autonomous log-frame emission while logging.

To feel like the real board, it mirrors sketch.c: test_count (0xfef00014, u32)
auto-increments ~1 per ms while isCountUp (0xfef00018, u8) is non-zero.

Run:  python rm_mock.py [--host 127.0.0.1] [--port 5005] [--quiet]
"""
import argparse
import socket
import sys
import threading
import time

# --- Address space ---------------------------------------------------------
RAM_BASE = 0xFEF00000
RAM_SIZE = 0x4000  # 16 KiB, covers the simple firmware and the FOC symbols

# Mirrored sketch.c symbols (addresses from the generated .rmmap).
ADDR_IS_RESET = 0xFEF0000C  # u8
ADDR_TEST_COUNT = 0xFEF00014  # u32, auto-increment
ADDR_IS_COUNT_UP = 0xFEF00018  # u8, gate for test_count

VERSION_STRING = b"RmSample"
PASS_KEY = 0x0000FFFF

# --- SLIP framing (matches CommProtocol.cs) --------------------------------
END = 0xC0
ESC = 0xDB
ESC_END = 0xDC
ESC_ESC = 0xDD

# --- CRC-8: poly 0xD5, init 0x00, MSB-first (matches rm_core.c) ------------
def _build_crc_table():
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = ((c << 1) ^ 0xD5) & 0xFF if (c & 0x80) else (c << 1) & 0xFF
        table.append(c)
    return table


CRC_TABLE = _build_crc_table()


def crc8(data):
    c = 0
    for b in data:
        c = CRC_TABLE[c ^ b]
    return c


# Instruction codes (lower nibble of OpCode).
INSTR_START_LOG = 0x01
INSTR_STOP_LOG = 0x02
INSTR_SET_PERIOD = 0x03
INSTR_WRITE = 0x04
INSTR_SET_ADDR = 0x05
INSTR_READ_INFO = 0x06
INSTR_READ_DUMP = 0x07

MAX_PAYLOAD = 128


class SlipDecoder:
    """Byte-at-a-time SLIP decoder; yields complete raw frames (incl. CRC)."""

    def __init__(self):
        self.receiving = False
        self.found_esc = False
        self.buf = bytearray()

    def feed(self, byte):
        frames = []
        if byte == END:
            if not self.receiving:
                self.receiving = True
                self.buf.clear()
            else:
                if len(self.buf) == 0:
                    self.receiving = True  # back-to-back END
                else:
                    self.receiving = False
                    if len(self.buf) >= 2:
                        frames.append(bytes(self.buf))
                    self.buf.clear()
        elif self.found_esc:
            self.found_esc = False
            if byte == ESC_END:
                self.buf.append(END)
            elif byte == ESC_ESC:
                self.buf.append(ESC)
        elif byte == ESC:
            self.found_esc = True
        elif self.receiving:
            self.buf.append(byte)
        return frames


def slip_encode(raw):
    out = bytearray([END])
    for b in raw:
        if b == END:
            out += bytes([ESC, ESC_END])
        elif b == ESC:
            out += bytes([ESC, ESC_ESC])
        else:
            out.append(b)
    out.append(END)
    return bytes(out)


class RmMock:
    def __init__(self, quiet=False):
        self.mem = bytearray(RAM_SIZE)
        self.quiet = quiet
        # protocol state
        self.is_approved = False
        self.is_logging = False
        self.mas_cnt = 0x00
        self.slv_cnt = 0x01
        self.period_ms = 500
        self.log_sizes = []
        self.log_addrs = []
        self.write_idx = 0
        self.last_frag = 0
        self._req = b""  # last received frame, to avoid echo-identical responses
        # init mirrored symbols
        self._w(ADDR_IS_COUNT_UP, 1, 1)
        self._w(ADDR_TEST_COUNT, 4, 0)
        self._w(ADDR_IS_RESET, 1, 0)

    # --- memory helpers ---
    def _in_range(self, addr, size):
        return RAM_BASE <= addr and (addr + size) <= (RAM_BASE + RAM_SIZE)

    def _r(self, addr, size):
        if not self._in_range(addr, size):
            return 0
        off = addr - RAM_BASE
        return int.from_bytes(self.mem[off:off + size], "little")

    def _w(self, addr, size, value):
        if not self._in_range(addr, size):
            return
        off = addr - RAM_BASE
        self.mem[off:off + size] = int(value).to_bytes(size, "little")

    def _read_bytes(self, addr, size):
        if not self._in_range(addr, size):
            return bytes(size)
        off = addr - RAM_BASE
        return bytes(self.mem[off:off + size])

    def log(self, *a):
        if not self.quiet:
            print(*a, flush=True)

    # --- slave counter ---
    def _next_slv(self):
        self.slv_cnt += 1
        if self.slv_cnt > 0x0F:
            self.slv_cnt = 0x01
        return self.slv_cnt

    # --- handlers: return payload bytes for a response, or None for silent/drop ---
    def handle(self, frame):
        """frame = raw bytes incl. CRC. Returns response payload bytes, or None."""
        if crc8(frame) != 0:
            self.log("  [drop] bad CRC")
            return None  # silently discard

        self._req = frame
        body = frame[:-1]  # strip CRC
        opcode = body[0]
        instr = opcode & 0x0F
        mas = opcode & 0xF0
        payload = body[1:]

        if not self.is_approved and instr != INSTR_READ_INFO:
            return None  # only ReadInfo before auth

        status, resp_payload = self._dispatch(instr, payload)
        if status == "err":
            return None
        # Any accepted frame echoes MasCnt and resets the keepalive timeout.
        self.mas_cnt = mas
        if status == "silent":
            return None
        return self._build_response(resp_payload)

    def _dispatch(self, instr, payload):
        if instr == INSTR_READ_INFO:
            if len(payload) != 4 or int.from_bytes(payload, "little") != PASS_KEY:
                self.is_approved = False
                return "err", None
            self.is_approved = True
            self.is_logging = False
            self.log("  ReadInfo: approved")
            return "ok", VERSION_STRING

        if instr == INSTR_START_LOG:
            if len(payload) != 0:
                return "err", None
            if self.is_logging:
                return "silent", None  # keepalive
            self.is_logging = True
            self.log("  StartLog")
            return "ok", b""

        if instr == INSTR_STOP_LOG:
            if len(payload) != 0:
                return "err", None
            self.is_logging = False
            self.log("  StopLog")
            return "ok", b""

        if instr == INSTR_SET_PERIOD:
            if len(payload) != 2:
                return "err", None
            period = int.from_bytes(payload, "little")
            if period == 0:
                return "err", None
            self.period_ms = period
            self.is_logging = False
            self.log(f"  SetTimeStep: {period} ms")
            return "ok", b""

        if instr == INSTR_WRITE:
            return self._handle_write(payload)

        if instr == INSTR_SET_ADDR:
            return self._handle_set_addr(payload)

        if instr == INSTR_READ_DUMP:
            return self._handle_read_dump(payload)

        return "err", None

    def _handle_write(self, payload):
        # [ size | addr(4 LE) | value(size LE) ]
        if len(payload) < 6:
            return "err", None
        size = payload[0]
        if size not in (1, 2, 4):
            return "err", None
        if len(payload) != 1 + 4 + size:
            return "err", None
        addr = int.from_bytes(payload[1:5], "little")
        value = int.from_bytes(payload[5:5 + size], "little")
        self._w(addr, size, value)
        self.log(f"  Write: [{addr:#010x}] = {value} ({size}B)")
        if self.is_logging:
            return "silent", None
        return "ok", b""

    def _handle_set_addr(self, payload):
        # [ frameCode | (size, addr(4))* ]
        if len(payload) < 1:
            return "err", None
        frame_code = payload[0]
        begin = (frame_code & 0x10) != 0
        end = (frame_code & 0x20) != 0
        frag = frame_code & 0x0F
        elem_bytes = payload[1:]
        if len(elem_bytes) % 5 != 0:
            return "err", None

        if begin:
            self.write_idx = 0
            self.last_frag = 0
            self.log_sizes = []
            self.log_addrs = []
        elif frag == self.last_frag:
            return "ok", b""  # duplicate
        elif frag != self.last_frag + 1:
            return "err", None
        self.last_frag = frag

        for i in range(0, len(elem_bytes), 5):
            size = elem_bytes[i]
            if size not in (1, 2, 4):
                return "err", None
            addr = int.from_bytes(elem_bytes[i + 1:i + 5], "little")
            self.log_sizes.append(size)
            self.log_addrs.append(addr)
            self.write_idx += 1

        if end:
            self.log(f"  SetAddr: {len(self.log_addrs)} var(s) "
                     f"{[hex(a) for a in self.log_addrs]}")
        self.is_logging = False
        return "ok", b""

    def _handle_read_dump(self, payload):
        # [ addr(4 LE) | size(1) ]
        if len(payload) != 5:
            return "err", None
        addr = int.from_bytes(payload[0:4], "little")
        size = payload[4]
        if size > MAX_PAYLOAD:
            return "err", None
        self.is_logging = False
        data = self._read_bytes(addr, size)
        self.log(f"  ReadDump: [{addr:#010x}] x{size} -> {data.hex().upper()}")
        return "ok", data

    def _build_response(self, payload):
        # Avoid producing a frame byte-identical to the request, which the host
        # would treat as EchoDetected (loopback). Advance slv_cnt until distinct.
        for _ in range(0x0F):
            opcode = (self.mas_cnt + self._next_slv()) & 0xFF
            frame = bytearray([opcode]) + bytes(payload)
            frame.append(crc8(frame))
            if bytes(frame) != self._req:
                break
        return bytes(frame)

    def build_log_frame(self):
        if not self.log_addrs:
            return None
        payload = bytearray()
        for size, addr in zip(self.log_sizes, self.log_addrs):
            payload += int(self._r(addr, size)).to_bytes(size, "little")
        opcode = 0x00 | self._next_slv()
        frame = bytearray([opcode]) + payload
        frame.append(crc8(frame))
        return bytes(frame)

    def tick_counter(self, elapsed_ms):
        """Mirror sketch.c: test_count += elapsed_ms while isCountUp != 0."""
        if self._r(ADDR_IS_COUNT_UP, 1) != 0 and elapsed_ms > 0:
            tc = (self._r(ADDR_TEST_COUNT, 4) + elapsed_ms) & 0xFFFFFFFF
            self._w(ADDR_TEST_COUNT, 4, tc)


def _handle_connection(conn, peer, quiet, stop):
    mock = RmMock(quiet=quiet)
    decoder = SlipDecoder()
    conn.settimeout(0.005)
    last = time.monotonic()
    last_emit = last
    last_keepalive = last
    try:
        while not stop.is_set():
            now = time.monotonic()
            elapsed_ms = int((now - last) * 1000)
            if elapsed_ms > 0:
                mock.tick_counter(elapsed_ms)
                last = now

            try:
                data = conn.recv(4096)
                if not data:
                    break
                for b in data:
                    for frame in decoder.feed(b):
                        last_keepalive = time.monotonic()
                        resp = mock.handle(frame)
                        if resp is not None:
                            conn.sendall(slip_encode(resp))
            except socket.timeout:
                pass

            # Autonomous log emission and keepalive timeout.
            if mock.is_logging:
                if (now - last_keepalive) * 1000 >= 2000:
                    mock.is_logging = False
                    mock.log("  [keepalive timeout] logging stopped")
                elif (now - last_emit) * 1000 >= mock.period_ms:
                    last_emit = now
                    lf = mock.build_log_frame()
                    if lf is not None:
                        conn.sendall(slip_encode(lf))
    except (ConnectionResetError, OSError):
        pass
    finally:
        conn.close()
        print(f"disconnected: {peer}", flush=True)


def serve(host, port, quiet):
    stop = threading.Event()

    # Quit when the console gets a real line (Enter). EOF is ignored, so a
    # background/piped launch (stdin closed) keeps running instead of exiting
    # immediately — no `sleep infinity | ...` workaround needed.
    def _watch_stdin():
        try:
            line = sys.stdin.readline()
        except Exception:
            line = ""
        if line != "":          # a real keypress -> quit; "" == EOF -> ignore
            stop.set()
    threading.Thread(target=_watch_stdin, daemon=True).start()
    hint = "  press Enter to quit (or Ctrl+C)"

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((host, port))
    srv.listen(1)
    srv.settimeout(0.3)  # so accept() unblocks to check the stop flag
    print(f"RM mock listening on {host}:{port} "
          f"(address space {RAM_BASE:#010x}..{RAM_BASE + RAM_SIZE:#010x})", flush=True)
    print(hint, flush=True)

    try:
        while not stop.is_set():
            try:
                conn, peer = srv.accept()
            except socket.timeout:
                continue
            print(f"connected: {peer}", flush=True)
            _handle_connection(conn, peer, quiet, stop)
    except KeyboardInterrupt:
        pass
    finally:
        srv.close()
        print("mock stopped", flush=True)


def main():
    ap = argparse.ArgumentParser(description="RM protocol MCU-side mock over TCP")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()
    serve(args.host, args.port, args.quiet)


if __name__ == "__main__":
    main()
