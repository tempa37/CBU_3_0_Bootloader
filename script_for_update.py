#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
UPDATER (simple, no UI, no autoconnect)

Два сценария:
  1) MODE = "app"  -> шлём 0x2B на APP_SLAVE_ID, ждём, переключаем UART на BOOT_* и шлём пакеты 0x2A на ID=1
  2) MODE = "boot" -> сразу открываем порт на BOOT_* и шлём пакеты 0x2A на ID=1

ВАЖНО:
  - Файл прошивки отправляется "как есть" (сырые байты), как в твоём GUI.
  - Размер чанка/пакетов и CRC такие же, как в исходнике.
"""

import math
import os
import struct
import sys
import time

import serial

# =========================================================
# ====================== CONFIG (HEADER) ==================
# =========================================================

# РЕЖИМ:
#   "app"  - обновление из приложения (0x2B затем 0x2A)
#   "boot" - обновление из загрузчика (только 0x2A)
MODE = ("app")

# ПОРТ И ФАЙЛ (можно задать тут)
PORT = "COM9"
FIRMWARE_PATH = "firmware.hex"

# Тайминги/надежность
SERIAL_TIMEOUT = 3.0
START_WAIT_SEC = 7.0          # ожидание после 0x2B (актуально для MODE="app")
MAX_RETRIES = 3
RETRY_DELAY_SEC = 1.0

# Лимит размера файла как в UI (можно отключить)
ENABLE_SIZE_CHECK = False
MAX_FIRMWARE_FILE_SIZE = 28 * 1024  # 28 KB

# Настройки UART В РЕЖИМЕ ПРИЛОЖЕНИЯ (актуально для MODE="app")
APP_SLAVE_ID = 1          # Modbus slave id устройства в приложении (куда отправляем 0x2B)
APP_BAUD = 115200
APP_PARITY = "none"       # "none" / "odd" / "even"
APP_STOPBITS = 1          # 1 или 2

# Настройки UART В РЕЖИМЕ ЗАГРУЗЧИКА (актуально для MODE="boot" и для передачи 0x2A)
BOOTLOADER_ID = 1         # ID для кадров 0x2A (в твоём коде = 1)
BOOT_BAUD = 115200
BOOT_PARITY = "none"      # "none" / "odd" / "even"
BOOT_STOPBITS = 1         # 1 или 2

# =========================================================
# =================== PROTOCOL CONSTANTS ==================
# =========================================================

FUNC_CODE_FIRMWARE = 0x2A
FUNC_CODE_START = 0x2B

MAX_PACKET_SIZE = 91
HEADER_SIZE = 5
CRC_SIZE = 2
MAX_PAYLOAD_SIZE = MAX_PACKET_SIZE - HEADER_SIZE - CRC_SIZE  # 84 bytes (как в твоём коде)

BYTESIZE = serial.EIGHTBITS


# =========================================================
# ===================== CRC16 MODBUS ======================
# =========================================================

def calc_crc(data: bytes) -> int:
    crc = 0xFFFF
    for ch in data:
        crc ^= ch
        for _ in range(8):
            if crc & 1:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def parity_from_str(s: str) -> str:
    s = (s or "").strip().lower()
    if s in ("none", "n"):
        return serial.PARITY_NONE
    if s in ("odd", "o"):
        return serial.PARITY_ODD
    if s in ("even", "e"):
        return serial.PARITY_EVEN
    raise ValueError(f"Unsupported parity: {s!r} (use none/odd/even)")


def stopbits_from_int(v: int) -> float:
    if v == 1:
        return serial.STOPBITS_ONE
    if v == 2:
        return serial.STOPBITS_TWO
    raise ValueError("STOPBITS must be 1 or 2")


# =========================================================
# ====================== CORE HELPERS =====================
# =========================================================

def load_firmware(path: str) -> bytes:
    if not os.path.isfile(path):
        raise FileNotFoundError(path)

    if ENABLE_SIZE_CHECK:
        size = os.path.getsize(path)
        if size > MAX_FIRMWARE_FILE_SIZE:
            raise ValueError(f"Firmware too large: {size} > {MAX_FIRMWARE_FILE_SIZE} bytes")

    with open(path, "rb") as f:
        return f.read()


def build_fw_frame(packet_idx: int, total_packets: int, payload: bytes) -> bytes:
    frame = bytearray()
    frame += struct.pack(">BB", BOOTLOADER_ID, FUNC_CODE_FIRMWARE)
    frame += packet_idx.to_bytes(2, "big", signed=False)
    frame += total_packets.to_bytes(2, "big", signed=False)
    frame += payload
    crc = calc_crc(frame)
    frame += crc.to_bytes(2, "little", signed=False)
    return bytes(frame)


def send_start_0x2b(ser: serial.Serial, slave_id: int) -> bool:
    """
    Как в твоём GUI:
      req = [slave_id, 0x2B] + CRC
      читаем 8 байт, считаем успехом если len(resp) >= 4
    """
    req = struct.pack(">BB", slave_id, FUNC_CODE_START)
    crc = calc_crc(req)
    pkt = req + crc.to_bytes(2, "little", signed=False)

    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    ser.write(pkt)
    resp = ser.read(6)
    return len(resp) >= 4


def send_firmware_0x2a(ser: serial.Serial, fw: bytes) -> bool:
    if not fw:
        raise ValueError("Firmware file is empty.")

    total_packets = math.ceil(len(fw) / MAX_PAYLOAD_SIZE)

    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    last_pct = -1

    for i in range(total_packets):
        idx = i + 1
        chunk = fw[i * MAX_PAYLOAD_SIZE : (i + 1) * MAX_PAYLOAD_SIZE]
        frame = build_fw_frame(idx, total_packets, chunk)

        ok = False
        for attempt in range(1, MAX_RETRIES + 1):
            print(f"[0x2A] Packet {idx}/{total_packets} attempt {attempt}")

            try:
                ser.write(frame)
                resp = ser.read(6)  # как в GUI
            except serial.SerialException as exc:
                print(f"  serial error: {exc}")
                resp = b""

            if len(resp) >= 4:
                ok = True
                print(f"  ack ok for {idx}/{total_packets}")
                break

            time.sleep(RETRY_DELAY_SEC)

        if not ok:
            print(f"[ERROR] Failed on packet {idx}/{total_packets}")
            return False

        pct = int(idx * 100 / total_packets)
        if pct != last_pct:
            last_pct = pct
            print(f"Progress: {pct}%")

    return True


# =========================================================
# =========================== RUN =========================
# =========================================================

def run_update(port: str, firmware_path: str) -> int:
    fw = load_firmware(firmware_path)

    if MODE.lower() == "boot":
        print(f"[BOOT] Open {port} @ {BOOT_BAUD} {BOOT_PARITY} stop={BOOT_STOPBITS}")
        try:
            with serial.Serial(
                port=port,
                baudrate=BOOT_BAUD,
                bytesize=BYTESIZE,
                parity=parity_from_str(BOOT_PARITY),
                stopbits=stopbits_from_int(BOOT_STOPBITS),
                timeout=SERIAL_TIMEOUT,
            ) as ser:
                ok = send_firmware_0x2a(ser, fw)
                print("[BOOT] DONE" if ok else "[BOOT] FAILED")
                return 0 if ok else 2
        except serial.SerialException as exc:
            print(f"[BOOT] Serial error: {exc}")
            return 3

    if MODE.lower() == "app":
        print(
            f"[APP] Open {port} @ {APP_BAUD} {APP_PARITY} stop={APP_STOPBITS} "
            f"(slave_id={APP_SLAVE_ID})"
        )
        try:
            with serial.Serial(
                port=port,
                baudrate=APP_BAUD,
                bytesize=BYTESIZE,
                parity=parity_from_str(APP_PARITY),
                stopbits=stopbits_from_int(APP_STOPBITS),
                timeout=SERIAL_TIMEOUT,
            ) as ser:
                print("[APP] Send START 0x2B ...")
                if not send_start_0x2b(ser, APP_SLAVE_ID):
                    print("[APP] START 0x2B failed (no ack).")
                    return 2

                print(f"[APP] Wait {START_WAIT_SEC} sec ...")
                time.sleep(float(START_WAIT_SEC))

                print(f"[APP] Switch UART to boot defaults: {BOOT_BAUD} {BOOT_PARITY} stop={BOOT_STOPBITS}")
                ser.baudrate = BOOT_BAUD
                ser.parity = parity_from_str(BOOT_PARITY)
                ser.stopbits = stopbits_from_int(BOOT_STOPBITS)

                ok = send_firmware_0x2a(ser, fw)
                print("[APP] DONE" if ok else "[APP] FAILED")
                return 0 if ok else 2

        except serial.SerialException as exc:
            print(f"[APP] Serial error: {exc}")
            return 3

    print(f"[ERROR] Unknown MODE={MODE!r}. Use 'app' or 'boot'.")
    return 1


def main() -> int:
    # Не делаем никаких меню/парсеров — просто обновление.
    # Но можно быстро переопределить порт/файл аргументами:
    #   python fw_update_simple.py COM3 firmware.hex
    port = PORT
    fw_path = FIRMWARE_PATH

    if len(sys.argv) >= 2:
        port = sys.argv[1]
    if len(sys.argv) >= 3:
        fw_path = sys.argv[2]

    print(f"=== Firmware update ===")
    print(f"MODE={MODE} PORT={port} FILE={fw_path}")
    return run_update(port, fw_path)


if __name__ == "__main__":
    raise SystemExit(main())
