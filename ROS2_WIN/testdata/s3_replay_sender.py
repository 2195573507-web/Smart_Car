#!/usr/bin/env python3
"""Offline sender for the experimental S3RD TCP envelope.

This utility sends deterministic YDLIDAR-compatible packets to a running
bridge. It never talks to an S3 device and is intended for LAN/replay tests.
"""

import argparse
import socket
import struct
import time


def crc16_modbus(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) if crc & 1 else (crc >> 1)
    return crc


def ydlidar_payload(intensity):
    ct = 0
    sample_count = 2
    fsa = (0 * 64 << 1) | 1
    lsa = (2 * 64 << 1) | 1
    distances = (4000, 8000)
    qualities = (24, 36)
    checksum = 0x55AA ^ (ct | (sample_count << 8)) ^ fsa ^ lsa
    body = bytearray()
    for index, distance in enumerate(distances):
        if intensity:
            quality = qualities[index]
            checksum ^= quality ^ distance
            body.extend(struct.pack("<BH", quality, distance))
        else:
            checksum ^= distance
            body.extend(struct.pack("<H", distance))
    return struct.pack("<2sBBHHH", b"\xAA\x55", ct, sample_count,
                       fsa, lsa, checksum) + body


def s3_frame(sequence, payload, *, version=1, message_type=1, flags=0,
             device_id=1, stream_id=1, bad_crc=False, bad_length=False):
    header = struct.pack("<4sBBHIII IH", b"S3RD", version, message_type,
                         flags, device_id, stream_id, sequence, 1234,
                         len(payload))
    if bad_length:
        # Zero is below the receiver's configured minimum payload length, so
        # this is a deterministic rejected-length test rather than a partial
        # TCP packet that remains ambiguous until the connection closes.
        header = header[:-2] + struct.pack("<H", 0)
    crc = crc16_modbus(header[4:] + payload)
    if bad_crc:
        crc ^= 1
    return header + payload + struct.pack("<H", crc)


def send_once(host, port, chunks, pause=0.0):
    with socket.create_connection((host, port), timeout=3.0) as sock:
        for chunk in chunks:
            sock.sendall(chunk)
            if pause:
                time.sleep(pause)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument(
        "--device-id", type=int, default=1,
        help="S3RD device identity (matches the bridge default)",
    )
    parser.add_argument(
        "--stream-id", type=int, default=1,
        help="S3RD stream identity (matches the bridge default)",
    )
    parser.add_argument("--scenario", required=True, choices=[
        "legal_no_intensity", "legal_intensity", "half", "sticky",
        "bad_crc", "bad_version", "bad_length", "duplicate_jump",
        "reconnect",
    ])
    parser.add_argument("--pause-ms", type=float, default=10.0)
    args = parser.parse_args()
    pause = args.pause_ms / 1000.0
    payload = ydlidar_payload(args.scenario == "legal_intensity")

    def frame(sequence, **kwargs):
        """Build a frame using the identities selected for this run."""
        return s3_frame(
            sequence,
            payload,
            device_id=args.device_id,
            stream_id=args.stream_id,
            **kwargs,
        )

    if args.scenario == "legal_no_intensity":
        send_once(args.host, args.port, [frame(1)])
    elif args.scenario == "legal_intensity":
        send_once(args.host, args.port, [frame(1)])
    elif args.scenario == "half":
        packet = frame(1)
        send_once(args.host, args.port, [packet[:7], packet[7:]], pause)
    elif args.scenario == "sticky":
        send_once(args.host, args.port,
                  [frame(1) + frame(2)])
    elif args.scenario == "bad_crc":
        send_once(args.host, args.port, [frame(1, bad_crc=True)])
    elif args.scenario == "bad_version":
        send_once(args.host, args.port, [frame(1, version=2)])
    elif args.scenario == "bad_length":
        send_once(args.host, args.port, [frame(1, bad_length=True)])
    elif args.scenario == "duplicate_jump":
        send_once(args.host, args.port,
                  [frame(1), frame(1), frame(3)], pause)
    elif args.scenario == "reconnect":
        send_once(args.host, args.port, [frame(1)])
        time.sleep(pause)
        send_once(args.host, args.port, [frame(2)])


if __name__ == "__main__":
    main()
