#!/usr/bin/env python3
"""pendulum_sysid 시리얼 로거.

Arduino 출력 전체를 파일로 저장하고, 터미널에서 입력한 줄을 명령으로 전송한다.
    python3 tools/serial_log.py /dev/ttyACM0 -o logs/t1.csv
    python3 tools/serial_log.py /dev/ttyACM0 -o logs/t4.csv --quiet   # '#' 줄만 화면 표시

Arduino IDE Serial Monitor 는 반드시 닫고 실행할 것 (포트 공유 불가).
포트를 열면 Mega 가 리셋되므로 약 2초 뒤 'ready' 가 뜬다.
"""
import argparse
import os
import sys
import threading
import time

import serial


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("-b", "--baud", type=int, default=250000)
    ap.add_argument("-o", "--out", required=True)
    ap.add_argument("--quiet", action="store_true", help="데이터 줄은 화면에 표시하지 않음")
    args = ap.parse_args()

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    stop = threading.Event()

    def reader(f):
        while not stop.is_set():
            raw = ser.readline()
            if not raw:
                continue
            text = raw.decode("utf-8", errors="replace")
            if not text.endswith("\n"):
                text += "\n"
            f.write(f"{time.time():.4f}\t{text}")  # PC 수신시각 \t Arduino 원문
            if not args.quiet or text.startswith("#"):
                sys.stdout.write(text)
                sys.stdout.flush()

    with open(args.out, "a", encoding="utf-8") as f:
        t = threading.Thread(target=reader, args=(f,), daemon=True)
        t.start()
        print(f"# logging {args.port} @ {args.baud} -> {args.out}. 명령 입력 후 Enter, Ctrl-D/Ctrl-C 종료")
        try:
            for line in sys.stdin:
                cmd = line.strip()
                f.write(f"{time.time():.4f}\t>>> {cmd}\n")
                ser.write((cmd + "\n").encode())
        except KeyboardInterrupt:
            pass
        finally:
            ser.write(b"0\n")  # 종료 시 IDLE (모터 OFF)
            time.sleep(0.2)
            stop.set()
            t.join(timeout=1)
            ser.close()


if __name__ == "__main__":
    main()
