#!/usr/bin/env python3
"""
Argo Mini — direct serial teleop (no ROS2, no serial_bridge).
Computes L/R RPM from O tick messages at 20 Hz.

  w   forward       s   reverse
  a   pivot left    d   pivot right
  space / k   stop
  q   quit
"""

import sys, tty, termios, threading, time, serial

PORT          = '/dev/ttyUSB1'
BAUD          = 115200
DAC_FWD       = 107
POLE_PAIRS    = 15
TICKS_PER_REV = POLE_PAIRS * 6   # CHANGE mode firmware (90/rev)

lock      = threading.Lock()
rpm_l     = 0.0
rpm_r     = 0.0
total_l   = 0
total_r   = 0
label     = 'STOP'
running   = True

MOVES = {
    'w': ( DAC_FWD,  DAC_FWD,  'FORWARD'),
    's': (-DAC_FWD, -DAC_FWD,  'REVERSE'),
    'a': ( 0,        DAC_FWD,  'PIVOT LEFT'),
    'd': ( DAC_FWD,  0,        'PIVOT RIGHT'),
    ' ': ( 0,        0,        'STOP'),
    'k': ( 0,        0,        'STOP'),
}

def reader(ser):
    global rpm_l, rpm_r, total_l, total_r, running
    prev_lt = prev_rt = prev_ts = None
    start_lt = start_rt = None
    while running:
        try:
            raw = ser.readline().decode('utf-8', errors='ignore').strip()
        except Exception:
            break
        if not raw.startswith('O '):
            continue
        parts = raw.split()
        if len(parts) != 3:
            continue
        lt, rt = int(parts[1]), int(parts[2])
        now = time.monotonic()
        if start_lt is None:
            start_lt, start_rt = lt, rt
        if prev_lt is not None:
            dt = now - prev_ts
            if dt > 0:
                rl = (abs(lt - prev_lt) / TICKS_PER_REV) / dt * 60.0
                rr = (abs(rt - prev_rt) / TICKS_PER_REV) / dt * 60.0
                with lock:
                    rpm_l, rpm_r = rl, rr
                    total_l = abs(lt - start_lt)
                    total_r = abs(rt - start_rt)
        prev_lt, prev_rt, prev_ts = lt, rt, now

def display():
    while running:
        with lock:
            rl, rr, lbl = rpm_l, rpm_r, label
            tl, tr = total_l, total_r
        ratio = tr / tl if tl > 0 else 1.0
        sys.stdout.write(
            f'\r  {lbl:<12}  L:{rl:5.1f}rpm  R:{rr:5.1f}rpm'
            f'  ticks L:{tl:6d} R:{tr:6d}  scale={ratio:.3f}  ')
        sys.stdout.flush()
        time.sleep(0.05)

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def send(ser, l, r):
    try:
        ser.write(f'V {l} {r}\n'.encode())
        ser.flush()
    except Exception:
        pass

def main():
    global label, running
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.05)
    except serial.SerialException as e:
        print(f'Cannot open {PORT}: {e}')
        sys.exit(1)
    time.sleep(2.0)
    ser.reset_input_buffer()

    threading.Thread(target=reader,  args=(ser,), daemon=True).start()
    threading.Thread(target=display, daemon=True).start()

    settings = termios.tcgetattr(sys.stdin)
    print(f"""
==========================================
  ARGO MINI DIRECT TELEOP  ({PORT})
==========================================
  w=forward  s=reverse
  a=pivot-left  d=pivot-right
  space=stop    q=quit
==========================================
""")
    try:
        while True:
            key = get_key(settings)
            if key in ('q', '\x03'):
                break
            if key not in MOVES:
                continue
            dac_l, dac_r, lbl = MOVES[key]
            with lock:
                label = lbl
            send(ser, dac_l, dac_r)
    finally:
        running = False
        send(ser, 0, 0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        print('\n[teleop] stopped.')
        ser.close()

if __name__ == '__main__':
    main()
