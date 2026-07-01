#!/usr/bin/env python3
"""
fpga_listener.py  --  start the FPGA over UART, capture the Volterra stream,
                      and rebuild the B-mode images (env22 quadratic, env23 cubic).

Protocol (matches mf_bf_uart_top.v):
    PC -> FPGA :  one START byte 'S' (0x53)
    FPGA -> PC :  4-byte preamble  AA 55 AA 55
                  then, per pixel (ax=M..AXIAL-1, lat=0..LATERAL-1):
                      env22 : 4 bytes big-endian (Q2.30 magnitude)
                      env23 : 4 bytes big-endian (Q2.30 magnitude)

Usage:
    python3 fpga_listener.py --port COM5 --baud 115200 --axial 25
    python3 fpga_listener.py --port /dev/ttyUSB0 --baud 921600 --axial 973
"""
import argparse, sys, time
import numpy as np

PREAMBLE = bytes([0xAA, 0x55, 0xAA, 0x55])
Q = 30  # Q2.30

def parse_stream(raw, axial, lateral, m):
    """raw = the 8-bytes/pixel payload AFTER the preamble. Returns env22, env23
    as float arrays shaped (rows, lateral) with rows = axial-m."""
    rows = axial - m
    npix = rows * lateral
    need = npix * 8
    if len(raw) < need:
        raise ValueError(f"short stream: got {len(raw)} bytes, need {need}")
    a = np.frombuffer(raw[:need], dtype=">u4")      # big-endian uint32
    env22 = a[0::2].astype(np.float64) / (1 << Q)
    env23 = a[1::2].astype(np.float64) / (1 << Q)
    return env22.reshape(rows, lateral), env23.reshape(rows, lateral)

def crop_axial(env, args, lo_mm, hi_mm):
    """Crop env (rows=axial-m) to a depth window [lo_mm, hi_mm] using the same
    grid MATLAB uses: depth(row) = start_depth + (row+m)*ddz,
    ddz = (end_depth - start_depth)/(AXIAL-1).  Returns cropped env + (r0,r1)."""
    ddz = (args.end_depth - args.start_depth) / (args.axial - 1)
    def mm_to_row(mm):
        return int(round((mm - args.start_depth) / ddz - args.m))
    r0 = max(0, mm_to_row(lo_mm))
    r1 = min(env.shape[0], mm_to_row(hi_mm) + 1)
    if r1 <= r0:
        print(f"  WARNING: crop window {lo_mm}-{hi_mm}mm gave rows [{r0}:{r1}] -- ignoring crop")
        return env, (0, env.shape[0])
    return env[r0:r1], (r0, r1)

def to_bmode(env, dr_db=80.0, pctl=None, floor=None):
    """Log-compress a magnitude image to 0..255 over a dr_db dynamic range.

    dr_db : dynamic range in dB. 0 dB (brightest) -> 255 (white),
            -dr_db and below -> 0 (black). Default 35 dB.
    pctl  : if set (e.g. 99.5), normalize to that percentile instead of the
            max, so a single hot pixel doesn't crush the whole image to black.
    floor : if set, any pixel with raw magnitude below this is forced to 0
            (black), so empty regions show black instead of amplified noise.
    """
    ref = np.percentile(env, pctl) if pctl else env.max()
    ref = ref if ref > 0 else 1.0
    log = 20.0 * np.log10(env / ref + np.finfo(float).eps)  # 0 dB = ref (matches MATLAB eps)
    log = np.clip(log, -dr_db, 0.0)
    img = (255.0 * (log + dr_db) / dr_db).astype(np.uint8)
    if floor is not None:
        img[env < floor] = 0
    return img

def save_png(img, path):
    try:
        from PIL import Image
        Image.fromarray(img, mode="L").save(path)
    except ImportError:
        import matplotlib.pyplot as plt
        plt.imsave(path, img, cmap="gray", vmin=0, vmax=255)
    print(f"  saved {path}  ({img.shape[0]}x{img.shape[1]})")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None, help="e.g. COM5 or /dev/ttyUSB0 (required unless --raw)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--axial", type=int, default=25)
    ap.add_argument("--lateral", type=int, default=128)
    ap.add_argument("--m", type=int, default=15)
    ap.add_argument("--dr", type=float, default=80.0, help="display dynamic range (dB); 80 matches MATLAB, 35 = higher contrast")
    ap.add_argument("--pctl", type=float, default=None, help="normalize to this percentile (e.g. 99.5) instead of max")
    ap.add_argument("--floor", type=float, default=None, help="raw magnitude below this renders black (e.g. 1e-4)")
    ap.add_argument("--axial-range", dest="axial_range", type=float, nargs=2, default=None,
                    metavar=("LO_MM","HI_MM"), help="crop image to this depth window in mm (e.g. 10 17 = MATLAB FOV)")
    ap.add_argument("--start-depth", dest="start_depth", type=float, default=2.026, help="axial grid start depth (mm), MATLAB start_depth")
    ap.add_argument("--end-depth", dest="end_depth", type=float, default=26.0, help="axial grid end depth (mm), MATLAB hh end")
    ap.add_argument("--out", default="volterra", help="output filename prefix")
    ap.add_argument("--raw", default=None, help="optional: parse this saved .bin instead of reading serial")
    args = ap.parse_args()
    if not args.raw and not args.port:
        ap.error("--port is required unless you pass --raw <file> to replay a saved capture")

    rows = args.axial - args.m
    npix = rows * args.lateral
    need = npix * 8
    print(f"grid: axial={args.axial} (rows={rows}) lateral={args.lateral} "
          f"-> {npix} pixels, {need} payload bytes")

    if args.raw:
        raw = open(args.raw, "rb").read()
    else:
        import serial  # pyserial
        ser = serial.Serial(args.port, args.baud, timeout=2)
        time.sleep(0.2); ser.reset_input_buffer()
        print(f"open {args.port} @ {args.baud}.  sending START...")
        ser.write(b"S")

        # sync on preamble
        win = b""
        t0 = time.time()
        while PREAMBLE not in win:
            b = ser.read(1)
            if not b:
                if time.time() - t0 > 10:
                    print("ERROR: no preamble (timeout). Is the bitstream loaded / SW7 released?")
                    sys.exit(1)
                continue
            win = (win + b)[-4:]
        print("preamble found, reading payload...")

        raw = bytearray()
        binf = open(args.out + "_raw.bin", "wb")   # store incrementally as it streams
        t0 = time.time()
        while len(raw) < need:
            chunk = ser.read(need - len(raw))
            if chunk:
                raw += chunk; binf.write(chunk); binf.flush(); t0 = time.time()
                print(f"\r  {len(raw)}/{need} bytes", end="", flush=True)
            elif time.time() - t0 > 30:
                print("\nERROR: stream stalled (no bytes for 30 s)."); break
        print()
        binf.close()
        ser.close()
        print(f"  saved {args.out}_raw.bin ({len(raw)} bytes)")

    env22, env23 = parse_stream(bytes(raw), args.axial, args.lateral, args.m)
    # full-grid .npy always saved (so you can re-crop later without re-streaming)
    np.save(args.out + "_env22.npy", env22)
    np.save(args.out + "_env23.npy", env23)
    suffix = ""
    if args.axial_range:
        lo_mm, hi_mm = args.axial_range
        env22, rng = crop_axial(env22, args, lo_mm, hi_mm)
        env23, _   = crop_axial(env23, args, lo_mm, hi_mm)
        suffix = f"_{lo_mm:g}-{hi_mm:g}mm"
        print(f"  cropped to {lo_mm:g}-{hi_mm:g} mm -> rows [{rng[0]}:{rng[1]}] "
              f"({env22.shape[0]} rows). Normalization now uses ONLY this window.")
        np.save(args.out + "_env22" + suffix + ".npy", env22)
        np.save(args.out + "_env23" + suffix + ".npy", env23)
    save_png(to_bmode(env22, args.dr, args.pctl, args.floor), args.out + "_env22_quadratic" + suffix + ".png")
    save_png(to_bmode(env23, args.dr, args.pctl, args.floor), args.out + "_env23_cubic" + suffix + ".png")
    print("done.")

if __name__ == "__main__":
    main()