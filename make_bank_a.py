#!/usr/bin/env python3
"""
make_bank_a.py -- unified x2/CSV/mem -> golden_bank.mem -> bank_a.coe pipeline.

Consolidates x2_to_bank.py + make_bank_a_coe.py into one tool. Input type is
auto-detected from the file extension:

    .mat        MATLAB v7.3 file, variable 'x2' (shape [L,nch] or [nch,L])
    .csv        Plain numeric CSV, one frame, shape [samples, nch] or [nch, samples]
                (e.g. a squeeze(data(:,:,k)) frame written by writematrix)
    .mem        Already-built golden_bank.mem -- skips quantization, goes
                straight to .coe.

bank_a layout (must match the RTL): addr = (ch << 11) | sample, value = Q2.30.
Depth = nch * stride (default 128 * 2048 = 262144). Unwritten slots -> 0.

Usage:
    # from MATLAB export (save('matlab_x2.mat','x2','-v7.3'))
    python3 make_bank_a.py --in matlab_x2.mat

    # from a CSV frame (default input if --in omitted)
    python3 make_bank_a.py --in output.csv

    # from an existing .mem, just (re)generate the .coe
    python3 make_bank_a.py --in golden_bank.mem --coe-out bank_a.coe

    # skip .coe generation, only write the .mem
    python3 make_bank_a.py --in output.csv --no-coe
"""
import argparse
import re
from pathlib import Path

import numpy as np


# --------------------------------------------------------------------------
# Stage 1: load raw samples from .mat or .csv into a [nch, L] float array
# --------------------------------------------------------------------------
def load_samples(path, var, nch, L):
    ext = path.suffix.lower()

    if ext in (".mat", ".h5", ".hdf5"):
        import h5py
        with h5py.File(path, "r") as h:
            if var not in h:
                raise KeyError(f"variable '{var}' not found in {path} "
                                f"(available: {list(h.keys())})")
            x2 = np.squeeze(np.array(h[var]))   # MATLAB (L,nch) -> HDF5 (nch,L)
        if x2.shape[0] == L:
            x2 = x2.T

    elif ext == ".csv":
        x2 = np.loadtxt(path, delimiter=",")
        if x2.ndim != 2:
            raise ValueError(f"{path}: expected a 2D frame, got shape {x2.shape}")
        # CSV frames from writematrix(squeeze(data(:,:,k))) are [samples, nch].
        # Normalize to [nch, samples].
        if x2.shape[1] == nch and x2.shape[0] != nch:
            x2 = x2.T
        elif x2.shape[0] != nch and x2.shape[1] != nch:
            print(f"warning: neither dim of {x2.shape} matches --nch {nch}; "
                  f"assuming already [nch, samples]")

    else:
        raise ValueError(f"unrecognized input extension '{ext}' (expected .mat/.csv/.mem)")

    return x2.astype(np.float64)


# --------------------------------------------------------------------------
# Stage 2: quantize to Q2.30 and pack channel-major into a .mem
# --------------------------------------------------------------------------
def samples_to_mem(x2, out_path, nch, L, stride, scale):
    x2 = x2[:nch, :L]
    if x2.shape != (nch, L):
        raise ValueError(f"after slicing, expected shape ({nch},{L}), got {x2.shape}")

    peak = np.abs(x2).max()
    # Q2.30 range is +-2. Auto-scale so the peak sits at ~0.5 (1/4 full scale) for headroom.
    if scale is None:
        scale = 0.5 / peak if peak > 0 else 1.0
    print(f"x2 peak |val| = {peak:.4g}   applied scale = {scale:.4g}   "
          f"(scaled peak {peak*scale:.3f} of +-2 FS)")

    q = np.round(x2 * scale * (1 << 30)).astype(np.int64)
    sat = np.mean((q > (1 << 31) - 1) | (q < -(1 << 31))) * 100
    q = np.clip(q, -(1 << 31), (1 << 31) - 1)
    print(f"saturated words: {sat:.3f}%")

    out = np.zeros((nch, stride), dtype=np.int64)
    out[:, :L] = q
    flat = out.reshape(-1)                       # row-major: ch outer, sample inner
    u = (flat & 0xFFFFFFFF).astype(np.uint32)

    with open(out_path, "w") as f:
        f.write("\n".join(f"{w:08X}" for w in u) + "\n")
    print(f"wrote {out_path}: {u.size} words (expect {nch*stride})")
    print("IMPORTANT: remember this scale factor -- it sets the bank_a amplitude and "
          "feeds the Volterra nonlinearity (quad ~scale^2, cubic ~scale^3).")
    return scale


# --------------------------------------------------------------------------
# Stage 3: .mem -> .coe
# --------------------------------------------------------------------------
def load_mem(path, depth):
    """Robust $readmemh-style loader: handles @addr, // and /* */ comments,
    and multiple hex words per line."""
    vals = [0] * depth
    addr = 0
    with open(path) as f:
        text = f.read()
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    for line in text.splitlines():
        line = line.split("//")[0].strip()
        if not line:
            continue
        for tok in line.split():
            if tok.startswith("@"):
                addr = int(tok[1:], 16)
                continue
            if addr < depth:
                vals[addr] = int(tok, 16) & 0xFFFFFFFF
            addr += 1
    return vals


def mem_to_coe(mem_path, coe_path, depth, bits):
    vals = load_mem(mem_path, depth)
    nz = sum(1 for v in vals if v)
    SH = 9                       # keep bits [24:9] -> 16-bit signed, when narrowing
    digits = bits // 4
    with open(coe_path, "w") as f:
        f.write("memory_initialization_radix=16;\n")
        f.write("memory_initialization_vector=\n")
        for i, v in enumerate(vals):
            if bits < 32:
                sv = v - (1 << 32) if v & 0x80000000 else v
                sv >>= SH
                sv = max(-32768, min(32767, sv))
                w = sv & 0xFFFF
            else:
                w = v
            sep = ";" if i == len(vals) - 1 else ","
            f.write(f"{w:0{digits}X}{sep}\n")
    print(f"wrote {coe_path}: {depth} words ({nz} non-zero)")


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--in", dest="infile", default="output.csv",
                     help="input file: .mat, .csv, or .mem (default: output.csv)")
    ap.add_argument("--var", default="x2", help="variable name inside .mat (default: x2)")
    ap.add_argument("--mem-out", default="golden_bank.mem")
    ap.add_argument("--coe-out", default="bank_a.coe")
    ap.add_argument("--nch", type=int, default=128)
    ap.add_argument("--L", type=int, default=1792, help="valid samples per channel")
    ap.add_argument("--stride", type=int, default=2048, help="words per channel in bank_a")
    ap.add_argument("--scale", type=float, default=None,
                     help="multiply samples by this before Q2.30 (default: auto)")
    ap.add_argument("--bits", type=int, default=32, choices=(16, 32),
                     help=".coe word width (default: 32, matches current RTL)")
    ap.add_argument("--no-coe", action="store_true", help="only write the .mem, skip .coe")
    args = ap.parse_args()

    infile = Path(args.infile)
    depth = args.nch * args.stride

    if infile.suffix.lower() == ".mem":
        mem_path = infile
        print(f"input is already a .mem -- skipping quantization")
    else:
        x2 = load_samples(infile, args.var, args.nch, args.L)
        samples_to_mem(x2, args.mem_out, args.nch, args.L, args.stride, args.scale)
        mem_path = Path(args.mem_out)

    if not args.no_coe:
        mem_to_coe(mem_path, args.coe_out, depth, args.bits)


if __name__ == "__main__":
    main()
