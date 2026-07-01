#!/usr/bin/env python3
"""
compile_results.py -- final hardware-vs-MATLAB validation report for the
Volterra ultrasound pipeline. Computes CTR, band/bright-spot locations, depth
offset, correlation/error stats, and renders MATLAB vs hardware envelopes side
by side. All input files expected in the same directory (pass --dir).

    python3 compile_results.py --dir . --mat chicon1e6vol3pt2T3.mat \
        --hw22 volterra_env22.npy --hw23 volterra_env23.npy

Outputs: prints a summary table and saves results_summary.png
"""
import argparse, os, numpy as np
import h5py
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ---------- loading ----------
def load_mat(path):
    """Return dict of MATLAB envelopes (axial,lateral) frame0 + hh,ww (mm)."""
    with h5py.File(path,"r") as h:
        def env(k):
            a=np.array(h[k])              # HDF5 dims reversed: (frames, lateral, axial)
            a=np.abs(a)
            if a.ndim==3: a=a[0]          # frame 0 -> (lateral, axial)
            return a.T                    # -> (axial, lateral)
        out={k:env(k+"_all") for k in ["env1","env2","env22","env23"]}
        out["hh"]=np.array(h["hh"]).squeeze()   # axial mm
        out["ww"]=np.array(h["ww"]).squeeze()   # lateral mm
    return out

# ---------- geometry ----------
def first_valid_row(mat):
    rs=mat.sum(axis=1); nz=np.nonzero(rs>rs.max()*1e-9)[0]
    return int(nz[0]) if nz.size else 0

def axial_profile(img): return np.abs(img).mean(axis=1)

# ---------- CTR ----------
def ctr(env, hh_mm, ww_mm, bub=(13.5,14.5), tis=(6.0,7.0), lat=(-10,10)):
    e=np.abs(env)
    def rows(lo,hi): 
        idx=np.where((hh_mm>=lo)&(hh_mm<=hi))[0]; return idx
    cols=np.where((ww_mm>=lat[0])&(ww_mm<=lat[1]))[0]
    br,tr=rows(*bub),rows(*tis)
    if br.size==0 or tr.size==0 or cols.size==0: return float('nan'),0,0
    b=e[np.ix_(br,cols)].mean(); t=e[np.ix_(tr,cols)].mean()
    return (20*np.log10(b/t) if t>0 else float('nan')), b, t

# ---------- main ----------
def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--dir", default=".")
    ap.add_argument("--mat", default="chicon1e6vol3pt2T3.mat")
    ap.add_argument("--hw22", default="volterra_env22.npy")
    ap.add_argument("--hw23", default="volterra_env23.npy")
    ap.add_argument("--m", type=int, default=15)
    ap.add_argument("--mask-deep", dest="mask_deep", type=float, default=20.0,
                    help="ignore HW rows deeper than this mm (aperture-clamp artifact)")
    ap.add_argument("--dr", type=float, default=60.0, help="display dynamic range dB")
    a=ap.parse_args()
    D=lambda f: os.path.join(a.dir,f)

    M=load_mat(D(a.mat))
    hh, ww = M["hh"], M["ww"]
    hw22=np.abs(np.load(D(a.hw22))); hw23=np.abs(np.load(D(a.hw23)))

    # HW axial grid: row r -> axial pixel (r+m) -> hh[r+m]
    def hw_depth(row): 
        j=row+a.m; return hh[j] if j<len(hh) else hh[-1]
    hw_hh = np.array([hw_depth(r) for r in range(hw22.shape[0])])

    # mask deep clamp artifact in HW for band-finding / display
    deep_cut = np.searchsorted(hw_hh, a.mask_deep)
    def mask(e): 
        m=e.copy(); m[deep_cut:]=0; return m
    hw22m, hw23m = mask(hw22), mask(hw23)

    # MATLAB envelopes, align first-valid
    m22,m23 = M["env22"], M["env23"]
    v0=first_valid_row(m22)

    # ---- band (bright spot) depth ----
    def band_mm_hw(e): 
        p=axial_profile(e); r=int(p.argmax()); return hw_hh[r], r
    def band_mm_mat(e):
        p=axial_profile(e); r=int(p.argmax()); return hh[r], r
    hb22,hr22=band_mm_hw(hw22m); hb23,hr23=band_mm_hw(hw23m)
    mb22,mr22=band_mm_mat(m22);  mb23,mr23=band_mm_mat(m23)

    # ---- CTR (identical function both sides) ----
    ctr_h22,_,_=ctr(hw22, hw_hh, ww); ctr_h23,_,_=ctr(hw23, hw_hh, ww)
    ctr_m22,_,_=ctr(m22,  hh,    ww); ctr_m23,_,_=ctr(m23,  hh,    ww)

    # ---- correlation / error (resample MATLAB band region onto HW depth) ----
    def prof_corr(hw_e, mat_e):
        # compare axial profiles over the shallow region (above mask)
        ph=axial_profile(hw_e)[:deep_cut]
        # interp MATLAB profile onto HW depth grid
        pm=np.interp(hw_hh[:deep_cut], hh, axial_profile(mat_e))
        ph=ph/ (ph.max()+1e-12); pm=pm/(pm.max()+1e-12)
        return float(np.corrcoef(ph,pm)[0,1])
    pc22=prof_corr(hw22m,m22); pc23=prof_corr(hw23m,m23)

    # ---- print table ----
    line="="*64
    print(line); print("  VOLTERRA PIPELINE -- HARDWARE vs MATLAB  (final report)"); print(line)
    print(f"{'metric':<28}{'MATLAB':>12}{'HARDWARE':>12}{'Δ':>10}")
    print("-"*64)
    def row(name,mv,hv,unit="",fmt="{:.2f}"):
        d=hv-mv
        print(f"{name:<28}{fmt.format(mv)+unit:>12}{fmt.format(hv)+unit:>12}{('{:+.2f}'.format(d)):>10}")
    row("Quadratic band depth", mb22, hb22, " mm")
    row("Cubic band depth",     mb23, hb23, " mm")
    row("Quadratic CTR",        ctr_m22, ctr_h22, " dB")
    row("Cubic CTR",            ctr_m23, ctr_h23, " dB")
    print("-"*64)
    print(f"{'Quad axial-profile corr':<28}{'':>12}{pc22:>12.3f}")
    print(f"{'Cubic axial-profile corr':<28}{'':>12}{pc23:>12.3f}")
    print(f"{'Quad band depth offset':<28}{'':>12}{hb22-mb22:>+11.2f} mm")
    print(f"{'Cubic band depth offset':<28}{'':>12}{hb23-mb23:>+11.2f} mm")
    print(line)

    # ---- figure: MATLAB vs HW, quad + cubic, side by side ----
    def bmode(e,dr):
        e=np.abs(e); mx=e.max() or 1.0
        log=20*np.log10(e/mx+1e-12); log=np.clip(log,-dr,0)
        return log
    fig,ax=plt.subplots(2,2,figsize=(11,9))
    panels=[("MATLAB Quadratic",m22,hh,ww,ctr_m22),
            ("Hardware Quadratic",hw22m,hw_hh,ww,ctr_h22),
            ("MATLAB Cubic",m23,hh,ww,ctr_m23),
            ("Hardware Cubic",hw23m,hw_hh,ww,ctr_h23)]
    order=[(0,0),(0,1),(1,0),(1,1)]
    for (title,e,yy,xx,c),(i,j) in zip(panels,order):
        im=ax[i,j].imshow(bmode(e,a.dr), aspect="auto", cmap="gray",
                          extent=[xx.min(),xx.max(),yy.max() if len(yy) else 26, yy.min() if len(yy) else 0],
                          vmin=-a.dr, vmax=0)
        ax[i,j].set_title(f"{title}\nCTR = {c:.2f} dB", fontsize=12, fontweight="bold")
        ax[i,j].set_xlabel("Lateral (mm)"); ax[i,j].set_ylabel("Axial (mm)")
        ax[i,j].set_ylim(17,8)   # MATLAB FOV-ish window for fair view
        plt.colorbar(im, ax=ax[i,j], fraction=0.046, pad=0.04)
    fig.suptitle("Volterra Pipeline Validation: Hardware vs MATLAB", fontsize=14, fontweight="bold")
    fig.tight_layout(rect=[0,0,1,0.97])
    out=D("results_summary.png"); fig.savefig(out, dpi=130, bbox_inches="tight")
    print(f"\nsaved figure: {out}")

if __name__=="__main__":
    main()
