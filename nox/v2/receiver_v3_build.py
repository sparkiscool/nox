"""Build-only pass for receiver v3 from the pruned layout (rcv3_pruned.npz) with the drawn side half cups."""
import sys, os, time
os.environ.setdefault("PYPARDISO_MKL_RT", "/usr/local/lib/libmkl_rt.so.3")
sys.argv = ["receiver_v3.py", "--none"]
exec(open("receiver_v3.py").read().split('if __name__ == "__main__":')[0])
t0 = time.time()
d = np.load(sys.argv_dens if False else "rcv3_pruned.npz"); dens = d["rho_phys"].copy()
dens[pb0.void] = 0.0; dens[pb0.solid] = 1.0
log(f"drawn now {drawn.volume()*1.27e-3:.1f} g exact; side cups {side_cups.volume()*1.27e-3:.2f} g")
con = NO.connectivity(dens, pb0); log(f"connectivity: {con}")
part = build(dens); P = tm(part)
thin = NO.thin_check(P, tm(drawn - cuts), FLOOR, log=log)
bodies = len(part.decompose())
fc_ = NO.function_check(P, drawn - cuts, cuts, allowed, log=log)
wr = NO.wrinkle(P, drawn=tm(drawn - cuts), allowed=tm(allowed))
parts_ = sorted(P.split(only_watertight=False), key=lambda q: -abs(q.volume))
specks = [abs(q.volume) for q in parts_[1:] if abs(q.volume) < 2.0]
if len(specks) == len(parts_) - 1 and specks:      # only sub-2 mm3 smoothing slivers off the main body: unprintable, dropped
    log(f"dropped {len(specks)} slivers under 2 mm3 (total {sum(specks):.2f} mm3)"); P = parts_[0]; bodies = 1
P.export("s2_receiver_v3.stl")
print(f"BUILD bodies {bodies} watertight {P.is_watertight} mass {P.volume / 1000 * 1.27:.1f} g | function "
      f"{fc_[0]:.1f}/{fc_[1]:.1f}/{fc_[2]:.1f} | thin {len(thin)} {thin} | wrinkle {wr[0]:.2f}/{wr[1]:.2f} | {time.time()-t0:.0f} s", flush=True)
