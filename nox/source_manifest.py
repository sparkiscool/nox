"""Nox Source manifest update (18 Oct 2026). Every script, config and mesh lives in the artifact "Nox Source"
(https://claude.ai/artifact/WvmWqQXwVzW12N486MBaTA) at its old Project path; MANIFEST.json there lists path, bytes, sha256.

Save flow for any chat (nox/README.md "Where files live"):
  1. Artifact read, url = Nox Source, paths = ["index.html", "MANIFEST.json"]  -> saved to a folder D (the result names it)
  2. copy each changed file to D/<its path>, e.g. D/nox/v2/receiver_v34.py
  3. python3 source_manifest.py D nox/v2/receiver_v34.py [more paths]     (adds or updates their rows; --rm <path> drops one)
  4. Artifact publish, url = Nox Source, file_path = D/index.html, root = D,
     files = [{"path": "<each changed path>", "contentType": "text/plain"}, {"path": "MANIFEST.json", "contentType": "application/json"}]
     (a removed file: also pass it as {"<path>": null} in the map form of files)
A refused publish (someone saved first) hands back the live version: read MANIFEST.json again, rerun step 3, publish again.
python3 source_manifest.py D --check   compares every file present under D with its manifest row."""
import sys, os, json, hashlib

def sha(p):
    return hashlib.sha256(open(p, "rb").read()).hexdigest()

if __name__ == "__main__":
    D = sys.argv[1]; args = sys.argv[2:]
    mp = os.path.join(D, "MANIFEST.json")
    rows = {r["path"]: r for r in json.load(open(mp))}
    if args == ["--check"]:
        bad = 0
        for p, r in sorted(rows.items()):
            f = os.path.join(D, p)
            if os.path.exists(f) and sha(f) != r["sha256"]:
                print("CHANGED", p); bad += 1
        print(f"checked {len(rows)} rows, {bad} differ"); sys.exit(1 if bad else 0)
    rm = False
    for a in args:
        if a == "--rm": rm = True; continue
        if rm:
            rows.pop(a, None); print("removed", a); rm = False; continue
        f = os.path.join(D, a)
        b = open(f, "rb").read()
        old = rows.get(a)
        rows[a] = dict(path=a, bytes=len(b), sha256=hashlib.sha256(b).hexdigest())
        print(("updated " if old else "added   ") + a, len(b), rows[a]["sha256"][:12])
    json.dump(sorted(rows.values(), key=lambda r: r["path"]), open(mp, "w"), indent=0)
    print(f"MANIFEST.json: {len(rows)} files, {sum(r['bytes'] for r in rows.values()) / 1e6:.2f} MB")
