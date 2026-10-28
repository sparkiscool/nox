"""Fill page4v_template.html for P3 / P4 -> runs/xswap/v4p/<key>/flip_swap_<key>.html"""
import json, os, sys
import xswap4 as X, xswap4p as P
key = sys.argv[1]
D = os.path.join(X.V3.ROOT, "runs", "xswap", "v4p", key)
res = json.load(open(os.path.join(X.V3.ROOT, "runs", "xswap", "v4p", "result.json")))[key]
m = res["mass_g"]; twin = P.VARIANTS[key]["twin"]; lid = P.VARIANTS[key]["lid"]
T = {
 "P3": dict(title="Flip Swap P3 Steel Core", eyebrow="Flip swap v4 · pincer P3 · PETG + steel · no print files yet", h1="Flip swap: P3 steel core",
            lede="The v4 swap station with the <b>P3 pincer</b>. The jaw is a base plus a screwed lid, each finger's arm bears up into the lid, and a <b>3 mm steel pin</b> runs up the middle of each finger post. The posts print upright, so PETG is weakest exactly where they are loaded; the steel takes the pull and most of the bending. Shell, latch and holder are v4's, unchanged."),
 "P4": dict(title="Flip Swap P4 Twin Steel", eyebrow="Flip swap v4 · pincer P4 · PETG + steel · no print files yet", h1="Flip swap: P4 twin post + steel",
            lede="The v4 swap station with the <b>P4 pincer</b>, the strongest option. Each finger has <b>two posts</b>: the drive post that swings the gate bar and an anchor post 18 mm further out in its own keyhole, so the pull is shared. Both posts have <b>steel pins</b>, the lid is 5 mm, and the shell bottom gains one keyhole per end. Latch and holder are v4's."),
}[key]
post_note = ("The pin is a standard Ø3 × 20 mm steel dowel, pressed up from under the arm into a 2.95 mm hole (a drop of CA glue). "
             "It stops 3.6 mm under the post tip, so it never touches the gate bar. Once the finger sits in the jaw, the jaw floor stops the pin from sliding out.")
if twin: post_note += " The anchor post 18 mm further out has a Ø3 × 12 mm pin."
steps = [
 ("Seated", 1.6, "Drone seated, pack latched", "Each gate bar's tip sits 4.2 mm deep in a pocket in a holder end leg, on a 12 mm ledge. The XT60 is mated."),
 ("Rise", 1.6, "Pincer rises under the pack",
  ("Both posts of each finger pass their keyholes" if twin else "The round posts pass the keyholes") + " in the shell bottom; the drive posts go 7.7 mm into the bars' slots. The lid pushes the pack up 0.3 mm so the ledges carry nothing."),
 ("Squeeze", 1.4, "Fingers move 3.8 mm: tips swing off the ledges",
  "Each drive post pushes its bar 12 mm from the pivot, so the tip swings 4.8 times as far. The flanges slide under the keyhole lips" + (" on both posts" if twin else "") + ": the pack now hangs on the pincer, and the pull goes post, steel pin, pad, lid."),
 ("Pull", 1.8, "Rail down 80 mm, XT60 apart", "The tips drop down the release channels while the XT60 pulls apart (~50 N)."),
 ("Flip", 2.2, "Flip 180°", "The jaw turns on short shafts in its sides; the pack is held by the flanges under the lips, not by friction."),
 ("Insert", 1.9, "Rail down 80 mm into the bay", "Bullet guide posts, the side funnel and the leg lead-ins centre the shell; the XT60 mates on the last 6 mm."),
 ("Release", 1.4, "Fingers back: springs swing the tips onto the bay ledges", "Steel springs swing both bars back; they release in opposite directions, so a side hit can't open both."),
]
steps_js = json.dumps([dict(k=a, d=b, t=c, p=d) for a, b, c, d in steps])
lists = dict(
 print=[f"Jaw base, {m['base']} g solid (less with infill)", f"Jaw lid ({lid:.0f} mm), {m['lid']} g", f"2 fingers (same part, one turned round), {m['finger_a']} g each", f"Pinion, {m['pinion']} g",
        "Shell, holder, gate bars: v4's" + (" (shell with 2 extra keyholes)" if twin else "")],
 buy=[("4 × Ø3 × 20 + " if False else "") + ("2 × Ø3 × 20 and 2 × Ø3 × 12 steel dowel pins" if twin else "2 × Ø3 × 20 steel dowel pins"),
      "4 × M3 × 10 screws + 4 × M3 heat-set inserts (lid)", "4 × M3 inserts + screws for the servo ears", "MG996R / DS3218 class servo",
      "2 × 8 mm shafts + 2 × M3 set screws (flip)"],
 asm=["Press the heat-set inserts into the base (lid corners, servo ears).",
      "Press the steel pins up into each finger from under the arm, flush with the arm's underside; a drop of CA.",
      "Servo into the base from below, pinion on the horn.",
      "Drop both fingers into their channels, racks meshing with the pinion at the open position.",
      "Lid on, 4 × M3. The fingers now can't lift out, and the pins can't fall out.",
      "Shafts into the side bores, set screws from below."])
s = open("page4v_template.html").read()
for a, b in [("__TITLE__", T["title"]), ("__EYEBROW__", T["eyebrow"]), ("__H1__", T["h1"]), ("__LEDE__", T["lede"]), ("__POSTNOTE__", post_note),
             ("__FOOT__", "Geometry: <code>nox/box/xswap/xswap4p.py</code> (pincer " + key + ") and <code>xswap4.py</code> (latch, shell, holder) on the <code>xswap-v1</code> branch of REPO1. Forces are estimates; print files come once you pick a version."),
             ("__STEPS__", steps_js), ("__LISTS__", json.dumps(lists))]:
    s = s.replace(a, b)
assert "__" not in s.replace("__agentproxy", ""), [l for l in s.split("\n") if "__" in l][:3]
d = open(os.path.join(D, "data.js")).read()
out = os.path.join(D, f"flip_swap_{key}.html")
open(out, "w").write(s.replace("<script>/*DATA*/</script>", "<script>" + d + "</script>")); print(out)
