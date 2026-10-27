"""Overseer printed arm v6, FRONT only (27 Oct): v5d with the chords made a space frame. v5d (228 g, strength 0.62) read
131 Hz vertical / 162 lateral against the 190 target: the 4 x 6/4 chords worked almost alone. v6 adds 12 x 3.0 carbon
diagonals (a Warren girder of 3 panels in each face of the chord box, stations t 0.02 / 0.34 / 0.66 / 0.98 along the
chords), each 7 off its face so it passes beside every tube, blind at its motor-side end and slid in from outside through
its root-side end, with drawn sleeves and node blocks; and the shoulder closed full depth (a solid wedge from the root
block to a post joining each side's chord sockets): the modal what-if (arm_diag_whatif_v6.py on the v5d mesh) put the soft
spot in the x 5..25 band, where the vertical shear crossed only the 6 mm top / bottom flanges. Same arguments as
arm_topo_v1.py; runs go to runs/overseer_arm_front/v6. Setup check: --rodcheck-setup.
"""
import os, runpy, sys
HERE = os.path.dirname(os.path.abspath(__file__))
V6 = (("NOX_ARM_VERSION", "v6"), ("NOX_ARM_DIAG", "1"), ("NOX_ARM_DIAG_T", "0.02,0.34,0.66,0.98"),
      ("NOX_ARM_DIAG_OFF", "7"), ("NOX_ARM_DIAG_GAP", "6"), ("NOX_ARM_DIAG_GAP_OPEN", "0"), ("NOX_ARM_DIAG_PAT", "0:1111"),
      ("NOX_ARM_DIAG_KIND", "rod3"), ("NOX_ARM_SHWEB", "solid"))
for k, v in V6:
    os.environ.setdefault(k, v)
if __name__ == "__main__":
    if "rear" in sys.argv[1:2]:
        sys.exit("arm_topo_v6.py is the front arm only (rear stays v5d)")
    runpy.run_path(os.path.join(HERE, "arm_topo_v5d.py"), run_name="__main__")
