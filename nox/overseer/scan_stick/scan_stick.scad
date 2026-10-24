// =====================================================================
//  Noxen scan stick: the 3/4-5 ACME paint-pole thread (23 Oct 2026)
//  The printed parts are nox/topo runs (base_topo_v8.py, head_topo_v8.py, cam_topo_v4.py); this file holds the
//  thread they cut (exported by thread_cut.scad) and the thread test ring.   part = "thread_test"
// =====================================================================
part = "thread_test";

/* [Pole] */
acme_major   = 19.05;   // 3/4" ACME major diameter
acme_pitch   = 5.08;    // 5 TPI
acme_clear   = 0.40;    // radial print clearance (0.3 tight ... 0.5 loose). Test-print `thread_test` first.
acme_depth   = 32;      // thread engagement length: the whole tip thread (~25 mm, E) + 7 so the tip seats on the boss face (Spark 24 Oct: "as much on there as possible")
boss_r       = 16;      // Ø32 boss

/* [Rendering] */
$fn = 48;
eps = 0.01;


// 3/4-5 ACME external thread solid (subtract it to get a female thread).
// Cross-section radius as a function of angle = trapezoid profile, swept with twist.
function acme_r(u, p, rmaj, rmin, crest, root) =
    let(du = abs(u - p/2))
    du <= crest/2 ? rmaj :
    du >= root/2  ? rmin :
    rmaj - (du - crest/2) / ((root - crest)/2) * (rmaj - rmin);

module thread_solid(d_maj, pitch, h, clearance=0, flank=14.5, steps=90) {
    rmaj  = d_maj/2 + clearance;
    depth = pitch/2;
    rmin  = rmaj - depth;
    crest = 0.3707*pitch + 2*clearance;
    root  = crest + 2*depth*tan(flank);
    pts = [for (i=[0:steps-1]) let(th=i*360/steps, r=acme_r(th/360*pitch, pitch, rmaj, rmin, crest, root)) [r*cos(th), r*sin(th)]];
    linear_extrude(height=h, twist=-360*h/pitch, slices=ceil(h/pitch*steps/2)) polygon(pts);  // negative twist = right-hand
}

// small ring to test the thread fit before printing the whole head
module thread_test() {
    difference() {
        cylinder(r=boss_r, h=12, $fn=96);
        translate([0,0,-eps]) thread_solid(acme_major, acme_pitch, 12 + 2*eps, acme_clear);
    }
}

if (part == "thread_test") thread_test();
