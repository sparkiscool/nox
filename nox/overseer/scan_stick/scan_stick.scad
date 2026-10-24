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
acme_depth   = 26.0;    // thread cut z 1.5..27.5 in the base: 0.5 into the 2 mm shank counterbore, 0.5 past the tip (25 thread, P Spark 24 Oct); 27 engaged
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

// v2 fit coupon (Spark 24 Oct: the stick fit test): the base's socket exactly, in print pose (thread end on the bed).
// In use coords (pole enters at z 0): Ø28.3 x 2 shank counterbore, thread cut z 1.5..27.5 with the 1 mm entry chamfer,
// open top so the tip shows 0.5 under the top face when the shank shoulder seats. Wall 3.1 round the thread, 3.35 round the
// counterbore; the 45 deg step between them prints without supports.
shank_r = 14.15; shank_d = 2.0; top_z = 27.5;
module fit_coupon_use() {
    difference() {
        union() {
            cylinder(r=17.5, h=4, $fn=96);
            translate([0,0,4]) cylinder(r1=17.5, r2=13, h=4.5, $fn=96);
            cylinder(r=13, h=top_z, $fn=96);
        }
        translate([0,0,-1]) cylinder(r=shank_r, h=shank_d + 1, $fn=96);
        translate([0,0,shank_d - 0.5]) thread_solid(acme_major, acme_pitch, top_z - shank_d + 0.5 + eps, acme_clear);
        translate([0,0,shank_d - 0.5]) cylinder(r1=10.9, r2=9.9, h=1.5, $fn=64);
    }
}
module thread_test() { translate([0,0,top_z]) rotate([180,0,0]) fit_coupon_use(); }   // print pose: the thread end on the bed (rotated, never mirrored: keeps the right-hand thread)

if (part == "thread_test") thread_test();
