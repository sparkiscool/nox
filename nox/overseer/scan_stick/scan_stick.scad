// =====================================================================
//  Noxen handheld scan stick  -  parametric 3D-printed mounts
//  Unitree L2 + DECXIN 2633V2 camera + Raspberry Pi 5 + 4S 6000 LiPo on the 3/4-5 ACME paint pole
//  Units: mm.   OpenSCAD 2021.01+
//
//  Parts (set `part`, or use the Makefile / render.sh):
//    lidar_head      - rod clamp + lidar plate + camera boss          (top of rod)
//    camera_bracket  - small plate for Pi Camera Module 3 (or custom) (bolts to head)
//    pi_bat_cradle   - rod clamp + Pi tray + battery cradle + buck shelf (lower on rod)
//    assembly        - everything positioned on a ghost rod (preview only)
//
//  >>> FIRST THING TO CHANGE: rod_d  (outer diameter of your pole) <<<
// =====================================================================

part = "assembly";   // [lidar_head, camera_bracket, pi_bat_cradle, thread_test, assembly]

/* [Pole] */
// Pole = EXTEND-A-REACH 1.5-3 ft telescoping extension pole (Amazon B08LL28XZW):
// universal 3/4"-5 ACME twist-on tip.  The head screws straight onto that tip.
mount_style  = "acme";  // "acme" = 3/4-5 ACME female thread (paint pole tip); "quarter20" = 1/4"-20 nut trap; "clamp" = split clamp on a plain tube
acme_major   = 19.05;   // 3/4" ACME major diameter
acme_pitch   = 5.08;    // 5 TPI
acme_clear   = 0.40;    // radial print clearance (0.3 tight ... 0.5 loose). Test-print `thread_test` first.
acme_depth   = 27;      // thread engagement length (tip threads are ~25 mm long)
rod_d        = 25.4;    // OUTER diameter of the pole's LOWER tube where the Pi/battery cradle clamps. MEASURE IT.
rod_clear    = 0.5;     // diametral clearance in the printed clamp bore

/* [Lidar orientation] */
lidar_facing = "up";    // "down" = lidar hangs under an arm, dome down (belly-mounted drone layout). "up" = lidar sits on top of the pole, dome up (top-mounted drone layout)
arm_len      = 115;     // down mode: horizontal offset pole axis -> lidar axis (total length 174 fits the A1 mini bed)
arm_w        = 50;      // down mode: arm width
cam_y        = 38;      // down mode: position of the camera-bracket inserts along the arm (camera looks straight down)

/* [Lidar - Unitree 4D LiDAR L2 (nox/dimensions.md section M, spec)] */
// L2: 75 x 75 x 65, 230 g, base flange Ø75. 4x M3, 6 deep, on a Ø51 circle, rotated 22.5° from the
// cable-outlet axis (cable leaves the base rim on -X). FOV = hemisphere above the base: nothing above the plate.
lidar_body     = 75;    // base flange diameter / footprint (ghost + clearance)
lidar_h        = 65;    // body height (ghost only)
lidar_bolt_circle = 51; // 4x M3 on this circle
lidar_bolt_rot = 22.5;  // first hole at this angle from +X (cable axis is X, outlet on -X)
lidar_screw_d  = 3.4;   // clearance for M3
lidar_plate_w  = 86;    // printed plate size (square)
lidar_plate_t  = 6;     // printed plate thickness
lidar_cable_w  = 14;    // notch for the L2's bottom cable outlet (on the -X side of the plate)
lidar_center_hole = 26; // air under the L2 base (bottom cover must stay <= 85 C, section M) and -16 g

/* [Camera - DECXIN 2633V2 USB global shutter (section M, P)] */
cam_hole_x   = 28;      // M2 hole spacing X
cam_hole_y   = 28;      // M2 hole spacing Y
cam_screw_d  = 1.8;     // M2 self-tap into PETG
cam_board    = 38;      // board size (clearance)
cam_plate_w  = 44;
cam_plate_h  = 46;
cam_plate_t  = 3;
cam_standoff = 3;       // board floats on 4 standoffs (components on the back, lens barrel open Q3)

/* [Raspberry Pi - Pi 5 defaults] */
pi_l        = 85;       // board length
pi_w        = 56;       // board width
pi_hole_x   = 58;       // hole spacing along length
pi_hole_y   = 49;       // hole spacing along width
pi_hole_off = 3.5;      // hole inset from board corner
pi_screw_d  = 2.3;      // M2.5 self-tap into print (use 2.7 for through + nut)
pi_standoff = 7;        // standoff height (room for solder tails / underside)

/* [Battery - set to your LiPo] */
bat_l = 148;            // along the rod   (4S 6000 XT60, P, section O)
bat_w = 48;             // across
bat_h = 45;             // thickness (sticking out from the plate)
strap_w = 20;           // hook-and-loop strap width
strap_t = 3.5;          // strap slot thickness

/* [Clamp blocks] */
wall      = 5;          // clamp wall around the rod
hub_h     = 60;         // height of the lower clamp hub
head_sock = 45;         // height of the upper clamp socket under the lidar plate
ear_t     = 7;          // clamp ear thickness
ear_gap   = 2.5;        // slit width
clamp_bolt = 4.3;       // M4 clearance
clamp_nut_af = 7.2;     // M4 nut across flats
clamp_nut_t  = 3.4;

/* [Panel cutouts on Pi tray] */
rocker_w = 19.4;        // KCD1 rocker switch cutout
rocker_h = 13.2;
button_d = 12.3;        // 12 mm momentary pushbutton
led_d    = 5.2;         // 5 mm LED

/* [Rendering] */
$fn = 48;
eps = 0.01;

bore_d = rod_d + rod_clear;
hub_r  = bore_d/2 + wall;

// ---------------------------------------------------------------------
//  helpers
// ---------------------------------------------------------------------
module rounded_box(size, r) {
    // box with vertical rounded edges, size=[x,y,z], centred in x,y, z from 0
    hull() for (sx=[-1,1], sy=[-1,1])
        translate([sx*(size[0]/2-r), sy*(size[1]/2-r), 0]) cylinder(r=r, h=size[2]);
}

module hex_pocket(af, t) { cylinder(d=af/cos(30), h=t, $fn=6); }

// split-clamp around a rod: cylinder of height h with a slit + two ears on the +Y side,
// clamp bolts along X at the given z positions.
module split_clamp(h, bolt_z=[]) {
    difference() {
        union() {
            cylinder(r=hub_r, h=h, $fn=96);
            // ears
            translate([-(ear_t + ear_gap/2), hub_r - 3, 0])
                cube([2*ear_t + ear_gap, 12, h]);
        }
        // bore
        translate([0,0,-eps]) cylinder(d=bore_d, h=h+2*eps, $fn=96);
        // slit
        translate([-ear_gap/2, 0, -eps]) cube([ear_gap, hub_r + 20, h+2*eps]);
        // bolts
        for (z = bolt_z) translate([0, hub_r + 3.5, z]) {
            rotate([0,90,0]) cylinder(d=clamp_bolt, h=60, center=true);
            // nut trap on -X ear
            translate([-(ear_gap/2 + ear_t) - eps, 0, 0]) rotate([0,90,0]) hex_pocket(clamp_nut_af, clamp_nut_t + eps);
            // bolt head counterbore on +X ear
            translate([ear_gap/2 + ear_t - 3, 0, 0]) rotate([0,90,0]) cylinder(d=7.5, h=10);
        }
    }
}

// ---------------------------------------------------------------------
//  LIDAR HEAD  (origin = centre of lidar plate underside; plate from z=0 up)
// ---------------------------------------------------------------------
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

boss_r = (mount_style == "clamp") ? hub_r : 16;
sock_h = (mount_style == "clamp") ? head_sock : (mount_style == "acme" ? acme_depth + 7 : 22);

// pole interface boss, z from 0 to sock_h, pole enters from z=0 (bottom). Ears (clamp) at -Y.
module pole_boss(extra_top=0) {
    difference() {
        union() {
            if (mount_style == "clamp") rotate([0,0,180]) split_clamp(sock_h, [10, sock_h - 10]);
            else cylinder(r=boss_r, h=sock_h, $fn=96);
            cylinder(r=boss_r, h=sock_h + extra_top, $fn=96);
        }
        if (mount_style == "acme") {
            translate([0,0,-eps]) thread_solid(acme_major, acme_pitch, acme_depth + eps, acme_clear);
            // no entry chamfer: it leaves a 1 mm sliver on the first thread turn (solid_check thin wall); the pole tip is chamfered
        }
        if (mount_style == "quarter20") {
            translate([0,0,-eps]) cylinder(d=6.8, h=sock_h - 2);
            translate([0,0,5]) hex_pocket(11.4, 6);             // 1/4"-20 nut (7/16" AF)
            translate([-11.4/2, -30, 5]) cube([11.4, 30, 6]);   // side slot for the nut
        }
    }
}

// lidar plate with the L2 hole pattern. `pin_down`: counterbores on top for a hanging lidar.
module lidar_plate(pin_down=false) {
    difference() {
        rounded_box([lidar_plate_w, lidar_plate_w, lidar_plate_t], 6);
        for (a=[0:90:270]) rotate([0,0,lidar_bolt_rot + a]) translate([lidar_bolt_circle/2, 0, 0]) {
            translate([0,0,-1]) cylinder(d=lidar_screw_d, h=lidar_plate_t + 2);
            if (pin_down) translate([0,0,lidar_plate_t - 3]) cylinder(d=6.2, h=4);   // heads on top
            else translate([0,0,-eps]) cylinder(d=6.2, h=3);                        // heads below
        }
        translate([0,0,-1]) cylinder(d=lidar_center_hole, h=lidar_plate_t + 2);
        // cable notch for the lidar's bottom cable outlet (-X edge in to the base rim)
        translate([-lidar_plate_w/2 - 1, -lidar_cable_w/2, -1]) cube([lidar_plate_w/2 + 1 - lidar_body/2 + 6, lidar_cable_w, lidar_plate_t + 2]);
    }
}

module lidar_head() {
    if (lidar_facing == "up") lidar_head_up(); else lidar_head_down();
}

// ---- UP: lidar on top of the pole. Plate z=0..t, boss below, camera boss on +Y face of the boss.
module lidar_head_up() {
    difference() {
        union() {
            lidar_plate(pin_down=false);
            translate([0,0,-sock_h]) {
                pole_boss();
                for (a=[45,135,225,315]) rotate([0,0,a])
                    hull() {
                        translate([boss_r-2, -2, sock_h - 1]) cube([lidar_plate_w/2 - boss_r - 3, 4, 1]);
                        translate([boss_r-2, -2, sock_h - 18]) cube([1, 4, 1]);
                    }
                translate([-16, boss_r - 4, sock_h - 32]) cube([32, 8, 30]);   // camera boss, front
            }
        }
        for (sx=[-1,1]) translate([sx*10, boss_r + 6, -sock_h + head_cam_z()]) rotate([90,0,0]) cylinder(d=4.0, h=8);
    }
}
function head_cam_z() = sock_h - 17;

// ---- DOWN: pole boss z=0..sock_h, arm on top running +Y, lidar plate at y=arm_len with the lidar hanging below.
module lidar_head_down() {
    arm_z  = sock_h;                 // arm underside
    rib_h  = 14;
    difference() {
        union() {
            pole_boss(extra_top = lidar_plate_t + rib_h);
            // arm
            translate([-arm_w/2, -boss_r, arm_z]) cube([arm_w, arm_len + boss_r, lidar_plate_t]);
            // plate at the end
            translate([0, arm_len, arm_z]) lidar_plate(pin_down=true);
            // side ribs + end rib
            for (sx=[-1,1]) translate([sx*(arm_w/2 - 2) - 2, 0, arm_z]) cube([4, arm_len - lidar_plate_w/2 + 6, lidar_plate_t + rib_h]);
            hull() {
                translate([-arm_w/2, arm_len - lidar_plate_w/2 + 2, arm_z]) cube([arm_w, 4, lidar_plate_t + rib_h]);
                translate([-lidar_plate_w/2 + 6, arm_len - lidar_plate_w/2 + 2, arm_z]) cube([lidar_plate_w - 12, 4, lidar_plate_t + 1]);
            }
        }
        // camera bracket inserts under the arm (2x M3 heat-set, 20 mm apart, camera looks down)
        for (sx=[-1,1]) translate([sx*10, cam_y, arm_z - 1]) cylinder(d=4.0, h=7);
        // zip-tie slots for the lidar cable along the arm
        for (y=[30, 70]) for (sx=[-1,1]) translate([sx*14 - 1.5, y, arm_z - 1]) cube([3, 5, lidar_plate_t + 2]);
        // re-cut the pole thread / bore through everything
        if (mount_style == "acme") translate([0,0,-eps]) thread_solid(acme_major, acme_pitch, acme_depth + eps, acme_clear);
    }
}

// small ring to test the thread fit before printing the whole head
module thread_test() {
    difference() {
        cylinder(r=boss_r, h=12, $fn=96);
        translate([0,0,-eps]) thread_solid(acme_major, acme_pitch, 12 + 2*eps, acme_clear);
    }
}

// ---------------------------------------------------------------------
//  CAMERA BRACKET  (bolts to the head's front boss; camera faces +Y)
//  Printed flat. Two M3 holes 20 mm apart at the top; camera pattern below.
// ---------------------------------------------------------------------
module camera_bracket() {
    // flat plate, thickness along Y. Top: 2x M3 holes (20 mm apart) for the head boss.
    // Below: 4 standoffs on the camera's 28 x 28 M2 pattern (board faces +Y) + a USB cable slot.
    cam_cz = -(cam_plate_h/2) - 2;   // camera centre height relative to the boss holes
    difference() {
        union() {
            translate([0, 0, -cam_plate_h]) rounded_box([cam_plate_w, cam_plate_t, cam_plate_h + 14], 1.4);
            for (sx=[-1,1], sy=[-1,1])
                translate([sx*cam_hole_x/2, cam_plate_t/2 - eps, cam_cz + sy*cam_hole_y/2]) rotate([-90,0,0]) cylinder(d=6, h=cam_standoff + eps);
        }
        for (sx=[-1,1]) translate([sx*10, 0, 7]) rotate([90,0,0]) cylinder(d=3.4, h=20, center=true);
        for (sx=[-1,1], sy=[-1,1])
            translate([sx*cam_hole_x/2, 0, cam_cz + sy*cam_hole_y/2]) rotate([90,0,0]) cylinder(d=cam_screw_d, h=20, center=true);
        // USB cable slot below the board
        translate([0, 0, cam_cz - cam_hole_y/2 - 6]) cube([16, 20, 4], center=true);
    }
}

// ---------------------------------------------------------------------
//  PI + BATTERY CRADLE (rod along Z; Pi tray on +X, battery on -X,
//  clamp ears on +Y, buck-converter shelf on -Y)
// ---------------------------------------------------------------------
module pi_bat_cradle() {
    plate_t   = 3.5;
    pi_pl_w   = pi_w + 8;                 // tray width (Y)
    pi_pl_h   = pi_l + 8 + 24;            // tray height (Z): board + panel strip for switch/button/LED
    bat_pl_w  = bat_w + 2*(strap_w/2 + 8);
    bat_pl_h  = bat_l + 8;
    shelf_w   = 52; shelf_h = hub_h;      // UBEC (5 V 5 A) + 12 V buck shelf, zip-tied

    difference() {
        union() {
            split_clamp(hub_h, [12, hub_h - 12]);

            // --- Pi tray (+X) ---
            translate([hub_r - plate_t, -pi_pl_w/2, 0]) cube([plate_t, pi_pl_w, pi_pl_h]);
            // web between hub and tray
            translate([0, -hub_r*0.72, 0]) cube([hub_r, hub_r*1.44, hub_h]);
            // standoffs for the Pi
            for (ix=[0,1], iy=[0,1])
                translate([hub_r - eps, -pi_w/2 + pi_hole_off + iy*pi_hole_y, 4 + pi_hole_off + ix*pi_hole_x])
                    rotate([0,90,0]) cylinder(d=7, h=pi_standoff + eps);
            // stiffening ribs along the tray edges
            for (sy=[-1,1]) translate([hub_r - plate_t, sy*(pi_pl_w/2) - (sy>0?3:0), 0]) cube([plate_t + 3, 3, pi_pl_h]);

            // --- Battery plate (-X) ---
            translate([-hub_r, -bat_pl_w/2, 0]) cube([plate_t, bat_pl_w, bat_pl_h]);
            translate([-hub_r, -hub_r*0.72, 0]) cube([hub_r, hub_r*1.44, hub_h]);
            // bottom ledge the battery sits on
            translate([-hub_r - bat_h - 3, -bat_w/2 - 2, 0]) cube([bat_h + 3 + plate_t, bat_w + 4, 4]);
            // ledge lip
            translate([-hub_r - bat_h - 3, -bat_w/2 - 2, 0]) cube([3, bat_w + 4, 12]);
            // side rails to keep the battery from sliding sideways
            for (sy=[-1,1]) translate([-hub_r - 10, sy*(bat_w/2 + 2) - (sy<0?3:0), 0]) cube([10 + plate_t, 3, bat_pl_h]);

            // --- Buck converter shelf (-Y) ---
            translate([-shelf_w/2, -hub_r - plate_t + 1, 0]) cube([shelf_w, plate_t, shelf_h]);
        }

        // Pi standoff screw holes
        for (ix=[0,1], iy=[0,1])
            translate([hub_r + pi_standoff - 6, -pi_w/2 + pi_hole_off + iy*pi_hole_y, 4 + pi_hole_off + ix*pi_hole_x])
                rotate([0,90,0]) cylinder(d=pi_screw_d, h=10);
        // vent / weight cutouts behind the Pi
        for (iz=[0,1,2]) translate([hub_r - plate_t - 1, 0, 4 + 14 + iz*25]) rotate([0,90,0]) rounded_box([12, pi_w - 18, plate_t + 2], 3);

        // panel strip above the Pi: rocker switch, button, LED
        pz = 4 + pi_l + 4 + 12;
        translate([hub_r - plate_t - 1, -14, pz]) rotate([0,90,0]) translate([-rocker_h/2, -rocker_w/2, 0]) cube([rocker_h, rocker_w, plate_t + 2]);
        translate([hub_r - plate_t - 1, 12, pz]) rotate([0,90,0]) cylinder(d=button_d, h=plate_t + 2);
        translate([hub_r - plate_t - 1, 24, pz]) rotate([0,90,0]) cylinder(d=led_d, h=plate_t + 2);

        // strap slots in the battery plate (two straps)
        for (zf=[0.25, 0.75], sy=[-1,1])
            translate([-hub_r - 1, sy*(bat_w/2 + 4) - strap_t/2, 4 + zf*bat_l - strap_w/2])
                cube([plate_t + 2, strap_t, strap_w]);
        // lightening slots in battery plate
        for (iz=[0,1,2]) translate([-hub_r - 1, 0, 4 + 20 + iz*(bat_l/3)]) rotate([0,90,0]) rounded_box([12, bat_w - 14, plate_t + 2], 3);

        // zip-tie slots on the buck shelf
        for (sx=[-1,1], iz=[0,1,2]) translate([sx*18 - 1.5, -hub_r - plate_t, 8 + iz*20]) cube([3, plate_t + 2, 8]);
    }
}

// ---------------------------------------------------------------------
//  ASSEMBLY PREVIEW
// ---------------------------------------------------------------------
module ghost_rod(len=400) { %color("gray", 0.35) translate([0,0,-len]) cylinder(d=rod_d, h=len, $fn=64); %color("silver",0.5) cylinder(d=acme_major, h=acme_depth-2, $fn=48); }
module ghost_lidar_body() { rounded_box([lidar_body, lidar_body, lidar_h*0.6], 6); translate([0,0,lidar_h*0.6]) scale([1,1,(lidar_h*0.4)/(lidar_body/2-2)]) sphere(d=lidar_body - 4); }
module ghost_pi() { %color("green", 0.5) translate([hub_r + pi_standoff, -pi_w/2, 4]) cube([1.6, pi_w, pi_l]); }
module ghost_bat() { %color("blue", 0.4) translate([-hub_r - bat_h, -bat_w/2, 4]) cube([bat_h, bat_w, bat_l]); }

module assembly() {
    ghost_rod();
    lidar_head();
    if (lidar_facing == "up") {
        %color("black", 0.4) translate([0,0,lidar_plate_t]) ghost_lidar_body();
        translate([0, boss_r + 4 + 1, -sock_h + head_cam_z() - 7]) camera_bracket();
    } else {
        %color("black", 0.4) translate([0, arm_len, sock_h]) mirror([0,0,1]) ghost_lidar_body();
        // camera bracket flat under the arm, tab at cam_y, camera further out, lens down
        translate([0, cam_y + 7, sock_h - 1.5]) mirror([0,1,0]) rotate([-90,0,0]) camera_bracket();
    }
    translate([0,0,-230]) { pi_bat_cradle(); ghost_pi(); ghost_bat(); }
}

// ---------------------------------------------------------------------
if (part == "lidar_head")          rotate([180,0,0]) lidar_head();      // print orientation: lidar plate (largest flat face) on the bed, boss up
else if (part == "thread_test")    thread_test();
else if (part == "camera_bracket") rotate([90,0,0]) camera_bracket();   // lies flat for printing
else if (part == "pi_bat_cradle")  pi_bat_cradle();
else                               assembly();
