use <scan_stick.scad>
acme_major = 19.05; acme_pitch = 5.08; acme_clear = 0.40; acme_depth = 27;
// z from 0 to acme_depth (pole enters at z=0); plus the entry chamfer cone
thread_solid(acme_major, acme_pitch, acme_depth, acme_clear);
