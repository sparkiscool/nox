use <scan_stick.scad>
acme_major = 19.05; acme_pitch = 5.08; acme_clear = 0.40; acme_depth = 25.5;
// z from 0 to acme_depth; unit_topo_v5 lifts it 1.5 onto the 2 mm shank counterbore (27 engaged, P Spark 24 Oct)
thread_solid(acme_major, acme_pitch, acme_depth, acme_clear);
