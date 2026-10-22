# Cage v3 (C3) node runs as used 17 Oct (one heavy run per box, each in its own background task; 2 h task cap)
# corner (fresh, filleted wheel pockets, fixed void):  R_REIN=2.0 GEN_CM3=1.4 PRUNE_T=900 PRUNE_T2=600 python3 node_gen.py corner 0.75 2700
#   then rebuild + growing reinforcement:          REBUILD=1 R_REIN=2.0 GEN_CM3=1.4 python3 node_gen.py corner 0.75 2700
# side node (one ring held, 0.8 cm3; generated TPU): R_REIN=2.0 GEN_CM3=0.8 PRUNE_T=900 PRUNE_T2=600 python3 node_gen.py snode 0.6 2400
# crown (pads tied, multigrid, 2.0 cm3):           GEN_CM3=2.0 NCRASH=8 NMIS=4 BIG_CELLS=60000 R_REIN=2.0 PRUNE_T=600 PRUNE_T2=400 python3 node_gen.py crown 1.0 3000
# vtee (generated TPU 17 Oct 10:57):               python3 node_gen.py vtee 0.5 900
# PETG recheck of a TPU-generated mesh:            MAT=PETG RECHECK=<stl> python3 node_gen.py snode 0.6
# print set: decimate to one watertight body each (mesh_seal), then python3 export_c3.py <node dir> <out dir>
