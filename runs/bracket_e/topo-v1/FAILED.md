# bracket_e topo-v1: FAILED before the first solve

Branch run/topo-bracket_e from origin/topo/new-engine @ 90b5afe, 23 Oct 2026. Box: 4 cores, 15 GB RAM. Command: `cd nox && python3 -m topo.run bracket_e --git`. Wall time about 30 s, no iterations, no checkpoint.

## Cause (setup geometry, not the engine solver)

The support region in `nox/box/stand/stand_v7_topo.py` bracket_e_setup is `box(X_IN - 1.0, X_IN + 0.5, ...)`: it reaches only 1.0 mm into the part (env and the sheet cut both end at X_IN). On the fine grid (h 1.0) it catches 4540 cells. Stage 1 of the engine rebuilds the setup on the 2h grid (h 2.0), where no cell centre falls in that 1 mm band, so `Part.support` raises "the region holds no part cells".

Best fix (one line, setup owner): make the support region at least one coarse cell deep, `box(X_IN - 2.0 * h - 0.5, X_IN + 0.5, ...)` or a fixed `X_IN - 4.5`. Alternative without a setup edit: rerun with `--no-coarse` (fine grid only, slower). Not done here: this worker may change files under runs/ only.

## Traceback

```
Traceback (most recent call last):
  File "<frozen runpy>", line 203, in _run_module_as_main
  File "<frozen runpy>", line 88, in _run_code
  File "/home/user/nox/nox/topo/run.py", line 128, in <module>
    sys.exit(main())
             ~~~~^^
  File "/home/user/nox/nox/topo/run.py", line 120, in main
    res = optimise(spec["setup"], h, gen, os.path.abspath(out), a.part, settings=s, coarse=not a.no_coarse,
                   git=a.git, maxit=a.maxit, extra=dict(baseline=spec.get("baseline")))
  File "/home/user/nox/nox/topo/run.py", line 73, in optimise
    part_c = setup(2.0 * h)
  File "/home/user/nox/nox/topo/demos/stand.py", line 35, in setup
    pp, drawn, cuts, env, gen = T.SETUP[name](h)
                                ~~~~~~~~~~~~~^^^
  File "/home/user/nox/nox/box/stand/stand_v7_topo.py", line 59, in bracket_e_setup
    pp.support("default", box(X_IN - 1.0, X_IN + 0.5, 120.0, 234.0, zc - 20, zc + 20))
    ~~~~~~~~~~^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/home/user/nox/nox/topo/part.py", line 115, in support
    raise ValueError(f"support {name}: the region holds no part cells")
ValueError: support default: the region holds no part cells
```

## Last 40 log lines

```
<frozen runpy>:130: RuntimeWarning: 'topo.run' found in sys.modules after import of package 'topo', but prior to execution of 'topo.run'; this may result in unpredictable behaviour
=== bracket_e: h 1.0, generated 60.0 cm3, min member 3.2 mm, cores 4
bracket_e: grid (88, 144, 80) h 1.0 = 1,013,760 cells; design 661,852 (661.9 cm3), drawn 24,276, void 327,632
  support default: 4540 cells, 9414 nodes
layout: min member 3.20 -> 5.00 mm so members survive the solid gate's 2.0 mm erosion round the drawn bodies
bracket_e: grid (48, 72, 40) h 2.0 = 138,240 cells; design 83,955 (671.6 cm3), drawn 3,018, void 51,267
Traceback (most recent call last):
  File "<frozen runpy>", line 203, in _run_module_as_main
  File "<frozen runpy>", line 88, in _run_code
  File "/home/user/nox/nox/topo/run.py", line 128, in <module>
    sys.exit(main())
             ~~~~^^
  File "/home/user/nox/nox/topo/run.py", line 120, in main
    res = optimise(spec["setup"], h, gen, os.path.abspath(out), a.part, settings=s, coarse=not a.no_coarse,
                   git=a.git, maxit=a.maxit, extra=dict(baseline=spec.get("baseline")))
  File "/home/user/nox/nox/topo/run.py", line 73, in optimise
    part_c = setup(2.0 * h)
  File "/home/user/nox/nox/topo/demos/stand.py", line 35, in setup
    pp, drawn, cuts, env, gen = T.SETUP[name](h)
                                ~~~~~~~~~~~~~^^^
  File "/home/user/nox/nox/box/stand/stand_v7_topo.py", line 59, in bracket_e_setup
    pp.support("default", box(X_IN - 1.0, X_IN + 0.5, 120.0, 234.0, zc - 20, zc + 20))
    ~~~~~~~~~~^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  File "/home/user/nox/nox/topo/part.py", line 115, in support
    raise ValueError(f"support {name}: the region holds no part cells")
ValueError: support default: the region holds no part cells
```
