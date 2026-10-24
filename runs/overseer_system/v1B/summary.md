# Overseer system v1B (alternative B, 24 Oct)

Layout B of the balance conflict: the pack / catch centre 30 behind the thrust axis (`SYS_TIP_OFF=30 python3 nox/overseer/overseer_system.py runs/overseer_system/v1B`, cage `runs/overseer_cage/v2`). Motors ~163 ahead of the plate centre, ~2.87 kg (E), cage first -20.4 mm FAIL (`checks.txt`). Not chosen: it breaks the balance rule (CG ≤ 3 mm off the thrust axis with the pack in and out) and still fails cage first. Layout A (pack on the axis) is the DEFAULT: `runs/overseer_system/v2A`.
