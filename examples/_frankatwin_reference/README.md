# FrankaTwin reference generators

The unmodified `chirp.py` and `multiband.py` are from
https://github.com/tsrobcvai/frankatwin at
`acf16009c09c99c9a2bb86050b57eb248982ba9f`, under `python/frankatwin/excitation/`.
Copyright 2026 Tao Sun, Patrick Yin and Harry He. Their Apache-2.0 LICENSE and
NOTICE are preserved. Only these two trajectory modules are included here;
this integration does not vendor their libfranka or Isaac Lab code.

Upstream SHA256 identities:

- `chirp.py`: `fe06cfd24596820efc6d5f19c411ec3a56268dc28f30502049400154d49553a9`
- `multiband.py`: `d64ce458e04a145772afa433353d0e04da951d83f7570fc0d3a40d1310a1a848`

`cart_sysid.py` calls these generators with the original duration, frequencies,
phases, rotation composition and envelope. `--scale` explicitly scales amplitudes
only. It uses this stack\'s existing Cartesian controller, not FrankaTwin\'s
controller or wire API. Simulation replay must match the actual stack controller.
