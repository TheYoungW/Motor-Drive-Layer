# Private Linux C++ runtime

The Linux x86_64 wheel includes a private, stripped `libstdc++.so.6` because
DaMiao's official DM_Device v1.0 binary references `GLIBCXX_3.4.31` and
`GLIBCXX_3.4.32`, while Ubuntu 22.04 provides symbols only through
`GLIBCXX_3.4.30`.

Artifact provenance:

- conda-forge package: `libstdcxx 15.2.0 h934c35e_18`
- feedstock: <https://github.com/conda-forge/ctng-compilers-feedstock>
- upstream source: <https://gcc.gnu.org/pub/gcc/releases/gcc-15.2.0/>
- license: `GPL-3.0-only WITH GCC-exception-3.1`

The corresponding license and GCC Runtime Library Exception are included in
the Python wheel. `SHA256SUMS` records the exact stripped artifact shipped by
this repository. Native motor libraries use `$ORIGIN/dm_device`, so this copy
is isolated from the host runtime and is not installed system-wide.
