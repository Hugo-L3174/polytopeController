# polytopeController

Example/test controller for the [Dynamic Polytopes](https://github.com/Hugo-L3174/mc_dynamic_polytopes) library

## Dependencies

- [mc_rtc](https://github.com/jrl-umi3218/mc_rtc)
- [mc_dynamic_polytopes](https://github.com/Hugo-L3174/mc_dynamic_polytopes)
- [stabiliplus](https://gite.lirmm.fr/mc-controllers/stabiliplus) (branch topic/Optimize) for comparisons with previous criteria


## Nix support

This project provides a `flake.nix` for reproducible builds and development environments using [Nix](https://nixos.org/). To enter a development shell with all dependencies available, run:

Make sure you have [Nix](https://nixos.org/download.html) installed with flakes enabled.

### Building/developing

```sh
nix develop
```

This allows to build the project itself

```sh
mkdir build
cd build
cmake -GNinja ..
ninja
```

To build the project using Nix flakes:

```sh
nix build
```

### Running/Testing

To use/test the project, use the `polytopeController-superbuild` development shell with

```sh
nix develop .#polytopeController-superbuild
```

This provides a shell with:
- All runtime dependencies needed by mc-rtc and the project installed
- A default `mc_rtc.yaml` configuration for the project
- `mc_mujoco`, `mc-rtc-magnum`, `mc-rtc-rviz` and other relevant gui tools

Within this shell, run

```sh
# run gui in the background (you can also run it in another shell)
mc-rtc-magnum &
# or mc-rtc-rviz &
# run ticker with the default configuration of the controller generated from etc/mc_rtc.in.yaml
mc_rtc_ticker
```

or for mujoco

```sh
# POLYTOPE_CONTROLLER_MUJOCO is an environment variable pointing to a configuration using the _MuJoCo variant of the robot module
mc_mujoco -f $POLYTOPE_CONTROLLER_MUJOCO
```
