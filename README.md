# mc_reacher_policy
mc-rtc controller for 3D reacher task using learned (RL) policy.

## INSTALL

- Clone:
```sh
$ git clone https://github.com/rohanpsingh/mc_reacher_policy.git
```
- Download `libtorch` and put it in a directory called `ext/`:
```sh
$ cd mc_reacher_policy
$ mkdir ext && cd ext
$ wget https://download.pytorch.org/libtorch/nightly/cpu/libtorch-shared-with-deps-latest.zip
$ unzip libtorch-shared-with-deps-latest.zip
```
- Build and install
```
$ cd mc_reacher_policy
$ mkdir build && cd build
$ cmake ..
$ make -j8 install
```


## USAGE

- Open your mc-rtc config file:
```yaml
MainRobot: HRP5P
Enabled: HandReacher
Timestep: 0.02
```

- Lauch your ticker:
```sh
$ rosrun mc_rtc_ticker mc_rtc_ticker
```
- Or, if you use [mc_mujoco](https://github.com/rohanpsingh/mc_mujoco)
```sh
$ mc_mujoco
```

By default, the controller expects a trained Torch Script model at `/tmp/actor.pt` (hard-coded for now).
