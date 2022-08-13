# Pendulum

# Depends

- [drake v0.39.0+](https://github.com/RobotLocomotion/drake/tree/v0.39.0)
- [lcm](https://github.com/lcm-proj/lcm)
- [elmo_driver](https://www.lejuhub.com/fuyou/elmo_driver)

# Build

```
mkdir build
cd build
cmake ..
make -j`nproc`
```

# Run

Start-up drake-visualizer
```
/opt/drake/bin/drake-visualizer
```

```
cd build
./src/pendulum/pendulum_lqr
```
