## Particle Simulation on the GPU with CUDA

### Compiling the Particle Simulation Visualizer using OpenGL

Note - GCC O2 and native architecture optimizations are enabled for better performance

- Requires the cnpy library (to be built from source) to read from npy files (https://github.com/rogersce/cnpy)

```bash
    g++ visualizer.cpp -o visualizer -L/usr/local/lib/libcnpy.so -lcnpy -lz -std=c++11 -lGL -lGLU -lglfw -lGLEW -O2
```
