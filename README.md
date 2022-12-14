# Computer-Graphics-Course
This repository is based on a Computer graphics course assignments https://sites.cs.ucsb.edu/~lingqi/teaching/games101.html

It contains C++ implmentations of rasterizer, raytracing, etc. Results are shown below.

# Setup

Required libraries

- CMake
- Eigen
- OpenCV
- stb_image
- OBJ Loader

# Results

## MP1 Viewing Transformation

- Model/View transformation, 
- Perspective/Orthogonal projection, 
- Viewport transformation, 
- Bresenham's line drawing algorithm

<img src="https://github.com/zkcr0000/Computer-Graphics-Course/blob/main/Supplementary/HW1.gif" width="250" height="250"/>

## MP2 Rasterizer

Implemented a rasterizer with the following features.

- Depth buffer
- Anti-aliasing(supersampling anti-aliasing(SSAA), multisample anti-aliasing(MSAA))

Left is the result without Anti-aliasing. Middle is the result with MSAA. Right is the result with SSAA.
After zooming in, we can see anti-aliasing has reduced the effect of zigzag at the boundary.

<img alt="Without Anti-alising" src="https://github.com/zkcr0000/Computer-Graphics-Course/blob/main/Supplementary/Vanilla.png" width="250" height="250"/><img alt="MSAA" src="https://github.com/zkcr0000/Computer-Graphics-Course/blob/main/Supplementary/MSAA.png" width="250" height="250"/><img alt="SSAA" src="https://github.com/zkcr0000/Computer-Graphics-Course/blob/main/Supplementary/SSAA.png" width="250" height="250"/>

<img alt="Without Anti-alising" src="https://github.com/zkcr0000/Computer-Graphics-Course/blob/main/Supplementary/Vanilla_zoomin.png" width="250" height="250"/><img alt="MSAA" src="https://github.com/zkcr0000/Computer-Graphics-Course/blob/main/Supplementary/MSAA_zoomin.png" width="250" height="250"/><img alt="SSAA" src="https://github.com/zkcr0000/Computer-Graphics-Course/blob/main/Supplementary/SSAA_zoomin.png" width="250" height="250"/>

## MP3 Shading model

Implemented the following shading methods.

- Blinn–Phong reflection model
- Bump mapping
- Displacement mapping
- Texture

From left to right: Blinn-Phong reflection model, Bump mapping, Displacement mapping, Texture

<img src="https://github.com/zkcr0000/Computer-Graphics-Course/blob/main/Supplementary/phong.png" width="250" height="250"/><img src="https://github.com/zkcr0000/Computer-Graphics-Course/blob/main/Supplementary/bump.png" width="250" height="250"/><img src="https://github.com/zkcr0000/Computer-Graphics-Course/blob/main/Supplementary/displacement.png" width="250" height="250"/><img src="https://github.com/zkcr0000/Computer-Graphics-Course/blob/main/Supplementary/texture.png" width="250" height="250"/>


## MP4 Bézier curve

Implemented de Casteljau's algorithm.

<img src="https://github.com/zkcr0000/Computer-Graphics-Course/blob/main/Supplementary/HW4.gif" width="250" height="250"/>

## MP5 Whitted-Style Ray tracing

Implemented Whitted-Style Ray tracing with reflection and diffraction.

<img src="https://github.com/zkcr0000/Computer-Graphics-Course/blob/main/Supplementary/HW5.jpg" width="240" height="180"/>

## MP6 Ray tracing with Bounded Volume Hierarchy(BVH)

- Ray tracing for Stanford Bunny
- Implemented BVH to accelerate ray tracing
- Implemented ray intersecting with Axis Aligned Bounding Box(AABB)

<img src="https://github.com/zkcr0000/Computer-Graphics-Course/blob/main/Supplementary/HW6.jpg" width="240" height="180"/>

## MP7 Path tracing

Implemented path tracing.

- Path tracing for Cornell Box
- Rendering equation
- bidirectional reflectance distribution function (BRDF)
- Monte Carlo Integration

<img src="https://github.com/zkcr0000/Computer-Graphics-Course/blob/main/Supplementary/HW7.jpg" width="250" height="250"/>

# Compilation

Inside each MP folder

```
mkdir build
cmake ..
make
./Rasterization # Or ./Raytracing
```

