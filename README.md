# Hybrid AStar Trailer

[![Build Status](https://travis-ci.org/AtsushiSakai/HybridAStarTrailer.svg?branch=master)](https://travis-ci.org/AtsushiSakai/HybridAStarTrailer)

A path planning algorithm based on Hybrid A\* for trailer truck.

# Goal

I want to achieve this on autonomous vehicle (click the image to see movie).

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/jhhqkHsGrsA/0.jpg)](https://www.youtube.com/watch?v=jhhqkHsGrsA)

[Fedex truck amazing reverse parking \- YouTube](https://www.youtube.com/watch?v=jhhqkHsGrsA)

# Simulation Examples

These are simulation results for autonomous parking in a narrow space.

This planner can generate a feasible path from diffent start states.

## Perpendicular parking 

![2](https://github.com/AtsushiSakai/HybridAStarTrailer/raw/master/movie/gif/perpendicular_parking3.gif)

![2](https://github.com/AtsushiSakai/HybridAStarTrailer/raw/master/movie/gif/perpendicular_parking6.gif)

![2](https://github.com/AtsushiSakai/HybridAStarTrailer/raw/master/movie/gif/perpendicular_parking7.gif)

## Parallel parking 

![2](https://github.com/AtsushiSakai/HybridAStarTrailer/raw/master/movie/gif/parallel_parking1.gif)

![2](https://github.com/AtsushiSakai/HybridAStarTrailer/raw/master/movie/gif/parallel_parking2.gif)

![2](https://github.com/AtsushiSakai/HybridAStarTrailer/raw/master/movie/gif/parallel_parking3.gif)


# Requirements

- [Julia 0.6](https://julialang.org/downloads/)

- [PyPlot](https://github.com/JuliaPy/PyPlot.jl)

- [DataFrames](https://github.com/JuliaData/DataFrames.jl)

- [NearestNeighbors](https://github.com/KristofferC/NearestNeighbors.jl)

- [DataStructures](https://github.com/JuliaCollections/DataStructures.jl) 

# How to use

1. Install the required libraries.

2. Clone this repo.

3. Run the script like: julia perpendicular_parking.jl or julia parallel_parking.jl

4. Add star to this repo if you like it :smiley:. 

# Algorithm

This algorithm is almost same as the original Hybrid A \* algorithm.

- [Practical Search Techniques in Path Planning for Autonomous Driving](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf)

I will explain the some diffent parts of it.

## Vehicle Model

This is the vehicle model which is used in this code.

<img src="https://latex.codecogs.com/gif.latex?x&space;=&space;[x,y,\theta_0,\theta_1]\\&space;x_{t&plus;1}=x_t&plus;D\cos(\theta_t)\\&space;y_{t&plus;1}=y_t&plus;D\sin(\theta_t)\\&space;\theta_{0,t&plus;1}=\theta_{0,t}&plus;\frac{D}{L}\tan(\delta_t)\\&space;\theta_{1,t&plus;1}=\theta_{1,t}&plus;\frac{D}{d}\sin(\theta_{1,t}-\theta_{0,t})\\" title="x = [x,y,\theta_0,\theta_1]\\ x_{t+1}=x_t+D\cos(\theta_t)\\ y_{t+1}=y_t+D\sin(\theta_t)\\ \theta_{0,t+1}=\theta_{0,t}+\frac{D}{L}\tan(\delta_t)\\ \theta_{1,t+1}=\theta_{1,t}+\frac{D}{d}\sin(\theta_{1,t}-\theta_{0,t})\\" />

x and y are 2D positions. 

θ0 and θ1 is the orientation of vehicle and trailer respectively.

Ref:

- [13\.1\.2\.4 A car pulling trailers](http://planning.cs.uiuc.edu/node661.html#77556)

## Hybrid A \* for trailer

This algorithm has 3 novelies for traier parking.

1. 4D (x,y,θ0,θ1) gridding of Hybrid A \*.

2. Adding the Jackknif cost: Σ\|θ0 - θ1\|.

3. Two rectangle (truck and trailer) collision check.


# License 

MIT

# Author

Atsushi Sakai ([@Atsushi_twi](https://twitter.com/Atsushi_twi))

