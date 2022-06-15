# srbd-mpc
Single Rigid-Body Dynamics (SRBD) based linear convex MPC for real-time ground reaction force (GRF) planning

# Requirements
- [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page) 
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio) 
- [hpipm-cpp](https://github.com/mayataka/hpipm-cpp) 

# Run with Docker
- Build a docker image from Dockerfile
```docker build -f docker/Dockerfile -t srbd-mpc .```
- Run a docker container
```docker run -it --rm --platform linux/x86_64 --mount type=bind,source=${PWD},target=/home/srbd-mpc srbd-mpc bash```
- Then compile and run in docker env
