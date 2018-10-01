# CarND-Controls-MPC
## Model Predictive Control Project
## Gary Holness
Self-Driving Car Engineer Nanodegree Program

---

## Building the system

I encountered several issues in getting Ipopt to build on MacOS X.
The instructions for installing Ipopt given with the assignment
did not work.

The steps that work for me are adapted from something I was able to find on the web.  It is outdated and had to be changed somewhat.  I am sharing with the hopes that it saves everyone else the time and frustration I had.

The instructions off the web I adapted are here:  https://sites.google.com/site/benjaminskrainka/computation-info/building-ipopt

1.  Download archive file containing source code for Ipopt 3.12.11  https://www.coin-or.org/download/source/Ipopt/
2.  extract the archive
3.  set environment variable IPOPTDIR pointing to top level directory where you extracted source code
4.  go to $IPOPTDIR and you will see a subdirectory ThirdParty  containing directories for each of the
    third party tools you will need (well not each, you still need to get fortran 77)
5.  cd to $IPOPTDIR/ThirdParty/Blas
6.  run script  ./get.Blas
7. cd ../Lapack
8. run script ./get.Lapack
9. cd ../ASL
10.  run script ./get.ASL
11.  download Mumps and Metis using the scripts in ThirdParty
12.  cd ../Metis
13.  run script ./get.Metis
14.  cd ../Mumps
15. run script ./get.Mumps
16.  note that you may have to install wget.  This can be done using MacPorts
      install MacPorts and perform "sudo port install wget"    If you don't have wget,
      the scripts will try to use ftp.  Make your life easier by installing MacPorts
      and have it install  wget
17.  You will need a fortran compiler.   The g77 compiler is no longer, rather you will
       need to install gfortran.   Go to the GNU website to download gfortran
       https://gcc.gnu.org/wiki/GFortranBinaries#GNU.2BAC8-Linux
18.   make sure your system has g++ and gcc both installed
19.   go to your ipopt directory:  cd $IPOPTDIR
20.   mkdir build
21.   ../configure --enable-loadable-library --prefix=/Applications/tools/coin-or/Ipopt-3.12.11 CXX=g++ CC=gcc F77=gfortran ADD_CFLAGS="-fPIC -fexceptions" ADD_FFLAGS="fPIC -fexceptions"
22.  all the configure tests will scroll by
23.  make
24.  make test    (note you want to make sure all the ipopt tests pass)
25.  make install   (this will put your libraries and includes inside /Applications/tools/coin-or/Ipopt-3.12.11)
26a. make sure you have Homebrew
26b. brew install cppad
26c.  Now you are ready to edit your MPC to line quiz files
27.  fork and clone repository  CarND-MPC-Quizzes
28.  cd CarND-MPC-Quizzes
29.  cd mpc_to_line
30.  edit CMakeLists.txt
31.  add lines for include path for ipopt headers and for link path to ipopt libraries
          include_directories(/Applications/tools/coin-or/Ipopt-3.12.11/include)
          link_directories(/Applications/tools/coin-or/Ipopt-3.12.11/lib)


* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
