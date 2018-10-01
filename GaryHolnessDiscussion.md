# CarND-Controls-MPC
## Model Predictive Control Project
## Gary Holness
Self-Driving Car Engineer Nanodegree Program

---

## Building the system

I encountered several issues in getting Ipopt to build on MacOS X.
The instructions for installing Ipopt given with the assignment
did not work.

The steps that work for me are adapted from something I was able to find on the web.  It is outdated and had to be changed somewhat.

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

The instructions for CppAD worked fine (using brew)

## Approach

I began with the MPC Quiz and adapted it to suit the MPC Project

MPC involves a number of state variables including the
current position in two dimensions along the surface of a map
in world coordinate frame, the steering angle, the velocity,
and the acceleration.  

The goal of MPC is, given a series of waypoints, to select
the control inputs that minimize a cost function.  The control
inputs are the steering angle and the trottle.   The MPC model
maintains a model of how the car moves from one time instant
to the next.  This model is iterated forward in time to
predict where the car would be as a result of a sequence of
control actions over future time steps.   A polynomial fit
is made to a series of waypoints.  A set of points along
the polynomial fit are sampled in regular increments.
The points along the polynomial fit are then used
in an optimization for the control sequence that 
optimizes an objective that inclues factors such as:

* cross track error
* orientation error
* velocity error with respect to a reference velocity
* steering actuation (delta)
* throttle (acceleration)
* derivative for steering signal (derivative by finite difference)
  to select for smooth steering actuation
* derivative for throttle signal (derivative by finite difference)
  to select for smooth throttle actuation


The state variables are

x- x position on map in world coordinates
y- y position on map in world coordinates
psi- orientation on map in world coordinates
v- velocity (longitudinal at angle psi in direction car is facing)
cte- cross track error
epsi- orientation error


The model equations are as follows...

x_t+1   = x_t + v_t * cos(psi_t) * dt
y_t+1   = y_t + v_t * sin(psi_t) * dt
psi_t+1 = psi_t + v_t/Lf * delta_t * dt
v_t+1   = v_t + a_t * dt
cte_t+1 = f(x_t) = y_t + v_t * sin(e_psi_t) * dt
e_psi_t+1= psi_t - psi_des_t + v_t/Lf * delta-t * dt

The MPC controller inputs the state (from waypoint) the
model (update equations), constraints on variables, and
cost function (FG_eval) to find the vetctor of control
inputs that minimizes the cost function.   The Ipopt
library is used as the solver. 

In computing my cost function, I multiplied some terms
by a scalar to encourage the solver to select control
action solutions that emphasized certain aspects of
the cost function more than others.   Specifically
increase emphasis was placed on CTE (cross track error)
and the actuation signals. The largest emphasis was
placed on the derivative of the acutation signal.  This
encourages the optimizer to select smooth actuations.


