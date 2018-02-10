# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---
## Main Objective:
The main goal of this project is to drive a car using simulator without going off the track and using PID controller method steering value should be calculate.

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

I have used the clion editor and it was very helpful. I copied the editor profile to ide_profiles folder.

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/
* /ide_profiles/

The README should explain what the profile does, how to take advantage of it,
and how to install it.

## Project Implementation:
 #### The effect of the P, I, D component of the PID algorithm 

PID (Proportional Integral Derivative) controller is widely used control mechanism in different applications. 
First i started with P value and I D controller values as zeros. Then i figureout the P value to get reasonable frequency. Then i changed the D controller value as 1 then increased with different trails. In this case car went around track but in some cases like in bridge and sharp turnings i see some oscillations. 
Then finally i added I controller with little value 0.001 then car was significantly center of the track. After the many trails with the paramter values i figuredout proper values for P , I and D.
The controller implemented very easyily but the paramter tuning takes more time to finish the track successfully with out any deviations. 

 #### Final hyperparameters tuning
Initially i started the hyperparamters tuning manually for P,I and D. I also tried by implementing the Twiddle approach which was explained in the class room. With the twiddle approach car was not on track and most of the places it is off the track. Then again i tried manual experiments tuning by changing different values for P, I and D. I figure out the best value at the end after many trials.

P (Proportion) controller value is very small 0.3 , if it is large then vehicle going off the track.
I (integral) controller value is too small 0.001 compare two other two controllers.
D (Derivative) is quite large value 4.0.

Then i implemented a PID controller for the throttle to increases the car speed on the track. I used throttle value as 0.3 and it maintains the speed about 30-35MPH speed. Cross track error (CTE) inversly mapped to throttle value. If i increase the throttle value then getting the high error and car is not stable on the track. so i used less throttle value to decrease the error value. 

To implement this project i gone through the youtube help video for the reference [Youtube Videos](https://www.youtube.com/watch?v=YamBuzDjrs8&index=4&list=PLAwxTw4SYaPnfR7TzRZN-uxlxGbqxhtm2)

## Output Result:

Here's a [link to my video result](https://youtu.be/47Lx9GF--J4) 
Due to the limitation in github, i uploaded my vide to the youtube and provided the link above.

