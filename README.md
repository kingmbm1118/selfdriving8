# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

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

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
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

# PID-Controller
suppose you are driving on the right hand side of your lane and you want to get centered in the middle. How do you steer? Do you just steer left until you get to the middle and then stop steering? 
usually you have a PID controller embedded in your mind that allow you to steer as required.

[PID](https://en.wikipedia.org/wiki/PID_controller) is one of the most popular control algorithms. It's a hundred years old and is still used because it gets the job done.

In this project I implemented two PID controllers to drive the car around the track.
The project is implemented in C++ and the source code can be found in the `src` folder above. The PID class is implemented under `PID.h` and `PID.cpp`. The steering angle and throttle values are calculated under `h.onMessage` function in `main.cpp`.

## What Is PID?

PID is a closed loop feedback controller. It produces a control value based on the error between the current state of the system and the desired state.

## PID Implementation
The PID controller is implemented using the PID class. 
Total error is used to calculate the control value in a similar way as the equation from above.

```
double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  double total_error;
  total_error = Kp * p_error + Kd * d_error + Ki * i_error;
  //std::cout<<"The composition of steering angle is: "<<std::endl;
  //std::cout<<"Kp "<< Kp <<" * p_error "<<p_error<<" + Kd "<< Kd <<" * d_error " << d_error <<"= "<<total_error<<std::endl;
  return total_error;  // TODO: Add your total error calc here!
}
```

The decomposition of the control value is useful for debugging when choosing the right `Kp`, `Kd` and `Ki` values.

The `p_error`, `i_error` and `d_error` are calculated below. The `p_error` is the straight value taken from the feedback loop. It represents the difference between the desired system state and its current state. `d_error`, or the derivate, is simply the substraction between the previous error and the current one.  `i_error`, or the integral, is the sum of errors over time.

```
void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  i_error += cte;
  d_error = cte - p_error; //it can be initialized with CTE value because the simulator is responsive only after 2 cycles
  p_error = cte;
}
```

## Controlling The Direction
The first controller in this project is the one that, through the steering angle, makes the car drive along the road as close as possible to its center.

At the beginning, I set the steering angle to a hardcoded value of zero and launched the simulator to see if the car goes straight. I noticed that the steering angle was correctly set to zero, with no deviation, and the car seemed (visually) to be driving straight. This is how I decided that, for the direction control, the integral component is not needed. I chose a PD controller.

For the `Kp` and `Kd` values, one needs to know the dynamics of the process to be able to calculate them. The car is approximated to a bycicle and the equations are already given in the Udacity class.

```
        # Execute motion
        turn = np.tan(steering) * distance / self.length

        if abs(turn) < tolerance:
            # approximate by straight line motion
            self.x += distance * np.cos(self.orientation)
            self.y += distance * np.sin(self.orientation)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
        else:
            # approximate bicycle model for motion
            radius = distance / turn
            cx = self.x - (np.sin(self.orientation) * radius)
            cy = self.y + (np.cos(self.orientation) * radius)
            self.orientation = (self.orientation + turn) % (2.0 * np.pi)
            self.x = cx + (np.sin(self.orientation) * radius)
            self.y = cy - (np.cos(self.orientation) * radius)
```

I changed the vehicle's length to 4 meters and used the motion model in the class to tune the `Kp` and `Kd` paramaters.


`Kp = 0.5` and `Kd = 2.5` seem to provide a good vehicle trajectory in following a reference line.

I plugged these values into the implementation and started the simulator. The vehicle was very unstable, the oscilations were strong, and the car was not even able to drive on a straight line.

This behavior was very surprising since the results looked good on the graph. Then, I realized that the car was steering with an angle in between -25deg and +25deg when the control values were given between -1rad and 1rad corresponding to -57deg and +57deg.

I integrated this factor in the vehicle's dymanic model and confirmed that this is the source of the unexpected oscillations.

I kept tuning the parameters from this point by trial and error. I got a pair `Kp = 0.1` and `Kd = 1.2` that seemed like a good compromise. These are my final values.

## Controlling The Velocity

For the velocity controller the values were tuned manually since I don't have the car's velocity dynamics equations.  Choosing a hardcoded value of 0.3 for the throttle gets the vehicle running at 37 MPH in a pretty constant manner. The only thing to notice is that the vehicle takes some time to get up the speed. 

I decided to use a PI controller. Here the integral component is obviously useful. When the error is zero, the throttle value is maintained to keep the car moving at the desired velocity. I could have included the Kd component as well. It could serve in case the error decreases too fast. Then, the Kd would reduce the throttle anticipating that the car will go over the set velocity.

I set the desired speed at 30MPH and chose `Kp = 0.01` and `Ki = 0.001` for my first trial. With these values the simulated car starts from 0MPH, increases speed, gets up to 50MPH and then slows down and stabilizes at 30MPH.

Even without a graph to look at, it is obvious that the `Ki` factor is too large. The error gets intergated over a long period of time. I didn't want to increase the `Kp` factor to make the initial acceleration faster since: `0.01(Kp) * 30(initial error) = 0.3` (the approximate throttle to drive at desired speed). So increasing `Kp` would lead to instability in reaching the desired velocity.

With `Ki = 0.001` the integrated error factor is too large. When the error reaches zero, the `Ki * sum(e(t))` is larger than 0.3, the approximated desired throttle. Because of this the car accelerates and increases the velocity up to 50MPH. During this time the error is negative and keeps being intergated. `Kp` component slows the vehicle down and when the 30MPH is reached again, the `sum(e(t))` is smaller than it was before, and manages to provide the desired throttle to maintain the speed.

I kept `Kp = 0.01` and lowered the integral factor ten times to `Ki = 0.0001`.
These are my final values.