# PSO to Improve PID Controller - Scientific Initiation
# Particle Swarm Optimization (PSO) for PID Controller

This project provides an implementation of Particle Swarm Optimization (PSO) to improve a PID Controller for a given system.

## Installation

To use this project, you will need to install the following dependencies:

- [Python 3.x](https://www.python.org/downloads/)
- [numpy](https://numpy.org/)
- [matplotlib](https://matplotlib.org/)

Once you have installed these dependencies, you can download or clone this repository to your local machine.

## Usage

To use the PSO algorithm to improve the PID controller, you can run the `pso_pid_controller.py` script with the following command:

![image](https://user-images.githubusercontent.com/76737266/228051302-0c4ec333-60ad-4647-be05-32033319d754.png)

# Proportional-Integral-Derivative (PID) Controller

A Proportional-Integral-Derivative (PID) controller is a type of feedback controller commonly used in industrial control systems. It is designed to maintain a desired setpoint by continuously adjusting a control variable based on the difference between the setpoint and the measured process variable.

The PID controller consists of three terms:

- The proportional term (P) provides an output that is proportional to the error between the setpoint and the process variable. The larger the error, the larger the output.
- The integral term (I) provides an output that is proportional to the integral of the error over time. This term is used to eliminate steady-state error and improve the controller's response to changes in the setpoint or disturbances in the system.
- The derivative term (D) provides an output that is proportional to the rate of change of the error. This term is used to reduce overshoot and improve the controller's response to changes in the setpoint.

The output of the PID controller is calculated as follows:

Output = `Kp*error + Ki*integral + Kd*derivative`


where `Kp`, `Ki`, and `Kd` are the proportional, integral, and derivative gains, respectively.

The gains can be tuned to optimize the performance of the controller for a particular system. The tuning process involves adjusting the gains to achieve a desired response, such as a fast response time, low overshoot, and low steady-state error.

PID controllers are widely used in various industries, such as manufacturing, chemical processing, and robotics, due to their simplicity, effectiveness, and flexibility. They are also commonly used in hobbyist projects, such as quadcopters and robotic arms.


## Particle Swarm Optimization
Particle Swarm Optimization (PSO) is a metaheuristic optimization algorithm that was inspired by the social behavior of bird flocks and fish schools. The algorithm models a swarm of particles that fly around in a search space, with each particle representing a potential solution to the optimization problem.

The particles are attracted towards the best solutions found so far, which are called the global best and local best solutions. The algorithm updates the positions and velocities of the particles based on these attractors, with the aim of finding the optimal solution.

The PSO algorithm consists of the following steps:

1. Initialize a swarm of particles with random positions and velocities.
2. Evaluate the fitness of each particle.
3. Update the local best solution for each particle.
4. Update the global best solution for the swarm.
5. Update the velocity and position of each particle based on the global and local best solutions.
6. Repeat steps 2 to 5 until a termination criterion is met.

The PSO algorithm has several advantages, such as its simplicity and ability to handle non-linear and non-convex optimization problems. However, it can sometimes get trapped in local optima and may require a large number of particles and iterations to find the optimal solution.

PSO has been applied to a wide range of optimization problems in various fields, such as engineering, finance, and biology.

By: Vitor and João

