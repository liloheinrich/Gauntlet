# Gauntlet

| Robot Path Planning Challenge code for Quantitative Engineering Analysis 1, Spring 2020 | <img src="https://github.com/liloheinrich/Gauntlet/blob/master/media/potential_gradient_field.png" width="150"> |
| ------------- | ------------ |

- Use Random Sampling Concensus (RANSAC) to fit lidar data and find circular target
- Use gradient descent to plot a path to the target while avoiding obstacles
- Calculate angular and forward velocity and execute on robot

Also see **[Bridge of Doom Challenge](https://github.com/liloheinrich/BridgeOfDoom)** for another project about parametric path following.

## Introduction

The gauntlet is a simulated environment with obstacles (boxes), walls, and the Barrel of Benevolence (BoB). The challenge is to autonomously guide a Neato robot from the origin of the map through the obstacles and to gently tap the BoB. There are three levels of missions:

- Level 1: the locations and dimensions of the obstacles and BoB are all given
- Level 2: the BoB location and radius is given but obstacles must be found using LIDAR
- Level 3: the radius of the BoB is given, obstacles and BoB must be found using LIDAR

Where levels 2 & 3 may be completed either through dynamic updating or just an initial scan from the LIDAR. I completed level 3.


## Approach

My general approach was to use only an initial scan. I figured out that running the circle detection and recalculating the potential field was going to be too computationally expensive causing the program to seriously slow down, as well as too complex for me to implement right now.

I also initially tried taking the polynomial best fit of the path points, turning it into a parametric equation, and running it through my Bridge of Doom parametric path follower code, but it wasn’t working very well and so I decided to go for a simpler approach to speed up the process.

The five main steps to my method of completing the challenge were:

1. Scan: collect initial LIDAR scan data ![img](https://lh4.googleusercontent.com/kgWpuCK5-OxbP6FTk6McCl8CWg7kdFzv3qN56wPLcaC9rGXkQcPTOr9BXThKCMDjOKRKc_9E9ZU-NLS9Q_1d2bJVO7Ys2xPoEUM8ipgOHcqRFVU-uOjyCt309okRSwAo90zcjdud)

2. Fit: run circle detection for center coordinate, then find obstacles as line segments![img](https://lh5.googleusercontent.com/mcHq-l3gfBZDij3bowaU7nu_tqbMdMUuZnOVv8C0HwFPtwxJhFFhYdEYYe6E9G6FJro2BbigDrngAOMrHLWAHshRRDMR08lmgAfaEc7BSTwO0LRRUAF1kW-wTwCN-Zfsan8Izz8Y)

3. Path: 

   1.  Create potential field with sources across line segments and sinks along BoB circle![img](https://lh3.googleusercontent.com/RXppRp6rfCVRdcE0V6H1y5ZCujMS_O-BUpkgchaMCY6hFY9j8Y5Kh-w72Svxyb_WNk1vaFUsHCvlVjFMqUeCt8OYnRDJWn130gQEca8xP1YzxEncqo6w7-Bk6dCqWVRkDbScpzXl)

   2. Run gradient descent to calculate path to local maximum![img](https://lh6.googleusercontent.com/0k8IL5rt1sM_tj2yXEgd0OzpWi4t-d99RotHfe83BmYEFwZAPf0QLgyaeAebhJZHA13k51dxB9_bytEYHjIiJzJduxwzuuX35C6Yuj3ooB-jbuzvBgIUSpyDoJQmdDMQPSEVG-1N)

4. Drive: use the change in heading to feed into the angular velocity in order to roughly follow the calculated points path (also: record encoder data as Neato drives)

   1. Calibrate coefficients for the angular velocity, forward velocity, and time increment

   2. Results: the neato drove the path in **4.405 seconds** according to my program
      1. Video link: https://youtu.be/aL6x6IM77Cw
      2. Or find it in the GitHub repository linked above

5. Analyze: reconstruct path from encoder data, compare it to calculated path ![img](https://lh4.googleusercontent.com/-uPqaXCor1a5aKWEWehVme9HrTA30CDOuIFXrs_FFWJXVndvUrCA1Pn5pnook35kIAzPA8xA-YC6wWfO7AwUGQ9whkWAImO1g3ijNsOGD60cKsqIifz2Qm48EOFufc858I4_XZtw)
