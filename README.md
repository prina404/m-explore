# Changes implemented in this fork

## Frontier size fix

Given the formula to calculate a frontier's centroid:

$$
\frac{1}{n}\sum_{x \in F} p_x
$$

where $F$ is the set of points $p_i \in \mathbb{R}^2$ that forms a frontier, and $n = |F|$.

In the original code the frontier's size was actually $n = |F| + 1$, which led to a misplacement of the centroid (= goal published).

Such misplacement is inversely proportional to the frontier's size, resulting in unreachable goals. Now fixed.


## Frontier distance calculation

The distance of a frontier from the robot is now calculated using A*, which approximates the real amount of space the robot needs to travel to reach it.

In case of a distance-greedy exploration config (for example by setting the gain component of the frontier to zero) the choice of a frontier was based exclusively on euclidean distance. In some cases (especially in environments with big rooms) this led to oscillating behaviours of the robot.

By using and actual path approximation the occurrence of such behaviours is greatly reduced (sadly not completely avoided). It works best with a low planner frequency (< 0.5).

---
A couple of details of this A* implementation:

- It tracks unknown space, it may flag a certain frontier as closest due to a path in unkown space, but the robot may not be able to actually traverse it.
- It ignores costmaps inflation layers, it simply looks for a plausible path.
- It has a state-space cutoff to limit the amount of resources needed, in big spaces if a frontier is outside of the search state space, an arbitrarily big distance is assigned to it.
- Once all frontiers have been computed, they are sorted according to their euclidean distance relative to the robot. The A* distance estimation is performed only on the 20 closest frontiers, to avoid computing thousands of searches for each planner loop. To the farther frontiers and arbitrarily big distance is assigned.


---
# m-explore

[![Build Status](http://build.ros.org/job/Kdev__m_explore__ubuntu_xenial_amd64/badge/icon)](http://build.ros.org/job/Kdev__m_explore__ubuntu_xenial_amd64)

ROS packages for multi robot exploration.

Installing
----------

Packages are released for ROS Kinetic and ROS Lunar.

```
	sudo apt install ros-${ROS_DISTRO}-multirobot-map-merge ros-${ROS_DISTRO}-explore-lite
```

Building
--------

Build as standard catkin packages. There are no special dependencies needed
(use rosdep to resolve dependencies in ROS). You should use brach specific for
your release i.e. `kinetic-devel` for kinetic. Master branch is for latest ROS.

WIKI
----

Packages are documented at ROS wiki.
* [explore_lite](http://wiki.ros.org/explore_lite)
* [multirobot_map_merge](http://wiki.ros.org/multirobot_map_merge)

COPYRIGHT
---------

Packages are licensed under BSD license. See respective files for details.
