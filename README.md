# Robotic-Tools

Robotic-Tools is a collection of tools, examples and small projects developed and needed during a robotics project group at the Rheinische-Friedrich-Wilhelms-Universität Bonn (specifically this [course](https://www.ais.uni-bonn.de/SS/PG_Mobile_Robotik.html) by Dr. Nils Goerke).

Disclaimer: This is not an official repository of the university, but merely a collection of things I deemed useful during the course. The tools are not guaranteed to be correct or complete, but I will try to keep them up to date and fix bugs if they occur.

## Tools

**Time-Difference of Lasers**:

```roboto timedif <distance1> <distance2>```

- Calculates the time a laser needs to travel a certain distance and the difference in time between two lasers

- example:

  - ```bash
    roboto timedif 7.5 7.35
    ```

- output:

  - ```bash
     Time for Distance 1: 25.0248 ns
     Time for Distance 2: 24.5243 ns
     Difference in Time: 0.500496 in ns
     ```

-------

**Distance between two lasers dependent on distance to source**:

```roboto distoflaser <arg1> <arg2> ...```

- Calculates the relative distance between two lasers depending on the distance to the source based on the angle between them (here 0.5°)
- example:

  - ```bash
    roboto distoflaser 3.5 5 7.5
    ```

- output:

  - ```bash
    Distance of laser (m)    | Distance Between lasers (cm)
    3.50                     | 3.05
    5.00                     | 4.36
    7.50                     | 6.54
    ```

Inverse option:

```roboto distoflaser -i <arg1> <arg2> ...```

- Calculates how far after the source the distance between the lasers is arg1...n 

- example:

  - ```bash
    roboto distoflaser -i 6 7.5
    ```

  - output:

  - ```bash
    Distance Between lasers (cm)     | Distance from laser (m)
    6.00                             | 6.88
    7.50                             | 8.59
    ```

------

**Conversion Between Laser number, Degrees and Radians**:

```roboto laserpos <value> <num|pi|deg>```

- Converts between laser number, degrees and radians
- example:

```bash

laserpos -89.3 deg
Lazer Number     | Degree        | Radian
----------------------------------------
91               | -89.300       | -1.559
----------------------------------------

laserpos 91 num

Lazer Number     | Degree        | Radian
----------------------------------------
91               | -89.500       | -1.562
----------------------------------------
```