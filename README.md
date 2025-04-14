# Robotic-Tools

Collection of tools needed for a robotic related project at university

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

```roboto distoflasers <arg1> <arg2> ...```

- Calculates the relative distance between two lasers depending on the distance to the source based on the angle between them (here 0.5°)
- example:

  - ```bash
    roboto distoflasers 3.5 5 7.5
    ```

- output:

  - ```bash
    Distance of laser (m)    | Distance Between lasers (cm)
    3.5                      | 3.05432
    5                        | 4.36331
    7.5                      | 6.54496
    ```

Inverse option:

```roboto distoflasers -i <arg1> <arg2> ...```

- Calculates how far after the source the distance between the lasers is arg1...n 

- example:

  - ```bash
    roboto distoflasers -i 6 7.5
    ```

  - output:

  - ```bash
    Distance of laser (m)    | Distance Between lasers (cm)
    6                        | 6.87552
    7.5                      | 8.59439
    ```

-----

**Conversion Between Laser number, Degrees and Radians**:

```roboto radconv <value> <num|pi|deg>```

- Converts between laser number, degrees and radians
- example:

```bash

radconv -89.3 deg
Lazer Number     | Degree        | Radian
----------------------------------------
91               | -89.300       | -1.559
----------------------------------------

radconv 91 num

Lazer Number     | Degree        | Radian
----------------------------------------
91               | -89.500       | -1.562
----------------------------------------
```