## OMNI-WHEEL CAR

![omni-wheel car](images/omni-car.png)

### Top view of Omni Wheel car showing wheel/motors and directional conventions

This configuration of omni-wheels is easily controlled to move in any direction: X, Y, θz:
- forward-back motion (along the Y axis)
- right-left sideways motion (along the X axis)
- spin motion (θz).

Moreover, all three of these motions can be superimposed. The motors can be commanded to combine these motions simultaneously. (More about that later.) For now, let's just consider how we can get the car to move in one DOF at a time.

Imagine that we want to drive the car along a 45 degree direction into the first quadrant (along the common axis of wheel motors M3 and M4). Only wheel motors **M1 and M2** would run. **M1** would need to be driven **clockwise** and **M2** would need to be driven **CCW** at the same speed. Wheel motors M3 and M4 would not run. To drive drive the car at speed S along the 45 degree direction, we would send the following drive values to the motors:

```
m1 = +S
m2 = -S
m3 = 0
m4 = 0
```

Now imagine that we want to drive the car along a 135 degree direction into the second quadrant (along the common axis of wheel motors M1 and M2). Only wheel motors **M3 and M4** would run. **M4** would need to be driven **clockwise** and **M3** would need to be driven **CCW** at the same speed. Wheel motors M1 and M2 would not run. To drive drive the car at speed S along the 135 degree direction, we would send the following drive values to the motors:

```
m1 = 0
m2 = 0
m3 = -S
m4 = +S
```

Notice that the car's motion along the 45 degree direction is completely **independent** from its motion along the **orthogonal** 135 degree direction. First, it suggests that the "natural coordinate system" for the omni-wheels runs through the wheel axes (which happens to be oblique to the car's coordinate system). But it also invites us to realize that we may combine the control of these two **independent degrees of freedom** in any combination we desire. Thus, it seems reasonable to consider driving the car in any arbitrary angular direction. The mathematics of this is simplified by using **polar coordinates (r, θ)** to specify the speed and direction of the car. To convert between the omni-wheels 'natural' coordinates and the car's coordinates, we simply add or subtract 45 degrees (pi/4) from the theta value. Below is the algorithm to drive the car in a direction theta at speed.

```
speed, theta = desired_speed_and_direction
theta += pi/4
u, v = convert_polar_to_rect(speed, theta)
m1 = u
m2 = -u
m3 = -v
m4 = v
```
Notice the rectangular coordinates in the omni-wheel's natural coordinate system have been labeled u, v. This is intended to remind us that they are not the same as x, y coordinates in the car's coordinate system.
If we want to simultaneously spin the car **CCW** about its own Z axis, all four wheel motors would receive an additional **CW** component as shown below.
```
spin = some_value
speed, theta  = desired_speed_and_direction
theta += pi/4
u, v = convert_polar_to_rect(speed, theta)
m1 = u + spin
m2 = -u + spin
m3 = -v + spin
m4 = v + spin
```