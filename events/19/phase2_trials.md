# Phase 2 Trials

Descriptions of the scenarios used for Phase 2 evaluation and scoring.

# Task 1: Station Keeping

|Run     |Waves   | Wind          | Ambient | Notes                                  | For all           |
|--------|--------|---------------|---------|----------------------------------------|-------------------|
|    0   | Easy   | Constant      | -       | Goal pose facing into wind             | varying goal pose |
|    1   | Medium | Variable      | -       | Goal pose approx. away from wind       | random seed, wind |
|    2   | Hard   | Faster Change | -       | Goal pose approx. perpendicual to wind | direction         |
|    3   | Easy   | Constant      | -       | Goal pose facing into wind             | varying goal pose |
|    4   | Medium | Variable      | -       | Goal pose approx. away from wind       | random seed, wind |
|    5   | Hard   | Faster Change | -       | Goal pose approx. perpendicual to wind | direction         |

# Task 2: Wayfinding

|Run     |Waves   | Wind          | Ambient | Notes                                               | Time |
|--------|--------|---------------|---------|-----------------------------------------------------|------|
|    0   | Easy   | Constant      | -       | 3 waypoints in a short line, little angle variation | TBD  |
|    1   | Medium | Variable      | -       | 4 waypoints in a spiral, greater distance           | TBD  |
|    2   | Hard   | Faster Change | -       | 5 waypoints: rough parallelogram with center point  | TBD  |
|    3   | Easy   | Constant      | -       | 2 waypoints in a short line, right in front of wamv ic's | TBD  |
|    4   | Medium | Variable      | -       | 4 waypoints in a rough box, clockwise          | TBD  |
|    5   | Hard   | Faster Change | -       | 4 waypoints: same as Run 4, but running CCW | TBD  |

# Task 3

|Run     | Notes                                                                       |
|--------|-----------------------------------------------------------------------------|
|    1   | 3 trials, up to one object per trial                                        | 
|    2   | 4 trials, up to two objects per trial                                       |
|    3   | 4 trials, up to six objects per trial, repeated objects, partialy occluded  |
|    4   | 3 trials, up to one object per trial                                        | 
|    5   | 4 trials, up to two objects per trial                                       |
|    6   | 4 trials, up to six objects per trial, repeated objects, partialy occluded  |

# Task 4

|Run     | Notes |
|--------|-------|
|    1   | Very similar to the RobotX navigation channel, no obstacles, no wind, mild waves, no fog | 
|    2   | 6 gates, low obstacle density, low wind, medium waves, no fog                            |
|    3   | 5 gates, medium obstacle density, medium-high wind, high waves, fog                      |
|    4   | Very similar to the RobotX navigation channel, no obstacles, no wind, mild waves, no fog | 
|    5   | 6 gates, low obstacle density, low wind, medium waves, no fog                            |
|    6   | 5 gates, medium obstacle density, medium-high wind, high waves, fog                      |

# Task 5: Dock

|Run     | Correct Bay | Incorrect Bay | Notes |
|--------|-------|-------|-------|
|    0   | red_cross | blue_circle |  Benign conditions.  Dock setup right in front of WAMV.    Correct and incorrect are different shape and different color. |
|    1   |  green_triangle | blue_cross | Medium waves, little wind.  A bit of dock separation, still single dock.  |
|    2   |  red_triangle | red_cross | High waves, more wind, fog  - perpendicular to starting yaw |
|    3   |  green_circle | blue_cross | Similar to 0.  Benign conditions.  Dock setup right in front of WAMV, but correct placard is on the other side.  Correct = geen_circle, incorrect = blue_cross.|
|    4   |  blue_circle | blue_triangle | Similar to 1. Constant wind perpendicular to dock. Dock is perpendicular to WAMV, so you have to move to view placards.  |
|    5   |  blue_cross | red_circle |  Similar to 2. Dock is perpendicular to WAMV, so you have to move to view placards.  Co|


# Task 6: Scan and Dock

Dock locations, dock configuration (which bay is correct and placards) and weather the same as Task 5.  This may change if we can get the docks to automatically generate consistent placards.

Currently only difference is addition of light buoy in each world.

|Run     | Correct Bay | Incorrect Bay | Color Seq | Notes |
|--------|-------|-------|-------|-------|
|    0   | red_cross | blue_circle | R G B | Same as Task 5, Run 0 |
|    1   | green_triangle | blue_cross | G B G     | Same as Task 5, Run 1|
|    2   | red_triangle | red_cross | R B G | Same as Task 5, Run 2 |
|    3   | blue_cross | red_circle | B R B | Similar to Task 5, Run 3 |
|    4   | green_circle | green_cross | G B R |  Similar to Task 5, Run 4 |
|    5   | blue_triangle | geen_triangle | B R G | Similar to  Task 5, Run 5 |