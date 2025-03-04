# Advantage Scope Custom Assets
Advantage Scope allows you to use custom robot assets to display whats happening to your robot in a more meaningful manner. It takes a lot of work to setup, but full instructions can be found in the Advantage Scope Custom Assets [Documentation Page](https://docs.advantagescope.org/more-features/custom-assets#overview).

For the 2025 Season we are using the [2025 MARS/WARS Mainbot](/layouts/adscope-assets/Robot_2025_Mainbot/config.json) asset. This asset includes all of setup for articulated components. If you just a static representation you can use the [2025 MARS/WARS Static]() asset.

## Loading Assets
#### Project Link
- Launch Advantage Scope and click `Help > Use Custom Assets Folder`
- Select the `adscope-assets` directory in the Robot Project.
- Select the new the new robot from Advantage Scope!
#### Manual
- Launch Advantage Scope and click `Help > Show Assets Folder`
- Copy Robot Asset Package into the `userAssets` directory. Make sure the Folder name keeps the `Robot_***` prefix
- Select the new robot from Advantage Scope!
    - Note: This will require you to redo this process every time something is updated in the asset package.

## Adjusting Assets
Each asset package has a `config.json` that links all of the models together. It allows you to adjust the "zero" rotation and position for each component. Mechanical Advantage has a [Youtube video](https://www.youtube.com/watch?v=unX1PsPi0VA&t=173s&ab_channel=LittletonRobotics) that shows the steps to attach components to their associated pose publishers.
```
// Sequence of rotations along the x, y, and z axes
zeroedRotations": [
    {
        "axis": "x",
        "degrees": 0
    },
    {
        "axis": "y",
        "degrees": 0
    },
    {
        "axis": "z",
        "degrees": 180
    }
],
// Position offset in meters relative to the robot, applied after rotation
"zeroedPosition": [
    5,
    5,
    5
]
```
