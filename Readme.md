# Calibration Documentation

# Step 1: Get Charuco Markers from Image

`get_charuco()`:

- Input:
    - The image frame, the camera matrix, the distortion coefficient of the camera
- Process:
    - Find the aruco markers using `aruco.DetectMarkers()` function
    - Refine the detection and interpolate the corners. These are the standard procedures of obtaining the `charucoCorners` and the `charucoIds`.
    - Finally, use the built in `caliberateCameraCharuco()` to find rotation and translation vector of the camera that transforms from the charuco board coordinate to the camera view coordinate (rel. to the camera).


# Step 2: Get Circles from Image

`get_circle()`:

- Inputs:
    - The image frame, the charuco parameters, the charucoConers
- Process:
    - Convert the image frame into black and white and find the circles grid using the built-in function `findCirclesGrid()`. Then visualize the circle corners.
    - Find the intersection between the circle and the board. This is done by the algorithm `intersectCirclesRaysToBoard()`.
- Outputs:
    - `circles`: The 2D coordinates of the detected circles returned from the `findCirclesGrid()`
    - `circles2D`: The 2D coordinate of the circles in the frame of the board.
    - `circles3D`: The 3D coordinate of the circles in the frame of the camera.

`intersectCirclesRaysToBoard()`:

- Notations:
    - plane_normal: The normal vector of the charuco board.
    - plane_point: The vector from the camera to the charuco board.
    - p: The homogeneous coordinate of a circle with `z=1`.
    - ray_point: The same as p. The coordinate of the circle from camera to the circle on the board.
        - ray_direction: The normalized coordinate (i.e. the unit vector) of the circle from camera to the circle on the board.
    - ndotu: The length of ray_direction along the plane_normal direction.
    - w: The vector from the center of the board to the ray_point.
    - si: The length of w along the plane_normal direction.
    - v: The vector from the center of the board to the circle on the board.
    - Psi: The vector from the camera to the circle on the board.
    - vx: The x-axis value of the coordinate of the circle in the coordinate frame of the board (where the origin is the center of the board and the z-axis is the plane_normal).
    - vy: The y-axis value of the coordinate of the circle in the coordinate frame of the board.
    - circles_2d: The coordinate of the circle in the coordinate frame of the board where `z=0`.
    - circles_3d: The coordinate of the circle in the coordinate frame of the camera, where the origin is the camera.
- Process:
    - Find `ndotu` by taking `plane_normal.dot(ray_direction.T)`.
    - Find the ratio between `si` and `ndotu`, and then find `v = w + si * ray_direction`.
    - Find `Psi = v  + plane_point`.


# Step 3: Calibrate the Projector

Calling `get_circle_coord` gives us the `projCirclePoints` in the screen coordinate. We append this coordinate to `projCirclePointsAccum`.

Calling `get_circle` gives us:

- `circle_cam`: The 2D coordinates of the detected circles returned from the `findCirclesGrid()`
- `ret_circle`: The 3D coordinate of the circles in the frame of the board where `z=0`.
- `ret_circle3D`: The 3D coordinate of the circles in the frame of the camera.

Then we append these coordinates into three lists:

- `circleCam` ← `circle_cam`
- `circleBoard` ← `ret_circle`
- `circleWorld` ← `ret_circle3D`

Then we calibrate the camera by calling
`calibrateCamera`:

- Inputs:
    - `circleBoard`: The 3D coordinate of the circles in the frame of the board where `z=0`.
    - `projCirclePointsAccum`: The coordinate of the circles in the frame of the screen.
- Outputs:
    - The calibrated projector matrix.

Finally we find the relative transformation between the camera coordinate and the projector coordinate by calling
`stereoCalibrate()`:

- Input:
    - `circleWorld`: The coordinate of the circles in the world coordinate (origin is the camera)
    - `circleCam`: The 2D image coordinate of the circles in the view of the camera.
    - `projCirclePointsAccum`: The 2D coordinate of the circles on the computer screen (i.e. in the view of the projector).
- Output:
    - The rotation and translation vector of the camera.

