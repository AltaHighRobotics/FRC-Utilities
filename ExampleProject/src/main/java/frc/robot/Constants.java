// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import utilities.CartesianVector;
import utilities.PIDConfiguration;
import utilities.TargetingCalibration;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
    public static final PIDConfiguration HEADING_PID_CONFIG = new PIDConfiguration(1, 0.1, 0, 1, 0, 0, 0, -0.05, 0.05, 0, 0, -1, 1);
    public static final PIDConfiguration DRIVE_PID_CONFIG = new PIDConfiguration(0.2, 0.1, 0, 0, 0, 0, 0, -0.1, 0.1, 0, 0, -1, 1);
    public static final CartesianVector CAMERA_POSITION = new CartesianVector(0, 0, 50);
    public static final CartesianVector GOAL_POSITION = new CartesianVector(0, 0, 100);
    public static final CartesianVector[] START_POSITIONS = new CartesianVector[]{
        new CartesianVector(50, 0),
        new CartesianVector(30, 60)
    };
    public static final CartesianVector[] CARGO_POSITIONS = new CartesianVector[]{
        new CartesianVector(25.91, 150.79, 0, 1),
        new CartesianVector(124.95, 88.3, 0, 1),
        new CartesianVector(200, 0, 0, 0),
        new CartesianVector(150, 150, 0, 0),
        new CartesianVector(129.4, -81.84, 0, 1),
        new CartesianVector(33.77, -149.23, 0, 1)
    };
    public static final CartesianVector[] SHOOTER_DATA_POINTS = new CartesianVector[]
    {
        new CartesianVector(0, 20, 100, 80),
        new CartesianVector(50, 50, 0, 0),
        new CartesianVector(100, 70, 0, 0),
        new CartesianVector(300, 150, 0, 0)
    };
    public static final TargetingCalibration SHOOTER_CALIBRATION = new TargetingCalibration(SHOOTER_DATA_POINTS);

}
