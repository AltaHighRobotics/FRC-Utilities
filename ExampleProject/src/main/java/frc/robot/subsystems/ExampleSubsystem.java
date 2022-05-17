// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import utilities.CartesianVector;
import utilities.CopyPastAutonomous;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final CopyPastAutonomous ExampleAuto = new CopyPastAutonomous(2, 1, 1, Constants.ExamplePIDConfig, Constants.ExamplePIDConfig);
  private final CartesianVector waypoint1 = new CartesianVector(150, 75);
  private final CartesianVector waypoint2 = new CartesianVector(0, 75);
  private final CartesianVector waypoint3 = new CartesianVector(-30, 30);
  private final CartesianVector waypoint4 = new CartesianVector(-60, 75);
  private final ArrayList<CartesianVector> waypoints = new ArrayList<CartesianVector>();
  private int waypointIndex = 0;
  private double leftEncoder;
  private double rightEncoder;
  private double yaw;
  public ExampleSubsystem() {
    ExampleAuto.setWaypoint(waypoint1);
    yaw = 0;
    leftEncoder = 0;
    rightEncoder = 0;
    this.waypoints.add(waypoint1);
    this.waypoints.add(waypoint2);
    this.waypoints.add(waypoint3);
    this.waypoints.add(waypoint4);
  }

  @Override
  public void periodic() {
  }

  public void run(double n) {
    ExampleAuto.updateRobotPositon(leftEncoder, rightEncoder, yaw);
    ExampleAuto.updateTargetHeadingAndSpeed();
    if (ExampleAuto.hasReachedWaypoint())
    {
      waypointIndex ++;
      waypointIndex = Math.min(waypointIndex,3);
      ExampleAuto.setWaypoint(waypoints.get(waypointIndex));
    }
    leftEncoder += ExampleAuto.getDrivePower()/10 + ExampleAuto.getSteeringPower()/10;
    rightEncoder += ExampleAuto.getDrivePower()/10 - ExampleAuto.getSteeringPower()/10;
    yaw = yaw + ExampleAuto.getSteeringPower()/3;
    if (yaw > 180) {yaw = -360+yaw;}
    if (yaw < -180) {yaw = 360+yaw;}
    SmartDashboard.putNumber("Robot X", ExampleAuto.position.x);
    SmartDashboard.putNumber("Robot Y", ExampleAuto.position.y);
    SmartDashboard.putNumber("Target X", ExampleAuto.target.x);
    SmartDashboard.putNumber("Target Y", ExampleAuto.target.y);
    SmartDashboard.putNumber("Target Angle", Math.toDegrees(ExampleAuto.directionToWaypoint));
    SmartDashboard.putNumber("Robot Angle", yaw);
    //SmartDashboard.putNumber("Steering", ExampleAuto.getSteeringPower());
    //SmartDashboard.putNumber("Throttle", ExampleAuto.getDrivePower());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
