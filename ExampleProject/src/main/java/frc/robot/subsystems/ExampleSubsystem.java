// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import utilities.CartesianVector;
import utilities.CopyPastAutonomous;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final CopyPastAutonomous ExampleAuto = new CopyPastAutonomous(5, 1, 1, Constants.HeadingPIDConfig, Constants.SpeedPIDConfig);
  private final CartesianVector waypoint1 = new CartesianVector(200, 0);
  private final CartesianVector waypoint2 = new CartesianVector(200, 100);
  private final CartesianVector waypoint3 = new CartesianVector(0, 0);
  private final CartesianVector waypoint4 = new CartesianVector(-200, 0);
  private final ArrayList<CartesianVector> waypoints = new ArrayList<CartesianVector>();
  private Rotation2d rotation = new Rotation2d();
  private int waypointIndex = 0;
  private double leftEncoder;
  private double rightEncoder;
  private double yaw;
  private final Field2d m_field = new Field2d();
  public ExampleSubsystem() {
    ExampleAuto.setWaypoint(waypoint1);
    yaw = 0;
    leftEncoder = 0;
    rightEncoder = 0;
    this.waypoints.add(waypoint1);
    this.waypoints.add(waypoint2);
    this.waypoints.add(waypoint3);
    this.waypoints.add(waypoint4);
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void periodic() {
    rotation = Rotation2d.fromDegrees(yaw);
    m_field.setRobotPose(ExampleAuto.position.x, ExampleAuto.position.y, rotation);
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
    leftEncoder += ExampleAuto.getDrivePower() + ExampleAuto.getSteeringPower()/10;
    rightEncoder += ExampleAuto.getDrivePower() - ExampleAuto.getSteeringPower()/10;
    yaw = yaw + ExampleAuto.getSteeringPower();
    //if (yaw > 180) {yaw = -360+yaw;}
    //if (yaw < -180) {yaw = 360+yaw;}
    SmartDashboard.putNumber("Robot Speed", ExampleAuto.motorVelocities.average);
    SmartDashboard.putNumber("Target X", ExampleAuto.target.x);
    SmartDashboard.putNumber("Target Y", ExampleAuto.target.y);
    SmartDashboard.putNumber("Target Angle", Math.toDegrees(ExampleAuto.directionToWaypoint));
    SmartDashboard.putNumber("Robot Angle", yaw);
    //SmartDashboard.putNumber("Steering", ExampleAuto.getSteeringPower());
    //SmartDashboard.putNumber("Throttle", ExampleAuto.getDrivePower());
  }

  @Override
  public void simulationPeriodic() {
    rotation = Rotation2d.fromDegrees(-yaw);
    m_field.setRobotPose(ExampleAuto.position.x/39.37+15.980/2, ExampleAuto.position.y/39.37+8.210/2, rotation);
  }
}
