// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import utilities.AutonomousRobot;
import utilities.TargetingSystem;

public class ExampleAutoSub extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public final AutonomousRobot ExampleAuto = new AutonomousRobot(1, 1, 1, Constants.HEADING_PID_CONFIG, Constants.DRIVE_PID_CONFIG);
  private final TargetingSystem targetingSystem = new TargetingSystem(Constants.CAMERA_POSITION, 0, Constants.GOAL_POSITION, Constants.SHOOTER_CALIBRATION, 0.05, 100, 30, 7);
  private Rotation2d rotation = new Rotation2d();
  private int waypointIndex;
  private int heldCargo = 1;
  private double leftEncoder;
  private double rightEncoder;
  public double pitch;
  public double roll;
  public double yaw;
  private final Field2d m_field = new Field2d();
  public ExampleAutoSub() {
    ExampleAuto.setPosition(Constants.START_POSITIONS[0]);
    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void periodic() {
    rotation = Rotation2d.fromDegrees(yaw);
    m_field.setRobotPose(ExampleAuto.position.x, ExampleAuto.position.y, rotation);
  }

  public void run() {
    ExampleAuto.updateRobotPositon(leftEncoder, rightEncoder, pitch, roll, yaw);
    if (heldCargo < 2)
    {
      if (ExampleAuto.goToWaypoint(Constants.CARGO_POSITIONS[waypointIndex]) > 30)
      {
        heldCargo += Constants.CARGO_POSITIONS[waypointIndex].a;
        waypointIndex = Math.min(waypointIndex + 1,Constants.CARGO_POSITIONS.length-1);
      }
    } else if (ExampleAuto.pointAtWaypoint(Constants.GOAL_POSITION) > 150)
    {
      heldCargo = 0;
    }
    leftEncoder += ExampleAuto.getDrivePower() + ExampleAuto.getSteeringPower()/10;
    rightEncoder += ExampleAuto.getDrivePower() - ExampleAuto.getSteeringPower()/10;
    yaw = yaw + ExampleAuto.getSteeringPower()*3;
    SmartDashboard.putNumber("Target Angle", Math.toDegrees(ExampleAuto.directionToWaypoint));
    SmartDashboard.putNumber("Robot Angle", yaw);
    targetingSystem.updateTarget(yaw, -yaw- Math.toDegrees(Constants.GOAL_POSITION.getSubtraction(ExampleAuto.position).direction2D()), Math.toDegrees(Math.atan2(Constants.GOAL_POSITION.z-Constants.CAMERA_POSITION.z,ExampleAuto.position.magnitude2D())));
  }

  @Override
  public void simulationPeriodic() {
    rotation = Rotation2d.fromDegrees(-yaw);
    m_field.setRobotPose(ExampleAuto.position.x/39.37+15.980/2, ExampleAuto.position.y/39.37+8.210/2, rotation);
    //m_field.getObject("Goal").setPose((ExampleAuto.position.x+targetingSystem.position.x)/39.37+15.980/2, (ExampleAuto.position.y+targetingSystem.position.y)/39.37+8.210/2, rotation);
    m_field.getObject("Lead").setPose((ExampleAuto.position.x+targetingSystem.leadPosition.x)/39.37+15.980/2, (ExampleAuto.position.y+targetingSystem.leadPosition.y)/39.37+8.210/2, rotation);
  }
}
