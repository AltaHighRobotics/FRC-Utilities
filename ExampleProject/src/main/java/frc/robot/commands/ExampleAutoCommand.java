// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ExampleAutoSub;
import utilities.TargetingSystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ExampleAutoCommand extends CommandBase {
  private final ExampleAutoSub m_subsystem;
  private final TargetingSystem m_targetingSystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExampleAutoCommand(ExampleAutoSub subsystem) {
    m_subsystem = subsystem;
    m_targetingSystem = new TargetingSystem(Constants.CAMERA_POSITION, 30, Constants.GOAL_POSITION, Constants.SHOOTER_CALIBRATION, 0.1, 100, 30, 80);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_subsystem.run();
    m_targetingSystem.updateTarget(0, Math.toDegrees(m_subsystem.ExampleAuto.position.getSubtraction(Constants.GOAL_POSITION).direction2D()), 15);
    SmartDashboard.putNumber("Targeting X", m_targetingSystem.leadPosition.x);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
