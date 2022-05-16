// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import utilities.ConfigurablePID;

public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final ConfigurablePID ExamplePID = new ConfigurablePID(Constants.ExamplePIDConfig);
  public ExampleSubsystem() {}

  @Override
  public void periodic() {
  }

  public void run() {
    double output = ExamplePID.runPID(10, 0);
    SmartDashboard.putNumber("Output", output);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
