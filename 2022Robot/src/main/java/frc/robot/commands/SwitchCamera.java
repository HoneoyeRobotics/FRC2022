// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;

public class SwitchCamera extends InstantCommand {
  /** Creates a new SwitchCamera. */
  private DriveTrain driveTrain;
  public SwitchCamera(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.switchCamera();
  }

  
}
