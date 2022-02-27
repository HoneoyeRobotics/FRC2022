// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CamerasAndNavX;

public class SwitchCamera extends InstantCommand {
  /** Creates a new SwitchCamera. */
  private CamerasAndNavX camerasAndNavX;
  public SwitchCamera(CamerasAndNavX camerasAndNavX) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(camerasAndNavX);
    this.camerasAndNavX = camerasAndNavX;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    camerasAndNavX.switchCamera();
  }

  
}
