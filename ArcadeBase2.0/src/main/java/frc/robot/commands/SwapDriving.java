// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CamerasAndNavX;
import frc.robot.subsystems.subDriveTrainArcade;
import frc.robot.subsystems.CamerasAndNavX.CameraSource;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwapDriving extends InstantCommand {
  private subDriveTrainArcade drivetrain; 
  private CamerasAndNavX can;
  public SwapDriving(subDriveTrainArcade drivetrain, CamerasAndNavX can) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.can = can;
    addRequirements(drivetrain, can);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(drivetrain.getReverse()){
      drivetrain.setReverse(false);
      can.useCamera(CameraSource.front);
    }else{
      can.useCamera(CameraSource.rear);
      drivetrain.setReverse(true);
    }
  }
}
