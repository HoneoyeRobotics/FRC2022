// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.InnerClimber;
import frc.robot.subsystems.OuterClimber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TogglePIDs extends InstantCommand {
  private InnerClimber innerClimber;
   private OuterClimber outerClimber;
  public TogglePIDs(InnerClimber innerClimber, OuterClimber outerClimber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.innerClimber = innerClimber;
    this.outerClimber = outerClimber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(innerClimber.isEnabled()){
      innerClimber.disable();
      outerClimber.disable();
    }
    else  {
      
      innerClimber.setSetpoint(0);
      innerClimber.enable();
      outerClimber.setSetpoint(0);
      outerClimber.enable();
    }

  }
}
