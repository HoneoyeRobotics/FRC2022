// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberPosition;
import frc.robot.subsystems.OuterLeftClimber;
import frc.robot.subsystems.OuterRightClimber;

public class LastOuterArms extends CommandBase {
  private OuterLeftClimber leftClimber;
  private OuterRightClimber rightClimber;
  /** Creates a new LastOuterArms. */
  public LastOuterArms(OuterRightClimber rightClimber, OuterLeftClimber leftClimber) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(rightClimber);
    addRequirements(leftClimber);
    this.rightClimber = rightClimber;
    this.leftClimber = leftClimber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rightClimber.enable();
    leftClimber.enable();

    rightClimber.setPosition(ClimberPosition.last);
    leftClimber.setPosition(ClimberPosition.last);
    //}
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
