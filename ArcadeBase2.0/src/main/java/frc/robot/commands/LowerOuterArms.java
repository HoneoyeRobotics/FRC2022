// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.OuterLeftClimber;
import frc.robot.subsystems.OuterRightClimber;

public class LowerOuterArms extends CommandBase {

  private OuterRightClimber rightClimber;
  private OuterLeftClimber leftClimber;
  private boolean bombout = false;
  private int counter = 0;
  private int lCounter = 0;
  private int rCounter =0;

  /** Creates a new RaiseRearArm. */
  public LowerOuterArms(OuterRightClimber rightClimber, OuterLeftClimber leftClimber) {
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

    rightClimber.setPosition(0);
    leftClimber.setPosition(0);

    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tempRightCurrent = 100;
    double tempLeftCurrent = 100;

    if (rightClimber.outputCurrent() > leftClimber.outputCurrent() + 1) {
      tempRightCurrent = rightClimber.outputCurrent();
      rCounter++;
      if (rCounter > 10) {
        rightClimber.setSetpoint(rightClimber.presentEncoderValue());
      } 
    }
    else if (tempRightCurrent <= leftClimber.outputCurrent()) {
      rightClimber.setPosition(0);
    }
    else rCounter = 0;


    if (leftClimber.outputCurrent() > rightClimber.outputCurrent() + 1) {
      tempLeftCurrent = leftClimber.outputCurrent();
      lCounter++;
      if (lCounter > 10) {
        leftClimber.setSetpoint(leftClimber.presentEncoderValue());
      } 
    }
    else if (tempLeftCurrent <= rightClimber.outputCurrent()) {
      leftClimber.setPosition(0);
    }
    else lCounter = 0;
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return leftClimber.atSetPoint(false) && rightClimber.atSetPoint(false);
  }
}
