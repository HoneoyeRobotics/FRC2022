// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.InnerLeftClimber;
import frc.robot.subsystems.InnerRightClimber;

public class RaiseInnerArms extends CommandBase {

  private InnerRightClimber rightClimber;
  private InnerLeftClimber leftClimber;
  private boolean bombout = false;
  private int counter = 0;

  /** Creates a new RaiseRearArm. */
  public RaiseInnerArms(InnerRightClimber rightClimber, InnerLeftClimber leftClimber) {
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

    rightClimber.setPosition(1);
    leftClimber.setPosition(1);

    counter = 0;
    bombout = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(counter++ >= Constants.CounterValue) {
      if(rightClimber.raiseCurrentBad() || leftClimber.raiseCurrentBad()) {
       bombout = true;
      }
    }
    SmartDashboard.putBoolean("BomboutInner", bombout);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(bombout) {
      rightClimber.disable();
      leftClimber.disable();
      rightClimber.runMotor(0, true);
      leftClimber.runMotor(0, true);
    }
    counter = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return bombout || (leftClimber.atSetPoint(true) && rightClimber.atSetPoint(true));
  }
}
