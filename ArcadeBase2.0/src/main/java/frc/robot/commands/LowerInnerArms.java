// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimberPosition;
import frc.robot.subsystems.InnerLeftClimber;
import frc.robot.subsystems.InnerRightClimber;

public class LowerInnerArms extends CommandBase {

  private InnerRightClimber rightClimber;
  private InnerLeftClimber leftClimber;
  //private boolean bombout = false;
  private int tickCountRight = 0;
  private int tickCountLeft = 0;
  private boolean resetCommand = false;

  /** Creates a new RaiseRearArm. */
  public LowerInnerArms(InnerRightClimber rightClimber, InnerLeftClimber leftClimber) {
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
    resetCommand = false;

    tickCountLeft = 0;
    tickCountRight = 0;
    //if (RobotContainer.climbContinue) {
      rightClimber.setPosition(ClimberPosition.bottom);
      leftClimber.setPosition(ClimberPosition.bottom);
    //}
    SmartDashboard.putString("State", "Lower Inner Arms Started");
  }

  @Override
  public void execute() {
    // if (rightClimber.atPosition() && leftClimber.atPosition()) {
    //   double differential = rightClimber.outputCurrent() - leftClimber.outputCurrent();
    //   if (differential < Constants.Max_Differential && differential > -1 * Constants.Max_Differential) {
    //     rightClimber.setPosition(ClimberPosition.bottom);
    //     leftClimber.setPosition(ClimberPosition.bottom);
    //     resetCommand = true;
    //   }
    //   else {
    //     rightClimber.setPosition(ClimberPosition.top);
    //     leftClimber.setPosition(ClimberPosition.top);
    //     resetCommand = true;
    //   }
    // }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      rightClimber.setSetpoint(rightClimber.presentEncoderValue());
      leftClimber.setSetpoint(leftClimber.presentEncoderValue());
    }
    SmartDashboard.putString("State", "Lower Inner Arms Ended");
  }

  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}