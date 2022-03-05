// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimberPosition;
import frc.robot.subsystems.OuterLeftClimber;
import frc.robot.subsystems.OuterRightClimber;

public class LowerOuterArms extends CommandBase {

  private OuterRightClimber rightClimber;
  private OuterLeftClimber leftClimber;
  private boolean resetCommand = false;
  private boolean finishCommand = false;
  private static int tickCountRight = 0;
  private static int tickCountLeft = 0;

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
    resetCommand = false;
    finishCommand = false;

    rightClimber.setPosition(ClimberPosition.bottom);
    leftClimber.setPosition(ClimberPosition.bottom);

    tickCountLeft   = 0;
    tickCountRight = 0;
    SmartDashboard.putString("State", "Lower Outer Started");
  }

  @Override
  public void execute() {
    //   // checks to see if the arm is on the bar based on current
    //   if (rightClimber.outputCurrent() > Constants.atStartValue) {
    //     // this counter is used to avoid current spikes
    //     tickCountRight++;
    //     if (tickCountRight > 25) 
    //       // if the current is high for 5 ticks we assume we have hooked the bar and set the setpoint to the present position
    //       rightClimber.setSetpoint(rightClimber.presentEncoderValue());
    //   }
    //   else tickCountRight = 0; // this resets the tick counter if we had a current spike
    //   if (leftClimber.outputCurrent() > Constants.atStartValue) {
    //     tickCountLeft++;
    //     if (tickCountLeft > 25) 
    //       leftClimber.setSetpoint(leftClimber.presentEncoderValue());
    //   }
    //   else tickCountLeft = 0;

    // if (rightClimber.atPosition() && leftClimber.atPosition()) {
    //   double differential = rightClimber.outputCurrent() - leftClimber.outputCurrent();

      // if (((leftClimber.outputCurrent() + rightClimber.outputCurrent()) /2) < 5) {
      //     rightClimber.setPosition(ClimberPosition.top);
      //     leftClimber.setPosition(ClimberPosition.top);
      //     resetCommand = true;
      // }
      // else {
        // if (differential < Constants.Max_Differential && differential > -1 * Constants.Max_Differential) {
        //   rightClimber.setPosition(ClimberPosition.bottom);
        //   leftClimber.setPosition(ClimberPosition.bottom);
        //   finishCommand = true;
        // }
        // else {
        //   rightClimber.setPosition(ClimberPosition.top);
        //   leftClimber.setPosition(ClimberPosition.top);
        //   resetCommand = true;
        //   SmartDashboard.putBoolean("thing", true);
        // }
    //}
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Set PID setpoint to current position, then disable PIDs
    // This will keep robot stationary if the PIDs get enabled again without resetting postion
    if (interrupted) {
      rightClimber.setSetpoint(rightClimber.presentEncoderValue());
      leftClimber.setSetpoint(leftClimber.presentEncoderValue());
    }

    if (resetCommand) {
      //RobotContainer.climbContinue = false;
    }
    // rightClimber.disable();
    // leftClimber.disable();
    
    SmartDashboard.putString("State", "Lower Outer Ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return resetCommand || finishCommand;
  }
}
