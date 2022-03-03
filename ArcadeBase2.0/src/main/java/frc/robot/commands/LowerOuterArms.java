// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.OuterLeftClimber;
import frc.robot.subsystems.OuterRightClimber;

public class LowerOuterArms extends CommandBase {

  private OuterRightClimber rightClimber;
  private OuterLeftClimber leftClimber;
  //private boolean bombout = false;
  private int tickCountRight=11;
  private int tickCountLeft=11;

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
  }

  // Called every time the scheduler runs while the command is scheduled.
    // NOTE: be sure to account for negative arm positions
// This function takes two arm positions and compares them to make sure the "compare" arm is within the MAX
// differential position of the "other" arm.  This funtion will normalize the positions to account for negative values.
// It returns TRUE if the value of the compare arm is out of bounds and FALSE if compare arm is in expected range.
  @Override
  public void execute() {
    // initialize tick values so first crack at lowering arms doesn't cause current comparison
      tickCountRight=11;
      tickCountLeft=11;
    
    // RIGHT ARM lower logic
    // Check is right arm is too much lower than left arm.
    if ( rightClimber.Arm_Postion_Too_Low(leftClimber.presentEncoderValue(), Constants.Max_Differential) ) {
      // pause PID for RIGHT arm at the current position
      rightClimber.setSetpoint(rightClimber.presentEncoderValue());
      tickCountRight = 0;
    }
    else {
      // ONCE tickCount expires, evaluate the motor currents
      // if they are within a sane range, then resume PID for RIGHT arm to the original setpoint for climb (0)
                      // if they are not in a sane range, then keep the PID setpoint where it is and stop evaluting motor currents
      // (basically, freeze the arm where it is until some other command changes PID setpoint)
      if ( tickCountRight < 10 )
        ++tickCountRight;
      else {
        // Once tickCount is reached, evaluate current on left and right arms to make sure they are both loaded
        // if left arm is not under loadloaded, DO NOT change setpoint for RIGHT arm
        // if left arm is under load, then it is OK to reinitialize RIGHT arm setpoint
        // use tickCount to only do this evaluation one time
        if ( tickCountRight == 10 ) {
          ++tickCountRight;  // this will stop this code from running more than once
          //      If they are similiar, reset the setpoint
          if ( leftClimber.outputCurrent() >= rightClimber.outputCurrent() - 5 )
            rightClimber.setSetpoint(0);   // this releases the PID to start tracking to setpoint again
        }
      }
    } // RIGHT arm lower logic
    
    // LEFT ARM lower logic
    // Check is Left arm is too much lower than Right arm.
    if ( leftClimber.Arm_Postion_Too_Low(rightClimber.presentEncoderValue(), Constants.Max_Differential) ) {
      // pause PID for LEFT arm at the current position
      leftClimber.setSetpoint(leftClimber.presentEncoderValue());
      tickCountLeft = 0;
    }
    else {
      // ONCE tickCount expires, evaluate the motor currents
      // if they are within a sane range, then resume PID for LEFT arm to the original setpoint for climb (0)
      // if they are not in a sane range, then keep the PID setpoint where it is and stop evaluting motor currents
      // (basically, freeze the arm where it is until some other command changes PID setpoint)
      if ( tickCountLeft < 10 )
        ++tickCountLeft;
      else {
        // Once tickCount is reached, evaluate current on left and right arms to make sure they are both loaded
        // if right arm is not under load, DO NOT change setpoint for LEFT arm
        // if rigth arm is under load, then it is OK to reinitialize LEFT arm setpoint
        // use tickCount to only do this evaluation one time
        if ( tickCountLeft == 10 ) {
          ++tickCountLeft;  // this will stop this code from running more than once
          //      If they are similiar, reset the setpoint
          if ( rightClimber.outputCurrent() >= leftClimber.outputCurrent() - 5 )  // you need to write a function or do logic for this
            leftClimber.setSetpoint(0);   // this releases the PID to start tracking to setpoint again
        }
      }
    }  // LEFT arm lower logic
  }  // execute (for Lower Arms command)



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Set PID setpoint to current position, then disable PIDs
    // This will keep robot stationary if the PIDs get enabled again without resetting postion
    rightClimber.setSetpoint(rightClimber.presentEncoderValue());
    leftClimber.setSetpoint(leftClimber.presentEncoderValue());
    rightClimber.disable();
    leftClimber.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return leftClimber.atSetPoint(false) && rightClimber.atSetPoint(false);
  }
}
