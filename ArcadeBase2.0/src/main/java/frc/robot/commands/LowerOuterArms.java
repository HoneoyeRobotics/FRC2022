// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberPosition;
import frc.robot.subsystems.OuterLeftClimber;
import frc.robot.subsystems.OuterRightClimber;
import frc.robot.Constants;

public class LowerOuterArms extends CommandBase {

  private OuterRightClimber rightClimber;
  private OuterLeftClimber leftClimber;
  private boolean resetCommand = false;
  private static int tickCountRight = 11;
  private static int tickCountLeft = 11;

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

    rightClimber.setPosition(ClimberPosition.start);;
    leftClimber.setPosition(ClimberPosition.start);
    tickCountLeft   = 0;
    tickCountRight = 0;
    SmartDashboard.putString("State", "Lower Outer Started");
  }

  // Called every time the scheduler runs while the command is scheduled.
    // NOTE: be sure to account for negative arm positions
// This function takes two arm positions and compares them to make sure the "compare" arm is within the MAX
// differential position of the "other" arm.  This funtion will normalize the positions to account for negative values.
// It returns TRUE if the value of the compare arm is out of bounds and FALSE if compare arm is in expected range.
  @Override
  public void execute() {
    if (rightClimber.atPosition() && leftClimber.atPosition()) {
      double differential = rightClimber.outputCurrent() - leftClimber.outputCurrent();
      if (differential < Constants.Max_Differential && differential > -1 * Constants.Max_Differential) {
        rightClimber.setPosition(ClimberPosition.bottom);
        leftClimber.setPosition(ClimberPosition.bottom);
        resetCommand = true;
      }
      else {
        rightClimber.setPosition(ClimberPosition.top);
        leftClimber.setPosition(ClimberPosition.top);
        resetCommand = true;
      }
    }
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
    // rightClimber.disable();
    // leftClimber.disable();
    
    SmartDashboard.putString("State", "Lower Outer Ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return resetCommand;
  }
}
