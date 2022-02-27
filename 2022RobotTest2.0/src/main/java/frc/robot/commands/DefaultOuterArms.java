// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class DefaultOuterArms extends CommandBase {

  private OuterRightClimber outerRightClimber;
  private OuterLeftClimber outerLeftClimber;
  private DoubleSupplier outerHeight;
  //private double currentOuterValue = 0;
  //private double outerPosition = 0;
  private BooleanSupplier leftOnly;
private BooleanSupplier rightOnly;

  /** Creates a new DefaultClimb. */
  public DefaultOuterArms(OuterRightClimber outerRightClimber, OuterLeftClimber outerLeftClimber, 
      DoubleSupplier outerHeight, BooleanSupplier leftOnly, BooleanSupplier rightOnly) {

    addRequirements(outerRightClimber);
    addRequirements(outerLeftClimber);

    this.outerRightClimber = outerRightClimber;
    this.outerLeftClimber = outerLeftClimber;
    this.outerHeight = outerHeight;
    this.rightOnly = rightOnly;
    this.leftOnly = leftOnly;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //outerPosition = outerClimber.getSetpoint();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(outerRightClimber.isEnabled() == false  && outerLeftClimber.isEnabled() == false) {
      double speed = outerHeight.getAsDouble();
      boolean runLeft = rightOnly.getAsBoolean() ? false : true;
      boolean runRight = leftOnly.getAsBoolean() ? false : true;
      if((speed > (Constants.JoystickDeadband * -1)) && (speed < Constants.JoystickDeadband)) {
        speed = 0;
      }
        outerRightClimber.runMotor(speed, runRight);
        outerLeftClimber.runMotor(speed, runLeft);
    }
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(outerRightClimber.isEnabled() == false && outerLeftClimber.isEnabled() == false)
      outerRightClimber.runMotor(0, true);
      outerLeftClimber.runMotor(0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
