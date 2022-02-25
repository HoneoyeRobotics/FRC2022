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

  private OuterClimber outerClimber;
  private DoubleSupplier outerHeight;
  //private double currentOuterValue = 0;
  //private double outerPosition = 0;
  private BooleanSupplier leftOnly;
private BooleanSupplier rightOnly;

  /** Creates a new DefaultClimb. */
  public DefaultOuterArms(OuterClimber outerClimber,  DoubleSupplier outerHeight, BooleanSupplier leftOnly, BooleanSupplier rightOnly) {

    addRequirements(outerClimber);
    this.outerClimber = outerClimber;
    this.outerHeight = outerHeight;
    this.leftOnly = leftOnly;
    this.rightOnly = rightOnly;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //outerPosition = outerClimber.getSetpoint();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(outerClimber.isEnabled() == false) {
      double speed = outerHeight.getAsDouble();
      boolean runLeft = rightOnly.getAsBoolean() ? false : true;
      boolean runRight = leftOnly.getAsBoolean() ? false : true;
      if((speed > (Constants.JoystickDeadband * -1)) && (speed < Constants.JoystickDeadband)) {
        speed = 0;
      }
        outerClimber.runMotor(speed, runLeft, runRight);
    }
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(outerClimber.isEnabled() == false)
      outerClimber.runMotor(0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
