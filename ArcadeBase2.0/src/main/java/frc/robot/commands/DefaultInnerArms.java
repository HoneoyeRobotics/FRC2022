// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class DefaultInnerArms extends CommandBase {

  private InnerRightClimber innerRightClimber;
  private InnerLeftClimber innerLeftClimber;
  private DoubleSupplier innerHeight;
  //private double currentInnerValue = 0;
  //private double InnerPosition = 0;
  private BooleanSupplier leftOnly;
private BooleanSupplier rightOnly;

  /** Creates a new DefaultClimb. */
  public DefaultInnerArms(InnerRightClimber innerRightClimber, InnerLeftClimber innerLeftClimber, 
      DoubleSupplier innerHeight, BooleanSupplier leftOnly, BooleanSupplier rightOnly) {

    addRequirements(innerRightClimber);
    addRequirements(innerLeftClimber);

    this.innerRightClimber = innerRightClimber;
    this.innerLeftClimber = innerLeftClimber;
    this.innerHeight = innerHeight;
    this.rightOnly = rightOnly;
    this.leftOnly = leftOnly;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //innerPosition = innerClimber.getSetpoint();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(innerRightClimber.isEnabled() == false  && innerLeftClimber.isEnabled() == false) {
      double speed = innerHeight.getAsDouble();
      boolean runLeft = rightOnly.getAsBoolean() ? false : true;
      boolean runRight = leftOnly.getAsBoolean() ? false : true;
      if((speed > (Constants.JoystickDeadband * -1)) && (speed < Constants.JoystickDeadband)) {
        speed = 0;
      }
        innerRightClimber.runMotor(speed, runRight);
        innerLeftClimber.runMotor(speed, runLeft);
    }
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(innerRightClimber.isEnabled() == false && innerLeftClimber.isEnabled() == false)
      innerRightClimber.runMotor(0, true);
      innerLeftClimber.runMotor(0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
