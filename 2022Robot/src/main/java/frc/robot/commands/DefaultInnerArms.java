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

  private InnerClimber innerClimber;
  private DoubleSupplier innerHeight;
  private double currentInnerValue = 0;
  private double outerPosition = 0;
  private BooleanSupplier leftOnly;
private BooleanSupplier rightOnly;
  /** Creates a new DefaultClimb. */
  public DefaultInnerArms( InnerClimber innerClimber, DoubleSupplier innerHeight, BooleanSupplier leftOnly, BooleanSupplier rightOnly)
{
    addRequirements(innerClimber);
    this.innerClimber = innerClimber;
    this.innerHeight = innerHeight;
    this.leftOnly = leftOnly;
    this.rightOnly = rightOnly;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(innerClimber.isEnabled() == false)
    {
      boolean runLeft = rightOnly.getAsBoolean() ? false : true;
      boolean runRight = leftOnly.getAsBoolean() ? false : true;
      innerClimber.runMotor(innerHeight.getAsDouble(), runLeft, runRight);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(innerClimber.isEnabled() == false)
    innerClimber.runMotor(0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

