// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class DefaultClimb extends CommandBase {

  private OuterClimber outerClimber;
  private InnerClimber innerClimber;
  private LeadScrew leadScrew;
  private DoubleSupplier outerHeight;
  private DoubleSupplier innerHeight;
  private DoubleSupplier movementValue;
  private double currentInnerValue = 0;
  private double currentOuterValue = 0;
  private double currentMoveValue = 0;
  private double innerPosition = 0;
  private double outerPosition = 0;

  /** Creates a new DefaultClimb. */
  public DefaultClimb(OuterClimber outerClimber, InnerClimber innerClimber, LeadScrew leadScrew, DoubleSupplier innerHeight,
      DoubleSupplier outerHeight, DoubleSupplier movementValue) {

    addRequirements(outerClimber, innerClimber, leadScrew);
    this.outerClimber = outerClimber;
    this.outerHeight = outerHeight;
    this.innerClimber = innerClimber;
    this.innerHeight = innerHeight;
    this.leadScrew = leadScrew;
    this.movementValue = movementValue;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    innerPosition = innerClimber.getSetpoint();
    outerPosition = outerClimber.getSetpoint();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentMoveValue = movementValue.getAsDouble();
    currentInnerValue = innerHeight.getAsDouble();
    currentOuterValue = outerHeight.getAsDouble();
    leadScrew.moveArms(currentMoveValue);
    innerPosition += (currentInnerValue * Constants.ArmVerticalJoystickEncoderMovement);
    innerClimber.setPosition(innerPosition);
    innerPosition = innerClimber.getSetpoint();
    outerClimber.setPosition(outerPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leadScrew.moveArms(0);
    innerClimber.setPosition(0);
    leadScrew.moveArms(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
