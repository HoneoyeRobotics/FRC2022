// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class DefaultLeadScrew extends CommandBase {

  private LeadScrew leadScrew;
  private DoubleSupplier movementValue;
  private double currentMoveValue = 0;

  /** Creates a new DefaultClimb. */
  public DefaultLeadScrew(LeadScrew leadScrew, DoubleSupplier movementValue) {

    addRequirements(leadScrew);
    this.leadScrew = leadScrew;
    this.movementValue = movementValue;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentMoveValue = movementValue.getAsDouble();
   
    leadScrew.moveArms(currentMoveValue);
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leadScrew.moveArms(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
