// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class DefaultClimb extends CommandBase {

  private Climber climber;
  private DoubleSupplier innerHeight;
  private DoubleSupplier outerHeight;
  private DoubleSupplier movementValue;
  
  /** Creates a new MoveArms. */
  public DefaultClimb(Climber climber, DoubleSupplier innerHeight, DoubleSupplier outerHeight, DoubleSupplier movementValue) {
    
  addRequirements(climber);
  this.climber = climber;
  this.innerHeight = innerHeight;
  this.outerHeight = outerHeight;
  this.movementValue = movementValue;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    climber.moveArms(movementValue.getAsDouble());
    climber.runInnerArms(innerHeight.getAsDouble());
    climber.runOuterArms(outerHeight.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.moveArms(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
