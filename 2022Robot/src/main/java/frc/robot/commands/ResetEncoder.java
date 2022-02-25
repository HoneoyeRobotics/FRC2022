// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.InnerClimber;
import frc.robot.subsystems.OuterClimber;

public class ResetEncoder extends CommandBase {
  private InnerClimber innerClimber;
  private OuterClimber outerclimber;
  /** Creates a new ResetEncoder. */
  public ResetEncoder(OuterClimber outerClimber, InnerClimber innerClimber) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(innerClimber);
    addRequirements(outerClimber);
    this.innerClimber = innerClimber;
    this.outerclimber = outerclimber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    innerClimber.resetEncoders();
    outerclimber.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
