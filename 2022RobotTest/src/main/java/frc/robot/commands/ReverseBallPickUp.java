// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Ball;

public class ReverseBallPickUp extends CommandBase {
  private Ball ball;

  /** Creates a new ReverseBallPickUp. */
  public ReverseBallPickUp(Ball ball) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ball);
    this.ball = ball;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ball.runPickUp(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ball.runPickUp(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}