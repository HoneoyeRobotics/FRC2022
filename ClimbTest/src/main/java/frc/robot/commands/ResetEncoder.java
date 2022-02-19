// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubInnerArms;
import frc.robot.subsystems.SubOuterArms;

public class ResetEncoder extends CommandBase {
  private SubInnerArms m_innerArms;
  private SubOuterArms m_outerArms;
  /** Creates a new ResetEncoder. */
  public ResetEncoder(SubOuterArms outerArms, SubInnerArms innerArms) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(innerArms);
    addRequirements(outerArms);
    m_innerArms = innerArms;
    m_outerArms = outerArms;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_innerArms.resetEncoders();
    m_outerArms.resetEncoders();}

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
