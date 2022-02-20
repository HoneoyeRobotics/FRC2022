// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubLeadScrew;

public class CmdRunLeadScrew extends CommandBase {
  private SubLeadScrew m_leadScrew;
  DoubleSupplier leadScrewPower;
  /** Creates a new CmdRunLeadScrew. */
  public CmdRunLeadScrew(SubLeadScrew leadScrew, DoubleSupplier leadScrewAxis) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(leadScrew);
    m_leadScrew = leadScrew;
    leadScrewPower = leadScrewAxis;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("LeadScrew Mode", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_leadScrew.moveArms(leadScrewPower.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("LeadScrew Mode", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
