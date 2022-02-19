// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubOuterArms;

public class CmdMoveOuterArm extends CommandBase {
private boolean m_left;
private double m_speed;


  private SubOuterArms outerArms;
  /** Creates a new CmdRaiseOuterArms. */
  public CmdMoveOuterArm(SubOuterArms outerArms, boolean left, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(outerArms);
    this.outerArms = outerArms;
    m_left = left;
    m_speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double mySpeed = m_speed;
    if (m_left) 
       outerArms.moveLeftArm(mySpeed);
    else
      outerArms.moveRightArm(mySpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    outerArms.moveLeftArm(0);
    outerArms.moveRightArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
