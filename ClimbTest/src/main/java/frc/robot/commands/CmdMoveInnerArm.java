// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubInnerArms;

public class CmdMoveInnerArm extends CommandBase {
private boolean m_left;
private double m_speed;


  private SubInnerArms innerArms;
  /** Creates a new CmdRaiseOuterArms. */
  public CmdMoveInnerArm(SubInnerArms innerArms, boolean left, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(innerArms);
    this.innerArms = innerArms;
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
       innerArms.moveLeftArm(mySpeed);
    else
      innerArms.moveRightArm(mySpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    innerArms.moveLeftArm(0);
    innerArms.moveRightArm(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
