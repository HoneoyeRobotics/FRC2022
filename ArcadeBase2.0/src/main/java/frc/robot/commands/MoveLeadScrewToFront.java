// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LeadScrew;

public class MoveLeadScrewToFront extends CommandBase {
  private LeadScrew m_leadScrew;
  private double speed = 0.0;
  /** Creates a new MoveLeadScrew. */
  public MoveLeadScrewToFront(LeadScrew leadScrew, double d) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(leadScrew);
    m_leadScrew = leadScrew;
    speed = d;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("State", "Move Lead Screw Started");
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //if (RobotContainer.climbContinue) {
      m_leadScrew.moveArms(speed);
    //}

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_leadScrew.moveArms(0);
    
    SmartDashboard.putString("State", "Move Lead Screw Ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
//    return (counter >= counterFinish);
      return m_leadScrew.armsAtFront();
  }
}
