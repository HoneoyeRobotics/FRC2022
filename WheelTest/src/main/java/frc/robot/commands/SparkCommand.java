// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SparkSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class SparkCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SparkSubsystem m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SparkCommand(SparkSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    SmartDashboard.putBoolean("Spark Running",true);
//    SmartDashboard.putNumber("motorSpeed", 0.0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // This method will be called once per scheduler run
    double power = SmartDashboard.getNumber("SparkPower", 0.0);
    SmartDashboard.putNumber("SparkPowerOut", power);
    m_subsystem.drive(power);
    
    
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    SmartDashboard.putBoolean("Running Command",false);
    SmartDashboard.putNumber("SparkPowerOut", 0);
    m_subsystem.drive(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
