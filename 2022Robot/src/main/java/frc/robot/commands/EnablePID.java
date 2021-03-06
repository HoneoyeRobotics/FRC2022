// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.InnerClimber;
import frc.robot.subsystems.OuterClimber;

public class EnablePID extends CommandBase {
  private InnerClimber innerClimber;
  private OuterClimber outerClimber;
  /** Creates a new EnablePID. */
  public EnablePID(InnerClimber innerClimber, OuterClimber outerClimber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.innerClimber = innerClimber;
    this.outerClimber = outerClimber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    innerClimber.resetAbortRaise();
    innerClimber.enable();
    outerClimber.enable();
    SmartDashboard.putBoolean("PIDsEnabled", true);}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
