// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class EnablePID extends CommandBase {
  private InnerLeftClimber innerLeftClimber;
  private OuterRightClimber outerRightClimber;
  private OuterLeftClimber outerLeftClimber;
  private InnerRightClimber innerRightClimber;

  /** Creates a new ResetArms. */
  public EnablePID(InnerLeftClimber innerLeftClimber, InnerRightClimber innerRightClimber, 
                   OuterLeftClimber outerLeftClimber, OuterRightClimber outerRightClimber) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(innerRightClimber);
    addRequirements(innerLeftClimber);
    addRequirements(outerRightClimber);
    addRequirements(outerLeftClimber);

    this.innerRightClimber = innerRightClimber;
    this.innerLeftClimber = innerLeftClimber;
    this.outerRightClimber = outerRightClimber;
    this.outerLeftClimber = outerLeftClimber;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void initialize() {
    innerRightClimber.resetAbortRaise();
    innerLeftClimber.resetAbortRaise();
    outerRightClimber.resetAbortRaise();
    outerLeftClimber.resetAbortRaise();
    innerRightClimber.enable();
    innerLeftClimber.enable();
    outerRightClimber.enable();
    outerLeftClimber.enable();
    SmartDashboard.putBoolean("PIDsEnabled", true);
  }

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
