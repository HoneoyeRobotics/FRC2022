// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.OuterClimber;

public class RaiseOuterArms extends CommandBase {

  private OuterClimber climber;
  private boolean bombout = false;
  private int counter = 0;

  /** Creates a new RaiseRearArm. */
  public RaiseOuterArms(OuterClimber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
    this.climber = climber;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.enable();
    climber.setPosition(1);
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(counter++ >= Constants.CounterValue) {
      if(climber.raiseCurrentBad()) {
       bombout = true;
      }
    }
    SmartDashboard.putBoolean("BomboutOuter", bombout);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(bombout) {
      climber.disable();
      climber.runMotor(0, true, true);
    }
    counter = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return bombout || climber.atSetPoint(true);
  }
}
