// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class LockSetpoints extends CommandBase {
  private InnerLeftClimber innerLeftClimber;
  private OuterRightClimber outerRightClimber;
  private OuterLeftClimber outerLeftClimber;
  private InnerRightClimber innerRightClimber;

  /** Creates a new ResetArms. */
  public LockSetpoints(InnerLeftClimber innerLeftClimber, InnerRightClimber innerRightClimber, 
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

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    innerRightClimber.setSetpoint(innerRightClimber.presentEncoderValue());;
    innerLeftClimber.setSetpoint(innerLeftClimber.presentEncoderValue());
    outerRightClimber.setSetpoint(outerRightClimber.presentEncoderValue());
    outerLeftClimber.setSetpoint(outerLeftClimber.presentEncoderValue());
  }

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
