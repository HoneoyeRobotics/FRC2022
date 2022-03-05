// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.InnerLeftClimber;
import frc.robot.subsystems.InnerRightClimber;
import frc.robot.subsystems.OuterLeftClimber;
import frc.robot.subsystems.OuterRightClimber;

public class GetMeasurement extends CommandBase {
  private InnerRightClimber innerRightClimber;
  private InnerLeftClimber innerLeftClimber;
  private OuterRightClimber outerRightClimber;
  private OuterLeftClimber outerLeftClimber;
  /** Creates a new ResetEncoders. */
  public GetMeasurement(InnerRightClimber innerRightClimber, InnerLeftClimber innerLeftClimber, 
                       OuterRightClimber outerRightClimber, OuterLeftClimber outerLeftClimber) {
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
    innerRightClimber.getMeasurement();
    innerLeftClimber.getMeasurement();
    outerRightClimber.getMeasurement();
    outerLeftClimber.getMeasurement();
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
