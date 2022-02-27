// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class PovCommand extends CommandBase {
  private IntSupplier pov;
  /** Creates a new POVCommand. */
  public PovCommand(IntSupplier pov) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pov = pov;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int angle = pov.getAsInt();
    SmartDashboard.putNumber("angle", angle);
    switch (angle) {
      default:
        SmartDashboard.putNumber("PovLocation", -1);
        break;
      case 0:
        break;
      case 45:
        break;
      case 90:
        SmartDashboard.putNumber("PovLocation", angle);
        break;
      case 135:
        break;
      case 180:
        break;
      case 225:
        break;
      case 270:
        break;
      case 315:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
