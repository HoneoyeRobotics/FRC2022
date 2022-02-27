// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainTank;


public class DriveTank extends CommandBase {
  private DriveTrainTank m_tank;
  DoubleSupplier m_tankLeftAxis;
  DoubleSupplier m_tankRightAxis;
  /** Creates a new DriveTank. */
  public DriveTank(DriveTrainTank tank, DoubleSupplier tankLeftAxis, DoubleSupplier tankRightAxis) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(tank);
    m_tank = tank; 
    m_tankLeftAxis = tankLeftAxis;
    m_tankLeftAxis = tankRightAxis;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double tempRight = m_tankRightAxis.getAsDouble();
    double rightAxis = ((tempRight > Constants.JoystickDeadband) || (tempRight < (-1 * Constants.JoystickDeadband))) ? tempRight : 0.0;

    double tempLeft = m_tankLeftAxis.getAsDouble();
    double leftAxis = ((tempLeft > Constants.JoystickDeadband) || (tempLeft < (-1 * Constants.JoystickDeadband))) ? tempLeft : 0.0;

    m_tank.driveTank(leftAxis, rightAxis);
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
