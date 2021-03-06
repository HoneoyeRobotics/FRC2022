// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubPickup;

public class CmdRunPickup extends CommandBase {
  private SubPickup m_pickup;

  /** Creates a new CmdRunPickup. */
  public CmdRunPickup(SubPickup pickup) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pickup);
    m_pickup = pickup;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pickupSpeed = Preferences.getDouble("PickupSpeed", .6);
    m_pickup.runPickup(pickupSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pickup.runPickup(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
