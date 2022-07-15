// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class FollowLimeLight extends CommandBase {
  /** Creates a new FollowLimeLight. */
  private DriveTrain driveTrain;
  public FollowLimeLight(DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-suits");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  SmartDashboard.putNumber("LL X", tx.getDouble(0.0));
  double zRotation = 0;
  double place = tx.getDouble(0.0);
    if(place >  3)
      zRotation = 0.5;
    else if (place < -3)
      zRotation = -0.5;
    driveTrain.drive( zRotation,0);
SmartDashboard.putNumber("moving",zRotation);
    SmartDashboard.putNumber("LL Y", ty.getDouble(0.0));
    SmartDashboard.putNumber("LL A", ta.getDouble(0.0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    driveTrain.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
