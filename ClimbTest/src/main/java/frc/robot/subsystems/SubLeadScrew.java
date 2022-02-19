// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SubLeadScrew extends SubsystemBase {

  private TalonSRX leadScrewMotor;
  
  /** Creates a new LeadScrew. */
  public SubLeadScrew() {

    leadScrewMotor = new TalonSRX(Constants.CANLSM);

  }

  public void resetEncoders() {
    leadScrewMotor.setSelectedSensorPosition(0);
  }

  public void moveArms(double speed) {
    leadScrewMotor.set(ControlMode.PercentOutput, speed);
    SmartDashboard.putNumber("Lead Screw Power", speed);
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
