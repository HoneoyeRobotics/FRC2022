// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Preferences;
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
    double maxLSMCurrent = Preferences.getDouble("MaxLSMCurrent", 30.0);
    double statorCurrent = leadScrewMotor.getStatorCurrent();
    if((statorCurrent > maxLSMCurrent) || (statorCurrent < (-1 * maxLSMCurrent))) {
      speed = 0;
    }
    else {
    leadScrewMotor.set(ControlMode.PercentOutput, speed);
    SmartDashboard.putNumber("LeadScrewPower", speed);
    }
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LSMEncoder", leadScrewMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("LSMCurrent", leadScrewMotor.getStatorCurrent());
  }
}
