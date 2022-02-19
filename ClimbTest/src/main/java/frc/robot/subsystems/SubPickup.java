// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SubPickup extends SubsystemBase {
  private VictorSPX pickupMotor;
  /** Creates a new SubPickup. */
  public SubPickup() {
    pickupMotor = new VictorSPX(Constants.CANPickup);
  }

  public void runPickup(double speed) {
    pickupMotor.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
