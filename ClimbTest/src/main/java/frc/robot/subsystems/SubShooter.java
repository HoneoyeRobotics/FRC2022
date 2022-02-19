// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SubShooter extends SubsystemBase {
  private VictorSPX shooterMotor;
  private VictorSPX shooterMotor1;

  /** Creates a new SubFeeder. */
  public SubShooter() {
    shooterMotor = new VictorSPX(Constants.CANShooter);
    shooterMotor1 = new VictorSPX(Constants.CANShooter1);

  }

  public void runShooter(double speed) {
    speed = speed * -1;
    shooterMotor.set(ControlMode.PercentOutput, speed);
    shooterMotor1.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
