// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Ball extends SubsystemBase {
  // declare motors
  private VictorSPX shooterMotor;
  private VictorSPX shooter1Motor;
  private VictorSPX feederMotor;
  private VictorSPX pickupMotor;

  /** Creates a new Ball. */
  public Ball() {
    shooterMotor = new VictorSPX(Constants.CANID_ShooterMotor);
    shooter1Motor = new VictorSPX(Constants.CANID_Shooter1Motor);
    shooter1Motor.setInverted(true);
    shooterMotor.setInverted(true);
    feederMotor = new VictorSPX(Constants.CANID_FeederMotor);
    pickupMotor = new VictorSPX(Constants.CANID_PickupMotor);
  }

  public void deployBallPickup() {
  }

  public void runPickUp(double speed) {
    pickupMotor.set(ControlMode.PercentOutput, speed);
  }

  public void runFeeder(double speed) {
    feederMotor.set(ControlMode.PercentOutput, speed);
  }

  public void runShooter() {
    double motorSpeed = Preferences.getDouble("ShooterMotorSpeed", 0.57);
    double motor1Speed = Preferences.getDouble("ShooterMotor1Speed", 0.57);
    shooterMotor.set(ControlMode.PercentOutput, motorSpeed);
    shooter1Motor.set(ControlMode.PercentOutput, motor1Speed);
  }

  public void stopShooter() {
    shooterMotor.set(ControlMode.PercentOutput, 0);
    shooter1Motor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
