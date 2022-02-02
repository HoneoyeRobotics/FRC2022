// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Ball extends SubsystemBase {
  //declare motors
  private VictorSPX shooterMotor;
  private VictorSPX feederMotor;
  private VictorSPX pickupMotor;

  /** Creates a new Ball. */
  public Ball() {
    shooterMotor = new VictorSPX(Constants.CANID_ShooterMotor);
    feederMotor = new VictorSPX(Constants.CANID_FeederMotor);
    pickupMotor = new VictorSPX(Constants.CANID_PickupMotor);
  }

  public void deployBallPickup() {}
  public void runFeeder(double speed) {
    
    shooterMotor.set(ControlMode.PercentOutput, speed);
  }
  public void runPickUp(double speed) {
    pickupMotor.set(ControlMode.PercentOutput, speed);
  }
  
  public void runShooter() {
    shooterMotor.set(ControlMode.PercentOutput, 0.60);
  }
  public void stopShooter() {
    shooterMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
