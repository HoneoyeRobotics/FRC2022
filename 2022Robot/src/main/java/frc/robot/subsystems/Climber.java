// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  // private CANSparkMax climberLeftOuterMotor;
  // private CANSparkMax climberRightOuterMotor;
  // private TalonSRX leadScrewMotor;

  // /** Creates a new Climber. */
  // public Climber() {
  //   climberLeftOuterMotor = new CANSparkMax(Constants.CANID_ClimberLeftOuterMotor, MotorType.kBrushless);
  //   climberRightOuterMotor = new CANSparkMax(Constants.CANID_ClimberRightOuterMotor, MotorType.kBrushless);
  //   leadScrewMotor = new TalonSRX(Constants.CANID_LeadScrewMotor);
  // }

  // public void resetEncoders() {
  //   climberLeftOuterMotor.getEncoder().setPosition(0);
  //   climberRightOuterMotor.getEncoder().setPosition(0);
  // }

  // public boolean outerArmsAtTop() {
  //   double avgPosition = (climberLeftOuterMotor.getEncoder().getPosition() +
  //       climberRightOuterMotor.getEncoder().getPosition()) / 2;

  //   return (avgPosition + Constants.ArmVerticalEncoderDeadband >= Constants.ArmVerticalEncoderMaxValue)
  //       || (avgPosition - Constants.ArmVerticalEncoderDeadband >= Constants.ArmVerticalEncoderMaxValue);
  // }

  // public boolean outerArmsAtBottom() {
  //   double avgPosition = (climberLeftOuterMotor.getEncoder().getPosition() +
  //       climberRightOuterMotor.getEncoder().getPosition()) / 2;

  //   return (avgPosition + Constants.ArmVerticalEncoderDeadband <= 0)
  //       || (avgPosition - Constants.ArmVerticalEncoderDeadband <= 0);
  // }

  // public void runOuterArms(double speed) {

  //   // if we are moving up, and it is at the top, OR we are moving down, and the
  //   // arms are at the bottom, don't move
  //   if (speed > 0 && outerArmsAtTop() || (speed < 0 && outerArmsAtBottom())) {
  //     speed = 0;
  //   }

  //   climberLeftOuterMotor.set(speed);
  //   climberRightOuterMotor.set(speed);
  // }

  // public boolean armsFullyIn() {
  //   double leadScrewEncoder = leadScrewMotor.getSelectedSensorPosition();
  //   return (leadScrewEncoder + Constants.ArmHorizontalEncoderDeadband <= 0)
  //       || (leadScrewEncoder - Constants.ArmHorizontalEncoderDeadband <= 0);
  // }

  // public boolean armsFullyOut() {
  //   double leadScrewEncoder = leadScrewMotor.getSelectedSensorPosition();
  //   return (leadScrewEncoder + Constants.ArmVerticalEncoderDeadband >= Constants.ArmVerticalEncoderMaxValue)
  //       || (leadScrewEncoder - Constants.ArmVerticalEncoderDeadband >= Constants.ArmVerticalEncoderMaxValue);
  // }

  // public void moveArms(double speed) {
  //   if (speed > 0 && armsFullyOut() || (speed < 0 && armsFullyIn())) {
  //     speed = 0;
  //   }
  //   leadScrewMotor.set(ControlMode.PercentOutput, speed);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
