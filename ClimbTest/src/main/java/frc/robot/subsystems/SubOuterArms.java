// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SubOuterArms extends SubsystemBase {
  private CANSparkMax climberLeftOuterMotor;
  private CANSparkMax climberRightOuterMotor;


  /** Creates a new SubOuterArms. */
  public SubOuterArms() {
    climberLeftOuterMotor = new CANSparkMax(Constants.CANCLOM, MotorType.kBrushless);
    climberRightOuterMotor = new CANSparkMax(Constants.CANCROM, MotorType.kBrushless);
    climberRightOuterMotor.setInverted(true);
  }

  public void resetEncoders() {
    climberRightOuterMotor.getEncoder().setPosition(0);
    climberLeftOuterMotor.getEncoder().setPosition(0);
  } 

  public void moveArms(double speed) {
    climberLeftOuterMotor.set(speed);
    climberRightOuterMotor.set(speed);
  }

  public void moveLeftArm(double speed) {
    SmartDashboard.putNumber("Outer LeftArmSpeed", speed);
    climberLeftOuterMotor.set(speed);
  }
  
  public void moveRightArm(double speed) {
    SmartDashboard.putNumber("Outer RightArmSpeed", speed);
    climberRightOuterMotor.set(speed);
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("CLOEncoder", climberLeftOuterMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("CROEncoder", climberRightOuterMotor.getEncoder().getPosition());
    // This method will be called once per scheduler run
  }
}
