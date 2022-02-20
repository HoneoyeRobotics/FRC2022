// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SubInnerArms extends SubsystemBase {
  private CANSparkMax climberLeftInnerMotor;
  private CANSparkMax climberRightInnerMotor;


  /** Creates a new SubOuterArms. */
  public SubInnerArms() {
    climberLeftInnerMotor = new CANSparkMax(Constants.CANCLIM, MotorType.kBrushless);
    climberRightInnerMotor = new CANSparkMax(Constants.CANCRIM, MotorType.kBrushless);
    climberRightInnerMotor.setInverted(true);
  }

  public void resetEncoders() {
    climberRightInnerMotor.getEncoder().setPosition(0);
    climberLeftInnerMotor.getEncoder().setPosition(0);
  } 

  public void moveArms(double speed) {
    climberLeftInnerMotor.set(speed);
    climberRightInnerMotor.set(speed);
  }

  public void moveLeftArm(double speed) {
    SmartDashboard.putNumber("InnerLeftArmOutput", speed);
    climberLeftInnerMotor.set(speed);
  }
  
  public void moveRightArm(double speed) {
    SmartDashboard.putNumber("InnerRightArmOutput", speed);
    climberRightInnerMotor.set(speed);
  }


  @Override
  public void periodic() {
    
    SmartDashboard.putNumber("CRIEncoder", climberRightInnerMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("CLIEncoder", climberLeftInnerMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("CLICurrent", climberLeftInnerMotor.getOutputCurrent());
    SmartDashboard.putNumber("CRICurrent", climberRightInnerMotor.getOutputCurrent());

    // This method will be called once per scheduler run
  }
}
