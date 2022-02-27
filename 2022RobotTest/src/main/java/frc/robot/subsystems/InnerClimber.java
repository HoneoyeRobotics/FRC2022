// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class InnerClimber extends PIDSubsystem {
  /** Creates a new InnerClimber. */
  public InnerClimber() {
    super(new PIDController(0.25, 0, 0));
    climberLeftInnerMotor = new CANSparkMax(Constants.CANID_ClimberLeftInnerMotor, MotorType.kBrushless);
    climberRightInnerMotor = new CANSparkMax(Constants.CANID_ClimberRightInnerMotor, MotorType.kBrushless);
    climberRightInnerMotor.setInverted(true);

    climberLeftInnerMotor.setIdleMode(IdleMode.kBrake);
    climberRightInnerMotor.setIdleMode(IdleMode.kBrake);
    resetEncoders();
  }

  public void resetEncoders() {

    climberLeftInnerMotor.getEncoder().setPosition(0);
    climberRightInnerMotor.getEncoder().setPosition(0);
  }

  private CANSparkMax climberRightInnerMotor;
  private CANSparkMax climberLeftInnerMotor;

  public void setPosition(int position) {
    if (position < 0)
      position = 0;
    else if (position > 1)
      position = 1;

    double setpoint = 0;
    switch(position){
      case 1:setpoint = Preferences.getDouble("InnerMax", 108);
        break;
      case 0:setpoint = Preferences.getDouble("InnerMin", 0);
        break;
    }
      
    setSetpoint(setpoint);
  }

  public boolean atTop() {
    double avgPosition = (climberLeftInnerMotor.getEncoder().getPosition() +
        climberRightInnerMotor.getEncoder().getPosition()) / 2;

    return (avgPosition + Constants.ArmHorizontalEncoderDeadband >= Constants.ArmHorizontalEncoderMaxValue)
        || (avgPosition - Constants.ArmHorizontalEncoderDeadband >= Constants.ArmHorizontalEncoderMaxValue);
  }

  public boolean atBottom() {
    double avgPosition = (climberLeftInnerMotor.getEncoder().getPosition() +
        climberRightInnerMotor.getEncoder().getPosition()) / 2;

    return (avgPosition + Constants.ArmVerticalEncoderDeadband <= 0)
        || (avgPosition - Constants.ArmVerticalEncoderDeadband <= 0);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    climberLeftInnerMotor.set(output);
    climberRightInnerMotor.set(output);
    SmartDashboard.putNumber("IC setpoint", output);
    SmartDashboard.putNumber("IC setpoint", setpoint);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    SmartDashboard.putNumber("CLIEncoder", climberLeftInnerMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("CRIEncoder", climberRightInnerMotor.getEncoder().getPosition());

    SmartDashboard.putNumber("CLICurrent", climberLeftInnerMotor.getOutputCurrent());
    SmartDashboard.putNumber("CRICurrent", climberRightInnerMotor.getOutputCurrent());

    return (climberLeftInnerMotor.getEncoder().getPosition() +
        climberRightInnerMotor.getEncoder().getPosition()) / 2;

  }
  public void runMotor(double speed, boolean runLeft, boolean runRight){
    SmartDashboard.putBoolean("Inner Run Left", runLeft);
    SmartDashboard.putBoolean("Inner Run Right", runRight);
    
    if(runLeft == true)
      climberLeftInnerMotor.set(speed);
    if(runRight == true)
      climberRightInnerMotor.set(speed);

    SmartDashboard.putNumber("InnerArmsPower", speed);
  }

  public boolean leftAtBottomCurrent() {
    return(climberLeftInnerMotor.getOutputCurrent() > Constants.MaxCurrent);
  }

  public boolean rightAtBottomCurrent() {
    return (climberRightInnerMotor.getOutputCurrent() > Constants.MaxCurrent);
  }
}
