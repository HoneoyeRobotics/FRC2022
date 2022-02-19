// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class InnerClimber extends PIDSubsystem {
  /** Creates a new InnerClimber. */
  public InnerClimber() {
    super(new PIDController(1.3, 0.0, 0.7));
    climberLeftInnerMotor = new CANSparkMax(Constants.CANID_ClimberLeftInnerMotor, MotorType.kBrushless);
    climberRightInnerMotor = new CANSparkMax(Constants.CANID_ClimberRightInnerMotor, MotorType.kBrushless);
    climberRightInnerMotor.setInverted(true);
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
     
      case 1:
        setpoint = Constants.InnerLeftMax;
        break;
        case 0:
        setpoint = 0;
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
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return (climberLeftInnerMotor.getEncoder().getPosition() +
        climberRightInnerMotor.getEncoder().getPosition()) / 2;

  }
}
