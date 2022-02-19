// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class OuterClimber extends PIDSubsystem {
  /** Creates a new OuterClimber. */
  public OuterClimber() {
    super(new PIDController(0, 0, 0));
    climberLeftOuterMotor = new CANSparkMax(Constants.CANID_ClimberLeftOuterMotor, MotorType.kBrushless);
    climberRightOuterMotor = new CANSparkMax(Constants.CANID_ClimberRightOuterMotor, MotorType.kBrushless);
    climberRightOuterMotor.setInverted(true);
    leadScrewMotor = new TalonSRX(Constants.CANID_LeadScrewMotor);
    resetEncoders();
  }

  public void resetEncoders() {
    climberLeftOuterMotor.getEncoder().setPosition(0);
    climberRightOuterMotor.getEncoder().setPosition(0);
    leadScrewMotor.setSelectedSensorPosition(0);
  }
  private CANSparkMax climberLeftOuterMotor;
  private CANSparkMax climberRightOuterMotor;
  private TalonSRX leadScrewMotor;

  public void setPosition(double position) {
    if(position < 0)
      position = 0;
    else if (position > Constants.OuterLeftMax)
      position = Constants.OuterLeftMax;
      setSetpoint(position);
  }

  public boolean atTop() {
    double avgPosition = (climberLeftOuterMotor.getEncoder().getPosition() +
        climberRightOuterMotor.getEncoder().getPosition()) / 2;

    return (avgPosition + Constants.ArmHorizontalEncoderDeadband >= Constants.ArmHorizontalEncoderMaxValue)
        || (avgPosition - Constants.ArmHorizontalEncoderDeadband >= Constants.ArmHorizontalEncoderMaxValue);
  }

  public boolean atBottom() {
    double avgPosition = (climberLeftOuterMotor.getEncoder().getPosition() +
        climberRightOuterMotor.getEncoder().getPosition()) / 2;

    return (avgPosition + Constants.ArmVerticalEncoderDeadband <= 0)
        || (avgPosition - Constants.ArmVerticalEncoderDeadband <= 0);
  }

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
  public void useOutput(double output, double setpoint) {
    // Use the output here
    climberLeftOuterMotor.set(output);
    climberRightOuterMotor.set(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return (climberLeftOuterMotor.getEncoder().getPosition() +
    climberRightOuterMotor.getEncoder().getPosition()) / 2;
  }
}
