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

public class OuterClimber extends PIDSubsystem {
  private CANSparkMax climberLeftOuterMotor;
  private CANSparkMax climberRightOuterMotor;
  /** Creates a new OuterClimber. */
  public OuterClimber() {
    super(new PIDController(0.25, 0, 0));
    climberLeftOuterMotor = new CANSparkMax(Constants.CANID_ClimberLeftOuterMotor, MotorType.kBrushless);
    climberRightOuterMotor = new CANSparkMax(Constants.CANID_ClimberRightOuterMotor, MotorType.kBrushless);
    climberRightOuterMotor.setInverted(true);
    
    climberLeftOuterMotor.setIdleMode(IdleMode.kBrake);
    climberRightOuterMotor.setIdleMode(IdleMode.kBrake);
    resetEncoders();
  }

  public void resetEncoders() {
    climberLeftOuterMotor.getEncoder().setPosition(0);
    climberRightOuterMotor.getEncoder().setPosition(0);
  }

  public void setPosition(int position) {
    if (position < 0)
      position = 0;
    else if (position > 1)
      position = 1;

    double setpoint = 0;
    switch(position){
      case 1:setpoint = Preferences.getDouble("OuterMax", 130);
        break;
      case 0:setpoint = Preferences.getDouble("OuterMin", 0);
        break;
    }
    SmartDashboard.putNumber("OC Set to", setpoint);
    setSetpoint(setpoint);
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
    if (output > Constants.MaxClimberOutput)
      output = Constants.MaxClimberOutput;
    // Use the output here
    climberLeftOuterMotor.set(output);
    climberRightOuterMotor.set(output);
    
    SmartDashboard.putNumber("OC output", output);
    SmartDashboard.putNumber("OC setpoint", setpoint);
  }

  public void runMotor(double speed, boolean runLeft, boolean runRight){
    SmartDashboard.putBoolean("Outer Run Left", runLeft);
    SmartDashboard.putBoolean("Outer Run Right", runRight);
    
    if(runLeft == true)
      climberLeftOuterMotor.set(speed);
    if(runRight  == true)
      climberRightOuterMotor.set(speed);

    
      SmartDashboard.putNumber("OuterArmsPower", speed);
  }



  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    SmartDashboard.putNumber("CLOEncoder", climberLeftOuterMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("CROEncoder", climberRightOuterMotor.getEncoder().getPosition());

    SmartDashboard.putNumber("CLOCurrent", climberLeftOuterMotor.getOutputCurrent());
    SmartDashboard.putNumber("CROCurrent", climberRightOuterMotor.getOutputCurrent());

    return (climberLeftOuterMotor.getEncoder().getPosition() +
    climberRightOuterMotor.getEncoder().getPosition()) / 2;
  }

  public boolean leftAtBottomCurrent() {
    return(climberLeftOuterMotor.getOutputCurrent() > Constants.MaxCurrent);
  }
  public boolean rightAtBottomCurrent() {
      return (climberRightOuterMotor.getOutputCurrent() > Constants.MaxCurrent);
  }
  
}

// if (climberleftOuterMotor.getEncoder() > climberRight.OuterMotor.getEncoder()) {
//   climerleftOuterMotor.set(output * .9);
// }
// else if (climberRightOuterMotor.getEncoder() > climberleftOuterMotor.getEncoder()) {
//   climberRightOuterMotor.set(output * .9);
// }