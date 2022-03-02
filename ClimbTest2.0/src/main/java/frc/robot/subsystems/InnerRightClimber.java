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

public class InnerRightClimber extends PIDSubsystem {
  private boolean abortRaise = false;
  private CANSparkMax climberInnerRightMotor;
  /** Creates a new InnerRightClimber. */
  public InnerRightClimber() {
    super(new PIDController(Constants.kpClimber, 0, 0));
    climberInnerRightMotor = new CANSparkMax(Constants.CANID_ClimberRightInnerMotor, MotorType.kBrushless);
    climberInnerRightMotor.setInverted(true);
    climberInnerRightMotor.setIdleMode(IdleMode.kBrake);
    resetEncoders();
  }

  public void resetEncoders() {
    climberInnerRightMotor.getEncoder().setPosition(0);
  }

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

  public boolean atSetPoint(boolean movingUp) {
    String direction = "hello";
    boolean retVal = false;
    if(movingUp){
      direction = "MovingUp";
      if(getSetpoint() <= getMeasurement()){
        direction = "AtTop";
        retVal = true;
      }
    }
    else {
      direction = "MovingDown";
      if(getSetpoint() >= getMeasurement()){
        direction = "AtBottom";
        retVal = true;
      }
    }
    SmartDashboard.putString("InnerArmsLocation", direction);
    return retVal;
  }
  
  public void resetAbortRaise() {
    abortRaise = false;
  }

  public boolean raiseCurrentBad() {
    return (climberInnerRightMotor.getOutputCurrent() > Constants.MaxRaiseCurrent);
  }

  public boolean atBottomCurrent() {
    return (climberInnerRightMotor.getOutputCurrent() > Constants.MaxCurrent);
  }

  public double outputCurrent() {
    return climberInnerRightMotor.getOutputCurrent();
  }

  public double presentEncoderValue() {
    return climberInnerRightMotor.getEncoder().getPosition();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    if(abortRaise) output = 0;

    climberInnerRightMotor.set(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    SmartDashboard.putNumber("CRICurrent", climberInnerRightMotor.getOutputCurrent());
    SmartDashboard.putNumber("CRIEncoder", climberInnerRightMotor.getEncoder().getPosition());

    return climberInnerRightMotor.getEncoder().getPosition();
  }

  public void runMotor(double speed, boolean run){
    if(run == true)
      climberInnerRightMotor.set(speed);
      SmartDashboard.putNumber("OuterArmsPower", speed);
  }
}
