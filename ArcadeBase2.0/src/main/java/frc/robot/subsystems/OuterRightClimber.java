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

public class OuterRightClimber extends PIDSubsystem {
  private boolean abortRaise = false;
  private CANSparkMax climberOuterRightMotor;
  /** Creates a new OuterRightClimber. */
  public OuterRightClimber() {
    super(new PIDController(Constants.kpClimber, 0, 0));
    climberOuterRightMotor = new CANSparkMax(Constants.CANID_ClimberRightOuterMotor, MotorType.kBrushless);
    climberOuterRightMotor.setInverted(true);
    climberOuterRightMotor.setIdleMode(IdleMode.kBrake);
    resetEncoders();
  }

  public void resetEncoders() {
    climberOuterRightMotor.getEncoder().setPosition(0);
  }

  public void setPosition(int position) {
    double setpoint = 0;
    
    if (position < 0)
      position = 0;
    else if (position > 1)
      position = 1;


    switch(position){
      case 1:setpoint = Preferences.getDouble("OuterMax", 108);
        break;
      case 0:setpoint = Preferences.getDouble("OuterMin", 0);
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
    SmartDashboard.putString("OuterArmsLocation", direction);
    return retVal;
  }
  
  public void resetAbortRaise() {
    abortRaise = false;
  }

  public boolean raiseCurrentBad() {
    return (climberOuterRightMotor.getOutputCurrent() > Constants.MaxRaiseCurrent);
  }

  public boolean atBottomCurrent() {
    return (climberOuterRightMotor.getOutputCurrent() > Constants.MaxCurrent);
  }

  public double outputCurrent() {
    return climberOuterRightMotor.getOutputCurrent();
  }

  public double presentEncoderValue() {
    return climberOuterRightMotor.getEncoder().getPosition();
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    if(abortRaise) output = 0;

    climberOuterRightMotor.set(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    SmartDashboard.putNumber("CROEncoder", climberOuterRightMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("CROCurrent", climberOuterRightMotor.getOutputCurrent());

    return climberOuterRightMotor.getEncoder().getPosition();
  }

  public void runMotor(double speed, boolean run){
    if(run == true)
      climberOuterRightMotor.set(speed);
      SmartDashboard.putNumber("OuterArmsPower", speed);
  }
}
