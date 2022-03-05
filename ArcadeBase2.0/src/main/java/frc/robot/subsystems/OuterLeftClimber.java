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


public class OuterLeftClimber extends PIDSubsystem {
  private boolean abortRaise = false;
  private CANSparkMax climberOuterLeftMotor;
  /** Creates a new OuterLeftClimber. */
  public OuterLeftClimber() {
    super(new PIDController(Constants.kpClimber, 0, 0));
    climberOuterLeftMotor = new CANSparkMax(Constants.CANID_ClimberLeftOuterMotor, MotorType.kBrushless);
    climberOuterLeftMotor.setInverted(false);
    climberOuterLeftMotor.setIdleMode(IdleMode.kBrake);
    resetEncoders();
  }


  public void resetEncoders() {
    climberOuterLeftMotor.getEncoder().setPosition(0);
  }

  public boolean atPosition() {
    if((getSetpoint() <= (getMeasurement() + 1)) && (getSetpoint() >= (getMeasurement() - 1))) 
      return true;
    return false;
  }

  public void setPosition(Constants.ClimberPosition position) {
    double setpoint = 0;
  
    switch(position){
      case start:setpoint = Preferences.getDouble("OuterHooked?", Constants.OuterClimberStart);
        break;
      case top: setpoint = Preferences.getDouble("OuterMax", Constants.OuterClimberTop);
        break;
      case bottom:setpoint = Preferences.getDouble("OuterMin", Constants.OuterClimberBottom);
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
    return (climberOuterLeftMotor.getOutputCurrent() > Constants.MaxRaiseCurrent);
  }

  public boolean atBottomCurrent() {
    return (climberOuterLeftMotor.getOutputCurrent() > Constants.MaxCurrent);
  }

  public double outputCurrent() {
    return climberOuterLeftMotor.getOutputCurrent();
  }

  public double presentEncoderValue() {
    return climberOuterLeftMotor.getEncoder().getPosition();
  }
  
  public double velocity() {
    //returns RPMs of motor
    return climberOuterLeftMotor.getEncoder().getVelocity();
  }

// NOTE: THIS will handle negative arm positions and adjust Max_Differential if needed
// This function takes the current susbystem arm position and compare it to the "Other arm" position that is passed in
// It will evaluate if THIS subsystems arm position is within the MAX differential position of the "Other" arm (compare arm).  
// It returns TRUE if the value of this subsystems arm is out of bounds and FALSE if compare arm is in expected range.
  public boolean Arm_Postion_Too_Low(double Other_Arm_Position, double Max_Differential_Count) {
    // Normalize Max Differential if it is negative
    double md_count = (Max_Differential_Count < 0.0) ? (Max_Differential_Count * -1) : Max_Differential_Count;
    if ( (climberOuterLeftMotor.getEncoder().getPosition() + md_count) < Other_Arm_Position )
      return (true);
    else
      return (false);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    if(abortRaise) output = 0;

    climberOuterLeftMotor.set(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    SmartDashboard.putNumber("CLOEncoder", climberOuterLeftMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("CLOCurrent", climberOuterLeftMotor.getOutputCurrent());

    return climberOuterLeftMotor.getEncoder().getPosition();
  }

  public void runMotor(double speed, boolean run){
    if(run == true)
      climberOuterLeftMotor.set(speed);
      SmartDashboard.putNumber("OuterArmsPower", speed);
  }
}
