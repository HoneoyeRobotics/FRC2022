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

public class InnerLeftClimber extends PIDSubsystem {
  private boolean abortRaise = false;
  private CANSparkMax climberInnerLeftMotor;
  /** Creates a new InnerLeftClimber. */
  public InnerLeftClimber() {
    super(new PIDController(Constants.kpClimber, 0, 0));
    climberInnerLeftMotor = new CANSparkMax(Constants.CANID_ClimberLeftInnerMotor, MotorType.kBrushless);
    climberInnerLeftMotor.setInverted(false);
    climberInnerLeftMotor.setIdleMode(IdleMode.kBrake);
    resetEncoders();
  }

  public void resetEncoders() {
    climberInnerLeftMotor.getEncoder().setPosition(0);
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
    setSetpoint(climberInnerLeftMotor.getEncoder().getPosition());
    abortRaise = false;
  }

  public boolean raiseCurrentBad() {
    return (climberInnerLeftMotor.getOutputCurrent() > Constants.MaxRaiseCurrent);
  }

  public boolean atBottomCurrent() {
    return (climberInnerLeftMotor.getOutputCurrent() > Constants.MaxCurrent);
  }

  public double outputCurrent() {
    return climberInnerLeftMotor.getOutputCurrent();
  }

  public double presentEncoderValue() {
    return climberInnerLeftMotor.getEncoder().getPosition();
  }
  
  public double velocity() {
    //returns RPMs of motor
    return climberInnerLeftMotor.getEncoder().getVelocity();
  }

  // NOTE: THIS will handle negative arm positions and adjust Max_Differential if needed
  // This function takes the current susbystem arm position and compare it to the "Other arm" position that is passed in
  // It will evaluate if THIS subsystems arm position is within the MAX differential position of the "Other" arm (compare arm).  
  // It returns TRUE if the value of this subsystems arm is out of bounds and FALSE if compare arm is in expected range.
    public boolean Arm_Postion_Too_Low(double Other_Arm_Position, double Max_Differential_Count) {
      // Normalize Max Differential if it is negative
      double md_count = (Max_Differential_Count < 0.0) ? (Max_Differential_Count * -1) : Max_Differential_Count;
      if ( (climberInnerLeftMotor.getEncoder().getPosition() + md_count) < Other_Arm_Position )
        return (true);
      else
        return (false);
    }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    if(abortRaise) output = 0;

    climberInnerLeftMotor.set(output);
  }
  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    SmartDashboard.putNumber("CLIEncoder", climberInnerLeftMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("CLICurrent", climberInnerLeftMotor.getOutputCurrent());

    return climberInnerLeftMotor.getEncoder().getPosition();
  }

  public void runMotor(double speed, boolean run){
    if(run == true)
      climberInnerLeftMotor.set(speed);
      SmartDashboard.putNumber("OuterArmsPower", speed);
  }
}
