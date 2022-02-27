// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.CmdRunLeadScrew;

public class SubLeadScrew extends SubsystemBase {

  private TalonSRX leadScrewMotor;
  private DigitalInput atFront;
  private DigitalInput atRear;
  private SubLeadScrew leadScrew;
  private Joystick localJoystick;
  private double localAxis0 = 0;
  private boolean i = false;

  /** Creates a new LeadScrew. */
  public SubLeadScrew() {

    leadScrewMotor = new TalonSRX(Constants.CANLSM);
    atFront = new DigitalInput(0);
    atRear = new DigitalInput(1);
    localJoystick = new Joystick(0);
  }

  public void resetEncoders() {
    leadScrewMotor.setSelectedSensorPosition(0);
  }

  public void moveArms(double speed) {
    double maxLSMCurrent = Preferences.getDouble("MaxLSMCurrent", 30.0);
    double statorCurrent = leadScrewMotor.getStatorCurrent();
    
    if((statorCurrent > maxLSMCurrent) || (statorCurrent < (-1 * maxLSMCurrent))) {
      speed = 0;
    }
    if ((atRear.get() == true && (speed > 0))) {
      speed = 0;
    }
    if((atFront.get() == true && (speed < 0))) {
      speed = 0;
    }
    leadScrewMotor.set(ControlMode.PercentOutput, speed);
    SmartDashboard.putNumber("LeadScrewPower", speed);
  } 

  // private boolean outDeadband(double value, double deadband) {
  //   if (deadband < 0) deadband *= -1;

  //   if (deadband == 0) return false;
  //   if (value > 0) {
  //     if (value > deadband) return true;
  //     }
  //   else {
  //     //value is negative
  //     if (value < (deadband * -1)) return true;
  //   }
  // }

  private boolean outDeadband(double value, double deadband) {
    if (deadband < 0) deadband *= -1;

    if (deadband == 0) i = false;
    else {
      if (value > deadband) i = true;
      else if (value < (deadband * -1)) i = true;
      else i = false;
    }
    return i;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double tempLocalAxis = localJoystick.getRawAxis(0);

    //SmartDashboard.putNumber("LSMEncoder", leadScrewMotor.getSelectedSensorPosition());
    //SmartDashboard.putNumber("LSMCurrent", leadScrewMotor.getStatorCurrent());
    //SmartDashboard.putBoolean("AtFront", atFront.get());
    //SmartDashboard.putBoolean("AtRear", atRear.get());

    if (outDeadband(tempLocalAxis, Constants.deadband)) {
      new CmdRunLeadScrew(leadScrew, 
      () -> localJoystick.getRawAxis(0));

      localAxis0 = tempLocalAxis;
    }
    else if (localAxis0 != 0) {
      new CmdRunLeadScrew(leadScrew,
      () -> 0);

      localAxis0 = 0;
    }
  }
}
