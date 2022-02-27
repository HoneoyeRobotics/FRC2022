// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



public class LeadScrew extends SubsystemBase {
  /** Creates a new LeadScrew. */
  
  private TalonSRX leadScrewMotor;
  private DigitalInput frontLimitSwitch;
  private DigitalInput rearLimitSwitch;

  public LeadScrew() { 

    leadScrewMotor = new TalonSRX(Constants.CANID_LeadScrewMotor);
    frontLimitSwitch = new DigitalInput(Constants.DIO_RearLimitSwitch);
    rearLimitSwitch = new DigitalInput(Constants.DIO_FrontLimitSwitch);

  }


  public void resetEncoder() {
    leadScrewMotor.setSelectedSensorPosition(0);
  }

  public boolean armsFullyIn() {
    SmartDashboard.putBoolean("AtRear", rearLimitSwitch.get());
    return rearLimitSwitch.get();
    // double leadScrewEncoder = leadScrewMotor.getSelectedSensorPosition();
    // return (leadScrewEncoder + Constants.ArmHorizontalEncoderDeadband <= 0)
    //     || (leadScrewEncoder - Constants.ArmHorizontalEncoderDeadband <= 0);
  }

  public boolean armsFullyOut() {
    SmartDashboard.putBoolean("AtFront", frontLimitSwitch.get());
    return frontLimitSwitch.get();
    // double leadScrewEncoder = leadScrewMotor.getSelectedSensorPosition();
    // return (Constants.ArmHorizontalEncoderMaxValue <= leadScrewEncoder + Constants.ArmHorizontalEncoderDeadband)
    //     && (Constants.ArmHorizontalEncoderMaxValue >= leadScrewEncoder - Constants.ArmHorizontalEncoderDeadband);
  }

  // public boolean armsCentered() {
  //   double leadScrewEncoder = leadScrewMotor.getSelectedSensorPosition();
  //   return (Constants.ArmHorizantalEncoderCenter <= leadScrewEncoder + Constants.ArmVerticalEncoderDeadband)
  //   && (Constants.ArmHorizantalEncoderCenter >= leadScrewEncoder - Constants.ArmVerticalEncoderDeadband);
  // }
  // public boolean armsGreaterThanCenter() {
  //   double leadScrewEncoder = leadScrewMotor.getSelectedSensorPosition();
  //   return (Constants.ArmHorizantalEncoderCenter > leadScrewEncoder);
  // }

  public void moveArms(double speed) {
    if (speed > 0 && armsFullyOut() || (speed < 0 && armsFullyIn())) {
      speed = 0;
    }
    leadScrewMotor.set(ControlMode.PercentOutput, speed);
    SmartDashboard.putNumber("LeadScrewPower", speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LSMCurrent", leadScrewMotor.getStatorCurrent());
  }
}
