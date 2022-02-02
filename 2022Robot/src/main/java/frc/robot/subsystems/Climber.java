// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private Spark climberLeftInnerMotor;
  private Spark climberLeftOuterMotor;
  private Spark climberRightInnerMotor;
  private Spark climberRightOuterMotor;
  private TalonSRX leadScrewMotor;

  /** Creates a new Climber. */
  public Climber() {
    climberLeftInnerMotor = new Spark(Constants.CANID_ClimberLeftInnerMotor);
    climberLeftOuterMotor = new Spark(Constants.CANID_ClimberLeftOuterMotor);
    climberRightInnerMotor = new Spark(Constants.CANID_ClimberRightInnerMotor);
    climberRightOuterMotor = new Spark(Constants.CANID_ClimberRightOuterMotor);
    leadScrewMotor = new TalonSRX(Constants.CANID_LeadScrewMotor);
  }
  
  public void runInnerArms(double innerArmsSpeed) {
    climberLeftInnerMotor.set(ControlMode.PercentOutput, 0.0);
    climberRightInnerMotor.set(ControlMode.PercentOutput, 0.0);
  }
  public void runOuterArms(double outerArmsSpeed) {
    climberLeftOuterMotor.set(ControlMode.PercentOutput, 0.0);
    climberRightOuterMotor.set(ControlMode.PercentOutput, 0.0);
  }
  public void moveArms(double armMovementSpeed){
    leadScrewMotor.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
