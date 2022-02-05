// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public SparkSubsystem() {}
  public double speed1;
  private CANSparkMax spark = new CANSparkMax(22, MotorType.kBrushless);
  public void drive(double drivePower){
    spark.set(drivePower);
}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("SparkEncoder", spark.getEncoder().getPosition());
  }
public void resetEncoders(){
  spark.getEncoder().setPosition(0);
}
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
