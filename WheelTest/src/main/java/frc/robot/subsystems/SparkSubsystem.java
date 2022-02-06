// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public SparkSubsystem() {}
  private CANSparkMax spark1 = new CANSparkMax(22, MotorType.kBrushless);
  private CANSparkMax spark2 = new CANSparkMax(21, MotorType.kBrushless);
  public void drive(double drivePower){
  spark1.set(drivePower);
  spark2.set(-drivePower);
  
  }

  @Override
  public void periodic() {
    double sparkAvgEncoder = (spark1.getEncoder().getPosition() + spark2.getEncoder().getPosition()) / 2;
    SmartDashboard.putNumber("SparkAvgEncoder", sparkAvgEncoder);
  }
public void resetEncoders(){
  spark1.getEncoder().setPosition(0);
  spark2.getEncoder().setPosition(0);
}
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
