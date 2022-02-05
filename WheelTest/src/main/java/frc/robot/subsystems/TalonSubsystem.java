// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TalonSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public TalonSubsystem() {}
  public double speed1;
  private TalonSRX talon = new TalonSRX(24);
  public void drive(double drivePower){
    talon.set(ControlMode.PercentOutput, drivePower );
}
public double motorSpeed(){
  
  return (talon.getSelectedSensorVelocity() );
  
}

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
