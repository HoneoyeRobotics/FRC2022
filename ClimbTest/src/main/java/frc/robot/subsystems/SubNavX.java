// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class SubNavX extends SubsystemBase {
  private AHRS navX;
  /** Creates a new SubNavX. */
  public SubNavX() {
    navX = new AHRS(Port.kUSB);
  }

  public void calibrate() {
    navX.calibrate();
  }

  public boolean isCalibrating() {
    return navX.isCalibrating();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("NavX X Pitch", navX.getPitch());
    SmartDashboard.putNumber("NavX Y Roll", navX.getRoll());
    SmartDashboard.putNumber("NavX Z Yaw", navX.getYaw());
  }
}
