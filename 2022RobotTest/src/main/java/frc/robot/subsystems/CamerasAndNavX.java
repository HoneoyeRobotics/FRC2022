// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CamerasAndNavX extends SubsystemBase {
  //declare navX
  private AHRS navX;
  //declare cameras
	public UsbCamera frontCamera;
	public UsbCamera rearCamera;
  private VideoSink server;
	public volatile boolean UseFrontCamera = true;
  /** Creates a new CamerasAndNavX. */
  public CamerasAndNavX() {
    navX = new AHRS(Port.kUSB);
    SmartDashboard.putBoolean("Front Camera", UseFrontCamera);
    
    frontCamera = CameraServer.startAutomaticCapture("front", 0);
    frontCamera.setFPS(15);

    rearCamera = new UsbCamera("rear", 1);
    rearCamera.setFPS(15);
    server = CameraServer.getServer();

    frontCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    rearCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
  }

  public void switchCamera(){
    if(UseFrontCamera){
      UseFrontCamera = false;
        server.setSource(rearCamera);
    }
    else{
      UseFrontCamera = true;
      server.setSource(frontCamera);
    }
    SmartDashboard.putBoolean("Front Camera", UseFrontCamera);
  }

  public void calibrate() {
    navX.calibrate();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("NavX X Pitch", navX.getPitch());
    SmartDashboard.putNumber("NavX Y Roll", navX.getRoll());
    SmartDashboard.putNumber("NavX Z Yaw", navX.getYaw());
  }
}
