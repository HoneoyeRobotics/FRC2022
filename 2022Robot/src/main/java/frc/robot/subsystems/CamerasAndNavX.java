// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.*;
import edu.wpi.first.cscore.*;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CamerasAndNavX extends SubsystemBase {
  private AHRS navX;

  
	public UsbCamera frontCamera;
	public UsbCamera rearCamera;
  public UsbCamera climbCamera;
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

    climbCamera = new UsbCamera("ClimbCamera", 2);
    server = CameraServer.getServer();
    
    frontCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    rearCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    climbCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);


  }

  public void calibrate() {
    navX.calibrate();
  }

  
  // public void switchCamera(){
  //   if(UseFrontCamera){
  //     UseFrontCamera = false;
  //           server.setSource(rearCamera);
  //   }
  //   else{
  //     UseFrontCamera = true;
  //     server.setSource(frontCamera);
  //   }
  //   SmartDashboard.putBoolean("Front Camera", UseFrontCamera);
  // }

    public void useCamera(CameraSource source) {
      switch (source) {
        case climb:
          server.setSource(climbCamera);
          break;
        case front:
          server.setSource(frontCamera);
          break;
        case rear:
          server.setSource(rearCamera);
          break;
      }
      SmartDashboard.putString("CameraSource", source.toString());
    }

    public enum CameraSource{
      front, rear, climb 
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("NavX X Pitch", navX.getPitch());
    SmartDashboard.putNumber("NavX Y Roll", navX.getRoll());
    SmartDashboard.putNumber("NavX Z Yaw", navX.getYaw());
  }
}
