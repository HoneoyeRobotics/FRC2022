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
  public double g_xAxis = 0;
  public double g_yAxis = 0;
  public double g_zAxis = 0;

  private double m_xAxisOffset = 0;
  private double m_yAxisOffset = 0;
  private double m_zAxisOffset = 0;
  
	public UsbCamera frontCamera;
	public UsbCamera rearCamera;
  public UsbCamera climbCamera;
  private VideoSink server;
	public volatile boolean UseFrontCamera = true;


  /** Creates a new CamerasAndNavX. */
  public CamerasAndNavX() {
    navX = new AHRS(Port.kUSB);

    normalizeAxis();
    
    frontCamera = CameraServer.startAutomaticCapture("front", 0);
    frontCamera.setFPS(15);
    

    rearCamera = new UsbCamera("rear", 1);
    rearCamera.setFPS(15);

    climbCamera = new UsbCamera("ClimbCamera", 2);
    server = CameraServer.getServer();
    
    frontCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    rearCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    climbCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    navX.calibrate();
  }

  public void calibrate() {
    navX.calibrate();
  }

  public void normalizeAxis() {
    m_xAxisOffset = navX.getPitch();
    m_yAxisOffset = navX.getRoll();
    m_zAxisOffset = navX.getYaw();
  }

  public void updateAxis(){
    g_xAxis = Math.round(navX.getPitch() - m_xAxisOffset);
    g_yAxis = Math.round(navX.getRoll() - m_yAxisOffset);
    g_zAxis = Math.round(navX.getYaw() - m_zAxisOffset);
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
    // updateAxis();
    // SmartDashboard.putNumber("NavX X Pitch", g_xAxis);
    // SmartDashboard.putNumber("NavX Y Roll", g_yAxis);
    // SmartDashboard.putNumber("NavX Z Yaw", g_zAxis);
  }
}
