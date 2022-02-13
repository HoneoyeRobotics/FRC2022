// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.*;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DriveRobot;

public class DriveTrain extends SubsystemBase {
  // declare motors for drivetrain
  private WPI_TalonSRX driveLeftFrontMotor;
  private WPI_VictorSPX driveLeftRearMotor;
  private WPI_TalonSRX driveRightFrontMotor;
  private WPI_VictorSPX driveRightRearMotor;
  private MotorControllerGroup driveRightMotorGroup;
  private MotorControllerGroup driveLeftMotorGroup;

  private DifferentialDrive driveMotors;



	public UsbCamera frontCamera;

	public UsbCamera rearCamera;
// private MjpegServer frontCameraServer;
//private MjpegServer cameraServer;
private VideoSink server;
	// public CvSink frontCameraCvSink;

	// public CvSink rearCameraCvSink;
//public CvSource cameraSource;
// public CvSink cameraSink;
	public volatile boolean UseFrontCamera = true;
  //private NetworkTableEntry cameraSelection;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    driveLeftFrontMotor = new WPI_TalonSRX(Constants.CANID_DriveLeftFrontMotor);
    driveLeftRearMotor = new WPI_VictorSPX(Constants.CANID_DriveLeftRearMotor);

    driveRightFrontMotor = new WPI_TalonSRX(Constants.CANID_DriveRightFrontMotor);
    driveRightRearMotor = new WPI_VictorSPX(Constants.CANID_DriveRightRearMotor);

    driveLeftMotorGroup = new MotorControllerGroup(driveLeftFrontMotor, driveLeftRearMotor);
    driveRightMotorGroup = new MotorControllerGroup(driveRightFrontMotor, driveRightRearMotor);

    driveMotors = new DifferentialDrive(driveLeftMotorGroup, driveRightMotorGroup);


    SmartDashboard.putBoolean("Front Camera", UseFrontCamera);
    
    frontCamera = CameraServer.startAutomaticCapture("front", 0);
    frontCamera.setFPS(30);

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

  public void drive(double xSpeed, double zRotation) {
    driveMotors.arcadeDrive(xSpeed, zRotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
