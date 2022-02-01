// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DriveRobot;

public class DriveTrain extends SubsystemBase {
  //declare motors for drivetrain
  private WPI_TalonSRX leftFrontDriveMotor;
  private WPI_VictorSPX leftRearDriveMotor;
  private WPI_TalonSRX rightFrontDriveMotor;
  private WPI_VictorSPX rightRearDriveMotor;
  private MotorControllerGroup rightDriveMotorGroup;
  private MotorControllerGroup leftDriveMotorGroup;

  private DifferentialDrive driveMotors;

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftFrontDriveMotor = new WPI_TalonSRX(Constants.CANID_LeftFrontDriveMotor);    
    leftRearDriveMotor = new WPI_VictorSPX(Constants.CANID_LeftRearDriveMotor);
    
    rightFrontDriveMotor = new WPI_TalonSRX(Constants.CANID_RightFrontDriveMotor);    
    rightRearDriveMotor = new WPI_VictorSPX(Constants.CANID_RightRearDriveMotor);

    leftDriveMotorGroup = new MotorControllerGroup(leftFrontDriveMotor, leftRearDriveMotor);
    rightDriveMotorGroup = new MotorControllerGroup(rightFrontDriveMotor, rightRearDriveMotor);

    driveMotors = new DifferentialDrive(leftDriveMotorGroup, rightDriveMotorGroup);
  }


  public void drive(double xSpeed, double zRotation){
    driveMotors.arcadeDrive(xSpeed, zRotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
