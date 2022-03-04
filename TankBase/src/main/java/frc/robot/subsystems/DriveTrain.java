// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  private WPI_VictorSPX driveLeftFrontMotor;
  private WPI_TalonSRX driveLeftRearMotor;
  private WPI_VictorSPX driveRightFrontMotor;
  private WPI_TalonSRX driveRightRearMotor;
  private MotorControllerGroup driveLeftMotorGroup;
  private MotorControllerGroup driveRightMotorGroup;
  private DifferentialDrive driveMotors;
  /** Creates a new DriveTrainTank. */
  public DriveTrain() {
    driveLeftFrontMotor = new WPI_VictorSPX(Constants.CANID_DriveLeftFrontMotor);
    driveLeftRearMotor = new WPI_TalonSRX(Constants.CANID_DriveLeftRearMotor);
    driveRightFrontMotor = new WPI_VictorSPX(Constants.CANID_DriveRightFrontMotor);
    driveRightRearMotor = new WPI_TalonSRX(Constants.CANID_DriveRightRearMotor);

    driveLeftMotorGroup = new MotorControllerGroup(driveLeftFrontMotor, driveLeftRearMotor);
    driveRightMotorGroup = new MotorControllerGroup(driveRightFrontMotor, driveRightRearMotor);

    driveRightMotorGroup.setInverted(true);

    driveMotors = new DifferentialDrive(driveLeftMotorGroup, driveRightMotorGroup);
  }

  public void drive(double xSpeed, double zRotation) {
    driveMotors.arcadeDrive( xSpeed, zRotation);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
