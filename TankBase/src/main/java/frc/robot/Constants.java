// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int CANID_DriveLeftFrontMotor = 15;
    public static final int CANID_DriveLeftRearMotor = 16;
    public static final int CANID_DriveRightFrontMotor = 13;
    public static final int CANID_DriveRightRearMotor = 14;
    public static final int CANID_ClimberLeftInnerMotor = 24;
    public static final int CANID_ClimberLeftOuterMotor = 26;
    public static final int CANID_ClimberRightInnerMotor = 23;
    public static final int CANID_ClimberRightOuterMotor = 25;
    public static final int CANID_LeadScrewMotor = 9;
    public static final int CANID_ShooterMotor = 10;
    public static final int CANID_Shooter1Motor = 12;
    public static final int CANID_PickupMotor = 11;
    public static final int CANID_FeederMotor = 8;

    public static final int DIO_FrontLimitSwitch = 0;
    public static final int DIO_RearLimitSwitch = 1;

    public static final int AXIS_LeftStickX  = 0;
    public static final int AXIS_LeftStickY = 1;
    public static final int AXIS_LeftTrigger = 2;    
    public static final int AXIS_RightTrigger = 3;
    public static final int AXIS_RightStickX = 4;
    public static final int AXIS_RightStickY =  5;

    public static final double MaxClimberOutput = 0.75;
    public static final double ArmVerticalEncoderDeadband = 0;
    public static final double ArmVerticalJoystickEncoderMovement = 1;

    public static final double ArmHorizontalEncoderMaxValue = 200;
    public static final double ArmHorizantalEncoderCenter = 100;
    public static final double ArmHorizontalEncoderDeadband = 10;

    public static final double OuterLeftMax = 130;
    public static final double InnerLeftMax = 108;
    public static final double InnerRightMax = 109;
    public static final double OuterRightMax = 137;

    public static final double JoystickDeadband = .08;

    public static final double MaxCurrent = 10;
    public static final double MaxRaiseCurrent = 15;

    public static final int CounterValue = 50;
}
