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
    public static final int CANID_DriveLeftFrontMotor = 13;
    public static final int CANID_DriveLeftRearMotor = 14;
    public static final int CANID_DriveRightFrontMotor = 15;
    public static final int CANID_DriveRightRearMotor = 16;
    public static final int CANID_ClimberLeftInnerMotor = 21;
    public static final int CANID_ClimberLeftOuterMotor = 22;
    public static final int CANID_ClimberRightInnerMotor = 23;
    public static final int CANID_ClimberRightOuterMotor = 24;
    public static final int CANID_LeadScrewMotor = 9;
    public static final int CANID_ShooterMotor = 10;
    public static final int CANID_PickupMotor = 11;
    public static final int CANID_FeederMotor = 12;

    public static final double ArmVerticalEncoderMaxValue = 140;
    public static final double ArmVerticalEncoderDeadband = 5;
    public static final double ArmVerticalJoystickEncoderMovement = 10;

    public static final double ArmHorizontalEncoderMaxValue = 200;
    public static final double ArmHorizontalEncoderDeadband = 10;
}
