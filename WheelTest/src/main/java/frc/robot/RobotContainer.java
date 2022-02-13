// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.TalonSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final TalonSubsystem talonSubsystem = new TalonSubsystem();
  private final SparkSubsystem sparkSubsystem = new SparkSubsystem();
  private final SpikeRelay spikeRelay = new SpikeRelay();

  private final TalonCommand m_autoCommand = new TalonCommand(talonSubsystem);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    CameraServer.startAutomaticCapture();
    // Configure the button bindings

    boolean mySpikeIsRunning = SmartDashboard.getBoolean("RunSpike", false);
    SmartDashboard.putBoolean("RunSpike", mySpikeIsRunning);
  

    double TalonPower = SmartDashboard.getNumber("TalonPower", 0.0);
    SmartDashboard.putNumber("TalonPower", TalonPower);

    double SparkPower = SmartDashboard.getNumber("SparkPower", 0.0);
    SmartDashboard.putNumber("SparkPower", SparkPower);
    configureButtonBindings();

    SmartDashboard.putData(new ResetSparkEncoder(sparkSubsystem));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Joystick driverJoystick = new Joystick(0);

    JoystickButton TalonButton = new JoystickButton(driverJoystick, 1);
    JoystickButton SparkButton = new JoystickButton(driverJoystick, 2);
    JoystickButton SparkButtonReverse = new JoystickButton(driverJoystick, 3);
    JoystickButton RunSpikeButton = new JoystickButton(driverJoystick, 4);

    TalonButton.whileHeld(new TalonCommand(talonSubsystem));
    SparkButton.whileHeld(new SparkCommand(sparkSubsystem));
    RunSpikeButton.whileHeld(new SpikeRelayPower(spikeRelay));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
