// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.CmdDefaultClimb;
import frc.robot.commands.CmdMoveInnerArm;
import frc.robot.commands.CmdMoveOuterArm;
import frc.robot.commands.CmdRunFeeder;
import frc.robot.commands.CmdRunLeadScrew;
import frc.robot.commands.CmdRunPickup;
import frc.robot.commands.CmdRunShooter;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ResetEncoder;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SubFeeder;
import frc.robot.subsystems.SubInnerArms;
import frc.robot.subsystems.SubLeadScrew;
import frc.robot.subsystems.SubOuterArms;
import frc.robot.subsystems.SubPickup;
import frc.robot.subsystems.SubShooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  Joystick driverJoystick = new Joystick(0);
  Joystick coDriverJoystick = new Joystick(1);

  DoubleSupplier junk1;

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SubOuterArms m_subOuterArms = new SubOuterArms();
  private final SubInnerArms m_subInnerArms = new SubInnerArms();
  private final SubLeadScrew m_subLeadScrew = new SubLeadScrew();
  private final SubFeeder m_subFeeder = new SubFeeder();
  private final SubShooter m_subShooter = new SubShooter();
  private final SubPickup m_subPickup = new SubPickup();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  private final CmdDefaultClimb m_cmdDefaultClimb = new CmdDefaultClimb(m_subOuterArms, m_subInnerArms, m_subLeadScrew, null, null, null, null, null);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
      
    
SmartDashboard.putData(new ResetEncoder(m_subOuterArms, m_subInnerArms, m_subLeadScrew));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureDriverJoystick() {
    JoystickButton startButton = new JoystickButton(driverJoystick, 8);
    JoystickButton backButton = new JoystickButton(driverJoystick, 7);
    JoystickButton rBumperButton = new JoystickButton(driverJoystick, 6);
    JoystickButton lBumperButton = new JoystickButton(driverJoystick, 5);


    JoystickButton aButton = new JoystickButton(driverJoystick, 1);
    JoystickButton bButton = new JoystickButton(driverJoystick, 2);
    JoystickButton xButton = new JoystickButton(driverJoystick, 3);
    JoystickButton yButton = new JoystickButton(driverJoystick, 4);
    
    aButton.whileHeld(new CmdMoveOuterArm(m_subOuterArms, true, -0.1));
    yButton.whileHeld(new CmdMoveOuterArm(m_subOuterArms, true, 0.1));
    xButton.whileHeld(new CmdMoveOuterArm(m_subOuterArms, false, -0.1));
    bButton.whileHeld(new CmdMoveOuterArm(m_subOuterArms, false, 0.1));
    
    rBumperButton.whileHeld(new CmdRunFeeder(m_subFeeder));
    lBumperButton.whileHeld(new CmdRunShooter(m_subShooter));
    backButton.whileHeld(new CmdRunPickup(m_subPickup));
    startButton.whenPressed(new ResetEncoder(m_subOuterArms, m_subInnerArms, m_subLeadScrew));
  }

  private void configureCoDriverJoystick() {
    JoystickButton aButton = new JoystickButton(coDriverJoystick, 1);
    JoystickButton bButton = new JoystickButton(coDriverJoystick, 2);
    JoystickButton xButton = new JoystickButton(coDriverJoystick, 3);
    JoystickButton yButton = new JoystickButton(coDriverJoystick, 4);
    JoystickButton rBumperButton = new JoystickButton(coDriverJoystick, 6);
    JoystickButton lBumperButton = new JoystickButton(coDriverJoystick, 5);

    aButton.whileHeld(new CmdMoveInnerArm(m_subInnerArms, true, -0.1));
    yButton.whileHeld(new CmdMoveInnerArm(m_subInnerArms, true, 0.1));
    xButton.whileHeld(new CmdMoveInnerArm(m_subInnerArms, false, -0.1));
    bButton.whileHeld(new CmdMoveInnerArm(m_subInnerArms, false, 0.1));
  
    rBumperButton.whenPressed(new CmdDefaultClimb(m_subOuterArms, m_subInnerArms, m_subLeadScrew,
    () -> driverJoystick.getRawAxis(2),
    () -> driverJoystick.getRawAxis(3),
    () -> coDriverJoystick.getRawAxis(2),
    () -> coDriverJoystick.getRawAxis(3),
    () -> driverJoystick.getRawAxis(5))
    );

    lBumperButton.whenPressed(new CmdRunLeadScrew(m_subLeadScrew,
    () -> coDriverJoystick.getRawAxis(1))
    );

  }
   
  private void configureButtonBindings() {
    configureDriverJoystick();
    configureCoDriverJoystick();
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
