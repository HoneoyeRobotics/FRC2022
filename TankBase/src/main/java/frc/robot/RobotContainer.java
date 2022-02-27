// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DefaultLeadScrew;
import frc.robot.commands.DriveTank;
import frc.robot.commands.FeedAndShootBalls;
import frc.robot.commands.FeedBalls;
import frc.robot.commands.PickUpBalls;
import frc.robot.commands.ShootBall;
import frc.robot.subsystems.Ball;
import frc.robot.subsystems.DriveTrainTank;
import frc.robot.subsystems.LeadScrew;
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

  private DriveTrainTank m_DriveTrainTank = new DriveTrainTank();
  Joystick tankDriveJoystickLeft = new Joystick(1);
  Joystick tankDriveJoystickRight = new Joystick(2);

  private LeadScrew m_leadScrew = new LeadScrew();
  Joystick coDriverJoystick = new Joystick(0);

  private Ball m_ball = new Ball();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    m_DriveTrainTank.setDefaultCommand(new DriveTank(m_DriveTrainTank,
    () -> tankDriveJoystickLeft.getRawAxis(1),
    () -> tankDriveJoystickRight.getRawAxis(1)));

    m_leadScrew.setDefaultCommand(new DefaultLeadScrew(m_leadScrew, 
    () -> coDriverJoystick.getRawAxis(4)));

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureCoDriverJoystick() {
      
    JoystickButton buttonA = new JoystickButton(coDriverJoystick, 1);
    JoystickButton buttonB = new JoystickButton(coDriverJoystick, 2);
    JoystickButton buttonX = new JoystickButton(coDriverJoystick, 3);
    JoystickButton buttonY = new JoystickButton(coDriverJoystick, 4);
    JoystickButton leftBumper = new JoystickButton(coDriverJoystick, 5);
    JoystickButton rightBumper = new JoystickButton(coDriverJoystick, 6);

    buttonA.whileHeld(new ShootBall(m_ball)); 
    leftBumper.whileHeld(new FeedBalls(m_ball));
    rightBumper.whileHeld(new PickUpBalls(m_ball));
    buttonB.whenPressed(new FeedAndShootBalls(m_ball));

  }
  private void configureButtonBindings() {
    configureCoDriverJoystick();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new FeedAndShootBalls(m_ball);
  }
}
