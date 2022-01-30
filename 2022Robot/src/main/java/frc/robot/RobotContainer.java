// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Ball;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //declare subsystems
  private DriveTrain driveTrain;
  private Climber climber;
  private Ball ball;
  Joystick driverJoystick = new Joystick(0);
  Joystick coDriverJoystick = new Joystick(1);
  
  private SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    //initialize subsystems
    driveTrain = new DriveTrain();
    climber = new Climber();
    ball = new Ball();

    
    //Default commands
      DriveRobot driveRobot = new DriveRobot (driveTrain,
      () -> driverJoystick.getRawAxis(3),
      () -> driverJoystick.getRawAxis(2),
      () -> driverJoystick.getRawAxis(0));
      driveTrain.setDefaultCommand(driveRobot);

      DefaultClimb defaultClimb = new DefaultClimb (climber,
      () -> coDriverJoystick.getRawAxis(5),
      () -> coDriverJoystick.getRawAxis(1),
      () -> coDriverJoystick.getRawAxis(0));
      climber.setDefaultCommand(defaultClimb);

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

      JoystickButton buttonA = new JoystickButton(driverJoystick, 1);
      JoystickButton buttonB = new JoystickButton(driverJoystick, 2);
      JoystickButton buttonX = new JoystickButton(driverJoystick, 3);
      JoystickButton buttonY = new JoystickButton(driverJoystick, 4);

      



    }

    private void configureCoDriverJoystick() {
      

      JoystickButton buttonA = new JoystickButton(coDriverJoystick, 1);
      JoystickButton buttonB = new JoystickButton(coDriverJoystick, 2);
      JoystickButton buttonX = new JoystickButton(coDriverJoystick, 3);
      JoystickButton buttonY = new JoystickButton(coDriverJoystick, 4);
      JoystickButton leftBumper = new JoystickButton(coDriverJoystick, 5);
      JoystickButton rightBumper = new JoystickButton(coDriverJoystick, 6);

      buttonA.whileHeld(new ShootBall(ball)); 
      leftBumper.whileHeld(new FeedBalls(ball));
      rightBumper.whileHeld(new PickUpBalls(ball));

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
    return m_chooser.getSelected();
  }
}