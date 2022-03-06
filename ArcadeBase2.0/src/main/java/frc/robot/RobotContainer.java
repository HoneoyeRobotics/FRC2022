// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  public InnerRightClimber innerRightClimber;
  public InnerLeftClimber innerLeftClimber;
  public OuterRightClimber outerRightClimber;
  public OuterLeftClimber outerLeftClimber;
  private LeadScrew leadScrew;
  public static Boolean climbContinue = false;
  public CamerasAndNavX camerasAndNavX;
  // The robot's subsystems and commands are defined here...

  private subDriveTrainArcade m_DriveTrainArcade = new subDriveTrainArcade();
  Joystick driverJoystick = new Joystick(1);
  Joystick coDriverJoystick = new Joystick(0);

  private Ball m_ball = new Ball();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    camerasAndNavX = new CamerasAndNavX();
    innerRightClimber = new InnerRightClimber();
    innerLeftClimber = new InnerLeftClimber();
    outerRightClimber = new OuterRightClimber();
    outerLeftClimber = new OuterLeftClimber();
    leadScrew = new LeadScrew();
    // Configure the button bindings
   
    // Default command for Arcade drivetrain
    m_DriveTrainArcade.setDefaultCommand(new DriveRobotArcade(m_DriveTrainArcade,
    () -> driverJoystick.getRawAxis(Constants.AXIS_RightTrigger),
    () -> driverJoystick.getRawAxis(Constants.AXIS_LeftTrigger),
    () -> driverJoystick.getRawAxis(Constants.AXIS_LeftStickX)));

    Shuffleboard.getTab("Pit").add(new DefaultLeadScrew(leadScrew, 
      () -> coDriverJoystick.getRawAxis(4)));

    Shuffleboard.getTab("Pit").add(new DefaultOuterArms(outerRightClimber, outerLeftClimber,
      () -> coDriverJoystick.getRawAxis(Constants.AXIS_RightStickY) * -1,
      () -> coDriverJoystick.getRawButton(7),
      () -> coDriverJoystick.getRawButton(8)));

    Shuffleboard.getTab("Pit").add(new DefaultInnerArms(innerRightClimber, innerLeftClimber,
      () -> coDriverJoystick.getRawAxis(Constants.AXIS_LeftStickY) * -1,
      () -> coDriverJoystick.getRawButton(7),
      () -> coDriverJoystick.getRawButton(8)));
    
    //SmartDashboard.putData(new ResetEncoders(innerRightClimber, innerLeftClimber, outerRightClimber, outerLeftClimber));
    Shuffleboard.getTab("Pit").add(new RaiseOuterArms(outerRightClimber, outerLeftClimber));
    Shuffleboard.getTab("Pit").add(new LowerOuterArms(outerRightClimber, outerLeftClimber));
    Shuffleboard.getTab("Pit").add(new RaiseInnerArms(innerRightClimber, innerLeftClimber));
    Shuffleboard.getTab("Pit").add(new LowerInnerArms(innerRightClimber, innerLeftClimber));
    Shuffleboard.getTab("Pit").add(new EnablePID(innerLeftClimber, innerRightClimber, outerLeftClimber, outerRightClimber));
    Shuffleboard.getTab("Pit").add(new DisablePID(innerLeftClimber, innerRightClimber, outerLeftClimber, outerRightClimber));
    Shuffleboard.getTab("Pit").add(new ResetArms(innerLeftClimber, innerRightClimber, outerLeftClimber, outerRightClimber));
    
    Shuffleboard.getTab("Pit").add(new Auto1(m_DriveTrainArcade, m_ball));
    //SmartDashboard.putData(new  GetMeasurement(innerRightClimber, innerLeftClimber, outerRightClimber, outerLeftClimber));

    // SmartDashboard.putData(m_ball);
    // SmartDashboard.putData(leadScrew);
    // SmartDashboard.putData(innerLeftClimber);
    // SmartDashboard.putData(innerRightClimber);
    // SmartDashboard.putData(outerLeftClimber);
    
    // SmartDashboard.putData(outerRightClimber);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

    
  private void configureDriverJoystick() {
    // JoystickButton buttonA = new JoystickButton(driverJoystick, 1);
    // JoystickButton buttonX = new JoystickButton(driverJoystick, 3);
    // JoystickButton buttonY = new JoystickButton(driverJoystick, 4);
    JoystickButton buttonB = new JoystickButton(driverJoystick, 2);
    JoystickButton buttonBack = new JoystickButton(driverJoystick, 7);
    JoystickButton buttonStart = new JoystickButton(driverJoystick, 8);

    buttonB.whenPressed(new SwapDriving(m_DriveTrainArcade, camerasAndNavX));
    buttonBack.whenPressed(new UseFrontCamera(camerasAndNavX));
    buttonStart.whenPressed(new UseClimbCamera(camerasAndNavX));
  }

  private void configureCoDriverJoystick() {
      
    JoystickButton buttonA = new JoystickButton(coDriverJoystick, 1);
    JoystickButton buttonB = new JoystickButton(coDriverJoystick, 2);
    JoystickButton buttonX = new JoystickButton(coDriverJoystick, 3);
    JoystickButton buttonY = new JoystickButton(coDriverJoystick, 4);
    JoystickButton leftBumper = new JoystickButton(coDriverJoystick, 5);
    JoystickButton rightBumper = new JoystickButton(coDriverJoystick, 6);
    JoystickButton buttonBack = new JoystickButton(coDriverJoystick, 7);
    JoystickButton buttonStart = new JoystickButton(coDriverJoystick, 8);

    buttonA.whileHeld(new ShootBall(m_ball)); 
    leftBumper.whileHeld(new FeedBalls(m_ball));
    rightBumper.whileHeld(new PickUpBalls(m_ball));
    buttonB.whenPressed(new FeedAndShootBalls(m_ball));
    
    buttonX.whenPressed(new Climb1(innerLeftClimber, innerRightClimber, outerLeftClimber, outerRightClimber, leadScrew));
    buttonY.whenPressed(new Climb2(innerLeftClimber, innerRightClimber, outerLeftClimber, outerRightClimber, leadScrew));

    buttonBack.whenPressed(new Climb3(outerLeftClimber, outerRightClimber, leadScrew));
    buttonStart.whenPressed(new Climb4(innerLeftClimber, innerRightClimber, outerLeftClimber, outerRightClimber));
  }
  private void configureButtonBindings() {
    configureCoDriverJoystick();
    configureDriverJoystick();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new Auto1(m_DriveTrainArcade, m_ball);
  }
}
