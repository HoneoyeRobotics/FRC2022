// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private InnerRightClimber innerRightClimber;
  private InnerLeftClimber innerLeftClimber;
  private OuterRightClimber outerRightClimber;
  private OuterLeftClimber outerLeftClimber;
  private Joystick driverJoystick = new Joystick(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    innerRightClimber = new InnerRightClimber();
    innerLeftClimber = new InnerLeftClimber();
    outerRightClimber = new OuterRightClimber();
    outerLeftClimber = new OuterLeftClimber();


    // Configure the button bindings
    configureButtonBindings();
    SmartDashboard.putNumber("Topo", 1);
    SmartDashboard.putData(new ResetEncoders(innerRightClimber, innerLeftClimber, outerRightClimber, outerLeftClimber));
    
    SmartDashboard.putData(new RaiseOuterArms(outerRightClimber, outerLeftClimber));
    SmartDashboard.putData(new LowerOuterArms(outerRightClimber, outerLeftClimber));
    SmartDashboard.putData(new RaiseInnerArms(innerRightClimber, innerLeftClimber));
    SmartDashboard.putData(new LowerInnerArms(innerRightClimber, innerLeftClimber));
    SmartDashboard.putData(new EnablePID(innerLeftClimber, innerRightClimber, outerLeftClimber, outerRightClimber));
    SmartDashboard.putData(new DisablePID(innerLeftClimber, innerRightClimber, outerLeftClimber, outerRightClimber));

    SmartDashboard.putNumber("Bot", 0);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureDriverJoystick() {
    JoystickButton aButton = new JoystickButton(driverJoystick, 1);

    aButton.whileHeld(new ResetEncoders(innerRightClimber, innerLeftClimber, outerRightClimber, outerLeftClimber));

  }

  private void configureButtonBindings() {
    configureDriverJoystick();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   return ;
  // }
}
