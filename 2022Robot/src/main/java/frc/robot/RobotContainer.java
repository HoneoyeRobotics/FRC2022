// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //declare subsystems
  private DriveTrain driveTrain;
  private OuterClimber outerClimber;
  private InnerClimber innerClimber;
  private LeadScrew leadScrew;
  private Ball ball;
  private CamerasAndNavX camerasAndNavX;
  Joystick driverJoystick = new Joystick(0);
  Joystick coDriverJoystick = new Joystick(1);
  private PowerDistribution pdp;
  private SendableChooser<Command> m_chooser = new SendableChooser<>();
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //initialize subsystems
    driveTrain = new DriveTrain();
    outerClimber = new OuterClimber();
    innerClimber = new InnerClimber();
    leadScrew = new LeadScrew();
    ball = new Ball();
    camerasAndNavX = new CamerasAndNavX();
    pdp = new PowerDistribution(0, ModuleType.kCTRE);

    Shuffleboard.getTab("Diagnostics").add("PDP", pdp).withWidget(BuiltInWidgets.kPowerDistribution).withPosition(0, 0).withSize(6,    3);

    //Default commands
      driveTrain.setDefaultCommand(new DriveRobot (driveTrain,
        () -> driverJoystick.getRawAxis(Constants.AXIS_RightTrigger),
        () -> driverJoystick.getRawAxis(Constants.AXIS_LeftTrigger),
        () -> driverJoystick.getRawAxis(Constants.AXIS_LeftStickX)));

      outerClimber.setDefaultCommand(new DefaultOuterArms(outerClimber, 
          () -> coDriverJoystick.getRawAxis(Constants.AXIS_RightStickY) * -1,
          () -> coDriverJoystick.getRawButton(7),
          () -> coDriverJoystick.getRawButton(8)));

      innerClimber.setDefaultCommand(new DefaultInnerArms(innerClimber, 
          () -> coDriverJoystick.getRawAxis(Constants.AXIS_LeftStickY) * -1,
          () -> coDriverJoystick.getRawButton(7),
          () -> coDriverJoystick.getRawButton(8)));

      leadScrew.setDefaultCommand(new DefaultLeadScrew(leadScrew, 
          () -> coDriverJoystick.getRawAxis(Constants.AXIS_LeftStickX)));
      
    // Configure the button bindings
    configureButtonBindings();
    
    SmartDashboard.putData(new UseFrontCamera(camerasAndNavX));
    SmartDashboard.putData(new RaiseOuterArms(outerClimber));
    SmartDashboard.putData(new LowerOuterArms(outerClimber));
    SmartDashboard.putData(new RaiseInnerArms(innerClimber));
    SmartDashboard.putData(new LowerInnerArms(innerClimber));
    SmartDashboard.putData(new EnablePID(innerClimber, outerClimber));
    SmartDashboard.putData(new DisablePID(innerClimber, outerClimber));
    SmartDashboard.putData(new FinalClimb(outerClimber, innerClimber, leadScrew));
    SmartDashboard.putData(new ResetArms(innerClimber, outerClimber));
    SmartDashboard.putData(new ResetEncoder(outerClimber, innerClimber));
    SmartDashboard.putData(new UseFrontCamera(camerasAndNavX));
    SmartDashboard.putData(new UseRearCamera(camerasAndNavX));
    SmartDashboard.putData(new UseClimbCamera(camerasAndNavX));

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
      //JoystickButton buttonX = new JoystickButton(driverJoystick, 3);
      //JoystickButton buttonY = new JoystickButton(driverJoystick, 4);
      JoystickButton leftBumper = new JoystickButton(driverJoystick, 5);
      JoystickButton rightBumper = new JoystickButton(driverJoystick, 6);
      
      // buttonA.whileHeld(new ShootBall(ball)); 
      // leftBumper.whileHeld(new FeedBalls(ball));
      // rightBumper.whileHeld(new PickUpBalls(ball));
      // buttonB.whenPressed(new FeedAndShootBalls(ball));
    }

//     private void stephenDriverJoystick() {

//       JoystickButton buttonA = new JoystickButton(driverJoystick, 1);
//       JoystickButton buttonB = new JoystickButton(driverJoystick, 2);
//       JoystickButton buttonX = new JoystickButton(driverJoystick, 3);
//       JoystickButton buttonY = new JoystickButton(driverJoystick, 4);

//       JoystickButton leftBumper = new JoystickButton(driverJoystick, 5);
//       JoystickButton rightBumper = new JoystickButton(driverJoystick, 6);
      
//       buttonA.whileHeld(new ShootBall(ball)); 
//       buttonY.whileHeld(new FeedBalls(ball));
//       buttonB.whileHeld(new PickUpBalls(ball));

// buttonB.whenPressed(new FeedAndShootBalls(ball));
//     }

    private void configureCoDriverJoystick() {
      
      JoystickButton buttonA = new JoystickButton(coDriverJoystick, 1);
      JoystickButton buttonB = new JoystickButton(coDriverJoystick, 2);
      JoystickButton buttonX = new JoystickButton(coDriverJoystick, 3);
      JoystickButton buttonY = new JoystickButton(coDriverJoystick, 4);
      JoystickButton leftBumper = new JoystickButton(coDriverJoystick, 5);
      JoystickButton rightBumper = new JoystickButton(coDriverJoystick, 6);
      int povAngle = 0;
      int povNumber = 0;
      POVButton POV = new POVButton(coDriverJoystick, povAngle, povNumber);
      

      // buttonX.whenPressed(new RaiseInnerArms(innerClimber));
      // buttonA.whenPressed(new LowerInnerArms(innerClimber));
      // buttonY.whenPressed(new RaiseOuterArms(outerClimber));
      // buttonB.whenPressed(new LowerOuterArms(outerClimber));
      // leftBumper.whileHeld(new MoveArmsForward(leadScrew));
      // rightBumper.whileHeld(new MoveArmsBackward(leadScrew));
      buttonA.whileHeld(new ShootBall(ball)); 
      leftBumper.whileHeld(new FeedBalls(ball));
      rightBumper.whileHeld(new PickUpBalls(ball));
      buttonB.whenPressed(new FeedAndShootBalls(ball));
      POV.whenPressed(new PovCommand(
        () -> coDriverJoystick.getPOV()));
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