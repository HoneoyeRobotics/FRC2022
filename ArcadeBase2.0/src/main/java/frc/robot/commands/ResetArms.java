// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.InnerRightClimber;
import frc.robot.subsystems.*;

public class ResetArms extends CommandBase {
  double RISpeed = 0;
  double LISpeed = 0;
  double ROSpeed = 0;
  double LOSpeed = 0;
  private InnerLeftClimber innerLeftClimber;
  private OuterRightClimber outerRightClimber;
  private OuterLeftClimber outerLeftClimber;
  private InnerRightClimber innerRightClimber;
  private static int counter = 0;

  /** Creates a new ResetArms. */
  public ResetArms(InnerLeftClimber innerLeftClimber, InnerRightClimber innerRightClimber, 
                   OuterLeftClimber outerLeftClimber, OuterRightClimber outerRightClimber) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(innerRightClimber);
    addRequirements(innerLeftClimber);
    addRequirements(outerRightClimber);
    addRequirements(outerLeftClimber);

    this.innerRightClimber = innerRightClimber;
    this.innerLeftClimber = innerLeftClimber;
    this.outerRightClimber = outerRightClimber;
    this.outerLeftClimber = outerLeftClimber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double speed = Preferences.getDouble("ResetSpeed", -.1);
    RISpeed = speed;
    LISpeed = speed;
    ROSpeed = speed;
    LOSpeed = speed;
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    innerLeftClimber.getMeasurement();
    innerRightClimber.getMeasurement();
    outerLeftClimber.getMeasurement();
    outerRightClimber.getMeasurement();

    if (counter >= 10) {
      if (innerRightClimber.atBottomCurrent()) {
        LISpeed = 0;
      } 

      if (innerLeftClimber.atBottomCurrent()) {
        RISpeed = 0;
      } 

      if (outerRightClimber.atBottomCurrent()) {
        LOSpeed = 0;
      } 

      if (outerLeftClimber.atBottomCurrent()) {
        ROSpeed = 0;
      } 
    }
    outerRightClimber.runMotor(ROSpeed, true);
    outerLeftClimber.runMotor(LOSpeed, true);
    innerRightClimber.runMotor(RISpeed, true);
    innerLeftClimber.runMotor(LISpeed, true);

    // counter is increased by 1 every time execute is called i.e. every 20ms
    counter += counter;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // counter is reset to 0 so the next time command is called it will be 0
    counter = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (RISpeed == 0 && LISpeed == 0 && ROSpeed == 0 && LOSpeed == 0);
    // once counter reaches 10 and temp is true, meaning all the arms have reached
    // the bottom, the command will end
  }

}
