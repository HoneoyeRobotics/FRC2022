// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.InnerClimber;
import frc.robot.subsystems.OuterClimber;

public class ResetArms extends CommandBase {
  double RISpeed = 0;
  double LISpeed = 0;
  double ROSpeed = 0;
  double LOSpeed = 0;
  private InnerClimber innerClimber;
  private OuterClimber outerClimber;
  private static int counter = 0;

  /** Creates a new ResetArms. */
  public ResetArms(InnerClimber innerClimber, OuterClimber outerClimber) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(innerClimber);
    addRequirements(outerClimber);
    this.innerClimber = innerClimber;
    this.outerClimber = outerClimber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double speed = Preferences.getDouble("ResetSpeed", .1);
    RISpeed = speed;
    LISpeed = speed;
    ROSpeed = speed;
    LOSpeed = speed;
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    

    if (counter >= 10) {
      if (innerClimber.leftAtBottomCurrent()) {
        LISpeed = 0;
      } 

      if (innerClimber.rightAtBottomCurrent()) {
        RISpeed = 0;
      } 

      if (outerClimber.leftAtBottomCurrent()) {
        LOSpeed = 0;
      } 

      if (outerClimber.rightAtBottomCurrent()) {
        ROSpeed = 0;
      } 
    }
    outerClimber.runMotor(ROSpeed, false, true);
    outerClimber.runMotor(LOSpeed, true, false);
    innerClimber.runMotor(RISpeed, false, true);
    innerClimber.runMotor(LISpeed, true, false);

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
