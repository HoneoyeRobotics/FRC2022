// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.InnerClimber;
import frc.robot.subsystems.OuterClimber;

public class ResetArms extends CommandBase {
  private InnerClimber innerClimber;
  private OuterClimber outerClimber;

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
  public void initialize() {}

  //Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
    double speed = Preferences.getDouble("ResetSpeed", .1);
    double RISpeed = 0;
    double LISpeed = 0;
    double ROSpeed = 0;
    double LOSpeed = 0;
    if(innerClimber.leftAtBottomCurrent() == false) {
      LISpeed = speed;
    }
    else  LISpeed = 0;

    if(innerClimber.rightAtBottomCurrent() == true) {
      RISpeed = speed;
    }
    else RISpeed = 0;

    if(outerClimber.leftAtBottomCurrent() == true) {
      LOSpeed = speed;
    }
    else LOSpeed = 0;

    if(outerClimber.rightAtBottomCurrent() == true) {
      ROSpeed = speed;
    }
    else ROSpeed = 0;
    
    outerClimber.runMotor(ROSpeed, false, true);
    outerClimber.runMotor(LOSpeed, true, false);
    innerClimber.runMotor(RISpeed, false, true);
    innerClimber.runMotor(ROSpeed, true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean temp = false;
    temp = innerClimber.leftAtBottomCurrent() & innerClimber.rightAtBottomCurrent() & outerClimber.leftAtBottomCurrent() & outerClimber.rightAtBottomCurrent();
    return temp;
  }


}
