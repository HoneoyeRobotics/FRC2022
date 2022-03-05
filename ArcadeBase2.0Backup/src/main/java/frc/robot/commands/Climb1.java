// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Climb1 extends ParallelCommandGroup {
  /** Creates a new Climber1. */
  public Climb1(InnerLeftClimber innerLeftClimber, InnerRightClimber innerRightClimber, 
                OuterLeftClimber outerLeftClimber, OuterRightClimber outerRightClimber, LeadScrew leadScrew) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
      /*
      A parralel command group to raise both the inner and outer arms 
      as well as move the leadScrew to climb position all at once
      */
    addCommands(
      new RaiseOuterArms(outerRightClimber, outerLeftClimber),
      new RaiseInnerArms(innerRightClimber, innerLeftClimber),
      new MoveLeadScrew(leadScrew, Constants.LeadScrewRaiseSpeed, Constants.LeadScrewRaiseCounter)
      );
  }
}
