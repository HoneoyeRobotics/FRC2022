// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Climb2 extends SequentialCommandGroup {
  /** Creates a new Climb2. */
  public Climb2(InnerLeftClimber innerLeftClimber, InnerRightClimber innerRightClimber, 
                OuterLeftClimber outerLeftClimber, OuterRightClimber outerRightClimber, LeadScrew leadScrew) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
      /*
      A Sequential command group to first lower the outer arms then immediatly move the leadscrew to the back of the robot
      Second it waits 2 seconds for robot to stabilize before lowering inner arms  for the final step of the climb
      */
    addCommands(
      new EnablePID(innerLeftClimber, innerRightClimber, outerLeftClimber, outerRightClimber),
      new LowerOuterArms(outerRightClimber, outerLeftClimber),
      new MoveLeadScrewToFront(leadScrew, Constants.LeadScrewLowerSpeed).withTimeout(2),      
      new WaitCommand(2),
      new LowerInnerArms(innerRightClimber, innerLeftClimber)
    );
  }
}
