// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.InnerLeftClimber;
import frc.robot.subsystems.InnerRightClimber;
import frc.robot.subsystems.OuterLeftClimber;
import frc.robot.subsystems.OuterRightClimber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Climb4 extends SequentialCommandGroup {
  /** Creates a new Climb4. */
  public Climb4(InnerLeftClimber innerLeftClimber, InnerRightClimber innerRightClimber, 
                OuterLeftClimber outerLeftClimber, OuterRightClimber outerRightClimber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new LowerOuterArms(outerRightClimber, outerLeftClimber).withTimeout(6),
      new LockSetpoints(innerLeftClimber, innerRightClimber, outerLeftClimber, outerRightClimber)
    );
  }
}
