// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubInnerArms;
import frc.robot.subsystems.SubLeadScrew;
import frc.robot.subsystems.SubOuterArms;

public class CmdDefaultClimb extends CommandBase {
  private SubLeadScrew leadScrew;
  private DoubleSupplier leadScrewPower; 

  private SubInnerArms innerArms;
  private DoubleSupplier innerAxisDown;
  private DoubleSupplier innerAxisUp;

  private SubOuterArms outerArms;
  private DoubleSupplier outerAxisDown;
  private DoubleSupplier outerAxisUp;
  /** Creates a new CmdDefaultClimb. */
  public CmdDefaultClimb(SubOuterArms outerArms, SubInnerArms innerArms, SubLeadScrew leadScrew, DoubleSupplier initOuterAxisDown, DoubleSupplier initOuterAxisUp, DoubleSupplier initInnerAxisDown, DoubleSupplier initInnerAxisUp, DoubleSupplier initLeadScrewPower) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(outerArms);
    addRequirements(innerArms);
    addRequirements(leadScrew);
    this.outerArms = outerArms;
    this.innerArms = innerArms;
    this.leadScrew = leadScrew;
    outerAxisDown = initOuterAxisDown;
    outerAxisUp = initOuterAxisUp;
    innerAxisDown = initInnerAxisDown;
    innerAxisUp = initInnerAxisUp;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double innerSpeedDown = 0;
    double innerSpeedUp = 0;
    double outerSpeedDown = 0;
    double outerSpeedUp = 0;

    innerSpeedDown = innerAxisDown.getAsDouble();
    innerSpeedUp = innerAxisUp.getAsDouble();
    outerSpeedDown = outerAxisDown.getAsDouble();
    outerSpeedUp = outerAxisUp.getAsDouble();

    innerArms.moveArms(innerSpeedUp - innerSpeedDown);
    outerArms.moveArms(outerSpeedUp - outerSpeedDown);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    innerArms.moveArms(0.0);
    outerArms.moveArms(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
