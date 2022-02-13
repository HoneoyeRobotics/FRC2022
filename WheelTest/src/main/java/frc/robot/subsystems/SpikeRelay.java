// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpikeRelay extends SubsystemBase {

  private Relay mySpike;
  /** Creates a new SpikeRelay. */
  public SpikeRelay() {
    mySpike = new Relay(0);
    
  }

    public void runMySpike() {
      mySpike.set(Value.kForward);
    }
    
    public void stopMySpike() {
      mySpike.set(Value.kOff);
    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
