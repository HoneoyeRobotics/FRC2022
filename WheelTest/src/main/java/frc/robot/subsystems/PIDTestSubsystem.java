// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj2.command.PIDSubsystem;
// import frc.robot.Constants;

// public class PIDTestSubsystem extends PIDSubsystem {

//   /** Creates a new PID_Test. */
//   public PIDTestSubsystem() {
//     super(new PIDController(Constants.testMotorkP, Constants.testMotorkI, Constants.testMotorkD));
//     testMotor = new CANSparkMax(1, MotorType.kBrushless);
//     resetEncoders();
//   }

//   public void resetEncoders() {
//     testMotor.getEncoder().setPosition(0);
//   }

//   private CANSparkMax testMotor;

//   public void setPosition(double position) {
//     setSetpoint(position);
//   }

//   @Override
//   public void useOutput(double output, double setpoint) {
//     // Use the output here
//     testMotor.set(output);
//   }

//   double testMotorEncoder = testMotor.getEncoder().getPosition();



//   @Override
//   public double getMeasurement() {
//     // Return the process variable measurement here
//     return testMotorEncoder;
//   }
// }
