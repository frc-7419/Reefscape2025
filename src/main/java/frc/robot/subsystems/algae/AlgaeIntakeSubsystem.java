// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.algae;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import com.ctre.phoenix6.hardware.TalonFX;

// public class AlgaeIntakeSubsystem extends SubsystemBase {
//   private final TalonFX motor1;
//   private final TalonFX motor2;
//   /** Creates a new AlgaeIntakeSubsystem. */
//   public AlgaeIntakeSubsystem(TalonFX motor1, TalonFX motor2) {
//     motor1 = new TalonFX(1);
//     motor2 = new TalonFX(2);
//     this.motor1 = motor1;
//     this.motor2 = motor2;
//   }

//   @Override
//   public void periodic() {}
//     // This method will be called once per scheduler run
//     SmartDashboard.putNumber("Motor 1 Claw current speed:", motor1.get());
//     SmartDashboard.putNumber("Motor 2 Claw current speed:", motor2.get());
//   }

//   public void set(double speed) {
//     motor1.set(speed);
//     motor2.set(-speed);
//   }

//   public int[2] getVoltage() {
//     return {motor1.getVoltage(), motor2.getVoltage()}
//   }

//   public int[2] getSpeeds() {
//     return {motor1.getSpeed(), motor2.getSpeed()}
//   }
//   public int[2] getAcceleration() {
//     return {motor1.getAcceleration(), motor2.getAcceleration()}
//   }

//   public void brake() {
//     motor1.setNeutralMode(NeutralModeValue.Brake);
//     motor2.setNeutralMode(NeutralModeValue.Brake);
//   }

//   public void coast() {
//     motor1.setNeutralMode(NeutralModeValue.Coast);
//     motor2.setNeutralMode(NeutralModeValue.Coast);
//   }
// }
