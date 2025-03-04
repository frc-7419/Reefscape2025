// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class RunElevatorWithJoystick extends Command {
  ElevatorSubsystem elevatorSubsystem;
  DoubleSupplier powerSupplier;
  Supplier<Angle> wristAngleSupplier;

  public RunElevatorWithJoystick(
      ElevatorSubsystem elevatorSubsystem,
      DoubleSupplier powerSupplier,
      Supplier<Angle> wristAngleSupplier) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.powerSupplier = powerSupplier;
    this.wristAngleSupplier = wristAngleSupplier;
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    elevatorSubsystem.setPower(powerSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
