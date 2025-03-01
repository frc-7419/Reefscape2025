// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ElevatorConstants;
import frc.robot.constants.Constants.WristConstants;
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

  private boolean checkMovementSafe(double velocity) {
    boolean aboveLimit =
        elevatorSubsystem.getPosition().gt(ElevatorConstants.kElevatorBarUpperLimit);
    boolean belowLimit =
        elevatorSubsystem.getPosition().lt(ElevatorConstants.kElevatorBarLowerLimit);
    boolean wristUnsafe = wristAngleSupplier.get().gt(WristConstants.kElevatorSafeWristAngle);

    if (wristUnsafe && ((belowLimit && velocity > 0) || (aboveLimit && velocity < 0))) {
      return false;
    }

    return true;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double targetVelocity = ElevatorConstants.kMaxVelocity * powerSupplier.getAsDouble();
    // if (!checkMovementSafe(targetVelocity)) targetVelocity = 0;
    // elevatorSubsystem.setVelocity(targetVelocity);
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
