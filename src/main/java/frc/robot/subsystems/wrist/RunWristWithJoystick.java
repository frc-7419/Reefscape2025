// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class RunWristWithJoystick extends Command {
  WristSubsystem wristSubsystem;
  DoubleSupplier powerSupplier;

  public RunWristWithJoystick(WristSubsystem wristSubsystem, DoubleSupplier powerSupplier) {
    this.wristSubsystem = wristSubsystem;
    this.powerSupplier = powerSupplier;
    addRequirements(wristSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    wristSubsystem.setPower(-powerSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    wristSubsystem.setPower(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
