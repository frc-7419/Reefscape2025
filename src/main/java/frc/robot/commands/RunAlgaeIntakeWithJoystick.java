// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.algaeintake.AlgaeIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunAlgaeIntakeWithJoystick extends Command {
  private final CommandXboxController joystick;
  private final AlgaeIntakeSubsystem algaeIntakeSubsystem;

  public RunAlgaeIntakeWithJoystick(
      CommandXboxController joystick, AlgaeIntakeSubsystem algaeIntakeSubsystem) {
    this.joystick = joystick;
    this.algaeIntakeSubsystem = algaeIntakeSubsystem;
    addRequirements(algaeIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    algaeIntakeSubsystem.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaeIntakeSubsystem.setPower(joystick.getLeftX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeIntakeSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
