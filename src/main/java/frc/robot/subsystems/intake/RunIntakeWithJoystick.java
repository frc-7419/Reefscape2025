// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunIntakeWithJoystick extends Command {
  private WristIntakeSubsystem wristIntakeSubsystem;
  private CommandXboxController commandXboxController;
  /** Creates a new RunIntakeWithJoystick. */
  public RunIntakeWithJoystick(
      WristIntakeSubsystem wristIntakeSubsystem, CommandXboxController commandXboxController) {
    this.wristIntakeSubsystem = wristIntakeSubsystem;
    this.commandXboxController = commandXboxController;
    addRequirements(wristIntakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wristIntakeSubsystem.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (commandXboxController.getRightTriggerAxis() >= 0.05) {
      wristIntakeSubsystem.setPower(Math.abs(commandXboxController.getRightTriggerAxis()) * 0.7);
    } else if (Math.abs(commandXboxController.getLeftTriggerAxis()) >= 0.05) {
      wristIntakeSubsystem.setPower(-commandXboxController.getLeftTriggerAxis() * 0.7);
    } else {
      wristIntakeSubsystem.setPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wristIntakeSubsystem.setPower(0);
    wristIntakeSubsystem.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
