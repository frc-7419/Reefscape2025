// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.intake.groundintake.GroundIntakeSubsystem;
import frc.robot.subsystems.intake.groundintake.GroundIntakeWristSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunGroundIntakeWithJoystick extends Command {
  /** Creates a new RunGroundIntakeWithJoystick. */
  private final CommandXboxController joystick;

  private final GroundIntakeWristSubsystem groundWristSubsystem;
  private final GroundIntakeSubsystem groundIntakeSubsystem;

  public RunGroundIntakeWithJoystick(CommandXboxController operator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.joystick = operator;
    this.groundWristSubsystem = new GroundIntakeWristSubsystem();
    this.groundIntakeSubsystem = new GroundIntakeSubsystem();
    addRequirements(groundWristSubsystem, groundIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    groundWristSubsystem.coast();
    groundIntakeSubsystem.coast();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    groundWristSubsystem.joystickControl(joystick.getLeftY());
    if (joystick.getLeftTriggerAxis() > 0.3) {
      groundIntakeSubsystem.setSpeed(0.5);
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
