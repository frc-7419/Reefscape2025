// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.constants.Constants.ScoringConstants.ScoringSetpoint;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeWithBeamBreak;
import frc.robot.subsystems.intake.WristIntakeSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoIntakeCoral extends ParallelDeadlineGroup {
  /** Creates a new AutoIntakeCoral. */
  public AutoIntakeCoral(
      WristIntakeSubsystem wristIntake, ElevatorSubsystem elevator, WristSubsystem wrist) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new IntakeWithBeamBreak(wristIntake));
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ScoringSetpoints(elevator, wrist, ScoringSetpoint.HOME));
  }
}
