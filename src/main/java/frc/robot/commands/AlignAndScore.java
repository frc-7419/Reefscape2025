// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ScoringSetpoints.ScoringSetpoint;
import frc.robot.constants.Constants.ScoringConstants.ScoringPosition;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.WristIntakeSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignAndScore extends SequentialCommandGroup {
  /**
   * Creates a new AlignAndScore.
   * 
   * ONLY USE WHEN ALREADY AGAINST REEF WALL OR ELSE ROBOT WILL TIP AND BREAK AND
   * THEY'RE GONNA BLAME SOFTWARE
   */
  public AlignAndScore(CommandSwerveDrivetrain drivetrain, ElevatorSubsystem elevator, WristSubsystem wrist,
      WristIntakeSubsystem wristIntake,
      ScoringPosition scoringPosition, ScoringSetpoint scoringSetpoint) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
            new AlignToReef(drivetrain, scoringPosition, true),
            new ScoringSetpoints(elevator, wrist, scoringSetpoint)),
        new ParallelDeadlineGroup(
            new RunCommand(() -> wristIntake.setPower(0.5), wristIntake)
                .until(() -> !wristIntake.beamBreakisTriggered()),
            wrist.setAngle(Rotations.of(scoringSetpoint.wristAngle))),
        new ScoringSetpoints(elevator, wrist, ScoringSetpoint.HOME));
  }
}
