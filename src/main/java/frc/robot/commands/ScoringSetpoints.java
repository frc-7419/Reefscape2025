// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants.ElevatorConstants;
import frc.robot.constants.Constants.ScoringConstants.ScoringSetpoint;
import frc.robot.subsystems.elevator.ElevatorMM;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.subsystems.wrist.WristToPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoringSetpoints extends SequentialCommandGroup {
  /** Creates a new ScoringSetpoint. */
  public ScoringSetpoints(ElevatorSubsystem elevator, WristSubsystem wrist, ScoringSetpoint targetPosition) {
    
    double elevatorRotations = elevator.getPosition().in(Rotations);
    Angle upAngle = targetPosition.name.equals("BARGE") ? Rotations.of(0) : Rotations.of(0.46);

    addCommands(Commands.parallel(
                        new ElevatorMM(elevator, ElevatorConstants.kElevatorBarLowerLimit),
                        new WristToPosition(wrist, upAngle)),
                Commands.parallel(
                        new ElevatorMM(elevator, Rotations.of(targetPosition.elevatorHeight)),
                        new WristToPosition(wrist, Rotations.of(targetPosition.wristAngle))));
  }
}
