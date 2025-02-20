// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.TunableValue;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristPIDTest extends Command {
  private final WristSubsystem wrist;
  private final ProfiledPIDController pidController;
  private final TunableValue kP = new TunableValue("Wrist kP", 0.6);
  private final TunableValue kI = new TunableValue("Wrist kI", 0.0);
  private final TunableValue kD = new TunableValue("Wrist kD", 0.0);
  private final TunableValue setpoint = new TunableValue("Wrist Setpoint", 6);

  /** Creates a new WristPIDTest. */
  public WristPIDTest(WristSubsystem wrist) {
    this.wrist = wrist;
    pidController =
        new ProfiledPIDController(
            kP.getValue(), kI.getValue(), kD.getValue(), new TrapezoidProfile.Constraints(5, 0.5));
    addRequirements(wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (kP.getValue() != pidController.getP()) {
      pidController.setP(kP.getValue());
    }
    if (kI.getValue() != pidController.getI()) {
      pidController.setI(kI.getValue());
    }
    if (kD.getValue() != pidController.getD()) {
      pidController.setD(kD.getValue());
    }
    if (pidController.getGoal().position != setpoint.getValue()) {
      pidController.setGoal(setpoint.getValue());
    }

    double pidCalculation = pidController.calculate(wrist.getPosition().in(Rotations));

    wrist.setPower(pidCalculation);

    SmartDashboard.putNumber("Wrist PID Output", pidCalculation);
    SmartDashboard.putBoolean("Wrist At Setpoint?", pidController.atSetpoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setPower(0);
    wrist.brake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
