// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.wrist;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WristToSetpoint extends Command {
  /** Creates a new WristToSetpoint. */
  private final WristSubsystem wrist;

  private final ProfiledPIDController pidController;
  private final ArmFeedforward feedForward;
  private final double setPoint;

  public WristToSetpoint(double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wrist = new WristSubsystem();
    this.pidController = new ProfiledPIDController(0, 0, 0, null);
    this.feedForward = new ArmFeedforward(0, 0, 0);
    pidController.setTolerance(0.5);
    this.setPoint = setPoint;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.coast();
    pidController.setGoal(setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double position = wrist.getPosition();
    double velocity = wrist.getVelocity();
    double feedforward = feedForward.calculate(position, velocity);
    double output = pidController.calculate(position) + feedforward;
    wrist.setPower(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atGoal();
  }
}
