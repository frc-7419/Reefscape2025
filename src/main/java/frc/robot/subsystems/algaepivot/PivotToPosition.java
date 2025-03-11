package frc.robot.subsystems.algaepivot;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

public class PivotToPosition extends Command {
  private AlgaePivot algaePivot;
  private double setpoint;
  private TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(1.0, 1.0));
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  public PivotToPosition(AlgaePivot algaePivot, double setpoint) {
    this.algaePivot = algaePivot;
    this.setpoint = setpoint;
    addRequirements(algaePivot);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_setpoint = profile.calculate(0.02, m_setpoint, new TrapezoidProfile.State(setpoint, 0));
    algaePivot.setVelocity(m_setpoint.velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_setpoint.velocity == 0;
  }
}
