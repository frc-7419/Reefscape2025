package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants.ScoringConstants.ScoringSetpoint;
import frc.robot.constants.Constants.WristConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.WristIntakeSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;

public class ScoreBarge extends Command {
  private final ElevatorSubsystem elevator;
  private final WristSubsystem wrist;
  private final WristIntakeSubsystem wristIntake;
  private final ScoringSetpoint bargeSetpoint;
  
  private final Angle upAngle = Rotations.of(0.0); // Wrist out position
  private final Angle bargeAngle = Rotations.of(0.38); // Barge scoring angle
  
  private PIDController pidController =
      new PIDController(WristConstants.pidKp, WristConstants.pidKi, WristConstants.pidKd);
  
  private enum State {
    MOVING_TO_HEIGHT,
    FLIPPING_WRIST,
    SCORING
  }
  
  private State currentState = State.MOVING_TO_HEIGHT;
  private boolean wristFlipped = false;
  private boolean intakeStarted = false;
  private Timer scoringTimer = new Timer();

  public ScoreBarge(
      ElevatorSubsystem elevator, 
      WristSubsystem wrist, 
      WristIntakeSubsystem wristIntake) {
    this.elevator = elevator;
    this.wrist = wrist;
    this.wristIntake = wristIntake;
    this.bargeSetpoint = ScoringSetpoint.BARGE;
    addRequirements(elevator, wrist, wristIntake);
  }

  private void setWristAngle(Angle targetAngle) {
    double currentPos = wrist.getPosition().in(Rotations);
    pidController.setSetpoint(targetAngle.in(Rotations));
    double pidOutput = pidController.calculate(currentPos);
    pidOutput = Math.max(-5, Math.min(pidOutput, 5));
    wrist.setVoltage(pidOutput);
  }

  @Override
  public void initialize() {
    pidController.setTolerance(0.02);
    currentState = State.MOVING_TO_HEIGHT;
    wristFlipped = false;
    intakeStarted = false;
    scoringTimer.reset();
    SmartDashboard.putString("ScoreBarge State", currentState.toString());
  }

  @Override
  public void execute() {
    double elevatorRotations = elevator.getPosition().in(Rotations);
    boolean elevatorAtSetpoint = elevator
        .getPosition()
        .isNear(Rotations.of(bargeSetpoint.elevatorHeight), Rotations.of(0.1));
    
    switch (currentState) {
      case MOVING_TO_HEIGHT:
        // Move elevator to barge height while keeping wrist out
        elevator.positionMM(Rotations.of(bargeSetpoint.elevatorHeight));
        setWristAngle(upAngle);
        
        if (elevatorAtSetpoint) {
          currentState = State.FLIPPING_WRIST;
          SmartDashboard.putString("ScoreBarge State", currentState.toString());
        }
        break;
        
      case FLIPPING_WRIST:
        // Keep elevator at height and flip wrist to barge angle
        elevator.positionMM(Rotations.of(bargeSetpoint.elevatorHeight));
        setWristAngle(bargeAngle);
        
        // Check if wrist is close to barge angle
        double wristError = Math.abs(wrist.getPosition().in(Rotations) - bargeAngle.in(Rotations));
        if (wristError < 0.05) { // Within 0.05 rotations of target
          currentState = State.SCORING;
          intakeStarted = true;
          scoringTimer.start();
          SmartDashboard.putString("ScoreBarge State", currentState.toString());
        }
        break;
        
      case SCORING:
        // Keep everything in position and run intake at full speed
        elevator.positionMM(Rotations.of(bargeSetpoint.elevatorHeight));
        setWristAngle(bargeAngle);
        wristIntake.setPower(1.0); // Full speed out
        break;
    }
    
    SmartDashboard.putBoolean("ElevatorAtBargeHeight", elevatorAtSetpoint);
    SmartDashboard.putBoolean("WristAtBargeAngle", pidController.atSetpoint());
    SmartDashboard.putBoolean("IntakeRunning", intakeStarted);
    SmartDashboard.putNumber("ScoreBarge Timer", scoringTimer.get());
    SmartDashboard.putNumber("Wrist Error", Math.abs(wrist.getPosition().in(Rotations) - bargeAngle.in(Rotations)));
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setPower(0);
    wrist.setPower(0);
    wristIntake.setPower(0);
    scoringTimer.stop();
    SmartDashboard.putString("ScoreBarge State", "FINISHED");
  }

  @Override
  public boolean isFinished() {
    // Run for 1 second in scoring state to ensure algae is flung out
    return currentState == State.SCORING && scoringTimer.hasElapsed(1.0);
  }
} 