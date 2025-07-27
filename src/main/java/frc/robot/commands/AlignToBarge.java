package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.Constants.DrivetrainConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class AlignToBarge extends Command {
  private final CommandSwerveDrivetrain drivetrain;
  private final CommandXboxController driver;
  private final double MaxSpeed = 4.5;
  private double targetX;
  private double targetTheta;

  private final ProfiledPIDController pidX = DrivetrainConstants.kPoseVelocityXController;
  private final ProfiledPIDController pidTheta = DrivetrainConstants.kPoseThetaController;

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // Define target positions for both alliances
  private static final double RED_TARGET_X = 9.35;
  private static final double BLUE_TARGET_X = 8.2;
  private static final double RED_TARGET_THETA = 0;
  private static final double BLUE_TARGET_THETA = 180;

  /**
   * Creates a new AlignToBarge command.
   *
   * @param drivetrain The drivetrain subsystem
   * @param driver The driver controller for Y-axis input
   */
  public AlignToBarge(CommandSwerveDrivetrain drivetrain, CommandXboxController driver) {
    this.drivetrain = drivetrain;
    this.driver = driver;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Pose2d robotPose = drivetrain.getState().Pose;

    // Determine which target is closer
    double distanceToRed = Math.abs(robotPose.getX() - RED_TARGET_X);
    double distanceToBlue = Math.abs(robotPose.getX() - BLUE_TARGET_X);

    if (distanceToRed <= distanceToBlue) {
      this.targetX = RED_TARGET_X;
      this.targetTheta = RED_TARGET_THETA;
    } else {
      this.targetX = BLUE_TARGET_X;
      this.targetTheta = BLUE_TARGET_THETA;
    }

    resetPID(robotPose);

    SmartDashboard.putNumber("Barge Target X", targetX);
    SmartDashboard.putString("Barge Target", targetX == RED_TARGET_X ? "Red" : "Blue");
    SmartDashboard.putNumber("Barge Distance to Red", distanceToRed);
    SmartDashboard.putNumber("Barge Distance to Blue", distanceToBlue);

    // Set tolerances
    pidX.setTolerance(0.02);
    pidTheta.setTolerance(Units.degreesToRadians(0.1));

    // Set constraints
    pidX.setConstraints(DrivetrainConstants.kVelocityConstraints);
    pidTheta.setConstraints(DrivetrainConstants.kThetaConstraints);
    pidTheta.enableContinuousInput(-Math.PI, Math.PI);

    // Set goals - only X position and rotation (facing forward)
    pidX.setGoal(targetX);
    pidTheta.setGoal(Units.degreesToRadians(targetTheta));
  }

  public boolean atGoal() {
    boolean xReached = pidX.atGoal();
    boolean angleReached = pidTheta.atGoal();

    return xReached && angleReached;
  }

  private void resetPID(Pose2d robotPose) {
    pidX.reset(robotPose.getX());
    pidTheta.reset(robotPose.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentPose = drivetrain.getState().Pose;

    // Calculate PID outputs for X position and rotation
    double vx = pidX.calculate(currentPose.getX());
    double omega = pidTheta.calculate(currentPose.getRotation().getRadians());

    // Get Y velocity from joystick (allowing driver control)
    double vy = -driver.getLeftX() * MaxSpeed * 0.5; // Reduced speed for precision

    SmartDashboard.putNumber("Barge PID vx", vx);
    SmartDashboard.putNumber("Barge PID vy", vy);
    SmartDashboard.putNumber("Barge PID omega", omega);
    SmartDashboard.putNumber("Barge Current X", currentPose.getX());
    SmartDashboard.putNumber("Barge X Error", targetX - currentPose.getX());
    SmartDashboard.putBoolean("Barge At Goal", atGoal());

    final double vxf = vx;
    final double vyf = vy;
    final double omegaf = omega;
    drivetrain.setControl(drive.withVelocityX(vxf).withVelocityY(vyf).withRotationalRate(omegaf));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
