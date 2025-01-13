package frc.robot.constants;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {
  public static class RobotConstants {
    public static final String kCANivoreBus = "";
    public static final double kLowBatteryVoltage = 11.8;
  }

  public static class DrivetrainConstants {
    public static final LinearVelocity kMaxVelocity = TunerConstants.kSpeedAt12Volts;
    public static final AngularVelocity kMaxAngularRate = RotationsPerSecond.of(0.75);
  }

  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(3, 5, 7);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  public static class ElevatorConstants {
    public static final int kLeftElevatorMotorId = 7; // Arbitrary ID (change)
    public static final int kRightElevatorMotorId = 8; // Arbitrary ID (change)
    public static final int kElevatorEncoderPort = 1; // Arbitrary port (change)
    public static final Distance kUpperSoftLimit = Feet.of(12); // Arbitrary height (change)
    public static final Distance kLowerSoftLimit = Feet.of(0);
    public static final Angle kElevatorEncoderOffset = Rotations.of(0); // Arbitrary offset (change)
    public static final double kRotationToMetersRatio = 1; // Arbitrary ratio (change)
  }
}
