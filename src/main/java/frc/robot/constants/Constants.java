package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;

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
    public static final Distance kUpperSoftLimit = Feet.of(12); // Arbitrary height (change)
    public static final Distance kLowerSoftLimit = Feet.of(0);
    public static final double kMetersPerRotationRatio = 1; // meters per rotation (change)
    public static final LinearVelocity kMaxVelocity =
        MetersPerSecond.of(1.27); // Arbitrary velocity (change)
    public static final TalonFXConfiguration elevatorTalonFXConfiguration =
        new TalonFXConfiguration();
    public static final Slot0Configs elevatorSlot0Configs = elevatorTalonFXConfiguration.Slot0;

    static {
      elevatorSlot0Configs.kG = 0; // output to overcome gravity (output)
      elevatorSlot0Configs.kS = 0; // output to overcome static friction (output)
      elevatorSlot0Configs.kV = 0; // output per unit of target velocity (output/rps)
      elevatorSlot0Configs.kA = 0; // output per unit of target acceleration (output/(rps/s))
      elevatorSlot0Configs.kP = 0; // output per unit of error in position (output)
      elevatorSlot0Configs.kI = 0; // output per unit of integrated error in position (output)
      elevatorSlot0Configs.kD = 0; // output per unit of error in velocity (output/rps)
    }
    // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html#motion-magic-expo
    public static final MotionMagicConfigs elevatorMotionMagicConfigs =
        elevatorTalonFXConfiguration.MotionMagic;

    static {
      elevatorMotionMagicConfigs.MotionMagicCruiseVelocity =
          0; // peak velocity of the profile; set to 0 to target the
      // system’s max velocity
      elevatorMotionMagicConfigs.MotionMagicExpo_kV =
          0; // voltage required to maintain a given velocity, in V/rps
      elevatorMotionMagicConfigs.MotionMagicExpo_kA =
          0; // voltage required to maintain a given velocity, in V/rps
    }

    public static final Distance overextensionTolerance = Inches.of(1); // 1 inch
    public static final LinearVelocity unsafeVelocity = InchesPerSecond.of(50); // 50 in/s
    public static final LinearAcceleration unsafeAcceleration =
        MetersPerSecondPerSecond.of(4 * 9.81); // 4g im not
    // mechanical i
    // wouldnt know
    // :(
    public static final Temperature maxTemperature = Celsius.of(100); // Max rated temperature
    public static final boolean runSafetyCheck = true; // Enable safety checks
  }
}
