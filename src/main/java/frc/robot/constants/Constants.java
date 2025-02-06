package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.*;

public class Constants {
  public static class RobotConstants {
    public static final String kCANivoreBus = "";
    public static final double kLowBatteryVoltage = 11.8;
    public static final double kTippingThresholdDeg = 10;
    public static final double kComHeight = 0.5; // meters
    public static final boolean runSafetyCheck = true;
  }

  public static class GroundIntakeConstants {
    public static final int kBeambreakId = 1;
    public static final int kRightIntakeMotorId = 1;
    public static final int kLeftIntakeMotorId = 1;
  }

  public static class DrivetrainConstants {
    public static final LinearVelocity kMaxVelocity = TunerConstants.kSpeedAt12Volts;
    public static final AngularVelocity kMaxAngularRate = RotationsPerSecond.of(0.75);
  }

  public static class VisionConstants {
    public static final Transform3d kRobotToCamOne =
        new Transform3d(new Translation3d(0.5, 0.1, 0.4), new Rotation3d(0, 0.34, 0));
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(3, 5, 7);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    public static final Transform3d kRobotToCamTwo =
        new Transform3d(new Translation3d(-0.5, 0.1, 0.4), new Rotation3d(0, 0.34, 0));
  }

  public static class ClawConstants {
    public static final int kClawMotorId = 10; // TODO: change this to the real ID
    public static final int kDutyEncoderChannel = 11; // TODO: change this to the real ID
    public static final int kBeambreakid = 12; // TODO: change this to the real ID
    public static final int kAbsoluteEncoderChannel = 13; // TODO: change this to the real ID
    public static final Temperature MAX_TEMPERATURE =
        Celsius.of(
            125); // I believe this sets the max temp to 125 celsius, this unit is not arbritary,
    // but i set it to 5 degrees below the actual max temp for safety
    public static final double kMetersPerRotation = 3;
    public static final Angle kClawOpenSetpoint =
        Degrees.of(19); // TODO: figure  out the value of setpoint for the desired claw open
    public static final Angle kClawCloseSetpoint =
        Degrees.of(20); // TODO: figure out the value for setpoint for claw close
    public static final Angle kAngleTolerance = Degrees.of(5); // Arbitrary guess
    public static final Angle kMaxAngle =
        Degrees.of(70); // Non Abritrary, according to mechanisms ref.
    public static final Angle kMinAngle = Degrees.of(10); // Arbitrary Guess
    public static final TalonFXConfiguration kClawTalonFXConfiguration = new TalonFXConfiguration();
    public static final Slot0Configs kClawSlot0Configs = kClawTalonFXConfiguration.Slot0;
    public static final LinearVelocity UNSAFE_SPEED = InchesPerSecond.of(200);

    static {
      kClawSlot0Configs.kG = 0; // output to overcome gravity (output)
      kClawSlot0Configs.kS = 0; // output to overcome static friction (output)
      kClawSlot0Configs.kV = 0; // output per unit of target velocity (output/rps)
      kClawSlot0Configs.kA = 0; // output per unit of target acceleration (output/(rps/s))
      kClawSlot0Configs.kP = 0; // output per unit of error in position (output)
      kClawSlot0Configs.kI = 0; // output per unit of integrated error in position (output)
      kClawSlot0Configs.kD = 0; // output per unit of error in velocity (output/rps)
    }

    // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html#motion-magic-expo
    public static final MotionMagicConfigs kMotionMagicConfig =
        kClawTalonFXConfiguration.MotionMagic;

    static {
      kMotionMagicConfig.MotionMagicCruiseVelocity =
          0; // peak velocity of the profile; set to 0 to target the
      // system’s max velocity
      kMotionMagicConfig.MotionMagicExpo_kV =
          0; // voltage required to maintain a given velocity, in V/rps
      kMotionMagicConfig.MotionMagicExpo_kA =
          0; // voltage required to maintain a given velocity, in V/rps
    }

    public static final CurrentLimitsConfigs kCurrentLimitConfig =
        kClawTalonFXConfiguration.CurrentLimits;

    static {
      kCurrentLimitConfig.StatorCurrentLimit = 80; // current limit in amps
      kCurrentLimitConfig.StatorCurrentLimitEnable = true; // enable current limiting
    }

    public static final Angle kMinPosition = Degrees.of(0);
    public static final Angle kMaxPosition = Degrees.of(0);
    public static final AngularVelocity kMaxSpeed = RotationsPerSecond.of(1);
    public static final AngularVelocity kMinSpeed = RotationsPerSecond.of(0.5);
  }

  public static class ElevatorConstants {
    public static final int kLeftElevatorMotorId = 7; // Arbitrary ID (change)
    public static final int kRightElevatorMotorId = 8; // Arbitrary ID (change)
    public static final Distance kMaxHeight = Feet.of(12); // Arbitrary height (change)
    public static final Distance kMinHeight = Feet.of(0);
    public static final double kMetersPerRotation = 1; // meters per rotation (change)
    public static final LinearVelocity kMaxSpeed =
        MetersPerSecond.of(1.27); // Arbitrary velocity (change)
    public static final TalonFXConfiguration kElevatorTalonFXConfiguration =
        new TalonFXConfiguration();
    public static final Slot0Configs kElevatorSlot0Configs = kElevatorTalonFXConfiguration.Slot0;

    static {
      kElevatorSlot0Configs.kG = 0; // output to overcome gravity (output)
      kElevatorSlot0Configs.kS = 0; // output to overcome static friction (output)
      kElevatorSlot0Configs.kV = 0; // output per unit of target velocity (output/rps)
      kElevatorSlot0Configs.kA = 0; // output per unit of target acceleration (output/(rps/s))
      kElevatorSlot0Configs.kP = 0; // output per unit of error in position (output)
      kElevatorSlot0Configs.kI = 0; // output per unit of integrated error in position (output)
      kElevatorSlot0Configs.kD = 0; // output per unit of error in velocity (output/rps)
    }

    // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html#motion-magic-expo
    public static final MotionMagicConfigs kMotionMagicConfig =
        kElevatorTalonFXConfiguration.MotionMagic;

    static {
      kMotionMagicConfig.MotionMagicCruiseVelocity =
          0; // peak velocity of the profile; set to 0 to target the
      // system’s max velocity
      kMotionMagicConfig.MotionMagicExpo_kV =
          0; // voltage required to maintain a given velocity, in V/rps
      kMotionMagicConfig.MotionMagicExpo_kA =
          0; // voltage required to maintain a given velocity, in V/rps
    }

    public static final CurrentLimitsConfigs kCurrentLimitConfig =
        kElevatorTalonFXConfiguration.CurrentLimits;

    static {
      kCurrentLimitConfig.StatorCurrentLimit = 80; // current limit in amps
      kCurrentLimitConfig.StatorCurrentLimitEnable = true; // enable current limiting
    }

    public static final Distance OVEREXTENSION_TOLERANCE = Inches.of(1); // 1 inch
    public static final LinearVelocity UNSAFE_SPEED = InchesPerSecond.of(50); // 50 in/s
    public static final LinearAcceleration UNSAFE_ACCELERATION =
        MetersPerSecondPerSecond.of(4 * 9.81); // 4g im not
    // mechanical i
    // wouldnt know
    // :(
    public static final Temperature MAX_TEMPERATURE = Celsius.of(100); // Max rated temperature
    public static final boolean runSafetyCheck = true; // Enable safety checks
  }

  public static class CameraConfig {
    public final String name;
    public final Transform3d cameraToRobot;

    public CameraConfig(String name, Transform3d cameraToRobot) {
      this.name = name;
      this.cameraToRobot = cameraToRobot;
    }
  }
}
