package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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
    public static final boolean runSafetyCheck = true; // Enable safety checks (DISABLE IN COMP)
  }

  public static class DrivetrainConstants {
    public static final LinearVelocity kMaxVelocity = TunerConstants.kSpeedAt12Volts;
    public static final AngularVelocity kMaxAngularRate = RotationsPerSecond.of(0.75);
  }

  public static class IntakeCoralConstants {
    public static final double intakeCoralPower = 0.1; // placeholder, insert actual value
  }

  public static class ScoringL4Constants {
    public static final double elevatorSetPoint = 0; // replace
    public static final double wristSetPoint = 0; // replace
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

  public static class WristConstants {
    public static final int kWristMotorID = 0; // Arbitrary ID (change)
    public static final int kWristEncoderID = 0; // Arbitrary ID (change)
    public static final AngularVelocity kMaxSpeed =
        RotationsPerSecond.of(1); // Arbitrary velocity (change)
    public static final Angle kAngleTolerance = Degrees.of(5); // Arbitrary angle (change)
    public static final Angle kMaxAngle = Degrees.of(90); // Arbitrary angle (change)
    public static final Angle kMinAngle = Degrees.of(90); // Arbitrary angle (change)
    public static final TalonFXConfiguration kWristTalonFXConfiguration =
        new TalonFXConfiguration();

    static {
      kWristTalonFXConfiguration.Feedback.FeedbackRemoteSensorID = kWristEncoderID;
      kWristTalonFXConfiguration.Feedback.FeedbackSensorSource =
          FeedbackSensorSourceValue.FusedCANcoder;
      kWristTalonFXConfiguration.Feedback.SensorToMechanismRatio = 1.0;
      kWristTalonFXConfiguration.Feedback.RotorToSensorRatio = 1; // Don't know yet
    }

    public static final CANcoderConfiguration kWristCANCoderConfig = new CANcoderConfiguration();

    static {
      kWristCANCoderConfig.MagnetSensor.SensorDirection =
          SensorDirectionValue.CounterClockwise_Positive;
      kWristCANCoderConfig.MagnetSensor.withMagnetOffset(Rotations.of(0)); // Change offset
    }

    public static final Slot0Configs kWristSlot0Configs = kWristTalonFXConfiguration.Slot0;

    static {
      kWristSlot0Configs.kG = 0; // output to overcome gravity (output)
      kWristSlot0Configs.kS = 0; // output to overcome static friction (output)
      kWristSlot0Configs.kV = 0; // output per unit of target velocity (output/rps)
      kWristSlot0Configs.kA = 0; // output per unit of target acceleration (output/(rps/s))
      kWristSlot0Configs.kP = 0; // output per unit of error in position (output)
      kWristSlot0Configs.kI = 0; // output per unit of integrated error in position (output)
      kWristSlot0Configs.kD = 0; // output per unit of error in velocity (output/rps)
    }

    // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html#motion-magic-expo
    public static final MotionMagicConfigs kMotionMagicConfig =
        kWristTalonFXConfiguration.MotionMagic;

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
        kWristTalonFXConfiguration.CurrentLimits;

    static {
      kCurrentLimitConfig.StatorCurrentLimit = 80; // current limit in amps
      kCurrentLimitConfig.StatorCurrentLimitEnable = true; // enable current limiting
    }

    public static final AngularVelocity UNSAFE_SPEED = RotationsPerSecond.of(1); // 1 rad/s
    public static final Temperature MAX_TEMPERATURE = Celsius.of(90); // Max rated temperature
  }

  public static class WristIntakeConstants {
    public static final int kWristIntakeMotorID = 0; // Arbitrary ID (change)
    public static final AngularVelocity kMaxSpeed =
        RotationsPerSecond.of(1); // Arbitrary velocity (change)
    public static final TalonFXConfiguration kWristIntakeTalonFXConfiguration =
        new TalonFXConfiguration();
    public static final Slot0Configs kWristIntakeSlot0Configs =
        kWristIntakeTalonFXConfiguration.Slot0;

    static {
      kWristIntakeSlot0Configs.kG = 0; // output to overcome gravity (output)
      kWristIntakeSlot0Configs.kS = 0; // output to overcome static friction (output)
      kWristIntakeSlot0Configs.kV = 0; // output per unit of target velocity (output/rps)
      kWristIntakeSlot0Configs.kA = 0; // output per unit of target acceleration (output/(rps/s))
      kWristIntakeSlot0Configs.kP = 0; // output per unit of error in position (output)
      kWristIntakeSlot0Configs.kI = 0; // output per unit of integrated error in position (output)
      kWristIntakeSlot0Configs.kD = 0; // output per unit of error in velocity (output/rps)
    }

    // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html#motion-magic-expo
    public static final MotionMagicConfigs kMotionMagicConfig =
        kWristIntakeTalonFXConfiguration.MotionMagic;

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
        kWristIntakeTalonFXConfiguration.CurrentLimits;

    static {
      kCurrentLimitConfig.StatorCurrentLimit = 80; // current limit in amps
      kCurrentLimitConfig.StatorCurrentLimitEnable = true; // enable current limiting
    }

    public static final AngularVelocity UNSAFE_SPEED = RotationsPerSecond.of(1); // 1 rad/s
    public static final Temperature MAX_TEMPERATURE = Celsius.of(90); // Max rated temperature
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
