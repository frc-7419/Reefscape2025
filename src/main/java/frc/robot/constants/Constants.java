package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class Constants {
  public static class RobotConstants {
    public static final String kCANivoreBus = "7419";
    public static final double kLowBatteryVoltage = 11.8;
    public static final double kTippingThresholdDeg = 10;
    public static final double kComHeight = 0.5; // meters
    public static final boolean runSafetyCheck = false; // Enable safety checks (DISABLE IN COMP)
  }

  public static class DrivetrainConstants {
    public static final LinearVelocity kMaxVelocity = TunerConstants.kSpeedAt12Volts;
    public static final AngularVelocity kMaxAngularRate = RotationsPerSecond.of(3);
    public static final PIDController kPoseVelocityXController = new PIDController(5, 0, 0);
    public static final PIDController kPoseVelocityYController = new PIDController(5, 0, 0);
    public static final PIDController kPoseThetaController = new PIDController(10, 0, 0);

    public static final double kTranslationDeadband = 0.01;
    public static final double kRotationDeadband = 0.01;
  }

  public static class IntakeCoralConstants {
    public static final double intakeCoralVoltage = -2; // placeholder, insert actual value
  }

  public static class ScoringConstants {
    public static final double elevatorSetPointL4 = 0; // replace
    public static final double elevatorSetPointL3 = 0; // replace
    public static final double elevatorSetPointL2 = 0; // replace
    public static final double elevatorSetPointL1 = 0; // replace
    public static final double wristSetPointL4 = 0; // replace
    public static final double wristSetPointL3 = 0; // replace
    public static final double wristSetPointL2 = 0; // replace
    public static final double wristSetPointL1 = 0; // replace

    public static int[] reefIds = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};

    public enum ScoringSetpoint {
      L1("L1", 0, 0.49, false),
      L2("L2", 5, 0.49, true),
      L3("L3", 12.7, 0.49, true),
      L4("L4", 27.5, 0.49, true),
      HIGH_ALGAE("HIGH_ALGAE", 11, 0.1, false),
      LOW_ALGAE("LOW_ALGAE", 0, 0.1, false),
      BARGE("BARGE", 29.5, 0.25, true),
      HOME("HOME", 0, 0.42, true);

      public final String name;
      public final double elevatorHeight;
      public final double wristAngle;
      public final boolean lateWrist;

      ScoringSetpoint(String name, double elevatorHeight, double wristAngle, boolean lateWrist) {
        this.name = name;
        this.elevatorHeight = elevatorHeight;
        this.wristAngle = wristAngle;
        this.lateWrist = lateWrist;
      }
    }

    public static Map<Integer, Pose2d> getReefPoseMap() {
      AprilTagFieldLayout fieldLayout =
          AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

      Map<Integer, Pose2d> poseMap = new HashMap<>();

      for (int tagId : reefIds) {
        Optional<Pose3d> optionalPose = fieldLayout.getTagPose(tagId);

        if (optionalPose.isPresent()) {
          Pose2d pose = optionalPose.get().toPose2d();
          poseMap.put(tagId, pose);
        }
      }

      return poseMap;
    }

    public static Map<Integer, Pose2d> reefPoseMap = getReefPoseMap();

    public static final Translation2d leftReefOffset =
        new Translation2d(0.2286, 0.1651); // should probally be robot
    // bumper
    // offset to be flush and then the left
    public static final Translation2d rightReefOffset =
        new Translation2d(0.2286, -0.1651); // should probally be robot
    // bumper
    // offset to be flush and then the
    // right

    public enum ScoringHeights {
      L1,
      L2,
      L3,
      L4
    }

    public enum ScoringPosition {
      LEFT,
      RIGHT
    }
  }

  public static class VisionConstants {
    public static final Transform3d kRobotToCamOne =
        new Transform3d(new Translation3d(Units.inchesToMeters(11.375), Units.inchesToMeters(11.375), Units.inchesToMeters(9)), new Rotation3d(0, Units.degreesToRadians(61.875), 0));
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(3, 5, 7);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    public static final AprilTagFieldLayout kTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public static final Transform3d kRobotToCamTwo =
    new Transform3d(new Translation3d(Units.inchesToMeters(11.375), Units.inchesToMeters(-11.375), Units.inchesToMeters(9)), new Rotation3d(0, Units.degreesToRadians(61.875), 0));
  }

  public static class WristConstants {
    public static final int kWristMotorID = 12; // Arbitrary ID (change)
    public static final int kWristEncoderID = 0; // Arbitrary ID (change)
    public static final AngularVelocity kMaxSpeed =
        RotationsPerSecond.of(1); // Arbitrary velocity (change)
    public static final Angle kAngleTolerance = Degrees.of(5); // Arbitrary angle (change)
    public static final Angle kMaxAngle = Degrees.of(90); // Arbitrary angle (change)
    public static final Angle kMinAngle = Degrees.of(90); // Arbitrary angle (change)
    public static final TalonFXConfiguration kWristTalonFXConfiguration =
        new TalonFXConfiguration();

    public static final double pidKp = 26;
    public static final double pidKi = 0.0;
    public static final double pidKd = 0.2;

    public static final double feedforwardKv = 0.0;
    public static final double feedforwardKg = 0.0;
    public static final double feedforwardKs = 0.0;

    public static final Angle wristAngleOffset = Degrees.of(-146.7);
    public static final MotorOutputConfigs kMotorOutputConfig =
        kWristTalonFXConfiguration.MotorOutput;

    static {
      kMotorOutputConfig.Inverted = InvertedValue.Clockwise_Positive;
    }

    public static final CurrentLimitsConfigs kCurrentLimitConfig =
        kWristTalonFXConfiguration.CurrentLimits;

    static {
      kCurrentLimitConfig.StatorCurrentLimit = 80; // current limit in amps
      kCurrentLimitConfig.StatorCurrentLimitEnable = true; // enable current limiting
    }

    public static final AngularVelocity UNSAFE_SPEED = RotationsPerSecond.of(1); // 1 rad/s
    public static final Temperature MAX_TEMPERATURE = Celsius.of(90); // Max rated temperature

    public static final Angle kElevatorSafeWristAngle = Rotations.of(0.36);
  }

  public static class WristIntakeConstants {
    public static final int kWristIntakeBeamBreakChannel = 1; // Arbritary ID
    public static final int kWristIntakeMotorID = 13; // Arbitrary ID (change)
    public static final AngularVelocity kMaxSpeed =
        RotationsPerSecond.of(1); // Arbitrary velocity (change)
    public static final TalonFXConfiguration kWristIntakeTalonFXConfiguration =
        new TalonFXConfiguration();

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
    public static final int kLeftElevatorMotorId = 9; // Arbitrary ID (change)
    public static final int kRightElevatorMotorId = 11; // Arbitrary ID (change)
    public static final int kTopElevatorMotorId = 10; // Arbitrary ID (change)
    public static final Angle kMaxRotations = Rotations.of(29.5); // Taken from canvas
    public static final Angle kMinRotations = Rotations.of(0); // Taken from canvas
    public static final AngularVelocity kMaxSpeed = RotationsPerSecond.of(5);
    public static final TalonFXConfiguration kElevatorTalonFXConfiguration =
        new TalonFXConfiguration();
    public static final Angle kElevatorBarUpperLimit = Rotations.of(2.5);
    public static final Angle kElevatorBarLowerLimit = Rotations.of(9.4);

    public static final Slot0Configs kElevatorSlot0Configs = kElevatorTalonFXConfiguration.Slot0;

    static {
      kElevatorSlot0Configs.kG = 0.42; // output to overcome gravity (output)
      kElevatorSlot0Configs.kS = 0.52; // output to overcome static friction (output)
      kElevatorSlot0Configs.kV = 0.01; // output per unit of target velocity (output/rps)
      kElevatorSlot0Configs.kA = 0.0005; // output per unit of target acceleration (output/(rps/s))
      kElevatorSlot0Configs.kP = 0.4; // output per unit of error in position (output)
      kElevatorSlot0Configs.kI = 0; // output per unit of integrated error in position (output)
      kElevatorSlot0Configs.kD = 0.1; // output per unit of error in velocity (output/rps)
    }

    // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html#motion-magic-expo
    public static final MotionMagicConfigs kMotionMagicConfig =
        kElevatorTalonFXConfiguration.MotionMagic;

    static {
      kMotionMagicConfig.MotionMagicCruiseVelocity = 0;
      // system’s max velocity
      kMotionMagicConfig.MotionMagicExpo_kV = 0.01;
      kMotionMagicConfig.MotionMagicExpo_kA = 0.0005;
    }

    public static final double pidKp = 0.59;
    public static final double pidKi = 0.0;
    public static final double pidKd = 0.1;

    public static final double feedforwardKv = 0.26;
    public static final double feedforwardKa = 0.01;
    public static final double feedforwardKg = 0.42;
    public static final double feedforwardKs = 0.52;

    public static final double kMaxVelocity = 5;
    public static final double kMaxAcceleration = 1;

    public static final double joystickDeadband = 0.01;

    public static final CurrentLimitsConfigs kCurrentLimitConfig =
        kElevatorTalonFXConfiguration.CurrentLimits;

    static {
      kCurrentLimitConfig.StatorCurrentLimit = 80; // current limit in amps
      kCurrentLimitConfig.StatorCurrentLimitEnable = true; // enable current limiting
    }

    public static final Angle OVEREXTENSION_TOLERANCE = Rotations.of(1); // 1 rotation
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
