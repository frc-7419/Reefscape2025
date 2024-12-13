package frc.robot.constants;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;

public class Constants {
    public static class DrivetrainConstants {
        public static final LinearVelocity kMaxVelocity = TunerConstants.kSpeedAt12Volts;
        public static final AngularVelocity kMaxAngularRate = RotationsPerSecond.of(0.75);
    }
}
