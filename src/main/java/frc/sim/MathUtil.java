package frc.sim;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class MathUtil {

    public static double positionRotationsToDegrees(double rotations, double gearRatio) {
        return rotations / gearRatio * 360;
    }

    public static double velocityRpmToVelocityMps(double rpm, double gearRatio, double wheelRadius) {
        return rpm / 60 * 2 * Math.PI * wheelRadius / gearRatio;
    }

    public static Angle positionMetersToRotorPosition(double positionMeters, double motorToWheelGearRatio, double wheelRadiusMeters) {
        double wheelCircumference = 2 * Math.PI * wheelRadiusMeters;
        return Units.Rotations.of((positionMeters * motorToWheelGearRatio) / wheelCircumference);
    }

    public static AngularVelocity velocityMpsToRotorVelocity(double velocityMps, double motorToWheelGearRatio, double wheelRadiusMeters) {
        double wheelCircumference = 2 * Math.PI * wheelRadiusMeters;
        return Units.RotationsPerSecond.of(velocityMps * motorToWheelGearRatio / wheelCircumference);
    }
}
