package frc.sim;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class MathUtil {

    public static Angle positionMetersToRotorPosition(double positionMeters, double motorToWheelGearRatio, double wheelRadiusMeters) {
        double wheelCircumference = 2 * Math.PI * wheelRadiusMeters;
        return Units.Rotations.of((positionMeters * motorToWheelGearRatio) / wheelCircumference);
    }

    public static AngularVelocity velocityMpsToRotorVelocity(double velocityMps, double motorToWheelGearRatio, double wheelRadiusMeters) {
        double wheelCircumference = 2 * Math.PI * wheelRadiusMeters;
        return Units.RotationsPerSecond.of(velocityMps * motorToWheelGearRatio / wheelCircumference);
    }
}
