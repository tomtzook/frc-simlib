package frc.sim.systems;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.sim.MathUtil;
import frc.sim.devices.GyroSim;
import frc.sim.devices.MotorSim;

public class TankDriveSim implements SystemSim<TankDriveSim.State> {

    public static class Config {
        public final DCMotor motorPerSide;
        public final double motorToWheelGearRatio;
        public final double momentOfInertiaJkgMSquared;
        public final double weightKg;
        public final double wheelRadiusMeters;
        public final double trackWidthMeters;

        public Config(DCMotor motorPerSide, double motorToWheelGearRatio, double momentOfInertiaJkgMSquared, double weightKg, double wheelRadiusMeters, double trackWidthMeters) {
            this.motorPerSide = motorPerSide;
            this.motorToWheelGearRatio = motorToWheelGearRatio;
            this.momentOfInertiaJkgMSquared = momentOfInertiaJkgMSquared;
            this.weightKg = weightKg;
            this.wheelRadiusMeters = wheelRadiusMeters;
            this.trackWidthMeters = trackWidthMeters;
        }
    }

    public static class State {
        public final double leftPositionMeters;
        public final double rightPositionMeters;
        public final Rotation2d rotation;

        public State(double leftPositionMeters, double rightPositionMeters, Rotation2d rotation) {
            this.leftPositionMeters = leftPositionMeters;
            this.rightPositionMeters = rightPositionMeters;
            this.rotation = rotation;
        }
    }

    private final MotorSim leftFrontMotor;
    private final MotorSim leftBackMotor;
    private final MotorSim rightFrontMotor;
    private final MotorSim rightBackMotor;
    private final GyroSim gyro;

    private final Config config;
    private final DifferentialDrivetrainSim sim;

    public TankDriveSim(MotorSim leftFrontMotor, MotorSim leftBackMotor,
                        MotorSim rightFrontMotor, MotorSim rightBackMotor,
                        GyroSim gyro,
                        Config config) {
        this.leftFrontMotor = leftFrontMotor;
        this.leftBackMotor = leftBackMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.rightBackMotor = rightBackMotor;
        this.gyro = gyro;

        this.config = config;
        sim = new DifferentialDrivetrainSim(
                config.motorPerSide,
                config.motorToWheelGearRatio,
                config.momentOfInertiaJkgMSquared,
                config.weightKg,
                config.wheelRadiusMeters,
                config.trackWidthMeters,
                // [x, y, heading, left velocity, right velocity, left distance, right distance]
                MatBuilder.fill(Nat.N7(), Nat.N1(), 0, 0, 0, 0, 0, 0, 0)
        );

        leftFrontMotor.setInverted(false);
        leftFrontMotor.setPosition(Units.Degrees.zero());
        leftFrontMotor.setVelocity(Units.Degrees.per(Units.Second).zero());

        leftBackMotor.setInverted(false);
        leftBackMotor.setPosition(Units.Degrees.zero());
        leftBackMotor.setVelocity(Units.Degrees.per(Units.Second).zero());

        rightFrontMotor.setInverted(false);
        rightFrontMotor.setPosition(Units.Degrees.zero());
        rightFrontMotor.setVelocity(Units.Degrees.per(Units.Second).zero());

        rightBackMotor.setInverted(false);
        rightBackMotor.setPosition(Units.Degrees.zero());
        rightBackMotor.setVelocity(Units.Degrees.per(Units.Second).zero());

        gyro.setYaw(Units.Degrees.zero());
    }

    @Override
    public SystemOutput<TankDriveSim.State> update(Voltage busVoltage, Time dt) {
        double dtSeconds = dt.in(Units.Second);

        Voltage leftFrontOutput = leftFrontMotor.updateOutput(busVoltage, dt);
        Voltage leftBackOutput = leftBackMotor.updateOutput(busVoltage, dt);
        Voltage rightFrontOutput = rightFrontMotor.updateOutput(busVoltage, dt);
        Voltage rightBackOutput = rightBackMotor.updateOutput(busVoltage, dt);

        double leftOutput = leftFrontOutput.in(Units.Volts) + leftBackOutput.in(Units.Volts);
        double rightOutput = rightFrontOutput.in(Units.Volts) + rightBackOutput.in(Units.Volts);
        sim.setInputs(leftOutput, rightOutput);

        sim.update(dtSeconds);

        Angle leftPosition = MathUtil.positionMetersToRotorPosition(sim.getLeftPositionMeters(), config.motorToWheelGearRatio, config.wheelRadiusMeters);
        AngularVelocity leftVelocity = MathUtil.velocityMpsToRotorVelocity(sim.getLeftVelocityMetersPerSecond(), config.motorToWheelGearRatio, config.wheelRadiusMeters);
        leftFrontMotor.setPosition(leftPosition);
        leftBackMotor.setPosition(leftPosition);
        leftFrontMotor.setVelocity(leftVelocity);
        leftBackMotor.setVelocity(leftVelocity);

        Angle rightPosition = MathUtil.positionMetersToRotorPosition(sim.getRightPositionMeters(), config.motorToWheelGearRatio, config.wheelRadiusMeters);
        AngularVelocity rightVelocity = MathUtil.velocityMpsToRotorVelocity(sim.getRightVelocityMetersPerSecond(), config.motorToWheelGearRatio, config.wheelRadiusMeters);
        rightFrontMotor.setPosition(rightPosition);
        rightBackMotor.setPosition(rightPosition);
        rightFrontMotor.setVelocity(rightVelocity);
        rightBackMotor.setVelocity(rightVelocity);

        Angle gyroYaw = Units.Degrees.of(sim.getHeading().getDegrees());
        gyro.setYaw(gyroYaw);

        return new SystemOutput<>(
                new State(sim.getLeftPositionMeters(), sim.getRightPositionMeters(), sim.getHeading()),
                Units.Amps.of(sim.getCurrentDrawAmps())
        );
    }
}
