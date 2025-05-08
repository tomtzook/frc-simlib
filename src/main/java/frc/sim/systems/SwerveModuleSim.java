package frc.sim.systems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.sim.MathUtil;
import frc.sim.devices.AbsEncoderSim;
import frc.sim.devices.MotorSim;

public class SwerveModuleSim implements SystemSim<SwerveModuleState> {

    public static class Config {
        public final DCMotor driveMotor;
        public final double driveMomentOfInertia;
        public final double driveToWheelGearRatio;
        public final double driveWheelRadiusMeters;
        public final DCMotor steerMotor;
        public final double steerMomentOfInertia;
        public final double steerToWheelGearRatio;

        public Config(DCMotor driveMotor, double driveMomentOfInertia, double driveToWheelGearRatio, double driveWheelRadiusMeters, DCMotor steerMotor, double steerMomentOfInertia, double steerToWheelGearRatio) {
            this.driveMotor = driveMotor;
            this.driveMomentOfInertia = driveMomentOfInertia;
            this.driveToWheelGearRatio = driveToWheelGearRatio;
            this.driveWheelRadiusMeters = driveWheelRadiusMeters;
            this.steerMotor = steerMotor;
            this.steerMomentOfInertia = steerMomentOfInertia;
            this.steerToWheelGearRatio = steerToWheelGearRatio;
        }
    }

    private final MotorSim driveMotor;
    private final MotorSim steerMotor;
    private final AbsEncoderSim absEncoder;
    private final Config config;

    private final DCMotorSim driveSim;
    private final DCMotorSim steerSim;

    public SwerveModuleSim(MotorSim driveMotor, MotorSim steerMotor, AbsEncoderSim absEncoder, Config config) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
        this.absEncoder = absEncoder;
        this.config = config;

        driveSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        config.driveMotor,
                        config.driveMomentOfInertia,
                        config.driveToWheelGearRatio),
                config.driveMotor
        );
        steerSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        config.steerMotor,
                        config.steerMomentOfInertia,
                        config.steerToWheelGearRatio),
                config.steerMotor
        );

        driveMotor.setInverted(false);
        driveMotor.setPosition(Units.Degrees.zero());
        driveMotor.setVelocity(Units.Degrees.per(Units.Second).zero());

        steerMotor.setInverted(false);
        steerMotor.setPosition(Units.Degrees.zero());
        steerMotor.setVelocity(Units.Degrees.per(Units.Second).zero());
    }

    @Override
    public SystemOutput<SwerveModuleState> update(Voltage busVoltage, Time dt) {
        double dtSeconds = dt.in(Units.Second);

        Voltage driveOutput = driveMotor.updateOutput(busVoltage, dt);
        driveSim.setInputVoltage(driveOutput.in(Units.Volts));

        Voltage steerOutput = steerMotor.updateOutput(busVoltage, dt);
        steerSim.setInputVoltage(steerOutput.in(Units.Volts));

        if (driveOutput.abs(Units.Volts) < 0.001) {
            driveSim.setState(driveSim.getAngularPositionRad(), 0);
        }
        if (steerOutput.abs(Units.Volts) < 0.001) {
            steerSim.setState(steerSim.getAngularPositionRad(), 0);
        }

        driveSim.update(dtSeconds);
        steerSim.update(dtSeconds);

        driveMotor.setPosition(Units.Rotations.of(driveSim.getAngularPositionRotations() * config.driveToWheelGearRatio));
        driveMotor.setVelocity(Units.RPM.of(driveSim.getAngularVelocityRPM() * config.driveToWheelGearRatio));

        steerMotor.setPosition(Units.Rotations.of(steerSim.getAngularPositionRotations() * config.steerToWheelGearRatio));
        steerMotor.setVelocity(Units.RPM.of(steerSim.getAngularVelocityRPM() * config.steerToWheelGearRatio));

        absEncoder.setPosition(steerSim.getAngularPosition());

        double driveVelocityMps = MathUtil.velocityRpmToVelocityMps(driveSim.getAngularVelocityRPM(), config.driveToWheelGearRatio, config.driveWheelRadiusMeters);
        double steerPositionDegrees = MathUtil.positionRotationsToDegrees(steerSim.getAngularPositionRotations(), config.steerToWheelGearRatio);
        return new SystemOutput<>(
                new SwerveModuleState(
                        driveVelocityMps,
                        Rotation2d.fromDegrees(steerPositionDegrees)
                ),
                Units.Amps.of(driveSim.getCurrentDrawAmps() + steerSim.getCurrentDrawAmps())
        );
    }
}
