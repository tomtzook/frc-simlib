package frc.sim.systems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.sim.MathUtil;
import frc.sim.devices.MotorSim;

public class ElevatorSim implements SystemSim<ElevatorSim.State> {

    public static class Config {
        public final DCMotor motor;
        public final double motorToDrumGearRatio;
        public final double carriageMassKg;
        public final double drumRadiusMeters;
        public final double minHeightMeters;
        public final double maxHeightMeters;

        public Config(DCMotor motor, double motorToDrumGearRatio, double carriageMassKg, double drumRadiusMeters, double minHeightMeters, double maxHeightMeters) {
            this.motor = motor;
            this.motorToDrumGearRatio = motorToDrumGearRatio;
            this.carriageMassKg = carriageMassKg;
            this.drumRadiusMeters = drumRadiusMeters;
            this.minHeightMeters = minHeightMeters;
            this.maxHeightMeters = maxHeightMeters;
        }
    }

    public static class State {
        public final double heightMeters;

        public State(double heightMeters) {
            this.heightMeters = heightMeters;
        }
    }

    private final MotorSim motor;
    private final Config config;
    private final edu.wpi.first.wpilibj.simulation.ElevatorSim sim;

    public ElevatorSim(MotorSim motor, Config config) {
        this.motor = motor;
        this.config = config;

        sim = new edu.wpi.first.wpilibj.simulation.ElevatorSim(
                config.motor,
                config.motorToDrumGearRatio,
                config.carriageMassKg,
                config.drumRadiusMeters,
                config.minHeightMeters,
                config.maxHeightMeters,
                true,
                0
        );
    }

    public SystemOutput<State> update(Voltage busVoltage, Time dt) {
        double dtSeconds = dt.in(Units.Second);

        Voltage output = motor.updateOutput(busVoltage, dt);
        sim.setInputVoltage(output.in(Units.Volts));

        sim.update(dtSeconds);

        motor.setReverseLimitSwitchPressed(sim.hasHitLowerLimit());
        motor.setForwardLimitSwitchPressed(sim.hasHitUpperLimit());

        Angle position = MathUtil.positionMetersToRotorPosition(sim.getPositionMeters(), config.motorToDrumGearRatio, config.drumRadiusMeters);
        AngularVelocity velocity = MathUtil.velocityMpsToRotorVelocity(sim.getVelocityMetersPerSecond(), config.motorToDrumGearRatio, config.drumRadiusMeters);
        motor.setPosition(position);
        motor.setVelocity(velocity);

        return new SystemOutput<>(
                new State(sim.getPositionMeters()),
                Units.Amps.of(sim.getCurrentDrawAmps())
        );
    }
}
