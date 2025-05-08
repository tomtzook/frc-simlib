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
import frc.robot.RobotMap;
import frc.sim.devices.GyroSim;
import frc.sim.devices.MotorSim;

public class TankDriveSim implements SystemSim<TankDriveSim.State> {

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

    private final DifferentialDrivetrainSim sim;

    public TankDriveSim(MotorSim leftFrontMotor, MotorSim leftBackMotor, MotorSim rightFrontMotor, MotorSim rightBackMotor, GyroSim gyro) {
        this.leftFrontMotor = leftFrontMotor;
        this.leftBackMotor = leftBackMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.rightBackMotor = rightBackMotor;
        this.gyro = gyro;

        // todo: receive these
        sim = new DifferentialDrivetrainSim(
                DCMotor.getFalcon500(RobotMap.DRIVE_SIDE_MOTOR_COUNT),
                RobotMap.DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO,
                RobotMap.DRIVE_MOMENT_OF_INERTIA,
                RobotMap.ROBOT_WEIGHT_KG,
                RobotMap.DRIVE_WHEEL_RADIUS_M,
                RobotMap.DRIVE_TRACK_WIDTH_M,
                // [x, y, heading, left velocity, right velocity, left distance, right distance]
                MatBuilder.fill(Nat.N7(), Nat.N1(), 0, 0, 0, 0, 0, 0, 0)
        );
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

        Angle leftPosition = positionMetersToRotorPosition(sim.getLeftPositionMeters());
        AngularVelocity leftVelocity = velocityMpsToRotorVelocity(sim.getLeftVelocityMetersPerSecond());
        leftFrontMotor.setPosition(leftPosition);
        leftBackMotor.setPosition(leftPosition);
        leftFrontMotor.setVelocity(leftVelocity);
        leftBackMotor.setVelocity(leftVelocity);

        Angle rightPosition = positionMetersToRotorPosition(sim.getRightPositionMeters());
        AngularVelocity rightVelocity = velocityMpsToRotorVelocity(sim.getRightVelocityMetersPerSecond());
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

    private static Angle positionMetersToRotorPosition(double positionMeters) {
        return Units.Rotations.of(
                (positionMeters * RobotMap.DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO) / RobotMap.DRIVE_WHEEL_CIRCUMFERENCE_M
        );
    }

    private static AngularVelocity velocityMpsToRotorVelocity(double velocityMps) {
        return Units.RotationsPerSecond.of(velocityMps * RobotMap.DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO / RobotMap.DRIVE_WHEEL_CIRCUMFERENCE_M);
    }
}
