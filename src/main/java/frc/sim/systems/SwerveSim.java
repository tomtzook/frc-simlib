package frc.sim.systems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.sim.devices.AbsEncoderSim;
import frc.sim.devices.GyroSim;
import frc.sim.devices.MotorSim;

public class SwerveSim implements SystemSim<SwerveSim.State> {

    public static class Config {
        public final ModuleConfig frontLeftConfig;
        public final ModuleConfig backLeftConfig;
        public final ModuleConfig frontRightConfig;
        public final ModuleConfig backRightConfig;

        public Config(ModuleConfig frontLeftConfig, ModuleConfig backLeftConfig, ModuleConfig frontRightConfig, ModuleConfig backRightConfig) {
            this.frontLeftConfig = frontLeftConfig;
            this.backLeftConfig = backLeftConfig;
            this.frontRightConfig = frontRightConfig;
            this.backRightConfig = backRightConfig;
        }
    }

    public static class ModuleConfig {
        public final MotorSim driveMotor;
        public final MotorSim steerMotor;
        public final AbsEncoderSim absEncoder;
        public final Translation2d modulePosition;
        public final SwerveModuleSim.Config additionalConfig;

        public ModuleConfig(MotorSim driveMotor, MotorSim steerMotor, AbsEncoderSim absEncoder, Translation2d modulePosition, SwerveModuleSim.Config additionalConfig) {
            this.driveMotor = driveMotor;
            this.steerMotor = steerMotor;
            this.absEncoder = absEncoder;
            this.modulePosition = modulePosition;
            this.additionalConfig = additionalConfig;
        }
    }

    public static class State {
        public final Pose2d pose;

        public State(Pose2d pose) {
            this.pose = pose;
        }
    }

    private final SwerveDriveKinematics kinematics;

    private final SwerveModuleSim[] modules;
    private final GyroSim gyro;

    private Pose2d lastPose;

    public SwerveSim(Config config, GyroSim gyro) {
        kinematics = new SwerveDriveKinematics(
                config.frontLeftConfig.modulePosition,
                config.backLeftConfig.modulePosition,
                config.frontRightConfig.modulePosition,
                config.backRightConfig.modulePosition
        );

        modules = new SwerveModuleSim[4];
        modules[0] = createModule(config.frontLeftConfig);
        modules[1] = createModule(config.backLeftConfig);
        modules[2] = createModule(config.frontRightConfig);
        modules[3] = createModule(config.backRightConfig);

        this.gyro = gyro;

        lastPose = Pose2d.kZero;
    }

    @Override
    public SystemOutput<State> update(Voltage busVoltage, Time dt) {
        double dtSeconds = dt.in(Units.Second);

        Current totalCurrentDraw = Units.Amps.zero();
        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        for (int i = 0; i < modules.length; i++) {
            SystemOutput<SwerveModuleState> output = modules[i].update(busVoltage, dt);
            totalCurrentDraw = totalCurrentDraw.plus(output.currentDraw);
            moduleStates[i] = output.state;
        }

        ChassisSpeeds speeds = kinematics.toChassisSpeeds(moduleStates);
        Pose2d newPose = new Pose2d(
                lastPose.getX() + speeds.vxMetersPerSecond * dtSeconds,
                lastPose.getY() + speeds.vyMetersPerSecond * dtSeconds,
                new Rotation2d(lastPose.getRotation().getRadians() + speeds.omegaRadiansPerSecond * dtSeconds)
        );

        gyro.setYaw(newPose.getRotation().getMeasure());

        lastPose = newPose;

        return new SystemOutput<>(
                new State(newPose),
                totalCurrentDraw
        );
    }

    private static SwerveModuleSim createModule(ModuleConfig config) {
        return new SwerveModuleSim(config.driveMotor, config.steerMotor, config.absEncoder, config.additionalConfig);
    }
}
