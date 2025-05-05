package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class DriveSystem2 extends SubsystemBase {

    private final SparkMax leftMotor;
    private final SparkMaxSim leftMotorSim;

    private final SparkMax rightMotor;
    private final SparkMaxSim rightMotorSim;

    private final Pigeon2 pigeon;
    private final Pigeon2SimState pigeonSim;

    private final DifferentialDrivetrainSim sim;

    public DriveSystem2() {
        SparkMaxConfig config;
        Pigeon2Configuration pigeon2Configuration;

        leftMotor = new SparkMax(RobotMap.DRIVE_FRONT_LEFT, SparkLowLevel.MotorType.kBrushless);
        leftMotorSim = new SparkMaxSim(leftMotor, DCMotor.getNEO(2));
        config = new SparkMaxConfig();
        leftMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        rightMotor = new SparkMax(RobotMap.DRIVE_FRONT_RIGHT, SparkLowLevel.MotorType.kBrushless);
        rightMotorSim = new SparkMaxSim(rightMotor, DCMotor.getNEO(2));
        config = new SparkMaxConfig();
        config.follow(leftMotor);
        rightMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        pigeon = new Pigeon2(RobotMap.DRIVE_PIGEON);
        pigeonSim = pigeon.getSimState();
        pigeon2Configuration = new Pigeon2Configuration();
        pigeon.getConfigurator().apply(pigeon2Configuration);

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

    public void drive(double left, double right) {
        leftMotor.set(left);
        //rightMotor.set(right);
    }

    public void stop() {
        leftMotor.stopMotor();
        //rightMotor.stopMotor();
    }

    @Override
    public void periodic() {
        double busVoltage = RobotController.getBatteryVoltage();
        leftMotorSim.setBusVoltage(busVoltage);
        rightMotorSim.setBusVoltage(busVoltage);

        double leftOutput = leftMotorSim.getAppliedOutput() * busVoltage;
        double rightOutput = rightMotorSim.getAppliedOutput() * busVoltage;
        sim.setInputs(leftOutput, rightOutput);
        sim.update(0.02);

        double leftVelocity = sim.getLeftVelocityMetersPerSecond() / RobotMap.DRIVE_WHEEL_CIRCUMFERENCE_M * RobotMap.DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO;
        double rightVelocity = sim.getRightVelocityMetersPerSecond() / RobotMap.DRIVE_WHEEL_CIRCUMFERENCE_M * RobotMap.DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO;

        leftMotorSim.iterate(leftVelocity * 60, busVoltage, 0.02);
        rightMotorSim.iterate(rightVelocity * 60, busVoltage, 0.02);

        pigeonSim.setRawYaw(sim.getHeading().getDegrees());

        SmartDashboard.putNumber("LeftPos", this.leftMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("RightPos", this.rightMotor.getEncoder().getPosition());
    }
}
