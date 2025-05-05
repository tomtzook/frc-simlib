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

public class DriveSystem extends SubsystemBase {

    private final TalonFX leftMotor;
    private final TalonFXSimState leftMotorSim;
    private final StatusSignal<Angle> leftPosition;

    private final TalonFX rightMotor;
    private final TalonFXSimState rightMotorSim;
    private final StatusSignal<Angle> rightPosition;

    private final Pigeon2 pigeon;
    private final Pigeon2SimState pigeonSim;

    private final DifferentialDrivetrainSim sim;

    public DriveSystem() {
        TalonFXConfiguration talonFXConfiguration;
        Pigeon2Configuration pigeon2Configuration;

        leftMotor = new TalonFX(RobotMap.DRIVE_FRONT_LEFT);
        leftMotorSim = leftMotor.getSimState();
        talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Feedback.SensorToMechanismRatio = RobotMap.DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO;
        leftMotor.getConfigurator().apply(talonFXConfiguration);
        leftPosition = leftMotor.getPosition();

        rightMotor = new TalonFX(RobotMap.DRIVE_FRONT_RIGHT);
        rightMotorSim = rightMotor.getSimState();
        talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.Feedback.SensorToMechanismRatio = RobotMap.DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO;
        rightMotor.getConfigurator().apply(talonFXConfiguration);
        rightPosition = rightMotor.getPosition();

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
        leftMotor.setControl(new DutyCycleOut(left));
        rightMotor.setControl(new Follower(leftMotor.getDeviceID(), false));
    }

    public void stop() {
        leftMotor.setControl(new NeutralOut());
        rightMotor.setControl(new NeutralOut());
    }

    @Override
    public void periodic() {
        Voltage busVoltage = Units.Volts.of(RobotController.getBatteryVoltage());
        leftMotorSim.setSupplyVoltage(busVoltage);
        rightMotorSim.setSupplyVoltage(busVoltage);

        double leftOutput = leftMotorSim.getMotorVoltage();
        double rightOutput = rightMotorSim.getMotorVoltage();
        sim.setInputs(leftOutput, rightOutput);
        sim.update(0.02);

        double leftPosition = sim.getLeftPositionMeters() / RobotMap.DRIVE_WHEEL_CIRCUMFERENCE_M * RobotMap.DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO;
        double leftVelocity = sim.getLeftVelocityMetersPerSecond() / RobotMap.DRIVE_WHEEL_CIRCUMFERENCE_M * RobotMap.DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO;
        leftMotorSim.setRawRotorPosition(leftPosition);
        leftMotorSim.setRotorVelocity(leftVelocity);

        double rightPosition = sim.getRightPositionMeters() / RobotMap.DRIVE_WHEEL_CIRCUMFERENCE_M * RobotMap.DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO;
        double rightVelocity = sim.getRightVelocityMetersPerSecond() / RobotMap.DRIVE_WHEEL_CIRCUMFERENCE_M * RobotMap.DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO;
        rightMotorSim.setRawRotorPosition(rightPosition);
        rightMotorSim.setRotorVelocity(rightVelocity);

        pigeonSim.setRawYaw(sim.getHeading().getDegrees());

        this.leftPosition.refresh();
        this.rightPosition.refresh();
        SmartDashboard.putNumber("LeftPos", this.leftPosition.getValue().in(Units.Rotations));
        SmartDashboard.putNumber("RightPos", this.rightPosition.getValue().in(Units.Rotations));
    }
}
