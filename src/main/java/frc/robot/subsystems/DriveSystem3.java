package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.sim.TankDriveSim;
import frc.sim.devices.Pigeon2GyroSim;
import frc.sim.devices.SparkMaxMotorSim;

public class DriveSystem3 extends SubsystemBase {

    private final SparkMax leftFrontMotor;
    private final SparkMax leftBackMotor;
    private final SparkMax rightFrontMotor;
    private final SparkMax rightBackMotor;
    private final Pigeon2 pigeon;

    private final TankDriveSim sim;

    public DriveSystem3() {
        SparkMaxConfig config;
        Pigeon2Configuration pigeon2Configuration;

        leftFrontMotor = new SparkMax(RobotMap.DRIVE_FRONT_LEFT, SparkLowLevel.MotorType.kBrushless);
        config = new SparkMaxConfig();
        config.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);
        config.encoder.positionConversionFactor(1 / RobotMap.DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO);
        config.encoder.velocityConversionFactor(1 / RobotMap.DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO);
        leftFrontMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        leftBackMotor = new SparkMax(RobotMap.DRIVE_BACK_LEFT, SparkLowLevel.MotorType.kBrushless);
        config = new SparkMaxConfig();
        config.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);
        config.encoder.positionConversionFactor(1 / RobotMap.DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO);
        config.encoder.velocityConversionFactor(1 / RobotMap.DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO);
        leftBackMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        rightFrontMotor = new SparkMax(RobotMap.DRIVE_FRONT_RIGHT, SparkLowLevel.MotorType.kBrushless);
        config = new SparkMaxConfig();
        config.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);
        config.encoder.positionConversionFactor(1 / RobotMap.DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO);
        config.encoder.velocityConversionFactor(1 / RobotMap.DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO);
        rightFrontMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        rightBackMotor = new SparkMax(RobotMap.DRIVE_BACK_RIGHT, SparkLowLevel.MotorType.kBrushless);
        config = new SparkMaxConfig();
        config.closedLoop.feedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);
        config.encoder.positionConversionFactor(1 / RobotMap.DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO);
        config.encoder.velocityConversionFactor(1 / RobotMap.DRIVE_MOTOR_TO_WHEEL_GEAR_RATIO);
        rightBackMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);

        pigeon = new Pigeon2(RobotMap.DRIVE_PIGEON);
        pigeon2Configuration = new Pigeon2Configuration();
        pigeon.getConfigurator().apply(pigeon2Configuration);

        sim = new TankDriveSim(
                new SparkMaxMotorSim(leftFrontMotor, DCMotor.getNEO(1)),
                new SparkMaxMotorSim(leftBackMotor, DCMotor.getNEO(1)),
                new SparkMaxMotorSim(rightFrontMotor, DCMotor.getNEO(1)),
                new SparkMaxMotorSim(rightBackMotor, DCMotor.getNEO(1)),
                new Pigeon2GyroSim(pigeon)
        );
    }

    public void drive(double left, double right) {
        leftFrontMotor.set(left);
        leftBackMotor.set(left);
        rightFrontMotor.set(right);
        rightBackMotor.set(right);
    }

    public void stop() {
        leftFrontMotor.stopMotor();
        leftBackMotor.stopMotor();
        rightFrontMotor.stopMotor();
        rightBackMotor.stopMotor();
    }

    @Override
    public void periodic() {
        double busVoltage = RobotController.getBatteryVoltage();
        sim.update(Units.Volts.of(busVoltage), Units.Millisecond.of(20));

        SmartDashboard.putNumber("LeftPos", this.leftFrontMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("RightPos", this.rightFrontMotor.getEncoder().getPosition());
    }
}
