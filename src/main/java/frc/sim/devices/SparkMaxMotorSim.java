package frc.sim.devices;

import com.revrobotics.sim.SparkLimitSwitchSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public class SparkMaxMotorSim implements MotorSim {

    private final SparkMax motor;

    private final SparkMaxSim sim;
    private final SparkLimitSwitchSim forwardSwitchSim;
    private final SparkLimitSwitchSim reverseSwitchSim;

    private double lastVelocity;

    public SparkMaxMotorSim(SparkMax motor, DCMotor dcMotor) {
        this.motor = motor;
        sim = new SparkMaxSim(motor, dcMotor);
        forwardSwitchSim = sim.getForwardLimitSwitchSim();
        reverseSwitchSim = sim.getReverseLimitSwitchSim();
        lastVelocity = 0;
    }

    @Override
    public void setInverted(boolean inverted) {
        // todo: support
    }

    @Override
    public void setPosition(Angle position) {
        // the sim class will calculate this on its own
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        lastVelocity = velocity.in(Units.RPM);
    }

    @Override
    public void setForwardLimitSwitchPressed(boolean pressed) {
        forwardSwitchSim.setPressed(pressed);
    }

    @Override
    public void setReverseLimitSwitchPressed(boolean pressed) {
        reverseSwitchSim.setPressed(pressed);
    }

    @Override
    public Voltage updateOutput(Voltage busVoltage, Time dt) {
        double busVolts = busVoltage.in(Units.Volts);

        // spark sim expects the velocity in post conversion factor values
        double velocity = getVelocity();
        sim.iterate(velocity, busVolts, dt.in(Units.Seconds));

        double outputVolts = sim.getAppliedOutput() * busVolts;
        return Units.Volts.of(outputVolts);
    }

    private double getVelocity() {
        // get user defined conversion factor so that no matter which factor is used, it will work
        double conversionFactor = getVelocityConversionFactor();
        return lastVelocity * conversionFactor;
    }

    private double getVelocityConversionFactor() {
        if (motor.configAccessor.closedLoop.getFeedbackSensor()
                == ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder) {
            return motor.configAccessor.absoluteEncoder.getVelocityConversionFactor();
        } else {
            return motor.configAccessor.encoder.getVelocityConversionFactor();
        }
    }
}
