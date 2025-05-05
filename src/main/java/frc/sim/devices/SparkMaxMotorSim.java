package frc.sim.devices;

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
    // todo: follow does not work
    // todo: inversion support

    private final SparkMax motor;
    private final SparkMaxSim sim;
    private double lastVelocity;

    public SparkMaxMotorSim(SparkMax motor, DCMotor dcMotor) {
        this.motor = motor;
        sim = new SparkMaxSim(motor, dcMotor);
        lastVelocity = 0;
    }

    @Override
    public void setInverted(boolean inverted) {

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
