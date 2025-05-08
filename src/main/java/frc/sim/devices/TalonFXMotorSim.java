package frc.sim.devices;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public class TalonFXMotorSim implements MotorSim {

    private final TalonFXSimState sim;

    public TalonFXMotorSim(TalonFX motor) {
        sim = motor.getSimState();
    }

    @Override
    public void setInverted(boolean inverted) {
        sim.Orientation = inverted ? ChassisReference.CounterClockwise_Positive : ChassisReference.Clockwise_Positive;
    }

    @Override
    public void setPosition(Angle position) {
        sim.setRawRotorPosition(position);
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        sim.setRotorVelocity(velocity);
    }

    @Override
    public void setForwardLimitSwitchPressed(boolean pressed) {
        sim.setForwardLimit(pressed);
    }

    @Override
    public void setReverseLimitSwitchPressed(boolean pressed) {
        sim.setReverseLimit(pressed);
    }

    @Override
    public Voltage updateOutput(Voltage busVoltage, Time dt) {
        sim.setSupplyVoltage(busVoltage);
        return sim.getMotorVoltageMeasure();
    }
}
