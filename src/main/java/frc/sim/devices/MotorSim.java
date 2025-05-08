package frc.sim.devices;


import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public interface MotorSim {

    void setInverted(boolean inverted);

    void setPosition(Angle position);
    void setVelocity(AngularVelocity velocity);

    void setForwardLimitSwitchPressed(boolean pressed);
    void setReverseLimitSwitchPressed(boolean pressed);

    Voltage updateOutput(Voltage busVoltage, Time dt);
}
