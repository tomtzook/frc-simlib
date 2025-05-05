package frc.sim.devices;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;

public class Pigeon2GyroSim implements GyroSim {

    private final Pigeon2SimState sim;

    public Pigeon2GyroSim(Pigeon2 pigeon) {
        sim = pigeon.getSimState();

        sim.setRawYaw(0);
        sim.setPitch(0);
        sim.setRoll(0);
        sim.setSupplyVoltage(RobotController.getBatteryVoltage());
    }

    @Override
    public void setYaw(Angle angle) {
        double angleDegrees = angle.in(Units.Degrees);
        sim.setRawYaw(angleDegrees);
    }
}
