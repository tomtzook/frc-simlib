package frc.sim.devices;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.MagnetHealthValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;

public class CanCoderEncoderSim implements AbsEncoderSim {

    private final CANcoderSimState sim;

    public CanCoderEncoderSim(CANcoder caNcoder) {
        sim = caNcoder.getSimState();

        sim.setMagnetHealth(MagnetHealthValue.Magnet_Green);
        sim.setSupplyVoltage(RobotController.getBatteryVoltage());
    }

    @Override
    public void setPosition(Angle position) {
        sim.setRawPosition(position.in(Units.Rotations));
    }
}
