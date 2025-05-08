package frc.sim.systems;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public interface SystemSim<State> {

    class SystemOutput<State> {
        public final State state;
        public final Current currentDraw;

        public SystemOutput(State state, Current currentDraw) {
            this.state = state;
            this.currentDraw = currentDraw;
        }
    }

    SystemOutput<State> update(Voltage busVoltage, Time dt);
}
