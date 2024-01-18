package frc.robot.auto_builder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

public interface LayerRestriction {
    public int initialize(int column, int row);

    public boolean evaluate();

    public class ToggleableLayer implements LayerRestriction {
        private GenericEntry toggleEntry;

        @Override
        public int initialize(int column, int row) {
            toggleEntry = AutoBuilder.SHUFFLEBOARD_TAB.add("Enabled", false)
                    .withWidget(BuiltInWidgets.kToggleSwitch).withPosition(column, row).getEntry();
            return 1;
        }

        @Override
        public boolean evaluate() {
            return toggleEntry.getBoolean(false);
        }
    }
}
