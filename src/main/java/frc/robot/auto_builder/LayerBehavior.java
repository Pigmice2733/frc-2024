package frc.robot.auto_builder;

import java.util.HashMap;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public interface LayerBehavior {
    public int initialize(int column, int row, String[] enumValues);

    public Command evaluate();

    public class MultiOptionLayer implements LayerBehavior {
        HashMap<String, Supplier<Command>> optionToCommandSet = new HashMap<String, Supplier<Command>>();
        private SendableChooser<String> optionChooser;

        @Override
        public int initialize(int column, int row, String[] enumValues) {

            optionChooser = new SendableChooser<String>();
            for (var name : enumValues) {
                optionChooser.addOption(name, name);
            }
            AutoBuilder.SHUFFLEBOARD_TAB.add("Choose Option", optionChooser).withPosition(column, 1);

            return 1;
        }

        @Override
        public Command evaluate() {
            String selectedOption = optionChooser.getSelected();
            if (optionToCommandSet.containsKey(selectedOption))
                return optionToCommandSet.get(selectedOption).get();
            else
                return null;
        }

        public MultiOptionLayer addCommand(Enum<?> optionName, Supplier<Command> command) {
            optionToCommandSet.put(optionName.toString(), command);
            return this;
        }
    }
}
