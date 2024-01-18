package frc.robot.auto_builder;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.ArrayList;

public class AutoLayer {
    private final String name;
    private final String[] enumValues;

    private int column;
    private int currentRow = 1;

    private ArrayList<LayerRestriction> restrictions = new ArrayList<LayerRestriction>();
    private LayerBehavior behavior;

    public AutoLayer(String name, Class<?> enumType) {
        this.name = name;

        System.out.println(enumType.getName());
        var enumObjs = enumType.getEnumConstants();
        System.out.println(enumObjs[0].toString());
        this.enumValues = new String[enumObjs.length];

        for (int i = 0; i < enumValues.length; i++) {
            enumValues[i] = enumObjs[i].toString();
        }
    }

    public void createHeader(int column) {
        this.column = column;
        AutoBuilder.SHUFFLEBOARD_TAB.add(name, "").withPosition(column, 0).getEntry();
    }

    public AutoLayer addRestriction(LayerRestriction restriction) {
        restrictions.add(restriction);
        currentRow += restriction.initialize(column, currentRow);

        return this;
    }

    public void setBehavior(LayerBehavior behavior) {
        this.behavior = behavior;
        currentRow += behavior.initialize(column, currentRow, name, enumValues);
    }

    public Command getCommand() {
        for (var restriction : restrictions) {
            if (restriction.evaluate() == false)
                return null;
        }

        if (behavior != null)
            return behavior.evaluate();
        else
            return null;
    }
}
