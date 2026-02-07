package com.team4687.frc2026.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SwerveSendables implements Sendable {
    private boolean fieldOriented = false;


    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveConfigurables");
        builder.addBooleanProperty("fieldOriented", this::getFieldOriented, this::setFieldOriented);
    }

    public boolean getFieldOriented() {
        return fieldOriented;
    }

    public void setFieldOriented(boolean set) {
        fieldOriented = set;
    }
}
