package org.mort11.subsystems;

import static org.mort11.config.constants.PhysicalConstants.Lights.*;
import static org.mort11.config.constants.PortConstants.Lights.*;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase{
    private static Lights lights;

    private Spark leds;

    private double ledsColor;

    private Lights() {
        leds = new Spark(LEDS_PORT);

        ledsColor = 0;
    }

    @Override
    public void periodic() {
        leds.set(ledsColor);
    }

    public void setLightsGreen() {
        setLights(GREEN_COLOR);
    }

    public void setLightsRed() {
        setLights(RED_COLOR);
    }
 
    public void setLightsBlue() {
        setLights(BLUE_COLOR);
    }

    public void setLights(double ledsColor) {
        this.ledsColor = ledsColor;
    }

    public static Lights getInstance() {
        if (lights == null) {
            lights = new Lights();
        }
        return lights;
    }
}
