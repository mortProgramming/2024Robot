package org.mort11.subsystems;

import static org.mort11.configuration.constants.PhysicalConstants.Lights.*;
import static org.mort11.configuration.constants.PortConstants.Lights.*;
import static org.mort11.configuration.constants.PortConstants.Vision.*;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase{
    private static Lights lights;

    private Spark leds;

    private double ledsColor;

    public Lights() {
        leds = new Spark(LEDS_PORT);

        ledsColor = 0;

        setLimelightsOff();
    }

    @Override
    public void periodic() {
        leds.set(ledsColor);

        LimelightHelpers.setLEDMode_ForceBlink("");
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

    public static void setLimelightsOff() {
        LimelightHelpers.setLEDMode_ForceOff(NOTE_CAMERA);
        LimelightHelpers.setLEDMode_ForceOff(TAG_CAMERA);
    }

    public static void setLimelightsBlink() {
        LimelightHelpers.setLEDMode_ForceBlink(NOTE_CAMERA);
        LimelightHelpers.setLEDMode_ForceBlink(TAG_CAMERA);
    }

    public static void setLimelightsOn() {
        LimelightHelpers.setLEDMode_ForceOn(NOTE_CAMERA);
        LimelightHelpers.setLEDMode_ForceOn(TAG_CAMERA);
    }

    public static Lights getInstance() {
        if (lights == null) {
            lights = new Lights();
        }
        return lights;
    }
}
