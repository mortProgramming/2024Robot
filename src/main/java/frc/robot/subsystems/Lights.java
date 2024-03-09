package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.utility.Constants.Lights.*;

public class Lights extends SubsystemBase{
    private static Lights lights;

    private static Spark leds;

    public Lights() {
        leds = new Spark(LEDS_PORT);
    }

    public void setLightsGreen() {
        leds.set(GREEN_COLOR);
    }

    public void setLightsRed() {
        leds.set(RED_COLOR);
    }

    public void setLightsBlue() {
        leds.set(BLUE_COLOR);
    }

    public void setLights(double color) {
        leds.set(color);
    }

    public static Lights getInstance() {
        if (lights == null) {
            lights = new Lights();
        }
        return lights;
    }
}
