package org.mort11.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;

import static org.mort11.config.constants.PortConstants.Intake.*;
import static org.mort11.config.constants.PIDConstants.Intake.*;

import static org.mort11.mortlib.hardware.motor.MotorTypeEnum.*;
import org.mort11.mortlib.subsystems.flywheel.FlywheelBase;

public class Intake extends FlywheelBase {
    private static Intake intake;

    private static DigitalInput input = new DigitalInput(INTAKE_SENSOR);

    private Intake() {
        super(
            VEL_KP, VEL_KI, VEL_KD, VEL_CONSTRAINTS, 
            FALCON, MASTER_INTAKE_MOTOR, FOLLOW_INTAKE_MOTOR
        );
    }

    public void setIntakeVelocity(double rpm){
        setFlywheelVelocity(rpm);
    }

    public static boolean hasNote() {
        return !input.get();
    }

    public static Intake getInstance() {
        if (intake == null) {
            intake = new Intake();
        }
        return intake;
    }
}
