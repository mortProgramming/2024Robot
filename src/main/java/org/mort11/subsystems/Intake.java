package org.mort11.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;

import static org.mort11.config.constants.PortConstants.Intake.*;
import static org.mort11.config.constants.PIDConstants.Intake.*;

import static org.mort11.mortlib.hardware.motor.MotorTypeEnum.*;
import static org.mort11.mortlib.logger.LoggerTypeEnum.*;
import org.mort11.mortlib.hardware.motor.MotorGroup;
import org.mort11.mortlib.logger.LoggerGroup;
import org.mort11.mortlib.subsystems.flywheel.FlywheelBase;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends FlywheelBase {
    private static Intake intake;

    private static DigitalInput input = new DigitalInput(INTAKE_SENSOR);

    private Intake() {
        super(
            VEL_KP, VEL_KI, VEL_KD, VEL_CONSTRAINTS, 
            FALCON, MASTER_INTAKE_MOTOR, FOLLOW_INTAKE_MOTOR
        );
    }

    @Override
    public void periodic() {
        motors.setVoltage(intakeSpeed);
    }

    public void setIntakeVelocity(double intakeSpeed){
        this.intakeSpeed = intakeSpeed;
    }

    public boolean hasNote() {
        return !input.get();
    }

    public static boolean hasNoteStatic() {
        return !input.get();
    }

    public static Intake getInstance() {
        if (intake == null) {
            intake = new Intake();
        }
        return intake;
    }
}
