package org.mort11.subsystems;

import static org.mort11.config.constants.PIDConstants.Arm.*;
import static org.mort11.config.constants.PhysicalConstants.Arm.*;
import static org.mort11.config.constants.PortConstants.Arm.*;
import static org.mort11.mortlib.hardware.encoder.EncoderTypeEnum.THROUGHBORE;
import static org.mort11.mortlib.hardware.motor.MotorTypeEnum.FALCON;

import org.mort11.mortlib.subsystems.arm.ArmBase;

import edu.wpi.first.math.geometry.Rotation2d;

public class Arm extends ArmBase {
    private static Arm arm;

    public Arm() {
        super(
            THROUGHBORE, ENCODER_PORT, 
            Rotation2d.fromDegrees(ARM_ENCODER_TO_0_DEGREES), -270, -90,
            POS_KP, POS_KI, POS_KD, POS_CONSTRAINTS,
            POS_KS, POS_KG, POS_KV, POS_KA,
            FALCON, MASTER_ARM_MOTOR, FOLLOW_ARM_MOTOR
        );
    }

    @Override
    public void periodic() {
        getPIDArm().setPIDPositionDeg(getEncoderPosDeg(), getArmPos());
    }

    public static Arm getInstance() {
        if (arm == null){
            arm = new Arm();
        }
        return arm;
    }
}
