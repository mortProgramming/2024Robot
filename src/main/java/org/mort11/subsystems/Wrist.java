package org.mort11.subsystems;

import static org.mort11.configuration.constants.PhysicalConstants.Wrist.*;
import static org.mort11.configuration.constants.PIDConstants.Wrist.*;
import static org.mort11.configuration.constants.PortConstants.Wrist.*;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Wrist extends SubsystemBase {
    private static Wrist wrist;

    private TalonFX wristMotor;
    private Servo trapServo;

    private double wristSpeed;
    private double servoPos;

    private ProfiledPIDController wristPositionController;
    private SimpleMotorFeedforward wristPostionFeedForward;

    private Wrist() {
        wristMotor = new TalonFX(WRIST_MOTOR);
        trapServo = new Servo(TRAP_SERVO_PORT);

        servoPos = 90;
    
        wristPositionController = new ProfiledPIDController(
            POS_KP, POS_KI, POS_KD, POS_CONSTRAINTS
        );

        wristPostionFeedForward = new SimpleMotorFeedforward(POS_KS, POS_KV, POS_KA);
    }

    @Override
    public void periodic() {
        wristMotor.set(wristSpeed);
        trapServo.setAngle(servoPos);

        SmartDashboard.putNumber("Wrist Pos", getWristPos());
        SmartDashboard.putNumber("Wrist Pos Degrees", getWristPosDeg());
        SmartDashboard.putNumber("Wrist output", wristSpeed);
        SmartDashboard.putNumber("ActualWristMotorOutput", wristMotor.get());

        SmartDashboard.putNumber("Servo Angle", trapServo.getAngle());
    }

    public void setWristVelocity(double wristSpeed) {
        this.wristSpeed = wristSpeed + wristPostionFeedForward.calculate(getWristPos(), getWristVel());
    }

    public void setSetpoint(double setpoint) {
        wristSpeed = - wristPositionController.calculate(getWristPosDeg(), setpoint) + 
            wristPostionFeedForward.calculate(getWristPos(), getWristVel());
    }

    public void setServoPos(double servoPos) {
        this.servoPos = servoPos;
    }



    public double getWristPos() {
        return wristMotor.getPosition().getValueAsDouble();
    }

    public double getWristVel() {
        return wristMotor.getVelocity().getValueAsDouble();
    }

    public double getWristPosDeg() {
        return (getWristPos() * WRIST_GEAR_RATIO)  + WRIST_DEGREES_TO_0;
    }

    public static Wrist getInstance() {
        if (wrist == null){
            wrist = new Wrist();
        }
        return wrist;
    }
}
