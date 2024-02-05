package frc.robot.subsystems;

import static frc.robot.util.Constants.Intake.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    private static Intake intake;

    //left is main motor
    private TalonFX followIntakeMotor;
    private TalonFX masterIntakeMotor;
    private DigitalInput input;

    private double intakeSpeed;

    public Intake() {
        masterIntakeMotor = new TalonFX(MASTER_INTAKE_MOTOR);
        followIntakeMotor = new TalonFX(FOLLOW_INTAKE_MOTOR);

        followIntakeMotor.setControl(new Follower(MASTER_INTAKE_MOTOR, false));

        input = new DigitalInput(INTAKE_SENSOR);
    }

    public void init() {
    //add motor initialization
    }

    /**
     * 
     */
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
        masterIntakeMotor.set(intakeSpeed);
        Shuffleboard.getTab("Intake Sensor").add("Piece In", input.get());
    }

    /**
     * 
     */
    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }

    /**
     * Set the intake to the desired intakeSpeed
     * @param intakeSpeed
     * The speed to set the intake to
     */
    public void setIntakeVelocity(double intakeSpeed){
        this.intakeSpeed = intakeSpeed;
    }

    /**
     * Not sure why this exists, The intake and outtake are one subsystem that can do both, but only one at a time. Just -1 the intakevelocity method.
     */
    public void setOuttakeVelocity(double intakeSpeed){
        this.intakeSpeed = -intakeSpeed;
    }

    /**
     * 
     * @return
     */
    public static Intake getInstance(){
        if (intake==null){
            intake = new Intake();
        }
        return intake;
    }
}
