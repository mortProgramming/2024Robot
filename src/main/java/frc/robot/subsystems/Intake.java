package frc.robot.subsystems;

import static frc.robot.utility.Constants.Intake.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    private static Intake intake;

    //left is main motor
    private TalonFX followIntakeMotor;
    private TalonFX masterIntakeMotor;
    private static DigitalInput input;

    private double intakeSpeed;

    public Intake() {
        masterIntakeMotor = new TalonFX(MASTER_INTAKE_MOTOR);
        followIntakeMotor = new TalonFX(FOLLOW_INTAKE_MOTOR);

        followIntakeMotor.setControl(new Follower(MASTER_INTAKE_MOTOR, true));

        input = new DigitalInput(INTAKE_SENSOR);
    }

   

    /**
     * Set the intake to the desired intakeSpeed
     * @param intakeSpeed
     * The speed to set the intake to, negative numbers will outtake the note
     */
    public void setIntakeVelocity(double intakeSpeed){
        this.intakeSpeed = intakeSpeed;
    }
    public boolean hasNote(){
        return input.get();
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
        masterIntakeMotor.set(intakeSpeed);

        SmartDashboard.putNumber("intake master voltage", masterIntakeMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("intake follower voltage", followIntakeMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putBoolean("HASNOTE", hasNote());

        Shuffleboard.update();
    }
    
    public static Intake getInstance(){
        if (intake==null){
            intake = new Intake();
            Shuffleboard.getTab("Intake Sensor").add("Piece In", input.get());
        }
        return intake;
    }
}
