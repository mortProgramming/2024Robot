package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.Util.Constants.Intake.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    private static Intake intake;

    //left is main motor
    private TalonFX followIntakeMotor;
    private TalonFX masterIntakeMotor;

    private double intakeSpeed;

    public Intake() {
        masterIntakeMotor = new TalonFX(MASTER_INTAKE_MOTOR);
        followIntakeMotor = new TalonFX(FOLLOW_INTAKE_MOTOR);

        followIntakeMotor.setControl(new Follower(MASTER_INTAKE_MOTOR, false));
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
    }

    /**
     * 
     */
    @Override
    public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    }

    /**
     * 
     */
    public void intake(){
        //Motor.setpercentOutput(speed);
    }

    /**
     * 
     */
    public void outtake(){
        //Motor.setpercentOutput(-speed);
    }

    /**
     * 
     * @return
     */
    public Intake getInstance(){
        if (intake==null){
            intake = new Intake();
        }
        return intake;
    }
}
