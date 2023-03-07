package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.commands.ArmOpenCmd;


public class GrabberSubsystem {
    //Motors
    private final CANSparkMax armMotor1;
    private final CANSparkMax armMotor2;

    
    public void testspeed() {
        armMotor1.set(0.1);
    }


    public void setDefaultCommand(ArmOpenCmd armOpenCmd) {
    }
    
}
