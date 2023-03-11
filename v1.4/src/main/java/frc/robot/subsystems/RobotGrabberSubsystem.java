package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
// import com.revrobotics.CANSparkMax;
// import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OperatorConstants.PneumaticsConstants;

public class RobotGrabberSubsystem {
    private final DoubleSolenoid solenoid1;
     
    
     //Compressor phCompressor = new Compressor(0, PneumaticsModuleType.REVPH);

     //double current = phCompressor.getPressure();

    public RobotGrabberSubsystem() {
        solenoid1 = new DoubleSolenoid(14, PneumaticsModuleType.REVPH, 
        PneumaticsConstants.forwardChannel, PneumaticsConstants.reverseChannel);

        solenoid1.set(Value.kReverse);
        
        SmartDashboard.putString("Solenoid Value", solenoid1.get().toString());


    }



    //Might be something useful
    public void TankPressurize() {
    
    }

    //toggles the grabber
    public void toggleGrabber() {
        solenoid1.toggle();
    }

    //Maybe will close the grabber
    public void openGrabber() {
        solenoid1.set(Value.kReverse);
    }
    //Maybe will close the grabber
    public void closeGrabber() {
        solenoid1.set(Value.kForward);
    }

    
}