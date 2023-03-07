package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
// import com.revrobotics.CANSparkMax;
// import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class RobotGrabberSubsystem {
     Solenoid solenoid1 = new Solenoid(PneumaticsModuleType.CTREPCM, 0);




    //Might be something useful
    // public void TankPressurize() {
    //     Compressor phCompressor = new Compressor(0, PneumaticsModuleType.REVPH);

    // double current = phCompressor.getPressure();
    // }

    //Maybe will close the grabber
    public void toggleGrabber() {
        solenoid1.toggle();
    }

    
}