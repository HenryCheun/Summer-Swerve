package frc.controlschemes;

import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.helpers.ControlScheme;
import frc.helpers.OI;
import frc.maps.ControlMap;
import frc.robot.subsystems.SwerveDrive;

public class Testing implements ControlScheme {
    public static void configure(SwerveDrive swerveDrive, int port){
        swerveDrive.setDefaultCommand(new RunCommand(() -> 
            swerveDrive.test(OI.axis(port, ControlMap.L_JOYSTICK_VERTICAL), OI.axis(port, ControlMap.R_JOYSTICK_HORIZONTAL)),
            swerveDrive));
    }
}
