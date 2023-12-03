package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;
import frc.maps.RobotMap;

public class Intake extends SubsystemBase {
    private CCSparkMax front = new CCSparkMax("Front", "F", RobotMap.FRONT_INTAKE, MotorType.kBrushless, IdleMode.kBrake, RobotMap.FRONT_INTAKE_REVERSED);
    private CCSparkMax back = new CCSparkMax("Back", "B", RobotMap.BACK_INTAKE, MotorType.kBrushless, IdleMode.kBrake, RobotMap.BACK_INTAKE_REVERSED);
    private boolean isReversed = false;
    // MotorControllerGroup intake = new MotorControllerGroup(intake1, intake2);
    public void spin(double frontSpeed, double backSpeed) {
        front.set(frontSpeed * (isReversed ? -1 : 1));
        back.set(backSpeed * (isReversed ? -1 : 1));
    }
    public void toggleReverse() {
        isReversed = !isReversed;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Reversed", isReversed);
    }
}
