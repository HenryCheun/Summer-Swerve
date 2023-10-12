package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.helpers.CCSparkMax;
import frc.maps.RobotMap;

public class Arm extends SubsystemBase{
    private CCSparkMax armRight1 = new CCSparkMax("Arm Right One", "ar1", RobotMap.ARM_RIGHT_1, MotorType.kBrushless, IdleMode.kBrake, RobotMap.ARM_RIGHT_1_REVERSED);
    private CCSparkMax armRight2 = new CCSparkMax("Arm Right Two", "ar2", RobotMap.ARM_RIGHT_2, MotorType.kBrushless, IdleMode.kBrake, RobotMap.ARM_RIGHT_2_REVERSED);
    MotorControllerGroup right = new MotorControllerGroup(armRight1, armRight2);
    private CCSparkMax armLeft1 = new CCSparkMax("Arm Left One", "al1", RobotMap.ARM_LEFT_1, MotorType.kBrushless, IdleMode.kBrake, RobotMap.ARM_LEFT_1_REVERSED);
    private CCSparkMax armLeft2 = new CCSparkMax("Arm Left Two", "al2", RobotMap.ARM_LEFT_2, MotorType.kBrushless, IdleMode.kBrake, RobotMap.ARM_LEFT_2_REVERSED);
    MotorControllerGroup left = new MotorControllerGroup(armLeft1, armLeft2);
    private CCSparkMax topArm = new CCSparkMax("Top Arm", "ta", RobotMap.TOP_ARM, MotorType.kBrushless, IdleMode.kBrake, RobotMap.TOP_ARM_REVERSED);
    public void moveArm(double speedBottom, double speedTop) {
        topArm.set(speedTop);
        right.set(speedBottom);
        left.set(speedBottom);
    }
}
