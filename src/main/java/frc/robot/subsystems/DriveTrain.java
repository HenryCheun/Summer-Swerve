// package frc.robot.subsystems;

// import com.kauailabs.navx.frc.AHRS;
// import com.revrobotics.CANSparkMax.IdleMode;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.networktables.BooleanEntry;
// import edu.wpi.first.wpilibj.SPI;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.helpers.CCSparkMax;
// import frc.maps.RobotMap;
// import frc.helpers.*;

// public class DriveTrain extends SubsystemBase{
//     // Declaring 4 motors on chassis
//     // add encoder error things later at end of constructors
//     private final CCSparkMax fl = new CCSparkMax("Front Left", "fl", RobotMap.FRONT_LEFT, MotorType.kBrushless, IdleMode.kBrake, RobotMap.FRONT_LEFT_REVERSE);
//     private final CCSparkMax fr = new CCSparkMax("Front Right", "fr", RobotMap.FRONT_LEFT, MotorType.kBrushless, IdleMode.kBrake, RobotMap.FRONT_RIGHT_REVERSE);
//     private final CCSparkMax bl = new CCSparkMax("Back Left", "bl", RobotMap.FRONT_LEFT, MotorType.kBrushless, IdleMode.kBrake, RobotMap.BACK_LEFT_REVERSE);
//     private final CCSparkMax br = new CCSparkMax("Back Right", "br", RobotMap.FRONT_LEFT, MotorType.kBrushless, IdleMode.kBrake, RobotMap.BACK_RIGHT_REVERSE);

//     // Connecting each side of the chassis' motors
//     private final MotorControllerGroup left = new MotorControllerGroup(fl, bl);
//     private final MotorControllerGroup right = new MotorControllerGroup(fr, br);

//     // Create gyro
//     public final AHRS gyro = new AHRS(SPI.Port.kMXP);

//     // Supply move and turn speed values
//     // crude, fix later
//     public void axisDrive(double speed, double turnSpeed){
//         arcadeDrive(speed * speed * Math.signum(speed), turnSpeed * turnSpeed * Math.signum(turnSpeed));
//     }

//     // Supply a forward speed, yAxis, and a sideways speed, xAxis
//     private void arcadeDrive(double yAxis, double xAxis){
//         double max = 1;
//         left.set(OI.normalize(yAxis - xAxis, -max, max));
//         right.set(OI.normalize(yAxis + xAxis, -max, max));
//     }

//     // PID controllers for auto movement and turning
//     // tune these later
//     PIDController movePID = new PIDController(.5, .01, 0);
//     PIDController turnPID = new PIDController(.5, .01, 0);

//     // Caps the speed of the auto movement
//     double autoSpeedCap = .7;
//     // countInError counts for how many frames the encoder has been within the error (timeCorrect)
//     int countInError;
//     int timeCorrect = 20;

//     double acceptedError = .1;

//     // Move a distance away in autonomous
//     public Command moveTo(double destination){
//         InstantCommand ins = new InstantCommand(() -> {
//             fr.reset();
//             countInError = 0;
//         });
        
//         RunCommand res = new RunCommand(() -> {
//             arcadeDrive(OI.normalize(movePID.calculate(fr.getPosition(), destination), -autoSpeedCap, autoSpeedCap), 0);
//             if(Math.abs(fr.getPosition() - destination) < acceptedError){
//                 countInError++;
//             } else {
//                 countInError = 0;
//             };
//         }, this){
//             @Override
//             public boolean isFinished(){
//                 return countInError > timeCorrect;
//             }
//         };

//         return new SequentialCommandGroup(ins, res);
//     }

//     // countInError counts for how many frames the encoder has been within the error (timeCorrect)
//     int turnCountInError;
//     int turnTimeCorrect = 20;
    
//     int turnAcceptedError = 10;

//     // Turn a certain amount of degrees
//     public Command turnAngle(int angle){
//         InstantCommand ins = new InstantCommand(() -> {
//             gyro.reset();
//             turnCountInError = 0;
//         });

//         RunCommand res = new RunCommand(() -> {
//             axisDrive(0, turnPID.calculate(angle, gyro.getYaw()));
//             if(Math.abs(gyro.getYaw() - angle) < turnAcceptedError){
//                 turnCountInError++;
//             } else {
//                 turnCountInError = 0;
//             };
//         }){
//             @Override
//             public boolean isFinished(){
//                 return turnCountInError > turnTimeCorrect;
//             }
//         };

//         return new SequentialCommandGroup(ins, res);
//     }
// }
