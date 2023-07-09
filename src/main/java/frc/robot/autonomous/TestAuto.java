// package frc.robot.autonomous;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;

// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.maps.RobotMap;
// import frc.robot.subsystems.SwerveDrive;

// public class TestAuto extends SequentialCommandGroup{

//     // Create constructor for when this auto is called in RobotContainer
//     // Constuctor must contain sybsystems used so that they can be referenced
//     public TestAuto(SwerveDrive swerveDrive, boolean firstPath){

//         // Set name for command selector
//         this.setName("Test Auto");
//         // This adds the commands to the class that this class extends, SequentialCommandGroup
//         // So, when you call this auto in RobotContainer, it will run these commands
//         // NOTE: ALL LINES HERE MUST BE COMMANDS
//         super.addCommands(
//             swerveDrive.followPath(PathPlanner.loadPath("First Test", new PathConstraints(RobotMap.MAX_SPEED_METERS_PER_SECOND, RobotMap.DRIVE_RATE_LIMIT)), firstPath)
//         );
//     }
// }

// // Inefficient, possibly condense everything into one autonomous class and use .getName()
// // Might not work due to command-based nature of command selector