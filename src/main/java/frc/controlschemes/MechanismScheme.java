package frc.controlschemes;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.helpers.OI;
import frc.maps.ControlMap;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;

public class MechanismScheme {
    public static void configure(Intake intake, Arm arm) {
        arm.setDefaultCommand(Commands.run(() -> arm.moveArm(OI.axis(1, ControlMap.L_JOYSTICK_VERTICAL) * 0.5,
                OI.axis(1, ControlMap.R_JOYSTICK_VERTICAL) * 0.5), arm));
        // arm.setDefaultCommand(Commands.run(() -> arm.test(1, OI.axis(1,
        // ControlMap.L_JOYSTICK_VERTICAL) * 0.5), arm));

        // arm.setDefaultCommand(Commands.run(() -> {arm.moveArm(OI.axis(1,
        // ControlMap.L_JOYSTICK_VERTICAL), OI.axis(1,
        // ControlMap.R_JOYSTICK_VERTICAL));}, arm));
        intake.setDefaultCommand(Commands.run(() -> intake.spin(
                OI.axis(1, ControlMap.LT), OI.axis(1, ControlMap.RT)), intake));

        new JoystickButton(OI.joystickArray[1], ControlMap.A_BUTTON)
                .onTrue(Commands.run(() -> intake.toggleReverse(), intake));
        // new JoystickButton(OI.joystickArray[1],
        // ControlMap.B_BUTTON).onTrue(Commands.run(() -> intake.spin(-0.5), intake));
    }
}
// :)