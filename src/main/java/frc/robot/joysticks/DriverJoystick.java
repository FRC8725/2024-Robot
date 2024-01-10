package frc.robot.joysticks;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.DriveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

@SuppressWarnings("unused")
public class DriverJoystick extends XboxController  {
    public static final double DEADBAND = 0.05;    

    public DriverJoystick(int port) {
        super(port);
    }
}
