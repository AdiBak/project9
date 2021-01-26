package team3647.frc2021.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2021.subsystems.Drivetrain;

public class ex_command extends CommandBase{
    private final Drivetrain drivetrain;
    Joystick joy1 = new Joystick(0);
    
    public ex_command(Drivetrain drive){
        drivetrain = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        drivetrain.arcadeDrive(joy1.getY(), joy1.getX(), false);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
}
