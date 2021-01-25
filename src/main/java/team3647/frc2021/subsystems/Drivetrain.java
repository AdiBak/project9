package team3647.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  //initializing motor controllers
    private VictorSPX vSpx1_left = new VictorSPX(12);
    private VictorSPX vSpx2_left = new VictorSPX(13);
    private TalonSRX tSrx_left = new TalonSRX(9);

    private VictorSPX vSpx1_right = new VictorSPX(5);
    private VictorSPX vSpx2_right = new VictorSPX(6);
    private TalonSRX tSrx_right = new TalonSRX(4);

    //DifferentialDrive drive = new DifferentialDrive(tSrx_left, tSrx_right);

  public Drivetrain(){
    //VictorSPX should follow TalonSRX's current demand
    vSpx1_left.follow(tSrx_left);
    vSpx2_left.follow(tSrx_left);

    vSpx1_right.follow(tSrx_right);
    vSpx2_right.follow(tSrx_right);
    
    tSrx_right.setInverted(true);
    tSrx_left.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler runs
  }

  public void arcadeDrive(){

  }
}
