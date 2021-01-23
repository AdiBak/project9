package team3647.frc2021.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
   // private VictorSPX vSpx1_left = new VictorSPX(1);
    private VictorSPX vSpx2_left = new VictorSPX(2);
    private TalonSRX tSrx_left = new TalonSRX(1);

   // private VictorSPX vSpx1_right = new VictorSPX(3);
    private VictorSPX vSpx2_right = new VictorSPX(4);
    private TalonSRX tSrx_right = new TalonSRX(3);

  public Drivetrain(){
    vSpx2_left.follow(tSrx_left);
    vSpx2_right.follow(tSrx_right);

    tSrx_right.setInverted(true);
    tSrx_left.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler runs
  }
}
