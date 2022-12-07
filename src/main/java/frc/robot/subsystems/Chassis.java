package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Chassis extends SubsystemBase {

    WPI_TalonFX drive_bl;
    WPI_TalonFX drive_br;
    WPI_TalonFX drive_fl;
    WPI_TalonFX drive_fr;

    DifferentialDrive diff_drive;

    public Chassis() {
        drive_bl = new WPI_TalonFX(Constants.DRIVE_TALON_BL_ID);
        drive_br = new WPI_TalonFX(Constants.DRIVE_TALON_BR_ID);
        drive_fl = new WPI_TalonFX(Constants.DRIVE_TALON_FL_ID);
        drive_fr = new WPI_TalonFX(Constants.DRIVE_TALON_FR_ID);

        drive_fl.setInverted(TalonFXInvertType.Clockwise);
        drive_fr.setInverted(TalonFXInvertType.CounterClockwise);

        drive_bl.follow(drive_fl);
        drive_bl.setInverted(TalonFXInvertType.FollowMaster);

        drive_br.follow(drive_fr);
        drive_br.setInverted(TalonFXInvertType.FollowMaster);

        drive_fl.setNeutralMode(NeutralMode.Brake);
        drive_fr.setNeutralMode(NeutralMode.Brake);

        diff_drive = new DifferentialDrive(drive_fl, drive_fr);

    }

    public void curvatureDrive(double fwd, double rot) {
        diff_drive.curvatureDrive(fwd, rot, true);
    }

}