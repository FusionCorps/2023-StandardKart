package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.lang.reflect.Field;

import static frc.robot.Constants.*;

public class Chassis extends SubsystemBase {

    WPI_TalonFX drive_bl;
    WPI_TalonFX drive_br;
    WPI_TalonFX drive_fl;
    WPI_TalonFX drive_fr;

    TalonFXSimCollection drive_l_sim;
    TalonFXSimCollection drive_r_sim;

    AHRS gyro = new AHRS();

    DifferentialDrive diff_drive;

    DifferentialDrivetrainSim drive_sim = new DifferentialDrivetrainSim(
            DCMotor.getFalcon500(2),
            Constants.kGearRatio,
            2.1, // insert moment of inertia here
            26.5, // insert mass of robot here
            Units.inchesToMeters(kWheelRadiusInches),
            kTrackwidthMeters, //wheel distance
            null
    );

    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));

    Field2d field = new Field2d();

    DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    public Chassis() {
        drive_bl = new WPI_TalonFX(Constants.DRIVE_TALON_BL_ID);
        drive_br = new WPI_TalonFX(Constants.DRIVE_TALON_BR_ID);
        drive_fl = new WPI_TalonFX(Constants.DRIVE_TALON_FL_ID);
        drive_fr = new WPI_TalonFX(Constants.DRIVE_TALON_FR_ID);

        drive_fl.setInverted(TalonFXInvertType.CounterClockwise);
        drive_fr.setInverted(TalonFXInvertType.Clockwise);

        drive_fl.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        drive_fr.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

        drive_bl.follow(drive_fl);
        drive_bl.setInverted(TalonFXInvertType.FollowMaster);

        drive_br.follow(drive_fr);
        drive_br.setInverted(TalonFXInvertType.FollowMaster);

        drive_fl.setNeutralMode(NeutralMode.Brake);
        drive_fr.setNeutralMode(NeutralMode.Brake);

        drive_bl.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 35, 0.5));
        drive_br.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 35, 0.5));
        drive_fl.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 35, 0.5));
        drive_fr.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 35, 0.5));

        drive_l_sim = drive_fl.getSimCollection();
        drive_r_sim = drive_fr.getSimCollection();

        diff_drive = new DifferentialDrive(drive_fl, drive_fr);

        SmartDashboard.putData("Field", field);

    }

    public void curvatureDrive(double fwd, double rot) {
        diff_drive.curvatureDrive(fwd, rot, true);
    }

    public void setDriveVolts(double left, double right) {
        drive_fl.setVoltage(left);
        drive_bl.setVoltage(left);

        drive_fr.setVoltage(right);
        drive_br.setVoltage(right);
    }

    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    @Override
    public void periodic() {
        diff_drive.feed();

    }

    @Override
    public void simulationPeriodic() {
        /* Pass the robot battery voltage to the simulated Talon FXs */
        drive_l_sim.setBusVoltage(RobotController.getBatteryVoltage());
        drive_r_sim.setBusVoltage(RobotController.getBatteryVoltage());

        // need to manually invert the right side motor
        drive_sim.setInputs(drive_l_sim.getMotorOutputLeadVoltage(),
                -drive_r_sim.getMotorOutputLeadVoltage());

        // advance model
        drive_sim.update(0.02);

        // update sensors
        drive_l_sim.setIntegratedSensorRawPosition(
                distanceToNativeUnits(
                        drive_sim.getLeftPositionMeters()
                ));
        drive_l_sim.setIntegratedSensorVelocity(
                velocityToNativeUnits(
                        drive_sim.getLeftVelocityMetersPerSecond()
                ));
        drive_r_sim.setIntegratedSensorRawPosition(
                distanceToNativeUnits(
                        -drive_sim.getRightPositionMeters()
                ));
        drive_r_sim.setIntegratedSensorVelocity(
                velocityToNativeUnits(
                        -drive_sim.getRightVelocityMetersPerSecond()
                ));

        angle.set(drive_sim.getHeading().getDegrees());

        m_odometry.update(drive_sim.getHeading(),
                nativeUnitsToDistanceMeters(drive_fl.getSelectedSensorPosition()),
                nativeUnitsToDistanceMeters(drive_fr.getSelectedSensorPosition()));
        field.setRobotPose(m_odometry.getPoseMeters());
    }

    private int distanceToNativeUnits(double positionMeters){
        double wheelRotations = positionMeters/(2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
        double motorRotations = wheelRotations * kGearRatio;
        int sensorCounts = (int)(motorRotations * kCountsPerRev);
        return sensorCounts;
    }

    private int velocityToNativeUnits(double velocityMetersPerSecond){
        double wheelRotationsPerSecond = velocityMetersPerSecond/(2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
        double motorRotationsPerSecond = wheelRotationsPerSecond * kGearRatio;
        double motorRotationsPer100ms = motorRotationsPerSecond / k100msPerSecond;
        int sensorCountsPer100ms = (int)(motorRotationsPer100ms * kCountsPerRev);
        return sensorCountsPer100ms;
    }

    private double nativeUnitsToDistanceMeters(double sensorCounts){
        double motorRotations = sensorCounts / kCountsPerRev;
        double wheelRotations = motorRotations / kGearRatio;
        double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(kWheelRadiusInches));
        return positionMeters;
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(drive_fl.getSelectedSensorVelocity()*Constants.kEncoderDistancePerPulse,
                drive_fr.getSelectedSensorVelocity()*Constants.kEncoderDistancePerPulse);
    }

}
