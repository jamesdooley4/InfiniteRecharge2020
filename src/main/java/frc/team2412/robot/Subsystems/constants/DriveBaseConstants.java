package frc.team2412.robot.Subsystems.constants;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

public class DriveBaseConstants {

	private static double wheelDiameterMeters = 0.1524;
	public static final double metersPerEncoderRevolution = wheelDiameterMeters * Math.PI;

	public static final double MAX_VOLTAGE = 12;

	public static final boolean kGyroReversed = false;

	public static final double ksVolts = 1;
	public static final double kvVoltSecondsPerMeter = 1;
	public static final double kaVoltSecondsSquaredPerMeter = 1;

	public static final int kPDriveVel = 8;

	public static final double kTrackwidthMeters = 0.5461; // Horizontal distance between wheels

	public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
			kTrackwidthMeters);

	public static final double kMaxSpeedMetersPerSecond = 1; // Max speed we can drive
	public static final double kMaxAccelerationMetersPerSecondSquared = 1;

	// Ramsete Controller Value
	public static final double kRamseteB = 2; // makes a more straight curve
	public static final double kRamseteZeta = 0.7; // limits the correction

}
