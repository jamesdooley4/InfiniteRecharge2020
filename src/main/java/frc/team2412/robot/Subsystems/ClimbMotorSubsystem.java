package frc.team2412.robot.Subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team2412.robot.RobotState;
import frc.team2412.robot.RobotState.ClimbState;
import frc.team2412.robot.Subsystems.constants.ClimbConstants;
import frc.team2412.robot.Subsystems.constants.ClimbConstants.ClimbHeight;

public class ClimbMotorSubsystem extends SubsystemBase {
	// private WPI_TalonSRX m_ClimbMotorSubsystem;

	private CANSparkMax m_leftClimbMotor;
	private CANSparkMax m_rightClimbMotor;
	private CANPIDController m_pidController;

	private CANEncoder m_encoder;

	public ClimbMotorSubsystem(CANSparkMax leftClimbMotor, CANSparkMax rightClimbMotor) {
		m_leftClimbMotor = leftClimbMotor;
		m_rightClimbMotor = rightClimbMotor;

		m_leftClimbMotor.follow(rightClimbMotor);

		m_pidController = m_rightClimbMotor.getPIDController();

		m_encoder = m_rightClimbMotor.getEncoder();

		m_pidController.setP(0.005);

	}

	public void climbExtendArm() {
		m_rightClimbMotor.set(ClimbConstants.MAX_SPEED);
	}

	public void climbRetractArm() {
		m_rightClimbMotor.set(-ClimbConstants.MAX_SPEED);
	}

	public void climbStop() {
		m_rightClimbMotor.set(0);
	}

	public double getEncoderValue() {
		return m_encoder.getPosition();
	}

	public void climbToHeight(ClimbHeight newHeight) {
		if (getEncoderValue() / ClimbConstants.inchesPerRevolution
				+ ClimbConstants.CLIMB_OFFSET_HEIGHT < newHeight.value) {
			RobotState.m_climbState = ClimbState.CLIMBING;
			climbExtendArm();
		} else {
			RobotState.m_climbState = ClimbState.NOT_CLIMBING;
			climbStop();
		}

	}

}
