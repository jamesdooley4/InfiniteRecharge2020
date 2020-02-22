package frc.team2412.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.robototes.sensors.Limelight;
import com.robototes.sensors.Limelight.CamMode;
import com.robototes.sensors.Limelight.LEDMode;
import com.robototes.sensors.Limelight.Pipeline;
import com.robototes.sensors.Limelight.SnapshotMode;
import com.robototes.sensors.Limelight.StreamMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

//This is the class in charge of all the motors, motor ids, and any other sensors the robot uses.
//remember to declare robot container at the bottom of this class
public class RobotMap {

    public static enum Motor {
        DRIVE_LEFT_FRONT(14),
        DRIVE_LEFT_BACK(15),
        DRIVE_RIGHT_FRONT(1),
        DRIVE_RIGHT_BACK(0),
        CLIMB1(2),
        CLIMB2(3),
        TURRET(4),
        FLYWHEEL_LEFT(5),
        FLYWHEEL_RIGHT(6),
        CONTROL_PANEL(7);

        public final int id;
        private Motor(int motorId) {
            id = motorId;
        }
    }

    //CHOICE FOR INDEX/INTAKE MODULE
    public static enum IndexIntakeModule{
        ONE(11, 12), TWO(21, 22), THREE(31, 32);
        private int indexCANID, intakeCANID;
        IndexIntakeModule(int intakeID, int indexID){
            indexCANID = indexID;
            intakeCANID = intakeID;
        }
        public int getIntakeCANID(){
            return intakeCANID;
        }
        public int getIndexCANID(){
            return indexCANID;
        }
    }

    //SET THESE TO CHANGE MODULE
    public static IndexIntakeModule frontIndexIntakeModule = IndexIntakeModule.ONE;
    public static IndexIntakeModule backIndexIntakeModule = IndexIntakeModule.TWO;

    //NOT THIS ONE
    public static IndexIntakeModule spareIndexIntakeModule = IndexIntakeModule.THREE;

    // DRIVEBASE SUBSYSTEM
    // -------------------------------------------------------------------------
    private static final int DRIVE_SOLENOID_PORT = 0;

    // DriveBase Motors
    public static WPI_TalonFX driveLeftFront = new WPI_TalonFX(Motor.DRIVE_LEFT_FRONT.id);
    public static WPI_TalonFX driveLeftBack = new WPI_TalonFX(Motor.DRIVE_LEFT_BACK.id);
    public static WPI_TalonFX driveRightFront = new WPI_TalonFX(Motor.DRIVE_RIGHT_FRONT.id);
    public static WPI_TalonFX driveRightBack = new WPI_TalonFX(Motor.DRIVE_RIGHT_BACK.id);

    // DriveBase SpeedControllerGroups
    public static SpeedControllerGroup driveLeftSide = new SpeedControllerGroup(driveLeftFront, driveLeftBack);
    public static SpeedControllerGroup driveRightSide = new SpeedControllerGroup(driveRightFront, driveRightBack);

    // DriveBase DifferentialDrive
    public static DifferentialDrive robotDrive = new DifferentialDrive(driveLeftSide, driveRightSide);

    // DriveBase Gyro
    public static ADXRS450_Gyro driveGyro = new ADXRS450_Gyro();

    // DriveBase Solenoid
    public static Solenoid driveSolenoid = new Solenoid(DRIVE_SOLENOID_PORT);


    // climb Pneumatics
    private static final int pneumatic1Open = 1;
    private static final int pneumatic2Open = 3;
    public static Solenoid climbLeftPneumatic = new Solenoid(pneumatic1Open);
    public static Solenoid climbRightPneumatic = new Solenoid(pneumatic2Open);

    // Motors
    public static CANSparkMax leftClimbMotor = new CANSparkMax(Motor.CLIMB1.id, MotorType.kBrushless);
    public static CANSparkMax rightClimbMotor = new CANSparkMax(Motor.CLIMB2.id, MotorType.kBrushless);

    // INDEX SUBSYSTEM
    // ---------------------------------------------------------------------------
    // IDs
    private final static int indexMidMotorID = 3;
    
    private final static int backSensorID = 6;
    private final static int backMidSensorID = 5;
    private final static int backInnerSensorID = 4;
    private final static int frontInnerSensorID = 3;
    private final static int frontMidSensorID = 2;
    private final static int frontSensorID = 1;
    private final static int intakeFrontSensorID = 0;
    private final static int intakeBackSensorID = 7;

    // motors
    public static CANSparkMax indexFrontMotor, indexBackMotor;
    public static CANSparkMax indexMidMotor = new CANSparkMax(indexMidMotorID, MotorType.kBrushless);
    
    public static CANSparkMax intakeFrontMotor, intakeBackMotor;

    // sensors
    public static DigitalInput back = new DigitalInput(backSensorID);
    public static DigitalInput backMid = new DigitalInput(backMidSensorID);
    public static DigitalInput backInner = new DigitalInput(backInnerSensorID);
    public static DigitalInput frontInner= new DigitalInput(frontInnerSensorID);
    public static DigitalInput frontMid = new DigitalInput(frontMidSensorID);
    public static DigitalInput front = new DigitalInput(frontSensorID);

    private static class IndexIntakeSelector {
        IndexIntakeSelector() {
            indexFrontMotor = tryGetMotor(IndexIntakeModule.ONE.getIndexCANID());
            indexBackMotor = tryGetMotor(IndexIntakeModule.TWO.getIndexCANID());
            intakeFrontMotor = tryGetMotor(IndexIntakeModule.ONE.getIntakeCANID());
            intakeBackMotor = tryGetMotor(IndexIntakeModule.TWO.getIntakeCANID());
            if (indexFrontMotor == null) {
                indexFrontMotor = tryGetMotor(IndexIntakeModule.THREE.getIndexCANID());
                intakeFrontMotor = tryGetMotor(IndexIntakeModule.THREE.getIntakeCANID());
            } else if (indexBackMotor == null) {
                indexBackMotor = tryGetMotor(IndexIntakeModule.THREE.getIndexCANID());
                intakeBackMotor = tryGetMotor(IndexIntakeModule.THREE.getIntakeCANID());
            }
        }

        private CANSparkMax tryGetMotor(int motorId) {
            try {
                return new CANSparkMax(motorId, MotorType.kBrushless);
            } catch (Exception ignored) {
                return null;
            }
        }
    }

    public static IndexIntakeSelector indexSelector = new IndexIntakeSelector();

    // INDEXER CONTROLS THESE NOT INTAKE FYI
    public static DigitalInput intakeFront = new DigitalInput(intakeFrontSensorID);
    public static DigitalInput intakeBack = new DigitalInput(intakeBackSensorID);
    public static final int exampleID = 1;

    // Turret Subsystem
    // ------------------------------------------------------------------------------
    public static WPI_TalonSRX turretMotor = new WPI_TalonSRX(Motor.TURRET.id);

    // Flywheel subsystem
    // ------------------------------------------------------------------------------
    public static CANSparkMax flywheelLeftMotor = new CANSparkMax(Motor.FLYWHEEL_LEFT.id, MotorType.kBrushless);
    public static CANSparkMax flywheelRightMotor = new CANSparkMax(Motor.FLYWHEEL_RIGHT.id, MotorType.kBrushless);

    // Hood Subsystem
    // -----------------------------------------------------------------------------
    public static final int HOOD_SERVO_PORT_1 = 0;
	public static final int HOOD_SERVO_PORT_2 = 1;
	public static Servo hoodServo1 = new Servo(HOOD_SERVO_PORT_1);
	public static Servo hoodServo2 = new Servo(HOOD_SERVO_PORT_2);

    // INTAKE SUBSYSTEM

    // Intake DoubleSolenoid Ports
    public static final int INTAKE_FRONT_UP_PORT = 1;
    public static final int INTAKE_FRONT_DOWN_PORT = 1;
    public static final int INTAKE_BACK_UP_PORT = 1;
    public static final int INTAKE_BACK_DOWN_PORT = 1;

    public static Solenoid frontIntakeliftSolenoid = new Solenoid(INTAKE_FRONT_UP_PORT);
    public static Solenoid backIntakeLiftSolenoid = new Solenoid(INTAKE_BACK_UP_PORT);


    // LIFT SUBSYSTEM
    // -------------------------------------------------------------------------------
    // Lift DoubleSolenoid Ports
    public static final int LIFT_UP_PORT = 1;
    public static final int LIFT_DOWN_PORT = 1;

    public static DoubleSolenoid liftUpDown = new DoubleSolenoid(LIFT_UP_PORT, LIFT_DOWN_PORT);

    // CONTROL PANEL SUBSYSTEM
    // ----------------------------------------------------------------------
    // Control Panel I2C
    public static I2C.Port COLOR_SESNOR_PORT = I2C.Port.kOnboard;

    public static ColorSensorV3 colorSensor = new ColorSensorV3(COLOR_SESNOR_PORT);
    public static ColorMatch colorMatcher = new ColorMatch();

    public static WPI_TalonFX colorSensorMotor = new WPI_TalonFX(Motor.CONTROL_PANEL.id);

    // Limelight subsystem
    // ----------------------------------------------------------------------------------------------
    public static NetworkTable limelightNetworkTable = NetworkTableInstance.create().getTable("limelight");
    public static Limelight limelight = new Limelight(limelightNetworkTable, LEDMode.OFF, CamMode.VISION_PROCESSER,
            Pipeline.ZERO, StreamMode.STANDARD, SnapshotMode.OFF);

    // Unknown RN
    // -------------------------------------------------------------------------------------------------
    // Compressor
    public static Compressor compressor = new Compressor();

    // Robot container
    public static RobotContainer m_robotContainer = new RobotContainer();

    // OI
    public static OI m_OI = new OI(m_robotContainer);
}
