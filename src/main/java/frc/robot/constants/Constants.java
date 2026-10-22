package frc.robot.constants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.DELib26.subsystems.swerve.swerveUtil.SwerveConfig;
import frc.DELib26.subsystems.vision.visionUtil.VisionSubsystemConfiguration;
import frc.DELib26.util.vision.CameraConfiguration;

import java.util.Optional;

public final class Constants {

	public static final class AutonomousConstants {
		private AutonomousConstants() {}

		public static final double MAX_START_TRANS_ERROR_METERS = 0.8;
		public static final double MAX_START_HEADING_ERROR_DEGREES = 180;
		public static final double AUTONOMOUS_VISION_LOOKBACK_TIME_SECONDS = 0.5;
	}

	public static final class RobotConstants {
		private RobotConstants() {}

		public static enum LoggingMode {
			NONE,
			BASIC,
			EXTENDED
		}

		public static final LoggingMode SIM_MODE = LoggingMode.EXTENDED;
		public static final LoggingMode REAL_MODE = LoggingMode.EXTENDED;

		public static final int TELEMETRY_INTERVAL_CYCLES = 4;

		public static final int USB_CAMERA_FPS = 15;

	}

	public static final class SwerveConstants {
		private SwerveConstants() {}

		public static final SwerveConfig SWERVE_CONFIG = new RobotSwerveConfig();

		private static final class RobotSwerveConfig extends SwerveConfig {
			public RobotSwerveConfig() {
				super();
				//TODO: Fill in the swerve configuration parameters specific to your robot here
			}

			@Override
			protected TalonFXConfiguration getDefaultDriveTalonFXConfig() {
				return new TalonFXConfiguration()
						.withCurrentLimits(new CurrentLimitsConfigs()
								.withSupplyCurrentLimitEnable(true)
								.withStatorCurrentLimitEnable(true)
								.withSupplyCurrentLimit(30.0)
								.withStatorCurrentLimit(DRIVE_STATOR_CURRENT_LIMIT))
						.withMotorOutput(new MotorOutputConfigs()
								.withNeutralMode(NeutralModeValue.Brake));
			}

			@Override
			protected TalonFXConfiguration getDefaultSteerTalonFXConfig() {
				return new TalonFXConfiguration()
						.withCurrentLimits(new CurrentLimitsConfigs()
								.withSupplyCurrentLimitEnable(true)
								.withStatorCurrentLimitEnable(true)
								.withSupplyCurrentLimit(30.0)
								.withStatorCurrentLimit(STEER_STATOR_CURRENT_LIMIT))
						.withMotorOutput(new MotorOutputConfigs()
								.withNeutralMode(NeutralModeValue.Brake));
			}
		}
	}

	public static final class VisionConstants {
		private VisionConstants() {}

		public static final VisionSubsystemConfiguration VISION_CONFIG = new RobotVisionConfig();
		public static final double VISION_POSE_ESTIMATE_MAX_AGE_SECONDS = 0.5;

		private static final class RobotVisionConfig extends VisionSubsystemConfiguration {
			public RobotVisionConfig() {
				super.cameraConfigurations = new CameraConfiguration[] {
					//TODO: Fill in the camera configurations specific to your robot here
				};
				this.filteredAprilTagDataForGyro = Optional.empty();
				this.aprilTagFieldLayout = FieldConstants.FIELD_LAYOUT;
			}
		}
	}

}