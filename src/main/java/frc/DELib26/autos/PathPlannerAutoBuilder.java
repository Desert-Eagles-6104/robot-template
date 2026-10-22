package frc.DELib26.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.DELib26.subsystems.swerve.SwerveSubsystem;
import frc.DELib26.subsystems.swerve.swerveUtil.SwerveConfig;
import frc.DELib26.util.FieldUtil;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

/**
 * A utility class to build Path Planner autos.
 */
public class PathPlannerAutoBuilder {
	private RobotConfig robotConfig;
	private final Map<String, Supplier<PathPlannerAuto>> autosMap = new HashMap<>();

	public PathPlannerAutoBuilder(
			SwerveSubsystem swerveSubsystem,
			SwerveConfig swerveConfig,
			Map<String, Command> namedCommands
		) {

		Pathfinding.setPathfinder(new LocalADStarAK());

		this.robotConfig = getRobotConfig(swerveConfig);

		AutoBuilder.configure(
				swerveSubsystem::getPose,
				swerveSubsystem::resetPose,
				swerveSubsystem::getRobotSpeeds,
				(speeds, feedforwards) -> swerveSubsystem.pathPlannerDriveAuto(speeds, feedforwards),
				new PPHolonomicDriveController(
						swerveConfig.PATH_PLANNER_TRANSLATION_CONTROLLER.toPIDConstants(),
						swerveConfig.PATH_PLANNER_ROTATION_CONTROLLER.toPIDConstants()),
				this.robotConfig,
				() -> !FieldUtil.isBlueAllianceOrDefault(),
				swerveSubsystem
			);

		namedCommands.forEach(NamedCommands::registerCommand);

		List<String> pathNames = AutoBuilder.getAllAutoNames();
		pathNames.forEach(
				name -> this.autosMap.put(name, () -> (PathPlannerAuto) AutoBuilder.buildAuto(name))
			);

		// Logging
		PathPlannerLogging.setLogTargetPoseCallback(
				pose -> Logger.recordOutput("PathPlanner/TargetPose", pose));

		PathPlannerLogging.setLogActivePathCallback(
				poses -> Logger.recordOutput("PathPlanner/ActivePath", poses.toArray(Pose2d[]::new)));
	}

	public PathPlannerAutoBuilder(SwerveSubsystem swerveSubsystem, SwerveConfig swerveConfig) {
		this(swerveSubsystem, swerveConfig, new HashMap<String, Command>());
	}

	public PathConstraints getDefaultPathConstraints(SwerveConfig swerveConfig) {
		return new PathConstraints(
			swerveConfig.INITIAL_MAX_VELOCITY_FOR_DRIVE_TO_POINT,
			swerveConfig.MAX_LINEAR_ACCELERATION,
			swerveConfig.getMaxAngularSpeed(),
			swerveConfig.MAX_ANGULAR_ACCELERATION
		);
	}

	private static RobotConfig getRobotConfig(SwerveConfig swerveConfig) {
		ModuleConfig moduleCfg = new ModuleConfig(
				swerveConfig.MODULE_HARDWARE_CONSTANTS.getWheelRadius(),
				// It wants the absolute max speed no reduction for friction,
				// it took me 7 hours to figure that out. fuck life.
				swerveConfig.MODULE_HARDWARE_CONSTANTS.getTheoreticalMaxLinearSpeedMps(),
				swerveConfig.WHEEL_COEFFICIENT_OF_FRICTION.in(Units.Value),
				swerveConfig.MODULE_HARDWARE_CONSTANTS.driveMotor,
				swerveConfig.MODULE_HARDWARE_CONSTANTS.driveGearRatio,
				swerveConfig.DRIVE_STATOR_CURRENT_LIMIT.in(Units.Amps),
				1);

		RobotConfig robotConfig = new RobotConfig(
				swerveConfig.ROBOT_MASS,
				swerveConfig.ROBOT_MOMENT_OF_INERTIA,
				moduleCfg,
				swerveConfig.getModuleLocations());

		return robotConfig;
	}

	public Map<String, Supplier<PathPlannerAuto>> getAutosMap() {
		return this.autosMap;
	}

	public static Command getResetOdomForPath(PathPlannerPath path) {
		// Get the starting pose from the path (blue-side origin)
		// At resetOdom it will fiip if needed
		Pose2d blueStart = path.getStartingHolonomicPose()
				.orElse(path.getStartingDifferentialPose());
		// Uses the resetPose consumer given in AutoBuilder.configure(...)
		return AutoBuilder.resetOdom(blueStart);
	}

	public static Command followPathWithReset(PathPlannerPath path) {
		return Commands.sequence(
				getResetOdomForPath(path),
				AutoBuilder.followPath(path)
		);
	}

	public static PathPlannerPath loadPath(String pathName) {
		try {
			return PathPlannerPath.fromPathFile(pathName);
		} catch (Exception e) {
			System.err.println("Error loading path: " + pathName);
			return null; // I dont care about null pointer exceptions here
		}
	}

	public static Command followNamedPath(String pathName) {
		return AutoBuilder.followPath(loadPath(pathName));
	}

	public static Command followPath(PathPlannerPath path) {
		return AutoBuilder.followPath(path);
	}

	public static Command addStopToPath(Command pathCommand, SwerveSubsystem swerveSubsystem) {
		return pathCommand.andThen(Commands.runOnce(() -> swerveSubsystem.stop() ));
	}

	public static Command followNamedPathWithStop(String pathName, SwerveSubsystem swerveSubsystem) {
		return addStopToPath(followNamedPath(pathName), swerveSubsystem);
	}

}
