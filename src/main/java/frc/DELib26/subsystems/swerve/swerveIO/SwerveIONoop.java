package frc.DELib26.subsystems.swerve.swerveIO;

import java.util.function.Consumer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.DELib26.subsystems.vision.visionUtil.VisionFieldPoseEstimate;

public class SwerveIONoop implements SwerveIO {

    @Override
    public void updateModuleInputs(ModuleIOInputs... inputs) {}

    @Override
    public void updatePigeon2Inputs(Pigeon2IOInputs inputs) {}

    @Override
    public void registerTelemetryFunction(SwerveIOInputs inputs) {}

    @Override
    public void refreshData() {}

    @Override
    public void setSwerveState(SwerveRequest request) {}

    @Override
    public void resetRobotPose(Pose2d pose) {}

    @Override
    public void resetToParamaterizedRotation(Rotation2d rotation2d) {}

    @Override
    public void updateSimState() {}

    @Override
    public void resetRobotTranslation(Translation2d translation2d) {}

    @Override
    public void disableMotors() {}

    @Override
    public void setStandardDeviationsForState(Matrix<N3, N1> stateStdDevs) {}

    @Override
    public void addVisionMeasurement(VisionFieldPoseEstimate visionFieldPoseEstimate) {}

    @Override
    public void forEachDriveMotor(Consumer<TalonFX> consumer) {}

    @Override
    public void forEachSteerMotor(Consumer<TalonFX> consumer) {}
    
}
