package lib.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveSetpoint {
    public final ChassisSpeeds mChassisSpeeds;
    public final SwerveModuleState[] mModuleStates;

    public SwerveSetpoint(ChassisSpeeds chassisSpeeds, SwerveModuleState[] initialStates){
        this.mChassisSpeeds = chassisSpeeds;
        this.mModuleStates = initialStates;
    }
}
