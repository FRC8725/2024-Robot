package frc.lib.motors;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.SwerveModule;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class SwerveModuleGroup {
    private final ArrayList<SwerveModule> modules = new ArrayList<>();

    public SwerveModuleGroup(SwerveModule... modules) {
        this.modules.addAll(List.of(modules));
    }

    public SwerveModuleState[] getStates() {
        return (SwerveModuleState[]) this.modules.stream().map(SwerveModule::getState).toArray();
    }

    public SwerveModulePosition[] getPositions() {
        return (SwerveModulePosition[]) this.modules.stream().map(SwerveModule::getPosition).toArray();
    }

    public SwerveDriveKinematics constructKinematics() {
        return new SwerveDriveKinematics((Translation2d[]) this.modules.stream().map(SwerveModule::getModulePosition).toArray());
    }

    public void setDesiredStates(SwerveModuleState[] desiredStates) {
        for (int i = 0; i < this.modules.size(); ++i) {
            this.modules.get(i).setDesiredState(desiredStates[i]);
        }
    }

    public void forEach(Consumer<SwerveModule> consumer) {
        this.modules.forEach(consumer);
    }
}
