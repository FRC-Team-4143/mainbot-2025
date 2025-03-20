package frc.mw_lib.logging;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.BuildConstants;

public abstract class GitLogger {

  private static DataLog log = DataLogManager.getLog();

  private static StringLogEntry project_name = new StringLogEntry(log, "/Metadata/PROJECT_NAME");
  private static StringLogEntry git_sha = new StringLogEntry(log, "/Metadata/GIT_SHA");
  private static StringLogEntry git_date = new StringLogEntry(log, "/Metadata/GIT_DATE");
  private static StringLogEntry git_branch = new StringLogEntry(log, "/Metadata/GIT_BRANCH");
  private static StringLogEntry build_date = new StringLogEntry(log, "/Metadata/BUILD_DATE");
  private static BooleanLogEntry dirty = new BooleanLogEntry(log, "/Metadata/DIRTY");

  public static void logGitData() {
    project_name.append(BuildConstants.MAVEN_NAME);
    git_sha.append(BuildConstants.GIT_SHA);
    git_date.append(BuildConstants.GIT_DATE);
    git_branch.append(BuildConstants.GIT_BRANCH);
    build_date.append(BuildConstants.BUILD_DATE);
    dirty.append((BuildConstants.DIRTY >= 1));
  }

  public static void putGitDataToDashboard() {
    SmartDashboard.putString("GitData/Project Name", BuildConstants.MAVEN_NAME);
    SmartDashboard.putString("GitData/Build Branch", BuildConstants.GIT_BRANCH);
    SmartDashboard.putString("GitData/Git SHA", BuildConstants.GIT_SHA.substring(0, 8));
    SmartDashboard.putString("GitData/Build Date", BuildConstants.BUILD_DATE);
    SmartDashboard.putBoolean("GitData/Dirty Git", !(BuildConstants.DIRTY >= 1));
  }
}
