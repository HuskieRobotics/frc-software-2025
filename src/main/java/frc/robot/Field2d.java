package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team6328.util.FieldConstants;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;

/**
 * This singleton class models the field as a collection of regions. This class is used to create a
 * path from a starting pose in one region to an ending pose in another region that passes through
 * the transition points defined for those regions.
 *
 * <p>The coordinate system of the field is oriented such that the origin is in the lower left
 * corner when the blue alliance is to the left (i.e., to the blue alliance driver's right).
 */
public class Field2d {
  private static Field2d instance = null;

  private Region2d[] regions;

  private Alliance alliance = DriverStation.Alliance.Blue;

  private Map<Pose2d, Pose2d> leftReefPoses = new HashMap<Pose2d, Pose2d>();
  private Map<Pose2d, Pose2d> rightReefPoses = new HashMap<Pose2d, Pose2d>();
  private Map<Pose2d, Pose2d> removeAlgaePoses = new HashMap<Pose2d, Pose2d>();
  private Pose2d[] allReefCenterFaces = new Pose2d[12];

  private final boolean COMPETITION_FIELD =
      true; // set TRUE if home field calibration or at competition

  private static final double PIPE_FROM_REEF_CENTER_INCHES =
      6.469; // taken from FieldConstants adjustY for reef y offset

  /**
   * Get the singleton instance of the Field2d class.
   *
   * @return the singleton instance of the Field2d class
   */
  public static Field2d getInstance() {
    if (instance == null) {
      instance = new Field2d();
    }
    return instance;
  }
  /**
   * Construct a Field2d from an array of regions. These regions should not be overlapping (aside
   * from edges) and any regions with overlapping edges should be neighbors (see
   * Region2d::addNeighbor).
   *
   * @param regions the regions that define the field
   */
  public void setRegions(Region2d[] regions) {
    this.regions = regions;
  }

  /**
   * Create a path from a starting pose in one region to an ending pose in another region that
   * passes through the transition points defined for those regions.
   *
   * @param start the starting pose
   * @param end the ending pose
   * @param pathConstants the path constraints (i.e., max velocity, max acceleration)
   * @param subsystem the drivetrain subsystem
   * @return the path from the starting pose to the ending pose; null if no path exists
   */
  public PathPlannerPath makePath(
      Pose2d start, Pose2d end, PathConstraints pathConstants, Drivetrain subsystem) {
    Region2d startRegion = null;
    Region2d endRegion = null;

    // find the starting and ending regions
    for (Region2d region : regions) {
      if (region.contains(start)) {
        startRegion = region;
      }
      if (region.contains(end)) {
        endRegion = region;
      }
    }

    // make sure both start and end are on the field
    if (startRegion == null || endRegion == null) return null;

    // BFS to find the shortest path to the end
    List<Region2d> path = breadthFirstSearch(startRegion, endRegion);
    if (path.isEmpty()) return null;

    // create point locations
    ArrayList<Translation2d> pointLocations = new ArrayList<>();

    // add the starting point
    pointLocations.add(start.getTranslation());

    // add all the transition points
    for (int i = 0; i < path.size() - 1; i++) {
      Region2d from = path.get(i);
      Region2d to = path.get(i + 1);
      pointLocations.add(from.getTransitionPoint(to));
    }

    // add a transition point if starting region & ending region same
    if (startRegion == endRegion) {
      pointLocations.add(
          new Translation2d((start.getX() + end.getX()) / 2, (start.getY() + end.getY()) / 2));
    }

    // add the ending point
    pointLocations.add(end.getTranslation());

    List<Pose2d> pathPoses = createPathPoses(pointLocations);
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(pathPoses);
    return new PathPlannerPath(
        waypoints,
        pathConstants,
        null,
        new GoalEndState(
            RobotConfig.getInstance().getMoveToPathFinalVelocity(), end.getRotation()));
  }

  /**
   * Create the path points based on the starting and ending poses and the point locations. The path
   * will be created such that the first path point matches the robot's current heading and velocity
   * to ensure a smooth transition to the path. The the starting and ending poses have different
   * rotations, the change in rotation will occur between the first and second points. The final
   * speed of the robot will be as specified by the robot's configuration class'
   * getMoveToPathFinalVelocity method.
   *
   * @param start the starting pose
   * @param end the ending pose
   * @param subsystem the drivetrain subsystem
   * @param pointLocations the locations of the points in the path
   * @return the path points
   */
  private List<Pose2d> createPathPoses(ArrayList<Translation2d> pointLocations) {
    List<Pose2d> pathPoses = new ArrayList<>();
    Rotation2d lastHeading = null;
    for (int i = 0; i < pointLocations.size() - 1; i++) {
      double deltaX = pointLocations.get(i + 1).getX() - pointLocations.get(i).getX();
      double deltaY = pointLocations.get(i + 1).getY() - pointLocations.get(i).getY();
      lastHeading = new Rotation2d(deltaX, deltaY);
      pathPoses.add(
          new Pose2d(pointLocations.get(i).getX(), pointLocations.get(i).getY(), lastHeading));
    }

    // the final path point will match the ending pose's rotation and the velocity as specified by
    // the robot's configuration class' getMoveToPathFinalVelocity method.
    pathPoses.add(
        new Pose2d(
            pointLocations.get(pointLocations.size() - 1).getX(),
            pointLocations.get(pointLocations.size() - 1).getY(),
            lastHeading));

    return pathPoses;
  }

  private List<Region2d> breadthFirstSearch(Region2d start, Region2d end) {
    Queue<ArrayList<Region2d>> todo = new LinkedList<>();
    Set<Region2d> explored = new HashSet<>();

    // add the starting region to the set of explored regions
    explored.add(start);

    // if the path starts and ends in the same region, return that region
    if (start == end) {
      return new ArrayList<>(Arrays.asList(start));
    }

    todo.add(
        new ArrayList<>(Arrays.asList(start))); // add a path starting with startRegion to the list

    while (!todo.isEmpty()) { // while the list isn't empty, keep looking over the list.
      ArrayList<Region2d> path = todo.poll();
      Region2d region = path.get(path.size() - 1); // last region in the path

      for (Region2d other : region.getNeighbors()) {
        if (!explored.contains(other)) {
          ArrayList<Region2d> newPath = new ArrayList<>(path);
          newPath.add(other);

          if (other == end) {
            return newPath;
          }

          explored.add(other);
          todo.add(newPath);
        }
      }
    }
    return new ArrayList<>();
  }

  /**
   * This method should be invoked once the alliance color is known. Refer to the RobotContainer's
   * checkAllianceColor method for best practices on when to check the alliance's color. The
   * alliance color is needed when running auto paths as those paths are always defined for
   * blue-alliance robots and need to be flipped for red-alliance robots.
   *
   * @param newAlliance the new alliance color
   */
  public void updateAlliance(DriverStation.Alliance newAlliance) {
    this.alliance = newAlliance;
  }

  /**
   * Get the alliance color.
   *
   * @return the alliance color
   */
  public Alliance getAlliance() {
    return alliance;
  }

  public void populateReefBranchPoseMaps() {
    if (COMPETITION_FIELD) {
      Pose2d[] blueReefRightBranches = populateBlueReefRightBranches();
      Pose2d[] blueReefLeftBranches = populateBlueReefLeftBranches();
      Pose2d[] redReefRightBranches = populateRedReefRightBranches();
      Pose2d[] redReefLeftBranches = populateRedReefLeftBranches();

      Pose2d[] blueCenterFaces = FieldConstants.Reef.centerFaces;
      for (int i = 0; i < 6; i++) {
        allReefCenterFaces[i] = blueCenterFaces[i];
        allReefCenterFaces[i + 6] = FlippingUtil.flipFieldPose(blueCenterFaces[i]);
      }

      // FIXME: make this more efficient / coherent
      // put all the right and left poses into their maps, corresponded by approximate center poses

      // right
      // HARDCODE REMOVE ALGAE POSES TO THE MIDDLE FOR NOW:
      // FIXME: unhardcode remove algae poses
      for (int i = 0; i < 6; i++) {
        rightReefPoses.put(allReefCenterFaces[i], blueReefRightBranches[i]);
        Pose2d removeAlgaePose =
            allReefCenterFaces[i].transformBy(
                new Transform2d(
                    RobotConfig.getInstance().getRobotLengthWithBumpers().in(Meters) / 2.0,
                    -Units.inchesToMeters(PIPE_FROM_REEF_CENTER_INCHES - 3.0),
                    Rotation2d.fromDegrees(180)));
        removeAlgaePoses.put(allReefCenterFaces[i], removeAlgaePose);
      }
      for (int i = 0; i < 6; i++) {
        rightReefPoses.put(allReefCenterFaces[i + 6], redReefRightBranches[i]);
        Pose2d removeAlgaePose =
            allReefCenterFaces[i + 6].transformBy(
                new Transform2d(
                    RobotConfig.getInstance().getRobotLengthWithBumpers().in(Meters) / 2.0,
                    -Units.inchesToMeters(PIPE_FROM_REEF_CENTER_INCHES - 3.0),
                    Rotation2d.fromDegrees(180)));
        removeAlgaePoses.put(allReefCenterFaces[i + 6], removeAlgaePose);
      }

      // left
      for (int i = 0; i < 6; i++) {
        leftReefPoses.put(allReefCenterFaces[i], blueReefLeftBranches[i]);
      }
      for (int i = 0; i < 6; i++) {
        leftReefPoses.put(allReefCenterFaces[i + 6], redReefLeftBranches[i]);
      }

    } else {
      // Populate pose maps with default, non-field calibrated values (for if we ever have an
      // inaccurate field and don't have time to calibrate)

      // get each transformed pose on the reef (center of the hexagonal side)
      // add left or right offset (y) as well as bumper offset (x)
      Pose2d[] reefCenterFaces = FieldConstants.Reef.centerFaces;
      for (Pose2d reefCenterFace : reefCenterFaces) {
        Pose2d leftPose =
            reefCenterFace.transformBy(
                new Transform2d(
                    RobotConfig.getInstance().getRobotLengthWithBumpers().in(Meters) / 2.0,
                    -Units.inchesToMeters(PIPE_FROM_REEF_CENTER_INCHES),
                    Rotation2d.fromDegrees(180)));
        Pose2d rightPose =
            reefCenterFace.transformBy(
                new Transform2d(
                    RobotConfig.getInstance().getRobotLengthWithBumpers().in(Meters) / 2.0,
                    Units.inchesToMeters(PIPE_FROM_REEF_CENTER_INCHES),
                    Rotation2d.fromDegrees(180)));
        Pose2d removeAlgaePose =
            reefCenterFace.transformBy(
                new Transform2d(
                    RobotConfig.getInstance().getRobotLengthWithBumpers().in(Meters) / 2.0,
                    -Units.inchesToMeters(PIPE_FROM_REEF_CENTER_INCHES - 3.0),
                    Rotation2d.fromDegrees(180)));

        leftReefPoses.put(reefCenterFace, leftPose);
        rightReefPoses.put(reefCenterFace, rightPose);
        removeAlgaePoses.put(reefCenterFace, removeAlgaePose);
      }
    }
  }

  @SuppressWarnings("unused")
  public Pose2d getNearestBranch(Side side) {
    Pose2d pose = RobotOdometry.getInstance().getEstimatedPose();

    Pose2d nearestReefCenterFace;
    if (COMPETITION_FIELD) {
      nearestReefCenterFace = pose.nearest(Arrays.asList(allReefCenterFaces));
    } else {
      // If we are on the red alliance, flip the current pose to the blue alliance to find the
      // nearest reef face. We will then flip back to the red alliance.
      // ONLY IF NOT USING COMPETITION FIELD
      if (getAlliance() == Alliance.Red) {
        pose = FlippingUtil.flipFieldPose(pose);
      }
      nearestReefCenterFace = pose.nearest(Arrays.asList(FieldConstants.Reef.centerFaces));
    }

    Pose2d bumpersOnReefAlignedToBranch;
    if (side == Side.LEFT) {
      bumpersOnReefAlignedToBranch = leftReefPoses.get(nearestReefCenterFace);
    } else if (side == Side.RIGHT) {
      bumpersOnReefAlignedToBranch = rightReefPoses.get(nearestReefCenterFace);
    } else {
      bumpersOnReefAlignedToBranch = removeAlgaePoses.get(nearestReefCenterFace);
    }

    // If we are on the red alliance, we have flipped the current pose to the blue alliance and
    // have found the nearest reef face on the blue alliance side. We now need to flip the pose for
    // that reef face back to the red alliance.

    if (!COMPETITION_FIELD && getAlliance() == Alliance.Red) {
      bumpersOnReefAlignedToBranch = FlippingUtil.flipFieldPose(bumpersOnReefAlignedToBranch);
    }

    return bumpersOnReefAlignedToBranch;
  }

  /*
   * These methods are for manually populating reef branch pose maps based on the measured robot pose in the correct scoring position.
   */
  private Pose2d[] populateBlueReefRightBranches() {
    Pose2d[] blueReefRightBranches = new Pose2d[6];
    // ORDER (clockwise): B, L, J, H, F, D
    blueReefRightBranches[0] =
        new Pose2d(
            3.228048879993923, 3.8643000772630605, Rotation2d.fromDegrees(0.7999547222254184));
    blueReefRightBranches[1] =
        new Pose2d(
            3.737009433677461, 5.047898467596413, Rotation2d.fromDegrees(-60.21431492159178));
    blueReefRightBranches[2] =
        new Pose2d(
            4.98060636803378, 5.200582300272024, Rotation2d.fromDegrees(-118.93184924091054));
    blueReefRightBranches[3] =
        new Pose2d(
            5.752773078265522, 4.184127098853673, Rotation2d.fromDegrees(-178.6494262725325));
    blueReefRightBranches[4] =
        new Pose2d(
            5.25999564889462, 3.0174300531873444, Rotation2d.fromDegrees(122.21138149309358));
    blueReefRightBranches[5] =
        new Pose2d(
            3.9721066115367143, 2.865513585855743, Rotation2d.fromDegrees(59.70930286825839));

    return blueReefRightBranches;
  }

  private Pose2d[] populateBlueReefLeftBranches() {
    Pose2d[] blueReefLeftBranches = new Pose2d[6];
    // ORDER (clockwise): A, K, I, G, E, C
    blueReefLeftBranches[0] =
        new Pose2d(
            3.2246574302099544, 4.2028565860716265, Rotation2d.fromDegrees(-0.21597423869477203));
    blueReefLeftBranches[1] =
        new Pose2d(
            4.029247001342495, 5.222526388520616, Rotation2d.fromDegrees(-60.74455044893526));
    blueReefLeftBranches[2] =
        new Pose2d(
            5.261139946496552, 5.035092764563711, Rotation2d.fromDegrees(-119.40435420966065));
    blueReefLeftBranches[3] =
        new Pose2d(
            5.751222634546609, 3.876354231340153, Rotation2d.fromDegrees(-178.9844811787821));
    blueReefLeftBranches[4] =
        new Pose2d(
            4.971456188992748, 2.8261406625586996, Rotation2d.fromDegrees(121.2141054970013));
    blueReefLeftBranches[5] =
        new Pose2d(3.671385976123253, 3.037272511721054, Rotation2d.fromDegrees(60.07464648544542));

    return blueReefLeftBranches;
  }

  private Pose2d[] populateRedReefRightBranches() {
    Pose2d[] redReefRightBranches = new Pose2d[6];

    // TEMPORARY HARDCODED TO NOT MESS WITH BLUE ALLIANCE POSES:
    // Normally, these will be field calibrated, but we don't have a red alliance on our home field.
    Pose2d[] blueReefRightBranches = new Pose2d[6];
    // ORDER (clockwise): B, L, J, H, F, D
    blueReefRightBranches[0] =
        new Pose2d(
            3.228048879993923, 3.8643000772630605, Rotation2d.fromDegrees(0.7999547222254184));
    blueReefRightBranches[1] =
        new Pose2d(
            3.737009433677461, 5.047898467596413, Rotation2d.fromDegrees(-60.21431492159178));
    blueReefRightBranches[2] =
        new Pose2d(
            4.98060636803378, 5.200582300272024, Rotation2d.fromDegrees(-118.93184924091054));
    blueReefRightBranches[3] =
        new Pose2d(
            5.752773078265522, 4.184127098853673, Rotation2d.fromDegrees(-178.6494262725325));
    blueReefRightBranches[4] =
        new Pose2d(
            5.25999564889462, 3.0174300531873444, Rotation2d.fromDegrees(122.21138149309358));
    blueReefRightBranches[5] =
        new Pose2d(
            3.9721066115367143, 2.865513585855743, Rotation2d.fromDegrees(59.70930286825839));

    redReefRightBranches[0] = FlippingUtil.flipFieldPose(blueReefRightBranches[0]);
    redReefRightBranches[1] = FlippingUtil.flipFieldPose(blueReefRightBranches[1]);
    redReefRightBranches[2] = FlippingUtil.flipFieldPose(blueReefRightBranches[2]);
    redReefRightBranches[3] = FlippingUtil.flipFieldPose(blueReefRightBranches[3]);
    redReefRightBranches[4] = FlippingUtil.flipFieldPose(blueReefRightBranches[4]);
    redReefRightBranches[5] = FlippingUtil.flipFieldPose(blueReefRightBranches[5]);

    return redReefRightBranches;
  }

  private Pose2d[] populateRedReefLeftBranches() {
    Pose2d[] redReefLeftBranches = new Pose2d[6];

    Pose2d[] blueReefLeftBranches = new Pose2d[6];
    // ORDER (clockwise): A, K, I, G, E, C
    blueReefLeftBranches[0] =
        new Pose2d(
            3.2246574302099544, 4.2028565860716265, Rotation2d.fromDegrees(-0.21597423869477203));
    blueReefLeftBranches[1] =
        new Pose2d(
            4.029247001342495, 5.222526388520616, Rotation2d.fromDegrees(-60.74455044893526));
    blueReefLeftBranches[2] =
        new Pose2d(
            5.261139946496552, 5.035092764563711, Rotation2d.fromDegrees(-119.40435420966065));
    blueReefLeftBranches[3] =
        new Pose2d(
            5.751222634546609, 3.876354231340153, Rotation2d.fromDegrees(-178.9844811787821));
    blueReefLeftBranches[4] =
        new Pose2d(
            4.971456188992748, 2.8261406625586996, Rotation2d.fromDegrees(121.2141054970013));
    blueReefLeftBranches[5] =
        new Pose2d(3.671385976123253, 3.037272511721054, Rotation2d.fromDegrees(60.07464648544542));

    redReefLeftBranches[0] = FlippingUtil.flipFieldPose(blueReefLeftBranches[0]);
    redReefLeftBranches[1] = FlippingUtil.flipFieldPose(blueReefLeftBranches[1]);
    redReefLeftBranches[2] = FlippingUtil.flipFieldPose(blueReefLeftBranches[2]);
    redReefLeftBranches[3] = FlippingUtil.flipFieldPose(blueReefLeftBranches[3]);
    redReefLeftBranches[4] = FlippingUtil.flipFieldPose(blueReefLeftBranches[4]);
    redReefLeftBranches[5] = FlippingUtil.flipFieldPose(blueReefLeftBranches[5]);

    return redReefLeftBranches;
  }

  public enum Side {
    LEFT,
    RIGHT,
    REMOVE_ALGAE
  }
}
