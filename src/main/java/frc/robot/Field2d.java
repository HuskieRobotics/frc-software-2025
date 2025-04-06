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
import frc.robot.operator_interface.OISelector;
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
  private Region2d reefZone;

  private Alliance alliance = DriverStation.Alliance.Blue;

  private Map<Pose2d, Pose2d> leftReefPoses = new HashMap<Pose2d, Pose2d>();
  private Map<Pose2d, Pose2d> rightReefPoses = new HashMap<Pose2d, Pose2d>();
  private Map<Pose2d, Pose2d> removeAlgaePoses = new HashMap<Pose2d, Pose2d>();
  private Pose2d[] allReefCenterFaces = new Pose2d[12];

  private Pose2d[] processors = new Pose2d[2];
  private Pose2d[] coralStations = new Pose2d[4];

  private static final Pose2d CENTER_BARGE_POSE =
      new Pose2d(
          new Translation2d(Units.inchesToMeters(305), Units.inchesToMeters(242.855)),
          Rotation2d.fromDegrees(0.0));

  private static final Pose2d RIGHT_BARGE_POSE =
      new Pose2d(
          new Translation2d(Units.inchesToMeters(305), Units.inchesToMeters(206.855)),
          Rotation2d.fromDegrees(0.0));

  private final boolean COMPETITION_FIELD =
      true; // set TRUE if home field calibration or at competition

  private static final double PIPE_FROM_REEF_CENTER_INCHES =
      6.469; // taken from FieldConstants adjustY for reef y offset

  private static final double REMOVE_ALGAE_Y_TRANSFORMATION_INCHES = -5;

  public class AlgaePosition {
    public Pose2d pose;
    public boolean isHigh;

    public AlgaePosition(Pose2d pose, boolean isHigh) {
      this.pose = pose;
      this.isHigh = isHigh;
    }
  }

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

  public void populateReefZone() {
    // make a region of the reef center faces transformed by 18 inches
    // the reef zone is 14 inches from the reef
    // however, a few inches are added for safety as well as the region measuring to the center of
    // the robot
    Translation2d[] transformedCenterFaces = new Translation2d[6];
    for (int i = 0; i < 6; i++) {
      Pose2d centerFace =
          FieldConstants.Reef.centerFaces[i].transformBy(
              new Transform2d(Units.inchesToMeters(36.0), 0.0, Rotation2d.fromDegrees(0.0)));
      transformedCenterFaces[i] = new Translation2d(centerFace.getX(), centerFace.getY());
    }
    this.reefZone = new Region2d(transformedCenterFaces);
  }

  public void populateStationsAndProcessors() {
    processors[0] = FieldConstants.Processor.centerFace;
    processors[1] = FlippingUtil.flipFieldPose(processors[0]);

    coralStations[0] = FieldConstants.CoralStation.leftCenterFace;
    coralStations[1] = FieldConstants.CoralStation.rightCenterFace;
    coralStations[2] = FlippingUtil.flipFieldPose(coralStations[0]);
    coralStations[3] = FlippingUtil.flipFieldPose(coralStations[1]);
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

      // remove algae poses are hardcoded to slightly off the center
      // blue alliance poses
      for (int i = 0; i < 6; i++) {
        rightReefPoses.put(allReefCenterFaces[i], blueReefRightBranches[i]);
        leftReefPoses.put(allReefCenterFaces[i], blueReefLeftBranches[i]);
        Pose2d removeAlgaePose =
            allReefCenterFaces[i].transformBy(
                new Transform2d(
                    RobotConfig.getInstance().getRobotLengthWithBumpers().in(Meters) / 2.0,
                    Units.inchesToMeters(REMOVE_ALGAE_Y_TRANSFORMATION_INCHES),
                    Rotation2d.fromDegrees(180)));
        removeAlgaePoses.put(allReefCenterFaces[i], removeAlgaePose);
      }

      // red alliance poses
      for (int i = 0; i < 6; i++) {
        rightReefPoses.put(allReefCenterFaces[i + 6], redReefRightBranches[i]);
        leftReefPoses.put(allReefCenterFaces[i + 6], redReefLeftBranches[i]);
        Pose2d removeAlgaePose =
            allReefCenterFaces[i + 6].transformBy(
                new Transform2d(
                    RobotConfig.getInstance().getRobotLengthWithBumpers().in(Meters) / 2.0,
                    Units.inchesToMeters(REMOVE_ALGAE_Y_TRANSFORMATION_INCHES),
                    Rotation2d.fromDegrees(180)));
        removeAlgaePoses.put(allReefCenterFaces[i + 6], removeAlgaePose);
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
                    -Units.inchesToMeters(REMOVE_ALGAE_Y_TRANSFORMATION_INCHES),
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

  public Pose2d getSelectedBranch() {
    int offset = 0;
    if (getAlliance() == Alliance.Red) {
      offset = 6;
    }

    if (OISelector.getOperatorInterface().getReefBranchATrigger().getAsBoolean()) {
      return leftReefPoses.get(allReefCenterFaces[offset]);
    } else if (OISelector.getOperatorInterface().getReefBranchBTrigger().getAsBoolean()) {
      return rightReefPoses.get(allReefCenterFaces[offset]);
    } else if (OISelector.getOperatorInterface().getReefBranchCTrigger().getAsBoolean()) {
      return leftReefPoses.get(allReefCenterFaces[offset + 5]);
    } else if (OISelector.getOperatorInterface().getReefBranchDTrigger().getAsBoolean()) {
      return rightReefPoses.get(allReefCenterFaces[offset + 5]);
    } else if (OISelector.getOperatorInterface().getReefBranchETrigger().getAsBoolean()) {
      return leftReefPoses.get(allReefCenterFaces[offset + 4]);
    } else if (OISelector.getOperatorInterface().getReefBranchFTrigger().getAsBoolean()) {
      return rightReefPoses.get(allReefCenterFaces[offset + 4]);
    } else if (OISelector.getOperatorInterface().getReefBranchGTrigger().getAsBoolean()) {
      return leftReefPoses.get(allReefCenterFaces[offset + 3]);
    } else if (OISelector.getOperatorInterface().getReefBranchHTrigger().getAsBoolean()) {
      return rightReefPoses.get(allReefCenterFaces[offset + 3]);
    } else if (OISelector.getOperatorInterface().getReefBranchITrigger().getAsBoolean()) {
      return leftReefPoses.get(allReefCenterFaces[offset + 2]);
    } else if (OISelector.getOperatorInterface().getReefBranchJTrigger().getAsBoolean()) {
      return rightReefPoses.get(allReefCenterFaces[offset + 2]);
    } else if (OISelector.getOperatorInterface().getReefBranchKTrigger().getAsBoolean()) {
      return leftReefPoses.get(allReefCenterFaces[offset + 1]);
    } else if (OISelector.getOperatorInterface().getReefBranchLTrigger().getAsBoolean()) {
      return rightReefPoses.get(allReefCenterFaces[offset + 1]);
    }

    // go to the nearest center face if nothing is selected
    Pose2d pose = RobotOdometry.getInstance().getEstimatedPose();
    return pose.nearest(Arrays.asList(FieldConstants.Reef.centerFaces));
  }

  public AlgaePosition getNearestAlgae() {
    Pose2d pose = RobotOdometry.getInstance().getEstimatedPose();
    boolean isHighAlgae = false;
    int offset = 0;
    if (getAlliance() == Alliance.Red) {
      offset = 6;
    }

    // high: A/B , E/F, I/J
    // low: C/D, G/H, K/L
    Pose2d nearestCenterFace = pose.nearest(Arrays.asList(allReefCenterFaces));
    if (nearestCenterFace == allReefCenterFaces[offset + 0]
        || nearestCenterFace == allReefCenterFaces[offset + 2]
        || nearestCenterFace == allReefCenterFaces[offset + 4]) {
      // high algae
      isHighAlgae = true;
    } else if (nearestCenterFace == allReefCenterFaces[offset + 1]
        || nearestCenterFace == allReefCenterFaces[offset + 3]
        || nearestCenterFace == allReefCenterFaces[offset + 5]) {
      // low algae
      isHighAlgae = false;
    }

    return new AlgaePosition(removeAlgaePoses.get(nearestCenterFace), isHighAlgae);
  }

  public Pose2d getCenterBargePose() {
    // x arbitrary from 20 inches x from the middle cage

    if (getAlliance() == Alliance.Red) {
      return FlippingUtil.flipFieldPose(CENTER_BARGE_POSE);
    }
    return CENTER_BARGE_POSE;
  }

  public Pose2d getRightBargePose() {
    if (getAlliance() == Alliance.Red) {
      return FlippingUtil.flipFieldPose(RIGHT_BARGE_POSE);
    }
    return RIGHT_BARGE_POSE;
  }

  public boolean isShortOfBarge() {
    Pose2d pose = RobotOdometry.getInstance().getEstimatedPose();
    Transform2d robotRelativeDifference = new Transform2d(pose, getCenterBargePose());

    // 12 inch buffer, lower if we need to
    return robotRelativeDifference.getX() > Units.inchesToMeters(12);
  }

  public boolean isFarFromBarge() {
    Pose2d pose = RobotOdometry.getInstance().getEstimatedPose();
    Transform2d robotRelativeDifference = new Transform2d(pose, getCenterBargePose());

    // do not absolute value this due to the chances of us also working with under the barge later
    // FIXME: tune to be closer possibly
    return robotRelativeDifference.getX() > Units.inchesToMeters(36);
  }

  public Pose2d getFourthAutoCoralPose(Side side) {
    int offset = 0;
    if (getAlliance() == Alliance.Red) {
      offset = 6;
    }

    if (side == Side.LEFT) {
      return leftReefPoses.get(allReefCenterFaces[offset]);
    } else {
      return rightReefPoses.get(allReefCenterFaces[offset]);
    }
  }

  public Pose2d getNearestProcessor() {
    Pose2d pose = RobotOdometry.getInstance().getEstimatedPose();

    Pose2d nearestProcessor = pose.nearest(Arrays.asList(processors));
    nearestProcessor =
        nearestProcessor.transformBy(
            new Transform2d(
                (RobotConfig.getInstance().getRobotLengthWithBumpers().in(Meters) / 2.0)
                    + Units.inchesToMeters(10.0), /* was 12 */
                Units.inchesToMeters(-4.5),
                Rotation2d.fromDegrees(180)));

    return nearestProcessor;
  }

  public Pose2d getNearestCoralStation() {
    Pose2d pose = RobotOdometry.getInstance().getEstimatedPose();

    Pose2d nearestCoralStation = pose.nearest(Arrays.asList(coralStations));
    nearestCoralStation =
        nearestCoralStation.transformBy(
            new Transform2d(
                (RobotConfig.getInstance().getRobotLengthWithBumpers().in(Meters) / 2.0),
                0,
                Rotation2d.fromDegrees(0)));

    return nearestCoralStation;
  }

  public Pose2d getLeftCoralStation() {
    int offset = getAlliance() == Alliance.Red ? 2 : 0;

    return coralStations[offset].transformBy(
        new Transform2d(
            (RobotConfig.getInstance().getRobotLengthWithBumpers().in(Meters) / 2.0),
            0,
            Rotation2d.fromDegrees(0)));
  }

  public Pose2d getRightCoralStation() {
    int offset = getAlliance() == Alliance.Red ? 3 : 1;

    return coralStations[offset].transformBy(
        new Transform2d(
            (RobotConfig.getInstance().getRobotLengthWithBumpers().in(Meters) / 2.0),
            0,
            Rotation2d.fromDegrees(0)));
  }

  /*
   * These methods are for manually populating reef branch pose maps based on the measured robot pose in the correct scoring position.
   */
  private Pose2d[] populateBlueReefRightBranches() {
    Pose2d[] blueReefRightBranches = new Pose2d[6];
    // ORDER (clockwise): B, L, J, H, F, D
    // B
    blueReefRightBranches[0] =
        new Pose2d(
            3.229818603562776, 3.8648756851682364, Rotation2d.fromDegrees(-0.17889520040049436));

    // L
    blueReefRightBranches[1] =
        new Pose2d(
            3.7176156282759605, 5.035380869555658, Rotation2d.fromDegrees(-60.10709497904928));

    // J
    blueReefRightBranches[2] =
        new Pose2d(
            4.966760203506612, 5.20700655083356, Rotation2d.fromDegrees(-120.50276965017787));

    // H
    blueReefRightBranches[3] =
        new Pose2d(
            5.751655098482316, 4.183732157427522, Rotation2d.fromDegrees(179.67501229375796));

    // F
    blueReefRightBranches[4] =
        new Pose2d(
            5.2563454537513445, 3.0141128709412994, Rotation2d.fromDegrees(119.39127568247405));

    // D
    blueReefRightBranches[5] =
        new Pose2d(4.014241609545086, 2.846973683493267, Rotation2d.fromDegrees(60.57340938572369));

    return blueReefRightBranches;
  }

  private Pose2d[] populateBlueReefLeftBranches() {
    Pose2d[] blueReefLeftBranches = new Pose2d[6];
    // ORDER (clockwise): A, K, I, G, E, C
    // A
    blueReefLeftBranches[0] =
        new Pose2d(
            3.230542336549387, 4.193007927492786, Rotation2d.fromDegrees(-0.10181025091822583));

    // K
    blueReefLeftBranches[1] =
        new Pose2d(
            4.010643260665861, 5.204141168931486, Rotation2d.fromDegrees(-59.92450083674441));

    // I
    blueReefLeftBranches[2] =
        new Pose2d(
            5.276753358736465, 5.0267056648963395, Rotation2d.fromDegrees(-119.95088757038484));

    // G
    blueReefLeftBranches[3] =
        new Pose2d(
            5.750908678104904, 3.8621098391612554, Rotation2d.fromDegrees(179.68958461210676));

    // E
    blueReefLeftBranches[4] =
        new Pose2d(4.967337814725686, 2.848124833495257, Rotation2d.fromDegrees(119.7932088156921));

    // C
    blueReefLeftBranches[5] =
        new Pose2d(3.7141747842348485, 3.019437777942879, Rotation2d.fromDegrees(60.4742856167002));

    return blueReefLeftBranches;
  }

  private Pose2d[] populateRedReefRightBranches() {
    Pose2d[] redReefRightBranches = new Pose2d[6];
    // ORDER (clockwise): B, L, J, H, F, D
    // B
    redReefRightBranches[0] =
        new Pose2d(
            14.320720415187344, 4.184033251040251, Rotation2d.fromDegrees(-179.5396778347839));

    // L
    redReefRightBranches[1] =
        new Pose2d(
            13.832900295444404, 3.0172264415239596, Rotation2d.fromDegrees(119.85502437679067));

    // J
    redReefRightBranches[2] =
        new Pose2d(
            12.571949778471065, 2.8488636363959317, Rotation2d.fromDegrees(60.29114158605732));

    // H
    redReefRightBranches[3] =
        new Pose2d(
            11.796071028860094, 3.867363990420004, Rotation2d.fromDegrees(-0.09948627549695879));

    // F
    redReefRightBranches[4] =
        new Pose2d(
            12.284519496572326, 5.036756443709353, Rotation2d.fromDegrees(-59.93637786402808));

    // D
    redReefRightBranches[5] =
        new Pose2d(
            13.55314624028025, 5.193721262283811, Rotation2d.fromDegrees(-120.24661569289086));

    return redReefRightBranches;
  }

  private Pose2d[] populateRedReefLeftBranches() {

    Pose2d[] redReefLeftBranches = new Pose2d[6];
    // ORDER (clockwise): A, K, I, G, E, C
    // A
    redReefLeftBranches[0] =
        new Pose2d(14.31979633815289, 3.856825038161692, Rotation2d.fromDegrees(179.8562374074041));

    // K
    redReefLeftBranches[1] =
        new Pose2d(
            13.538278124818714, 2.8473068940183475, Rotation2d.fromDegrees(120.00124833818488));

    // I
    redReefLeftBranches[2] =
        new Pose2d(
            12.268909270987784, 3.024518569894161, Rotation2d.fromDegrees(60.16963198950504));

    // G
    redReefLeftBranches[3] =
        new Pose2d(
            11.795148549502741, 4.198638905923342, Rotation2d.fromDegrees(-0.10766688992053104));

    // E
    redReefLeftBranches[4] =
        new Pose2d(
            12.574952391375794, 5.204771483613289, Rotation2d.fromDegrees(-59.815946018525125));

    // C
    redReefLeftBranches[5] =
        new Pose2d(
            13.838407776798107, 5.0317099908377445, Rotation2d.fromDegrees(-119.90855915364374));

    return redReefLeftBranches;
  }

  public boolean isOutsideOfReefZone() {
    Pose2d pose = RobotOdometry.getInstance().getEstimatedPose();
    if (getAlliance() == Alliance.Red) {
      pose = FlippingUtil.flipFieldPose(pose);
    }

    return !reefZone.contains(pose);
  }

  public enum Side {
    LEFT,
    RIGHT,
    REMOVE_ALGAE
  }
}
