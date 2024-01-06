package frc.robot.utilities;

import java.util.ArrayList;
import java.util.TreeMap;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

// AprilTag field in our coordinate system
public class RVAprilTagField {

    // See https://firstfrc.blob.core.windows.net/frc2023/FieldAssets/TeamVersions/AprilTags-UserGuideandImages.pdf
    // https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2023-chargedup.json
    public static final Rotation3d FACING_AWAY_FROM_RED_SIDE = new Rotation3d(new Quaternion(0, 0, 0, 1));
    public static final Rotation3d FACING_AWAY_FROM_BLUE_SIDE = new Rotation3d(new Quaternion(1, 0, 0, 0));

    public static ArrayList<AprilTag> tags = new ArrayList<>();
    public static final double FIELD_LENGTH = 16.54175;
    public static final double FIELD_WIDTH = 8.0137;

    public static final double PIECE_X = Units.inchesToMeters(48.0);
    public static final double LOW_PIECE_Y = Units.inchesToMeters(-122.0);
    public static final double LOW_MID_PIECE_Y = Units.inchesToMeters(-73.0);
    public static final double TOP_MID_PIECE_Y = Units.inchesToMeters(-25.0);
    public static final double TOP_PIECE_Y = Units.inchesToMeters(24.0);

    public static final double CHARGE_STATION_EDGE_X = Units.inchesToMeters(131.4);

    public static TreeMap<Double, Translation2d> pieceYMap = new TreeMap<>();

    static{
        double y = 0.975 - Units.inchesToMeters(2);
        for (int i = 0; i < 9; i++)
        {
            pieceYMap.put(y, new Translation2d(0, y));
            y -= Units.inchesToMeters(22);
        }
    }
    
    static {
        tags.add(new AprilTag(1, new Pose3d(new Translation3d(7.27, -2.9797, 0.462788), FACING_AWAY_FROM_RED_SIDE)));
        tags.add(new AprilTag(2, new Pose3d(new Translation3d(7.27, -1.3033, 0.462788), FACING_AWAY_FROM_RED_SIDE)));
        tags.add(new AprilTag(3, new Pose3d(new Translation3d(7.27, 0.3731, 0.462788), FACING_AWAY_FROM_RED_SIDE)));
        tags.add(new AprilTag(4, new Pose3d(new Translation3d(7.94, 2.6985, 0.695452), FACING_AWAY_FROM_RED_SIDE)));
        tags.add(new AprilTag(5, new Pose3d(new Translation3d(-7.88, 2.6985, 0.695452), FACING_AWAY_FROM_BLUE_SIDE)));
        tags.add(new AprilTag(6, new Pose3d(new Translation3d(-7.21, 0.3731, 0.462788), FACING_AWAY_FROM_BLUE_SIDE)));
        tags.add(new AprilTag(7, new Pose3d(new Translation3d(-7.21, -1.3033, 0.462788), FACING_AWAY_FROM_BLUE_SIDE)));
        tags.add(new AprilTag(8, new Pose3d(new Translation3d(-7.21, -2.9797, 0.462788), FACING_AWAY_FROM_BLUE_SIDE)));
    };

    public static AprilTagFieldLayout getFieldLayout()
    {
        return new AprilTagFieldLayout(tags, FIELD_LENGTH, FIELD_WIDTH);
    }
}
