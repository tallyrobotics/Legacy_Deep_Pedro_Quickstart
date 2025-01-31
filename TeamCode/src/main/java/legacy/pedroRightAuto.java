//package legacy;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.localization.Pose;
//import com.pedropathing.pathgen.BezierCurve;
//import com.pedropathing.pathgen.BezierLine;
//import com.pedropathing.pathgen.Path;
//import com.pedropathing.pathgen.PathChain;
//import com.pedropathing.pathgen.Point;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import pedroPathing.constants.FConstants;
//import pedroPathing.constants.LConstants;
//
//public class pedroRightAuto {
//
//    private final Pose startPose = new Pose(9, 111, Math.toRadians(270));  // Starting position
//    private final Pose scorePose = new Pose(14, 129, Math.toRadians(315)); // Scoring position
//
//    private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0)); // First sample pickup
//    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0)); // Second sample pickup
//    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Third sample pickup
//
//    private final Pose parkPose = new Pose(60, 98, Math.toRadians(90));    // Parking position
//    private final Pose parkControlPose = new Pose(60, 98, Math.toRadians(90)); // Control point for curved path
//
//
//}
//
//        private Path scorePreload, park;
//        private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;
//
//public void buildPaths() {
//    // Path for scoring preload
//    scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
//    scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
//
//    // Path chains for picking up and scoring samples
//    grabPickup1 = follower.pathBuilder()
//            .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
//            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
//            .build();
//
//    scorePickup1 = follower.pathBuilder()
//            .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
//            .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
//            .build();
//
//    grabPickup2 = follower.pathBuilder()
//            .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
//            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
//            .build();
//
//    scorePickup2 = follower.pathBuilder()
//            .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
//            .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
//            .build();
//
//    grabPickup3 = follower.pathBuilder()
//            .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
//            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
//            .build();
//
//    scorePickup3 = follower.pathBuilder()
//            .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
//            .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
//            .build();
//
//    // Curved path for parking
//    park = new Path(new BezierCurve(new Point(scorePose), new Point(parkControlPose), new Point(parkPose)));
//    park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
//}