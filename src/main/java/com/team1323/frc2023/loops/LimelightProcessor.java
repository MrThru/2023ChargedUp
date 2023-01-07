package com.team1323.frc2023.loops;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import com.team1323.frc2023.Constants;
import com.team1323.frc2023.RobotState;
import com.team1323.frc2023.vision.TargetInfo;
import com.team1323.lib.util.Util;
import com.team254.lib.geometry.Translation2d;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightProcessor implements Loop{
	static LimelightProcessor instance = new LimelightProcessor();
	edu.wpi.first.networktables.NetworkTable table;
	RobotState robotState = RobotState.getInstance();
	NetworkTableEntry ledMode;
	NetworkTableEntry pipeline;
	NetworkTableEntry camMode;
	NetworkTableEntry stream;
	public List<NetworkTableEntry> target;
	public NetworkTableEntry cornerX, cornerY;
	private List<TargetInfo> mTargets = new ArrayList<>();
	private double[] mZeroArray = new double[]{0, 0, 0, 0, 0, 0, 0, 0};
	private boolean mSeesTarget = false;

	boolean updatesAllowed = true;
	public void enableUpdates(boolean enable){
		updatesAllowed = enable;
	}
	
	public static LimelightProcessor getInstance(){
		return instance;
	}
	
	public LimelightProcessor(){
	}
	
	@Override 
	public void onStart(double timestamp){
		table = NetworkTableInstance.getDefault().getTable("limelight");
		ledMode = table.getEntry("ledMode");
		pipeline = table.getEntry("pipeline");
		camMode = table.getEntry("camMode");
		stream = table.getEntry("stream");
		setStreamMode(0);//0 2
		target = Arrays.asList(table.getEntry("tx"), table.getEntry("ty"),
			table.getEntry("ta"), table.getEntry("tv"));
		cornerX = table.getEntry("tcornx");
		cornerY = table.getEntry("tcorny");
	}
	
	@Override 
	public void onLoop(double timestamp){
		List<TargetInfo> targets = new ArrayList<TargetInfo>();
		if(seesTarget() && updatesAllowed){
			targets.add(new TargetInfo(Math.tan(Math.toRadians(target.get(0).getDouble(0))), Math.tan(Math.toRadians(target.get(1).getDouble(0)))));
			/*List<TargetInfo> corners = getTarget();
			if (corners != null) {
				getTarget().forEach((t) -> targets.add(t));
			} else {
				targets.clear();
			}*/
		}

		robotState.addVisionUpdate(timestamp - (table.getEntry("tl").getDouble(0.0) / 1000.0) - Constants.kImageCaptureLatency, targets);		
	}
	
	@Override
	public void onStop(double timestamp){
		
	}
	
	public void blink(){
		if(ledMode.getDouble(0) != 2)
			ledMode.setNumber(2);
	}
	
	public void ledOn(boolean on){
		if(ledMode.getDouble(1) != 0 && on)
			ledMode.setNumber(0);
		else if(ledMode.getDouble(0) != 1 && !on)
			ledMode.setNumber(1);
	}
	
	public void setDriverMode(){
		camMode.setNumber(1);
	}
	
	public void setVisionMode(){
		camMode.setNumber(0);
	}

	public void setStreamMode(int id){
		stream.setNumber(id);
	}
	
	public void setPipeline(int id){
		pipeline.setNumber(id);
	}

	public void setPipeline(Pipeline p){
		setPipeline(p.id);
		System.out.println("Pipeline set to " + p.id);
	}

	public enum Pipeline{
		CLOSE_SHOT(0);

		int id;
		private Pipeline(int id){
			this.id = id;
		}
	}

	private boolean seesTarget(){
		boolean targetInSight = (target.get(3).getDouble(0) == 1.0) ? true : false;
		return targetInSight;
	}

	public TargetInfo getTargetInfo(double nx, double ny){
		double vpw = 2.0 * Math.tan(Math.toRadians(29.8));// 27.0 29.8
		double vph = 2.0 * Math.tan(Math.toRadians(24.85));//24.85  20.5
		double x = vpw / 2.0 * nx;
		double y = vph / 2.0 * ny;
		double ax = Math.atan2(x, 1.0);
		double ay = Math.atan2(y, 1.0);

		return new TargetInfo(Math.tan(ax), Math.tan(ay));
	}

	public TargetInfo getTargetInfo(List<NetworkTableEntry> target){
		double nx = target.get(0).getDouble(0);
		double ny = target.get(1).getDouble(0);
		double vpw = 2.0 * Math.tan(Math.toRadians(29.8));// 27.0 29.8
		double vph = 2.0 * Math.tan(Math.toRadians(24.85));//24.85  20.5
		double x = vpw / 2.0 * nx;
		double y = vph / 2.0 * ny;
		double ax = Math.atan2(x, 1.0);
		double ay = Math.atan2(y, 1.0);

		return new TargetInfo(Math.tan(ax), Math.tan(ay));
	}

	/**
     * @return two targets that make up one hatch/port or null if less than two targets are found
     */
    public synchronized List<TargetInfo> getTarget() {
        List<TargetInfo> targets = getRawTargetInfos();
        if (seesTarget() && targets != null) {
            return targets;
        }

        return null;
    }

    private synchronized List<TargetInfo> getRawTargetInfos() {
        List<double[]> corners = getTopCorners();
        if (corners == null) {
            return null;
        }

        double slope = 1.0;
        if (Math.abs(corners.get(1)[0] - corners.get(0)[0]) > Util.kEpsilon) {
            slope = (corners.get(1)[1] - corners.get(0)[1]) /
                    (corners.get(1)[0] - corners.get(0)[0]);
        }

		mTargets.clear();
		
		// Average of y and z;
		/*double y_pixels = (corners.get(0)[0] + corners.get(1)[0]) / 2.0;
		double z_pixels = (corners.get(0)[1] + corners.get(1)[1]) / 2.0;

		// Redefine to robot frame of reference.
		double nY = ((y_pixels - 160.0) / 160.0);
		double nZ = -((z_pixels - 120.0) / 120.0);

		double y = Constants.kVPW / 2 * nY;
		double z = Constants.kVPH / 2 * nZ;

		TargetInfo target = new TargetInfo(y, z);
		target.setSkew(slope);
		mTargets.add(target);*/

		for (int i = 0; i < 2; i++) {
			double y_pixels = corners.get(i)[0];
			double z_pixels = corners.get(i)[1];

			// Redefine to robot frame of reference.
			double nY = ((y_pixels - 160.0) / 160.0);
			double nZ = -((z_pixels - 120.0) / 120.0);

			double y = Constants.kVPW / 2 * nY;
			double z = Constants.kVPH / 2 * nZ;

			TargetInfo target = new TargetInfo(y, z);
			target.setSkew(slope);
			mTargets.add(target);
		}

        return mTargets;
    }

    /**
     * Returns raw top-left and top-right corners
     *
     * @return list of corners: index 0 - top left, index 1 - top right
     */
    private List<double[]> getTopCorners() {
        double[] xyCorners = table.getEntry("tcornxy").getDoubleArray(mZeroArray);
        mSeesTarget = table.getEntry("tv").getDouble(0) == 1.0;

        // something went wrong
        if (!mSeesTarget ||
                Arrays.equals(xyCorners, mZeroArray)
                || xyCorners.length < 8) {
            return null;
        }

		double[] xCorners = new double[]{ xyCorners[0], xyCorners[2], xyCorners[4], xyCorners[6] };
        double[] yCorners = new double[]{ xyCorners[1], xyCorners[3], xyCorners[5], xyCorners[7] };
        return extractTopCornersFromBoundingBoxes(xCorners, yCorners);
    }

    private static final Comparator<Translation2d> xSort = Comparator.comparingDouble(Translation2d::x);
    private static final Comparator<Translation2d> ySort = Comparator.comparingDouble(Translation2d::y);

    /**
     * Returns raw top-left and top-right corners
     *
     * @return list of corners: index 0 - top left, index 1 - top right
     */
    public static List<double[]> extractTopCornersFromBoundingBoxes(double[] xCorners, double[] yCorners) {
        List<Translation2d> corners = new ArrayList<>();
        for (int i = 0; i < xCorners.length; i++) {
            corners.add(new Translation2d(xCorners[i], yCorners[i]));
        }

        corners.sort(xSort);

        List<Translation2d> left = corners.subList(0, 2);
        List<Translation2d> right = corners.subList(2, 4);

        left.sort(ySort);
        right.sort(ySort);

        Translation2d leftCorner = left.get(0);
        Translation2d rightCorner = right.get(0);

        return List.of(new double[]{leftCorner.x(), leftCorner.y()}, new double[]{rightCorner.x(), rightCorner.y()});
    }
}
