package main;

import java.util.ArrayList;
import java.util.List;

import org.joml.Matrix3f;
import org.joml.Matrix4f;
import org.joml.Vector2f;
import org.joml.Vector3f;

public class RobotSpecs {
	
	private static float d2 = 0, d3 = 0, l2, l3;
	private static float COLLISION_RANGE = 1;
	private static Matrix4f CORRECTION = new Matrix4f().rotateXYZ((float)Math.toRadians(90), (float)Math.toRadians(180), 0);
	private static Vector3f previousEndPose = new Vector3f();
	private static Vector3f previousBasePose = new Vector3f();
	private static List<Vector3f> jointPoses = new ArrayList<>();
	
	public static void setD2(float d2) {
		RobotSpecs.d2 = d2;
	}

	public static void setD3(float d3) {
		RobotSpecs.d3 = d3;
	}

	public static void setL2(float l2) {
		RobotSpecs.l2 = l2;
	}

	public static void setL3(float l3) {
		RobotSpecs.l3 = l3;
	}
	
	public static float getL3() {
		return l3;
	}
	
	public static float getL2() {
		return l2;
	}
	
	public static float getCOLLISION_RANGE() {
		return COLLISION_RANGE;
	}

	public static void setJointPose(int index, Vector3f jointPose) {
		jointPoses.add(index, jointPose);
	}
	
	public static Vector3f getJointPose(int index) {
		return jointPoses.get(index);
	}
	
	public static List<Vector3f> getPosition(Vector3f theta, Vector3f base) {
		Matrix3f rotation1 = new Matrix3f().rotate(theta.x, new Vector3f(0, 1, 0));
		Matrix3f rotation2 = new Matrix3f().rotate(theta.y, new Vector3f(0, 0, 1));
		Matrix3f rotation3 = new Matrix3f().rotate(theta.z, new Vector3f(0, 0, 1));
		
		List<Vector3f> positions = new ArrayList<>();
		positions.add(getTransformedPosition(rotation1, jointPoses.get(0), base));
		rotation1.mul(rotation2, rotation2);
		positions.add(getTransformedPosition(rotation2, jointPoses.get(1), base));
		rotation2.mul(rotation3, rotation3);
		positions.add(getTransformedPosition(rotation3, jointPoses.get(2), base));
		return positions;
	}
	
	public static Vector3f getPreviousBasePose() {
		return previousBasePose;
	}

	public static void setPreviousBasePose(Vector3f previousBasePose) {
		RobotSpecs.previousBasePose = previousBasePose;
	}

	private static Vector3f getTransformedPosition(Matrix3f rotation, Vector3f point, Vector3f base) {
		Vector3f updatedPoint = new Vector3f();
		updatedPoint.x = (rotation.m00 * point.x) + (rotation.m10() * point.y) + (rotation.m20() * point.z) + base.x;
		updatedPoint.y = (rotation.m01 * point.x) + (rotation.m11() * point.y) + (rotation.m21() * point.z) + base.y;
		updatedPoint.z = (rotation.m02 * point.x) + (rotation.m12() * point.y) + (rotation.m22() * point.z) + base.z;
		return updatedPoint;
	}

	public static Vector3f getThetaSpecs(Vector3f position, Vector3f rotation) {
		Matrix4f endPose = new Matrix4f().rotateZYX((float)Math.toRadians(rotation.z), (float)Math.toRadians(rotation.y), (float)Math.toRadians(rotation.x)).setTranslation(position);
		endPose.getEulerAnglesZYX(rotation);
//		System.out.println(endPose);
//		System.out.println("Angles1: ("+Math.toDegrees(rotation.x)+" "+Math.toDegrees(rotation.y)+" "+Math.toDegrees(rotation.z)+")");
		CORRECTION.mul(endPose, endPose);
		rotation = new Vector3f();
		endPose.getEulerAnglesZYX(rotation);
//		System.out.println("Angles: ("+Math.toDegrees(rotation.x)+" "+Math.toDegrees(rotation.y)+" "+Math.toDegrees(rotation.z));
		float theta1 = (float) Math.toDegrees(Math.atan2(endPose.m31(), endPose.m30()));
//		System.out.println(theta23);
		float ctheta3 = (float) ((Math.pow(endPose.m30(), 2) + Math.pow(endPose.m31(), 2) + Math.pow(endPose.m32(), 2) - Math.pow(l3, 2) - Math.pow(l2, 2)) / (2 * l2 * l3));
//		System.out.println(endPose.m30() +"  "+ endPose.m31()+" "+endPose.m32() + " " + l2+" "+l3+" "+(l2+l3));
		if(Math.abs(ctheta3) > 1 || Float.isNaN(theta1))
			return previousEndPose;
		float stheta3 = (float) Math.sqrt(1 - Math.pow(ctheta3, 2));
		float theta3_1 = (float) Math.toDegrees(Math.atan2(stheta3, ctheta3));
		float theta3_2 = (float) Math.toDegrees(Math.atan2(-stheta3, ctheta3));
		float a = ctheta3 * RobotSpecs.getL3(), b = stheta3 * RobotSpecs.getL3();
		float k = (float) (Math.sqrt(Math.pow(endPose.m30(), 2) + Math.pow(endPose.m31(), 2)) / endPose.m32());
		float theta2_1 = (float) Math.atan2((k * (a + RobotSpecs.getL2())) - (RobotSpecs.getL2() + b), a + (k * b));
		float theta2_2 = (float) Math.atan2((k * (a + RobotSpecs.getL2())) - (RobotSpecs.getL2() - b), a - (k * b));
		Vector3f thetas1 = new Vector3f(theta1, theta2_1, theta3_1);
		Vector3f thetas2 = new Vector3f(theta1, theta2_2, theta3_2);
//		System.out.println(thetas1);
//		System.out.println(thetas2);
		previousEndPose = Float.compare(distance(thetas1), distance(thetas2)) <= 0 ? thetas1 : thetas2;
//		System.out.println(previousEndPose);
		return previousEndPose;
	}
	
	private static float distance(Vector3f vector1) {
		float x = (previousEndPose.x - vector1.x) * (previousEndPose.x - vector1.x);
		float y = (previousEndPose.y - vector1.y) * (previousEndPose.y - vector1.y);
		float z = (previousEndPose.z - vector1.z) * (previousEndPose.z - vector1.z);
		return (float) Math.sqrt(x+y+z);
	}

	public static Vector3f getPreviousEndPose() {
		return previousEndPose;
	}

	public static void setPreviousEndPose(Vector3f previousEndPose) {
		RobotSpecs.previousEndPose = previousEndPose;
	}
	
}
