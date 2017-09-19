package main;

import org.joml.Matrix4f;
import org.joml.Vector3f;
import org.lwjgl.glfw.GLFW;
import org.lwjgl.glfw.GLFWScrollCallbackI;

import scene.ICamera;
import utils.DisplayManager;
import utils.SmoothFloat;

/**
 * Represents the in-game camera. This class is in charge of keeping the
 * projection-view-matrix updated. It allows the user to alter the pitch and yaw
 * with the left mouse button.
 * 
 * @author Karl
 *
 */
public class Camera implements ICamera, GLFWScrollCallbackI {

	private static final float PITCH_SENSITIVITY = 1.5f;
	private static final float YAW_SENSITIVITY = 2.5f;
	private static final float MAX_PITCH = 90;
	private static final float MAX_DISTANCE = 100f;
	private static final float DISTANCE_UPDATE = 0.05f;

	private static final float FOV = 70;
	private static final float NEAR_PLANE = 0.2f;
	private static final float FAR_PLANE = 400;

	private static final float Y_OFFSET = 5;

	private Matrix4f projectionMatrix;
	private Matrix4f viewMatrix = new Matrix4f();

	private Vector3f position = new Vector3f(0, 0, 0);

	private float yaw = 0;
	private SmoothFloat pitch = new SmoothFloat(0, 0);
	private SmoothFloat angleAroundPlayer = new SmoothFloat(0, 10);
	private SmoothFloat distanceFromPlayer = new SmoothFloat(30, 10);

	public Camera() {
		this.projectionMatrix = createProjectionMatrix();
		GLFW.glfwSetScrollCallback(DisplayManager.getWindow(), this);
	}

	@Override
	public void move() {
		calculateZoom();
		float horizontalDistance = calculateHorizontalDistance();
		float verticalDistance = calculateVerticalDistance();
		calculateCameraPosition(horizontalDistance, verticalDistance);
		this.yaw = 360 - angleAroundPlayer.get();
		yaw %= 360;
		updateViewMatrix();
	}

	@Override
	public Matrix4f getViewMatrix() {
		return viewMatrix;
	}

	@Override
	public Matrix4f getProjectionMatrix() {
		return projectionMatrix;
	}

	@Override
	public Matrix4f getProjectionViewMatrix() {
//		return Matrix4f.mul(projectionMatrix, viewMatrix, null);
		return new Matrix4f().mul(projectionMatrix).mul(viewMatrix);
	}

	private void calculateZoom() {
		if(GLFW.glfwGetKey(DisplayManager.getWindow(), GLFW.GLFW_KEY_Z) == GLFW.GLFW_PRESS)
			distanceFromPlayer.increaseTarget(-DISTANCE_UPDATE);
		else if(GLFW.glfwGetKey(DisplayManager.getWindow(), GLFW.GLFW_KEY_X) == GLFW.GLFW_PRESS)
			distanceFromPlayer.increaseTarget(DISTANCE_UPDATE);
		clampDistance();
		distanceFromPlayer.update(DisplayManager.getFrameTime());
	}
	
	private void updateViewMatrix() {
		viewMatrix.identity();
		viewMatrix.rotate((float) Math.toRadians(pitch.get()), new Vector3f(1, 0, 0), viewMatrix);
		viewMatrix.rotate((float) Math.toRadians(yaw), new Vector3f(0, 1, 0), viewMatrix);
		Vector3f negativeCameraPos = new Vector3f(-position.x, -position.y, -position.z);
		viewMatrix.translate(negativeCameraPos, viewMatrix);
	}

	private static Matrix4f createProjectionMatrix() {
		Matrix4f projectionMatrix = new Matrix4f();
		float aspectRatio = (float) DisplayManager.getWidth() / (float) DisplayManager.getHeight();
		float y_scale = (float) ((1f / Math.tan(Math.toRadians(FOV / 2f))));
		float x_scale = y_scale / aspectRatio;
		float frustum_length = FAR_PLANE - NEAR_PLANE;
		projectionMatrix.m00(x_scale);
		projectionMatrix.m11(y_scale);
		projectionMatrix.m22(-((FAR_PLANE + NEAR_PLANE) / frustum_length));
		projectionMatrix.m23(-1);
		projectionMatrix.m32(-((2 * NEAR_PLANE * FAR_PLANE) / frustum_length));
		projectionMatrix.m33(0);
		return projectionMatrix;
	}

	private void calculateCameraPosition(float horizDistance, float verticDistance) {
		float theta = angleAroundPlayer.get();
		position.x = (float) (horizDistance * Math.sin(Math.toRadians(theta)));
		position.y = verticDistance + Y_OFFSET;
		position.z = (float) (horizDistance * Math.cos(Math.toRadians(theta)));
	}

	/**
	 * @return The horizontal distance of the camera from the origin.
	 */
	private float calculateHorizontalDistance() {
		return (float) (distanceFromPlayer.get() * Math.cos(Math.toRadians(pitch.get())));
	}

	/**
	 * @return The height of the camera from the aim point.
	 */
	private float calculateVerticalDistance() {
		return (float) (distanceFromPlayer.get() * Math.sin(Math.toRadians(pitch.get())));
	}

	/**
	 * Ensures the camera's pitch isn't too high or too low.
	 */
	private void clampPitch() {
		if (pitch.getTarget() < 0) {
			pitch.setTarget(0);
		} else if (pitch.getTarget() > MAX_PITCH) {
			pitch.setTarget(MAX_PITCH);
		}
	}
	
	private void clampDistance() {
		if (distanceFromPlayer.getTarget() < 0) {
			distanceFromPlayer.setTarget(0);
		} else if (distanceFromPlayer.getTarget() > MAX_DISTANCE) {
			distanceFromPlayer.setTarget(MAX_DISTANCE);
		}
	}

	@Override
	public void invoke(long arg0, double arg1, double arg2) {
		// TODO Auto-generated method stub
		if(arg2 != 0) {
			pitch.increaseTarget(-(float) (arg2 * PITCH_SENSITIVITY));
			clampPitch();
			pitch.update(DisplayManager.getFrameTime());
		} else if(arg1 != 0) {
			angleAroundPlayer.increaseTarget(-(float) (arg1 * YAW_SENSITIVITY));
			angleAroundPlayer.update(DisplayManager.getFrameTime());
		}
	}
	
}
