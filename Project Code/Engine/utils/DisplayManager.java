package utils;

import org.lwjgl.glfw.GLFW;
import org.lwjgl.opengl.GL;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL13;

public class DisplayManager {

	private static final String TITLE = "ThinMatrix Animation Tutorial";
	private static final int WIDTH = 1280;
	private static final int HEIGHT = 720;
	private static final int FPS_CAP = 100;

	private static double lastFrameTime;
	private static long window;
	private static float delta;

	public static void createDisplay() {
		try {
			GLFW.glfwInit();
			GLFW.glfwWindowHint(GLFW.GLFW_RESIZABLE, GL11.GL_TRUE);
			GLFW.glfwWindowHint(GLFW.GLFW_CONTEXT_VERSION_MAJOR, 3);
			GLFW.glfwWindowHint(GLFW.GLFW_CONTEXT_VERSION_MINOR, 2);
			GLFW.glfwWindowHint(GLFW.GLFW_OPENGL_PROFILE, GLFW.GLFW_OPENGL_CORE_PROFILE);
			GLFW.glfwWindowHint(GLFW.GLFW_OPENGL_FORWARD_COMPAT, GL11.GL_TRUE);
			window = GLFW.glfwCreateWindow(WIDTH, HEIGHT, TITLE, 0, 0);
			if(window == 0)
		    	throw new RuntimeException("Failed to create window");
			GLFW.glfwMakeContextCurrent(window);
			GL.createCapabilities();
			GLFW.glfwShowWindow(window);
			//Display.setInitialBackground(1, 1, 1);
			GL11.glEnable(GL13.GL_MULTISAMPLE);
		} catch (Exception e) {
			e.printStackTrace();
			System.err.println(e.getMessage());
			System.exit(-1);
		}
		lastFrameTime = getCurrentTime();
	}

	public static void update() {
		GLFW.glfwWindowHint(GLFW.GLFW_REFRESH_RATE, FPS_CAP);
		GLFW.glfwPollEvents();
		GLFW.glfwSwapBuffers(window);
		double currentFrameTime = getCurrentTime();
		delta = (float) (currentFrameTime - lastFrameTime);
		lastFrameTime = currentFrameTime;
	}

	public static float getFrameTime() {
		return delta;
	}

	public static void closeDisplay() {
		GLFW.glfwDestroyWindow(window);
		GLFW.glfwTerminate();
	}

	private static double getCurrentTime() {
		return GLFW.glfwGetTime();
	}
	
	public static boolean isCloseRequested() {
		return GLFW.glfwWindowShouldClose(window);
	}

	public static int getWidth() {
		return WIDTH;
	}

	public static int getHeight() {
		return HEIGHT;
	}

	public static long getWindow() {
		return window;
	}

}
