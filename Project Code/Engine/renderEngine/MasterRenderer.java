package renderEngine;

import java.util.List;

import org.lwjgl.opengl.GL11;

import main.Entity;
import renderer.AnimatedModelRenderer;
import scene.Scene;
import skybox.SkyboxRenderer;

/**
 * This class is in charge of rendering everything in the scene to the screen.
 * @author Karl
 *
 */
public class MasterRenderer {

	private SkyboxRenderer skyRenderer;
	private AnimatedModelRenderer entityRenderer;

	protected MasterRenderer(AnimatedModelRenderer renderer, SkyboxRenderer skyRenderer) {
		this.skyRenderer = skyRenderer;
		this.entityRenderer = renderer;
	}

	public static void enableCulling() {
		GL11.glEnable(GL11.GL_CULL_FACE);
		GL11.glCullFace(GL11.GL_BACK);
	}
	
	public static void disableCulling() {
		GL11.glDisable(GL11.GL_CULL_FACE);
	}
	
	/**
	 * Renders the scene to the screen.
	 * @param scene
	 */
	protected void renderScene(Scene scene, List<Entity> environment) {
		prepare();
		entityRenderer.render(scene.getAnimatedModel(), scene.getCamera(), scene.getLightDirection());
		for(Entity entity : environment)
			entityRenderer.render(entity.getModel(), scene.getCamera(), scene.getLightDirection());
		skyRenderer.render(scene.getCamera());
	}

	/**
	 * Clean up when the game is closed.
	 */
	protected void cleanUp() {
		skyRenderer.cleanUp();
		entityRenderer.cleanUp();
	}

	/**
	 * Prepare to render the current frame by clearing the framebuffer.
	 */
	private void prepare() {
		GL11.glClearColor(1, 1, 1, 1);
		GL11.glClear(GL11.GL_COLOR_BUFFER_BIT | GL11.GL_DEPTH_BUFFER_BIT);
	}


}
