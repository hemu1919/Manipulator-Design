package loaders;

import java.io.FileInputStream;
import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.ArrayList;
import java.util.List;

import org.lwjgl.BufferUtils;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL15;
import org.lwjgl.opengl.GL20;
import org.lwjgl.opengl.GL30;
import de.matthiasmann.twl.utils.PNGDecoder;
import entityModel.RawModel;

public class Loader {
	
	private List<Integer> vaos = new ArrayList<>();
	private List<Integer> vbos = new ArrayList<>();
	private List<Integer> textures = new ArrayList<>();
	
	public RawModel loadToVAO(float[] positions, float[] testureCoords, float[] normalsArray, int[] indices) {
		int vaoID = createVAO();
		bindIndicesBuffer(indices);
		storeDataInAttributeList(0, 3, positions);
		storeDataInAttributeList(1, 2, testureCoords);
		storeDataInAttributeList(2, 3, normalsArray);
		unBindVAO();
		return new RawModel(vaoID, indices.length);
	}
	
	public int loadTexture(String fileName) {
		try {
			FileInputStream in = new FileInputStream("res/"+fileName+".png");
			PNGDecoder decoder = new PNGDecoder(in);
			ByteBuffer buffer = ByteBuffer.allocateDirect(3*decoder.getWidth()*decoder.getHeight());
			decoder.decode(buffer, decoder.getWidth()*3, PNGDecoder.Format.RGB);
			buffer.flip();
			int texture = GL11.glGenTextures();
			GL11.glBindTexture(GL11.GL_TEXTURE_2D, texture);
			GL11.glTexImage2D(GL11.GL_TEXTURE_2D, 0, GL11.GL_RGB, decoder.getWidth(), decoder.getHeight(), 0,GL11.GL_RGB, GL11.GL_UNSIGNED_BYTE, buffer);
			GL11.glTexParameteri(GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_MIN_FILTER, GL11.GL_NEAREST);
			GL11.glTexParameteri(GL11.GL_TEXTURE_2D, GL11.GL_TEXTURE_MAG_FILTER, GL11.GL_NEAREST);
			GL11.glBindTexture(GL11.GL_TEXTURE_2D, 0);
			in.close();
			textures.add(texture);
			return texture;
		} catch(Exception e) {
			System.err.println("Invalid Data Source");
			System.exit(-1);
		}
		return -1;
	}
	
	public void cleanUp() {
		for(int vaoID : vaos)
			GL30.glDeleteVertexArrays(vaoID);
		for(int vboID : vbos)
			GL15.glDeleteBuffers(vboID);
		for(int texture : textures)
			GL11.glDeleteTextures(texture);;
	}
	
	private int createVAO() {
		int vaoID = GL30.glGenVertexArrays();
		vaos.add(vaoID);
		GL30.glBindVertexArray(vaoID);
		return vaoID;
	}
	
	private void storeDataInAttributeList(int attributeNumber, int coordinateSize, float[] data) {
		int vboID = GL15.glGenBuffers();
		vbos.add(vboID);
		GL15.glBindBuffer(GL15.GL_ARRAY_BUFFER, vboID);
		FloatBuffer buffer = storeDataInFloatBuffer(data);
		GL15.glBufferData(GL15.GL_ARRAY_BUFFER, buffer, GL15.GL_STATIC_DRAW);
		GL20.glVertexAttribPointer(attributeNumber, coordinateSize, GL11.GL_FLOAT, false, 0, 0);
		GL15.glBindBuffer(GL15.GL_ARRAY_BUFFER, 0);
	}
	
	private void unBindVAO() {
		GL30.glBindVertexArray(0);
	}
	
	private void bindIndicesBuffer(int[] indices) {
		int vboID = GL15.glGenBuffers();
		vbos.add(vboID);
		GL15.glBindBuffer(GL15.GL_ELEMENT_ARRAY_BUFFER, vboID);
		IntBuffer buffer = storeDataInIntBuffer(indices);
		GL15.glBufferData(GL15.GL_ELEMENT_ARRAY_BUFFER, buffer, GL15.GL_STATIC_DRAW);
	}
	
	private IntBuffer storeDataInIntBuffer(int[] data) {
		IntBuffer buffer = BufferUtils.createIntBuffer(data.length);
		buffer.put(data);
		buffer.flip();
		return buffer;
	}
	
	private FloatBuffer storeDataInFloatBuffer(float[] data) {
		FloatBuffer buffer = BufferUtils.createFloatBuffer(data.length);
		buffer.put(data);
		buffer.flip();
		return buffer;
	}
	
}
