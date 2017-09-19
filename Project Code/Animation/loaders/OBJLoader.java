package loaders;

import java.util.List;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.util.ArrayList;
import org.joml.Vector3f;

import entityModel.RawModel;

import org.joml.Vector2f;

public class OBJLoader {
	
	public static RawModel loadObjModel(String fileName, Loader loader) {
		FileReader fr = null;
		try {
			fr = new FileReader(new File("res/"+fileName+".obj"));
		} catch (FileNotFoundException e) {
			System.out.println("Couldn't load file!");
			e.printStackTrace();
		}
		BufferedReader reader = new BufferedReader(fr);
		String line = null;
		List<Vector3f> vertices = new ArrayList<>();
		List<Vector2f> textures = new ArrayList<>();
		List<Vector3f> normals = new ArrayList<>();
		List<Integer> indices = new ArrayList<>();
		float[] verticesArray = null;
		float[] texturesArray = null;
		float[] normalsArray = null;
		int[] indicesArray = null;
		try {
			while(true) {
				line = reader.readLine();
				String[] currentLine = line.split(" ");
				if(line.startsWith("v ")) {
					Vector3f vertex = new Vector3f(Float.valueOf(currentLine[1]), Float.valueOf(currentLine[2]), Float.valueOf(currentLine[3]));
					vertices.add(vertex);
				} else if(line.startsWith("vn ")) {
					Vector3f normal = new Vector3f(Float.valueOf(currentLine[1]), Float.valueOf(currentLine[2]), Float.valueOf(currentLine[3]));
					normals.add(normal);
				} else if(line.startsWith("vt ")) {
					Vector2f texture = new Vector2f(Float.valueOf(currentLine[1]), Float.valueOf(currentLine[2]));
					textures.add(texture);
				} else if(line.startsWith("f ")) {
					texturesArray = new float[vertices.size() * 2];
					normalsArray = new float[vertices.size() * 3];
					break;
				}
			}
			
			while(line!=null) {
				if(!line.startsWith("f ")) {
					line = reader.readLine();
					continue;
				}
				String[] currentLine = line.split(" ");
				String[] vertex1 = currentLine[1].split("/");
				String[] vertex2 = currentLine[2].split("/");
				String[] vertex3 = currentLine[3].split("/");
				processVertex(vertex1, indices, normals, textures, texturesArray, normalsArray);
				processVertex(vertex2, indices, normals, textures, texturesArray, normalsArray);
				processVertex(vertex3, indices, normals, textures, texturesArray, normalsArray);
				line = reader.readLine();
			}
			reader.close();
			
			verticesArray = new float[vertices.size() * 3];
			indicesArray = new int[indices.size()];
			
			int vertexPointer = 0;
			for(Vector3f vertex : vertices)	{
				verticesArray[vertexPointer++] = vertex.x;
				verticesArray[vertexPointer++] = vertex.y;
				verticesArray[vertexPointer++] = vertex.z;
			}
			
			int indexPointer = 0;
			for(int index : indices)
				indicesArray[indexPointer++] = index;
			
			return loader.loadToVAO(verticesArray, texturesArray, normalsArray, indicesArray); 
			
		} catch(Exception e) {
			e.printStackTrace();
		}
		return null;
	}
	
	private static void processVertex(String[] vertexData, List<Integer> indices, List<Vector3f> normals, List<Vector2f> textures, float[] texturesArray, float[] normalsArray) {
		int currentVertexPointer = Integer.valueOf(vertexData[0]) - 1;
		indices.add(currentVertexPointer);
		Vector2f currentTex = textures.get(Integer.valueOf(vertexData[1])-1);
		texturesArray[currentVertexPointer*2] = currentTex.x;
		texturesArray[currentVertexPointer * 2 + 1] = 1 - currentTex.y;
		Vector3f currentNorm = normals.get(Integer.valueOf(vertexData[2])-1);
		normalsArray[currentVertexPointer * 3] = currentNorm.x;
		normalsArray[currentVertexPointer * 3 + 1] = currentNorm.y;
		normalsArray[currentVertexPointer * 3 + 2] = currentNorm.z;
	}
	
}
