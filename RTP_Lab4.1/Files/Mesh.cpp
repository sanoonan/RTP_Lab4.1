#include "Mesh.h"

using namespace std;

Mesh :: Mesh()
{
	filename = "";
	vao = 0;
	point_count = 0;
}

Mesh :: Mesh(const char* _filename)
{
	filename = _filename;
	vao = 0;
	point_count = 0;
}

bool Mesh :: load_mesh (std::vector<glm::vec3> &v, std::vector<glm::vec3> &r)
{
	
	v = getVertices(r);
	
  const aiScene* scene = aiImportFile (filename, aiProcess_Triangulate); // TRIANGLES!

  
  if (!scene)
  {
    fprintf (stderr, "ERROR: reading mesh %s\n", filename);
    return false;
  }
  printf ("  %i animations\n", scene->mNumAnimations);
  printf ("  %i cameras\n", scene->mNumCameras);
  printf ("  %i lights\n", scene->mNumLights);
  printf ("  %i materials\n", scene->mNumMaterials);
  printf ("  %i meshes\n", scene->mNumMeshes);
  printf ("  %i textures\n", scene->mNumTextures);
  
  

  for(int i=0; i<scene->mNumMeshes; i++)
  {
	assign_vao(scene);
  }
 
  aiReleaseImport (scene);
  printf("mesh loaded\n");
  
  return true;
}


//loads assimp mesh data into the vao
void Mesh :: assign_vao(const aiScene* scene)
{
	 const aiMesh* mesh = scene->mMeshes[0]; 

	 printf("   %i vertices in mesh[%i]\n", mesh->mNumVertices, 0);

	  point_count = mesh->mNumVertices;


	  glGenVertexArrays(1, &vao);
	  glBindVertexArray(vao);

		GLfloat* points = NULL;
		GLfloat* normals = NULL;
		GLfloat* texcoords = NULL;


		GLint* bone_ids_lbs = NULL;
		GLfloat* bone_weights = NULL;

		//get vertex positions
		  if (mesh->HasPositions ())
		  {

			points = (GLfloat*)malloc (point_count * 3 * sizeof (GLfloat));

			for (int i=0; i< point_count; i++)
			{
				const aiVector3D* vp = &(mesh->mVertices[i]);
				points[i*3] = (GLfloat)vp->x;
				points[i*3+1] = (GLfloat)vp->y;
				points[i*3+2] = (GLfloat)vp->z;
			}


		  }

		  //get normals
		  if (mesh->HasNormals ()) {
			normals = (GLfloat*)malloc(point_count * 3 * sizeof (GLfloat));

			for (int i=0; i<point_count; i++)
			{
				const aiVector3D* vn = &(mesh->mNormals[i]);
				normals[i*3] = (GLfloat)vn->x;
				normals[i*3+1] = (GLfloat)vn->y;
				normals[i*3+2] = (GLfloat)vn->z;
			}
		  }

		  //get texture coordinates
		  if (mesh->HasTextureCoords (0)) {
			texcoords = (GLfloat*)malloc(point_count * 2 * sizeof (GLfloat));

			for (int i=0; i<point_count; i++)
			{
				const aiVector3D* vt = &(mesh->mTextureCoords[0][i]);
				texcoords[i*2] = (GLfloat)vt->x;
				texcoords[i*2+1] = (GLfloat)vt->y;
		
			}
		  }

	
		  // copy mesh data in VBOs


		  //load in vertex positions
		  if(mesh->HasPositions())
		  {
			  GLuint vbo;
			  glGenBuffers (1, &vbo);
			  glBindBuffer(GL_ARRAY_BUFFER, vbo);
			  glBufferData(GL_ARRAY_BUFFER, 3 * point_count * sizeof(GLfloat), points, GL_STATIC_DRAW);

			  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);
			  glEnableVertexAttribArray(0);
			  free (points);
		  }

		  //load in normals
		  if(mesh->HasNormals())
		  {
			  GLuint vbo;
			  glGenBuffers (1, &vbo);
			  glBindBuffer(GL_ARRAY_BUFFER, vbo);
			  glBufferData(GL_ARRAY_BUFFER, 3 * point_count * sizeof(GLfloat), normals, GL_STATIC_DRAW);

			  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, NULL);
			  glEnableVertexAttribArray(1);
			  free (normals);
		  }

		  //load in texture coordinates
		  if(mesh->HasTextureCoords(0))
		  {
			  GLuint vbo;
			  glGenBuffers (1, &vbo);
			  glBindBuffer(GL_ARRAY_BUFFER, vbo);
			  glBufferData(GL_ARRAY_BUFFER, 2 * point_count * sizeof(GLfloat), texcoords, GL_STATIC_DRAW);

			  glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, NULL);
			  glEnableVertexAttribArray(2);
			  free (texcoords);
		  }


}



void Mesh :: draw(GLuint spID, glm::mat4 model_mat)
{
	int matrix_location = glGetUniformLocation (spID, "model");
	glUniformMatrix4fv(matrix_location, 1, GL_FALSE, glm::value_ptr(model_mat));
	glBindVertexArray(vao);
	glDrawArrays(GL_TRIANGLES, 0, point_count);
}


void Mesh :: draw(GLuint spID, glm::mat4 model_mat, glm::vec3 colour)
{

	int colour_location = glGetUniformLocation (spID, "Kd");
	glUniform3fv(colour_location, 1, glm::value_ptr(colour));

	int matrix_location = glGetUniformLocation (spID, "model");
	glUniformMatrix4fv(matrix_location, 1, GL_FALSE, glm::value_ptr(model_mat));
	glBindVertexArray(vao);
	glDrawArrays(GL_TRIANGLES, 0, point_count);
}


std::vector<glm::vec3> Mesh :: getVertices(std::vector<glm::vec3> &reduced)
{
	const aiScene* scene = aiImportFile (filename, aiProcess_JoinIdenticalVertices);

	const aiMesh* mesh = scene->mMeshes[0]; 

	int vertex_count = mesh->mNumVertices;
	std::vector<glm::vec3> v(vertex_count);

	//get vertex positions
	if (mesh->HasPositions ())
	{

		for (int i=0; i< vertex_count; i++)
		{
			glm::vec3 curr;
			const aiVector3D* vp = &(mesh->mVertices[i]);
			curr.x = (GLfloat)vp->x;
			curr.y = (GLfloat)vp->y;
			curr.z = (GLfloat)vp->z;

			v[i] = curr;

			if(!checkForRepeat(curr, reduced))
				reduced.push_back(curr);
		}
	}
	return v;
}

bool Mesh :: checkForRepeat(glm::vec3 v, std::vector<glm::vec3> &verts)
{
	for(int i=0; i<verts.size(); i++)
		if(glm::distance2(v, verts[i]) < 0.000001)
			return true;

	return false;
		
}