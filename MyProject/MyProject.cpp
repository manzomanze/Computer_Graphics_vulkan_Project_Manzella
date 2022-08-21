// This has been adapted from the Vulkan tutorial

#include "MyProject.hpp"



// The uniform buffer object used in this example
struct UniformBufferObject {
	alignas(16) glm::mat4 model;
	alignas(16) glm::mat4 view;
	alignas(16) glm::mat4 proj;
};



// MAIN ! 
class MyProject : public BaseProject {
	protected:
	// Here you list all the Vulkan objects you need:
	
	// Descriptor Layouts [what will be passed to the shaders]
	DescriptorSetLayout DSL1;

	// Pipelines [Shader couples]
	Pipeline P1;

	// Models, textures and Descriptors (values assigned to the uniforms)
	Model Body;
	Texture BodyTexture;
	DescriptorSet DSBody;

	Model Puller;
	Texture PullerTexture;
	DescriptorSet DSPuller;

	Model Ball;
	Texture BallTexture;
	DescriptorSet DSBall;
	
	// Here you set the main application parameters
	void setWindowParameters() {
		// window size, titile and initial background
		windowWidth = 1600;
		windowHeight = 900;
		windowTitle = "My Project";
		initialBackgroundColor = {1.0f, 1.0f, 1.0f, 1.0f};
		
		// Descriptor pool sizes
		uniformBlocksInPool = 3;
		texturesInPool = 3;
		setsInPool = 3;
	}
	
	// Here you load and setup all your Vulkan objects
	void localInit() {
		// Descriptor Layouts [what will be passed to the shaders]
		DSL1.init(this, {
					// this array contains the binding:
					// first  element : the binding number
					// second element : the time of element (buffer or texture)
					// third  element : the pipeline stage where it will be used
					{0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, VK_SHADER_STAGE_VERTEX_BIT},
					{1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT}
				  });

		// Pipelines [Shader couples]
		// The last array, is a vector of pointer to the layouts of the sets that will
		// be used in this pipeline. The first element will be set 0, and so on..
		P1.init(this, "shaders/vert.spv", "shaders/frag.spv", {&DSL1});

		// Models, textures and Descriptors (values assigned to the uniforms)
		Body.init(this, "models/PinballDark/Body2.obj");
		BodyTexture.init(this, "textures/StarWarsPinball.png");


		DSBody.init(this, &DSL1, {
		// the second parameter, is a pointer to the Uniform Set Layout of this set
		// the last parameter is an array, with one element per binding of the set.
		// first  elmenet : the binding number
		// second element : UNIFORM or TEXTURE (an enum) depending on the type
		// third  element : only for UNIFORMs, the size of the corresponding C++ object
		// fourth element : only for TEXTUREs, the pointer to the corresponding texture object
					{0, UNIFORM, sizeof(UniformBufferObject), nullptr},
					{1, TEXTURE, 0, &BodyTexture}
				});

				
		Puller.init(this, "models/PinballDark/Puller1.obj");
		PullerTexture.init(this, "textures/StarWarsPinballColors.png");

		DSPuller.init(this, &DSL1, {
					{0, UNIFORM, sizeof(UniformBufferObject), nullptr},
					{1, TEXTURE, 0, &PullerTexture}
				});
		

		Ball.init(this, "models/PinballDark/BallColored.obj");
		BallTexture.init(this, "textures/StarWarsPinballColors.png");


		DSBall.init(this, &DSL1, {
					{0, UNIFORM, sizeof(UniformBufferObject), nullptr},
					{1, TEXTURE, 0, &BallTexture}
				});

		
	}

	// Here you destroy all the objects you created!		
	void localCleanup() {
		DSBody.cleanup();
		BodyTexture.cleanup();
		Body.cleanup();
		

		DSPuller.cleanup();
		PullerTexture.cleanup();
		Puller.cleanup();

		DSBall.cleanup();
		BallTexture.cleanup();
		Ball.cleanup();

		DSL1.cleanup();
		P1.cleanup();
	}
	
	// Here it is the creation of the command buffer:
	// You send to the GPU all the objects you want to draw,
	// with their buffers and textures
	void populateCommandBuffer(VkCommandBuffer commandBuffer, int currentImage) {
				
		vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS,
				P1.graphicsPipeline);
				
		VkBuffer vertexBuffers[] = {Body.vertexBuffer};
		// property .vertexBuffer of models, contains the VkBuffer handle to its vertex buffer
		VkDeviceSize offsets[] = {0};
		vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffers, offsets);
		// property .indexBuffer of models, contains the VkBuffer handle to its index buffer
		vkCmdBindIndexBuffer(commandBuffer, Body.indexBuffer, 0,
								VK_INDEX_TYPE_UINT32);

		// property .pipelineLayout of a pipeline contains its layout.
		// property .descriptorSets of a descriptor set contains its elements.
		vkCmdBindDescriptorSets(commandBuffer,
						VK_PIPELINE_BIND_POINT_GRAPHICS,
						P1.pipelineLayout, 0, 1, &DSBody.descriptorSets[currentImage],
						0, nullptr);
		
		// property .indices.size() of models, contains the number of triangles * 3 of the mesh.
		vkCmdDrawIndexed(commandBuffer,
					static_cast<uint32_t>(Body.indices.size()), 1, 0, 0, 0);

		// Puller
		VkBuffer vertexBuffersPuller[] = {Puller.vertexBuffer};
		VkDeviceSize offsetsPuller[] = {0};
		vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffersPuller, offsetsPuller);
		vkCmdBindIndexBuffer(commandBuffer, Puller.indexBuffer, 0,
								VK_INDEX_TYPE_UINT32);

		vkCmdBindDescriptorSets(commandBuffer,
						VK_PIPELINE_BIND_POINT_GRAPHICS,
						P1.pipelineLayout, 0, 1, &DSPuller.descriptorSets[currentImage],
						0, nullptr);
						
		vkCmdDrawIndexed(commandBuffer,
					static_cast<uint32_t>(Puller.indices.size()), 1, 0, 0, 0);


		// Ball
		VkBuffer vertexBuffersBall[] = {Ball.vertexBuffer};
		VkDeviceSize offsetsBall[] = {0};
		vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffersBall, offsetsBall);
		vkCmdBindIndexBuffer(commandBuffer, Ball.indexBuffer, 0,
								VK_INDEX_TYPE_UINT32);

		vkCmdBindDescriptorSets(commandBuffer,
						VK_PIPELINE_BIND_POINT_GRAPHICS,
						P1.pipelineLayout, 0, 1, &DSBall.descriptorSets[currentImage],
						0, nullptr);
						
		vkCmdDrawIndexed(commandBuffer,
					static_cast<uint32_t>(Ball.indices.size()), 1, 0, 0, 0);


					
	}

	// Here is where you update the uniforms.
	// Very likely this will be where you will be writing the logic of your application.
	void updateUniformBuffer(uint32_t currentImage) {
		static auto startTime = std::chrono::high_resolution_clock::now();
		auto currentTime = std::chrono::high_resolution_clock::now();
		float time = std::chrono::duration<float, std::chrono::seconds::period>
					(currentTime - startTime).count();
					
					
		UniformBufferObject ubo{};

		static float lastTime=0.0f;

		float deltaT = time - lastTime;
		lastTime = time;

		glm::mat4 BodyPosition = glm::translate(glm::mat4(1.0f),glm::vec3(0.0f, 5.0f, 0.0f))/* *glm::rotate(glm::mat4(1.0f),
								glm::radians(45.0f),
								glm::vec3(1.0f, 0.0f, 0.0f)) */;
		//glm::mat4 BodyPosition = glm::mat4(1.0f);

		static float cameraX = 0.0f;
		static float cameraY = 0.0f;
		static float cameraZ = 0.0f;

		static float cameraPitch = 0.0f;
		static float cameraYaw = 0.0f;
		static float cameraRotationSpeed = 10.0f;
		

		static float valueX = 0.0f;
		static float valueY = 0.0f;
		static float valueZ = 0.0f;
		static float zoomOut = 2.0f;

		static float pullerMax = 2.0f;
		static float pullerDistanceCovered = 0.0f;
		static float pullSpeed = 2.0f;
		static float springSpeed = 10.0f;
		static float launchBallSpeed = 0.0f;
		
		
		
		static float ballX = 0.0f;
		static float ballY = 0.0f;
		static float ballZ = 0.0f;


		static float rightZMargin = -2.47737f;
		static float leftZMargin = 2.54627f;
		static float topXMargin = -5.45043f;
		static float bottomXMargin = 4.26388f;
		static float topYMargin = 2.0f;
		static float bottomYMargin = 0.0f;
		static float rightFlipperMargin = 1.37733f;
		static float leftFlipperMargin = -1.09777f;

		static float ballXstart = 4.0f;
		static float ballZstart = -2.0f;
		
		//definition of physics interacting objects

		static MovingRotationalObjectDimensions Ball;
		static OrientableObjectDimensions TopWall;
		static OrientableObjectDimensions RightWall;
		static OrientableObjectDimensions LeftWall;
		
		static auto previousReleaseValueOfSpace = GLFW_RELEASE;

		///Ball.speedX = 0.0f;
		/* 
			Camera Linear Movement
		 */
		{
			if(glfwGetKey(window,GLFW_KEY_D)){
				valueZ = valueZ-zoomOut*deltaT;
			}
			if(glfwGetKey(window,GLFW_KEY_A)){
				valueZ = valueZ+zoomOut*deltaT;
			}


			if(glfwGetKey(window,GLFW_KEY_W)){
				valueX = valueX-zoomOut*deltaT;
			}
			if(glfwGetKey(window,GLFW_KEY_S)){
				valueX = valueX+zoomOut*deltaT;
			}


			if(glfwGetKey(window,GLFW_KEY_R)){
				valueY = valueY+zoomOut*deltaT;
			}
			if(glfwGetKey(window,GLFW_KEY_F)){
				valueY = valueY-zoomOut*deltaT;
			}
		}
		/* 
			Camera Rotation Movement
		 */
		{
			if(glfwGetKey(window,GLFW_KEY_UP)){
				cameraPitch = cameraPitch+cameraRotationSpeed*deltaT;
			}
			if(glfwGetKey(window,GLFW_KEY_DOWN)){
				cameraPitch = cameraPitch-cameraRotationSpeed*deltaT;
			}


			if(glfwGetKey(window,GLFW_KEY_LEFT)){
				cameraYaw = cameraYaw+cameraRotationSpeed*deltaT;
			}
			if(glfwGetKey(window,GLFW_KEY_RIGHT)){
				cameraYaw = cameraYaw-cameraRotationSpeed*deltaT;
			}
		}

		/* 
			Puller 
		*/
		{
			if(glfwGetKey(window,GLFW_KEY_SPACE)){
				if(pullerDistanceCovered<=0.8f){
					pullerDistanceCovered = pullerDistanceCovered+pullSpeed*deltaT;
					
				}
			}else{
				if(pullerDistanceCovered>=0.0f){
					pullerDistanceCovered = pullerDistanceCovered-springSpeed*deltaT;
					
				}
			}
			if(glfwGetKey(window,GLFW_KEY_SPACE) == GLFW_RELEASE && previousReleaseValueOfSpace == GLFW_PRESS){
				//The Minus is because positive x values come toward the viewer and positive Z values go to the left looking straght on at the table
				launchBallSpeed = -pullerDistanceCovered*4.0f;
				Ball.speedX = launchBallSpeed;
				// next line must be cancelled, only to test right wall
				Ball.speedZ = Ball.speedX;
			}

			previousReleaseValueOfSpace = glfwGetKey(window,GLFW_KEY_SPACE);
		}
	

		
		// Update of ball speed in the coordinates of the body
		ballX = ballX+Ball.speedX*deltaT;
		ballY = ballY+Ball.speedY*deltaT;
		ballZ = ballZ+Ball.speedZ*deltaT;

		// Positions of the objects when they are models

		glm::mat4 PullerCurrentPosition = glm::translate(glm::mat4(1.0f),glm::vec3(ballXstart+1.9f+pullerDistanceCovered, ballRadius, ballZstart))*BodyPosition;

		glm::vec3 BallReady = glm::vec3(ballXstart+ballRadius, ballRadius, ballZstart);
		glm::mat4 BallCurrentPosition = glm::translate(glm::mat4(1.0f),glm::vec3(BallReady.x+ballX, BallReady.y, BallReady.z+ballZ))*BodyPosition;
		glm::vec4 BallCurrentPositionvector = BallCurrentPosition*glm::vec4(1.0f,1.0f,1.0f,1.0f);
		BallCurrentPositionvector = glm::vec4(BallCurrentPositionvector.x-1.0f,BallCurrentPositionvector.y-1.0f,BallCurrentPositionvector.z-1.0f,BallCurrentPositionvector.w-1.0f);


		Ball.originX = BallCurrentPositionvector.x;
		Ball.originY = BallCurrentPositionvector.y;
		Ball.originZ = BallCurrentPositionvector.z;

		glm::vec4 BodyPositionVector = BodyPosition*glm::vec4(1.0f,1.0f,1.0f,1.0f);
		//std::cout<< "result X:"<<result.x<< " Y "<< result.y<<" Z "<< result.z<<std::endl;
		BodyPositionVector = glm::vec4(BodyPositionVector.x-1.0f,BodyPositionVector.y-1.0f,BodyPositionVector.z-1.0f,BodyPositionVector.w-1.0f);

		glm::vec4 AnyWallminYPositionVector = BodyPosition*glm::vec4(0.0f,bottomYMargin,0.0f,1.0f);
		glm::vec4 AnyWallmaxYPositionVector = BodyPosition*glm::vec4(0.0f,topYMargin,0.0f,1.0f);

		glm::vec4 TopWallminXPositionVector = BodyPosition*glm::vec4(topXMargin-sideWallDepth,0.0f,0.0f,1.0f);
		glm::vec4 TopWallmaxXPositionVector = BodyPosition*glm::vec4(topXMargin,0.0f,1.0f,1.0f);
		glm::vec4 TopWallminZPositionVector = BodyPosition*glm::vec4(0.0f,0.0f,rightZMargin,1.0f);
		glm::vec4 TopWallmaxZPositionVector = BodyPosition*glm::vec4(0.0f,0.0f,leftZMargin,1.0f);

		TopWall.minX = TopWallminXPositionVector.x;
		TopWall.maxX = TopWallmaxXPositionVector.x;
		TopWall.minY = AnyWallminYPositionVector.y;
		TopWall.maxY = AnyWallmaxYPositionVector.y;
		TopWall.minZ = TopWallminZPositionVector.z;
		TopWall.maxZ = TopWallmaxZPositionVector.z;
		TopWall.orientationWithRespectToNegativeZaxis = 3.14/2;

		glm::vec4 RightWallminXPositionVector = BodyPosition*glm::vec4(topXMargin,0.0f,0.0f,1.0f);
		glm::vec4 RightWallmaxXPositionVector = BodyPosition*glm::vec4(bottomXMargin,0.0f,1.0f,1.0f);
		glm::vec4 RightWallminZPositionVector = BodyPosition*glm::vec4(0.0f,0.0f,rightZMargin-sideWallDepth,1.0f);
		glm::vec4 RightWallmaxZPositionVector = BodyPosition*glm::vec4(0.0f,0.0f,rightZMargin,1.0f);


		RightWall.minX = RightWallminXPositionVector.x;
		RightWall.maxX = RightWallmaxXPositionVector.x;
		RightWall.minY = AnyWallminYPositionVector.y;
		RightWall.maxY = AnyWallmaxYPositionVector.y;
		RightWall.minZ = RightWallminZPositionVector.z;
		RightWall.maxZ = RightWallmaxZPositionVector.z;
		RightWall.orientationWithRespectToNegativeZaxis = 0.0f;

		glm::vec4 LeftWallminXPositionVector = BodyPosition*glm::vec4(topXMargin,0.0f,0.0f,1.0f);
		glm::vec4 LeftWallmaxXPositionVector = BodyPosition*glm::vec4(bottomXMargin,0.0f,1.0f,1.0f);
		glm::vec4 LeftWallminZPositionVector = BodyPosition*glm::vec4(0.0f,0.0f,leftZMargin,1.0f);
		glm::vec4 LeftWallmaxZPositionVector = BodyPosition*glm::vec4(0.0f,0.0f,leftZMargin+sideWallDepth,1.0f);

		LeftWall.minX = LeftWallminXPositionVector.x;
		LeftWall.maxX = LeftWallmaxXPositionVector.x;
		LeftWall.minY = AnyWallminYPositionVector.y;
		LeftWall.maxY = AnyWallmaxYPositionVector.y;
		LeftWall.minZ = LeftWallminZPositionVector.z;
		LeftWall.maxZ = LeftWallmaxZPositionVector.z;
		LeftWall.orientationWithRespectToNegativeZaxis = 0.0f;

		std::cout<< "ball origin X:"<<Ball.originX<< " Y "<< Ball.originY<<" Z "<< Ball.originZ<<std::endl;
		std::cout<< "right wall Y: min"<<RightWall.minY<< " max "<< RightWall.maxY<<" Z "<< Ball.originZ<<std::endl;
		std::cout<< "ball speed X:"<<Ball.speedX<< " Z "<< Ball.speedZ<<std::endl;


		Ball = intersectBallOrientedObstacle(Ball,RightWall);
		Ball = intersectBallOrientedObstacle(Ball,LeftWall);
		Ball = intersectBallOrientedObstacle(Ball,TopWall);

		// Movement of the models and drawing is described here

		ubo.model = BodyPosition;
		// Camera Position parallell to the playing plane
		ubo.view = glm::lookAt(glm::vec3(5.0f+cameraX+valueX, 0.0f+cameraY+valueY, cameraZ+valueZ),
							   glm::vec3(0.0f+cameraX, 0.0f+cameraY, 0.0f+cameraZ),
							   glm::vec3(0.0f, 1.0f, 0.0f))*

								glm::rotate(glm::mat4(1.0f),
								glm::radians(cameraPitch),
								glm::vec3(0.0f, 0.0f, 1.0f))*

								glm::rotate(glm::mat4(1.0f),
								glm::radians(cameraYaw),
								glm::vec3(0.0f, 1.0f, 0.0f));
		
		ubo.proj = glm::perspective(glm::radians(70.0f),
						swapChainExtent.width / (float) swapChainExtent.height,
						0.1f, 100.0f);
		ubo.proj[1][1] *= -1;
		
		void* data;

		// Here is where you actually update your uniforms
		//Body
		vkMapMemory(device, DSBody.uniformBuffersMemory[0][currentImage], 0,
							sizeof(ubo), 0, &data);
		memcpy(data, &ubo, sizeof(ubo));
		vkUnmapMemory(device, DSBody.uniformBuffersMemory[0][currentImage]);

		//Puller Position
		ubo.model = PullerCurrentPosition*
					glm::rotate(glm::mat4(1.0f),
								glm::radians(-90.0f),
								glm::vec3(0.0f, 0.0f, 1.0f));
		vkMapMemory(device, DSPuller.uniformBuffersMemory[0][currentImage], 0,
							sizeof(ubo), 0, &data);
		memcpy(data, &ubo, sizeof(ubo));
		vkUnmapMemory(device, DSPuller.uniformBuffersMemory[0][currentImage]);

		//Ball Position
		ubo.model = BallCurrentPosition;
					
		vkMapMemory(device, DSBall.uniformBuffersMemory[0][currentImage], 0,
							sizeof(ubo), 0, &data);
		memcpy(data, &ubo, sizeof(ubo));
		vkUnmapMemory(device, DSBall.uniformBuffersMemory[0][currentImage]);
	}	
};

// This is the main: probably you do not need to touch this!
int main() {
    MyProject app;

    try {
        app.run();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}