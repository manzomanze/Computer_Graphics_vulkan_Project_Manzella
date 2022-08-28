// This has been adapted from the Vulkan tutorial

#include "MyProject.hpp"



// The uniform buffer object used in this example
struct UniformBufferObject {
	alignas(16) glm::mat4 model;
	alignas(16) glm::mat4 view;
	alignas(16) glm::mat4 proj;
	alignas(16) glm::vec3 lightDir;
	alignas(16) glm::vec3 lightPos;
	alignas(16) glm::vec3 lightColor;
	alignas(16) glm::vec3 eyePos;
	alignas(16) glm::vec4 coneInOutDecayExp;
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

	Model LeftFlipper;
	Texture LeftFlipperTexture;
	DescriptorSet DSLeftFlipper;

	Model RightFlipper;
	Texture RightFlipperTexture;
	DescriptorSet DSRightFlipper;

	Model LeftBumper;
	Texture LeftBumperTexture;
	DescriptorSet DSLeftBumper;

	Model CentreBumper;
	Texture CentreBumperTexture;
	DescriptorSet DSCentreBumper;

	Model RightBumper;
	Texture RightBumperTexture;
	DescriptorSet DSRightBumper;

	Model Floor;
	Texture FloorTexture;
	DescriptorSet DSFloor;
	
	// Here you set the main application parameters
	void setWindowParameters() {
		// window size, titile and initial background
		windowWidth = 1600;
		windowHeight = 900;
		windowTitle = "My Project";
		initialBackgroundColor = {0.0f, 0.0f, 0.0f, 1.0f};
		
		// Descriptor pool sizes
		uniformBlocksInPool = 9;
		texturesInPool = 9;
		setsInPool = 9;
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

		LeftFlipper.init(this, "models/PinballDark/LeftFlipper.obj");
		LeftFlipperTexture.init(this, "textures/StarWarsPinball.png");


		DSLeftFlipper.init(this, &DSL1, {
					{0, UNIFORM, sizeof(UniformBufferObject), nullptr},
					{1, TEXTURE, 0, &LeftFlipperTexture}
				});

		RightFlipper.init(this, "models/PinballDark/RightFlipper.obj");
		RightFlipperTexture.init(this, "textures/StarWarsPinball.png");


		DSRightFlipper.init(this, &DSL1, {
					{0, UNIFORM, sizeof(UniformBufferObject), nullptr},
					{1, TEXTURE, 0, &RightFlipperTexture}
				});

		LeftBumper.init(this, "models/PinballDark/bumper1.obj");
		LeftBumperTexture.init(this, "textures/StarWarsPinball.png");


		DSLeftBumper.init(this, &DSL1, {
					{0, UNIFORM, sizeof(UniformBufferObject), nullptr},
					{1, TEXTURE, 0, &LeftBumperTexture}
				});

		CentreBumper.init(this, "models/PinballDark/bumper2.obj");
		CentreBumperTexture.init(this, "textures/StarWarsPinball.png");


		DSCentreBumper.init(this, &DSL1, {
					{0, UNIFORM, sizeof(UniformBufferObject), nullptr},
					{1, TEXTURE, 0, &CentreBumperTexture}
				});
		
		RightBumper.init(this, "models/PinballDark/bumper3.obj");
		RightBumperTexture.init(this, "textures/StarWarsPinball.png");


		DSRightBumper.init(this, &DSL1, {
					{0, UNIFORM, sizeof(UniformBufferObject), nullptr},
					{1, TEXTURE, 0, &RightBumperTexture}
				});

		Floor.init(this, "models/PinballDark/floor.obj");
		FloorTexture.init(this, "textures/horizontal-wood-plank-texture.jpg");


		DSFloor.init(this, &DSL1, {
					{0, UNIFORM, sizeof(UniformBufferObject), nullptr},
					{1, TEXTURE, 0, &FloorTexture}
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

		DSLeftFlipper.cleanup();
		LeftFlipperTexture.cleanup();
		LeftFlipper.cleanup();

		DSRightFlipper.cleanup();
		RightFlipperTexture.cleanup();
		RightFlipper.cleanup();

		DSLeftBumper.cleanup();
		LeftBumperTexture.cleanup();
		LeftBumper.cleanup();

		DSCentreBumper.cleanup();
		CentreBumperTexture.cleanup();
		CentreBumper.cleanup();

		DSRightBumper.cleanup();
		RightBumperTexture.cleanup();
		RightBumper.cleanup();

		DSFloor.cleanup();
		FloorTexture.cleanup();
		Floor.cleanup();


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

		// LeftFlipper
		VkBuffer vertexBuffersLeftFlipper[] = {LeftFlipper.vertexBuffer};
		VkDeviceSize offsetsLeftFlipper[] = {0};
		vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffersLeftFlipper, offsetsLeftFlipper);
		vkCmdBindIndexBuffer(commandBuffer, LeftFlipper.indexBuffer, 0,
								VK_INDEX_TYPE_UINT32);

		vkCmdBindDescriptorSets(commandBuffer,
						VK_PIPELINE_BIND_POINT_GRAPHICS,
						P1.pipelineLayout, 0, 1, &DSLeftFlipper.descriptorSets[currentImage],
						0, nullptr);
						
		vkCmdDrawIndexed(commandBuffer,
					static_cast<uint32_t>(LeftFlipper.indices.size()), 1, 0, 0, 0);

		// RightFlipper
		VkBuffer vertexBuffersRightFlipper[] = {RightFlipper.vertexBuffer};
		VkDeviceSize offsetsRightFlipper[] = {0};
		vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffersRightFlipper, offsetsRightFlipper);
		vkCmdBindIndexBuffer(commandBuffer, RightFlipper.indexBuffer, 0,
								VK_INDEX_TYPE_UINT32);

		vkCmdBindDescriptorSets(commandBuffer,
						VK_PIPELINE_BIND_POINT_GRAPHICS,
						P1.pipelineLayout, 0, 1, &DSRightFlipper.descriptorSets[currentImage],
						0, nullptr);
						
		vkCmdDrawIndexed(commandBuffer,
					static_cast<uint32_t>(RightFlipper.indices.size()), 1, 0, 0, 0);

		VkBuffer vertexBuffersLeftBumper[] = {LeftBumper.vertexBuffer};
		VkDeviceSize offsetsLeftBumper[] = {0};
		vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffersLeftBumper, offsetsLeftBumper);
		vkCmdBindIndexBuffer(commandBuffer, LeftBumper.indexBuffer, 0,
								VK_INDEX_TYPE_UINT32);

		vkCmdBindDescriptorSets(commandBuffer,
						VK_PIPELINE_BIND_POINT_GRAPHICS,
						P1.pipelineLayout, 0, 1, &DSLeftBumper.descriptorSets[currentImage],
						0, nullptr);
						
		vkCmdDrawIndexed(commandBuffer,
					static_cast<uint32_t>(LeftBumper.indices.size()), 1, 0, 0, 0);

		VkBuffer vertexBuffersCentreBumper[] = {CentreBumper.vertexBuffer};
		VkDeviceSize offsetsCentreBumper[] = {0};
		vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffersCentreBumper, offsetsCentreBumper);
		vkCmdBindIndexBuffer(commandBuffer, CentreBumper.indexBuffer, 0,
								VK_INDEX_TYPE_UINT32);

		vkCmdBindDescriptorSets(commandBuffer,
						VK_PIPELINE_BIND_POINT_GRAPHICS,
						P1.pipelineLayout, 0, 1, &DSCentreBumper.descriptorSets[currentImage],
						0, nullptr);
						
		vkCmdDrawIndexed(commandBuffer,
					static_cast<uint32_t>(CentreBumper.indices.size()), 1, 0, 0, 0);

		VkBuffer vertexBuffersRightBumper[] = {RightBumper.vertexBuffer};
		VkDeviceSize offsetsRightBumper[] = {0};
		vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffersRightBumper, offsetsRightBumper);
		vkCmdBindIndexBuffer(commandBuffer, RightBumper.indexBuffer, 0,
								VK_INDEX_TYPE_UINT32);

		vkCmdBindDescriptorSets(commandBuffer,
						VK_PIPELINE_BIND_POINT_GRAPHICS,
						P1.pipelineLayout, 0, 1, &DSRightBumper.descriptorSets[currentImage],
						0, nullptr);
						
		vkCmdDrawIndexed(commandBuffer,
					static_cast<uint32_t>(RightBumper.indices.size()), 1, 0, 0, 0);

		VkBuffer vertexBuffersFloor[] = {Floor.vertexBuffer};
		VkDeviceSize offsetsFloor[] = {0};
		vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffersFloor, offsetsFloor);
		vkCmdBindIndexBuffer(commandBuffer, Floor.indexBuffer, 0,
								VK_INDEX_TYPE_UINT32);

		vkCmdBindDescriptorSets(commandBuffer,
						VK_PIPELINE_BIND_POINT_GRAPHICS,
						P1.pipelineLayout, 0, 1, &DSFloor.descriptorSets[currentImage],
						0, nullptr);
						
		vkCmdDrawIndexed(commandBuffer,
					static_cast<uint32_t>(Floor.indices.size()), 1, 0, 0, 0);

					
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
		
		glm::mat4 BodyPositionRotation = glm::mat4(1.0f)/* glm::rotate(glm::mat4(1.0f),
								glm::radians(45.0f),
								glm::vec3(1.0f, 0.0f, 0.0f)) */;
		//glm::mat4 BodyPosition = glm::translate(glm::mat4(1.0f),glm::vec3(0.0f, 5.0f, 0.0f))*BodyPositionRotation;
		glm::mat4 BodyPosition = glm::mat4(1.0f);

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
		
		static float rightZMargin = -2.47737f;
		static float leftZMargin = 2.54627f;
		static float topXMargin = -5.45043f;
		static float bottomXMargin = 4.26388f;
		static float topYMargin = 2.0f;
		static float bottomYMargin = 0.0f;
		static float leftFlipperMargin = 1.0f;
		static float rightFlipperMargin = -1.2f;

		float FlipperBottomLeftX = 0.174519f;
		float FlipperBottomLeftZ = 0.167087f;
		float FlipperBottomRightX = 0.174519f;
		float FlipperBottomRightZ = -0.167087f;
		float FlipperTopLeftX = -0.919164f;
		float FlipperTopLeftZ = 0.092294f;
		float FlipperTopRightX = -0.919164f;
		float FlipperTopRightZ = -0.092294f;


		static float ballX = 0.0f;
		static float ballY = 0.0f;
		static float ballZ = 0.0f;
		static float ballXstart = 3.5f;
		static float ballZstart = -2.0f;
		
		//definition of physics interacting objects

		static MovingRotationalObjectDimensions Ball;

		static float leftFlipperRotate = 0.0f;
		static float rightFlipperRotate = 0.0f;
		static float gravityconstant = 1.0f;
		static float activeGravity = 0.0f;
		
		static auto previousReleaseValueOfSpace = GLFW_RELEASE;

		bool leftFlipperMovingUp, leftFlipperMovingDown;

		Wall TopWall(topXMargin-sideWallDepth, topXMargin, bottomYMargin, 	topYMargin, rightZMargin, leftZMargin, 3.14/2, "top-wall", BodyPosition);
		
		Wall RightWall(topXMargin, bottomXMargin, bottomYMargin, topYMargin, rightZMargin-sideWallDepth, rightZMargin, 0.0f, "right-wall", BodyPosition);

		Wall LeftWall(topXMargin, bottomXMargin, bottomYMargin, topYMargin, leftZMargin, leftZMargin+sideWallDepth, 0.0f, "right-wall", BodyPosition);
		
		Wall BottomRightWall(bottomXMargin, bottomXMargin+sideWallDepth, bottomYMargin, topYMargin, rightZMargin, rightFlipperMargin, -3.14/2, "bottom-right-wall", BodyPosition);

		Wall BottomLeftWall(bottomXMargin, bottomXMargin+sideWallDepth, bottomYMargin, topYMargin, leftFlipperMargin, leftZMargin, -3.14/2, "bottom-right-wall", BodyPosition);
				
		Flipper LeftFlipperTest(FlipperBottomLeftX, FlipperBottomLeftZ, FlipperBottomRightX, FlipperBottomRightZ, FlipperTopLeftX, FlipperTopLeftZ, FlipperTopRightX, FlipperTopRightZ, leftFlipperRotate-120.0f, "left-flipper",bottomXMargin-0.7f, ballRadius,leftFlipperMargin,BodyPosition);

		Flipper RightFlipperTest(FlipperBottomLeftX, FlipperBottomLeftZ, FlipperBottomRightX, FlipperBottomRightZ, FlipperTopLeftX, FlipperTopLeftZ, FlipperTopRightX, FlipperTopRightZ, rightFlipperRotate+120.0f, "left-flipper",bottomXMargin-0.7f, ballRadius,rightFlipperMargin, BodyPosition);
		
		LeftFlipperTest.setFlipperMovingDown(false);
		LeftFlipperTest.setFlipperMovingUp(false);
		RightFlipperTest.setFlipperMovingDown(false);
		RightFlipperTest.setFlipperMovingUp(false);

		Bumper LeftBumperTest(/* radius input */0.2f, "left-bumper", 0.0f, 1.5f, BodyPosition);

		Bumper CentreBumperTest(/* radius input */0.2f, "centre-bumper", -1.0f, 0.0f, BodyPosition);

		Bumper RightBumperTest(/* radius input */0.2f, "right-bumper", 0.0f, -1.5f, BodyPosition);

		
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
				ballX = 0.0f; 
				ballY = 0.0f;
				ballZ = 0.0f;
				activeGravity = 0.0f;
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
				launchBallSpeed = -pullerDistanceCovered*7.0f;
				Ball.speedX = launchBallSpeed;
				// next line must be cancelled, only to test right wall
				Ball.speedZ = Ball.speedX;
				Ball.speedY = 0.0f;
				activeGravity = 1.0f;

			}

			previousReleaseValueOfSpace = glfwGetKey(window,GLFW_KEY_SPACE);
		}

		/* left flipper rotate */
		{
			if(glfwGetKey(window,GLFW_KEY_G)){

				if(leftFlipperRotate==60.0f){
					LeftFlipperTest.setFlipperMovingUp(false);
					LeftFlipperTest.setFlipperMovingDown(false);
				}
				if(leftFlipperRotate<60.0f){
					leftFlipperRotate = leftFlipperRotate+flipperRotateSpeed*deltaT;
					LeftFlipperTest.setFlipperMovingUp(true);
				}
			}else{
				if(leftFlipperRotate==0.0f){
					LeftFlipperTest.setFlipperMovingUp(false);
					LeftFlipperTest.setFlipperMovingDown(false);					
				}
				if(leftFlipperRotate>0.0f){
					leftFlipperRotate = leftFlipperRotate-flipperRotateSpeed*deltaT;
					LeftFlipperTest.setFlipperMovingDown(true);			
					
				}
			}

		}
		

		/* right flipper rotate */
		{
			if(glfwGetKey(window,GLFW_KEY_H)){
				
				if(rightFlipperRotate>=-60.0f){
					rightFlipperRotate = rightFlipperRotate-flipperRotateSpeed*deltaT;
					
				}
			}else{
				if(rightFlipperRotate<=0.0f){
					rightFlipperRotate = rightFlipperRotate+flipperRotateSpeed*deltaT;
					
				}
			}

		}
		
		if(glfwGetKey(window,GLFW_KEY_I)){
				ballX = ballX-2.0f*deltaT;
		}
		if(glfwGetKey(window,GLFW_KEY_K)){
			ballX = ballX+2.0f*deltaT;
		}
		if(glfwGetKey(window,GLFW_KEY_J)){
			ballZ = ballZ+2.0f*deltaT;
		}
		if(glfwGetKey(window,GLFW_KEY_L)){
			ballZ = ballZ-2.0f*deltaT;
		}

		 

		
		Ball.speedY = 0.0f;
		Ball.speedX = Ball.speedX+activeGravity*gravityconstant*deltaT;
		glm::vec4 BallSpeedWRTBody = BodyPositionRotation*glm::vec4(Ball.speedX,Ball.speedY,Ball.speedZ,1.0f);

		// Update of ball speed in the coordinates of the body
		ballX = ballX+BallSpeedWRTBody.x*deltaT;
		ballY = ballY+BallSpeedWRTBody.y*deltaT;
		ballZ = ballZ+BallSpeedWRTBody.z*deltaT;

		// Positions of the objects when they are models

		glm::mat4 PullerCurrentPosition = glm::translate(glm::mat4(1.0f),glm::vec3(ballXstart+2.5f+pullerDistanceCovered, ballRadius, ballZstart))*BodyPosition;

		glm::vec3 BallReady = glm::vec3(ballXstart+ballRadius, ballRadius, ballZstart);
		glm::mat4 BallCurrentPosition = glm::translate(glm::mat4(1.0f),glm::vec3(BallReady.x+ballX, BallReady.y+ballY, BallReady.z+ballZ))*BodyPosition;
		glm::vec4 BallCurrentPositionvector = BallCurrentPosition*glm::vec4(1.0f,1.0f,1.0f,1.0f);
		BallCurrentPositionvector = glm::vec4(BallCurrentPositionvector.x-1.0f,BallCurrentPositionvector.y-1.0f,BallCurrentPositionvector.z-1.0f,BallCurrentPositionvector.w-1.0f);


		Ball.originX = BallCurrentPositionvector.x;
		Ball.originY = BallCurrentPositionvector.y;
		Ball.originZ = BallCurrentPositionvector.z;

		glm::vec4 BodyPositionVector = BodyPosition*glm::vec4(1.0f,1.0f,1.0f,1.0f);
		//std::cout<< "result X:"<<result.x<< " Y "<< result.y<<" Z "<< result.z<<std::endl;
		BodyPositionVector = glm::vec4(BodyPositionVector.x-1.0f,BodyPositionVector.y-1.0f,BodyPositionVector.z-1.0f,BodyPositionVector.w-1.0f);

		/* std::cout<< "rightwall X: min"<<RightWall.minX<< " max "<< RightWall.maxX <<" min Z "<< RightWall.minZ <<" max  " <<RightWall.maxZ <<std::endl;
		std::cout<< "rightwall Y: min"<<RightWall.minY<< " max "<< RightWall.minY<<std::endl; */

		std::cout<< "ball origin X:"<<Ball.originX<< " Y "<< Ball.originY<<" Z "<< Ball.originZ<<std::endl;
		/* std::cout<< "right wall Y: min"<<RightWall.minY<< " max "<< RightWall.maxY<<" Z "<< Ball.originZ<<std::endl;
		std::cout<< "ball speed X:"<<Ball.speedX<< " Y "<< Ball.speedY<<" Z "<< Ball.speedZ<<std::endl;
		std::cout<< "ball speed in Global reference X:"<<BallSpeedWRTBody.x<< " Y "<< BallSpeedWRTBody.y<<" Z "<< BallSpeedWRTBody.z<<std::endl; */




		Ball = RightWall.bounceBall(Ball);
		Ball = TopWall.bounceBall(Ball);
		Ball = LeftWall.bounceBall(Ball);
		Ball = BottomRightWall.bounceBall(Ball);
		Ball = BottomLeftWall.bounceBall(Ball);
		Ball = LeftFlipperTest.bounceBall(Ball);
		Ball = RightFlipperTest.bounceBall(Ball);
		Ball = LeftBumperTest.bounceBall(Ball);
		Ball = CentreBumperTest.bounceBall(Ball);
		Ball = RightBumperTest.bounceBall(Ball);

		

		
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

		static float time2 = 0;
		time2 = time2 + deltaT;

		ubo.lightDir = glm::vec3(-0.4830f, 0.8365f, -0.2588f);
		/* glm::vec3(cos(glm::radians(0.0f))  * cos(glm::radians(30.0f*time2)) , sin(glm::radians(90.0f)), 	cos(glm::radians(90.0f))  * sin(glm::radians(30.0f*time2)) ); */
		ubo.lightPos = glm::vec3(3.0f, 5.0f, 0.0f);
		ubo.lightColor = glm::vec4(1.0f, 1.0f, 1.0f, 1.0f);
		ubo.eyePos = glm::vec3(5.0f, 3.0f, 0.0f);
		ubo.coneInOutDecayExp = glm::vec4(0.9f, 0.92f, 2.0f, 1.0f);
		
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

		//Left Flipper Position
		ubo.model = LeftFlipperTest.getResultingTransformationMatrix();
					
		vkMapMemory(device, DSLeftFlipper.uniformBuffersMemory[0][currentImage], 0,
							sizeof(ubo), 0, &data);
		memcpy(data, &ubo, sizeof(ubo));
		vkUnmapMemory(device, DSLeftFlipper.uniformBuffersMemory[0][currentImage]);

		//Right Flipper Position
		ubo.model = RightFlipperTest.getResultingTransformationMatrix();
					
		vkMapMemory(device, DSRightFlipper.uniformBuffersMemory[0][currentImage], 0,
							sizeof(ubo), 0, &data);
		memcpy(data, &ubo, sizeof(ubo));
		vkUnmapMemory(device, DSRightFlipper.uniformBuffersMemory[0][currentImage]);

		// Left Bumper
		ubo.model = LeftBumperTest.getResultingTransformationMatrix();
					
		vkMapMemory(device, DSLeftBumper.uniformBuffersMemory[0][currentImage], 0,
							sizeof(ubo), 0, &data);
		memcpy(data, &ubo, sizeof(ubo));
		vkUnmapMemory(device, DSLeftBumper.uniformBuffersMemory[0][currentImage]);

		// Centre Bumper
		ubo.model = CentreBumperTest.getResultingTransformationMatrix();
					
		vkMapMemory(device, DSCentreBumper.uniformBuffersMemory[0][currentImage], 0,
							sizeof(ubo), 0, &data);
		memcpy(data, &ubo, sizeof(ubo));
		vkUnmapMemory(device, DSCentreBumper.uniformBuffersMemory[0][currentImage]);

		// Right Bumper
		ubo.model = RightBumperTest.getResultingTransformationMatrix();
					
		vkMapMemory(device, DSRightBumper.uniformBuffersMemory[0][currentImage], 0,
							sizeof(ubo), 0, &data);
		memcpy(data, &ubo, sizeof(ubo));
		vkUnmapMemory(device, DSRightBumper.uniformBuffersMemory[0][currentImage]);

		ubo.model = glm::translate(glm::mat4(1.0f),glm::vec3(0.0f,-3.0f,0.0f));
					
		vkMapMemory(device, DSFloor.uniformBuffersMemory[0][currentImage], 0,
							sizeof(ubo), 0, &data);
		memcpy(data, &ubo, sizeof(ubo));
		vkUnmapMemory(device, DSFloor.uniformBuffersMemory[0][currentImage]);
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