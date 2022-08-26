CCFLAGS = -std=c++17
LDFLAGS = -lglfw -lvulkan -ldl -lpthread -lX11 -L/lib64 -lXrandr -lXi
INC=-I./headers
VulkanTest: MyProject.cpp
	g++ $(CFLAGS) $(INC) -o Vulkan MyProject.cpp $(LDFLAGS)

.PHONY: test clean

test: Vulkan
	g++ $(CFLAGS) $(INC) -o Vulkan MyProject.cpp $(LDFLAGS)
	./Vulkan

gdb: Vulkan
	gdb ./Vulkan


clean:
	rm -f Vulkan

comp:
	glslc ./shaders/shader.frag -o ./shaders/frag.spv	
	glslc ./shaders/shader.vert -o ./shaders/vert.spv	
	./Vulkan
