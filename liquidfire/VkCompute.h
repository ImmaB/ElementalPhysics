
#ifndef VK_COMPUTE_H
#define VK_COMPUTE_H

#include <Box2D/Common/b2Math.h>
#include <vulkan/vulkan.h>
#include <vector>
#include <fstream>
#include <assert.h>
#include <stdexcept>

#ifdef NDEBUG
const bool enableValidationLayers = false;
#else
const bool enableValidationLayers = true;
#endif
const int WIDTH = 3200; // Size of rendered mandelbrot set.
const int HEIGHT = 2400; // Size of renderered mandelbrot set.
uint32 m_workGroupSize = 32; // Workgroup size in compute shader.

#pragma once
class VkCompute
{
public:
	VkCompute();
	~VkCompute();

private:
	// The pixels of the rendered mandelbrot set are in this format:
	struct Pixel {
		float r, g, b, a;
	};

	/*
	In order to use Vulkan, you must create an instance.
	*/
	VkInstance instance;

	VkDebugReportCallbackEXT debugReportCallback;
	/*
	The physical device is some device on the system that supports usage of Vulkan.
	Often, it is simply a graphics card that supports Vulkan.
	*/
	VkPhysicalDevice physicalDevice;
	/*
	Then we have the logical device VkDevice, which basically allows
	us to interact with the physical device.
	*/
	VkDevice device;

	/*
	The pipeline specifies the pipeline that all graphics and compute commands pass though in Vulkan.

	We will be creating a simple compute pipeline in this application.
	*/
	VkPipeline pipeline;
	VkPipelineLayout pipelineLayout;
	VkShaderModule computeShaderModule;

	/*
	The command buffer is used to record commands, that will be submitted to a queue.

	To allocate such command buffers, we use a command pool.
	*/
	VkCommandPool commandPool;
	VkCommandBuffer commandBuffer;

	/*

	Descriptors represent resources in shaders. They allow us to use things like
	uniform buffers, storage buffers and images in GLSL.

	A single descriptor represents a single resource, and several descriptors are organized
	into descriptor sets, which are basically just collections of descriptors.
	*/
	VkDescriptorPool descriptorPool;
	VkDescriptorSet descriptorSet;
	VkDescriptorSetLayout descriptorSetLayout;

	/*
	The mandelbrot set will be rendered to this buffer.

	The memory that backs the buffer is bufferMemory.
	*/
	VkBuffer posXbuffer;
	VkDeviceMemory posXbufferMemory;
	VkBuffer posYbuffer;
	VkDeviceMemory posYbufferMemory;
	VkBuffer posZbuffer;
	VkDeviceMemory posZbufferMemory;

	uint32 bufferSize; // size of `buffer` in bytes.

	std::vector<const char *> enabledLayers;

	/*
	In order to execute commands on a device(GPU), the commands must be submitted
	to a queue. The commands are stored in a command buffer, and this command buffer
	is given to the queue.

	There will be different kinds of queues on the device. Not all queues support
	graphics operations, for instance. For this application, we at least want a queue
	that supports compute operations.
	*/
	VkQueue queue; // a queue supporting compute operations.

	/*
	Groups of queues that have the same capabilities(for instance, they all supports graphics and computer operations),
	are grouped into queue families.

	When submitting a command buffer, you must specify to which queue in the family you are submitting to.
	This variable keeps track of the index of that queue in its family.
	*/
	uint32 queueFamilyIndex;

	void run();
	void saveRenderedImage();
	void createInstance();
	void findPhysicalDevice();
	uint32 getComputeQueueFamilyIndex();
	void createDevice();
	uint32 findMemoryType(uint32 memoryTypeBits, VkMemoryPropertyFlags properties);
	void createBuffer();
	void createDescriptorSetLayout();
	void createDescriptorSet();
	std::vector<char> readFile(const std::string& fileName);
	void createComputePipeline();
	void createShaderModule(const std::vector<char>& code, VkShaderModule* shaderModule);
	void createCommandBuffer();
	void runCommandBuffer();

public:
	void SetParticleBuffers(uint32 count,
		float32* posXData, float32* posYData, float32* posZData);

	void GetParticleBuffers(uint32 count,
		float32* posXData, float32* posYData, float32* posZData);
	void ComputePositions();
};


#endif