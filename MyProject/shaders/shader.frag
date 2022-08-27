#version 450

layout(binding = 1) uniform sampler2D texSampler;

layout(location = 0) in vec3 fragViewDir;
layout(location = 1) in vec3 fragNorm;
layout(location = 2) in vec2 fragTexCoord;
layout(location = 3) in vec3 fragPos;

layout(location = 0) out vec4 outColor;

layout(binding = 0) uniform UniformBufferObject {
	mat4 model;
	mat4 view;
	mat4 proj;
	vec3 lightDir;
	vec3 lightPos;
	vec3 lightColor;
	vec3 eyePos;
	vec4 coneInOutDecayExp;
}ubo;

vec3 direct_light_dir(vec3 pos) {
	// Directional light direction
	return vec3(ubo.lightDir.x, ubo.lightDir.y, ubo.lightDir.z);
}

vec3 direct_light_color(vec3 pos) {
	// Directional light color
	return vec3(ubo.lightColor.x,ubo.lightColor.y ,ubo.lightColor.z );
}

vec3 point_light_dir(vec3 pos) {
	// Point light direction
	vec3 direction = vec3(ubo.lightPos-pos);
	return normalize(direction);
}

vec3 point_light_color(vec3 pos) {
	// Point light color
	float g = ubo.coneInOutDecayExp.z;
	vec3 direction = vec3(ubo.lightPos-pos);
	float distance = length(direction);
	float decay = ubo.coneInOutDecayExp.w;
	float lightModel = pow(g/distance,decay);
	return vec3(ubo.lightColor.x*lightModel,ubo.lightColor.y*lightModel ,ubo.lightColor.z*lightModel );
}

void main() {
	const vec3  diffColor = texture(texSampler, fragTexCoord).rgb;
	const vec3  specColor = vec3(1.0f, 1.0f, 1.0f);
	const float specPower = 150.0f;
	const vec3  L = vec3(-0.4830f, 0.8365f, -0.2588f);
	
	vec3 N = normalize(fragNorm);
	vec3 R = -reflect(L, N);
	vec3 V = normalize(fragViewDir);
	
	vec3 lD = point_light_dir(fragPos);

	vec3 lC = point_light_color(fragPos);
	// Lambert diffuse
	vec3 diffuse  = diffColor * max(dot(N, lD),0.0f);
	//
	// Phong specular
	vec3 specular = specColor * pow(max(dot(R,V), 0.0f), specPower);
	// Hemispheric ambient
	vec3 ambient  = (vec3(0.1f,0.1f, 0.1f) * (1.0f + N.y) + vec3(0.0f,0.0f, 0.1f) * (1.0f - N.y)) * diffColor;
	
	outColor = vec4(ambient + diffuse, 1.0f);
}