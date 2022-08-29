#version 450

layout(binding = 1) uniform sampler2D texSampler;

layout(location = 0) in vec3 fragViewDir;
layout(location = 1) in vec3 fragNorm;
layout(location = 2) in vec2 fragTexCoord;
layout(location = 3) in vec3 fragPos;

layout(location = 0) out vec4 outColor;

layout(binding = 0) uniform GlobalUniformBufferObject {
	mat4 model;
	mat4 view;
	mat4 proj;
	vec3 lightDir;
	vec3 lightPos;
	vec3 lightColor;
	vec3 eyePos;
	vec4 coneInOutDecayExp;
}gubo;

vec3 direct_light_dir(vec3 pos) {
	// Directional light direction
	return vec3(gubo.lightDir.x, gubo.lightDir.y, gubo.lightDir.z);
}

vec3 direct_light_color(vec3 pos) {
	// Directional light color
	return vec3(gubo.lightColor.x,gubo.lightColor.y ,gubo.lightColor.z );
}

vec3 point_light_dir(vec3 pos) {
	// Point light direction
	vec3 direction = vec3(gubo.lightPos-pos);
	return normalize(direction);
}

vec3 point_light_color(vec3 pos) {
	// Point light color
	float g = gubo.coneInOutDecayExp.z;
	vec3 direction = vec3(gubo.lightPos-pos);
	float distance = length(direction);
	float decay = gubo.coneInOutDecayExp.w;
	float lightModel = pow(g/distance,decay);
	return vec3(gubo.lightColor.x*lightModel,gubo.lightColor.y*lightModel ,gubo.lightColor.z*lightModel );
}

void main() {
	const vec3  diffColor = texture(texSampler, fragTexCoord).rgb;
	const vec3  specColor = vec3(1.0f, 1.0f, 1.0f);
	const float specPower = 150.0f;
	vec3 EyeDir = normalize(gubo.eyePos.xyz - fragPos);

	vec3 N = normalize(fragNorm);
	vec3 R = EyeDir;
	
	vec3 lD = point_light_dir(fragPos);

	vec3 lC = point_light_color(fragPos);

	vec3 V = -reflect(lD, N);
	// Lambert diffuse
	vec3 diffuse  = diffColor * max(dot(N, lD),0.0f);
	//
	// Phong specular
	vec3 specular = specColor * pow(max(dot(R,V), 0.0f), specPower);
	// Hemispheric ambient
	vec3 ambient  = (vec3(0.1f,0.1f, 0.1f) * (1.0f + N.y) + vec3(0.0f,0.0f, 0.1f) * (1.0f - N.y)) * diffColor;
	
	outColor = vec4(ambient + diffuse + specular, 1.0f);
}