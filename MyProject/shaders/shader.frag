#version 450

layout(set = 0, binding = 1) uniform sampler2D texSampler;

layout(location = 0) in vec3 fragViewDir;
layout(location = 1) in vec3 fragNorm;
layout(location = 2) in vec2 fragTexCoord;
layout(location = 3) in vec3 fragPos;

layout(location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform GlobalUniformBufferObject {

	//point light overhead
	vec3 lightDir;
	vec3 lightPos;
	vec3 lightColor;

	// eye position
	vec3 eyePos;
	
	//spotlight 
	vec3 spotlightColor;
	vec3 spotlightDir;
	vec4 coneInOutDecayExp;
}gubo;



// direct light not used because not appropriate
vec3 direct_light_dir(vec3 pos) {
	// Directional light direction
	return vec3(gubo.lightDir.x, gubo.lightDir.y, gubo.lightDir.z);
}

vec3 direct_light_color(vec3 pos) {
	// Directional light color
	return vec3(gubo.lightColor.x,gubo.lightColor.y ,gubo.lightColor.z );
}

vec3 spot_light_dir(vec3 pos, vec3 lightPosition) {
	// Spot light direction
	vec3 direction = vec3(lightPosition-pos);
	return normalize(direction);
}

vec3 spot_light_color(vec3 pos, vec3 lightPosition, vec3 lightDirection, vec3 lightColor, vec4 coneInOutDecayExp) {
	// Spot light color
	float g = coneInOutDecayExp.z;
	vec3 direction = vec3(lightPosition-pos);
	float cosAngle = dot(normalize(direction),lightDirection);
	float clampValue = (cosAngle-coneInOutDecayExp.x)/(coneInOutDecayExp.y-coneInOutDecayExp.x);
	clampValue = max(clampValue,0.0);
	clampValue = min(clampValue,1.0);
	float distance = length(direction);
	float decay = coneInOutDecayExp.w;
	float lightModel = pow(g/distance,decay)*clampValue;
	return vec3(lightColor.x*lightModel,lightColor.y*lightModel ,lightColor.z*lightModel );
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

	vec3 FragNorm = normalize(fragNorm);
	
	vec3 PointLightDirection = point_light_dir(fragViewDir);

	vec3 PointLightColor = point_light_color(fragViewDir);

	//vec3 SpotLightColor = spot_light_color(fragViewDir,gubo.spotlightDir,gubo.spotlightColor, gubo.coneInOutDecayExp)

	vec3 V = -reflect(PointLightDirection, FragNorm);
	// Lambert diffuse
	vec3 diffuse  = PointLightColor* diffColor * max(dot(FragNorm, PointLightDirection),0.0f);

	diffuse += PointLightColor* diffColor * max(dot(FragNorm, PointLightDirection),0.0f);

	// Phong specular
	vec3 specular = specColor * pow(max(dot(EyeDir,V), 0.0f), specPower);
	// Hemispheric ambient


	vec3 ambient  = (vec3(0.1f,0.1f, 0.1f) * (1.0f + FragNorm.y) + vec3(0.0f,0.0f, 0.1f) * (1.0f - FragNorm.y)) * diffColor;
	
	outColor = vec4(ambient + diffuse + specular, 1.0f);
}