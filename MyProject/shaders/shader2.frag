#version 450
#extension GL_ARB_separate_shader_objects : enable

layout(binding = 1) uniform sampler2D texSampler;

layout(binding = 0) uniform GlobalUniformBufferObject {
	// Direct lights parameters (for diffuse and specular)
    //pointlight
	vec3 lightDir;
    vec3 lighPos;
	vec3 lightColor;
    vec3 spotlightColor;
    vec3 spotlightDir;
	vec3 eyePos;
    vec4 coneInOutDecayExp;
	vec4 selector;
} gubo;


/**** Modify from here *****/

//vec2 fragTexCoord;
//vec3 fragNorm;
//vec3 fragPos;

//vec4 outColor;

layout(location = 0) in vec3 fragViewDir;
layout(location = 1) in vec3 fragNorm;
layout(location = 2) in vec2 fragTexCoord;
layout(location = 3) in vec3 fragPos;

layout(location = 0) out vec4 outColor;

vec3 Oren_Nayar_Diffuse_and_Ambient(vec3 N, vec3 V, vec3 Cd, vec3 Ca, float sigma) {
	// Oren Nayar Diffuse + Ambient
	// No Specular
	// One directional light (lightDir and lightColor0)
	//
	// Parameters are:
	//
	// vec3 N : normal vector
	// vec3 V : view direction
	// vec3 Cd : main color (diffuse color)
	// vec3 Ca : ambient color
	// float sigma : roughness of the material
	
	float theta_i = acos(dot(gubo.lightDir,N));
	float theta_r = acos(dot(V,N));
	float alpha = max(theta_i,theta_r);
	float beta = min(theta_i,theta_r);
	vec3 ni_i = normalize(gubo.lightDir-dot(gubo.lightDir,N)*N);
	vec3 ni_r = normalize(V-dot(V,N)*N);
	float G = max(0,dot(ni_i,ni_r));
	float clampValue = dot(gubo.lightDir,N);
	clampValue = max(clampValue,0.0);
	clampValue = min(clampValue,1.0);
	float A = 1- 0.5 * (pow(sigma,2)/(pow(sigma,2)+0.33));
	float B = 0.45 * (pow(sigma,2)/(pow(sigma,2)+0.09));
	vec3 Oren_Nayar =  Cd * clampValue * (A+B*G * sin(alpha)*tan(beta) ) + Ca ;
	return Oren_Nayar;
}

vec3 Lambert(vec3 N, vec3 V, vec3 Cd, vec3 Ca) {
	// Lambert Diffuse
	// No Specular
	// One directional light (lightDir0 and lightColor0)
	// Hemispheric light oriented along the y-axis
	//
	// Parameters are:
	//
	// vec3 N : normal vector
	// vec3 V : view direction
	// vec3 Cd : main color (diffuse color)
	// vec3 Ca : ambient color

	
	float clampValue = dot(gubo.lightDir0,N);
	clampValue = max(clampValue,0.0);
	clampValue = min(clampValue,1.0);
	vec3 lambertDiffuse = gubo.lightColor0 * Cd * clampValue;


	return lambertDiffuse;	
}

vec3 blinn_specular(vec3 N, vec3 V, vec3 Cs, vec3 Ca, float gamma,lightDir,lightColor)  {
	// Spherical Harmonics
	// Blinn Specular
	// Four directional lights (lightDir0 to lightDir3, and lightColor0 to lightColor3)
	//
	// Parameters are:
	//
	// vec3 N : normal vector
	// vec3 V : view direction
	// vec3 Cs : specular color
	// vec3 Ca : ambient color
	// float gamma : Blinn exponent
	
	vec3 halfVector;
	float clampValue;
	float sumOfLights = 0;
	halfVector = normalize(lightDir+V);
	clampValue = dot(N,halfVector);
	clampValue = max(clampValue,0.0);
	clampValue = min(clampValue,1.0);
	Light = pow(clampValue,gamma);
	

	vec3 blinnSpecular = Cs * Light;

	return blinnSpecular;
}



/**** To here *****/




void main() {
	//vec3 Norm = normalize(fragNorm);
	//vec3 EyeDir = normalize(gubo.eyePos.xyz - fragPos);
	
	//float AmbFact = 0.025;
	
	//vec3 DifCol = texture(texSampler, fragTexCoord).rgb * gubo.selector.w +
	//			  vec3(1,1,1) * (1-gubo.selector.w);

	//vec3 CompColor = gubo.selector.x *
						Oren_Nayar_Diffuse_and_Ambient(Norm, EyeDir, DifCol, DifCol, 1.2f) +
	//				 gubo.selector.y *
	//					Lambert(Norm, EyeDir, DifCol, DifCol) +
	//				 gubo.selector.z *
	//					blinn_specular(Norm, EyeDir, vec3(1.0f,1.0f,1.0f,lightDir, lightColor), DifCol, 200.0f);
	
	//outColor = vec4(CompColor, 1.0f);	
}