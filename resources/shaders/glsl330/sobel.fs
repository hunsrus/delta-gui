#version 330

// Input vertex attributes (from vertex shader)
in vec2 fragTexCoord;
in vec4 fragColor;

// Input uniform values
uniform sampler2D texture0;
uniform vec4 colDiffuse;

// Output fragment color
out vec4 finalColor;

// NOTE: Add here your custom variables
uniform vec2 resolution = vec2(1000, 1000);

void main()
{
    float x = 1.0/resolution.x;
    float y = 1.0/resolution.y;

    vec4 horizEdge = vec4(0.0);
    horizEdge -= texture2D(texture0, vec2(fragTexCoord.x - x, fragTexCoord.y - y))*1.0;
    horizEdge -= texture2D(texture0, vec2(fragTexCoord.x - x, fragTexCoord.y    ))*2.0;
    horizEdge -= texture2D(texture0, vec2(fragTexCoord.x - x, fragTexCoord.y + y))*1.0;
    horizEdge += texture2D(texture0, vec2(fragTexCoord.x + x, fragTexCoord.y - y))*1.0;
    horizEdge += texture2D(texture0, vec2(fragTexCoord.x + x, fragTexCoord.y    ))*2.0;
    horizEdge += texture2D(texture0, vec2(fragTexCoord.x + x, fragTexCoord.y + y))*1.0;

    vec4 vertEdge = vec4(0.0);
    vertEdge -= texture2D(texture0, vec2(fragTexCoord.x - x, fragTexCoord.y - y))*1.0;
    vertEdge -= texture2D(texture0, vec2(fragTexCoord.x    , fragTexCoord.y - y))*2.0;
    vertEdge -= texture2D(texture0, vec2(fragTexCoord.x + x, fragTexCoord.y - y))*1.0;
    vertEdge += texture2D(texture0, vec2(fragTexCoord.x - x, fragTexCoord.y + y))*1.0;
    vertEdge += texture2D(texture0, vec2(fragTexCoord.x    , fragTexCoord.y + y))*2.0;
    vertEdge += texture2D(texture0, vec2(fragTexCoord.x + x, fragTexCoord.y + y))*1.0;

    vec3 edge = sqrt((horizEdge.rgb*horizEdge.rgb) + (vertEdge.rgb*vertEdge.rgb));

    // Texel color fetching from texture sampler
    vec4 texelColor = texture(texture0, fragTexCoord)*colDiffuse*fragColor;

    // Convert texel color to grayscale using NTSC conversion weights
    float gray = dot(edge.rgb, vec3(1.0, 1.0, 1.0));
    edge = vec3(gray, gray, gray);

    finalColor = vec4(edge, texture2D(texture0, fragTexCoord).a);
}