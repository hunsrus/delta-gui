#version 100

precision mediump float;

// Input vertex attributes (from vertex shader)
varying vec2 fragTexCoord;

// Input uniform values
uniform sampler2D texture0;

// Customizable edge and background colors
uniform vec2 resolution;
uniform vec3 edgeColor;      // Color for edges
uniform vec3 backgroundColor; // Color for background

void main()
{
    float x = 1.0 / resolution.x;
    float y = 1.0 / resolution.y;

    vec3 horizEdge = vec3(0.0);
    horizEdge -= texture2D(texture0, vec2(fragTexCoord.x - x, fragTexCoord.y - y)).rgb * 1.0;
    horizEdge -= texture2D(texture0, vec2(fragTexCoord.x - x, fragTexCoord.y    )).rgb * 2.0;
    horizEdge -= texture2D(texture0, vec2(fragTexCoord.x - x, fragTexCoord.y + y)).rgb * 1.0;
    horizEdge += texture2D(texture0, vec2(fragTexCoord.x + x, fragTexCoord.y - y)).rgb * 1.0;
    horizEdge += texture2D(texture0, vec2(fragTexCoord.x + x, fragTexCoord.y    )).rgb * 2.0;
    horizEdge += texture2D(texture0, vec2(fragTexCoord.x + x, fragTexCoord.y + y)).rgb * 1.0;

    vec3 vertEdge = vec3(0.0);
    vertEdge -= texture2D(texture0, vec2(fragTexCoord.x - x, fragTexCoord.y - y)).rgb * 1.0;
    vertEdge -= texture2D(texture0, vec2(fragTexCoord.x    , fragTexCoord.y - y)).rgb * 2.0;
    vertEdge -= texture2D(texture0, vec2(fragTexCoord.x + x, fragTexCoord.y - y)).rgb * 1.0;
    vertEdge += texture2D(texture0, vec2(fragTexCoord.x - x, fragTexCoord.y + y)).rgb * 1.0;
    vertEdge += texture2D(texture0, vec2(fragTexCoord.x    , fragTexCoord.y + y)).rgb * 2.0;
    vertEdge += texture2D(texture0, vec2(fragTexCoord.x + x, fragTexCoord.y + y)).rgb * 1.0;

    vec3 edge = sqrt((horizEdge * horizEdge) + (vertEdge * vertEdge));

    // Convert to grayscale by averaging the color channels
    float intensity = dot(edge, vec3(1.0, 1.0, 1.0)) / 3.0;

    // Mix edge and background colors based on the intensity
    vec3 finalEdgeColor = mix(backgroundColor, edgeColor, intensity);

    gl_FragColor = vec4(finalEdgeColor, texture2D(texture0, fragTexCoord).a);
}
