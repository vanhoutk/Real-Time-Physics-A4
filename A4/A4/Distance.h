#pragma once

#include <GL/glew.h>
#include <vector>	

#include "Antons_maths_funcs.h"

float getDistance(vec3 p1, vec3 p2)
{
	float x_sq = (p1.v[0] - p2.v[0]) * (p1.v[0] - p2.v[0]);
	float y_sq = (p1.v[1] - p2.v[1]) * (p1.v[1] - p2.v[1]);
	float z_sq = (p1.v[2] - p2.v[2]) * (p1.v[2] - p2.v[2]);
	return sqrt(x_sq + y_sq + z_sq);
}

float pointToPoint(vec3 p1, vec3 p2)
{
	return getDistance(p1, p2);
}

float pointToPoint(vec4 p1, vec4 p2)
{
	return pointToPoint(vec3(p1.v[0], p1.v[1], p1.v[2]), vec3(p2.v[0], p2.v[1], p2.v[2]));
}

float minimum(float a, float b)
{
	if (a < b)
	{
		return a;
	}
	else return b;
}

float minimum(float a, float b, float c)
{
	return minimum(minimum(a, b), c);
}

// Given a point p0 and edge <p1, p2>
float pointToEdge(vec3 p0, vec3 p1, vec3 p2)
{
	float p0p1 = pointToPoint(p0, p1);
	float p0p2 = pointToPoint(p0, p2);

	float p1p2 = pointToPoint(p1, p2);
	float p0_edge = (length(cross(p2 - p1, p0 - p1))) / p1p2;

	return minimum(p0p1, p0p2, p0_edge);
}

float pointToEdge(vec4 p0, vec4 p1, vec4 p2)
{
	return pointToEdge(vec3(p0.v[0], p0.v[1], p0.v[2]), 
					   vec3(p1.v[0], p1.v[1], p1.v[2]), 
					   vec3(p2.v[0], p2.v[1], p2.v[2]));
}

// For use when we don't know the closest feature
vec3 closestPointOnEdge(vec3 p0, vec3 p1, vec3 p2)
{
	float distance = pointToEdge(p0, p1, p2);
	if (distance == pointToPoint(p0, p1))
	{
		return p1;
	}
	else if (distance == pointToPoint(p0, p2))
	{
		return p2;
	}
	else
	{
		vec3 u_hat = normalise(p2 - p1); // Unit vector from p1 along edge
		vec3 point_on_edge = p1 - u_hat * (dot((p0 - p2), u_hat)) ; // Dot product gives distance along line
		return point_on_edge;
	}
}

vec3 closestPointOnEdgeVoronoi(vec3 p0, vec3 p1, vec3 p2)
{
	vec3 u_hat = normalise(p2 - p1); // Unit vector from p1 along edge
	vec3 point_on_edge = p1 + u_hat * (dot((p0 - p1), u_hat)); // Dot product gives distance along line
	return point_on_edge;
}

vec4 closestPointOnEdge(vec4 p0, vec4 p1, vec4 p2)
{
	vec3 result = closestPointOnEdge(vec3(p0.v[0], p0.v[1], p0.v[2]), 
									 vec3(p1.v[0], p1.v[1], p1.v[2]), 
									 vec3(p2.v[0], p2.v[1], p2.v[2]));
	return vec4(result.v[0], result.v[1], result.v[2], 0.0f);
}

vec4 closestPointOnEdgeVoronoi(vec4 p0, vec4 p1, vec4 p2)
{
	vec3 result = closestPointOnEdgeVoronoi(vec3(p0.v[0], p0.v[1], p0.v[2]), 
											vec3(p1.v[0], p1.v[1], p1.v[2]), 
											vec3(p2.v[0], p2.v[1], p2.v[2]));
	return vec4(result.v[0], result.v[1], result.v[2], 0.0f);
}

// Given a point p0 and a plane defined by a normal n_hat and some arbitrary point on the plane, p1;
float pointToPlane(vec3 p0, vec3 n_hat, vec3 p1)
{
	return dot(p0 - p1, n_hat);
}

float pointToPlane(vec4 p0, vec4 n_hat, vec4 p1)
{
	return pointToPlane(vec3(p0.v[0], p0.v[1], p0.v[2]), 
						vec3(n_hat.v[0], n_hat.v[1], n_hat.v[2]), 
						vec3(p1.v[0], p1.v[1], p1.v[2]));
}

vec3 closestPointOnPlane(vec3 p0, vec3 n_hat, vec3 p1)
{
	return p0 - n_hat * (dot(p0 - p1, n_hat));
}

vec4 closestPointOnPlane(vec4 p0, vec4 n_hat, vec4 p1)
{
	vec3 result = closestPointOnPlane(vec3(p0.v[0], p0.v[1], p0.v[2]), 
									  vec3(n_hat.v[0], n_hat.v[1], n_hat.v[2]), 
									  vec3(p1.v[0], p1.v[1], p1.v[2]));
	return vec4(result.v[0], result.v[1], result.v[2], 0.0f);
}

// Given a point p0 and a triangle defined by three vertices, p1, p2, p3
float pointToTriangleVoronoi(vec3 p0, vec3 p1, vec3 p2, vec3 p3)
{
	vec3 p0p1 = p0 - p1;
	vec3 p2p1 = p2 - p1;
	vec3 p3p1 = p3 - p1;
	vec3 p0p2 = p0 - p2;
	vec3 p1p2 = p1 - p2;
	vec3 p3p2 = p3 - p2;
	vec3 p0p3 = p0 - p3;
	vec3 p1p3 = p1 - p3;
	vec3 p2p3 = p2 - p3;
	
	if (dot(p0p1, p2p1) <= 0 && dot(p0p1, p3p1) <= 0)
	{
		// Voronoi region of p1
		return pointToPoint(p0, p1);
	}
	else if (dot(p0p2, p1p2) <= 0 && dot(p0p2, p3p2) <= 0)
	{
		// Voronoi region of p2
		return pointToPoint(p0, p2);
	}
	else if (dot(p0p3, p1p3) <= 0 && dot(p0p3, p2p3) <= 0)
	{
		// Voronoi region of p3
		return pointToPoint(p0, p2);
	}
	else if (dot(cross(cross(p3p2, p1p2), p2p1), p0p2) >= 0 &&
		dot(p0p1, p2p1) >= 0 && 
		dot(p0p2, p1p2) >= 0)
	{
		// Voronoi region of edge <p1, p2>
		return pointToEdge(p0, p1, p2);
	}
	else if (dot(cross(cross(p1p3, p2p3), p3p2), p0p3) >= 0 &&
		dot(p0p2, p3p2) >= 0 &&
		dot(p0p3, p2p3) >= 0)
	{
		// Voronoi region of edge <p2, p3>
		return pointToEdge(p0, p2, p3);
	}
	else if (dot(cross(cross(p2p1, p3p1), p1p3), p0p1) >= 0 &&
		dot(p0p3, p1p3) >= 0 &&
		dot(p0p1, p3p1) >= 0)
	{
		// Voronoi region of edge <p3, p1>
		return pointToEdge(p0, p3, p1);
	}
	else
	{
		// Voronoi region of triangle face <p1, p2, p3>
		vec3 n_hat = normalise(cross(p2p1, p3p1));
		return pointToPlane(p0, n_hat, p1);
	}
}

float pointToTriangleVoronoi(vec4 p0, vec4 p1, vec4 p2, vec4 p3)
{
	return pointToTriangleVoronoi(vec3(p0.v[0], p0.v[1], p0.v[2]),
								  vec3(p1.v[0], p1.v[1], p1.v[2]),
								  vec3(p2.v[0], p2.v[1], p2.v[2]),
								  vec3(p3.v[0], p3.v[1], p3.v[2]));
}

// Given a point p0 and a triangle defined by three vertices, p1, p2, p3
vec3 closestPointOnTriangleVoronoi(vec3 p0, vec3 p1, vec3 p2, vec3 p3)
{
	vec3 p0p1 = p0 - p1;
	vec3 p2p1 = p2 - p1;
	vec3 p3p1 = p3 - p1;
	vec3 p0p2 = p0 - p2;
	vec3 p1p2 = p1 - p2;
	vec3 p3p2 = p3 - p2;
	vec3 p0p3 = p0 - p3;
	vec3 p1p3 = p1 - p3;
	vec3 p2p3 = p2 - p3;

	if (dot(p0p1, p2p1) <= 0 && dot(p0p1, p3p1) <= 0)
	{
		// Voronoi region of p1
		return p1;
	}
	else if (dot(p0p2, p1p2) <= 0 && dot(p0p2, p3p2) <= 0)
	{
		// Voronoi region of p2
		return p2;
	}
	else if (dot(p0p3, p1p3) <= 0 && dot(p0p3, p2p3) <= 0)
	{
		// Voronoi region of p3
		return p3;
	}
	else if (dot(cross(p2p1, cross(p2p1, p3p1)), p0p1) >= 0 &&
		dot(p0p1, p2p1) >= 0 &&
		dot(p0p2, p1p2) >= 0)
	{
		// dot(cross(cross(p3p2, p1p2), p2p1), p0p2) >= 0
		// Voronoi region of edge <p1, p2>
		return closestPointOnEdgeVoronoi(p0, p1, p2);
	}
	else if (dot(cross(p3p2, cross(p3p2, p1p2)), p0p2) >= 0 &&
		dot(p0p2, p3p2) >= 0 &&
		dot(p0p3, p2p3) >= 0)
	{
		// dot(cross(cross(p1p3, p2p3), p3p2), p0p3) >= 0
		// Voronoi region of edge <p2, p3>
		return closestPointOnEdgeVoronoi(p0, p2, p3);
	}
	else if (dot(cross(p1p3, cross(p1p3, p2p3)), p0p3) >= 0 &&
		dot(p0p3, p1p3) >= 0 &&
		dot(p0p1, p3p1) >= 0)
	{
		// dot(cross(cross(p2p1, p3p1), p1p3), p0p1) >= 0
		// Voronoi region of edge <p3, p1>
		return closestPointOnEdgeVoronoi(p0, p3, p1);
	}
	else
	{
		// Voronoi region of triangle face <p1, p2, p3>
		vec3 n_hat = normalise(cross(p2p1, p3p1));
		return closestPointOnPlane(p0, n_hat, p1);
	}
}

// Given a point p0 and a triangle defined by three vertices, p1, p2, p3
vec4 closestPointOnTriangleVoronoi(vec4 p0, vec4 p1, vec4 p2, vec4 p3)
{
	vec3 result = closestPointOnTriangleVoronoi(vec3(p0.v[0], p0.v[1], p0.v[2]), 
												vec3(p1.v[0], p1.v[1], p1.v[2]), 
												vec3(p2.v[0], p2.v[1], p2.v[2]),
												vec3(p3.v[0], p3.v[1], p3.v[2]));
	return vec4(result.v[0], result.v[1], result.v[2], 0.0f);
}

// Given a point p0 and a pyramid defined by four vertices, p1, p2, p3, p4
vec3 closestPointOnPyramidVoronoi(vec3 p0, vec3 p1, vec3 p2, vec3 p3, vec3 p4)
{
	vec3 p0p1 = p0 - p1;
	vec3 p2p1 = p2 - p1;
	vec3 p3p1 = p3 - p1;
	vec3 p4p1 = p4 - p1;

	vec3 p0p2 = p0 - p2;
	vec3 p1p2 = p1 - p2;
	vec3 p3p2 = p3 - p2;
	vec3 p4p2 = p4 - p2;

	vec3 p0p3 = p0 - p3;
	vec3 p1p3 = p1 - p3;
	vec3 p2p3 = p2 - p3;
	vec3 p4p3 = p4 - p3;

	vec3 p0p4 = p0 - p4;
	vec3 p1p4 = p1 - p4;
	vec3 p2p4 = p2 - p4;
	vec3 p3p4 = p3 - p4;

	if (dot(p0p1, p2p1) <= 0 && 
		dot(p0p1, p3p1) <= 0 &&
		dot(p0p1, p4p1) <= 0)
	{
		// Voronoi region of p1
		return p1;
	}
	else if (dot(p0p2, p1p2) <= 0 && 
			 dot(p0p2, p3p2) <= 0 &&
			 dot(p0p2, p4p2) <= 0)
	{
		// Voronoi region of p2
		return p2;
	}
	else if (dot(p0p3, p1p3) <= 0 && 
			 dot(p0p3, p2p3) <= 0 &&
			 dot(p0p3, p4p3) <= 0)
	{
		// Voronoi region of p3
		return p3;
	}
	else if (dot(p0p4, p1p4) <= 0 &&
			 dot(p0p4, p2p4) <= 0 &&
			 dot(p0p4, p3p4) <= 0)
	{
		// Voronoi region of p4
		return p4;
	}
	else if (dot(cross(p2p1, cross(p2p1, p3p1)), p0p1) >= 0 &&
		dot(cross(cross(p4p1, p2p1), p2p1), p0p1) >= 0 &&
		dot(p0p1, p2p1) >= 0 &&
		dot(p0p2, p1p2) >= 0)
	{
		// Voronoi region of edge <p1, p2>
		return closestPointOnEdgeVoronoi(p0, p1, p2);
	}
	else if (dot(cross(p3p2, cross(p3p2, p1p2)), p0p2) >= 0 &&
		dot(cross(cross(p4p2, p3p2), p3p2), p0p2) >= 0 &&
		dot(p0p2, p3p2) >= 0 &&
		dot(p0p3, p2p3) >= 0)
	{
		// Voronoi region of edge <p2, p3>
		return closestPointOnEdgeVoronoi(p0, p2, p3);
	}
	else if (dot(cross(p1p3, cross(p1p3, p2p3)), p0p3) >= 0 &&
		dot(cross(cross(p4p3, p1p3), p1p3), p0p3) >= 0 &&
		dot(p0p3, p1p3) >= 0 &&
		dot(p0p1, p3p1) >= 0)
	{
		// Voronoi region of edge <p3, p1>
		return closestPointOnEdgeVoronoi(p0, p3, p1);
	}
	else if (dot(cross(p4p1, cross(p4p1, p3p1)), p0p1) >= 0 &&
		dot(cross(cross(p2p1, p4p1), p4p1), p0p1) >= 0 &&
		dot(p0p1, p4p1) >= 0 &&
		dot(p0p4, p1p4) >= 0)
	{
		// Voronoi region of edge <p1, p4>
		return closestPointOnEdgeVoronoi(p0, p1, p4);
	}
	else if (dot(cross(p4p2, cross(p4p2, p1p2)), p0p2) >= 0 &&
		dot(cross(cross(p3p2, p4p2), p4p2), p0p2) >= 0 &&
		dot(p0p2, p4p2) >= 0 &&
		dot(p0p4, p2p4) >= 0)
	{
		// Voronoi region of edge <p2, p4>
		return closestPointOnEdgeVoronoi(p0, p2, p4);
	}
	else if (dot(cross(p4p3, cross(p4p3, p2p3)), p0p3) >= 0 &&
		dot(cross(cross(p1p3, p4p3), p4p3), p0p3) >= 0 &&
		dot(p0p3, p4p3) >= 0 &&
		dot(p0p4, p3p4) >= 0)
	{
		// Voronoi region of edge <p3, p4>
		return closestPointOnEdgeVoronoi(p0, p3, p4);
	}
	else if (dot(p0p1, cross(p2p1, p3p1)) * dot(p4p1, cross(p2p1, p3p1)) < 0)
	{
		// Voronoi region of triangle face <p1, p2, p3>
		vec3 n_hat = normalise(cross(p2p1, p3p1));
		return closestPointOnPlane(p0, n_hat, p1);
	}
	else if (dot(p0p1, cross(p2p1, p4p1)) * dot(p3p1, cross(p2p1, p4p1)) < 0)
	{
		// Voronoi region of triangle face <p1, p2, p4>
		vec3 n_hat = normalise(cross(p2p1, p4p1));
		return closestPointOnPlane(p0, n_hat, p1);
	}
	else if (dot(p0p1, cross(p4p1, p3p1)) * dot(p2p1, cross(p4p1, p3p1)) < 0)
	{
		// Voronoi region of triangle face <p1, p4, p3>
		vec3 n_hat = normalise(cross(p4p1, p3p1));
		return closestPointOnPlane(p0, n_hat, p1);
	}
	else if (dot(p0p2, cross(p3p2, p4p2)) * dot(p1p2, cross(p3p2, p4p2)) < 0)
	{
		// Voronoi region of triangle face <p2, p3, p4>
		vec3 n_hat = normalise(cross(p3p2, p4p2));
		return closestPointOnPlane(p0, n_hat, p2);
	}
	else
	{
		// The point is inside the object, so assuming a solid, the closest point is itself
		return p0;
	}
}

// Given a point p0 and a pyramid defined by four vertices, p1, p2, p3, p4
vec3 closestPointOnPyramidVoronoi(vec3 p0, vec3 p1, vec3 p2, vec3 p3, vec3 p4, vec3* p_1, vec3* p_2, vec3* p_3, int* type)
{
	vec3 p0p1 = p0 - p1;
	vec3 p2p1 = p2 - p1;
	vec3 p3p1 = p3 - p1;
	vec3 p4p1 = p4 - p1;

	vec3 p0p2 = p0 - p2;
	vec3 p1p2 = p1 - p2;
	vec3 p3p2 = p3 - p2;
	vec3 p4p2 = p4 - p2;

	vec3 p0p3 = p0 - p3;
	vec3 p1p3 = p1 - p3;
	vec3 p2p3 = p2 - p3;
	vec3 p4p3 = p4 - p3;

	vec3 p0p4 = p0 - p4;
	vec3 p1p4 = p1 - p4;
	vec3 p2p4 = p2 - p4;
	vec3 p3p4 = p3 - p4;

	if (dot(p0p1, p2p1) <= 0 &&
		dot(p0p1, p3p1) <= 0 &&
		dot(p0p1, p4p1) <= 0)
	{
		// Voronoi region of p1
		*type = 0;
		*p_1 = p1;
		return p1;
	}
	else if (dot(p0p2, p1p2) <= 0 &&
		dot(p0p2, p3p2) <= 0 &&
		dot(p0p2, p4p2) <= 0)
	{
		// Voronoi region of p2
		*type = 0;
		*p_1 = p2;
		return p2;
	}
	else if (dot(p0p3, p1p3) <= 0 &&
		dot(p0p3, p2p3) <= 0 &&
		dot(p0p3, p4p3) <= 0)
	{
		// Voronoi region of p3
		*type = 0;
		*p_1 = p3;
		return p3;
	}
	else if (dot(p0p4, p1p4) <= 0 &&
		dot(p0p4, p2p4) <= 0 &&
		dot(p0p4, p3p4) <= 0)
	{
		// Voronoi region of p4
		*type = 0;
		*p_1 = p4;
		return p4;
	}
	else if (dot(cross(p2p1, cross(p2p1, p3p1)), p0p1) >= 0 &&
		dot(cross(cross(p4p1, p2p1), p2p1), p0p1) >= 0 &&
		dot(p0p1, p2p1) >= 0 &&
		dot(p0p2, p1p2) >= 0)
	{
		// Voronoi region of edge <p1, p2>
		*type = 1;
		*p_1 = p1;
		*p_2 = p2;
		return closestPointOnEdgeVoronoi(p0, p1, p2);
	}
	else if (dot(cross(p3p2, cross(p3p2, p1p2)), p0p2) >= 0 &&
		dot(cross(cross(p4p2, p3p2), p3p2), p0p2) >= 0 &&
		dot(p0p2, p3p2) >= 0 &&
		dot(p0p3, p2p3) >= 0)
	{
		// Voronoi region of edge <p2, p3>
		*type = 1;
		*p_1 = p2;
		*p_2 = p3;
		return closestPointOnEdgeVoronoi(p0, p2, p3);
	}
	else if (dot(cross(p1p3, cross(p1p3, p2p3)), p0p3) >= 0 &&
		dot(cross(cross(p4p3, p1p3), p1p3), p0p3) >= 0 &&
		dot(p0p3, p1p3) >= 0 &&
		dot(p0p1, p3p1) >= 0)
	{
		// Voronoi region of edge <p3, p1>
		*type = 1;
		*p_1 = p3;
		*p_2 = p1;
		return closestPointOnEdgeVoronoi(p0, p3, p1);
	}
	else if (dot(cross(p4p1, cross(p4p1, p3p1)), p0p1) >= 0 &&
		dot(cross(cross(p2p1, p4p1), p4p1), p0p1) >= 0 &&
		dot(p0p1, p4p1) >= 0 &&
		dot(p0p4, p1p4) >= 0)
	{
		// Voronoi region of edge <p1, p4>
		*type = 1;
		*p_1 = p1;
		*p_2 = p4;
		return closestPointOnEdgeVoronoi(p0, p1, p4);
	}
	else if (dot(cross(p4p2, cross(p4p2, p1p2)), p0p2) >= 0 &&
		dot(cross(cross(p3p2, p4p2), p4p2), p0p2) >= 0 &&
		dot(p0p2, p4p2) >= 0 &&
		dot(p0p4, p2p4) >= 0)
	{
		// Voronoi region of edge <p2, p4>
		*type = 1;
		*p_1 = p2;
		*p_2 = p4;
		return closestPointOnEdgeVoronoi(p0, p2, p4);
	}
	else if (dot(cross(p4p3, cross(p4p3, p2p3)), p0p3) >= 0 &&
		dot(cross(cross(p1p3, p4p3), p4p3), p0p3) >= 0 &&
		dot(p0p3, p4p3) >= 0 &&
		dot(p0p4, p3p4) >= 0)
	{
		// Voronoi region of edge <p3, p4>
		*type = 1;
		*p_1 = p3;
		*p_2 = p4;
		return closestPointOnEdgeVoronoi(p0, p3, p4);
	}
	else if (dot(p0p1, cross(p2p1, p3p1)) * dot(p4p1, cross(p2p1, p3p1)) < 0)
	{
		// Voronoi region of triangle face <p1, p2, p3>
		*type = 2;
		*p_1 = p1;
		*p_2 = p2;
		*p_3 = p3;
		vec3 n_hat = normalise(cross(p2p1, p3p1));
		return closestPointOnPlane(p0, n_hat, p1);
	}
	else if (dot(p0p1, cross(p2p1, p4p1)) * dot(p3p1, cross(p2p1, p4p1)) < 0)
	{
		// Voronoi region of triangle face <p1, p2, p4>
		*type = 2;
		*p_1 = p1;
		*p_2 = p2;
		*p_3 = p4;
		vec3 n_hat = normalise(cross(p2p1, p4p1));
		return closestPointOnPlane(p0, n_hat, p1);
	}
	else if (dot(p0p1, cross(p4p1, p3p1)) * dot(p2p1, cross(p4p1, p3p1)) < 0)
	{
		// Voronoi region of triangle face <p1, p4, p3>
		*type = 2;
		*p_1 = p1;
		*p_2 = p4;
		*p_3 = p3;
		vec3 n_hat = normalise(cross(p4p1, p3p1));
		return closestPointOnPlane(p0, n_hat, p1);
	}
	else if (dot(p0p2, cross(p3p2, p4p2)) * dot(p1p2, cross(p3p2, p4p2)) < 0)
	{
		// Voronoi region of triangle face <p2, p3, p4>
		*type = 2;
		*p_1 = p2;
		*p_2 = p3;
		*p_3 = p4;
		vec3 n_hat = normalise(cross(p3p2, p4p2));
		return closestPointOnPlane(p0, n_hat, p2);
	}
	else
	{
		// The point is inside the object, so assuming a solid, the closest point is itself
		*type = -1;
		return p0;
	}
}

// Given a point p0 and a pyramid defined by four vertices, p1, p2, p3, p4
vec4 closestPointOnPyramidVoronoi(vec4 p0, vec4 p1, vec4 p2, vec4 p3, vec4 p4)
{
	vec3 result = closestPointOnPyramidVoronoi(vec3(p0.v[0], p0.v[1], p0.v[2]),
		vec3(p1.v[0], p1.v[1], p1.v[2]),
		vec3(p2.v[0], p2.v[1], p2.v[2]),
		vec3(p3.v[0], p3.v[1], p3.v[2]),
		vec3(p4.v[0], p4.v[1], p4.v[2]));
	return vec4(result.v[0], result.v[1], result.v[2], 0.0f);
}