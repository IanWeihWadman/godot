#ifndef BOX2D_PATHFINDING_H
#define BOX2D_PATHFINDING_H

#include "core/error/error_macros.h"
#include "core/math/rect2i.h"
#include "core/math/vector2.h"
#include "core/templates/hash_map.h"
#include "core/templates/vector.h"

#include <optional>

class Path {
public:
	Vector2 GetNextWaypoint() {
		return waypoints[waypoints.size() - 1];
	}

	Vector2 GetDestination() {
		return waypoints[0];
	}

	void AddWaypoint(Vector2 waypoint) {
		waypoints.push_back(waypoint);
	}

	void PopWaypoint() {
		waypoints.remove_at(waypoints.size() - 1);
	}

	bool IsValid() {
		return waypoints.size() > 0;
	}

	float GetLength() {
		if (length.has_value()) {
			return *length;
		}

		length = 0.0f;
		for (int64_t i = 1; i < waypoints.size(); i++) {
			length = *length + (waypoints[i] - waypoints[i - 1]).length();
		}

		return *length;
	}

private:
	Vector<Vector2> waypoints;
	std::optional<float> length;
};

class Pathfinder {
private:
	void *impl;

	Path *ComputePathInternal(Vector2i origin, Vector2i destination, float radius);
	Path *ReconstructPath(Vector2i origin, Vector2i destination);

public:
	Pathfinder();
	~Pathfinder();

	void AddObstacleOutline(int32_t p_id, const Vector2 *p_outline, int32_t p_outlineCount);
	Path *ComputePath(Vector2 origin, Vector2 destination, float radius);
};

#endif