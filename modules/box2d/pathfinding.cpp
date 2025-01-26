#include "pathfinding.h"

#include <core/templates/local_vector.h>
#include <core/templates/sort_array.h>

class OpenList {
	struct Entry {
		float estimate;
		Vector2i position;
	};

	struct EntryCompare {
		bool operator()(const Entry &a, const Entry &b) const {
			return a.estimate > b.estimate;
		}
	};

	SortArray<Entry, EntryCompare, true> sorter;
	LocalVector<Entry> sortedVector;
	HashMap<int32_t, float> lowestSeenValueMap;

public:
	void AddValue(float estimate, Vector2i position) {
		int32_t key = position.x + 4096 * position.y;
		auto lowestSeenValue = lowestSeenValueMap.find(key);

		if (lowestSeenValue == lowestSeenValueMap.end() || estimate < lowestSeenValue->value) {
			auto newEntry = Entry{
				estimate = estimate,
				position = position
			};

			lowestSeenValueMap[key] = estimate;
			sortedVector.push_back(Entry{});
			sorter.push_heap(0, sortedVector.size() - 1, 0, newEntry, sortedVector.ptr());
		}
	}

	std::optional<Vector2i> PopValue() {
		while (sortedVector.size() > 0) {
			Entry topEntry = sortedVector[0];
			sorter.pop_heap(0, sortedVector.size() - 1, sortedVector.ptr());
			sortedVector.remove_at(sortedVector.size() - 1);

			int32_t key = topEntry.position.x + 4096 * topEntry.position.y;
			auto lowestSeenValue = lowestSeenValueMap.find(key);

			if (lowestSeenValue == lowestSeenValueMap.end() || topEntry.estimate <= lowestSeenValue->value) {
				return std::make_optional(topEntry.position);
			}
		}
		return std::nullopt;
	}

	void Clear() {
		sortedVector.clear();
		lowestSeenValueMap.clear();
	}
};

static constexpr float PATH_GRID_SCALE = 20.0f;

Vector2i WorldToPathgrid(Vector2 world) {
	return Vector2i(static_cast<int32_t>(Math::round(world.x / PATH_GRID_SCALE)), static_cast<int32_t>(Math::round(world.y / PATH_GRID_SCALE)));
}
Vector2 PathgridToWorld(Vector2i grid) {
	return Vector2(PATH_GRID_SCALE * grid.x, PATH_GRID_SCALE * grid.y);
}

class MapGridCache {
private:
	struct MapChunk {
		Rect2i chunkExtents;
		Vector<float> distanceField;

		float GetValue(Vector2i vector) {
			Vector2 offset = vector - chunkExtents.position;
			ERR_FAIL_COND_V_MSG(!HasPoint(vector), MAX_DISTANCEFIELD_VALUE, "Tried to access outside the bounds of this MapChunk");
			return distanceField[offset.y * MAP_CHUNK_SIZE + offset.x];
		}

		bool HasPoint(Vector2i vector) {
			Vector2 offset = vector - chunkExtents.position;
			return (offset.x >= 0 && offset.y >= 0 && offset.x < MAP_CHUNK_SIZE && offset.y < MAP_CHUNK_SIZE);
		}
	};

	HashMap<int32_t, MapChunk *> chunks;
	mutable MapChunk *lastAccessedChunk;

public:
	static constexpr int32_t MAP_CHUNK_SIZE = 256;
	static constexpr float MAX_DISTANCEFIELD_VALUE = 16 * PATH_GRID_SCALE;
	static constexpr int32_t MAX_MAP_EXTENTS = 2048;

	void AddObstacleOutline(int32_t id, const Vector2 *outline, int32_t outlineCount) {
	}

	float GetDistanceFieldValue(Vector2i position) const {
		if (lastAccessedChunk && lastAccessedChunk->HasPoint(position)) {
			return lastAccessedChunk->GetValue(position);
		}

		auto iter = chunks.find(ComputeChunkKey(position));
		if (iter == chunks.end()) {
			return MAX_DISTANCEFIELD_VALUE;
		}

		lastAccessedChunk = iter->value;
		return iter->value->GetValue(position);
	}

	int32_t ComputeChunkKey(Vector2i position) const {
		return (MAX_MAP_EXTENTS / MAP_CHUNK_SIZE) * (MAX_MAP_EXTENTS + position.x / MAP_CHUNK_SIZE) + (MAX_MAP_EXTENTS + position.y) / MAP_CHUNK_SIZE;
	}
};

struct PathfinderImpl {
	MapGridCache map;
	OpenList openList;
	HashMap<Vector2i, Vector2i> cameFrom;
	HashMap<Vector2i, float> calculatedDistances;

	float EstimateDistance(Vector2i current, Vector2i destination) {
		return 0.5f * (Math::abs(current.x - destination.x) + Math::abs(current.y - destination.y));
	}
};

Pathfinder::Pathfinder() {
	impl = new PathfinderImpl();
}

Pathfinder::~Pathfinder() {
	delete impl;
}

Path *Pathfinder::ComputePath(Vector2 origin, Vector2 destination, float radius) {
	return ComputePathInternal(WorldToPathgrid(origin), WorldToPathgrid(destination), radius);
}

Path *Pathfinder::ComputePathInternal(Vector2i origin, Vector2i destination, float radius) {
	PathfinderImpl *implementation = static_cast<PathfinderImpl *>(impl);

	implementation->openList.Clear();
	implementation->cameFrom.clear();
	implementation->calculatedDistances.clear();

	implementation->calculatedDistances[origin] = 0;
	implementation->openList.AddValue(implementation->EstimateDistance(origin, destination), origin);

	int iterations = 0;

	while (iterations++ < 20000) {
		auto currentNode = implementation->openList.PopValue();
		if (!currentNode.has_value()) {
			break;
		}

		if (*currentNode == destination) {
			return ReconstructPath(origin, destination);
		}

		for (int32_t i = -1; i <= 1; i++) {
			for (int32_t j = -1; j <= 1; j++) {
				if (i == 0 && j == 0) {
					continue;
				}

				Vector2i neighbour = Vector2i(currentNode->x + i, currentNode->y + j);
				if (implementation->map.GetDistanceFieldValue(neighbour) < radius) {
					continue;
				}

				float distance = (i == 0 || j == 0) ? 1.0f : Math_SQRT2;
				float newComputedDistance = implementation->calculatedDistances[*currentNode] + distance;
				auto lastComputedDistance = implementation->calculatedDistances.find(neighbour);

				if (lastComputedDistance == implementation->calculatedDistances.end() || newComputedDistance < lastComputedDistance->value) {
					implementation->calculatedDistances[neighbour] = newComputedDistance;
					implementation->cameFrom[neighbour] = *currentNode;

					float newEstimatedDistance = newComputedDistance + implementation->EstimateDistance(neighbour, destination);
					implementation->openList.AddValue(newEstimatedDistance, neighbour);
				}
			}
		}
	}

	return new Path();
}

Path *Pathfinder::ReconstructPath(Vector2i origin, Vector2i destination) {
	Vector2i next = destination;
	Vector2i prev = destination;
	PathfinderImpl *implementation = static_cast<PathfinderImpl *>(impl);
	Vector<Vector2i> waypoints;

	while (next != origin) {
		waypoints.push_back(next);
		next = implementation->cameFrom[prev];
		prev = next;
	}

	Path *path = new Path();
	for (const Vector2i &waypoint : waypoints) {
		path->AddWaypoint(PathgridToWorld(waypoint));
	}
	return path;
}

void Pathfinder::AddObstacleOutline(int32_t id, const Vector2 *outline, int32_t outlineCount) {
	static_cast<PathfinderImpl *>(impl)->map.AddObstacleOutline(id, outline, outlineCount);
}