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
static constexpr int32_t MAX_MAP_EXTENTS = 2048;
static constexpr int32_t MAP_CHUNK_SIZE = 256;
static constexpr int32_t MAX_DISTANCEFIELD_GRID = 16;
static constexpr float MAX_DISTANCEFIELD_VALUE = MAX_DISTANCEFIELD_GRID * PATH_GRID_SCALE;

Vector2i WorldToPathgrid(Vector2 world) {
	return Vector2i(MAX_MAP_EXTENTS, MAX_MAP_EXTENTS) + Vector2i(static_cast<int32_t>(Math::round(world.x / PATH_GRID_SCALE)), static_cast<int32_t>(Math::round(world.y / PATH_GRID_SCALE)));
}
Vector2i WorldDiffToPathgrid(Vector2 world) {
	return WorldToPathgrid(world) - WorldToPathgrid(Vector2(0, 0));
}

Vector2 PathgridToWorld(Vector2i grid) {
	grid -= Vector2i(MAX_MAP_EXTENTS, MAX_MAP_EXTENTS);
	return Vector2(PATH_GRID_SCALE * grid.x, PATH_GRID_SCALE * grid.y);
}

class MapGridCache {
private:
	struct MapChunk {
		Rect2i chunkExtents;
		LocalVector<float> distanceField;

		MapChunk(Rect2i extents) :
				chunkExtents{ extents } {
			distanceField.resize(MAP_CHUNK_SIZE * MAP_CHUNK_SIZE);
			for (uint32_t i = 0; i < distanceField.size(); i++) {
				distanceField[i] = MAX_DISTANCEFIELD_VALUE;
			}
		}

		_FORCE_INLINE_ float GetValue(Vector2i vector) {
			Vector2 offset = vector - chunkExtents.position;
			ERR_FAIL_COND_V_MSG(!HasPoint(vector), MAX_DISTANCEFIELD_VALUE, "Tried to access outside the bounds of this MapChunk");
			return distanceField[offset.y * MAP_CHUNK_SIZE + offset.x];
		}

		_FORCE_INLINE_ void UpdateValue(Vector2i vector, float value) {
			Vector2 offset = vector - chunkExtents.position;
			ERR_FAIL_COND_MSG(!HasPoint(vector), "Tried to access outside the bounds of this MapChunk");
			distanceField[offset.y * MAP_CHUNK_SIZE + offset.x] = std::min(value, distanceField[offset.y * MAP_CHUNK_SIZE + offset.x]);
		}

		_FORCE_INLINE_ bool HasPoint(Vector2i vector) {
			Vector2 offset = vector - chunkExtents.position;
			return (offset.x >= 0 && offset.y >= 0 && offset.x < MAP_CHUNK_SIZE && offset.y < MAP_CHUNK_SIZE);
		}
	};

	HashMap<int32_t, MapChunk *> chunks;
	mutable MapChunk *lastAccessedChunk;

	float DistanceFromPointToSegmentList(const Vector2 &point, const Vector2 *outline, int32_t outlineCount) {
		float distanceSqr = MAX_DISTANCEFIELD_VALUE * MAX_DISTANCEFIELD_VALUE;

		for (int i = 0; i < outlineCount; i++) {
			Vector2 segmentStart = outline[i];
			Vector2 segmentEnd = outline[(i + 1) % outlineCount];

			Vector2 segmentVector = segmentEnd - segmentStart;
			Vector2 pointToStartVector = segmentStart - point;

			float segmentParameter = -pointToStartVector.dot(segmentVector) / segmentVector.length_squared();
			segmentParameter = CLAMP(segmentParameter, 0.0f, 1.0f);

			distanceSqr = std::min(distanceSqr, (segmentStart + segmentParameter * segmentVector - point).length_squared());
		}

		return Math::sqrt(distanceSqr);
	}

public:
	void AddObstacleOutline(int32_t id, const Vector2 *outline, int32_t outlineCount) {
		Rect2 aabb{ outline[0], Vector2(0, 0) };
		for (int32_t i = 1; i < outlineCount; i++) {
			aabb.expand_to(outline[i]);
		}

		Rect2i gridRect(WorldToPathgrid(aabb.position) - Vector2i(MAX_DISTANCEFIELD_GRID, MAX_DISTANCEFIELD_GRID), WorldDiffToPathgrid(aabb.size) + Vector2i(2 * MAX_DISTANCEFIELD_GRID, 2 * MAX_DISTANCEFIELD_GRID));

		for (int32_t y = gridRect.position.y; y < gridRect.get_end().y; y++) {
			for (int32_t x = gridRect.position.x; x < gridRect.get_end().x; x++) {
				Vector2i position(x, y);
				UpdateDistanceFieldValue(position, DistanceFromPointToSegmentList(PathgridToWorld(position), outline, outlineCount));
			}
		}
	}

	void UpdateDistanceFieldValue(Vector2i position, float value) {
		MapChunk *relevantChunk = nullptr;
		if (lastAccessedChunk && lastAccessedChunk->HasPoint(position)) {
			relevantChunk = lastAccessedChunk;
		} else {
			auto iter = chunks.find(ComputeChunkKey(position));
			if (iter != chunks.end()) {
				relevantChunk = iter->value;
			}
		}

		if (relevantChunk == nullptr) {
			Vector2i chunkPosition = Vector2i(MAP_CHUNK_SIZE * (position.x / MAP_CHUNK_SIZE), MAP_CHUNK_SIZE * (position.y / MAP_CHUNK_SIZE));
			relevantChunk = new MapChunk(Rect2i(chunkPosition, Vector2i(MAP_CHUNK_SIZE, MAP_CHUNK_SIZE)));

			chunks[ComputeChunkKey(position)] = relevantChunk;
		}

		lastAccessedChunk = relevantChunk;
		relevantChunk->UpdateValue(position, value);
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
		return (MAX_MAP_EXTENTS / MAP_CHUNK_SIZE) * (position.x / MAP_CHUNK_SIZE) + (position.y) / MAP_CHUNK_SIZE;
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
	if (LinearPathExistsInternal(origin, destination, radius)) {
		Path *result = new Path();
		result->AddWaypoint(destination);
		result->AddWaypoint(origin);
	}

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
			return ReconstructPath(origin, destination, radius);
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

Path *Pathfinder::ReconstructPath(Vector2i origin, Vector2i destination, float radius) {
	Vector2i next = destination;
	Vector2i prev = destination;
	PathfinderImpl *implementation = static_cast<PathfinderImpl *>(impl);
	LocalVector<Vector2i> waypoints;

	waypoints.push_back(destination);
	Vector2i lastStepDirection = implementation->cameFrom[destination] - destination;

	while (next != origin) {
		next = implementation->cameFrom[prev];
		if (next - prev != lastStepDirection) {
			waypoints.push_back(next);
			lastStepDirection = next - prev;
		}
		prev = next;
	}

	waypoints.push_back(origin);

	Path *path = new Path();
	path->AddWaypoint(PathgridToWorld(waypoints[0]));
	int32_t lastNeededWaypoint = 0;

	for (uint32_t i = 1; i < waypoints.size() - 1; i++) {
		if (!LinearPathExistsInternal(waypoints[lastNeededWaypoint], waypoints[i + 1], radius)) {
			path->AddWaypoint(PathgridToWorld(waypoints[i]));
			lastNeededWaypoint = i;
		}
	}
	path->AddWaypoint(PathgridToWorld(waypoints[waypoints.size() - 1]));
	return path;
}

void Pathfinder::AddObstacleOutline(int32_t id, const Vector2 *outline, int32_t outlineCount) {
	static_cast<PathfinderImpl *>(impl)->map.AddObstacleOutline(id, outline, outlineCount);
}

bool Pathfinder::LinearPathExists(Vector2 origin, Vector2 destination, float radius) {
	return LinearPathExistsInternal(WorldToPathgrid(origin), WorldToPathgrid(destination), radius);
}

bool Pathfinder::LinearPathExistsInternal(Vector2i origin, Vector2i destination, float radius) {
	PathfinderImpl *implementation = static_cast<PathfinderImpl *>(impl);

	int32_t x0 = origin.x;
	int32_t y0 = origin.y;
	int32_t x1 = destination.x;
	int32_t y1 = destination.y;

	int32_t dx = abs(x1 - x0);
	int32_t dy = abs(y1 - y0);

	int32_t sx = (x0 < x1) ? 1 : -1;
	int32_t sy = (y0 < y1) ? 1 : -1;

	int32_t err = dx - dy;

	while (true) {
		if (implementation->map.GetDistanceFieldValue(Vector2i(x0, y0)) < radius) {
			return false;
		}

		if (x0 == x1 && y0 == y1) {
			break;
		}

		// Update error and coordinates
		int32_t e2 = 2 * err;
		if (e2 > -dy) {
			err -= dy;
			x0 += sx;
		}
		if (e2 < dx) {
			err += dx;
			y0 += sy;
		}
	}

	return true;
}