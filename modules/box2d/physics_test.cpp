#include "physics_test.h"
#include "box2d.h"
#include "pathfinding.h"

void TestFindLinearPath() {
	Pathfinder pathfinder;
	ERR_FAIL_COND_MSG(!pathfinder.LinearPathExists(Vector2(0, 0), Vector2(400, 700), 100), "Expected a linear path to exist");
	Path *result = pathfinder.ComputePath(Vector2(0, 0), Vector2(400, 700), 100);
	ERR_FAIL_COND_MSG(!result->IsValid(), "ComputePath should have found the available linear path");
}

void TestNoLinearPath() {
	Pathfinder pathfinder;
	Vector2 outline[4];
	outline[0] = Vector2(-100, -100);
	outline[1] = Vector2(100, -100);
	outline[2] = Vector2(100, 100);
	outline[3] = Vector2(-100, 100);

	pathfinder.AddObstacleOutline(0, &outline[0], 4);

	ERR_FAIL_COND_MSG(pathfinder.LinearPathExists(Vector2(-300, 0), Vector2(300, 0), 100), "Expected no linear path to exist");
}

void TestFindNontrivialPath() {
	Pathfinder pathfinder;
	Vector2 outline[4];
	outline[0] = Vector2(-100, -100);
	outline[1] = Vector2(100, -100);
	outline[2] = Vector2(100, 100);
	outline[3] = Vector2(-100, 100);

	pathfinder.AddObstacleOutline(0, &outline[0], 4);

	Path *result = pathfinder.ComputePath(Vector2(-300, 0), Vector2(300, 0), 100);
	ERR_FAIL_COND_MSG(!result->IsValid(), "Should have found a path around the obstacle");
	ERR_FAIL_COND(result->GetLength() < 650);
	ERR_FAIL_COND(result->GetLength() > 850);
}

void RunPhysicsTests() {
	TestFindLinearPath();
	TestNoLinearPath();
	TestFindNontrivialPath();
}
