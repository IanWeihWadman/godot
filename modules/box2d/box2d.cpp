/**************************************************************************/
/*  grid_map.cpp                                                          */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#include "box2d.h"
#include "box2d/box2d.h"
#include "pathfinding.h"
#include <optional>

namespace {

constexpr float PHYSICS_WORLD_SCALE = 50.0f;

b2Vec2 WorldSpaceToPhysicsSpace(Vector2 p_worldVec) {
	b2Vec2 vec;
	vec.x = p_worldVec.x / PHYSICS_WORLD_SCALE;
	vec.y = p_worldVec.y / PHYSICS_WORLD_SCALE;
	return vec;
}

Vector2 PhysicsSpaceToWorldSpace(b2Vec2 p_b2Vec) {
	return Vector2(PHYSICS_WORLD_SCALE * p_b2Vec.x, PHYSICS_WORLD_SCALE * p_b2Vec.y);
}

} //namespace

struct Box2dImpl {
	struct PursueTargetInfo {
		PursueTargetInfo(int32_t p_targetId, float p_targetDistance) :
				targetId{ p_targetId },
				targetDistance{ p_targetDistance } {}

		int32_t targetId;
		float targetDistance;
	};

	struct PhysicsEntity {
		PhysicsEntity() = default;

		PhysicsEntity(b2BodyId p_bodyId, float p_weight, float p_radius, float p_speed) :
				bodyId{ p_bodyId },
				wantedVelocity{ b2Vec2_zero },
				weight{ p_weight },
				control{ 1.0f },
				radius{ p_radius },
				movementSpeed{ p_speed },
				activePath{ nullptr },
				movementPaused{ 0 } {}

		void CancelMove() {
			if (activePath) {
				delete activePath;
			}
			activePath = nullptr;
		}

		b2Vec2 wantedVelocity;
		std::optional<b2Vec2> forcedVelocity;
		std::optional<PursueTargetInfo> pursueTarget;
		b2BodyId bodyId;
		float weight;
		float control;
		float radius;
		float movementSpeed;
		Path *activePath;
		int32_t movementPaused;
	};

	Box2dImpl() {
		auto worldDef = b2DefaultWorldDef();
		worldDef.gravity = b2Vec2_zero;
		worldId = b2CreateWorld(&worldDef);
	}

	~Box2dImpl() {
		b2DestroyWorld(worldId);
	}

	void StepWorld() {
		//CollisionAvoidancePass();

		for (auto &entity : entityLookup) {
			auto worldPosition = PhysicsSpaceToWorldSpace(b2Body_GetPosition(entity.value.bodyId));
			bool targetTooClose = false;

			if (entity.value.forcedVelocity) { // Dash case
				entity.value.CancelMove();
				entity.value.wantedVelocity = *entity.value.forcedVelocity;
				entity.value.control = 1.0f;
			} else if (entity.value.pursueTarget) { // Pursue target case
				auto iter = entityLookup.find(entity.value.pursueTarget->targetId);

				if (iter == entityLookup.end()) {
					entity.value.pursueTarget = std::nullopt;
				} else {
					auto targetWorldPosition = PhysicsSpaceToWorldSpace(b2Body_GetPosition(iter->value.bodyId));

					if (!entity.value.activePath || (entity.value.activePath->GetDestination() - targetWorldPosition).length_squared() > 64.0f) {
						MoveTo(entity.key, targetWorldPosition);
					}

					if ((worldPosition - targetWorldPosition).length() < entity.value.pursueTarget->targetDistance) {
						targetTooClose = true;
					}
				}
			}

			if (entity.value.movementPaused > 0 || (!entity.value.activePath && !entity.value.forcedVelocity) || targetTooClose) {
				entity.value.wantedVelocity = b2Vec2_zero;
			} else if (entity.value.activePath) {
				if ((entity.value.activePath->GetNextWaypoint() - worldPosition).length_squared() < 25.0f * 25.0f) {
					entity.value.activePath->PopWaypoint();

					if (!entity.value.activePath->IsValid()) {
						delete entity.value.activePath;
						entity.value.activePath = nullptr;
						entity.value.wantedVelocity = b2Vec2_zero;
					}
				}

				if (entity.value.activePath) {
					entity.value.wantedVelocity = WorldSpaceToPhysicsSpace(20.0f * entity.value.movementSpeed * (entity.value.activePath->GetNextWaypoint() - worldPosition).normalized());
				}
			}

			entity.value.control = 0.04f + 0.96f * entity.value.control;
			b2Vec2 currentVelocity = b2Body_GetLinearVelocity(entity.value.bodyId);
			b2Vec2 newVelocity = (1.0f - 0.5f * entity.value.control) * currentVelocity + 0.5f * entity.value.control * entity.value.wantedVelocity;
			b2Body_SetLinearVelocity(entity.value.bodyId, newVelocity);
		}

		b2World_Step(worldId, 1 / 20.0f, 10);
	}

	/* 	void CollisionAvoidancePass() {
			for (auto &entityA : entityLookup) {
				if (entityA.value.control < 0.8) {
					continue;
				}

				auto positionA = b2Body_GetPosition(entityA.value.bodyId);

				for (auto &entityB : entityLookup) {
					if (entityA.key == entityB.key) {
						continue;
					}

					auto positionB = b2Body_GetPosition(entityB.value.bodyId);
					if (b2Length(positionA - positionB) > 2.0f * (entityA.value.physicsRadius + entityB.value.physicsRadius)) {
						continue;
					}

					auto dotProduct = b2Dot(positionB - positionA, entityA.value.wantedVelocity);
					auto cosineSquared = dotProduct * dotProduct / (b2LengthSquared(positionB - positionA) * b2LengthSquared(entityA.value.wantedVelocity));
					if (cosineSquared < 0.8f) {
						continue;
					}

					if (b2Dot(positionB - positionA, entityA.value.wantedVelocity)) {
						entityA.value.wantedVelocity = b2RotateVector(b2MakeRot(0.6f), entityA.value.wantedVelocity);
					}
				}
			}
		} */

	void ApplyImpulse(int32_t p_id, Vector2 p_impulse) {
		auto &entity = entityLookup[p_id];

		b2Vec2 physicsImpulse = WorldSpaceToPhysicsSpace(p_impulse);
		float impact = p_impulse.length() / (PHYSICS_WORLD_SCALE * entity.weight);
		b2Body_ApplyLinearImpulseToCenter(entity.bodyId, physicsImpulse, /*wake*/ true);
		entity.control = std::max(0.0f, entity.control - impact);
	}

	Box2dPhysics::MovementMode GetEntityMovementMode(int32_t p_id) {
		auto &entity = entityLookup[p_id];
		if (entity.pursueTarget) {
			return Box2dPhysics::MovementMode::TARGET;
		}
		if (entity.activePath) {
			return Box2dPhysics::MovementMode::DESTINATION;
		}
		return Box2dPhysics::MovementMode::NONE;
	}

	void CreateEntity(int32_t p_id, Vector2 p_position, float p_radius, float p_weight, float p_speed) {
		float physicsRadius = p_radius / PHYSICS_WORLD_SCALE;

		b2Circle circle;
		circle.center = b2Vec2_zero;
		circle.radius = physicsRadius;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = WorldSpaceToPhysicsSpace(p_position);
		bodyDef.type = b2_dynamicBody;
		b2BodyId bodyId = b2CreateBody(worldId, &bodyDef);

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = p_weight / (Math_PI * physicsRadius * physicsRadius);
		b2CreateCircleShape(bodyId, &shapeDef, &circle);

		entityLookup[p_id] = PhysicsEntity(bodyId, p_weight, p_radius, p_speed);
	}

	void CreateObstacle(int32_t p_id, const Vector2 *p_outline, int32_t p_outlineCount) {
		pathfinder.AddObstacleOutline(p_id, p_outline, p_outlineCount);
		Vector2 approxCenter = Vector2(0, 0);

		for (int32_t i = 0; i < std::min(64, p_outlineCount); i++) {
			approxCenter += p_outline[i];
		}

		approxCenter /= (real_t)p_outlineCount;

		b2Vec2 conversionBuffer[64];
		for (int32_t i = 0; i < std::min(64, p_outlineCount); i++) {
			conversionBuffer[i] = WorldSpaceToPhysicsSpace(p_outline[i] - approxCenter);
		}

		b2Hull hull = b2ComputeHull(&conversionBuffer[0], p_outlineCount);
		b2Polygon polygon = b2MakePolygon(&hull, 0.0f);
		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = WorldSpaceToPhysicsSpace(approxCenter);

		b2BodyId bodyId = b2CreateBody(worldId, &bodyDef);
		b2ShapeDef shapeDef = b2DefaultShapeDef();

		b2CreatePolygonShape(bodyId, &shapeDef, &polygon);
		obstacleLookup[p_id] = bodyId;
	}

	void DestroyObstacle(int32_t p_id) {
		auto iter = obstacleLookup.find(p_id);
		b2DestroyBody(iter->value);
		obstacleLookup.remove(iter);
	}

	void DestroyEntity(int32_t p_id) {
		auto iter = entityLookup.find(p_id);
		iter->value.CancelMove();
		b2DestroyBody(iter->value.bodyId);
		entityLookup.remove(iter);
	}

	bool MoveTo(int32_t p_id, Vector2 destination) {
		auto &entity = entityLookup[p_id];
		entity.CancelMove();

		auto worldPosition = PhysicsSpaceToWorldSpace(b2Body_GetPosition(entity.bodyId));
		auto path = pathfinder.ComputePath(worldPosition, destination, entityLookup[p_id].radius);

		if (path->IsValid()) {
			entity.activePath = path;
			return true;
		}

		delete path;
		entity.activePath = nullptr;
		return false;
	}

	bool MoveToTarget(int32_t p_id, int32_t p_targetId, float p_targetDistance) {
		if (entityLookup.find(p_targetId) != entityLookup.end()) {
			entityLookup[p_id].pursueTarget = PursueTargetInfo(p_targetId, p_targetDistance);
			return true;
		}
		return false;
	}

	void CancelMove(int32_t p_id) {
		auto &entity = entityLookup[p_id];
		entity.CancelMove();
		entity.pursueTarget = std::nullopt;
	}

	b2WorldId worldId;
	HashMap<int32_t, b2BodyId> obstacleLookup;
	HashMap<int32_t, PhysicsEntity> entityLookup;
	Pathfinder pathfinder;
};

void Box2dPhysics::_notification(int p_what) {
}

void Box2dPhysics::_bind_methods() {
	ClassDB::bind_method(D_METHOD("step_world"), &Box2dPhysics::step_world);
	ClassDB::bind_method(D_METHOD("add_entity", "id", "radius", "weight", "position", "speed"), &Box2dPhysics::add_entity);
	ClassDB::bind_method(D_METHOD("remove_entity", "id"), &Box2dPhysics::remove_entity);
	ClassDB::bind_method(D_METHOD("get_entity_position", "id"), &Box2dPhysics::get_entity_position);
	ClassDB::bind_method(D_METHOD("get_entity_movement_mode", "id"), &Box2dPhysics::get_entity_movement_mode);
	ClassDB::bind_method(D_METHOD("apply_impulse", "id", "impulse"), &Box2dPhysics::apply_impulse);
	ClassDB::bind_method(D_METHOD("reset_control", "id"), &Box2dPhysics::reset_control);
	ClassDB::bind_method(D_METHOD("set_forced_velocity", "id", "velocity"), &Box2dPhysics::set_forced_velocity);
	ClassDB::bind_method(D_METHOD("clear_forced_velocity", "id"), &Box2dPhysics::clear_forced_velocity);
	ClassDB::bind_method(D_METHOD("move_to", "id", "destination"), &Box2dPhysics::move_to);
	ClassDB::bind_method(D_METHOD("move_to_target", "id", "target", "target_distance"), &Box2dPhysics::move_to_target);
	ClassDB::bind_method(D_METHOD("linear_path_exists", "origin", "destination", "radius"), &Box2dPhysics::linear_path_exists);
	ClassDB::bind_method(D_METHOD("cancel_move", "id"), &Box2dPhysics::cancel_move);
	ClassDB::bind_method(D_METHOD("pause_movement", "id"), &Box2dPhysics::pause_movement);
	ClassDB::bind_method(D_METHOD("unpause_movement", "id"), &Box2dPhysics::unpause_movement);
	ClassDB::bind_method(D_METHOD("add_obstacle_outline", "id", "outline"), &Box2dPhysics::add_obstacle_outline);
	ClassDB::bind_method(D_METHOD("remove_obstacle", "id"), &Box2dPhysics::remove_obstacle);

	BIND_ENUM_CONSTANT(NONE);
	BIND_ENUM_CONSTANT(DESTINATION);
	BIND_ENUM_CONSTANT(TARGET);
}

Box2dPhysics::Box2dPhysics() {
	impl = new Box2dImpl();
}

Box2dPhysics::~Box2dPhysics() {
	delete impl;
}

void Box2dPhysics::step_world() {
	static_cast<Box2dImpl *>(impl)->StepWorld();
}

void Box2dPhysics::add_entity(int32_t p_id, float p_radius, float p_weight, Vector2 p_position, float p_speed) {
	static_cast<Box2dImpl *>(impl)->CreateEntity(p_id, p_position, p_radius, p_weight, p_speed);
}

void Box2dPhysics::remove_entity(int32_t p_id) {
	static_cast<Box2dImpl *>(impl)->DestroyEntity(p_id);
}

Vector2 Box2dPhysics::get_entity_position(int32_t p_id) {
	auto bodyId = static_cast<Box2dImpl *>(impl)->entityLookup[p_id].bodyId;
	return PhysicsSpaceToWorldSpace(b2Body_GetPosition(bodyId));
}

Box2dPhysics::MovementMode Box2dPhysics::get_entity_movement_mode(int32_t p_id) {
	return static_cast<Box2dImpl *>(impl)->GetEntityMovementMode(p_id);
}

void Box2dPhysics::apply_impulse(int32_t p_id, Vector2 p_impulse) {
	static_cast<Box2dImpl *>(impl)->ApplyImpulse(p_id, p_impulse);
}

void Box2dPhysics::reset_control(int32_t p_id) {
	static_cast<Box2dImpl *>(impl)->entityLookup[p_id].control = 1.0f;
}

void Box2dPhysics::set_forced_velocity(int32_t p_id, Vector2 p_velocity) {
	static_cast<Box2dImpl *>(impl)->entityLookup[p_id].forcedVelocity = WorldSpaceToPhysicsSpace(p_velocity);
}

void Box2dPhysics::clear_forced_velocity(int32_t p_id) {
	static_cast<Box2dImpl *>(impl)->entityLookup[p_id].forcedVelocity = std::nullopt;
}

void Box2dPhysics::pause_movement(int32_t p_id) {
	static_cast<Box2dImpl *>(impl)->entityLookup[p_id].movementPaused++;
}

void Box2dPhysics::unpause_movement(int32_t p_id) {
	static_cast<Box2dImpl *>(impl)->entityLookup[p_id].movementPaused--;
}

bool Box2dPhysics::move_to(int32_t p_id, Vector2 p_position) {
	return static_cast<Box2dImpl *>(impl)->MoveTo(p_id, p_position);
}

bool Box2dPhysics::move_to_target(int32_t p_id, int32_t p_targetId, float p_targetDistance) {
	return static_cast<Box2dImpl *>(impl)->MoveToTarget(p_id, p_targetId, p_targetDistance);
}

bool Box2dPhysics::linear_path_exists(Vector2 origin, Vector2 destination, float radius) {
	return static_cast<Box2dImpl *>(impl)->pathfinder.LinearPathExists(origin, destination, radius);
}

void Box2dPhysics::cancel_move(int32_t p_id) {
	static_cast<Box2dImpl *>(impl)->CancelMove(p_id);
}

void Box2dPhysics::add_obstacle_outline(int32_t p_id, const PackedVector2Array &p_outline) {
	static_cast<Box2dImpl *>(impl)->CreateObstacle(p_id, p_outline.ptr(), p_outline.size());
}

void Box2dPhysics::remove_obstacle(int32_t p_id) {
	static_cast<Box2dImpl *>(impl)->DestroyObstacle(p_id);
}