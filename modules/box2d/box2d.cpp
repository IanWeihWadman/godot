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

class Box2dImpl {
	struct PhysicsEntity {
		PhysicsEntity() = default;

		PhysicsEntity(b2BodyId p_bodyId, float p_weight) :
				bodyId{ p_bodyId },
				wantedVelocity{ b2Vec2_zero },
				weight{ p_weight },
				control{ 1.0f } {}

		b2Vec2 wantedVelocity;
		b2BodyId bodyId;
		float weight;
		float control;
	};

	b2WorldId worldId;
	HashMap<int32_t, b2BodyId> obstacleLookup;
	HashMap<int32_t, PhysicsEntity> entityLookup;

public:
	Box2dImpl() {
		auto worldDef = b2DefaultWorldDef();
		worldDef.gravity = b2Vec2_zero;
		worldId = b2CreateWorld(&worldDef);
	}

	~Box2dImpl() {
		b2DestroyWorld(worldId);
	}

	void StepWorld() {
		for (auto &entity : entityLookup) {
			entity.value.control = 0.04f + 0.96f * entity.value.control;
			b2Vec2 currentVelocity = b2Body_GetLinearVelocity(entity.value.bodyId);
			b2Vec2 newVelocity = (1.0f - 0.5f * entity.value.control) * currentVelocity + 0.5f * entity.value.control * entity.value.wantedVelocity;
			b2Body_SetLinearVelocity(entity.value.bodyId, newVelocity);
		}

		b2World_Step(worldId, 1 / 20.0f, 10);
	}

	void SetWantedVelocity(int32_t p_id, Vector2 p_velocity) {
		b2Vec2 physicsVelocity = WorldSpaceToPhysicsSpace(p_velocity);
		entityLookup[p_id].wantedVelocity = physicsVelocity;
	}

	void ResetControl(int32_t p_id) {
		entityLookup[p_id].control = 1.0f;
	}

	void ApplyImpulse(int32_t p_id, Vector2 p_impulse) {
		auto &entity = entityLookup[p_id];

		b2Vec2 physicsImpulse = WorldSpaceToPhysicsSpace(p_impulse);
		float impact = p_impulse.length() / (PHYSICS_WORLD_SCALE * entity.weight);
		b2Body_ApplyLinearImpulseToCenter(entity.bodyId, physicsImpulse, /*wake*/ true);
		entity.control = std::max(0.0f, entity.control - impact);
	}

	Vector2 GetEntityPosition(int32_t p_id) {
		return PhysicsSpaceToWorldSpace(b2Body_GetPosition((entityLookup[p_id].bodyId)));
	}

	void CreateEntity(int32_t p_id, Vector2 p_position, float p_radius, float p_weight) {
		p_radius = p_radius / PHYSICS_WORLD_SCALE;

		b2Circle circle;
		circle.center = b2Vec2_zero;
		circle.radius = p_radius;

		b2BodyDef bodyDef = b2DefaultBodyDef();
		bodyDef.position = WorldSpaceToPhysicsSpace(p_position);
		bodyDef.type = b2_dynamicBody;
		b2BodyId bodyId = b2CreateBody(worldId, &bodyDef);

		b2ShapeDef shapeDef = b2DefaultShapeDef();
		shapeDef.density = p_weight / (Math_PI * p_radius * p_radius);
		b2CreateCircleShape(bodyId, &shapeDef, &circle);

		entityLookup[p_id] = PhysicsEntity(bodyId, p_weight);
	}

	void CreateObstacle(int32_t p_id, const Vector2 *p_outline, int32_t p_outlineCount) {
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
		b2DestroyBody(iter->value.bodyId);
		entityLookup.remove(iter);
	}
};

void Box2dPhysics::_notification(int p_what) {
}

void Box2dPhysics::_bind_methods() {
	ClassDB::bind_method(D_METHOD("step_world"), &Box2dPhysics::step_world);
	ClassDB::bind_method(D_METHOD("add_entity", "id", "radius", "weight", "position"), &Box2dPhysics::add_entity);
	ClassDB::bind_method(D_METHOD("remove_entity", "id"), &Box2dPhysics::remove_entity);
	ClassDB::bind_method(D_METHOD("get_entity_position", "id"), &Box2dPhysics::get_entity_position);
	ClassDB::bind_method(D_METHOD("apply_impulse", "id", "impulse"), &Box2dPhysics::apply_impulse);
	ClassDB::bind_method(D_METHOD("reset_control", "id"), &Box2dPhysics::reset_control);
	ClassDB::bind_method(D_METHOD("set_wanted_velocity", "id", "velocity"), &Box2dPhysics::set_wanted_velocity);
	ClassDB::bind_method(D_METHOD("add_obstacle_outline", "id", "outline"), &Box2dPhysics::add_obstacle_outline);
	ClassDB::bind_method(D_METHOD("remove_obstacle", "id"), &Box2dPhysics::remove_obstacle);
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

void Box2dPhysics::add_entity(int32_t p_id, float p_radius, float p_weight, Vector2 p_position) {
	static_cast<Box2dImpl *>(impl)->CreateEntity(p_id, p_position, p_radius, p_weight);
}

void Box2dPhysics::remove_entity(int32_t p_id) {
	static_cast<Box2dImpl *>(impl)->DestroyEntity(p_id);
}

Vector2 Box2dPhysics::get_entity_position(int32_t p_id) {
	return static_cast<Box2dImpl *>(impl)->GetEntityPosition(p_id);
}

void Box2dPhysics::apply_impulse(int32_t p_id, Vector2 p_impulse) {
	static_cast<Box2dImpl *>(impl)->ApplyImpulse(p_id, p_impulse);
}

void Box2dPhysics::reset_control(int32_t p_id) {
	static_cast<Box2dImpl *>(impl)->ResetControl(p_id);
}

void Box2dPhysics::set_wanted_velocity(int32_t p_id, Vector2 p_velocity) {
	static_cast<Box2dImpl *>(impl)->SetWantedVelocity(p_id, p_velocity);
}

void Box2dPhysics::add_obstacle_outline(int32_t p_id, const PackedVector2Array &p_outline) {
	static_cast<Box2dImpl *>(impl)->CreateObstacle(p_id, p_outline.ptr(), p_outline.size());
}

void Box2dPhysics::remove_obstacle(int32_t p_id) {
	static_cast<Box2dImpl *>(impl)->DestroyObstacle(p_id);
}