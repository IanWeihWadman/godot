/**************************************************************************/
/*  grid_map.h                                                            */
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

#ifndef BOX2D_PHYSICS_H
#define BOX2D_PHYSICS_H

#include "scene/main/node.h"

class Box2dPhysics : public Node {
	GDCLASS(Box2dPhysics, Node);

	void *impl = nullptr;

protected:
	void _notification(int p_what);
	static void _bind_methods();

public:
	enum MovementMode {
		NONE,
		DESTINATION,
		TARGET
	};

	Box2dPhysics();
	~Box2dPhysics();

	void step_world();
	void add_entity(int32_t p_id, float p_radius, float p_weight, Vector2 p_position, float p_speed);
	void remove_entity(int32_t p_id);
	Vector2 get_entity_position(int32_t p_id);
	MovementMode get_entity_movement_mode(int32_t p_id);
	void apply_impulse(int32_t p_id, Vector2 impulse);
	void set_forced_velocity(int32_t p_id, Vector2 velocity);
	void clear_forced_velocity(int32_t p_id);
	bool move_to(int32_t p_id, Vector2 position);
	bool move_to_target(int32_t p_id, int32_t p_targetId, float minimumDistance);
	void pause_movement(int32_t p_id);
	void unpause_movement(int32_t p_id);
	void cancel_move(int32_t p_id);
	void reset_control(int32_t p_id);
	void add_obstacle_outline(int32_t p_id, const PackedVector2Array &p_outline);
	void remove_obstacle(int32_t p_id);
};

VARIANT_ENUM_CAST(Box2dPhysics::MovementMode);

#endif // BOX2D_PHYSICS_H
