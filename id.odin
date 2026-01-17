// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
package box2d

foreign import lib "lib/box2d.lib"
_ :: lib

/// World id references a world instance. This should be treated as an opaque handle.
WorldId :: struct {
	index1:     u16,
	generation: u16,
}

/// Body id references a body instance. This should be treated as an opaque handle.
BodyId :: struct {
	index1:     i32,
	world0:     u16,
	generation: u16,
}

/// Shape id references a shape instance. This should be treated as an opaque handle.
ShapeId :: struct {
	index1:     i32,
	world0:     u16,
	generation: u16,
}

/// Chain id references a chain instances. This should be treated as an opaque handle.
ChainId :: struct {
	index1:     i32,
	world0:     u16,
	generation: u16,
}

/// Joint id references a joint instance. This should be treated as an opaque handle.
JointId :: struct {
	index1:     i32,
	world0:     u16,
	generation: u16,
}

/// Contact id references a contact instance. This should be treated as an opaque handled.
ContactId :: struct {
	index1:     i32,
	world0:     u16,
	padding:    i16,
	generation: u32,
}

/// Use these to make your identifiers null.
/// You may also use zero initialization to get null.
nullWorldId :: WorldId{}
nullBodyId  :: BodyId{}
nullShapeId :: ShapeId{}
nullChainId :: ChainId{}
nullJointId :: JointId{}

/// Macro to determine if any id is null.
IS_NULL :: #force_inline proc "c" (id: $T) -> bool
	where intrinsics.type_is_struct(T),
	      intrinsics.type_has_field(T, "index1") {
	return id.index1 == 0
}

/// Macro to determine if any id is non-null.
IS_NON_NULL :: #force_inline proc "c" (id: $T) -> bool
	where intrinsics.type_is_struct(T),
	      intrinsics.type_has_field(T, "index1") {
	return id.index1 != 0
}

/// Compare two ids for equality. Doesn't work for b2WorldId.
ID_EQUALS :: #force_inline proc "c" (id1, id2: $T) -> bool
	where intrinsics.type_is_struct(T),
	      intrinsics.type_has_field(T, "index1"),
	      intrinsics.type_has_field(T, "world0"),
	      intrinsics.type_has_field(T, "generation") {
	return id1.index1 == id2.index1 && id1.world0 == id2.world0 && id1.generation == id2.generation
}
