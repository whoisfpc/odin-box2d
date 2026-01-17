// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT
package box2d

foreign import lib "lib/box2d.lib"
_ :: lib

/**
* @defgroup geometry Geometry
* @brief Geometry types and algorithms
*
* Definitions of circles, capsules, segments, and polygons. Various algorithms to compute hulls, mass properties, and so on.
* Functions should take the shape as the first argument to assist editor auto-complete.
* @{
*/

/// The maximum number of vertices on a convex polygon. Changing this affects performance even if you
/// don't use more vertices.
MAX_POLYGON_VERTICES :: 8

/// Low level ray cast input data
RayCastInput :: struct {
	/// Start point of the ray cast
	origin: Vec2,

	/// Translation of the ray cast
	translation: Vec2,

	/// The maximum fraction of the translation to consider, typically 1
	maxFraction: f32,
}

/// A distance proxy is used by the GJK algorithm. It encapsulates any shape.
/// You can provide between 1 and B2_MAX_POLYGON_VERTICES and a radius.
ShapeProxy :: struct {
	/// The point cloud
	points: [8]Vec2 `fmt:"v,count"`,

	/// The number of points. Must be greater than 0.
	count: i32,

	/// The external radius of the point cloud. May be zero.
	radius: f32,
}

/// Low level shape cast input in generic form. This allows casting an arbitrary point
/// cloud wrap with a radius. For example, a circle is a single point with a non-zero radius.
/// A capsule is two points with a non-zero radius. A box is four points with a zero radius.
ShapeCastInput :: struct {
	/// A generic shape
	proxy: ShapeProxy,

	/// The translation of the shape cast
	translation: Vec2,

	/// The maximum fraction of the translation to consider, typically 1
	maxFraction: f32,

	/// Allow shape cast to encroach when initially touching. This only works if the radius is greater than zero.
	canEncroach: bool,
}

/// Low level ray cast or shape-cast output data. Returns a zero fraction and normal in the case of initial overlap.
CastOutput :: struct {
	/// The surface normal at the hit point
	normal: Vec2,

	/// The surface hit point
	point: Vec2,

	/// The fraction of the input translation at collision
	fraction: f32,

	/// The number of iterations used
	iterations: i32,

	/// Did the cast hit?
	hit: bool,
}

/// This holds the mass data computed for a shape.
MassData :: struct {
	/// The mass of the shape, usually in kilograms.
	mass: f32,

	/// The position of the shape's centroid relative to the shape's origin.
	center: Vec2,

	/// The rotational inertia of the shape about the shape center.
	rotationalInertia: f32,
}

/// A solid circle
Circle :: struct {
	/// The local center
	center: Vec2,

	/// The radius
	radius: f32,
}

/// A solid capsule can be viewed as two semicircles connected
/// by a rectangle.
Capsule :: struct {
	/// Local center of the first semicircle
	center1: Vec2,

	/// Local center of the second semicircle
	center2: Vec2,

	/// The radius of the semicircles
	radius: f32,
}

/// A solid convex polygon. It is assumed that the interior of the polygon is to
/// the left of each edge.
/// Polygons have a maximum number of vertices equal to B2_MAX_POLYGON_VERTICES.
/// In most cases you should not need many vertices for a convex polygon.
/// @warning DO NOT fill this out manually, instead use a helper function like
/// b2MakePolygon or b2MakeBox.
Polygon :: struct {
	/// The polygon vertices
	vertices: [8]Vec2 `fmt:"v,count"`,

	/// The outward normal vectors of the polygon sides
	normals: [8]Vec2 `fmt:"v,count"`,

	/// The centroid of the polygon
	centroid: Vec2,

	/// The external radius for rounded polygons
	radius: f32,

	/// The number of polygon vertices
	count: i32,
}

/// A line segment with two-sided collision.
Segment :: struct {
	/// The first point
	point1: Vec2,

	/// The second point
	point2: Vec2,
}

/// A line segment with one-sided collision. Only collides on the right side.
/// Several of these are generated for a chain shape.
/// ghost1 -> point1 -> point2 -> ghost2
ChainSegment :: struct {
	/// The tail ghost vertex
	ghost1: Vec2,

	/// The line segment
	segment: Segment,

	/// The head ghost vertex
	ghost2: Vec2,

	/// The owning chain shape index (internal usage only)
	chainId: i32,
}

@(default_calling_convention="c", link_prefix="b2")
foreign lib {
	/// Validate ray cast input data (NaN, etc)
	IsValidRay :: proc(#by_ptr input: RayCastInput) -> bool ---

	/// Make a convex polygon from a convex hull. This will assert if the hull is not valid.
	/// @warning Do not manually fill in the hull data, it must come directly from b2ComputeHull
	MakePolygon :: proc(#by_ptr hull: Hull, radius: f32) -> Polygon ---

	/// Make an offset convex polygon from a convex hull. This will assert if the hull is not valid.
	/// @warning Do not manually fill in the hull data, it must come directly from b2ComputeHull
	MakeOffsetPolygon :: proc(#by_ptr hull: Hull, position: Vec2, rotation: Rot) -> Polygon ---

	/// Make an offset convex polygon from a convex hull. This will assert if the hull is not valid.
	/// @warning Do not manually fill in the hull data, it must come directly from b2ComputeHull
	MakeOffsetRoundedPolygon :: proc(#by_ptr hull: Hull, position: Vec2, rotation: Rot, radius: f32) -> Polygon ---

	/// Make a square polygon, bypassing the need for a convex hull.
	/// @param halfWidth the half-width
	MakeSquare :: proc(halfWidth: f32) -> Polygon ---

	/// Make a box (rectangle) polygon, bypassing the need for a convex hull.
	/// @param halfWidth the half-width (x-axis)
	/// @param halfHeight the half-height (y-axis)
	MakeBox :: proc(halfWidth: f32, halfHeight: f32) -> Polygon ---

	/// Make a rounded box, bypassing the need for a convex hull.
	/// @param halfWidth the half-width (x-axis)
	/// @param halfHeight the half-height (y-axis)
	/// @param radius the radius of the rounded extension
	MakeRoundedBox :: proc(halfWidth: f32, halfHeight: f32, radius: f32) -> Polygon ---

	/// Make an offset box, bypassing the need for a convex hull.
	/// @param halfWidth the half-width (x-axis)
	/// @param halfHeight the half-height (y-axis)
	/// @param center the local center of the box
	/// @param rotation the local rotation of the box
	MakeOffsetBox :: proc(halfWidth: f32, halfHeight: f32, center: Vec2, rotation: Rot) -> Polygon ---

	/// Make an offset rounded box, bypassing the need for a convex hull.
	/// @param halfWidth the half-width (x-axis)
	/// @param halfHeight the half-height (y-axis)
	/// @param center the local center of the box
	/// @param rotation the local rotation of the box
	/// @param radius the radius of the rounded extension
	MakeOffsetRoundedBox :: proc(halfWidth: f32, halfHeight: f32, center: Vec2, rotation: Rot, radius: f32) -> Polygon ---

	/// Transform a polygon. This is useful for transferring a shape from one body to another.
	TransformPolygon :: proc(transform: Transform, #by_ptr polygon: Polygon) -> Polygon ---

	/// Compute mass properties of a circle
	ComputeCircleMass :: proc(#by_ptr shape: Circle, density: f32) -> MassData ---

	/// Compute mass properties of a capsule
	ComputeCapsuleMass :: proc(#by_ptr shape: Capsule, density: f32) -> MassData ---

	/// Compute mass properties of a polygon
	ComputePolygonMass :: proc(#by_ptr shape: Polygon, density: f32) -> MassData ---

	/// Compute the bounding box of a transformed circle
	ComputeCircleAABB :: proc(#by_ptr shape: Circle, transform: Transform) -> AABB ---

	/// Compute the bounding box of a transformed capsule
	ComputeCapsuleAABB :: proc(#by_ptr shape: Capsule, transform: Transform) -> AABB ---

	/// Compute the bounding box of a transformed polygon
	ComputePolygonAABB :: proc(#by_ptr shape: Polygon, transform: Transform) -> AABB ---

	/// Compute the bounding box of a transformed line segment
	ComputeSegmentAABB :: proc(#by_ptr shape: Segment, transform: Transform) -> AABB ---

	/// Test a point for overlap with a circle in local space
	PointInCircle :: proc(#by_ptr shape: Circle, point: Vec2) -> bool ---

	/// Test a point for overlap with a capsule in local space
	PointInCapsule :: proc(#by_ptr shape: Capsule, point: Vec2) -> bool ---

	/// Test a point for overlap with a convex polygon in local space
	PointInPolygon :: proc(#by_ptr shape: Polygon, point: Vec2) -> bool ---

	/// Ray cast versus circle shape in local space.
	RayCastCircle :: proc(#by_ptr shape: Circle, #by_ptr input: RayCastInput) -> CastOutput ---

	/// Ray cast versus capsule shape in local space.
	RayCastCapsule :: proc(#by_ptr shape: Capsule, #by_ptr input: RayCastInput) -> CastOutput ---

	/// Ray cast versus segment shape in local space. Optionally treat the segment as one-sided with hits from
	/// the left side being treated as a miss.
	RayCastSegment :: proc(#by_ptr shape: Segment, #by_ptr input: RayCastInput, oneSided: bool) -> CastOutput ---

	/// Ray cast versus polygon shape in local space.
	RayCastPolygon :: proc(#by_ptr shape: Polygon, #by_ptr input: RayCastInput) -> CastOutput ---

	/// Shape cast versus a circle.
	ShapeCastCircle :: proc(#by_ptr shape: Circle, #by_ptr input: ShapeCastInput) -> CastOutput ---

	/// Shape cast versus a capsule.
	ShapeCastCapsule :: proc(#by_ptr shape: Capsule, #by_ptr input: ShapeCastInput) -> CastOutput ---

	/// Shape cast versus a line segment.
	ShapeCastSegment :: proc(#by_ptr shape: Segment, #by_ptr input: ShapeCastInput) -> CastOutput ---

	/// Shape cast versus a convex polygon.
	ShapeCastPolygon :: proc(#by_ptr shape: Polygon, #by_ptr input: ShapeCastInput) -> CastOutput ---
}

/// A convex hull. Used to create convex polygons.
/// @warning Do not modify these values directly, instead use b2ComputeHull()
Hull :: struct {
	/// The final points of the hull
	points: [8]Vec2 `fmt:"v,count"`,

	/// The number of points
	count: i32,
}

@(default_calling_convention="c", link_prefix="b2")
foreign lib {
	/// Compute the convex hull of a set of points. Returns an empty hull if it fails.
	/// Some failure cases:
	/// - all points very close together
	/// - all points on a line
	/// - less than 3 points
	/// - more than B2_MAX_POLYGON_VERTICES points
	/// This welds close points and removes collinear points.
	/// @warning Do not modify a hull once it has been computed
	ComputeHull :: proc(points: ^Vec2, count: i32) -> Hull ---

	/// This determines if a hull is valid. Checks for:
	/// - convexity
	/// - collinear points
	/// This is expensive and should not be called at runtime.
	ValidateHull :: proc(#by_ptr hull: Hull) -> bool ---
}

/// Result of computing the distance between two line segments
SegmentDistanceResult :: struct {
	/// The closest point on the first segment
	closest1: Vec2,

	/// The closest point on the second segment
	closest2: Vec2,

	/// The barycentric coordinate on the first segment
	fraction1: f32,

	/// The barycentric coordinate on the second segment
	fraction2: f32,

	/// The squared distance between the closest points
	distanceSquared: f32,
}

@(default_calling_convention="c", link_prefix="b2")
foreign lib {
	/// Compute the distance between two line segments, clamping at the end points if needed.
	SegmentDistance :: proc(p1: Vec2, q1: Vec2, p2: Vec2, q2: Vec2) -> SegmentDistanceResult ---
}

/// Used to warm start the GJK simplex. If you call this function multiple times with nearby
/// transforms this might improve performance. Otherwise you can zero initialize this.
/// The distance cache must be initialized to zero on the first call.
/// Users should generally just zero initialize this structure for each call.
SimplexCache :: struct {
	/// The number of stored simplex points
	count: u16,

	/// The cached simplex indices on shape A
	indexA: [3]u8 `fmt:"v,count"`,

	/// The cached simplex indices on shape B
	indexB: [3]u8 `fmt:"v,count"`,
}

/// Input for b2ShapeDistance
DistanceInput :: struct {
	/// The proxy for shape A
	proxyA: ShapeProxy,

	/// The proxy for shape B
	proxyB: ShapeProxy,

	/// The world transform for shape A
	transformA: Transform,

	/// The world transform for shape B
	transformB: Transform,

	/// Should the proxy radius be considered?
	useRadii: bool,
}

/// Output for b2ShapeDistance
DistanceOutput :: struct {
	pointA:       Vec2, ///< Closest point on shapeA
	pointB:       Vec2, ///< Closest point on shapeB
	normal:       Vec2, ///< Normal vector that points from A to B. Invalid if distance is zero.
	distance:     f32,  ///< The final distance, zero if overlapped
	iterations:   i32,  ///< Number of GJK iterations used
	simplexCount: i32,  ///< The number of simplexes stored in the simplex array
}

/// Simplex vertex for debugging the GJK algorithm
SimplexVertex :: struct {
	wA:     Vec2, ///< support point in proxyA
	wB:     Vec2, ///< support point in proxyB
	w:      Vec2, ///< wB - wA
	a:      f32,  ///< barycentric coordinate for closest point
	indexA: i32,  ///< wA index
	indexB: i32,  ///< wB index
}

/// Simplex from the GJK algorithm
Simplex :: struct {
	v1, v2, v3: SimplexVertex, ///< vertices
	count:      i32,           ///< number of valid vertices
}

@(default_calling_convention="c", link_prefix="b2")
foreign lib {
	/// Compute the closest points between two shapes represented as point clouds.
	/// b2SimplexCache cache is input/output. On the first call set b2SimplexCache.count to zero.
	/// The underlying GJK algorithm may be debugged by passing in debug simplexes and capacity. You may pass in NULL and 0 for these.
	ShapeDistance :: proc(#by_ptr input: DistanceInput, cache: ^SimplexCache, simplexes: ^Simplex, simplexCapacity: i32) -> DistanceOutput ---
}

/// Input parameters for b2ShapeCast
ShapeCastPairInput :: struct {
	proxyA:       ShapeProxy, ///< The proxy for shape A
	proxyB:       ShapeProxy, ///< The proxy for shape B
	transformA:   Transform,  ///< The world transform for shape A
	transformB:   Transform,  ///< The world transform for shape B
	translationB: Vec2,       ///< The translation of shape B
	maxFraction:  f32,        ///< The fraction of the translation to consider, typically 1
	canEncroach:  bool,       ///< Allows shapes with a radius to move slightly closer if already touching
}

@(default_calling_convention="c", link_prefix="b2")
foreign lib {
	/// Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point, normal, and translation fraction.
	/// Initially touching shapes are treated as a miss.
	ShapeCast :: proc(#by_ptr input: ShapeCastPairInput) -> CastOutput ---

	/// Make a proxy for use in overlap, shape cast, and related functions. This is a deep copy of the points.
	MakeProxy :: proc(points: ^Vec2, count: i32, radius: f32) -> ShapeProxy ---

	/// Make a proxy with a transform. This is a deep copy of the points.
	MakeOffsetProxy :: proc(points: ^Vec2, count: i32, radius: f32, position: Vec2, rotation: Rot) -> ShapeProxy ---
}

/// This describes the motion of a body/shape for TOI computation. Shapes are defined with respect to the body origin,
/// which may not coincide with the center of mass. However, to support dynamics we must interpolate the center of mass
/// position.
Sweep :: struct {
	localCenter: Vec2, ///< Local center of mass position
	c1:          Vec2, ///< Starting center of mass world position
	c2:          Vec2, ///< Ending center of mass world position
	q1:          Rot,  ///< Starting world rotation
	q2:          Rot,  ///< Ending world rotation
}

@(default_calling_convention="c", link_prefix="b2")
foreign lib {
	/// Evaluate the transform sweep at a specific time.
	GetSweepTransform :: proc(#by_ptr sweep: Sweep, time: f32) -> Transform ---
}

/// Time of impact input
TOIInput :: struct {
	proxyA:      ShapeProxy, ///< The proxy for shape A
	proxyB:      ShapeProxy, ///< The proxy for shape B
	sweepA:      Sweep,      ///< The movement of shape A
	sweepB:      Sweep,      ///< The movement of shape B
	maxFraction: f32,        ///< Defines the sweep interval [0, maxFraction]
}

/// Describes the TOI output
TOIState :: enum i32 {
	Unknown    = 0,
	Failed     = 1,
	Overlapped = 2,
	Hit        = 3,
	Separated  = 4,
}

/// Time of impact output
TOIOutput :: struct {
	/// The type of result
	state: TOIState,

	/// The hit point
	point: Vec2,

	/// The hit normal
	normal: Vec2,

	/// The sweep time of the collision
	fraction: f32,
}

@(default_calling_convention="c", link_prefix="b2")
foreign lib {
	/// Compute the upper bound on time before two shapes penetrate. Time is represented as
	/// a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
	/// non-tunneling collisions. If you change the time interval, you should call this function
	/// again.
	TimeOfImpact :: proc(#by_ptr input: TOIInput) -> TOIOutput ---
}

/// A manifold point is a contact point belonging to a contact manifold.
/// It holds details related to the geometry and dynamics of the contact points.
/// Box2D uses speculative collision so some contact points may be separated.
/// You may use the totalNormalImpulse to determine if there was an interaction during
/// the time step.
ManifoldPoint :: struct {
	/// Location of the contact point in world space. Subject to precision loss at large coordinates.
	/// @note Should only be used for debugging.
	point: Vec2,

	/// Location of the contact point relative to shapeA's origin in world space
	/// @note When used internally to the Box2D solver, this is relative to the body center of mass.
	anchorA: Vec2,

	/// Location of the contact point relative to shapeB's origin in world space
	/// @note When used internally to the Box2D solver, this is relative to the body center of mass.
	anchorB: Vec2,

	/// The separation of the contact point, negative if penetrating
	separation: f32,

	/// The impulse along the manifold normal vector.
	normalImpulse: f32,

	/// The friction impulse
	tangentImpulse: f32,

	/// The total normal impulse applied across sub-stepping and restitution. This is important
	/// to identify speculative contact points that had an interaction in the time step.
	totalNormalImpulse: f32,

	/// Relative normal velocity pre-solve. Used for hit events. If the normal impulse is
	/// zero then there was no hit. Negative means shapes are approaching.
	normalVelocity: f32,

	/// Uniquely identifies a contact point between two shapes
	id: u16,

	/// Did this contact point exist the previous step?
	persisted: bool,
}

/// A contact manifold describes the contact points between colliding shapes.
/// @note Box2D uses speculative collision so some contact points may be separated.
Manifold :: struct {
	/// The unit normal vector in world space, points from shape A to bodyB
	normal: Vec2,

	/// Angular impulse applied for rolling resistance. N * m * s = kg * m^2 / s
	rollingImpulse: f32,

	/// The manifold points, up to two are possible in 2D
	points: [2]ManifoldPoint,

	/// The number of contacts points, will be 0, 1, or 2
	pointCount: i32,
}

@(default_calling_convention="c", link_prefix="b2")
foreign lib {
	/// Compute the contact manifold between two circles
	CollideCircles :: proc(#by_ptr circleA: Circle, xfA: Transform, #by_ptr circleB: Circle, xfB: Transform) -> Manifold ---

	/// Compute the contact manifold between a capsule and circle
	CollideCapsuleAndCircle :: proc(#by_ptr capsuleA: Capsule, xfA: Transform, #by_ptr circleB: Circle, xfB: Transform) -> Manifold ---

	/// Compute the contact manifold between an segment and a circle
	CollideSegmentAndCircle :: proc(#by_ptr segmentA: Segment, xfA: Transform, #by_ptr circleB: Circle, xfB: Transform) -> Manifold ---

	/// Compute the contact manifold between a polygon and a circle
	CollidePolygonAndCircle :: proc(#by_ptr polygonA: Polygon, xfA: Transform, #by_ptr circleB: Circle, xfB: Transform) -> Manifold ---

	/// Compute the contact manifold between a capsule and circle
	CollideCapsules :: proc(#by_ptr capsuleA: Capsule, xfA: Transform, #by_ptr capsuleB: Capsule, xfB: Transform) -> Manifold ---

	/// Compute the contact manifold between an segment and a capsule
	CollideSegmentAndCapsule :: proc(#by_ptr segmentA: Segment, xfA: Transform, #by_ptr capsuleB: Capsule, xfB: Transform) -> Manifold ---

	/// Compute the contact manifold between a polygon and capsule
	CollidePolygonAndCapsule :: proc(#by_ptr polygonA: Polygon, xfA: Transform, #by_ptr capsuleB: Capsule, xfB: Transform) -> Manifold ---

	/// Compute the contact manifold between two polygons
	CollidePolygons :: proc(#by_ptr polygonA: Polygon, xfA: Transform, #by_ptr polygonB: Polygon, xfB: Transform) -> Manifold ---

	/// Compute the contact manifold between an segment and a polygon
	CollideSegmentAndPolygon :: proc(#by_ptr segmentA: Segment, xfA: Transform, #by_ptr polygonB: Polygon, xfB: Transform) -> Manifold ---

	/// Compute the contact manifold between a chain segment and a circle
	CollideChainSegmentAndCircle :: proc(#by_ptr segmentA: ChainSegment, xfA: Transform, #by_ptr circleB: Circle, xfB: Transform) -> Manifold ---

	/// Compute the contact manifold between a chain segment and a capsule
	CollideChainSegmentAndCapsule :: proc(#by_ptr segmentA: ChainSegment, xfA: Transform, #by_ptr capsuleB: Capsule, xfB: Transform, cache: ^SimplexCache) -> Manifold ---

	/// Compute the contact manifold between a chain segment and a rounded polygon
	CollideChainSegmentAndPolygon :: proc(#by_ptr segmentA: ChainSegment, xfA: Transform, #by_ptr polygonB: Polygon, xfB: Transform, cache: ^SimplexCache) -> Manifold ---
}

/// The dynamic tree structure. This should be considered private data.
/// It is placed here for performance reasons.
DynamicTree :: struct {
	/// The tree nodes
	nodes: [^]TreeNode,

	/// The root index
	root: i32,

	/// The number of nodes
	nodeCount: i32,

	/// The allocated node space
	nodeCapacity: i32,

	/// Node free list
	freeList: i32,

	/// Number of proxies created
	proxyCount: i32,

	/// Leaf indices for rebuild
	leafIndices: [^]i32,

	/// Leaf bounding boxes for rebuild
	leafBoxes: [^]AABB,

	/// Leaf bounding box centers for rebuild
	leafCenters: [^]Vec2,

	/// Bins for sorting during rebuild
	binIndices: [^]i32,

	/// Allocated space for rebuilding
	rebuildCapacity: i32,
}

TreeNode :: struct {}

/// These are performance results returned by dynamic tree queries.
TreeStats :: struct {
	/// Number of internal nodes visited during the query
	nodeVisits: i32,

	/// Number of leaf nodes visited during the query
	leafVisits: i32,
}

@(default_calling_convention="c", link_prefix="b2")
foreign lib {
	/// Constructing the tree initializes the node pool.
	DynamicTree_Create :: proc() -> DynamicTree ---

	/// Destroy the tree, freeing the node pool.
	DynamicTree_Destroy :: proc(tree: ^DynamicTree) ---

	/// Create a proxy. Provide an AABB and a userData value.
	DynamicTree_CreateProxy :: proc(tree: ^DynamicTree, aabb: AABB, categoryBits: u64, userData: u64) -> i32 ---

	/// Destroy a proxy. This asserts if the id is invalid.
	DynamicTree_DestroyProxy :: proc(tree: ^DynamicTree, proxyId: i32) ---

	/// Move a proxy to a new AABB by removing and reinserting into the tree.
	DynamicTree_MoveProxy :: proc(tree: ^DynamicTree, proxyId: i32, aabb: AABB) ---

	/// Enlarge a proxy and enlarge ancestors as necessary.
	DynamicTree_EnlargeProxy :: proc(tree: ^DynamicTree, proxyId: i32, aabb: AABB) ---

	/// Modify the category bits on a proxy. This is an expensive operation.
	DynamicTree_SetCategoryBits :: proc(tree: ^DynamicTree, proxyId: i32, categoryBits: u64) ---

	/// Get the category bits on a proxy.
	DynamicTree_GetCategoryBits :: proc(tree: ^DynamicTree, proxyId: i32) -> u64 ---
}

/// This function receives proxies found in the AABB query.
/// @return true if the query should continue
TreeQueryCallbackFcn :: proc "c" (proxyId: i32, userData: u64, _context: rawptr) -> bool

@(default_calling_convention="c", link_prefix="b2")
foreign lib {
	/// Query an AABB for overlapping proxies. The callback class is called for each proxy that overlaps the supplied AABB.
	///	@return performance data
	DynamicTree_Query :: proc(#by_ptr tree: DynamicTree, aabb: AABB, maskBits: u64, callback: TreeQueryCallbackFcn, _context: rawptr) -> TreeStats ---

	/// Query an AABB for overlapping proxies. The callback class is called for each proxy that overlaps the supplied AABB.
	/// No filtering is performed.
	///	@return performance data
	DynamicTree_QueryAll :: proc(tree: ^DynamicTree, aabb: AABB, callback: TreeQueryCallbackFcn, _context: rawptr) -> TreeStats ---
}

/// This function receives clipped ray cast input for a proxy. The function
/// returns the new ray fraction.
/// - return a value of 0 to terminate the ray cast
/// - return a value less than input->maxFraction to clip the ray
/// - return a value of input->maxFraction to continue the ray cast without clipping
TreeRayCastCallbackFcn :: proc "c" (input: ^RayCastInput, proxyId: i32, userData: u64, _context: rawptr) -> f32

@(default_calling_convention="c", link_prefix="b2")
foreign lib {
	/// Ray cast against the proxies in the tree. This relies on the callback
	/// to perform a exact ray cast in the case were the proxy contains a shape.
	/// The callback also performs the any collision filtering. This has performance
	/// roughly equal to k * log(n), where k is the number of collisions and n is the
	/// number of proxies in the tree.
	/// Bit-wise filtering using mask bits can greatly improve performance in some scenarios.
	///	However, this filtering may be approximate, so the user should still apply filtering to results.
	/// @param tree the dynamic tree to ray cast
	/// @param input the ray cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1)
	/// @param maskBits mask bit hint: `bool accept = (maskBits & node->categoryBits) != 0;`
	/// @param callback a callback class that is called for each proxy that is hit by the ray
	/// @param context user context that is passed to the callback
	///	@return performance data
	DynamicTree_RayCast :: proc(#by_ptr tree: DynamicTree, #by_ptr input: RayCastInput, maskBits: u64, callback: TreeRayCastCallbackFcn, _context: rawptr) -> TreeStats ---
}

/// This function receives clipped ray cast input for a proxy. The function
/// returns the new ray fraction.
/// - return a value of 0 to terminate the ray cast
/// - return a value less than input->maxFraction to clip the ray
/// - return a value of input->maxFraction to continue the ray cast without clipping
TreeShapeCastCallbackFcn :: proc "c" (input: ^ShapeCastInput, proxyId: i32, userData: u64, _context: rawptr) -> f32

@(default_calling_convention="c", link_prefix="b2")
foreign lib {
	/// Ray cast against the proxies in the tree. This relies on the callback
	/// to perform a exact ray cast in the case were the proxy contains a shape.
	/// The callback also performs the any collision filtering. This has performance
	/// roughly equal to k * log(n), where k is the number of collisions and n is the
	/// number of proxies in the tree.
	/// @param tree the dynamic tree to ray cast
	/// @param input the ray cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
	/// @param maskBits filter bits: `bool accept = (maskBits & node->categoryBits) != 0;`
	/// @param callback a callback class that is called for each proxy that is hit by the shape
	/// @param context user context that is passed to the callback
	///	@return performance data
	DynamicTree_ShapeCast :: proc(#by_ptr tree: DynamicTree, #by_ptr input: ShapeCastInput, maskBits: u64, callback: TreeShapeCastCallbackFcn, _context: rawptr) -> TreeStats ---

	/// Get the height of the binary tree.
	DynamicTree_GetHeight :: proc(#by_ptr tree: DynamicTree) -> i32 ---

	/// Get the ratio of the sum of the node areas to the root area.
	DynamicTree_GetAreaRatio :: proc(#by_ptr tree: DynamicTree) -> f32 ---

	/// Get the bounding box that contains the entire tree
	DynamicTree_GetRootBounds :: proc(#by_ptr tree: DynamicTree) -> AABB ---

	/// Get the number of proxies created
	DynamicTree_GetProxyCount :: proc(#by_ptr tree: DynamicTree) -> i32 ---

	/// Rebuild the tree while retaining subtrees that haven't changed. Returns the number of boxes sorted.
	DynamicTree_Rebuild :: proc(tree: ^DynamicTree, fullBuild: bool) -> i32 ---

	/// Get the number of bytes used by this tree
	DynamicTree_GetByteCount :: proc(tree: ^DynamicTree) -> i32 ---

	/// Get proxy user data
	DynamicTree_GetUserData :: proc(#by_ptr tree: DynamicTree, proxyId: i32) -> u64 ---

	/// Get the AABB of a proxy
	DynamicTree_GetAABB :: proc(#by_ptr tree: DynamicTree, proxyId: i32) -> AABB ---

	/// Validate this tree. For testing.
	DynamicTree_Validate :: proc(#by_ptr tree: DynamicTree) ---

	/// Validate this tree has no enlarged AABBs. For testing.
	DynamicTree_ValidateNoEnlarged :: proc(#by_ptr tree: DynamicTree) ---
}

/// These are the collision planes returned from b2World_CollideMover
PlaneResult :: struct {
	/// The collision plane between the mover and a convex shape
	plane: Plane,

	// The collision point on the shape.
	point: Vec2,

	/// Did the collision register a hit? If not this plane should be ignored.
	hit: bool,
}

/// These are collision planes that can be fed to b2SolvePlanes. Normally
/// this is assembled by the user from plane results in b2PlaneResult
CollisionPlane :: struct {
	/// The collision plane between the mover and some shape
	plane: Plane,

	/// Setting this to FLT_MAX makes the plane as rigid as possible. Lower values can
	/// make the plane collision soft. Usually in meters.
	pushLimit: f32,

	/// The push on the mover determined by b2SolvePlanes. Usually in meters.
	push: f32,

	/// Indicates if b2ClipVector should clip against this plane. Should be false for soft collision.
	clipVelocity: bool,
}

/// Result returned by b2SolvePlanes
PlaneSolverResult :: struct {
	/// The translation of the mover
	translation: Vec2,

	/// The number of iterations used by the plane solver. For diagnostics.
	iterationCount: i32,
}

@(default_calling_convention="c", link_prefix="b2")
foreign lib {
	/// Solves the position of a mover that satisfies the given collision planes.
	/// @param targetDelta the desired movement from the position used to generate the collision planes
	/// @param planes the collision planes
	/// @param count the number of collision planes
	SolvePlanes :: proc(targetDelta: Vec2, planes: ^CollisionPlane, count: i32) -> PlaneSolverResult ---

	/// Clips the velocity against the given collision planes. Planes with zero push or clipVelocity
	/// set to false are skipped.
	ClipVector :: proc(vector: Vec2, planes: ^CollisionPlane, count: i32) -> Vec2 ---
}

