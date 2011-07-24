-- Lua FFI binding to Bullet-2.75 using its (unmaintained) C-API
-- Garbage Collection is done implicitly (TODO: check if working)

-- beware: code not tested !!!

local ffi = require 'ffi'
ffi.cdef[[
typedef float	plReal;   // *** 'double' if BT_USE_DOUBLE_PRECISION ***
typedef plReal	plVector3[3];
typedef plReal	plQuaternion[4];

// Structs
typedef struct { int unused; } *plPhysicsSdkHandle;
typedef struct { int unused; } *plDynamicsWorldHandle;
typedef struct { int unused; } *plRigidBodyHandle;
typedef struct { int unused; } *plCollisionShapeHandle;
typedef struct { int unused; } *plConstraintHandle;
typedef struct { int unused; } *plMeshInterfaceHandle;
typedef struct { int unused; } *plCollisionBroadphaseHandle;
typedef struct { int unused; } *plBroadphaseProxyHandle;
typedef struct { int unused; } *plCollisionWorldHandle;

// Physics SDK
plPhysicsSdkHandle plNewBulletSdk(void);
void plDeletePhysicsSdk(plPhysicsSdkHandle	physicsSdk);

// Dynamics World
plDynamicsWorldHandle plCreateDynamicsWorld(plPhysicsSdkHandle physicsSdkHandle);
void plDeleteDynamicsWorld(plDynamicsWorldHandle world);
void plSetGravity(plDynamicsWorldHandle world, const plVector3 gravity);
void plStepSimulation(plDynamicsWorldHandle world, plReal timeStep);
void plAddRigidBody(plDynamicsWorldHandle world, plRigidBodyHandle object);
void plRemoveRigidBody(plDynamicsWorldHandle world, plRigidBodyHandle object);

// Rigid Body
plRigidBodyHandle plCreateRigidBody(void* user_data, float mass, plCollisionShapeHandle cshape);
void plDeleteRigidBody(plRigidBodyHandle cbody);

// Collision Shape definition
void plDeleteShape(plCollisionShapeHandle cshape);

plCollisionShapeHandle plNewSphereShape(plReal radius);
plCollisionShapeHandle plNewBoxShape(plReal x, plReal y, plReal z);
plCollisionShapeHandle plNewCapsuleShape(plReal radius, plReal height);
plCollisionShapeHandle plNewConeShape(plReal radius, plReal height);
plCollisionShapeHandle plNewCylinderShape(plReal radius, plReal height);

// Convex Meshes                    // not done
// Concave static triangle meshes   // not done

// Compound Shape
plCollisionShapeHandle plNewCompoundShape();
void plAddChildShape(plCollisionShapeHandle compoundShapeHandle, plCollisionShapeHandle childShapeHandle, plVector3 childPos, plQuaternion childOrn);

// Set
void plSetPosition(plRigidBodyHandle object, const plVector3 position);
void plSetOrientation(plRigidBodyHandle object, const plQuaternion orientation);
void plSetScaling(plCollisionShapeHandle cshape, plVector3 cscaling);
void plSetOpenGLMatrix(plRigidBodyHandle object, plReal* matrix);

// Get
void plGetPosition(plRigidBodyHandle object,plVector3 position);
void plGetOrientation(plRigidBodyHandle object,plQuaternion orientation);
// void plGetScale ??
void plGetOpenGLMatrix(plRigidBodyHandle object, plReal* matrix);

// Extra
//void plSetEuler(plReal yaw, plReal pitch, plReal roll, plQuaternion orient);
//double plNearestPoints(float p1[3], float p2[3], float p3[3], float q1[3], float q2[3], float q3[3], float *pa, float *pb, float normal[3]);
]]

local bullet = ffi.load 'BulletDynamics'

local sdk = bullet.plNewBulletSdk()
ffi.gc(sdk, bullet.plDeletePhysicsSdk)

-- Util- Vector
local Vector_metatable = {__tostring = function(self) 
    return string.format("Vector<%f,%f,%f>", self.x, self.y, self.z) 
end}

function Vector(x,y,z) 
    local foo = {x=x, y=y, z=z}
    setmetatable(foo, Vector_metatable)
    return foo
end 

-- Dynamics World
local World = {}
World.__index = World

function World:new()
    local foo   = {}
    foo.bodies  = {}
    
    foo.btWorld = bullet.plCreateDynamicsWorld(sdk)
    ffi.gc(foo.btWorld, bullet.plDeleteDynamicsWorld)
    
    setmetatable(foo, World)
    return foo
end

function World:step(delta)
    if not (delta and type(delta) == 'number') then error("Time not given") end

    bullet.plStepSimulation(self.btWorld, delta)

    local pos = ffi.new('plVector3')
    local rot = ffi.new('plQuaternion')
    for i,v in ipairs(self.bodies) do
        bullet.plGetPosition(v.btBody, pos)
        bullet.plGetOrientation(v.btBody, rot)

        rawset(v, 'position', Vector(pos[0], pos[1], pos[2]))
        --rawset(v, 'rotation', {rot[0]. rot[1]. rot[2]}) -- convert quat to v3
    end
end

function World:add(body) 
    if not (body and body.btBody) then error("Body not given") end
    
    bullet.plAddRigidBody(self.btWorld, body.btBody)
    
    table.insert(self.bodies, body)
    self.bodies[body] = #self.bodies
end

function World:remove(body)
    if not (body and body.btBody) then error("Body not given") end

    if self.bodies[body] then
        bullet.plRemoveRigidBody(self.btWorld, body.btBody)
        
        local index = self.bodies[body]
        self.bodies[body] = nil
        self.bodies[index] = nil
    end
end

-- Rigid Body
local Body = {}
Body.__index = Body
Body.__newindex = function(self, key, value)
    if not (key=='position' or key=='rotation' or key=='scale') then
        rawset(self, key, value)
    else
        if not (value and type(value)=='table') then error("Vector not given") end

        if key == 'position' then
            local v3 = ffi.new('plVector3')
            v3[0], v3[1], v3[2] = value.x, value.y, value.z

            bullet.plSetPosition(self.btBody, v3)
            rawset(self, 'position', value)
        end -- Rotation not done
    end
end
Body.shape  = nil
Body.mass   = nil

function Body:new(shape, mass)
    if not (shape and shape.btShape) then error("Shape not given") end
    mass = mass or 0

    local foo       = {}
    foo.position    = Vector(0,0,0)
    foo.rotation    = Vector(0,0,0)
    foo.scale       = Vector(1,1,1)

    foo.mass    = mass
    foo.shape   = shape
    
    foo.btBody  = bullet.plCreateRigidBody(nil, mass, shape.btShape)
    ffi.gc(foo.btBody, bullet.plDeleteRigidBody)
    
    setmetatable(foo, Body)
    return foo
end

-- Shape Factory
local Shape = {}
Shape.__index = Shape
Shape.type = nil        -- Box, Sphere

function Shape:newBox(width, height, depth) 
    if not (width and height and depth and type(width)=='number') then
        error("Width, height, or depth not given")
    end

    local foo   = {}
    foo.type    = 'box' 
    
    foo.width   = width
    foo.height  = height
    foo.depth   = depth
    
    foo.btShape = bullet.plNewBoxShape(width, height, depth)
    ffi.gc(foo.btShape, bullet.plDeleteShape)
    
    setmetatable(foo, Shape)
    return foo
end

function Shape:newSphere(radius) 
    if not (radius and type(radius)=='number') then error("Radius not given") end

    local foo   = {}
    foo.type    = 'sphere'
    
    foo.radius  = radius
    
    foo.btShape = bullet.plNewSphereShape(radius)
    ffi.gc(foo.btShape, bullet.plDeleteShape)
    
    setmetatable(foo, Shape)
    return foo
end

local library = {Vector=Vector, World=World, Body=Body, Shape=Shape}
return library
