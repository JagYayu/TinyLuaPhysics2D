--- [TinyLuaPhysics2D](https://github.com/JagYayu/TinyLuaPhysics2D)
--- A tiny 2d physics engine written in lua, licensed under WTFPL.
--- Nov 2025, v0.3
---
--- ### Modules that not implement yet
--- * DynAABBTree
--- * SpatialHashMap
---
--- @author JagYayu
--- @readonly
--- @class TLP2D
local TLP2D = {}

--#region Initialization

--! Please carefully confirm whether the following functions are supported by lua environment.
--! If some of them cannot be used, it may cause load error or runtime errors.
local error = error
local ipairs = ipairs
local math_abs = math.abs
local math_cos = math.cos
local math_exp = math.exp
local math_floor = math.floor
local math_huge = math.huge
local math_max = math.max
local math_min = math.min
local math_pi = math.pi
local math_sin = math.sin
local math_sqrt = math.sqrt
local pcall = pcall
local setmetatable = setmetatable
local tonumber = tonumber
local table_concat = table.concat
local tostring = tostring
local type = type

local ENV_LUA_51 = "Lua 5.1"
local ENV_LUA_52 = "Lua 5.2"
local ENV_LUA_53 = "Lua 5.3"
local ENV_LUA_54 = "Lua 5.4"
local ENV_LUA_JIT = "LuaJIT"
local ENV_LUA_JIT_WOS = "LuaJIT-WOS" -- Lua environment of WOS game engine (Crypt of the Necrodancer: Synchrony)

--! Setup your environment first
local env = select(1, { ... }) or _VERSION
if not env then
	error("please modify the source code of 'TinyLuaPhysics2D':\
	locate the line where this error occurred, find the `env` variable above,\
	then manually setup your lua environment.")
end

--- @class (exact) TLP2D.Vector
--- @field [1] number
--- @field [2] number

--- @class (exact) TLP2D.Rectangle
--- @field [1] number
--- @field [2] number
--- @field [3] number
--- @field [4] number

local epsilon = 1e-9

--* The "Angular Mass" in this engine is equivalent to "Rotational Inertia"

--#endregion

--#region Vector

--- @readonly
TLP2D.Vector = {}

--- @param x1 number
--- @param y1 number
--- @param x2 number
--- @param y2 number
--- @return number
--- @nodiscard
function TLP2D.Vector.cross(x1, y1, x2, y2)
	return x1 * y2 - y1 * x2
end

local TLP2D_Vector_cross = TLP2D.Vector.cross

--- @param x1 number
--- @param y1 number
--- @param x2 number
--- @param y2 number
--- @return number
--- @nodiscard
function TLP2D.Vector.distance(x1, y1, x2, y2)
	local dx = x2 - x1
	local dy = y2 - y1
	return math_sqrt(dx * dx + dy * dy)
end

local TLP2D_Vector_distance = TLP2D.Vector.distance

--- @param x1 number
--- @param y1 number
--- @param x2 number
--- @param y2 number
--- @return number
--- @nodiscard
function TLP2D.Vector.dot(x1, y1, x2, y2)
	return x1 * x2 + y1 * y2
end

local TLP2D_Vector_dot = TLP2D.Vector.dot

--- @param x number
--- @param y number
--- @return number
--- @nodiscard
function TLP2D.Vector.magnitude(x, y)
	return math_sqrt(x * x + y * y)
end

local TLP2D_Vector_magnitude = TLP2D.Vector.magnitude

--- @type TLP2D.Vector
TLP2D.Vector.normalizeFallback = { 1, 0 }

local TLP2D_Vector_normalizeFallbackVector = TLP2D.Vector.normalizeFallback

--- @param x number
--- @param y number
--- @return number nx
--- @return number ny
--- @nodiscard
function TLP2D.Vector.normalize(x, y)
	if x ~= 0 or y ~= 0 then
		local len = math_sqrt(x * x + y * y)
		return x / len, y / len
	else -- Resolve edge case
		return TLP2D_Vector_normalizeFallbackVector[1], TLP2D_Vector_normalizeFallbackVector[2]
	end
end

local TLP2D_Vector_normalize = TLP2D.Vector.normalize

--- @param x1 number
--- @param y1 number
--- @param x2 number
--- @param y2 number
--- @return number sqDist
--- @nodiscard
function TLP2D.Vector.squareDistance(x1, y1, x2, y2)
	local dx = x2 - x1
	local dy = y2 - y1
	return dx * dx + dy * dy
end

local TLP2D_Vector_squareDistance = TLP2D.Vector.squareDistance

--- @param x number
--- @param y number
--- @return number
--- @nodiscard
function TLP2D.Vector.squareMagnitude(x, y)
	return x * x + y * y
end

local TLP2D_Vector_squareMagnitude = TLP2D.Vector.squareMagnitude

--- @param x number
--- @param y number
--- @param x1 number
--- @param y1 number
--- @param x2 number
--- @param y2 number
--- @return number pointX
--- @return number pointY
--- @return number sqDist @square distance from origin(x, y) to contacting point.
--- @nodiscard
function TLP2D.Vector.findClosestPointToSegment(x, y, x1, y1, x2, y2)
	local segX = x2 - x1
	local segY = y2 - y1

	local proj = TLP2D_Vector_dot(x - x1, y - y1, segX, segY)
	local segLenSq = TLP2D_Vector_squareMagnitude(segX, segY)

	local t = proj / segLenSq
	if t <= 0 then
		return x1, y1, TLP2D_Vector_squareDistance(x, y, x1, y1)
	elseif t >= 1 then
		return x2, y2, TLP2D_Vector_squareDistance(x, y, x2, y2)
	else
		local px = x1 + segX * t
		local py = y1 + segY * t
		return px, py, TLP2D_Vector_squareDistance(x, y, px, py)
	end
end

local TLP2D_Vector_findClosestPointToSegment = TLP2D.Vector.findClosestPointToSegment

--#endregion

--#region Utils

--- @readonly
TLP2D.Utils = {}

--- @param min number
--- @param value number
--- @param max number
--- @return number
--- @nodiscard
function TLP2D.Utils.clamp(min, value, max)
	if value < min then
		return min
	elseif value > max then
		return max
	else
		return value
	end
end

--- @generic T
--- @param list T[]
--- @param depth integer
--- @return T[]
--- @nodiscard
local function utilsCopyArray(list, depth, seenSet)
	local result = {}

	if depth > 0 then
		for index, value in ipairs(list) do
			if type(value) == "table" and not seenSet[value] then
				seenSet[value] = value
				result[index] = utilsCopyArray(value, depth - 1)
			end
		end
	else
		for index, value in ipairs(list) do
			result[index] = value
		end
	end

	return result
end

--- @generic T
--- @param list T[]
--- @param depth integer? @default: 0
--- @return T[]
--- @nodiscard
function TLP2D.Utils.copyArray(list, depth)
	return utilsCopyArray(list, tonumber(depth) or 0)
end

function TLP2D.Utils.emptyFunction() end

local TLP2D_Utils_emptyFunction = TLP2D.Utils.emptyFunction

TLP2D.Utils.emptyTable = setmetatable({}, { __newindex = TLP2D_Utils_emptyFunction })

local TLP2D_Utils_emptyTable = TLP2D.Utils.emptyTable

--- @generic T
--- @param list T[]
--- @param value T
--- @return integer?
function TLP2D.Utils.listFastRemoveFirst(list, value)
	for i = 1, #list do
		if list[i] == value then
			list[i] = list[#list]
			list[#list] = nil

			return i
		end
	end
end

local TLP2D_Utils_listFastRemoveFirst = TLP2D.Utils.listFastRemoveFirst

local serializeNumberArrayList = {}

--- @param array table
--- @param index integer
--- @param level integer
--- @return integer
local function serializeNumberArrayImpl(array, index, seenSet, level)
	serializeNumberArrayList[index] = "{"
	index = index + 1

	for i = 1, #array do
		local value = array[i]
		local type_ = type(value)

		if type_ == "number" then
			serializeNumberArrayList[index] = tostring(value)
			index = index + 1
		elseif type_ == "table" then
			if seenSet[value] then
				error("failed to serialize: cyclic reference detected " .. tostring(value), level)
			end

			seenSet[value] = true
			index = serializeNumberArrayImpl(value, index, seenSet, level + 1)
		else
			error("failed to serialize: unsupported type detected " .. type_, level)
		end

		if i < #array then
			serializeNumberArrayList[index] = ","
			index = index + 1
		end
	end

	serializeNumberArrayList[index] = "}"
	index = index + 1
	return index
end

--- @param array table
--- @return string
--- @nodiscard
function TLP2D.Utils.serializeNumberArray(array)
	serializeNumberArrayImpl(array, 1, {}, 3)
	return table_concat(serializeNumberArrayList, ",")
end

local TLP2D_Utils_serializeNumberArray = TLP2D.Utils.serializeNumberArray

function TLP2D.Utils.deserializeNumberArray(arrayBuffer)
	--
end

local TLP2D_Utils_deserializeNumberArray = TLP2D.Utils.deserializeNumberArray

--#endregion

--#region DynAABBTree

--- @readonly
TLP2D.DynAABBTree = {}

function TLP2D.DynAABBTree.new() end

--#endregion

--#region SpatialHashMap

--- TODO NOT DONE YET
--- @class TLP2D.SpatialHashMap
TLP2D.SpatialHashMap = {}

local spatialHashMapMetatable = { __index = TLP2D.SpatialHashMap }

--- @param gridSize integer
--- @return TLP2D.SpatialHashMap
function TLP2D.SpatialHashMap.new(gridSize)
	--- @class (exact) TLP2D.SpatialHashMapKey : integer

	--- @class (exact) TLP2D.SpatialHashMapGrid
	--- @field list TLP2D.SpatialHashMapElement[]
	--- @field set table<TLP2D.SpatialHashMapElementID, integer>

	--- @class TLP2D.SpatialHashMap
	local spatialHashMap = {}

	--- @package
	spatialHashMap._gridSize = gridSize

	--- @package
	spatialHashMap._invGridSize = 1 / gridSize

	--- @package
	--- @class TLP2D.SpatialHashMapElementID : integer
	spatialHashMap._elementLatestID = 0

	--- @package
	--- @type table<TLP2D.SpatialHashMapElementID, TLP2D.SpatialHashMapElement>
	spatialHashMap._elements = {}

	--- @package
	--- @type table<TLP2D.SpatialHashMapKey, TLP2D.SpatialHashMapGrid>
	spatialHashMap._gridMap = {}

	setmetatable(spatialHashMap, spatialHashMapMetatable)

	return spatialHashMap
end

function TLP2D.SpatialHashMap:clearElements()
	self._elementLatestID = 0
	self._elements = {}
	self._gridMap = {}
end

--- @param x number
--- @param y number
--- @return integer gridX
--- @return integer gridY
--- @nodiscard
function TLP2D.SpatialHashMap:reducePosition(x, y)
	local invSize = self._invGridSize
	return math_floor(x * invSize), math_floor(y * invSize)
end

--- @param gridX integer
--- @param gridY integer
--- @return number x
--- @return number y
--- @nodiscard
function TLP2D.SpatialHashMap:unwrapPosition(gridX, gridY)
	local size = self._gridSize
	return gridX * size, gridY * size
end

--- @param gridX number
--- @param gridY number
--- @return TLP2D.SpatialHashMapKey hashKey
--- @nodiscard
function TLP2D.SpatialHashMap:hashGrid(gridX, gridY)
	return gridX * 73856093 + gridY * 19349663
end

--- @param posX any
--- @param posY any
--- @return TLP2D.SpatialHashMapKey hashKey
function TLP2D.SpatialHashMap:hashPosition(posX, posY)
	local invSize = self._invGridSize
	local gridX = math_floor(posX * invSize)
	local gridY = math_floor(posY * invSize)
	return gridX * 73856093 + gridY * 19349663
end

function TLP2D.SpatialHashMap:getGridSize()
	return self._gridSize
end

--- @param self TLP2D.SpatialHashMap
--- @param elementID TLP2D.SpatialHashMapElementID
--- @param element TLP2D.SpatialHashMapElement
--- @param minX number
--- @param minY number
--- @param maxX number
--- @param maxY number
local function spatialHashMapEmplaceElement(self, elementID, element, minX, minY, maxX, maxY)
	local hashKeyList = element.hashKeyList
	local hashKeySet = element.hashKeySet

	do
		local invGridSize = self._invGridSize
		local minGridX = math_floor(minX * invGridSize)
		local minGridY = math_floor(minY * invGridSize)
		local maxGridX = math_floor(maxX * invGridSize)
		local maxGridY = math_floor(maxY * invGridSize)

		for gy = minGridY, maxGridY do
			for gx = minGridX, maxGridX do
				local hashKey = self:hashGrid(gx, gy)
				if not hashKeySet[hashKey] then
					local index = #hashKeyList + 1
					hashKeyList[index] = hashKey
					hashKeySet[hashKey] = index
				end
			end
		end
	end

	do
		local gridMap = self._gridMap

		for _, hashKey in ipairs(element.hashKeyList) do
			gridMap[hashKey] = gridMap[hashKey] or {
				list = {},
				set = {},
			}
			local grid = gridMap[hashKey]
			local index = #grid.list + 1
			grid.list[index] = element
			grid.set[elementID] = index
		end
	end
end

--- @return TLP2D.SpatialHashMapElementID
--- @nodiscard
function TLP2D.SpatialHashMap:addElement(customData)
	local elementID = self._elementLatestID + 1
	self._elementLatestID = elementID

	--- @class (exact) TLP2D.SpatialHashMapElement
	--- @field elementID TLP2D.SpatialHashMapElementID
	--- @field customData any
	--- @field hashKeyList TLP2D.SpatialHashMapKey[]
	--- @field hashKeySet table<TLP2D.SpatialHashMapKey, integer>
	local element = {
		elementID = elementID,
		customData = customData,
		hashKeyList = {},
		hashKeySet = {},
	}

	self._elements[elementID] = element

	return elementID
end

--- @param minX number
--- @param minY number
--- @param maxX number
--- @param maxY number
--- @return TLP2D.SpatialHashMapElementID
--- @nodiscard
function TLP2D.SpatialHashMap:insertElement(minX, minY, maxX, maxY, customData)
	local elementID = self:addElement(customData)

	spatialHashMapEmplaceElement(self, elementID, self._elements[elementID], minX, minY, maxX, maxY)

	return elementID
end

--- @param elementID TLP2D.SpatialHashMapElementID
--- @return boolean
--- @nodiscard
function TLP2D.SpatialHashMap:containsElement(elementID)
	return not not self._elements[elementID]
end

--- @param self TLP2D.SpatialHashMap
--- @param elementID TLP2D.SpatialHashMapElementID
--- @param element TLP2D.SpatialHashMapElement
local function spatialHashMapEraseElement(self, elementID, element)
	local gridMap = self._gridMap
	local hashKeyList = element.hashKeyList
	local hashKeySet = element.hashKeySet

	for i = #hashKeyList, 1, -1 do
		local grid = gridMap[hashKeyList[i]]
		local gridList = grid.list
		local gridSet = grid.set
		local removeIndex = gridSet[elementID]
		if removeIndex then
			local lastIndex = #gridList
			if removeIndex ~= lastIndex then
				local movedElement = gridList[lastIndex]
				gridList[removeIndex] = movedElement
				gridSet[movedElement.elementID] = removeIndex
			end
			gridList[lastIndex] = nil
			gridSet[elementID] = nil
		end
	end
end

--- @param elementID TLP2D.SpatialHashMapElementID
--- @param minX number
--- @param minY number
--- @param maxX number
--- @param maxY number
--- @return boolean
function TLP2D.SpatialHashMap:updateElement(elementID, minX, minY, maxX, maxY)
	local element = self._elements[elementID]
	if element then
		spatialHashMapEraseElement(self, elementID, element)
		spatialHashMapEmplaceElement(self, elementID, element, minX, minY, maxX, maxY)

		return true
	else
		return false
	end
end

--- @param elementID TLP2D.SpatialHashMapElementID
--- @return boolean
function TLP2D.SpatialHashMap:removeElement(elementID)
	local element = self._elements[elementID]
	if element then
		spatialHashMapEraseElement(self, elementID, element)
		self._elements[elementID] = nil

		return true
	else
		return false
	end
end

--- @param gridX integer
--- @param gridY integer
--- @return fun(table: TLP2D.SpatialHashMapElement[], i?: integer): (integer, TLP2D.SpatialHashMapElement), TLP2D.SpatialHashMapElement[], integer
--- @nodiscard
function TLP2D.SpatialHashMap:iterateGrid(gridX, gridY)
	local hashKey = self:hashGrid(gridX, gridY)
	local grid = self._gridMap[hashKey]
	if grid then
		return ipairs(grid.list)
	else
		return ipairs(TLP2D_Utils_emptyTable)
	end
end

--- @param posX integer
--- @param posY integer
--- @return fun(table: TLP2D.SpatialHashMapElement[], i?: integer): (integer, TLP2D.SpatialHashMapElement), TLP2D.SpatialHashMapElement[], integer
--- @nodiscard
function TLP2D.SpatialHashMap:iteratePosition(posX, posY)
	local hashKey = self:hashPosition(posX, posY)
	local grid = self._gridMap[hashKey]
	if grid then
		return ipairs(grid.list)
	else
		return ipairs(TLP2D_Utils_emptyTable)
	end
end

function TLP2D.SpatialHashMap:setGridSize()
	-- TODO
end

function TLP2D.SpatialHashMap:clearCaches()
	local gridMap = self._gridMap
	for hashKey, grid in pairs(gridMap) do
		if not grid.list[1] then
			gridMap[hashKey] = nil
		end
	end
end

--#endregion

--#region Collision

--- @readonly
TLP2D.Collision = {}

--- @param aabb1 TLP2D.Rectangle
--- @param aabb2 TLP2D.Rectangle
--- @return boolean hasCollision
function TLP2D.Collision.checkAABB(aabb1, aabb2)
	return not (aabb1[3] <= aabb2[1] or aabb2[3] <= aabb1[1] or aabb1[4] <= aabb2[2] or aabb2[4] <= aabb1[2])
end

local TLP2D_Collision_checkAABB = TLP2D.Collision.checkAABB

--- @param x1 number
--- @param y1 number
--- @param r1 number
--- @param x2 number
--- @param y2 number
--- @param r2 number
--- @return boolean
function TLP2D.Collision.checkCircleWithCircle(x1, y1, r1, x2, y2, r2)
	local distance = TLP2D_Vector_distance(x1, y1, x2, y2)
	local radii = r1 + r2
	return distance < radii
end

--- @param x1 number
--- @param y1 number
--- @param r1 number
--- @param x2 number
--- @param y2 number
--- @param r2 number
--- @return number? depth
--- @return number normalX
--- @return number normalY
function TLP2D.Collision.intersectCircleWithCircle(x1, y1, r1, x2, y2, r2)
	local distance = TLP2D_Vector_distance(x1, y1, x2, y2)
	local radii = r1 + r2
	if radii > distance then
		return radii - distance, TLP2D_Vector_normalize(x2 - x1, y2 - y1)
	else
		return nil, 0, 0
	end
end

local TLP2D_Collision_intersectCircleWithCircle = TLP2D.Collision.intersectCircleWithCircle

--- @param axisX number
--- @param axisY number
--- @param vertices TLP2D.Vertex[]
--- @return number min
--- @return number max
local function collisionProjectPolygonToAxis(axisX, axisY, vertices)
	local min = math_huge
	local max = -math_huge

	for i = 1, #vertices do
		local vertex = vertices[i]
		local proj = TLP2D_Vector_dot(vertex[1], vertex[2], axisX, axisY)
		if proj < min then
			min = proj
		end
		if proj > max then
			max = proj
		end
	end

	return min, max
end

--- @generic T1, T2, T3
--- @param intersectSATHandle function
--- @param vertices TLP2D.Vertex[]
--- @param projectShapeToAxis fun(axisX: number, axisY: number, param1: T1, param2: T2, param3: T3)
--- @parma param1 T1
--- @parma param2 T2
--- @return boolean
local function collisionStepPolygonSAT(intersectSATHandle, vertices, projectShapeToAxis, param1, param2, param3)
	local verticesCount = #vertices
	for i = 1, verticesCount do
		local vertex1 = vertices[i]
		local vertex2 = vertices[i == verticesCount and 1 or (i + 1)]
		local axisX, axisY = TLP2D_Vector_normalize(vertex1[2] - vertex2[2], vertex2[1] - vertex1[1])

		local min1, max1 = collisionProjectPolygonToAxis(axisX, axisY, vertices)
		local min2, max2 = projectShapeToAxis(axisX, axisY, param1, param2, param3)
		if min1 > max2 or min2 > max1 then
			return false
		end

		intersectSATHandle(axisX, axisY, min1, max1, min2, max2)
	end

	return true
end

--- @param vertices1 TLP2D.Vertex[]
--- @param vertices2 TLP2D.Vertex[]
--- @return boolean
function TLP2D.Collision.checkPolygonWithPolygon(vertices1, vertices2)
	return collisionStepPolygonSAT(TLP2D_Utils_emptyFunction, vertices1, collisionProjectPolygonToAxis, vertices2)
		and collisionStepPolygonSAT(TLP2D_Utils_emptyFunction, vertices2, collisionProjectPolygonToAxis, vertices1)
end

local intersectionPolygonSATDepth = 0
local intersectionPolygonSATNormalX = 0
local intersectionPolygonSATNormalY = 0

--- @param axisX number
--- @param axisY number
--- @param min1 number
--- @param max1 number
--- @param min2 number
--- @param max2 number
local function collisionStepPolygonSATIntersectResultHandle(axisX, axisY, min1, max1, min2, max2)
	local depth = math_min(max2 - min1, max1 - min2)
	if depth < intersectionPolygonSATDepth then
		intersectionPolygonSATDepth = depth
		intersectionPolygonSATNormalX = axisX
		intersectionPolygonSATNormalY = axisY
	end
end

--- @param vertices1 TLP2D.Vertex[]
--- @param vertices2 TLP2D.Vertex[]
--- @return number? depth
--- @return number normalX
--- @return number normalY
function TLP2D.Collision.intersectPolygonWithPolygon(x1, y1, vertices1, x2, y2, vertices2)
	intersectionPolygonSATDepth = math_huge
	if
		collisionStepPolygonSAT(
			collisionStepPolygonSATIntersectResultHandle,
			vertices1,
			collisionProjectPolygonToAxis,
			vertices2
		)
		and collisionStepPolygonSAT(
			collisionStepPolygonSATIntersectResultHandle,
			vertices2,
			collisionProjectPolygonToAxis,
			vertices1
		)
	then
		if TLP2D_Vector_dot(x2 - x1, y2 - y1, intersectionPolygonSATNormalX, intersectionPolygonSATNormalY) < 0 then
			intersectionPolygonSATNormalX = -intersectionPolygonSATNormalX
			intersectionPolygonSATNormalY = -intersectionPolygonSATNormalY
		end

		return intersectionPolygonSATDepth, intersectionPolygonSATNormalX, intersectionPolygonSATNormalY
	else
		return nil, 0, 0
	end
end

local TLP2D_Collision_intersectPolygonWithPolygon = TLP2D.Collision.intersectPolygonWithPolygon

--- @param axisX number
--- @param axisY number
--- @param x number
--- @param y number
--- @param r number
--- @return number min
--- @return number max
local function collisionProjectCircleToAxis(axisX, axisY, x, y, r)
	local proj1 = TLP2D_Vector_dot(x + axisX * r, y + axisY * r, axisX, axisY)
	local proj2 = TLP2D_Vector_dot(x - axisX * r, y - axisY * r, axisX, axisY)
	if proj1 < proj2 then
		return proj1, proj2
	else
		return proj2, proj1
	end
end

--- @param vertices TLP2D.Vertex[]
--- @param x number
--- @param y number
--- @return number
--- @return number
local function collisionFindClosestVertexToPointOnPolygonAxis(vertices, x, y)
	local vertexIndex
	local minDistance = math_huge

	for i = 1, #vertices do
		local vertex = vertices[i]
		local distance = TLP2D_Vector_distance(vertex[1], vertex[2], x, y)
		if distance < minDistance then
			minDistance = distance
			vertexIndex = i
		end
	end

	local vertex = vertices[vertexIndex]
	return TLP2D_Vector_normalize(vertex[1] - x, vertex[2] - y)
end

--- @param x1 number
--- @param y1 number
--- @param vertices1 TLP2D.Vertex[]
--- @param x2 number
--- @param y2 number
--- @param r2 number
--- @return number? depth
--- @return number normalX
--- @return number normalY
function TLP2D.Collision.intersectPolygonWithCircle(x1, y1, vertices1, x2, y2, r2)
	intersectionPolygonSATDepth = math_huge
	if
		collisionStepPolygonSAT(
			collisionStepPolygonSATIntersectResultHandle,
			vertices1,
			collisionProjectCircleToAxis,
			x2,
			y2,
			r2
		)
	then
		local axisX, axisY = collisionFindClosestVertexToPointOnPolygonAxis(vertices1, x2, y2)
		local min1, max1 = collisionProjectPolygonToAxis(axisX, axisY, vertices1)
		local min2, max2 = collisionProjectCircleToAxis(axisX, axisY, x2, y2, r2)
		if min1 > max2 or min2 > max1 then
			return nil, 0, 0
		end

		local depth = math_min(max2 - min1, max1 - min2)
		if depth < intersectionPolygonSATDepth then
			intersectionPolygonSATDepth = depth
			intersectionPolygonSATNormalX = axisX
			intersectionPolygonSATNormalY = axisY
		end

		if TLP2D_Vector_dot(x1 - x2, y1 - y2, intersectionPolygonSATNormalX, intersectionPolygonSATNormalY) < 0 then
			intersectionPolygonSATNormalX = -intersectionPolygonSATNormalX
			intersectionPolygonSATNormalY = -intersectionPolygonSATNormalY
		end

		return intersectionPolygonSATDepth, intersectionPolygonSATNormalX, intersectionPolygonSATNormalY
	else
		return nil, 0, 0
	end
end

local Collision_intersectPolygonWithCircle = TLP2D.Collision.intersectPolygonWithCircle

--- @class (exact) TLP2D.ContactResult
--- @field count integer | 0 | 1 | 2
--- @field list TLP2D.Vector[]
local contactResult = {
	count = 0,
	list = { { 0, 0 }, { 0, 0 } },
}

--- @param x1 number
--- @param y1 number
--- @param r1 number
--- @param x2 number
--- @param y2 number
--- @return TLP2D.ContactResult
--- @nodiscard
function TLP2D.Collision.contactCircleWithCircle(x1, y1, r1, x2, y2)
	local dirX, dirY = TLP2D_Vector_normalize(x2 - x1, y2 - y1)
	contactResult.count = 1
	local contactVector = contactResult.list[1]
	contactVector[1] = x1 + dirX * r1
	contactVector[2] = y1 + dirY * r1
	return contactResult
end

local TLP2D_Collision_contactCircleWithCircle = TLP2D.Collision.contactCircleWithCircle

--- @param vertices1 TLP2D.Vertex[]
--- @param vertices2 TLP2D.Vertex[]
--- @param minSqDist number
--- @return number
local function contactStepPolygonWithPolygon(vertices1, vertices2, minSqDist)
	local contactVector1 = contactResult.list[1]
	local contactVector2 = contactResult.list[2]

	for i1 = 1, #vertices1 do
		local pointX = vertices1[i1][1]
		local pointY = vertices1[i1][2]
		local vertex2Count = #vertices2

		for i2 = 1, #vertices2 do
			local vertex1 = vertices2[i2]
			local vertex2 = vertices2[i2 == vertex2Count and 1 or i2 + 1]
			local x, y, sqDist = TLP2D_Vector_findClosestPointToSegment( --
				pointX,
				pointY,
				vertex1[1],
				vertex1[2],
				vertex2[1],
				vertex2[2]
			)

			if math_abs(sqDist - minSqDist) < 1e-6 then
				if math_abs(x - contactVector1[1]) > epsilon or math_abs(y - contactVector1[2]) > epsilon then
					contactResult.count = 2
					contactVector2[1] = x
					contactVector2[2] = y
				end
			elseif sqDist < minSqDist then
				minSqDist = sqDist
				contactResult.count = 1
				contactVector1[1] = x
				contactVector1[2] = y
			end
		end
	end

	return minSqDist
end

--- @param vertices1 TLP2D.Vertex[]
--- @param vertices2 TLP2D.Vertex[]
--- @return TLP2D.ContactResult
--- @nodiscard
function TLP2D.Collision.contactPolygonWithPolygon(vertices1, vertices2)
	contactResult.count = 0
	local minSqDist = contactStepPolygonWithPolygon(vertices1, vertices2, math_huge)
	contactStepPolygonWithPolygon(vertices2, vertices1, minSqDist)
	return contactResult
end

local TLP2D_Collision_contactPolygonWithPolygon = TLP2D.Collision.contactPolygonWithPolygon

--- @param x1 number
--- @param y1 number
--- @param vertices1 TLP2D.Vertex[]
--- @param x2 number
--- @param y2 number
function TLP2D.Collision.contactPolygonWithCircle(x1, y1, vertices1, x2, y2)
	contactResult.count = 1
	local contactVector = contactResult.list[1]

	local minSqDist = math_huge

	local vertex1Count = #vertices1
	for i = 1, #vertices1 do
		local vertex1 = vertices1[i]
		local vertex2 = vertices1[i == vertex1Count and 1 or i + 1]
		local x, y, sqDist = TLP2D_Vector_findClosestPointToSegment( --
			x2,
			y2,
			vertex1[1],
			vertex1[2],
			vertex2[1],
			vertex2[2]
		)

		if sqDist < minSqDist then
			minSqDist = sqDist
			contactVector[1] = x
			contactVector[2] = y
		end
	end

	return contactResult
end

local TLP2D_Collision_contactPolygonWithCircle = TLP2D.Collision.contactPolygonWithCircle

--#endregion

--#region Material

TLP2D.Material = {}

--- @enum TLP2D.FrictionCombine
TLP2D.Material.FrictionCombine = {
	Average = 1,
	Minimum = 2,
	Maximum = 3,
	Multiply = 4,
}

local TLP2D_Material_FrictionCombine_Average = TLP2D.Material.FrictionCombine.Average
local TLP2D_Material_FrictionCombine_Minimum = TLP2D.Material.FrictionCombine.Minimum
local TLP2D_Material_FrictionCombine_Maximum = TLP2D.Material.FrictionCombine.Maximum
local TLP2D_Material_FrictionCombine_Multiply = TLP2D.Material.FrictionCombine.Multiply

--- @class TLP2D.MaterialName : string

--- @type TLP2D.Material[] | table<string, TLP2D.Material>
local materials
--- @type TLP2D.Material
local materialDefault

--- @param materialName string
--- @param density number
--- @param restitution number
--- @param staticFriction number
--- @param dynamicFriction number
--- @param frictionCombine TLP2D.FrictionCombine
--- @param linearDrag number
--- @param angularDrag number
function TLP2D.Material.register(
	materialName,
	density,
	restitution,
	staticFriction,
	dynamicFriction,
	frictionCombine,
	linearDrag,
	angularDrag
)
	if density <= 0 then
		error("bad argument to #2 'density: value must be greater than 0", 2)
	elseif restitution < 0 or restitution > 1 then
		error("bad argument to #3 'restitution: value must be between 0 to 1", 2)
	elseif staticFriction < 0 or staticFriction > 1 then
		error("bad argument to #4 'staticFriction: value must be between 0 to 1", 2)
	elseif dynamicFriction < 0 or dynamicFriction > 1 then
		error("bad argument to #5 'dynamicFriction: value must be between 0 to 1", 2)
	end
	materialName = tostring(materialName)

	--- @class TLP2D.MaterialID : integer
	local materialID = #materials + 1
	--- @class (exact) TLP2D.Material
	--- @field id TLP2D.MaterialID
	--- @field name TLP2D.MaterialName
	--- @field density number
	--- @field restitution number
	--- @field staticFriction number
	--- @field dynamicFriction number
	--- @field frictionCombine TLP2D.FrictionCombine
	--- @field linearDrag number
	--- @field angularDrag number
	local materialData = {
		id = materialID,
		name = materialName,
		density = density,
		restitution = restitution,
		staticFriction = staticFriction,
		dynamicFriction = dynamicFriction,
		frictionCombine = frictionCombine,
		linearDrag = linearDrag,
		angularDrag = angularDrag,
	}
	materials[materialID] = materialData
	materials[materialName] = materialData
	materials[materialData] = materialData
end

--- @param materialIDOrName TLP2D.MaterialID | string
--- @return boolean
function TLP2D.Material.contains(materialIDOrName)
	return not not materials[materialIDOrName]
end

local TLP2D_Material_contains = TLP2D.Material.contains

--- @return TLP2D.Material
--- @nodiscard
function TLP2D.Material.getDefaultMaterial()
	return materialDefault
end

--- @param material TLP2D.MaterialID | TLP2D.MaterialName
local function materialNotFound(material)
	error(("bad argument to #1 'material': material %s does not exist"):format(material), 3)
end

--- @param materialIDOrName TLP2D.MaterialID | TLP2D.MaterialName
function TLP2D.Material.setDefaultMaterial(materialIDOrName)
	if materials[materialIDOrName] then
		materialDefault = materials[materialIDOrName]
	end
end

--- @param material string | TLP2D.MaterialID
--- @return TLP2D.MaterialID
--- @nodiscard
function TLP2D.Material.getID(material)
	local material_ = materials[material]
	if not material_ then
		materialNotFound(material_)
	end
	return material_.id
end

--- @param material TLP2D.MaterialID | string
--- @return string
--- @nodiscard
function TLP2D.Material.getName(material)
	local material_ = materials[material]
	if not material_ then
		materialNotFound(material_)
	end
	return material_.name
end

--- @param material TLP2D.MaterialID | string
--- @return number density
--- @nodiscard
function TLP2D.Material.getDensity(material)
	local material_ = materials[material]
	if not material_ then
		materialNotFound(material_)
	end
	return material_.density
end

local TLP2D_Material_getDensity = TLP2D.Material.getDensity

--- @param material TLP2D.MaterialID | string
--- @return number restitution
--- @nodiscard
function TLP2D.Material.getRestitution(material)
	local material = materials[material]
	if not material then
		materialNotFound(material)
	end
	return material.restitution
end

--- @param material TLP2D.MaterialID | string
--- @return number staticFriction
--- @nodiscard
function TLP2D.Material.getStaticFriction(material)
	local material = materials[material]
	if not material then
		materialNotFound(material)
	end
	return material.staticFriction
end

--- @param material TLP2D.MaterialID | string
--- @return number dynamicFriction
--- @nodiscard
function TLP2D.Material.getDynamicFriction(material)
	local material = materials[material]
	if not material then
		materialNotFound(material)
	end
	return material.dynamicFriction
end

--- @return fun(material: TLP2D.Material[], i?: integer): (i: integer, material: TLP2D.Material), TLP2D.Material[], integer
function TLP2D.Material.iterate()
	return ipairs(materials)
end

function TLP2D.Material.reset()
	materials = {}

	for _, params in ipairs({
		{ "Glass", 2.5, 0.1, 0.3, 0.25, TLP2D_Material_FrictionCombine_Average, 0.06, 0.02 },
		{ "Ice", 0.9, 0.05, 0.05, 0.01, TLP2D_Material_FrictionCombine_Average, 0.03, 0.01 },
		{ "Lead", 11.3, 0.05, 0.3, 0.28, TLP2D_Material_FrictionCombine_Average, 0.03, 0.05 },
		{ "Plastic", 1.2, 0.4, 0.4, 0.35, TLP2D_Material_FrictionCombine_Average, 0.12, 0.06 },
		{ "Rubber", 1.1, 0.8, 0.9, 0.75, TLP2D_Material_FrictionCombine_Average, 0.3, 0.2 },
		{ "Steel", 7.8, 0.03, 0.35, 0.30, TLP2D_Material_FrictionCombine_Average, 0.06, 0.03 },
		{ "Stone", 2.4, 0.1, 0.45, 0.40, TLP2D_Material_FrictionCombine_Average, 0.15, 0.04 },
		{ "Wood", 0.6, 0.3, 0.5, 0.45, TLP2D_Material_FrictionCombine_Average, 0.25, 0.08 },
	}) do
		local name = params[1]
		local density = params[2]
		local restitution = params[3]
		local staticFriction = params[4]
		local dynamicFriction = params[5]
		local frictionCombine = params[6]
		local linearDrag = params[7]
		local angularDrag = params[8]
		TLP2D.Material.register(
			name,
			density,
			restitution,
			staticFriction,
			dynamicFriction,
			frictionCombine,
			linearDrag,
			angularDrag
		)
	end

	materialDefault = materials["Wood"]
end

TLP2D.Material.reset()

--#endregion

--#region Shape

--- @readonly
TLP2D.Shape = {}

--- @readonly
--- @enum TLP2D.BodyShape
TLP2D.Shape.Type = {
	None = 0,
	Circle = 1,
	Rectangle = 2,
	Polygon = 3,
	-- PredefPolygon = 4,
}

local TLP2D_Shape_Type_None = TLP2D.Shape.Type.None
local TLP2D_Shape_Type_Circle = TLP2D.Shape.Type.Circle
local TLP2D_Shape_Type_Rectangle = TLP2D.Shape.Type.Rectangle
local TLP2D_Shape_Type_Polygon = TLP2D.Shape.Type.Polygon
-- local TLP2D_Shape_Type_PredefPolygon = TLP2D.Shape.Type.PredefPolygon

--- @type table<string, TLP2D.PredefinedPolygon> | TLP2D.PredefinedPolygon[]
local shapePredefinedPolygons = {}
--- @type integer
local shapePredefinedPolygonID = 0

function TLP2D.Shape.reset()
	shapePredefinedPolygons = {}
	shapePredefinedPolygonID = 0
end

--- @param name TLP2D.PredefinedPolygonName
--- @param vertices TLP2D.Vertex[]
--- @return TLP2D.PredefinedPolygonID id
function TLP2D.Shape.predefinePolygon(name, vertices)
	--- @class TLP2D.PredefinedPolygonID : integer
	local id = shapePredefinedPolygonID + 1
	shapePredefinedPolygonID = id

	--- @class TLP2D.PredefinedPolygonName : string

	--- @class (exact) TLP2D.PredefinedPolygon
	--- @field id TLP2D.PredefinedPolygonID
	--- @field name TLP2D.PredefinedPolygonName
	--- @field vertices TLP2D.Vertex[]
	local predefinedPolygon = {
		id = id,
		name = name,
		vertices = TLP2D.Utils.copyArray(vertices),
	}
	shapePredefinedPolygons[id] = predefinedPolygon
	shapePredefinedPolygons[name] = predefinedPolygon

	return id
end

--- @param predefPolygon integer | string
--- @return TLP2D.Vertex[]
--- @nodiscard
function TLP2D.Shape.getPredefinedPolygonVertices(predefPolygon)
	local data = shapePredefinedPolygons[predefPolygon]
	if data then
		return data.vertices
	end
	error(("bad argument to #1 'predefPolygon': predefined polygon %s does not exist"):format(predefPolygon), 2)
end

--#endregion

--#region Layer

--#endregion

--#region Body

--- @readonly
TLP2D.Body = {}

local BODY_DATA_ID = 1
local BODY_DATA_TYPE = 2
local BODY_DATA_POSITION_X = 3
local BODY_DATA_POSITION_Y = 4
local BODY_DATA_LINEAR_VELOCITY_X = 5
local BODY_DATA_LINEAR_VELOCITY_Y = 6
local BODY_DATA_ROTATION = 7
local BODY_DATA_ANGULAR_VELOCITY = 8
local BODY_DATA_MATERIAL = 9
local BODY_DATA_SHAPE = 10
local BODY_DATA_SHAPE_DATA = 11

--- @enum TLP2D.BodyType
--- @readonly
TLP2D.Body.Type = {
	Static = 1,
	Kinematic = 2,
	Dynamic = 3,
}
local TLP2D_Body_Type_Static = TLP2D.Body.Type.Static
local TLP2D_Body_Type_Kinematic = TLP2D.Body.Type.Kinematic
local TLP2D_Body_Type_Dynamic = TLP2D.Body.Type.Dynamic

TLP2D.Body.DefaultID = 0
TLP2D.Body.DefaultType = TLP2D_Body_Type_Dynamic
TLP2D.Body.DefaultPositionX = 0
TLP2D.Body.DefaultPositionY = 0
TLP2D.Body.DefaultLinearVelocityX = 0
TLP2D.Body.DefaultLinearVelocityY = 0
TLP2D.Body.DefaultRotation = 0
TLP2D.Body.DefaultAngularVelocity = 0
TLP2D.Body.DefaultMaterial = materialDefault.id
TLP2D.Body.DefaultShape = TLP2D_Shape_Type_None
TLP2D.Body.DefaultShapeData = 0

--- @alias TLP2D.BodyShapeDataNone 0

--- @alias TLP2D.BodyShapeDataCircle number

--- @alias TLP2D.BodyShapeDataSquare number

--- @alias TLP2D.BodyShapeDataRectangle TLP2D.Vector

--- @alias TLP2D.BodyShapeDataPolygon TLP2D.Vertex[]

--- @alias TLP2D.BodyShapeData TLP2D.BodyShapeDataNone | TLP2D.BodyShapeDataCircle | TLP2D.BodyShapeDataSquare | TLP2D.BodyShapeDataRectangle | TLP2D.BodyShapeDataPolygon

--- @param materialID TLP2D.MaterialID
--- @param bodyType TLP2D.BodyType
--- @param shape TLP2D.BodyShape
--- @param shapeData TLP2D.BodyShapeData
--- @return TLP2D.BodyData, TLP2D.BodyCache
--- @nodiscard
local function TLP2D_Body_create(bodyID, bodyType, materialID, shape, shapeData)
	--- @class TLP2D.BodyData
	local bodyData = {
		bodyID,
		bodyType,
		0,
		0,
		0,
		0,
		0,
		0,
		materialID,
		shape,
		shapeData,
	}

	--- @alias TLP2D.Vertex TLP2D.Vector

	--- @class (exact) TLP2D.BodyCache
	--- @field needUpdateTransform boolean @Fields: transformedVertices, transformedAABB
	--- @field transformedVertices TLP2D.Vertex[]
	--- @field transformedAABB TLP2D.Rectangle
	--- @field needUpdateMass boolean @Fields: mass, invMass; Chain: needUpdateAngularMass
	--- @field mass number
	--- @field invMass number
	--- @field needUpdateAngularMass boolean @Fields: angularMass, invAngularMass
	--- @field angularMass number
	--- @field invAngularMass number
	--- @field spatialHashMapElementID TLP2D.SpatialHashMapElementID?
	local bodyCache = {
		needUpdateTransform = true,
		transformedVertices = {},
		transformedAABB = { 0, 0, 0, 0 },
		needUpdateMass = true,
		mass = 0,
		invMass = 0,
		needUpdateAngularMass = true,
		angularMass = 0,
		invAngularMass = 0,
	}

	return bodyData, bodyCache
end

--- @param bodyData TLP2D.BodyData
--- @param bodyCache TLP2D.BodyCache
local function bodyUpdateCircleTransform(bodyData, bodyCache)
	bodyCache.transformedVertices[1] = nil
	local aabb = bodyCache.transformedAABB

	local positionX = bodyData[BODY_DATA_POSITION_X]
	local positionY = bodyData[BODY_DATA_POSITION_Y]
	local radius = bodyData[BODY_DATA_SHAPE_DATA]
	aabb[1] = positionX - radius
	aabb[2] = positionY - radius
	aabb[3] = positionX + radius
	aabb[4] = positionY + radius
end

local function bodyUpdatePolygonTransformedAABBCache(aabb, vertices)
	local minX = math_huge
	local minY = math_huge
	local maxX = -math_huge
	local maxY = -math_huge

	for i = 1, #vertices do
		local vertex = vertices[i]
		local x = vertex[1]
		local y = vertex[2]
		if x < minX then
			minX = x
		end
		if x > maxX then
			maxX = x
		end
		if y < minY then
			minY = y
		end
		if y > maxY then
			maxY = y
		end
	end

	aabb[1] = minX
	aabb[2] = minY
	aabb[3] = maxX
	aabb[4] = maxY
end

--- @param bodyData TLP2D.BodyData
--- @param bodyCache TLP2D.BodyCache
local function bodyUpdateRectangleTransform(bodyData, bodyCache)
	local vertices = bodyCache.transformedVertices
	local verticesCount = #vertices
	if verticesCount > 4 then
		for i = verticesCount, 5, -1 do
			vertices[i] = nil
		end
	elseif verticesCount < 4 then
		for i = math_max(1, verticesCount), 4 do
			vertices[i] = { 0, 0 }
		end
	end

	local positionX = bodyData[BODY_DATA_POSITION_X]
	local positionY = bodyData[BODY_DATA_POSITION_Y]
	local cos, sin
	do
		local rotation = bodyData[BODY_DATA_ROTATION]
		cos = math_cos(rotation)
		sin = math_sin(rotation)
	end
	local shapeData = bodyData[BODY_DATA_SHAPE_DATA]
	local halfWidth = shapeData[1] * 0.5
	local halfHeight = shapeData[2] * 0.5

	local vertex = vertices[1]
	vertex[1] = positionX + halfWidth * cos - halfHeight * sin
	vertex[2] = positionY + halfWidth * sin + halfHeight * cos
	vertex = vertices[2]
	vertex[1] = positionX + -halfWidth * cos - halfHeight * sin
	vertex[2] = positionY + -halfWidth * sin + halfHeight * cos
	vertex = vertices[3]
	vertex[1] = positionX + -halfWidth * cos - -halfHeight * sin
	vertex[2] = positionY + -halfWidth * sin + -halfHeight * cos
	vertex = vertices[4]
	vertex[1] = positionX + halfWidth * cos - -halfHeight * sin
	vertex[2] = positionY + halfWidth * sin + -halfHeight * cos

	bodyUpdatePolygonTransformedAABBCache(bodyCache.transformedAABB, vertices)
end

--- @param bodyData TLP2D.BodyData
--- @param bodyCache TLP2D.BodyCache
local function bodyUpdatePolygonTransform(bodyData, bodyCache)
	local vertices = bodyCache.transformedVertices
	local shapeData = bodyData[BODY_DATA_SHAPE_DATA]
	local count = #shapeData

	if #vertices > count then
		for i = #vertices, count + 1, -1 do
			vertices[i] = nil
		end
	elseif #vertices < count then
		for i = #vertices + 1, count do
			vertices[i] = { 0, 0 }
		end
	end

	local positionX = bodyData[BODY_DATA_POSITION_X]
	local positionY = bodyData[BODY_DATA_POSITION_Y]
	local cos, sin
	do
		local rotation = bodyData[BODY_DATA_ROTATION]
		cos = math_cos(rotation)
		sin = math_sin(rotation)
	end

	local vertex
	for i = 1, count do
		local localVertex = shapeData[i]
		local x = localVertex[1]
		local y = localVertex[2]

		vertex = vertices[i]
		vertex[1] = positionX + x * cos - y * sin
		vertex[2] = positionY + x * sin + y * cos
	end

	bodyUpdatePolygonTransformedAABBCache(bodyCache.transformedAABB, vertices)
end

--- @type table<TLP2D.BodyShape, fun(bodyData: TLP2D.BodyData, bodyCache: TLP2D.BodyCache)>
local bodyUpdateTransformFunctions = {
	bodyUpdateCircleTransform, -- Circle
	bodyUpdateRectangleTransform, -- Rectangle
	bodyUpdatePolygonTransform, -- Polygon
}

--- @param bodyData TLP2D.BodyData
--- @param bodyCache TLP2D.BodyCache
--- @return TLP2D.Vertex[]
--- @nodiscard
local function TLP2D_Body_getTransformedVertices(bodyData, bodyCache)
	if bodyCache.needUpdateTransform then
		bodyCache.needUpdateTransform = false
		local updateTransform = bodyUpdateTransformFunctions[bodyData[BODY_DATA_SHAPE]]
		if updateTransform then
			updateTransform(bodyData, bodyCache)
		end
	end
	return bodyCache.transformedVertices
end

--- @param bodyData TLP2D.BodyData
--- @param bodyCache TLP2D.BodyCache
--- @return TLP2D.Rectangle
--- @nodiscard
local function TLP2D_Body_getTransformedAABB(bodyData, bodyCache)
	if bodyCache.needUpdateTransform then
		bodyCache.needUpdateTransform = false
		local updateTransform = bodyUpdateTransformFunctions[bodyData[BODY_DATA_SHAPE]]
		if updateTransform then
			updateTransform(bodyData, bodyCache)
		end
	end
	return bodyCache.transformedAABB
end

--- @param bodyData TLP2D.BodyData
--- @param bodyCache TLP2D.BodyCache
local function bodyUpdateMassCircleShape(bodyData, bodyCache)
	local r = bodyData[BODY_DATA_SHAPE_DATA]
	local mass = math_pi * r * r * TLP2D_Material_getDensity(bodyData[BODY_DATA_MATERIAL])
	bodyCache.mass = mass
	bodyCache.invMass = 1 / mass
end

--- @param bodyData TLP2D.BodyData
--- @param bodyCache TLP2D.BodyCache
local function bodyUpdateMassRectangleShape(bodyData, bodyCache)
	local size = bodyData[BODY_DATA_SHAPE_DATA]
	local mass = size[1] * size[2] * TLP2D_Material_getDensity(bodyData[BODY_DATA_MATERIAL])
	bodyCache.mass = mass
	bodyCache.invMass = 1 / mass
end

--- @param bodyData TLP2D.BodyData
--- @param bodyCache TLP2D.BodyCache
local function bodyUpdateMassPolygonShape(bodyData, bodyCache)
	local vertices = bodyData[BODY_DATA_SHAPE_DATA]
	if #vertices < 3 then
		bodyCache.mass = 0
		bodyCache.invMass = 0

		return
	end

	local area = 0
	local count = #vertices
	local vertex
	for i = 1, count do
		vertex = vertices[i]
		local x1 = vertex[1]
		local y1 = vertex[2]
		vertex = vertices[i == count and 1 or (i + 1)]
		local x2 = vertex[1]
		local y2 = vertex[2]
		area = area + (x1 * y2 - y1 * x2)
	end

	local mass = area * 0.5 * TLP2D_Material_getDensity(bodyData[BODY_DATA_MATERIAL])
	bodyCache.mass = mass
	bodyCache.invMass = 1 / mass
end

local bodyUpdateMassFunctions = {
	bodyUpdateMassCircleShape, -- Circle
	bodyUpdateMassRectangleShape, -- Rectangle
	bodyUpdateMassPolygonShape, -- Polygon
}

--- @param bodyData TLP2D.BodyData
--- @param bodyCache TLP2D.BodyCache
local function bodyUpdateMass(bodyData, bodyCache)
	if bodyData[BODY_DATA_TYPE] == TLP2D_Body_Type_Static then
		bodyCache.mass = 0
		bodyCache.invMass = 0
	else
		local func = bodyUpdateMassFunctions[bodyData[BODY_DATA_SHAPE]]
		if func then
			func(bodyData, bodyCache)
		end
	end
end

--- @param bodyData TLP2D.BodyData
--- @param bodyCache TLP2D.BodyCache
--- @return number mass
--- @return number invMass
--- @nodiscard
local function TLP2D_Body_getMass(bodyData, bodyCache)
	if bodyCache.needUpdateMass then
		bodyCache.needUpdateMass = false
		bodyUpdateMass(bodyData, bodyCache)
	end
	return bodyCache.mass, bodyCache.invMass
end

--- @param bodyData TLP2D.BodyData
--- @param bodyCache TLP2D.BodyCache
local function bodyUpdateAngularMassCircleShape(bodyData, bodyCache)
	local mass = TLP2D_Body_getMass(bodyData, bodyCache)
	local radius = bodyData[BODY_DATA_SHAPE_DATA]
	local angularMass = 0.5 * mass * radius * radius
	bodyCache.angularMass = angularMass
	bodyCache.invAngularMass = 1 / angularMass
end

--- @param bodyData TLP2D.BodyData
--- @param bodyCache TLP2D.BodyCache
local function bodyUpdateAngularMassRectangleShape(bodyData, bodyCache)
	local shapeData = bodyData[BODY_DATA_SHAPE_DATA]
	local width = shapeData[1]
	local height = shapeData[2]
	local angularMass = (1 / 12) * TLP2D_Body_getMass(bodyData, bodyCache) * (width * width + height * height)
	bodyCache.angularMass = angularMass
	bodyCache.invAngularMass = 1 / angularMass
end

--- @param bodyData TLP2D.BodyData
--- @param bodyCache TLP2D.BodyCache
local function bodyUpdateAngularMassPolygonShape(bodyData, bodyCache)
	local vertices = bodyData[BODY_DATA_SHAPE_DATA]
	local count = #vertices
	if count < 3 then
		bodyCache.angularMass = 0
		bodyCache.invAngularMass = 0

		return
	end

	local area2 = 0
	local cx_num = 0
	local cy_num = 0
	local I_orig = 0
	for i = 1, count do
		local j = i == count and 1 or (i + 1)
		local xi = vertices[i][1]
		local yi = vertices[i][2]
		local xj = vertices[j][1]
		local yj = vertices[j][2]

		local cross = xi * yj - xj * yi
		area2 = area2 + cross

		cx_num = cx_num + (xi + xj) * cross
		cy_num = cy_num + (yi + yj) * cross

		local xi2 = xi * xi
		local xj2 = xj * xj
		local yi2 = yi * yi
		local yj2 = yj * yj

		I_orig = I_orig + cross * (xi2 + xi * xj + xj2 + yi2 + yi * yj + yj2)
	end

	local area = 0.5 * area2
	if area == 0 or math_abs(area) < epsilon then
		bodyCache.angularMass = 0
		bodyCache.invAngularMass = 0

		return
	end

	local mass = TLP2D_Body_getMass(bodyData, bodyCache)
	local density = TLP2D_Material_getDensity(bodyData[BODY_DATA_MATERIAL])

	-- unholy math
	local I_area_orig = I_orig / 12.0
	local I_about_origin = density * I_area_orig
	local cx = cx_num / (6 * area)
	local cy = cy_num / (6 * area)
	local d2 = cx * cx + cy * cy
	local I_centroid = I_about_origin - mass * d2
	if I_centroid < 0 and I_centroid > -epsilon then
		I_centroid = 0
	end
	if I_centroid < 0 then
		I_centroid = math_abs(I_centroid)
	end
	local invInertia = 0
	if I_centroid > 0 then
		invInertia = 1 / I_centroid
	end

	bodyCache.angularMass = I_centroid
	bodyCache.invAngularMass = invInertia
end

--- @type table<TLP2D.BodyType, fun(bodyData: TLP2D.BodyData, bodyCache: TLP2D.BodyCache)>
local bodyUpdateAngularMassFunctions = {
	bodyUpdateAngularMassCircleShape, -- Circle
	bodyUpdateAngularMassRectangleShape, -- Rectangle
	bodyUpdateAngularMassPolygonShape, -- Polygon
}

--- @param bodyData TLP2D.BodyData
--- @param bodyCache TLP2D.BodyCache
local function bodyUpdateAngularMass(bodyData, bodyCache)
	if bodyData[BODY_DATA_TYPE] == TLP2D_Body_Type_Static then
		bodyCache.angularMass = 0
		bodyCache.invAngularMass = 0
	else
		bodyUpdateAngularMassFunctions[bodyData[BODY_DATA_SHAPE]](bodyData, bodyCache)
	end
end

--- @param bodyData TLP2D.BodyData
--- @param bodyCache TLP2D.BodyCache
--- @nodiscard
local function TLP2D_Body_getAngularMass(bodyData, bodyCache)
	if bodyCache.needUpdateAngularMass then
		bodyCache.needUpdateAngularMass = false
		bodyUpdateAngularMass(bodyData, bodyCache)
	end
	return bodyCache.angularMass, bodyCache.invAngularMass
end

--- @param bodyData TLP2D.BodyData
--- @param bodyCache TLP2D.BodyCache
local function TLP2D_Body_update(bodyData, bodyCache, deltaTime)
	local needUpdateTransform = false

	do
		local lvx = bodyData[BODY_DATA_LINEAR_VELOCITY_X]
		local lvy = bodyData[BODY_DATA_LINEAR_VELOCITY_Y]
		if lvx ~= 0 or lvy ~= 0 then
			local linearDrag = math_exp(-materials[bodyData[BODY_DATA_MATERIAL]].linearDrag * deltaTime)
			lvx = lvx * linearDrag
			lvy = lvy * linearDrag
			bodyData[BODY_DATA_POSITION_X] = bodyData[BODY_DATA_POSITION_X] + lvx * deltaTime
			bodyData[BODY_DATA_POSITION_Y] = bodyData[BODY_DATA_POSITION_Y] + lvy * deltaTime
			needUpdateTransform = true
			bodyData[BODY_DATA_LINEAR_VELOCITY_X] = lvx
			bodyData[BODY_DATA_LINEAR_VELOCITY_Y] = lvy
		end
	end

	do
		local av = bodyData[BODY_DATA_ANGULAR_VELOCITY]
		if av ~= 0 then
			local angularDrag = math_exp(-materials[bodyData[BODY_DATA_MATERIAL]].angularDrag * deltaTime)
			av = av * angularDrag
			bodyData[BODY_DATA_ROTATION] = bodyData[BODY_DATA_ROTATION] + av * deltaTime
			needUpdateTransform = true
			bodyData[BODY_DATA_ANGULAR_VELOCITY] = av
		end
	end

	if needUpdateTransform then
		bodyCache.needUpdateTransform = true
	end
end

local function bodyKinematicApplyImpulse(bodyData, impX, impY, invMass)
	if invMass ~= 0 then
		bodyData[BODY_DATA_LINEAR_VELOCITY_X] = bodyData[BODY_DATA_LINEAR_VELOCITY_X] + impX * invMass
		bodyData[BODY_DATA_LINEAR_VELOCITY_Y] = bodyData[BODY_DATA_LINEAR_VELOCITY_Y] + impY * invMass
	end
end

local function bodyDynamicApplyImpulse(bodyData, relX, relY, impX, impY, invMass, invAngMass)
	if invMass ~= 0 then
		bodyData[BODY_DATA_LINEAR_VELOCITY_X] = bodyData[BODY_DATA_LINEAR_VELOCITY_X] + impX * invMass
		bodyData[BODY_DATA_LINEAR_VELOCITY_Y] = bodyData[BODY_DATA_LINEAR_VELOCITY_Y] + impY * invMass
	end

	if invAngMass ~= 0 then
		bodyData[BODY_DATA_ANGULAR_VELOCITY] = bodyData[BODY_DATA_ANGULAR_VELOCITY]
			+ TLP2D_Vector_cross(relX, relY, impX, impY) * invAngMass
	end
end

--- @param bodyData1 TLP2D.BodyData
--- @param bodyData2 TLP2D.BodyData
--- @return number staticFriction
--- @return number dynamicFriction
local function bodyGetCombinedFrictions(bodyData1, bodyData2)
	local m1 = materials[bodyData1[BODY_DATA_MATERIAL]]
	local m2 = materials[bodyData2[BODY_DATA_MATERIAL]]
	local fc = m1.frictionCombine
	if fc == TLP2D_Material_FrictionCombine_Average then
		return (m1.staticFriction + m2.staticFriction) * 0.5, (m1.dynamicFriction + m2.dynamicFriction) * 0.5
	elseif fc == TLP2D_Material_FrictionCombine_Minimum then
		return math_min(m1.staticFriction, m2.staticFriction), math_min(m1.dynamicFriction, m2.dynamicFriction)
	elseif fc == TLP2D_Material_FrictionCombine_Maximum then
		return math_max(m1.staticFriction, m2.staticFriction), math_max(m1.dynamicFriction, m2.dynamicFriction)
	elseif fc == TLP2D_Material_FrictionCombine_Multiply then
		return m1.staticFriction * m2.staticFriction, m1.dynamicFriction * m2.dynamicFriction
	else
		return 0, 0
	end
end

--- @param bodyData1 TLP2D.BodyData
--- @param bodyData2 TLP2D.BodyData
--- @return number restitution
local function bodyGetCombinedRestitution(bodyData1, bodyData2)
	return math_min(
		materials[bodyData1[BODY_DATA_MATERIAL]].restitution,
		materials[bodyData2[BODY_DATA_MATERIAL]].restitution
	)
end

--- @param bodyData1 TLP2D.BodyData
--- @param bodyCache1 TLP2D.BodyCache
--- @param bodyData2 TLP2D.BodyData
--- @param bodyCache2 TLP2D.BodyCache
--- @param normalX number
--- @param normalY number
--- @param depth number
local function bodySolvePositionConstraints(bodyData1, bodyCache1, bodyData2, bodyCache2, normalX, normalY, depth)
	if bodyData1[BODY_DATA_TYPE] == TLP2D_Body_Type_Static then
		if bodyData2[BODY_DATA_TYPE] == TLP2D_Body_Type_Static then
			-- pass
		else
			bodyData2[BODY_DATA_POSITION_X] = bodyData2[BODY_DATA_POSITION_X] + normalX * depth
			bodyData2[BODY_DATA_POSITION_Y] = bodyData2[BODY_DATA_POSITION_Y] + normalY * depth
			bodyCache2.needUpdateTransform = true
		end
	else
		if bodyData2[BODY_DATA_TYPE] == TLP2D_Body_Type_Static then
			bodyData1[BODY_DATA_POSITION_X] = bodyData1[BODY_DATA_POSITION_X] - normalX * depth
			bodyData1[BODY_DATA_POSITION_Y] = bodyData1[BODY_DATA_POSITION_Y] - normalY * depth
			bodyCache1.needUpdateTransform = true
		else
			local mass1 = TLP2D_Body_getMass(bodyData1, bodyCache1)
			local mass2 = TLP2D_Body_getMass(bodyData2, bodyCache2)
			local ratio1 = mass1 / (mass1 + mass2)
			local ratio2 = 1 - ratio1
			bodyData1[BODY_DATA_POSITION_X] = bodyData1[BODY_DATA_POSITION_X] - normalX * depth * ratio1
			bodyData1[BODY_DATA_POSITION_Y] = bodyData1[BODY_DATA_POSITION_Y] - normalY * depth * ratio1
			bodyData2[BODY_DATA_POSITION_X] = bodyData2[BODY_DATA_POSITION_X] + normalX * depth * ratio2
			bodyData2[BODY_DATA_POSITION_Y] = bodyData2[BODY_DATA_POSITION_Y] + normalY * depth * ratio2
			bodyCache1.needUpdateTransform = true
			bodyCache2.needUpdateTransform = true
		end
	end
end

--- @param bodyData1 TLP2D.BodyData
--- @param bodyData2 TLP2D.BodyData
--- @param normalX number
--- @param normalY number
--- @param invMass1 number
--- @param invMass2 number
local function bodyVelocityConstraintsKinematic(
	bodyData1,
	bodyCache1,
	bodyData2,
	bodyCache2,
	normalX,
	normalY,
	invMass1,
	invMass2
)
	local rvx = bodyData2[BODY_DATA_LINEAR_VELOCITY_X] - bodyData1[BODY_DATA_LINEAR_VELOCITY_X]
	local rvy = bodyData2[BODY_DATA_LINEAR_VELOCITY_Y] - bodyData1[BODY_DATA_LINEAR_VELOCITY_Y]
	if TLP2D_Vector_dot(rvx, rvy, normalX, normalY) > 0 then
		return
	end

	local e = bodyGetCombinedRestitution(bodyData1, bodyData2)
	local j = -(1 + e) * TLP2D_Vector_dot(rvx, rvy, normalX, normalY) / (invMass1 + invMass2)

	do
		local impX = j * normalX
		local impY = j * normalY

		bodyKinematicApplyImpulse(bodyData1, -impX, -impY, invMass1)
		bodyKinematicApplyImpulse(bodyData2, impX, impY, invMass2)
	end

	local tx = -normalY
	local ty = normalX
	if math_abs(tx) > epsilon or math_abs(ty) > epsilon then
		tx, ty = TLP2D_Vector_normalize(tx, ty)

		local staticFriction, dynamicFriction = bodyGetCombinedFrictions(bodyData1, bodyData2)
		local vt = TLP2D_Vector_dot(rvx, rvy, tx, ty)
		local jt = -vt / (invMass1 + invMass2)

		local impX, impY
		if math_abs(jt) <= j * staticFriction then
			impX = jt * tx
			impY = jt * ty
		else
			local df = j * dynamicFriction * (vt > 0 and -1 or 1)
			impX = df * tx
			impY = df * ty
		end

		bodyKinematicApplyImpulse(bodyData1, -impX, -impY, invMass1)
		bodyKinematicApplyImpulse(bodyData2, impX, impY, invMass2)
	end
end

local bodyVelocityConstraintsDynamicJList = {}

local function bodyVelocityConstraintsDynamicCalParams(
	bodyData1,
	bodyData2,
	contactList,
	contactIndex,
	av1,
	av2,
	lvx1,
	lvy1,
	lvx2,
	lvy2
)
	local contactVector = contactList[contactIndex]

	local rx1 = contactVector[1] - bodyData1[BODY_DATA_POSITION_X]
	local ry1 = contactVector[2] - bodyData1[BODY_DATA_POSITION_Y]
	local rx2 = contactVector[1] - bodyData2[BODY_DATA_POSITION_X]
	local ry2 = contactVector[2] - bodyData2[BODY_DATA_POSITION_Y]

	local rpx1 = -ry1
	local rpy1 = rx1
	local rpx2 = -ry2
	local rpy2 = rx2

	local alvx1 = rpx1 * av1
	local alvy1 = rpy1 * av1
	local alvx2 = rpx2 * av2
	local alvy2 = rpy2 * av2

	local rvx = (lvx2 + alvx2) - (lvx1 + alvx1)
	local rvy = (lvy2 + alvy2) - (lvy1 + alvy1)

	return rpx1, rpy1, rpx2, rpy2, rx1, ry1, rx2, ry2, rvx, rvy
end

--- @param bodyData1 any
--- @param bodyCache1 any
--- @param bodyData2 any
--- @param bodyCache2 any
--- @param normalX any
--- @param normalY any
--- @param invMass1 any
--- @param invMass2 any
local function bodyVelocityConstraintsDynamic(
	bodyData1,
	bodyCache1,
	bodyData2,
	bodyCache2,
	normalX,
	normalY,
	invMass1,
	invMass2,
	contactCount,
	contactList
)
	if contactCount <= 0 then
		return
	end

	local e = bodyGetCombinedRestitution(bodyData1, bodyData2)

	local lv1x = bodyData1[BODY_DATA_LINEAR_VELOCITY_X]
	local lv1y = bodyData1[BODY_DATA_LINEAR_VELOCITY_Y]
	local lv2x = bodyData2[BODY_DATA_LINEAR_VELOCITY_X]
	local lv2y = bodyData2[BODY_DATA_LINEAR_VELOCITY_Y]
	local av1 = bodyData1[BODY_DATA_ANGULAR_VELOCITY]
	local av2 = bodyData2[BODY_DATA_ANGULAR_VELOCITY]

	local invAngMass1, invAngMass2
	do
		local _
		_, invAngMass1 = TLP2D_Body_getAngularMass(bodyData1, bodyCache1)
		_, invAngMass2 = TLP2D_Body_getAngularMass(bodyData2, bodyCache2)
	end

	for i = 1, contactCount do
		local rpx1, rpy1, rpx2, rpy2, rx1, ry1, rx2, ry2, rvx, rvy = bodyVelocityConstraintsDynamicCalParams(
			bodyData1,
			bodyData2,
			contactList,
			i,
			av1,
			av2,
			lv1x,
			lv1y,
			lv2x,
			lv2y
		)
		local cvm = TLP2D_Vector_dot(rvx, rvy, normalX, normalY)
		if cvm <= 0 then
			local rpn1 = TLP2D_Vector_dot(rpx1, rpy1, normalX, normalY)
			local rpn2 = TLP2D_Vector_dot(rpx2, rpy2, normalX, normalY)
			local denom = invMass1 + invMass2 + rpn1 * rpn1 * invAngMass1 + rpn2 * rpn2 * invAngMass2
			local j = -(1 + e) * cvm / denom
			bodyVelocityConstraintsDynamicJList[i] = j

			local impX = j * normalX
			local impY = j * normalY

			bodyDynamicApplyImpulse(bodyData1, rx1, ry1, -impX, -impY, invMass1, invAngMass1)
			bodyDynamicApplyImpulse(bodyData2, rx2, ry2, impX, impY, invMass2, invAngMass2)
		else
			bodyVelocityConstraintsDynamicJList[i] = 0
		end
	end

	lv1x = bodyData1[BODY_DATA_LINEAR_VELOCITY_X]
	lv1y = bodyData1[BODY_DATA_LINEAR_VELOCITY_Y]
	lv2x = bodyData2[BODY_DATA_LINEAR_VELOCITY_X]
	lv2y = bodyData2[BODY_DATA_LINEAR_VELOCITY_Y]
	av1 = bodyData1[BODY_DATA_ANGULAR_VELOCITY]
	av2 = bodyData2[BODY_DATA_ANGULAR_VELOCITY]

	local staticFriction, dynamicFriction = bodyGetCombinedFrictions(bodyData1, bodyData2)

	for i = 1, contactCount do
		local rpx1, rpy1, rpx2, rpy2, rx1, ry1, rx2, ry2, rvx, rvy = bodyVelocityConstraintsDynamicCalParams(
			bodyData1,
			bodyData2,
			contactList,
			i,
			av1,
			av2,
			lv1x,
			lv1y,
			lv2x,
			lv2y
		)
		local tx, ty
		do
			local cvm = TLP2D_Vector_dot(rvx, rvy, normalX, normalY)
			tx = rvx - cvm * normalX
			ty = rvy - cvm * normalY
		end
		if math_abs(tx) > epsilon or math_abs(ty) > epsilon then
			tx, ty = TLP2D_Vector_normalize(tx, ty)
			local rpt1 = TLP2D_Vector_dot(rpx1, rpy1, tx, ty)
			local rpt2 = TLP2D_Vector_dot(rpx2, rpy2, tx, ty)
			local denom = invMass1 + invMass2 + rpt1 * rpt1 * invAngMass1 + rpt2 * rpt2 * invAngMass2
			local jt = -TLP2D_Vector_dot(rvx, rvy, tx, ty) / denom
			local j = bodyVelocityConstraintsDynamicJList[i]

			local impX, impY
			if math_abs(jt) <= j * staticFriction then
				impX = jt * tx
				impY = jt * ty
			else
				local df = -j * dynamicFriction
				impX = df * tx
				impY = df * ty
			end

			bodyDynamicApplyImpulse(bodyData1, rx1, ry1, -impX, -impY, invMass1, invAngMass1)
			bodyDynamicApplyImpulse(bodyData2, rx2, ry2, impX, impY, invMass2, invAngMass2)
		end
	end
end

--- @param bodyData1 TLP2D.BodyData
--- @param bodyCache1 TLP2D.BodyCache
--- @param bodyData2 TLP2D.BodyData
--- @param bodyCache2 TLP2D.BodyCache
--- @param normalX number
--- @param normalY number
--- @param invMass1 number
--- @param invMass2 number
--- @param contactCount number
--- @param contactList TLP2D.Vector[]
local function bodyVelocityConstraintsDynamicWithKinematic(
	bodyData1,
	bodyCache1,
	bodyData2,
	bodyCache2,
	normalX,
	normalY,
	invMass1,
	invMass2,
	contactCount,
	contactList
)
	if contactCount <= 0 then
		return
	end

	local e = bodyGetCombinedRestitution(bodyData1, bodyData2)

	local lvx = bodyData1[BODY_DATA_LINEAR_VELOCITY_X]
	local lvy = bodyData1[BODY_DATA_LINEAR_VELOCITY_Y]
	local av = bodyData1[BODY_DATA_ANGULAR_VELOCITY]

	local invAngMass1
	do
		local _
		_, invAngMass1 = TLP2D_Body_getAngularMass(bodyData1, bodyCache1)
	end

	for i = 1, contactCount do
		local _, rpx1, rpy1, rx1, ry1, rvx, rvy
		rpx1, rpy1, _, _, rx1, ry1, _, _, rvx, rvy = bodyVelocityConstraintsDynamicCalParams(
			bodyData1,
			bodyData2,
			contactList,
			i,
			av,
			0,
			lvx,
			lvy,
			bodyData2[BODY_DATA_LINEAR_VELOCITY_X],
			bodyData2[BODY_DATA_LINEAR_VELOCITY_Y]
		)

		local cvm = TLP2D_Vector_dot(rvx, rvy, normalX, normalY)
		if cvm <= 0 then
			local rpn = TLP2D_Vector_dot(rpx1, rpy1, normalX, normalY)

			local denom = invMass1 + rpn ^ 2 * invAngMass1

			local j = -(1 + e) * cvm / denom / contactCount
			bodyVelocityConstraintsDynamicJList[i] = j

			local impX = j * normalX
			local impY = j * normalY

			bodyDynamicApplyImpulse(bodyData1, rx1, ry1, -impX, -impY, invMass1, invAngMass1)
			bodyKinematicApplyImpulse(bodyData2, impX, impY, invMass2)
		else
			bodyVelocityConstraintsDynamicJList[i] = 0
		end
	end

	local staticFriction, dynamicFriction = bodyGetCombinedFrictions(bodyData1, bodyData2)

	lvx = bodyData1[BODY_DATA_LINEAR_VELOCITY_X]
	lvy = bodyData1[BODY_DATA_LINEAR_VELOCITY_Y]
	av = bodyData1[BODY_DATA_ANGULAR_VELOCITY]

	for i = 1, contactCount do
		local rpxD, rpyD, _, _, rxD, ryD, _, _, rvx, rvy = bodyVelocityConstraintsDynamicCalParams(
			bodyData1,
			bodyData2,
			contactList,
			i,
			av,
			0,
			lvx,
			lvy,
			bodyData2[BODY_DATA_LINEAR_VELOCITY_X],
			bodyData2[BODY_DATA_LINEAR_VELOCITY_Y]
		)

		local cvm = TLP2D_Vector_dot(rvx, rvy, normalX, normalY)

		local tx = rvx - cvm * normalX
		local ty = rvy - cvm * normalY
		if math_abs(tx) > epsilon or math_abs(ty) > epsilon then
			tx, ty = TLP2D_Vector_normalize(tx, ty)

			local rpt = TLP2D_Vector_dot(rpxD, rpyD, tx, ty)
			local denom = invMass1 + rpt ^ 2 * invAngMass1

			local jt = -TLP2D_Vector_dot(rvx, rvy, tx, ty) / denom / contactCount

			local j = bodyVelocityConstraintsDynamicJList[i]
			local maxFriction = j * staticFriction
			local impX, impY

			if math_abs(jt) <= maxFriction then
				impX = jt * tx
				impY = jt * ty
			else
				impX = -j * dynamicFriction * tx
				impY = -j * dynamicFriction * ty
			end

			bodyDynamicApplyImpulse(bodyData1, rxD, ryD, -impX, -impY, invMass1, invAngMass1)
			bodyKinematicApplyImpulse(bodyData2, impX, impY, invMass2)
		end
	end
end

--- @param bodyData1 TLP2D.BodyData
--- @param bodyCache1 TLP2D.BodyCache
--- @param bodyData2 TLP2D.BodyData
--- @param bodyCache2 TLP2D.BodyCache
local function TLP2D_Body_solveVelocityConstraints(
	bodyData1,
	bodyCache1,
	bodyData2,
	bodyCache2,
	normalX,
	normalY,
	contactCount,
	contactList
)
	local _, invMass1 = TLP2D_Body_getMass(bodyData1, bodyCache1)
	local _, invMass2 = TLP2D_Body_getMass(bodyData2, bodyCache2)

	local bodyType1 = bodyData1[BODY_DATA_TYPE]
	local bodyType2 = bodyData2[BODY_DATA_TYPE]

	if bodyType1 == TLP2D_Body_Type_Static then
		if bodyType2 == TLP2D_Body_Type_Static then
			-- pass
		elseif bodyType2 == TLP2D_Body_Type_Kinematic then
			bodyVelocityConstraintsKinematic( --
				bodyData2,
				bodyCache2,
				bodyData1,
				bodyCache1,
				normalX,
				normalY,
				invMass2,
				invMass1
			)
		elseif bodyType2 == TLP2D_Body_Type_Dynamic then
			bodyVelocityConstraintsDynamic(
				bodyData1,
				bodyCache1,
				bodyData2,
				bodyCache2,
				normalX,
				normalY,
				invMass1,
				invMass2,
				contactCount,
				contactList
			)
		end
	elseif bodyType1 == TLP2D_Body_Type_Kinematic then
		if bodyType2 == TLP2D_Body_Type_Static then
			bodyVelocityConstraintsKinematic( --
				bodyData1,
				bodyCache1,
				bodyData2,
				bodyCache2,
				normalX,
				normalY,
				invMass1,
				invMass2
			)
		elseif bodyType2 == TLP2D_Body_Type_Kinematic then
			bodyVelocityConstraintsKinematic(
				bodyData1,
				bodyCache1,
				bodyData2,
				bodyCache2,
				normalX,
				normalY,
				invMass1,
				invMass2
			)
		elseif bodyType2 == TLP2D_Body_Type_Dynamic then
			bodyVelocityConstraintsDynamicWithKinematic(
				bodyData2,
				bodyCache2,
				bodyData1,
				bodyCache1,
				-normalX,
				-normalY,
				invMass2,
				invMass1,
				contactCount,
				contactList
			)
		end
	elseif bodyType1 == TLP2D_Body_Type_Dynamic then
		if bodyData2[BODY_DATA_TYPE] == TLP2D_Body_Type_Static then
			bodyVelocityConstraintsDynamic(
				bodyData1,
				bodyCache1,
				bodyData2,
				bodyCache2,
				normalX,
				normalY,
				invMass1,
				invMass2,
				contactCount,
				contactList
			)
		elseif bodyType2 == TLP2D_Body_Type_Kinematic then
			bodyVelocityConstraintsDynamicWithKinematic(
				bodyData1,
				bodyCache1,
				bodyData2,
				bodyCache2,
				normalX,
				normalY,
				invMass1,
				invMass2,
				contactCount,
				contactList
			)
		elseif bodyData2[BODY_DATA_TYPE] == TLP2D_Body_Type_Dynamic then
			bodyVelocityConstraintsDynamic(
				bodyData1,
				bodyCache1,
				bodyData2,
				bodyCache2,
				normalX,
				normalY,
				invMass1,
				invMass2,
				contactCount,
				contactList
			)
		end
	end
end

--#endregion

--#region Serializer

--- You can override the serializer if you're not happy with the default one.
TLP2D.Serializer = {}

if env == ENV_LUA_JIT_WOS then -- WOS only
	TLP2D.Serializer.type = "LJBuffer"

	local LJBuffer = require("system.utils.serial.LJBuffer")
	local encode = LJBuffer.encode
	local decode = LJBuffer.decode

	function TLP2D.Serializer.serialize(latestBodyID, bodiesData)
		return encode({ latestBodyID, bodiesData })
	end

	function TLP2D.Serializer.deserialize(buffer)
		local tbl = decode(buffer)
		return tbl[1], tbl[2]
	end
elseif env == ENV_LUA_JIT then -- LuaJIT only
	TLP2D.Serializer.type = "LJBuffer"

	local stringBuffer = require("string.buffer")
	local encode = stringBuffer.encode
	local decode = stringBuffer.decode
else -- Default serializer
	TLP2D.Serializer.type = "Default"

	function TLP2D.Serializer.serialize(latestBodyID, bodiesData)
		-- return TLP2D_Utils_serializeNumberArray({latestBodyID}, bodiesData)
	end

	if pcall(function()
		if type(load) ~= "function" then
			error()
		end
	end) then -- use `load` function to
		function TLP2D.Serializer.deserializeBodiesData(bodiesDataBuffer)
			--- @diagnostic disable-next-line: param-type-mismatch
			return load("return " .. bodiesDataBuffer)()
		end
	else -- if `load` is not supported in some restricted environment
		function TLP2D.Serializer.deserializeBodiesData(bodiesDataBuffer)
			return TLP2D_Utils_deserializeNumberArray(bodiesDataBuffer)
		end
	end
end

--#endregion

--#region World

--- @class TLP2D.WorldID : integer
local worldLatestID = 0
--- @type table<TLP2D.WorldID, TLP2D.WorldData>
local worldsData = {}

--- @readonly
--- @class TLP2D.World
TLP2D.World = {}

TLP2D.World.DefaultIterations = 4

--- @return TLP2D.WorldID
function TLP2D.World.create()
	--- @class TLP2D.WorldData
	local worldData = {}

	local worldID = worldLatestID + 1
	worldLatestID = worldID

	--- @type TLP2D.WorldID
	worldData.id = worldID

	--- @type integer
	worldData.iterations = TLP2D.World.DefaultIterations

	--- @class TLP2D.BodyID : integer
	worldData.bodyLatestID = 0

	--- @type table<TLP2D.BodyID, TLP2D.BodyData>
	worldData.bodiesData = {}

	--- @type table<TLP2D.BodyID, TLP2D.BodyCache>
	worldData.bodyCaches = {}

	--- @type TLP2D.BodyID[]
	worldData.bodyIDs = {}

	--- @type TLP2D.Rectangle?
	worldData.boundary = nil

	--- TODO NOT DONE YET
	--- @type TLP2D.SpatialHashMap?
	worldData.spatialHashMap = nil -- TLP2D.SpatialHashMap.new(50)

	--- @type TLP2D.BodyID[]
	worldData.freeBodyIDList = {}

	worldsData[worldID] = worldData

	return worldID
end

local TLP2D_World_create = TLP2D.World.create

--- @param worldID TLP2D.WorldID
--- @return boolean
--- @nodiscard
function TLP2D.World.exists(worldID)
	return not not worldsData[worldID]
end

local TLP2D_World_exists = TLP2D.World.exists

--- @param worldID TLP2D.WorldID
function TLP2D.World.destroy(worldID)
	if worldsData[worldID] then
		worldsData[worldID] = nil
	elseif worldID == 0 then
		error(("bad argument to #1 'worldID': world %s has already been destroyed"):format(worldID), 2)
	else
		error(("bad argument to #1 'worldID': world %s does not exist"):format(worldID), 2)
	end
end

local TLP2D_World_destroy = TLP2D.World.destroy

--- @param worldID TLP2D.WorldID
--- @param extraLevel integer?
--- @return TLP2D.WorldData
local function worldGetDataOrError(worldID, extraLevel)
	local worldData = worldsData[worldID]
	if worldData then
		return worldData
	end
	error(("bad argument to #1 'worldID': world %s not a physics world"):format(worldID), 3 + (extraLevel or 0))
end

--- @param worldID TLP2D.WorldID
function TLP2D.World.clearBodies(worldID)
	local worldData = worldGetDataOrError(worldID)
	worldData.bodyLatestID = 0
	worldData.bodiesData = {}
	worldData.bodyCaches = {}
	worldData.bodyIDs = {}
end

local TLP2D_World_clearBodies = TLP2D.World.clearBodies

--- @param worldID TLP2D.WorldID
--- @param bodyID TLP2D.BodyID
--- @return boolean
--- @nodiscard
function TLP2D.World.hasBody(worldID, bodyID)
	local worldData = worldGetDataOrError(worldID)
	return not not worldData.bodiesData[bodyID]
end

local TLP2D_World_hasBody = TLP2D.World.hasBody

--- @param worldID TLP2D.WorldID
--- @param bodyType TLP2D.BodyType
--- @return TLP2D.BodyID bodyID
local function worldCreateBody(worldID, bodyType)
	local worldData = worldGetDataOrError(worldID, 1)

	local bodyID = worldData.bodyLatestID + 1
	worldData.bodyLatestID = bodyID

	worldData.bodyIDs[#worldData.bodyIDs + 1] = bodyID

	local bodyCache
	worldData.bodiesData[bodyID], bodyCache = TLP2D_Body_create( --
		bodyID,
		bodyType,
		materialDefault.id,
		TLP2D_Shape_Type_None,
		0
	)
	worldData.bodyCaches[bodyID] = bodyCache

	if worldData.spatialHashMap then
		bodyCache.spatialHashMapElementID = worldData.spatialHashMap:addElement(bodyID)
	end

	return bodyID
end

--- @param worldID TLP2D.WorldID
--- @return TLP2D.BodyID bodyID
function TLP2D.World.createStaticBody(worldID)
	return worldCreateBody(worldID, TLP2D_Body_Type_Static)
end

local TLP2D_World_createStaticBody = TLP2D.World.createStaticBody

--- @param worldID TLP2D.WorldID
--- @return TLP2D.BodyID bodyID
function TLP2D.World.createKinematicBody(worldID)
	return worldCreateBody(worldID, TLP2D_Body_Type_Kinematic)
end

local TLP2D_World_createKinematicBody = TLP2D.World.createKinematicBody

--- @param worldID TLP2D.WorldID
--- @return TLP2D.BodyID bodyID
function TLP2D.World.createDynamicBody(worldID)
	return worldCreateBody(worldID, TLP2D_Body_Type_Dynamic)
end

local TLP2D_World_createDynamicBody = TLP2D.World.createDynamicBody

--- @param worldID TLP2D.WorldID
--- @param bodyID TLP2D.BodyID
function TLP2D.World.destroyBody(worldID, bodyID)
	local worldData = worldGetDataOrError(worldID)
	local bodiesData = worldData.bodiesData
	if not bodiesData[bodyID] then
		error(("body %s does not exits in world %d"):format(bodyID, worldID), 2)
	end

	local bodyCache = worldData.bodyCaches
	if bodyCache[bodyID].spatialHashMapElementID then -- TODO
	else
		-- worldData.freeBodyIDList
	end

	bodiesData[bodyID] = nil
	bodyCache[bodyID] = nil
	TLP2D_Utils_listFastRemoveFirst(worldData.bodyIDs, bodyID)
end

local TLP2D_World_destroyBody = TLP2D.World.destroyBody

--- @param worldID TLP2D.WorldID
--- @param bodyID TLP2D.BodyID
--- @return TLP2D.BodyData
--- @return TLP2D.WorldData
local function worldGetBodyDataOrError(worldID, bodyID)
	local worldData = worldGetDataOrError(worldID, 1)
	local body = worldData.bodiesData[bodyID]
	if body then
		return body, worldData
	end
	error("bad argument to #2 'bodyID': body not found", 3)
end

--- @param worldID TLP2D.WorldID
--- @param bodyID TLP2D.BodyID
--- @return TLP2D.BodyType
--- @nodiscard
function TLP2D.World.getBodyType(worldID, bodyID)
	return worldGetBodyDataOrError(worldID, bodyID)[BODY_DATA_TYPE]
end

local TLP2D_World_getBodyType = TLP2D.World.getBodyType

--- @param worldID TLP2D.WorldID
--- @param bodyID TLP2D.BodyID
--- @return number x
--- @return number y
--- @nodiscard
function TLP2D.World.getBodyPosition(worldID, bodyID)
	local bodyData = worldGetBodyDataOrError(worldID, bodyID)
	return bodyData[BODY_DATA_POSITION_X], bodyData[BODY_DATA_POSITION_Y]
end

local TLP2D_World_getBodyPosition = TLP2D.World.getBodyPosition

--- Set body's position to a new value, this will not trigger advanced collision response.
--- @param worldID TLP2D.WorldID
--- @param bodyID TLP2D.BodyID
--- @param x number
--- @param y number
function TLP2D.World.setBodyPosition(worldID, bodyID, x, y)
	local bodyData, worldData = worldGetBodyDataOrError(worldID, bodyID)
	bodyData[BODY_DATA_POSITION_X] = x
	bodyData[BODY_DATA_POSITION_Y] = y
	worldData.bodyCaches[bodyID].needUpdateTransform = true
end

local TLP2D_World_setBodyPosition = TLP2D.World.setBodyPosition

--- Change body's position to a new value, this will not trigger advanced collision response.
--- @param worldID TLP2D.WorldID
--- @param bodyID TLP2D.BodyID
--- @param dx number
--- @param dy number
function TLP2D.World.changeBodyPosition(worldID, bodyID, dx, dy)
	local bodyData, worldData = worldGetBodyDataOrError(worldID, bodyID)
	bodyData[BODY_DATA_POSITION_X] = bodyData[BODY_DATA_POSITION_X] + dx
	bodyData[BODY_DATA_POSITION_Y] = bodyData[BODY_DATA_POSITION_Y] + dy
	worldData.bodyCaches[bodyID].needUpdateTransform = true
end

local TLP2D_World_changeBodyPosition = TLP2D.World.changeBodyPosition

--- @param worldID TLP2D.WorldID
--- @param bodyID TLP2D.BodyID
--- @return number rotation
--- @nodiscard
function TLP2D.World.getBodyRotation(worldID, bodyID)
	return worldGetBodyDataOrError(worldID, bodyID)[BODY_DATA_ROTATION]
end

local TLP2D_World_getBodyRotation = TLP2D.World.getBodyRotation

--- Set body's rotation to a new value, this will not trigger advanced collision response.
--- @param worldID TLP2D.WorldID
--- @param bodyID TLP2D.BodyID
--- @param r number
function TLP2D.World.setBodyRotation(worldID, bodyID, r)
	local bodyData, worldData = worldGetBodyDataOrError(worldID, bodyID)
	bodyData[BODY_DATA_ROTATION] = r
	worldData.bodyCaches[bodyID].needUpdateTransform = true
end

local TLP2D_World_setBodyRotation = TLP2D.World.setBodyRotation

--- @param worldID TLP2D.WorldID
--- @param bodyID TLP2D.BodyID
--- @param dr number
function TLP2D.World.changeBodyRotation(worldID, bodyID, dr)
	local bodyData = worldGetBodyDataOrError(worldID, bodyID)
	bodyData[BODY_DATA_ROTATION] = bodyData[BODY_DATA_ROTATION] + dr
end

local TLP2D_World_changeBodyRotation = TLP2D.World.changeBodyRotation

--- @param worldID TLP2D.WorldID
--- @param bodyID TLP2D.BodyID
--- @param dvx number
--- @param dvy number
function TLP2D.World.changeBodyVelocity(worldID, bodyID, dvx, dvy)
	local bodyData = worldGetBodyDataOrError(worldID, bodyID)
	bodyData[BODY_DATA_LINEAR_VELOCITY_X] = bodyData[BODY_DATA_LINEAR_VELOCITY_X] + dvx
	bodyData[BODY_DATA_LINEAR_VELOCITY_Y] = bodyData[BODY_DATA_LINEAR_VELOCITY_Y] + dvy
end

local TLP2D_World_changeBodyVelocity = TLP2D.World.changeBodyVelocity

--- @param worldID TLP2D.WorldID
--- @param bodyID TLP2D.BodyID
--- @return number angularVelocity
--- @nodiscard
function TLP2D.World.getBodyAngularVelocity(worldID, bodyID)
	local bodyData = worldGetBodyDataOrError(worldID, bodyID)
	return bodyData[BODY_DATA_ANGULAR_VELOCITY]
end

local TLP2D_World_getBodyAngularVelocity = TLP2D.World.getBodyAngularVelocity

--- @param worldID TLP2D.WorldID
--- @param bodyID TLP2D.BodyID
--- @param av number
function TLP2D.World.setBodyAngularVelocity(worldID, bodyID, av)
	local bodyData = worldGetBodyDataOrError(worldID, bodyID)
	bodyData[BODY_DATA_ANGULAR_VELOCITY] = av
end

local TLP2D_World_setBodyAngularVelocity = TLP2D.World.setBodyAngularVelocity

--- @param worldID TLP2D.WorldID
--- @param bodyID TLP2D.BodyID
--- @param dav number
function TLP2D.World.changeBodyAngularVelocity(worldID, bodyID, dav)
	local bodyData = worldGetBodyDataOrError(worldID, bodyID)
	bodyData[BODY_DATA_ANGULAR_VELOCITY] = bodyData[BODY_DATA_ANGULAR_VELOCITY] + dav
end

local TLP2D_World_changeBodyAngularVelocity = TLP2D.World.changeBodyAngularVelocity

--- @param worldID TLP2D.WorldID
--- @param bodyID TLP2D.BodyID
--- @return TLP2D.MaterialID materialID
--- @nodiscard
function TLP2D.World.getBodyMaterial(worldID, bodyID)
	local bodyData = worldGetBodyDataOrError(worldID, bodyID)
	return bodyData[BODY_DATA_MATERIAL]
end

--- @param worldID TLP2D.WorldID
--- @param material TLP2D.MaterialID | TLP2D.MaterialName
function TLP2D.World.setBodyMaterial(worldID, bodyID, material)
	local bodyData = worldGetBodyDataOrError(worldID, bodyID)
	if TLP2D_Material_contains(material) then
		bodyData[BODY_DATA_MATERIAL] = TLP2D.Material.getID(material)
	else
		error("bad argument to #2: material is not valid", 3)
	end
end

local TLP2D_World_setBodyMaterial = TLP2D.World.setBodyMaterial

--- @param worldID TLP2D.WorldID
--- @param bodyID TLP2D.BodyID
--- @return TLP2D.BodyShape
--- @nodiscard
function TLP2D.World.getBodyShape(worldID, bodyID)
	return worldGetBodyDataOrError(worldID, bodyID)[BODY_DATA_SHAPE]
end

local TLP2D_World_getBodyShape = TLP2D.World.getBodyShape

--- @param worldID TLP2D.WorldID
--- @param bodyID TLP2D.BodyID
--- @return TLP2D.BodyShape
--- @nodiscard
function TLP2D.World.getBodyData(worldID, bodyID)
	return worldGetBodyDataOrError(worldID, bodyID)[BODY_DATA_SHAPE_DATA]
end

local TLP2D_World_getBodyShapeData = TLP2D.World.getBodyData

--- @param worldID TLP2D.WorldID
--- @param bodyID TLP2D.BodyID
--- @param radius number
function TLP2D.World.setBodyShapeCircle(worldID, bodyID, radius)
	local bodyData, worldData = worldGetBodyDataOrError(worldID, bodyID)
	if bodyData[BODY_DATA_SHAPE] ~= TLP2D_Shape_Type_Circle then
		if type(radius) ~= "number" then
			error("bad argument to #2 'radius': number expected got " .. type(radius))
		end
		bodyData[BODY_DATA_SHAPE] = TLP2D_Shape_Type_Circle
		bodyData[BODY_DATA_SHAPE_DATA] = radius
		worldData.bodyCaches[bodyID].needUpdateTransform = true
	end
end

local TLP2D_World_setBodyShapeCircle = TLP2D.World.setBodyShapeCircle

--- @param worldID TLP2D.WorldID
--- @param bodyID TLP2D.BodyID
--- @param width number
--- @param height number
function TLP2D.World.setBodyShapeRectangle(worldID, bodyID, width, height)
	local bodyData, worldData = worldGetBodyDataOrError(worldID, bodyID)
	if bodyData[BODY_DATA_SHAPE] ~= TLP2D_Shape_Type_Rectangle then
		if type(width) ~= "number" then
			error("bad argument to #2 'radius': number expected got " .. type(width), 2)
		elseif type(height) ~= "number" then
			error("bad argument to #2 'radius': number expected got " .. type(height), 2)
		end
		bodyData[BODY_DATA_SHAPE] = TLP2D_Shape_Type_Rectangle
		bodyData[BODY_DATA_SHAPE_DATA] = { width, height }
		worldData.bodyCaches[bodyID].needUpdateTransform = true
	end
end

local TLP2D_World_setBodyShapeRectangle = TLP2D.World.setBodyShapeRectangle

--- @param worldID TLP2D.WorldID
--- @param bodyID TLP2D.BodyID
--- @param vertices TLP2D.Vertex[]
function TLP2D.World.setBodyShapePolygon(worldID, bodyID, vertices)
	local bodyData, worldData = worldGetBodyDataOrError(worldID, bodyID)
	if bodyData[BODY_DATA_SHAPE] ~= TLP2D_Shape_Type_Polygon then
		if type(vertices) ~= "table" then
			error("bad argument to #2 'vertices': table expected got " .. type(vertices), 2)
		elseif #vertices < 3 then
			error("bad argument to #2 'vertices': polygon vertices number must be greater than or equals to 3", 2)
		end
		local verticesClone = {}
		for index, vertex in ipairs(vertices) do
			verticesClone[index] = { tonumber(vertex[1]) or 0, tonumber(vertex[2]) or 0 }
		end
		bodyData[BODY_DATA_SHAPE] = TLP2D_Shape_Type_Polygon
		bodyData[BODY_DATA_SHAPE_DATA] = verticesClone
		worldData.bodyCaches[bodyID].needUpdateTransform = true
	end
end

local TLP2D_World_setBodyShapePolygon = TLP2D.World.setBodyShapePolygon

--- @param worldID TLP2D.WorldID
--- @param bodyID TLP2D.BodyID
--- @return number radius @return 0 if the body is not in circle shape.
--- @nodiscard
function TLP2D.World.getBodyRadius(worldID, bodyID)
	local body = worldGetBodyDataOrError(worldID, bodyID)
	if body[BODY_DATA_SHAPE] == TLP2D_Shape_Type_Circle then
		return body[BODY_DATA_SHAPE_DATA]
	else
		return 0
	end
end

local TLP2D_World_getBodyRadius = TLP2D.World.getBodyRadius

--- @param worldID TLP2D.WorldID
--- @param bodyID TLP2D.BodyID
--- @return TLP2D.Vertex[] @return an empty table if the body is in circle shape.
--- @nodiscard
function TLP2D.World.getBodyVertices(worldID, bodyID)
	local bodyData, worldData = worldGetBodyDataOrError(worldID, bodyID)
	return TLP2D_Body_getTransformedVertices(bodyData, worldData.bodyCaches[bodyID])
end

local TLP2D_World_getBodyVertices = TLP2D.World.getBodyVertices

--- @param worldID TLP2D.WorldID
--- @param bodyID TLP2D.BodyID
--- @return number | TLP2D.Vertex[] dataRadiusOrVertices @return radius if the body is in circle shape, return vertices otherwise.
--- @nodiscard
function TLP2D.World.getBodyShapeRadiusOrVertices(worldID, bodyID)
	local bodyData, worldData = worldGetBodyDataOrError(worldID, bodyID)
	if bodyData[BODY_DATA_SHAPE] == TLP2D_Shape_Type_Circle then
		return bodyData[BODY_DATA_SHAPE_DATA]
	else
		return TLP2D_Body_getTransformedVertices(bodyData, worldData.bodyCaches[bodyID])
	end
end

local TLP2D_World_getBodyRadiusOrVertices = TLP2D.World.getBodyShapeRadiusOrVertices

--- @param worldID TLP2D.WorldID
--- @return fun(table: TLP2D.BodyID[], i?: integer): (i: integer, bodyID: TLP2D.BodyID) func
--- @return TLP2D.BodyID bodyIDs
--- @return integer i
function TLP2D.World.iterateBodyIDs(worldID)
	local worldData = worldGetDataOrError(worldID)
	return ipairs(worldData.bodyIDs)
end

--- @param worldID TLP2D.WorldID
--- @return integer
--- @nodiscard
function TLP2D.World.getIterations(worldID)
	return worldGetDataOrError(worldID).iterations
end

local TLP2D_World_getIterations = TLP2D.World.getIterations

--- @param worldID TLP2D.WorldID
--- @param iterations integer
function TLP2D.World.setIterations(worldID, iterations)
	if type(iterations) ~= "number" then
		error("bad argument to #2 'iterations': number expected got " .. type(iterations))
	elseif iterations > 0 then
		iterations = math_floor(iterations)
	else
		iterations = TLP2D.World.DefaultIterations
	end
	worldGetDataOrError(worldID).iterations = iterations
end

local TLP2D_World_setIterations = TLP2D.World.setIterations

--- @param worldData TLP2D.WorldData
--- @param deltaTime number
local function worldUpdateBodies(worldData, deltaTime)
	local bodiesData = worldData.bodiesData
	local bodyCaches = worldData.bodyCaches
	local bodyIDs = worldData.bodyIDs
	local spatialHashMap = worldData.spatialHashMap

	for bodyIndex = 1, #bodyIDs do
		local bodyID = bodyIDs[bodyIndex]
		local bodyData = bodiesData[bodyID]
		local bodyCache = bodyCaches[bodyID]

		TLP2D_Body_update(bodyData, bodyCache, deltaTime)

		if spatialHashMap and bodyCache.spatialHashMapElementID then
			local aabb = TLP2D_Body_getTransformedAABB(bodyData, bodyCache)
			spatialHashMap:updateElement(bodyCache.spatialHashMapElementID, aabb[1], aabb[2], aabb[3], aabb[4])
		end
	end
end

local function worldResolveBodyBoundaryCollision(bodyData, bodyCache, minX, minY, maxX, maxY)
	local aabb = TLP2D_Body_getTransformedAABB(bodyData, bodyCache)
	local px = bodyData[BODY_DATA_POSITION_X]
	local py = bodyData[BODY_DATA_POSITION_Y]

	local w = aabb[3] - aabb[1]
	local h = aabb[4] - aabb[2]
	local halfW = w * 0.5
	local halfH = h * 0.5

	local corrected

	if aabb[1] < minX then
		local depth = minX - aabb[1]
		px = px + depth
		corrected = 0
	elseif aabb[3] > maxX then
		local depth = aabb[3] - maxX
		px = px - depth
		corrected = 0
	end

	if aabb[2] < minY then
		local pen = minY - aabb[2]
		py = py + pen
		corrected = 1
	elseif aabb[4] > maxY then
		local pen = aabb[4] - maxY
		py = py - pen
		corrected = 1
	end

	if w >= (maxX - minX) then
		px = (minX + maxX) * 0.5
		corrected = 0
	else
		px = math_max(px, minX + halfW)
		px = math_min(px, maxX - halfW)
	end

	if h >= (maxY - minY) then
		py = (minY + maxY) * 0.5
		corrected = 1
	else
		py = math_max(py, minY + halfH)
		py = math_min(py, maxY - halfH)
	end

	if corrected ~= nil then
		bodyData[BODY_DATA_POSITION_X] = px
		bodyData[BODY_DATA_POSITION_Y] = py
		if corrected == 0 then
			bodyData[BODY_DATA_LINEAR_VELOCITY_X] = 0
		else
			bodyData[BODY_DATA_LINEAR_VELOCITY_Y] = 0
		end
		bodyCache.needUpdateTransform = true
	end
end

--- @param bodyData1 TLP2D.BodyData
--- @param bodyCache1 TLP2D.BodyCache
--- @param bodyData2 TLP2D.BodyData
--- @param bodyCache2 TLP2D.BodyCache
local function worldCollisionResolveCircleWithCircle(bodyData1, bodyCache1, bodyData2, bodyCache2)
	local depth, normalX, normalY = TLP2D_Collision_intersectCircleWithCircle(
		bodyData1[BODY_DATA_POSITION_X],
		bodyData1[BODY_DATA_POSITION_Y],
		bodyData1[BODY_DATA_SHAPE_DATA],
		bodyData2[BODY_DATA_POSITION_X],
		bodyData2[BODY_DATA_POSITION_Y],
		bodyData2[BODY_DATA_SHAPE_DATA]
	)
	if depth then
		bodySolvePositionConstraints(bodyData1, bodyCache1, bodyData2, bodyCache2, normalX, normalY, depth)
		local contact = TLP2D_Collision_contactCircleWithCircle( --
			bodyData1[BODY_DATA_POSITION_X],
			bodyData1[BODY_DATA_POSITION_Y],
			bodyData1[BODY_DATA_SHAPE_DATA],
			bodyData2[BODY_DATA_POSITION_X],
			bodyData2[BODY_DATA_POSITION_Y]
		)
		TLP2D_Body_solveVelocityConstraints( --
			bodyData1,
			bodyCache1,
			bodyData2,
			bodyCache2,
			normalX,
			normalY,
			contact.count,
			contact.list
		)
	end
end

--- @param bodyData1 TLP2D.BodyData
--- @param bodyCache1 TLP2D.BodyCache
--- @param bodyData2 TLP2D.BodyData
--- @param bodyCache2 TLP2D.BodyCache
local function worldCollisionResolveCircleWithPolygon(bodyData1, bodyCache1, bodyData2, bodyCache2)
	local depth, normalX, normalY = Collision_intersectPolygonWithCircle(
		bodyData2[BODY_DATA_POSITION_X],
		bodyData2[BODY_DATA_POSITION_Y],
		TLP2D_Body_getTransformedVertices(bodyData2, bodyCache2),
		bodyData1[BODY_DATA_POSITION_X],
		bodyData1[BODY_DATA_POSITION_Y],
		bodyData1[BODY_DATA_SHAPE_DATA]
	)
	if depth then
		bodySolvePositionConstraints(bodyData1, bodyCache1, bodyData2, bodyCache2, normalX, normalY, depth)
		local contact = TLP2D_Collision_contactPolygonWithCircle(
			bodyData2[BODY_DATA_POSITION_X],
			bodyData2[BODY_DATA_POSITION_Y],
			TLP2D_Body_getTransformedVertices(bodyData2, bodyCache2),
			bodyData1[BODY_DATA_POSITION_X],
			bodyData1[BODY_DATA_POSITION_Y]
		)
		TLP2D_Body_solveVelocityConstraints( --
			bodyData1,
			bodyCache1,
			bodyData2,
			bodyCache2,
			normalX,
			normalY,
			contact.count,
			contact.list
		)
	end
end

--- @param bodyData1 TLP2D.BodyData
--- @param bodyCache1 TLP2D.BodyCache
--- @param bodyData2 TLP2D.BodyData
--- @param bodyCache2 TLP2D.BodyCache
local function worldCollisionResolvePolygonWithPolygon(bodyData1, bodyCache1, bodyData2, bodyCache2)
	local depth, normalX, normalY = TLP2D_Collision_intersectPolygonWithPolygon( --
		bodyData1[BODY_DATA_POSITION_X],
		bodyData1[BODY_DATA_POSITION_Y],
		TLP2D_Body_getTransformedVertices(bodyData1, bodyCache1),
		bodyData2[BODY_DATA_POSITION_X],
		bodyData2[BODY_DATA_POSITION_Y],
		TLP2D_Body_getTransformedVertices(bodyData2, bodyCache2)
	)
	if depth then
		bodySolvePositionConstraints(bodyData1, bodyCache1, bodyData2, bodyCache2, normalX, normalY, depth)
		local contact = TLP2D_Collision_contactPolygonWithPolygon(
			TLP2D_Body_getTransformedVertices(bodyData1, bodyCache1),
			TLP2D_Body_getTransformedVertices(bodyData2, bodyCache2)
		)
		TLP2D_Body_solveVelocityConstraints( --
			bodyData1,
			bodyCache1,
			bodyData2,
			bodyCache2,
			normalX,
			normalY,
			contact.count,
			contact.list
		)
	end
end

--- @param bodyData1 TLP2D.BodyData
--- @param bodyCache1 TLP2D.BodyCache
--- @param bodyData2 TLP2D.BodyData
--- @param bodyCache2 TLP2D.BodyCache
local function worldResolveBodiesCollisionNarrowly(bodyData1, bodyCache1, bodyData2, bodyCache2)
	if bodyData1[BODY_DATA_SHAPE] == TLP2D_Shape_Type_Circle then
		if bodyData2[BODY_DATA_SHAPE] == TLP2D_Shape_Type_Circle then
			worldCollisionResolveCircleWithCircle(bodyData1, bodyCache1, bodyData2, bodyCache2)
		else
			worldCollisionResolveCircleWithPolygon(bodyData1, bodyCache1, bodyData2, bodyCache2)
		end
	else
		if bodyData2[BODY_DATA_SHAPE] == TLP2D_Shape_Type_Circle then
			worldCollisionResolveCircleWithPolygon(bodyData2, bodyCache2, bodyData1, bodyCache1)
		else
			worldCollisionResolvePolygonWithPolygon(bodyData1, bodyCache1, bodyData2, bodyCache2)
		end
	end
end

--- @param worldData TLP2D.WorldData
--- @param bodyID TLP2D.BodyID
local function worldResolveBodyCollisions(worldData, bodyID)
	local bodiesData = worldData.bodiesData
	local bodyCaches = worldData.bodyCaches

	local bodyData = bodiesData[bodyID]
	local bodyCache = bodyCaches[bodyID]

	-- resolve world boundary collisions

	local boundary = worldData.boundary
	if boundary and bodyData[BODY_DATA_TYPE] ~= TLP2D_Body_Type_Static then
		worldResolveBodyBoundaryCollision(bodyData, bodyCache, boundary[1], boundary[2], boundary[3], boundary[4])
	end

	-- resolve collisions each other

	for _, possibleBodyID in ipairs(worldData.bodyIDs) do
		if
			bodyID ~= possibleBodyID
			and TLP2D_Collision_checkAABB(
				TLP2D_Body_getTransformedAABB(bodyData, bodyCache),
				TLP2D_Body_getTransformedAABB(bodiesData[possibleBodyID], bodyCaches[possibleBodyID])
			)
		then
			worldResolveBodiesCollisionNarrowly(
				bodyData,
				bodyCache,
				bodiesData[possibleBodyID],
				bodyCaches[possibleBodyID]
			)
		end
	end
end

--- @param worldID TLP2D.WorldID
--- @param deltaTime number
function TLP2D.World.tick(worldID, deltaTime)
	if type(deltaTime) ~= "number" then
		error("bad argument to #2 'deltaTime': number expected got " .. type(deltaTime))
	elseif deltaTime <= 0 then
		return
	end

	local worldData = worldGetDataOrError(worldID)
	deltaTime = deltaTime / worldData.iterations

	for _ = 1, worldData.iterations do
		worldUpdateBodies(worldData, deltaTime)

		for _, bodyID in ipairs(worldData.bodyIDs) do
			worldResolveBodyCollisions(worldData, bodyID)
		end
	end
end

local TLP2D_World_tick = TLP2D.World.tick

--- @param worldID TLP2D.WorldID
--- @param x number
--- @param y number
function TLP2D.World.applyGravity(worldID, x, y)
	local worldData = worldGetDataOrError(worldID)
	for _, bodyData in ipairs(worldData.bodiesData) do
		if bodyData[BODY_DATA_TYPE] ~= TLP2D_Body_Type_Static then
			bodyData[BODY_DATA_LINEAR_VELOCITY_X] = bodyData[BODY_DATA_LINEAR_VELOCITY_X] + x
			bodyData[BODY_DATA_LINEAR_VELOCITY_Y] = bodyData[BODY_DATA_LINEAR_VELOCITY_Y] + y
		end
	end
end

local TLP2D_World_applyGravity = TLP2D.World.applyGravity

--- @param worldID TLP2D.WorldID
--- @return number? minX
--- @return number minY
--- @return number maxX
--- @return number maxY
--- @nodiscard
function TLP2D.World.getBoundary(worldID)
	local boundary = worldGetDataOrError(worldID).boundary
	if boundary then
		return boundary[1], boundary[2], boundary[3], boundary[4]
	else
		return nil, 0, 0, 0
	end
end

local TLP2D_World_getBoundary = TLP2D.World.getBoundary

--- @param worldID TLP2D.WorldID
--- @param minX number?
--- @param minY number?
--- @param maxX number?
--- @param maxY number?
function TLP2D.World.setBoundary(worldID, minX, minY, maxX, maxY)
	local worldData = worldGetDataOrError(worldID)
	if minX and minY and maxX and maxY then
		if worldData.boundary then
			local boundary = worldData.boundary
			boundary[1], boundary[2], boundary[3], boundary[4] = minX, minY, maxX, maxY
		else
			worldData.boundary = { minX, minY, maxX, maxY }
		end
	else
		worldData.boundary = nil
	end
end

local TLP2D_World_setBoundary = TLP2D.World.setBoundary

function TLP2D.World.getGridSize(worldID)
	return worldGetDataOrError(worldID).spatialHashMap._gridSize
end

--- @param worldID TLP2D.WorldID
--- @param size number
function TLP2D.World.setGridSize(worldID, size) end

--- @param worldID TLP2D.WorldID
function TLP2D.World.serialize(worldID)
	local worldData = worldGetDataOrError(worldID).bodiesData
	TLP2D.Serializer.serialize()
end

local TLP2D_World_serialize = TLP2D.World.serialize

--- @param worldID TLP2D.WorldID
function TLP2D.World.deserialize(worldID, bodiesBuffer)
	error("Cannot serialize bodies: call `TLP2D.setSerializer` to set serializing functions first", 2)
	bodiesBuffer = bodiesBuffer
end

local TLP2D_World_deserialize = TLP2D.World.deserialize

--#endregion

--#region Object Oriented

local setObjectMetatable
if env == ENV_LUA_JIT_WOS then
	setObjectMetatable = function(object, index, gc)
		return require("system.utils.Proxy").setMetatable(object, {
			__index = index,
			__gc = gc,
		})
	end
elseif env == ENV_LUA_JIT or env == ENV_LUA_51 then
	setObjectMetatable = function(object, index, gc)
		local metatable = {
			__index = index,
			__gc = gc,
		}
		local instance = newproxy(true)
		getmetatable(instance).__gc = metatable.__gc
		metatable[instance] = true
		return setmetatable(object, metatable)
	end
elseif env == ENV_LUA_53 or env == ENV_LUA_54 then
	setObjectMetatable = function(object, index, gc)
		return setmetatable(object, {
			__index = index,
			__gc = gc,
		})
	end
end

--- Determine whether the object-oriented style API is available.
--- Check this value before you use `TLP2D.WorldObject`, `TLP2D.BodyObject`
--- @type boolean
--- @readonly
TLP2D.Object = not not setObjectMetatable

if setObjectMetatable then
	--#region Body Object

	--- @readonly
	--- @class TLP2D.BodyObject
	--- @field package _world TLP2D.WorldObject
	--- @field package _bodyID TLP2D.BodyID
	TLP2D.BodyObject = {}

	--- @return boolean success
	function TLP2D.BodyObject:delete()
		if self._bodyID ~= 0 then
			TLP2D_Utils_listFastRemoveFirst(self._world._bodies, self)
			TLP2D_World_destroyBody(self._world._worldID, self._bodyID)
			self._bodyID = 0

			return true
		else
			return false
		end
	end

	--- @return TLP2D.BodyID
	--- @nodiscard
	function TLP2D.BodyObject:getID()
		return self._bodyID
	end

	--- @return TLP2D.WorldObject
	--- @nodiscard
	function TLP2D.BodyObject:getWorld()
		return self._world
	end

	--- @param dx number
	--- @param dy number
	function TLP2D.BodyObject:changePosition(dx, dy)
		TLP2D_World_changeBodyPosition(self._world._worldID, self._bodyID, dx, dy)
	end

	--- @param dvx number
	--- @param dvy number
	function TLP2D.BodyObject:changeVelocity(dvx, dvy)
		TLP2D_World_changeBodyVelocity(self._world._worldID, self._bodyID, dvx, dvy)
	end

	--- @param dr number
	function TLP2D.BodyObject:changeRotation(dr)
		TLP2D_World_changeBodyRotation(self._world._worldID, self._bodyID, dr)
	end

	--- @param dav number
	function TLP2D.BodyObject:changeAngularVelocity(dav)
		TLP2D_World_changeBodyAngularVelocity(self._world._worldID, self._bodyID, dav)
	end

	--- @return TLP2D.BodyType
	function TLP2D.BodyObject:getType()
		return TLP2D_World_getBodyType(self._world._worldID, self._bodyID)
	end

	--- @return number x
	--- @return number y
	--- @nodiscard
	function TLP2D.BodyObject:getPosition()
		return TLP2D_World_getBodyPosition(self._world._worldID, self._bodyID)
	end

	--- @return number radius
	--- @nodiscard
	function TLP2D.BodyObject:getRadius()
		return TLP2D_World_getBodyRadius(self._world._worldID, self._bodyID)
	end

	--- @return number | TLP2D.Vector[] radiusOrVertices
	--- @nodiscard
	function TLP2D.BodyObject:getRadiusOrVertices()
		return TLP2D_World_getBodyRadiusOrVertices(self._world._worldID, self._bodyID)
	end

	--- @return number rotation
	--- @nodiscard
	function TLP2D.BodyObject:getRotation()
		return TLP2D_World_getBodyRotation(self._world._worldID, self._bodyID)
	end

	--- @return TLP2D.BodyShape shape
	--- @nodiscard
	function TLP2D.BodyObject:getShape()
		return TLP2D_World_getBodyShape(self._world._worldID, self._bodyID)
	end

	--- @return TLP2D.BodyShape
	--- @nodiscard
	function TLP2D.BodyObject:getShapeData()
		return TLP2D_World_getBodyShapeData(self._world._worldID, self._bodyID)
	end

	--- @return TLP2D.Vector[] vertices
	--- @nodiscard
	function TLP2D.BodyObject:getVertices()
		return TLP2D_World_getBodyVertices(self._world._worldID, self._bodyID)
	end

	--- @return boolean
	--- @nodiscard
	function TLP2D.BodyObject:exists()
		return TLP2D_World_hasBody(self._world._worldID, self._bodyID)
	end

	--- @param material TLP2D.MaterialID | TLP2D.MaterialName
	function TLP2D.BodyObject:setMaterial(material)
		TLP2D_World_setBodyMaterial(self._world._worldID, self._bodyID, material)
	end

	--- @param x number
	--- @param y number
	function TLP2D.BodyObject:setPosition(x, y)
		TLP2D_World_setBodyPosition(self._world._worldID, self._bodyID, x, y)
	end

	--- @param rotation number
	function TLP2D.BodyObject:setRotation(rotation)
		TLP2D_World_setBodyRotation(self._world._worldID, self._bodyID, rotation)
	end

	--- @param radius number
	function TLP2D.BodyObject:setShapeCircle(radius)
		TLP2D_World_setBodyShapeCircle(self._world._worldID, self._bodyID, radius)
	end

	--- @param vertices TLP2D.Vertex[]
	function TLP2D.BodyObject:setShapePolygon(vertices)
		TLP2D_World_setBodyShapePolygon(self._world._worldID, self._bodyID, vertices)
	end

	--- @param width number
	--- @param height number
	function TLP2D.BodyObject:setShapeRectangle(width, height)
		TLP2D_World_setBodyShapeRectangle(self._world._worldID, self._bodyID, width, height)
	end

	--#endregion

	--#region World Object

	--- @readonly
	--- @class TLP2D.WorldObject
	--- @field package _worldID TLP2D.WorldID
	--- @field package _bodies TLP2D.BodyObject[]
	TLP2D.WorldObject = {}

	--- @return TLP2D.WorldObject
	function TLP2D.WorldObject.new()
		--- @type TLP2D.WorldObject
		local worldObject = {
			_worldID = TLP2D_World_create(),
			_bodies = {},
		}
		setObjectMetatable(worldObject, TLP2D.WorldObject, TLP2D.WorldObject.delete)
		return worldObject
	end

	function TLP2D.WorldObject:delete()
		if self._worldID ~= 0 then
			self:clearBodies()

			TLP2D_World_destroy(self._worldID)
			self._worldID = 0
		end
	end

	--- @return TLP2D.WorldID worldID
	--- @nodiscard
	function TLP2D.WorldObject:getID()
		return self._worldID
	end

	--- @return boolean
	--- @nodiscard
	function TLP2D.WorldObject:exists()
		return TLP2D_World_exists(self._worldID)
	end

	--- @param self TLP2D.WorldObject
	--- @param create function
	--- @return TLP2D.BodyObject
	local function newBodyObject(self, create)
		--- @type TLP2D.BodyObject
		local bodyObject = {
			_world = self,
			_bodyID = create(self._worldID),
		}
		setObjectMetatable(bodyObject, TLP2D.BodyObject, TLP2D.BodyObject.delete)
		self._bodies[#self._bodies + 1] = bodyObject
		return bodyObject
	end

	--- @return TLP2D.BodyObject
	function TLP2D.WorldObject:newStaticBody()
		return newBodyObject(self, TLP2D_World_createStaticBody)
	end

	--- @return TLP2D.BodyObject
	function TLP2D.WorldObject:newKinematicBody()
		return newBodyObject(self, TLP2D_World_createKinematicBody)
	end

	--- @return TLP2D.BodyObject
	function TLP2D.WorldObject:newDynamicBody()
		return newBodyObject(self, TLP2D_World_createDynamicBody)
	end

	--- @nodiscard
	function TLP2D.WorldObject:getBodyCount()
		return #self._bodies
	end

	--- @nodiscard
	function TLP2D.WorldObject:iterateBodies()
		return ipairs(self._bodies)
	end

	function TLP2D.WorldObject:clearBodies()
		local bodies = self._bodies
		self._bodies = {}

		for _, bodyObject in ipairs(bodies) do
			bodyObject:delete()
		end

		TLP2D_World_clearBodies(self._worldID)
	end

	--- @param x number
	--- @param y number
	function TLP2D.WorldObject:applyGravity(x, y)
		TLP2D_World_applyGravity(self._worldID, x, y)
	end

	--- @return number? minX
	--- @return number minY
	--- @return number maxX
	--- @return number maxY
	--- @nodiscard
	function TLP2D.WorldObject:getBoundary()
		return TLP2D_World_getBoundary(self._worldID)
	end

	--- @return integer iterations
	--- @nodiscard
	function TLP2D.WorldObject:getIterations()
		return TLP2D_World_getIterations(self._worldID)
	end

	--- @param minX number?
	--- @param minY number?
	--- @param maxX number?
	--- @param maxY number?
	function TLP2D.WorldObject:setBoundary(minX, minY, maxX, maxY)
		TLP2D_World_setBoundary(self._worldID, minX, minY, maxX, maxY)
	end

	--- @param iterations integer
	function TLP2D.WorldObject:setIterations(iterations)
		TLP2D_World_setIterations(self._worldID, iterations)
	end

	--- @param deltaTime number
	function TLP2D.WorldObject:tick(deltaTime)
		TLP2D_World_tick(self._worldID, deltaTime)
	end

	--#endregion
end

--#endregion

return TLP2D
